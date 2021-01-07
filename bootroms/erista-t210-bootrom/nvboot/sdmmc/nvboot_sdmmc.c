/*
 * Copyright (c) 2008 - 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_sdmmc_local.h"
#include "nvrm_drf.h"
#include "arclk_rst.h"
#include "arahb_arbc.h"
#include "arapb_misc_gp.h"
#include "arsdmmc.h"
#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_config.h"
#include "nvboot_device_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_sdmmc_context.h"
#include "nvboot_sdmmc_int.h"
#include "nvboot_sdmmc_param.h"
#include "nvboot_arc_int.h"
#include "nvboot_util_int.h"
#include "project.h"


#define DEBUG_SDMMC 0
#define USE_SDMMC1 0

#if USE_SDMMC1
#define NVBOOT_SDMMC_BASE_ADDRESS NV_ADDRESS_MAP_SDMMC1_BASE
#else
#define NVBOOT_SDMMC_BASE_ADDRESS NV_ADDRESS_MAP_SDMMC4_BASE
#endif

#if DEBUG_SDMMC
#include "nvos.h"
#define PRINT_SDMMC_REG_ACCESS(...)  NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_SDMMC_MESSAGES(...)    NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_SDMMC_ERRORS(...)      NvOsDebugPrintf(__VA_ARGS__);
#else
#define PRINT_SDMMC_REG_ACCESS(...)
#define PRINT_SDMMC_MESSAGES(...)
#define PRINT_SDMMC_ERRORS(...)
#endif

// Compile-time assertions
NV_CT_ASSERT( (SdmmcResponseType_NoResponse == 0) &&
              (SdmmcResponseType_R1 == 1) &&
              (SdmmcResponseType_R2 == 2) &&
              (SdmmcResponseType_R3 == 3) &&
              (SdmmcResponseType_R4 == 4) &&
              (SdmmcResponseType_R5 == 5) &&
              (SdmmcResponseType_R6 == 6) &&
              (SdmmcResponseType_R7 == 7) &&
              (SdmmcResponseType_R1B== 8) );

NV_CT_ASSERT( (NvBootSdmmcDataWidth_1Bit == 0) &&
              (NvBootSdmmcDataWidth_4Bit == 1) &&
              (NvBootSdmmcDataWidth_8Bit == 2) );

NV_CT_ASSERT( (SdmmcAccessRegion_UserArea == 0) &&
              (SdmmcAccessRegion_BootPartition1 == 1) &&
              (SdmmcAccessRegion_BootPartition2 == 2) );

NV_CT_ASSERT((NV_FIELD_SIZE(SDMMC_VENDOR_CLOCK_CNTRL_0_TAP_VAL_RANGE) +
             NV_FIELD_SIZE(SDMMC_VENDOR_CLOCK_CNTRL_0_TRIM_VAL_RANGE)) <= 16);
#define NV_SDMMC_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_ADDRESS_MAP_SDMMC4_BASE + SDMMC_##reg##_0)); \
        PRINT_SDMMC_REG_ACCESS("\r\nRead %s Offset 0x%x = 0x%8.8x", #reg, \
            SDMMC_##reg##_0, value); \
    } while (0)

#define NV_SDMMC_WRITE(reg, value) \
    do { \
        NV_WRITE32((NV_ADDRESS_MAP_SDMMC4_BASE + SDMMC_##reg##_0), value); \
        PRINT_SDMMC_REG_ACCESS("\r\nWrite %s Offset 0x%x = 0x%8.8x", #reg, \
            SDMMC_##reg##_0, value); \
    } while (0)

#define NV_SDMMC_WRITE_08(reg, offset, value) \
    do { \
        NV_WRITE08((NV_ADDRESS_MAP_SDMMC4_BASE + SDMMC_##reg##_0 + offset), \
        value); \
        PRINT_SDMMC_REG_ACCESS("\r\nWrite %s Offset 0x%x = 0x%8.8x", #reg, \
            (SDMMC_##reg##_0 + offset), value); \
    } while (0)

// Can't use do while for this, as it is also called from PRINT_SDMMC_XXX.
#define QUOTIENT_CEILING(dividend, divisor) \
    ((dividend + divisor - 1) / divisor)

static NvBootSdmmcParams s_DefaultSdmmcParams;
static NvBootSdmmcContext *s_SdmmcContext = NULL;
static NvBool s_IsBootModeDataValid = NV_FALSE;
// Boot Info table.
extern NvBootInfoTable BootInfoTable;
// Pointer to Nand Bit info.
static NvBootSdmmcStatus* s_SdmmcBitInfo =
                                &BootInfoTable.SecondaryDevStatus.SdmmcStatus;

// Bug 1475512. This holds the trim/tap values for sdr/ddr mode.
// SDMMC_VENDOR_CLOCK_CNTRL_0_TRIM_VAL : OB trimmer tap value = 0x8
// SDMMC_VENDOR_CLOCK_CNTRL_0_TAP_VAL : IB trimmer tap value = 0x4.
static const NvBootSdmmcVendorClkCtrl s_VendorClockCtrl = 
          { NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TAP_VAL, 0x4) |   //sdr
            NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TRIM_VAL, 0x8) ,
            NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TAP_VAL, 0x4) |   //ddr
            NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TRIM_VAL, 0x8)
          };
                                                                    
// This struct holds the Info from fuses.
typedef struct
{
    /*
     * Voltage range to use during card identification.
     * This must be a static variable to preserve its value from GetParams()
     * to Init(). The fuse value, as opposed to the OCR register value, is
     * stored in the static variable because it is easier to perform validation
     * on the fuse value enumeration.
     */
    NvBootSdmmcVoltageRange VoltageRange;
    // Holds Boot mode support.
    NvBool DisableBootMode;
    // Ddr mode selection
    NvU8 DdrMode;
    // Sdmmc4 pads voltage
    NvU8 Sdmmc4PadsVoltage;
    // Sdmmc4 clock divider
    NvU8 Sdmmc4ClkDivider;
    // sdmmc4 Multi Page support
    NvU8 SdmmcMultiPageSupport;
} NvBootSdmmcFuseInfo;

static NvBootSdmmcFuseInfo s_FuseInfo =
{NvBootSdmmcVoltageRange_QueryVoltage, NV_FALSE, 0, 0, 0};

// Table that maps fuse values to CMD1 OCR argument values.
static NvU32 s_OcrVoltageRange[] =
{
    EmmcOcrVoltageRange_QueryVoltage, // NvBootSdmmcVoltageRange_QueryVoltage
    EmmcOcrVoltageRange_HighVoltage,  // NvBootSdmmcVoltageRange_HighVoltage
    EmmcOcrVoltageRange_DualVoltage,  // NvBootSdmmcVoltageRange_DualVoltage
    EmmcOcrVoltageRange_LowVoltage    // NvBootSdmmcVoltageRange_LowVoltage
};

/* Forward Private Function declarations. */

/* Private Function Definitions. */

/*
 * Functions HwSdmmcxxx --> does Sdmmc register programming.
 * Functions Sdmmcxxx --> Common to Emmc and Esd cards.
 * Functions Emmcxxx --> Specific to Emmc card.
 * Functions NvBootSdmmcxxx --> Public API's
 */

static NvBootError HwSdmmcWaitForClkStable(void)
{
    NvU32 StcReg;
    NvU32 ClockReady;
    NvU32 TimeOut = SDMMC_TIME_OUT_IN_US;

    while (TimeOut)
    {
        NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
        ClockReady = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                        INTERNAL_CLOCK_STABLE, StcReg);
        if (ClockReady)
            break;
        NvBootUtilWaitUS(1);
        TimeOut--;
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\nHwSdmmcInitController()-Clk stable timed out.");
            return NvBootError_HwTimeOut;
        }
    }
    return NvBootError_Success;
}

static NvBootError HwSdmmcSetCardClock(NvBootSdmmcCardClock ClockRate)
{
    NvU32 taac;
    NvU32 nsac;
    NvU32 StcReg;
    NvBootError e;
    NvU32 CardClockInMHz;
    NvU32 CardClockDivisor;
    NvU32 TimeOutCounter = 0;
    NvU32 ClockCyclesRequired;
    NvU32 ControllerClockInMHz;
    NvU32 CardCycleTimeInNanoSec;
    NvU32 ControllerClockDivisor;
    NvU32 ContollerCycleTimeInNanoSec;
    // These array values are as per Emmc/Esd Spec's.
    const NvU32 TaacTimeUnitArray[] = {1, 10, 100, 1000, 10000, 100000, 1000000,
                                       10000000};
    const NvU32 TaacMultiplierArray[] = {10, 10, 12, 13, 15, 20, 25, 30, 35, 40,
                                         45, 50, 55, 60, 70, 80};

    s_SdmmcContext->CurrentClockRate = ClockRate;
    // Disable Card clock before changing it's Frequency.
    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SD_CLOCK_EN, DISABLE, StcReg);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);

    if (ClockRate == NvBootSdmmcCardClock_Identification)
    {
        /*
         * Set the clock divider as 17 for card identification. With clock divider
         * as 17, controller gets a frequency of 408/17 = 24MHz and it will be
         * furthur divided by 64. After dividing it by 64, card gets a
         * frequency of 375KHz. It is the frequency at which card should
         * be identified.
         */
        ControllerClockDivisor = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, 24);
        CardClockDivisor = 64;
    }
    else if (ClockRate == NvBootSdmmcCardClock_DataTransfer)
    {
        ControllerClockDivisor = s_SdmmcContext->ClockDivisor;
        CardClockDivisor = s_SdmmcContext->CardClockDivisor;
    }
    else //if (ClockRate == NvBootSdmmcCardClock_20MHz)
    {
        ControllerClockDivisor = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, 20);
        CardClockDivisor = 1;
    }

    ControllerClockInMHz = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ,
                                ControllerClockDivisor);
    ContollerCycleTimeInNanoSec = QUOTIENT_CEILING(1000, ControllerClockInMHz);
    CardClockInMHz = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ,
                        (ControllerClockDivisor * CardClockDivisor));
    CardCycleTimeInNanoSec = QUOTIENT_CEILING(1000, CardClockInMHz);
    // Find read time out.
    if (s_SdmmcContext->taac != 0)
    {
        // For Emmc, Read time is 10 times of (TAAC + NSAC).
        // for Esd, Read time is 100 times of (TAAC + NSAC) or 100ms, which ever
        // is lower.
        taac = TaacTimeUnitArray[s_SdmmcContext->taac &
               EMMC_CSD_TAAC_TIME_UNIT_MASK] *
               TaacMultiplierArray[(s_SdmmcContext->taac >>
               EMMC_CSD_TAAC_TIME_VALUE_OFFSET) & \
               EMMC_CSD_TAAC_TIME_VALUE_MASK];
        nsac = CardCycleTimeInNanoSec * s_SdmmcContext->nsac * 1000;
        // taac and nsac are already multiplied by 10.
        s_SdmmcContext->ReadTimeOutInUs = QUOTIENT_CEILING((taac + nsac), 1000);
        PRINT_SDMMC_MESSAGES("\r\nCard ReadTimeOutInUs=%d",
            s_SdmmcContext->ReadTimeOutInUs);
        // Use 200ms time out instead of 100ms. This could be helpful in case
        // old version of cards.
        if (s_SdmmcContext->ReadTimeOutInUs < 200000)
            s_SdmmcContext->ReadTimeOutInUs = 200000;
        else if (s_SdmmcContext->ReadTimeOutInUs > 800000)
        {
            //NV_ASSERT(NV_FALSE);
            // Calculation seem to have gone wrong or TAAc is not valid.
            // Set it to 800msec, which is max timeout.
            s_SdmmcContext->ReadTimeOutInUs = 800000;
        }
    }
    PRINT_SDMMC_MESSAGES("\r\nBase Clock=%dMHz",
        QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, ControllerClockDivisor));
    PRINT_SDMMC_MESSAGES("\r\nHwSdmmcSetCardClock Div=%d", CardClockDivisor);

    NvBootClocksConfigureClock(NvBootClocksClockId_SdmmcId,
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(ControllerClockDivisor, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4_0_SDMMC4_CLK_SRC_PLLP_OUT0);

    // If the card clock divisor is 64, the register should be written with 32.
    StcReg = NV_FLD_SET_DRF_NUM(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SDCLK_FREQUENCYSELECT, (CardClockDivisor >> 1), StcReg);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Wait till clock is stable.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForClkStable());
    // Reload reg value after clock is stable.
    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Enable card's clock after clock frequency is changed.
    StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SD_CLOCK_EN, ENABLE, StcReg);
    /*
     * Set Data timeout.
     * If the time out bit field is set to 0xd here, which means the time out at
     * base clock 63MHz is 1065 msec. i.e 2^26 / 63MHz = 1065msec.
     * ControllerClockInMHz = SDMMC_PLL_FREQ_IN_MHZ/ ControllerClockDivisor;
     * ControllerCycleTimeInNanoSec = 1000 / ControllerClockInMHz;
     * ClockCyclesRequired = (s_SdmmcContext->ReadTimeOutInUs * 1000) /
     *                       ControllerClockTimeInNanoSec;
     *                     = (s_SdmmcContext->ReadTimeOutInUs * 1000)/
     *                       (1000 / ControllerClockInMHz);
     *                     = (s_SdmmcContext->ReadTimeOutInUs * ControllerClockInMHz);
     *                     = (s_SdmmcContext->ReadTimeOutInUs *
     *                       (SDMMC_PLL_FREQ_IN_MHZ/ ControllerClockDivisor));
     *                     = (s_SdmmcContext->ReadTimeOutInUs * SDMMC_PLL_FREQ_IN_MHZ) /
     *                       ControllerClockDivisor;
     */
    ClockCyclesRequired = QUOTIENT_CEILING( (s_SdmmcContext->ReadTimeOutInUs *
                            SDMMC_PLL_FREQ_IN_MHZ), ControllerClockDivisor );
    // TimeOutCounter value zero means that the time out is (1 << 13).
    while ( ClockCyclesRequired > (1 << (13 + TimeOutCounter)) )
    {
        TimeOutCounter++;
        // This is max value. so break out from here.
        if (TimeOutCounter == 0xE)
            break;
    }
    // Recalculate the ReadTimeOutInUs based value that is set to register.
    // We shouldn't timout in the code before the controller times out.
    s_SdmmcContext->ReadTimeOutInUs = (1 << (13 + TimeOutCounter));
    s_SdmmcContext->ReadTimeOutInUs = QUOTIENT_CEILING(
                                        s_SdmmcContext->ReadTimeOutInUs, 1000);
    s_SdmmcContext->ReadTimeOutInUs = s_SdmmcContext->ReadTimeOutInUs *
                                      ContollerCycleTimeInNanoSec;
    // The code should never time out before controller. Give some extra time for
    // read time out. Add 50msecs.
    s_SdmmcContext->ReadTimeOutInUs += 50000;
    if (s_SdmmcContext->ReadTimeOutInUs < 200000)
        s_SdmmcContext->ReadTimeOutInUs = 200000;
    else if (s_SdmmcContext->ReadTimeOutInUs > 800000)
    {
        //NV_ASSERT(NV_FALSE);
        // Calculation seem to have gone wrong. Set it to 800msec, which
        // is max timeout.
        s_SdmmcContext->ReadTimeOutInUs = 800000;
    }

    StcReg = NV_FLD_SET_DRF_NUM(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                DATA_TIMEOUT_COUNTER_VALUE, TimeOutCounter, StcReg);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    PRINT_SDMMC_MESSAGES("\r\nTimeOutCounter=%d, ClockCyclesRequired=%d, "
        "CardCycleTimeInNanoSec=%d, ContollerCycleTimeInNanoSec=%d",
        TimeOutCounter, ClockCyclesRequired, CardCycleTimeInNanoSec,
        ContollerCycleTimeInNanoSec);
    PRINT_SDMMC_MESSAGES("\r\nRecalc ReadTimeOutInUs=%d, clk cycles time in ns=%d",
        s_SdmmcContext->ReadTimeOutInUs, ((1 << (13 + TimeOutCounter)) *
        ContollerCycleTimeInNanoSec));
    return NvBootError_Success;
}

static void HwSdmmcSetDataWidth(NvBootSdmmcDataWidth DataWidth)
{
    NvU32 PowerControlHostReg = 0;

    NV_SDMMC_READ(POWER_CONTROL_HOST, PowerControlHostReg);
    PowerControlHostReg = NV_FLD_SET_DRF_NUM(SDMMC, POWER_CONTROL_HOST,
                            DATA_XFER_WIDTH, DataWidth, PowerControlHostReg);
    // When 8-bit data width is enabled, the bit field DATA_XFER_WIDTH
    // value is not valid.
    PowerControlHostReg = NV_FLD_SET_DRF_NUM(SDMMC, POWER_CONTROL_HOST,
                            EXTENDED_DATA_TRANSFER_WIDTH,
                            ((DataWidth == NvBootSdmmcDataWidth_8Bit) ||
                            (DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit)? 1 : 0),
                            PowerControlHostReg);
    NV_SDMMC_WRITE(POWER_CONTROL_HOST, PowerControlHostReg);
    s_SdmmcBitInfo->DataWidthUnderUse = DataWidth;
}

#if !NVBOOT_TARGET_FPGA
static void HwSdmmcAutoCalibrate(void)
{
    NvU32 StcReg = 0;
    NvU32 ActiveInProgress = 0;
    NvU32 TimeOut = SDMMC_TIME_OUT_IN_US;

    s_SdmmcBitInfo->AutoCalStatus = NV_TRUE;

    // Bug #1053446
    // PAD_E_INPUT_OR_E_PWRD bit is set to 1 before starting calibration and cleared after calibration.
    // E_PWRD (for SDMMC4) input of pu/pd comp pad should be cleared once auto-calibration is done (for power saving)

    // setting by default.
    NV_SDMMC_READ(SDMEMCOMPPADCTRL, StcReg);
    StcReg |= NV_FLD_SET_DRF_NUM(SDMMC, SDMEMCOMPPADCTRL,PAD_E_INPUT_OR_E_PWRD, 1, StcReg);
    NV_SDMMC_WRITE(SDMEMCOMPPADCTRL, StcReg);

    NV_SDMMC_READ(AUTO_CAL_CONFIG, StcReg);
    StcReg |= NV_DRF_DEF(SDMMC,AUTO_CAL_CONFIG,AUTO_CAL_ENABLE,ENABLED);
    StcReg |= NV_FLD_SET_DRF_NUM(SDMMC,AUTO_CAL_CONFIG,AUTO_CAL_START,1,StcReg);
    NV_SDMMC_WRITE(AUTO_CAL_CONFIG, StcReg);

    NV_SDMMC_READ(AUTO_CAL_CONFIG, StcReg);

    // Wait for AUTO_CAL_ACTIVE to be asserted.
    // This should take a maximum of 1us
    NvBootUtilWaitUS(1);

    // Wait till Auto cal active is cleared or timeout upto 100ms

    while (TimeOut)
    {
        NV_SDMMC_READ(AUTO_CAL_STATUS, StcReg);
        ActiveInProgress = NV_DRF_VAL(SDMMC, AUTO_CAL_STATUS,
                            AUTO_CAL_ACTIVE, StcReg);
        if (!ActiveInProgress)
            break;
        NvBootUtilWaitUS(1);
        TimeOut--;
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\n Auto Cal active clear timed out.");
            // though timed out..return and try booting!!
            // Autocalibrate Bug # 839636
            s_SdmmcBitInfo->AutoCalStatus = NV_FALSE;
            NV_ASSERT(0);
            // Disable AUTO_CAL_ENABLE mode, in case of timeout.
            NV_SDMMC_READ(AUTO_CAL_CONFIG, StcReg);
            StcReg = NV_FLD_SET_DRF_DEF(SDMMC, AUTO_CAL_CONFIG, AUTO_CAL_ENABLE, DISABLED, StcReg);
            NV_SDMMC_WRITE(AUTO_CAL_CONFIG, StcReg);
            
            // Read back to ensure write went through.
            NV_SDMMC_READ(AUTO_CAL_CONFIG, StcReg);

            // Bug 1504850. Drive codes in case of auto cal failure.
            StcReg = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE+ APB_MISC_GP_EMMC4_PAD_CFGPADCTRL_0);
            StcReg = NV_FLD_SET_DRF_NUM(APB_MISC_GP,
                                        EMMC4_PAD_CFGPADCTRL,
                                        CFG2TMC_EMMC4_PAD_DRVUP_COMP,
                                        0x10,
                                        StcReg);
            StcReg = NV_FLD_SET_DRF_NUM(APB_MISC_GP,
                                        EMMC4_PAD_CFGPADCTRL,
                                        CFG2TMC_EMMC4_PAD_DRVDN_COMP,
                                        0x10,
                                        StcReg);
            NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE+ APB_MISC_GP_EMMC4_PAD_CFGPADCTRL_0, StcReg);
            
            break;
        }
    }

    // Bug #1053446
    // PAD_E_INPUT_OR_E_PWRD bit is set to 1 before starting calibration and cleared after calibration.
    // E_PWRD (for SDMMC4) input of pu/pd comp pad should be cleared once auto-calibration is done (for power saving)
    // clearing
    NV_SDMMC_READ(SDMEMCOMPPADCTRL, StcReg);
    StcReg = NV_FLD_SET_DRF_NUM(SDMMC, SDMEMCOMPPADCTRL, PAD_E_INPUT_OR_E_PWRD, 0, StcReg);
    NV_SDMMC_WRITE(SDMEMCOMPPADCTRL, StcReg);

}
#endif

static NvBootError HwSdmmcInitController(void)
{
    NvU32 StcReg, RegData;
    NvBootError e;
    NvU32 CapabilityReg;
    NvU32 PinmuxSelection;
    NvU32 IntStatusEnableReg;
    NvU32 PowerControlHostReg;
    const NvBootSdmmcVendorClkCtrl *pVendorClockCtrl;

    if ((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_8Bit) ||
        (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit))
    {
        PRINT_SDMMC_MESSAGES("\r\nSetting Pinmux for 8-bit");
        PinmuxSelection = NvBootPinmuxConfig_Sdmmc_Std_x8;
    }
    else
    {
        PRINT_SDMMC_MESSAGES("\r\nSetting Pinmux for 4-bit");
        PinmuxSelection = NvBootPinmuxConfig_Sdmmc_Std_x4;
    }

    if (s_FuseInfo.Sdmmc4PadsVoltage) {
        //Set the pads voltage
        RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_PWR_0);
        RegData = NV_FLD_SET_DRF_DEF(APBDEV_PMC, DDR_PWR, EMMC,
            E_12V, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_PWR_0, RegData);
    }

    (void)NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_Sdmmc, PinmuxSelection);

    NvBootResetSetEnable(NvBootResetDeviceId_SdmmcId, NV_TRUE);

    // Configure the clock source with divider 17, which gives 24MHz.
    PRINT_SDMMC_MESSAGES("\r\nBase Clock=%dMHz",
        QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, 17));
    NvBootClocksConfigureClock(NvBootClocksClockId_SdmmcId,
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(17, 0),
        CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4_0_SDMMC4_CLK_SRC_PLLP_OUT0);

    // Enable the clock.
    NvBootClocksSetEnable(NvBootClocksClockId_SdmmcId, NV_TRUE);

    // Remove the controller from Reset.
    NvBootResetSetEnable(NvBootResetDeviceId_SdmmcId, NV_FALSE);

    // Autocalibrate Bug # 839636
#if !NVBOOT_TARGET_FPGA
    HwSdmmcAutoCalibrate();
#endif
    // Bug 1475512. 
    // To avoid last min ECO, these steps are done here.

    // Selects one cycle delayed version of cmd_oen to mask wdata of IB capture flop
    NV_SDMMC_READ(IO_SPARE, RegData);
    // Write ‘1’ into bit 19 (SPARE_OUT[3]) of register SDMMC_IO_SPARE_0.
    RegData = NV_FLD_SET_DRF_NUM(SDMMC, IO_SPARE, SPARE_OUT,
              NV_DRF_VAL(SDMMC, IO_SPARE, SPARE_OUT, RegData) | (1<<3),
              RegData);
    NV_SDMMC_WRITE(IO_SPARE, RegData);

    // Set SEL_VREG (voltage supply for trimmer delay chain) to 0
    // to make trimmer delay independent of VDD_CORE
    NV_SDMMC_READ(VENDOR_IO_TRIM_CNTRL, RegData);
    RegData = NV_FLD_SET_DRF_NUM(SDMMC, VENDOR_IO_TRIM_CNTRL, SEL_VREG, 0x0, RegData);
    NV_SDMMC_WRITE(VENDOR_IO_TRIM_CNTRL, RegData);

    // Update trimmer trim/tap values.
    NV_SDMMC_READ(VENDOR_CLOCK_CNTRL, RegData);
    pVendorClockCtrl = &s_VendorClockCtrl;
    if((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit)||
       (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit))
    {
        RegData = NV_FLD_SET_DRF_NUM(SDMMC,VENDOR_CLOCK_CNTRL, TRIM_VAL,
                  NV_DRF_VAL(SDMMCBOOT, VENDOR_CLOCK, TRIM_VAL, pVendorClockCtrl->ddrVendorClkCtrl),
                  RegData);
        RegData = NV_FLD_SET_DRF_NUM(SDMMC,VENDOR_CLOCK_CNTRL, TAP_VAL,
                  NV_DRF_VAL(SDMMCBOOT, VENDOR_CLOCK, TAP_VAL, pVendorClockCtrl->ddrVendorClkCtrl),
                  RegData);    
    }
    else
    {
        RegData = NV_FLD_SET_DRF_NUM(SDMMC,VENDOR_CLOCK_CNTRL, TRIM_VAL,
                  NV_DRF_VAL(SDMMCBOOT, VENDOR_CLOCK, TRIM_VAL, pVendorClockCtrl->sdrVendorClkCtrl),
                  RegData);
        RegData = NV_FLD_SET_DRF_NUM(SDMMC,VENDOR_CLOCK_CNTRL, TAP_VAL,
                  NV_DRF_VAL(SDMMCBOOT, VENDOR_CLOCK, TAP_VAL, pVendorClockCtrl->sdrVendorClkCtrl),
                  RegData);
    }
    
    NV_SDMMC_WRITE(VENDOR_CLOCK_CNTRL, RegData);

    // Few sd clocks cycles wait. Approx 1us.
    NvBootUtilWaitUS(1);

    // Set Internal Clock Enable and SDCLK Frequency Select in the
    // Clock Control register.
    StcReg = NV_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                INTERNAL_CLOCK_EN, OSCILLATE) |
             NV_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SDCLK_FREQUENCYSELECT, DIV64);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Wait till clock is stable.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForClkStable());
    // Reload reg value after clock is stable.
    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);

    // Find out what volatage is supported.
    NV_SDMMC_READ(CAPABILITIES, CapabilityReg);
    PowerControlHostReg = 0;

    // voltage setting logic changed 'TO' Required voltage "FROM" supported voltage !!!
    if(s_FuseInfo.VoltageRange == NvBootSdmmcVoltageRange_QueryVoltage)
    {
        if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
            VOLTAGE_SUPPORT_3_3_V, CapabilityReg))
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V3_3);
        }
        else if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
            VOLTAGE_SUPPORT_3_0_V, CapabilityReg))
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V3_0);
        }
        else
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V1_8);
        }
    }
    else if(s_FuseInfo.VoltageRange == NvBootSdmmcVoltageRange_LowVoltage)
    {
        if (NV_DRF_VAL(SDMMC, CAPABILITIES, VOLTAGE_SUPPORT_1_8_V, CapabilityReg))
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V1_8);
        }
        else
        {
            // host controller does not support 1.8v mode
            // fall back to available voltage level and boot without ddr/sdr for eSD support
            // for eSD SDR50/SDR105/DDR50 bus modes are not supported!!! make note and initialize

            if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
                VOLTAGE_SUPPORT_3_3_V, CapabilityReg))
            {
                PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                        SD_BUS_VOLTAGE_SELECT, V3_3);
            }
            else if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
                VOLTAGE_SUPPORT_3_0_V, CapabilityReg))
            {
                PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                        SD_BUS_VOLTAGE_SELECT, V3_0);
            }
        }
    }
    else // high/dual/query logic
    {
        if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
            VOLTAGE_SUPPORT_3_3_V, CapabilityReg))
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V3_3);
        }
        else if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
            VOLTAGE_SUPPORT_3_0_V, CapabilityReg))
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V3_0);
        }
        else
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                    SD_BUS_VOLTAGE_SELECT, V1_8);
        }
    }
    // Enable bus power.
    PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                            SD_BUS_POWER, POWER_ON);
    NV_SDMMC_WRITE(POWER_CONTROL_HOST, PowerControlHostReg);

    s_SdmmcContext->HostSupportsHighSpeedMode = NV_FALSE;
    if (NV_DRF_VAL(SDMMC, CAPABILITIES, HIGH_SPEED_SUPPORT, CapabilityReg))
        s_SdmmcContext->HostSupportsHighSpeedMode = NV_TRUE;
    PRINT_SDMMC_MESSAGES("\r\nHostSupportsHighSpeedMode=%d",
        s_SdmmcContext->HostSupportsHighSpeedMode);
    // Enable Command complete, Transfer complete and various error events.
    IntStatusEnableReg =
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, DATA_END_BIT_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, DATA_CRC_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, DATA_TIMEOUT_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, COMMAND_INDEX_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, COMMAND_END_BIT_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, COMMAND_CRC_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, COMMAND_TIMEOUT_ERR, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, CARD_REMOVAL, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, CARD_INSERTION, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, DMA_INTERRUPT, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, TRANSFER_COMPLETE, ENABLE) |
    NV_DRF_DEF(SDMMC, INTERRUPT_STATUS_ENABLE, COMMAND_COMPLETE, ENABLE);
    NV_SDMMC_WRITE(INTERRUPT_STATUS_ENABLE, IntStatusEnableReg);
    // This method resets card clock divisor. So, set it again.

    // Commenting  this clock setting, since the same is called in
    // EmmcReadDataInBootMode and EmmcIdentifyCard.
    // HwSdmmcSetCardClock(s_SdmmcContext->CurrentClockRate);

    // SDMMC needs ARC to access IRAM.
    NvBootArcEnable();
    return NvBootError_Success;
}


static void
HwSdmmcReadResponse(
    SdmmcResponseType ResponseType,
    NvU32* pRespBuffer)
{
    NvU32* pTemp = pRespBuffer;

    switch (ResponseType)
    {
        case SdmmcResponseType_R1:
        case SdmmcResponseType_R1B:
        case SdmmcResponseType_R3:
        case SdmmcResponseType_R4:
        case SdmmcResponseType_R5:
        case SdmmcResponseType_R6:
        case SdmmcResponseType_R7:
            // bits 39:8 of response are mapped to 31:0.
            NV_SDMMC_READ(RESPONSE_R0_R1, *pTemp);
            break;
        case SdmmcResponseType_R2:
            // bits 127:8 of response are mapped to 119:0.
            NV_SDMMC_READ(RESPONSE_R0_R1, *pTemp);
            pTemp++;
            NV_SDMMC_READ(RESPONSE_R2_R3, *pTemp);
            pTemp++;
            NV_SDMMC_READ(RESPONSE_R4_R5, *pTemp);
            pTemp++;
            NV_SDMMC_READ(RESPONSE_R6_R7, *pTemp);
            break;
        case SdmmcResponseType_NoResponse:
        default:
            *pTemp = 0;
    }
}

static NvBootError HwSdmmcWaitForDataLineReady(void)
{
    NvU32 PresentState;
    NvU32 DataLineActive;
    NvU32 TimeOut = s_SdmmcContext->ReadTimeOutInUs;

    while (TimeOut)
    {
        NV_SDMMC_READ(PRESENT_STATE, PresentState);
        DataLineActive = NV_DRF_VAL(SDMMC, PRESENT_STATE, DAT_LINE_ACTIVE,
                            PresentState);
        if (!DataLineActive)
            break;
        NvBootUtilWaitUS(1);
        TimeOut--;
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\nDataLineActive is not set to 0 and timed out");
            return NvBootError_HwTimeOut;
        }
    }
    return NvBootError_Success;
}

static NvBootError HwSdmmcWaitForCmdInhibitData(void)
{
    NvU32 PresentState;
    NvU32 CmdInhibitData;
    NvU32 TimeOut = s_SdmmcContext->ReadTimeOutInUs;

    while (TimeOut)
    {
        NV_SDMMC_READ(PRESENT_STATE, PresentState);
        // This bit is set to zero after busy line is deasserted.
        // For response R1b, need to wait for this.
        CmdInhibitData = NV_DRF_VAL(SDMMC, PRESENT_STATE, CMD_INHIBIT_DAT,
                            PresentState);
        if (!CmdInhibitData)
            break;
        NvBootUtilWaitUS(1);
        TimeOut--;
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\nCmdInhibitData is not set to 0 and timed out");
            return NvBootError_HwTimeOut;
        }
    }
    return NvBootError_Success;
}

static NvBootError HwSdmmcWaitForCmdInhibitCmd(void)
{
    NvU32 PresentState;
    NvU32 CmdInhibitCmd;
    NvU32 TimeOut = SDMMC_COMMAND_TIMEOUT_IN_US;

    while (TimeOut)
    {
        NV_SDMMC_READ(PRESENT_STATE, PresentState);
        // This bit is set to zero after response is received. So, response
        // registers should be read only after this bit is cleared.
        CmdInhibitCmd = NV_DRF_VAL(SDMMC, PRESENT_STATE, CMD_INHIBIT_CMD,
                            PresentState);
        if (!CmdInhibitCmd)
            break;
        NvBootUtilWaitUS(1);
        TimeOut--;
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\nCmdInhibitCmd is not set to 0 and timed out");
            return NvBootError_HwTimeOut;
        }
    }
    return NvBootError_Success;
}

static NvBootError HwSdmmcWaitForCommandComplete(void)
{
    NvU32 CommandDone;
    NvU32 InterruptStatus;
    NvU32 TimeOutCounter = SDMMC_COMMAND_TIMEOUT_IN_US;
    NvU32 ErrorMask = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_INDEX_ERR,
                        ERR) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_END_BIT_ERR,
                        END_BIT_ERR_GENERATED) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_CRC_ERR,
                        CRC_ERR_GENERATED) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_TIMEOUT_ERR,
                        TIMEOUT);

    while (TimeOutCounter)
    {
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
        CommandDone = NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, CMD_COMPLETE,
                        InterruptStatus);
        if (InterruptStatus & ErrorMask)
        {
            PRINT_SDMMC_ERRORS("\r\nErrors in HwSdmmcWaitForCommandComplete, "
                "InterruptStatus = 0x%x", InterruptStatus);
            return NvBootError_DeviceError;
        }
        if (CommandDone)
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
        {
            PRINT_SDMMC_ERRORS("\r\nTimed out in HwSdmmcWaitForCommandComplete");
            return NvBootError_HwTimeOut;
        }
    }
    return NvBootError_Success;
}

static NvBootError HwSdmmcIssueAbortCommand(void)
{
    NvBootError e;
    NvU32 retries = 2;
    NvU32 CommandXferMode;
    NvU32 InterruptStatus;
    NvU32* pSdmmcResponse = &s_SdmmcContext->SdmmcResponse[0];

    PRINT_SDMMC_MESSAGES("\r\n\r\n     Sending Abort CMD%d",
        SdmmcCommand_StopTransmission);

    CommandXferMode =
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_INDEX,
            SdmmcCommand_StopTransmission) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, COMMAND_TYPE, ABORT) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT, NO_DATA_TRANSFER) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, CMD_INDEX_CHECK_EN, ENABLE) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, CMD_CRC_CHECK_EN, ENABLE) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT, RESP_LENGTH_48BUSY) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DATA_XFER_DIR_SEL, WRITE) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, BLOCK_COUNT_EN, DISABLE) |
        NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DMA_EN, DISABLE);

    while (retries)
    {
        // Clear Status bits what ever is set.
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
        NV_SDMMC_WRITE(INTERRUPT_STATUS, InterruptStatus);
        // This redundant read is for debug purpose.
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
        NV_SDMMC_WRITE(ARGUMENT, 0);
        NV_SDMMC_WRITE(CMD_XFER_MODE, CommandXferMode);
        // Wait for the command to be sent out.if it fails, retry.
        e = HwSdmmcWaitForCommandComplete();
        if (e == NvBootError_Success)
            break;
        HwSdmmcInitController();
        retries--;
    }
    if (retries)
    {
        // Wait till response is received from card.
        NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitCmd());
        // Wait till busy line is deasserted by card. It is for R1b response.
        NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitData());
        HwSdmmcReadResponse(SdmmcResponseType_R1B, pSdmmcResponse);
    }
    return e;
}

static NvBootError HwSdmmcRecoverControllerFromErrors(NvBool IsDataCmd)
{
    NvU32 StcReg;
    NvU32 PresentState;
    NvU32 ResetInProgress;
    NvU32 InterruptStatus;
    NvU32 TimeOut = SDMMC_TIME_OUT_IN_US;
    NvU32 CommandError = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_INDEX_ERR,
                            ERR) |
                         NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_END_BIT_ERR,
                            END_BIT_ERR_GENERATED) |
                         NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_CRC_ERR,
                            CRC_ERR_GENERATED) |
                         NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_TIMEOUT_ERR,
                            TIMEOUT);
    NvU32 DataError = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_END_BIT_ERR,
                        ERR) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_CRC_ERR,
                        ERR) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_TIMEOUT_ERR,
                        TIMEOUT);
    NvU32 DataStateMask = NV_DRF_DEF(SDMMC, PRESENT_STATE, DAT_3_0_LINE_LEVEL,
                            DEFAULT_MASK);

    NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    if (InterruptStatus & CommandError)
    {
        // Reset Command line.
        StcReg |= NV_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                    SW_RESET_FOR_CMD_LINE, RESETED);
        NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
        // Wait till Reset is completed.
        while (TimeOut)
        {
            NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
            ResetInProgress = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                SW_RESET_FOR_CMD_LINE, StcReg);
            if (!ResetInProgress)
                break;
            NvBootUtilWaitUS(1);
            TimeOut--;
        }
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\nReset Command line timed out.");
            return NvBootError_HwTimeOut;
        }
    }
    if (InterruptStatus & DataError)
    {
        // Reset Data line.
        StcReg |= NV_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                    SW_RESET_FOR_DAT_LINE, RESETED);
        NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
        // Wait till Reset is completed.
        while (TimeOut)
        {
            NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
            ResetInProgress = NV_DRF_VAL(SDMMC,
                                        SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                        SW_RESET_FOR_DAT_LINE, StcReg);
            if (!ResetInProgress)
                break;
            NvBootUtilWaitUS(1);
            TimeOut--;
        }
        if (!TimeOut)
        {
            PRINT_SDMMC_ERRORS("\r\nReset Data line timed out.");
            return NvBootError_HwTimeOut;
        }
    }
    // Clear Interrupt Status
    NV_SDMMC_WRITE(INTERRUPT_STATUS, InterruptStatus);
    // Issue abort command.
    if (IsDataCmd)
        (void)HwSdmmcIssueAbortCommand();
    // Wait for 40us as per spec.
    NvBootUtilWaitUS(40);
    // Read Present State register.
    NV_SDMMC_READ(PRESENT_STATE, PresentState);
    if ( (PresentState & DataStateMask) != DataStateMask )
    {
        // Before give up, try full reset once.
        HwSdmmcInitController();
        NV_SDMMC_READ(PRESENT_STATE, PresentState);
        if ( (PresentState & DataStateMask) != DataStateMask)
        {
            PRINT_SDMMC_ERRORS("\r\nError Recovery Failed.");
            return NvBootError_DeviceError;
        }
    }
    return NvBootError_Success;
}

static void HwSdmmcAbortDataRead(void)
{
    NvU32 StcReg;
    NvU32 PresentState;
    NvU32 ResetInProgress;
    NvU32 TimeOut = SDMMC_TIME_OUT_IN_US;
    NvU32 DataStateMask = NV_DRF_DEF(SDMMC, PRESENT_STATE,
                            DAT_3_0_LINE_LEVEL, DEFAULT_MASK);

    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Reset Data line.
    StcReg |= NV_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SW_RESET_FOR_DAT_LINE, RESETED);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Wait till Reset is completed.
    while (TimeOut)
    {
        NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
        ResetInProgress = NV_DRF_VAL(SDMMC,
                                    SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                                    SW_RESET_FOR_DAT_LINE, StcReg);
        if (!ResetInProgress)
            break;
        NvBootUtilWaitUS(1);
        TimeOut--;
    }
    if (!TimeOut)
    {
        PRINT_SDMMC_ERRORS("\r\nAbortDataRead-Reset Data line timed out.");
    }
    // Read Present State register.
    NV_SDMMC_READ(PRESENT_STATE, PresentState);
    if ( (PresentState & DataStateMask) != DataStateMask )
    {
        // Before give up, try full reset once.
        HwSdmmcInitController();
        NV_SDMMC_READ(PRESENT_STATE, PresentState);
        if ( (PresentState & DataStateMask) != DataStateMask)
        {
            PRINT_SDMMC_ERRORS("\r\nError Recovery Failed.");
        }
    }
}

// This array indicates the response formats that need to be programmed to
// controller for various responses along with contstant arguments.
//
// EMMC does not have responses R6, R7. SD only has additional R6 and R7
// responses. EMMC R4 needs command and crc checks in response.
// SD R4 doesn't need command and crc checks in response.
// Do we use R4 Response at all? No, it isn't as of now in this driver.
//
// This array indicates what responses need Command Index check.
//  NR,R1,R2,R3,R4,R5,R6,R7,R1b
// {0, 1, 0, 0, 0, 1, 1, 1, 1};
//
// This array indicates what responses need Crc check.
//  NR,R1,R2,R3,R4,R5,R6,R7,R1b
// {0, 1, 1, 0, 0, 1, 1, 1, 1};
#define RESPONSE_DATA(type, index_check, crc_check)                        \
    ( NV_DRF_DEF(SDMMC, CMD_XFER_MODE, COMMAND_TYPE,       NORMAL)      | \
      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DATA_XFER_DIR_SEL,  READ)        | \
      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, RESP_TYPE_SELECT,   type)        | \
      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, CMD_INDEX_CHECK_EN, index_check) | \
      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, CMD_CRC_CHECK_EN,   crc_check) )

static NvBootError
HwSdmmcSendCommand(
    SdmmcCommand CommandIndex,
    NvU32 CommandArg,
    SdmmcResponseType ResponseType,
    NvBool IsDataCmd)
{
    NvBootError e;
    NvU32 retries = 3;
    NvU32 CommandXferMode;
    NvU32 InterruptStatus;
    NvU32* pSdmmcResponse = &s_SdmmcContext->SdmmcResponse[0];

    NV_ASSERT(ResponseType < SdmmcResponseType_Num);
    PRINT_SDMMC_MESSAGES("\r\n\r\n     Sending CMD%d", CommandIndex);
    PRINT_SDMMC_MESSAGES("\r\nCmd Index=0x%x, Arg=0x%x, RespType=%d, data=%d",
        CommandIndex, CommandArg, ResponseType, IsDataCmd);
    // Wait till Controller is ready.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitCmd());

    CommandXferMode =
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_INDEX, CommandIndex) |
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT, (IsDataCmd ? 1 : 0)) |
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_XFER_DIR_SEL, (IsDataCmd ? 1 : 0)) |
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, BLOCK_COUNT_EN, (IsDataCmd ? 1 : 0)) |
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DMA_EN, (IsDataCmd ? 1 : 0));

    if(CommandIndex == SdmmcCommand_ReadMulti)
       CommandXferMode |=
            NV_DRF_NUM(SDMMC, CMD_XFER_MODE,MULTI_BLOCK_SELECT , 1);
    //cmd index check
    if((ResponseType != SdmmcResponseType_NoResponse) &&
        (ResponseType != SdmmcResponseType_R2) &&
        (ResponseType != SdmmcResponseType_R3) &&
        (ResponseType != SdmmcResponseType_R4) )
        {
          CommandXferMode |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE,
                                                CMD_INDEX_CHECK_EN, ENABLE);
        }

    //crc index check
    if((ResponseType != SdmmcResponseType_NoResponse) &&
        (ResponseType != SdmmcResponseType_R3) &&
        (ResponseType != SdmmcResponseType_R4) )
        {
          CommandXferMode |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE,
                                                CMD_CRC_CHECK_EN, ENABLE);
        }

    //response type check
    if(ResponseType == SdmmcResponseType_NoResponse)
        {
          CommandXferMode |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE,
                                                RESP_TYPE_SELECT,
                                                NO_RESPONSE);
        }
    else if(ResponseType == SdmmcResponseType_R2 )
        {
          CommandXferMode |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE,
                                                RESP_TYPE_SELECT,
                                                RESP_LENGTH_136);
        }
    else if(ResponseType == SdmmcResponseType_R1B)
        {
          CommandXferMode |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE,
                                                RESP_TYPE_SELECT,
                                                RESP_LENGTH_48BUSY);
        }
    else
        {
          CommandXferMode |= NV_DRF_DEF(SDMMC, CMD_XFER_MODE,
                                                RESP_TYPE_SELECT,
                                                RESP_LENGTH_48);
        }
    while (retries)
    {
        // Clear Status bits what ever is set.
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
        NV_SDMMC_WRITE(INTERRUPT_STATUS, InterruptStatus);
        // This redundant read is for debug purpose.
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
        NV_SDMMC_WRITE(ARGUMENT, CommandArg);
        NV_SDMMC_WRITE(CMD_XFER_MODE, CommandXferMode);
        // Wait for the command to be sent out. If it fails, retry.
        e = HwSdmmcWaitForCommandComplete();
        if (e == NvBootError_Success)
            break;
        // Recover Controller from Errors.
        HwSdmmcRecoverControllerFromErrors(IsDataCmd);
        retries--;
    }
    if (retries)
    {
        // Wait till response is received from card.
        NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitCmd());
        if (ResponseType == SdmmcResponseType_R1B)
            // Wait till busy line is deasserted by card.
            NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitData());
        HwSdmmcReadResponse(ResponseType, pSdmmcResponse);
    }
    return e;
}

static void HwSdmmcSetNumOfBlocks(NvU32 BlockLength, NvU32 NumOfBlocks)
{
    NvU32 BlockReg;

    BlockReg = NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT, BLOCKS_COUNT,
                NumOfBlocks) |
               NV_DRF_DEF(SDMMC, BLOCK_SIZE_BLOCK_COUNT,
               /*
                * This makes controller halt when ever it detects 512KB boundary.
                * When controller halts on this boundary, need to clear the
                * dma block boundary event and write SDMA base address again.
                * Writing address again triggers controller to continue.
                * We can't disable this. We have to live with it.
                */
                HOST_DMA_BUFFER_SIZE, DMA512K) |
               NV_DRF_NUM(SDMMC, BLOCK_SIZE_BLOCK_COUNT,
                XFER_BLOCK_SIZE_11_0, BlockLength);
    NV_SDMMC_WRITE(BLOCK_SIZE_BLOCK_COUNT, BlockReg);
}

static void HwSdmmcSetupDma(NvU8 *pBuffer, NvU32 NumOfBytes)
{
    // Program Single DMA base address.
    NV_SDMMC_WRITE(SYSTEM_ADDRESS, (NvU32)(pBuffer));
}

static void HwSdmmcEnableHighSpeed(NvBool Enable)
{
    NvU32 PowerControlHostReg = 0;

    NV_SDMMC_READ(POWER_CONTROL_HOST, PowerControlHostReg);
    PowerControlHostReg = NV_FLD_SET_DRF_NUM(SDMMC, POWER_CONTROL_HOST,
                            HIGH_SPEED_EN, ((Enable == NV_TRUE) ? 1 : 0),
                            PowerControlHostReg);
    NV_SDMMC_WRITE(POWER_CONTROL_HOST, PowerControlHostReg);
}

static void HwSdmmcCalculateCardClockDivisor(void)
{
    NvU32 TotalClockDivisor = s_SdmmcContext->ClockDivisor;

    s_SdmmcContext->CardClockDivisor = 1;
    s_SdmmcContext->HighSpeedMode = NV_FALSE;
    if ( (s_SdmmcContext->HostSupportsHighSpeedMode == NV_FALSE) ||
         (s_SdmmcContext->CardSupportsHighSpeedMode == NV_FALSE) ||
         (s_SdmmcContext->SpecVersion < 4) )
    {
        // Either card or host doesn't support high speed. So reduce the clock
        // frequency if required.
        if (QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ,
            s_SdmmcContext->ClockDivisor) >
            s_SdmmcContext->TranSpeedInMHz)
            s_SdmmcContext->ClockDivisor =
            QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ,
            s_SdmmcContext->TranSpeedInMHz);
    }
    else
    {
        while (QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, TotalClockDivisor) >
                SDMMC_MAX_CLOCK_FREQUENCY_IN_MHZ)
        {
            s_SdmmcContext->CardClockDivisor <<= 1;
            TotalClockDivisor <<= 1;
        }
        if (QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, TotalClockDivisor) >
            s_SdmmcContext->TranSpeedInMHz)
            s_SdmmcContext->HighSpeedMode = NV_TRUE;
    }
    PRINT_SDMMC_MESSAGES("\r\nClockDivisor=%d, CardClockDivisor=%d, "
        "HighSpeedMode=%d", s_SdmmcContext->ClockDivisor,
        s_SdmmcContext->CardClockDivisor, s_SdmmcContext->HighSpeedMode);
}

static NvBootError EmmcReadDataInBootMode(NvU8* pBuffer, NvU32 NumOfBlocks)
{
    NvBootError e;
    NvU32 BootControl;
    NvU32 CommandXferMode;
    NvU32 InterruptStatus;
    NvBootDeviceStatus DevStatus;
    NvU32 TimeOut = s_SdmmcContext->ReadTimeOutInUs;
    NvU32 StcReg;
    NvU32 HostControl2Reg=0;
    NvU32 CapabilityReg;
    NvU32 CapabilityHighReg;

    if (s_IsBootModeDataValid == NV_TRUE)
        return NvBootError_Success;
    HwSdmmcSetDataWidth(s_SdmmcContext->DataWidth);
    // Set card clock to 20MHz.
    HwSdmmcSetCardClock(NvBootSdmmcCardClock_20MHz);
    HwSdmmcSetNumOfBlocks(1 << SDMMC_MAX_PAGE_SIZE_LOG_2, NumOfBlocks);
    HwSdmmcSetupDma(pBuffer, NumOfBlocks << SDMMC_MAX_PAGE_SIZE_LOG_2);
    /*
     * Set Boot Ack and Data time outs 50msec and 1sec.
     * 20MHz --> 50ns cycle time.
     * 0xF4240 * 50ns = 50msec.
     * 0x1312D00 * 50ns = 1sec.
     */

    if((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit) ||
            (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit))
    {
         /*
         * HS boot mode support
         * DDR boot mode configuration
         */
        //set the ddr mode in Host controller and other misc things..
        // check capabilities register for Ddr support
        // read capabilities and capabilities higher reg
        NV_SDMMC_READ(CAPABILITIES, CapabilityReg);
        NV_SDMMC_READ(CAPABILITIES_HIGHER, CapabilityHighReg);

        if (NV_DRF_VAL(SDMMC, CAPABILITIES_HIGHER, DDR50, CapabilityHighReg) &&
            NV_DRF_VAL(SDMMC, CAPABILITIES, VOLTAGE_SUPPORT_1_8_V, CapabilityReg) &&
            NV_DRF_VAL(SDMMC, CAPABILITIES, HIGH_SPEED_SUPPORT, CapabilityReg))
        {
            // configure HC2R
            // reset SD clock enable
            NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
            StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                        SD_CLOCK_EN, DISABLE, StcReg);
            NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);

            // set DDR50 UHS Mode
            NV_SDMMC_READ(AUTO_CMD12_ERR_STATUS, HostControl2Reg);
            HostControl2Reg |= NV_DRF_DEF(SDMMC, AUTO_CMD12_ERR_STATUS,
                                    UHS_MODE_SEL, DDR50);
            NV_SDMMC_WRITE(AUTO_CMD12_ERR_STATUS, HostControl2Reg);

            // set enable SD clock
            StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                        SD_CLOCK_EN, ENABLE, StcReg);
            NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
        }
    }

    NV_SDMMC_WRITE(VENDOR_BOOT_ACK_TIMEOUT, 0xF4240);
    NV_SDMMC_WRITE(VENDOR_BOOT_DAT_TIMEOUT, 0x1312D00);
    // Setup Command Xfer reg.
    CommandXferMode = NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT,
                        DATA_TRANSFER) |
                      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DATA_XFER_DIR_SEL, READ) |
                      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, BLOCK_COUNT_EN, ENABLE) |
                      NV_DRF_DEF(SDMMC, CMD_XFER_MODE, DMA_EN, ENABLE);
    NV_SDMMC_WRITE_08(CMD_XFER_MODE, 0, (CommandXferMode & 0xFF));
    NV_SDMMC_WRITE_08(CMD_XFER_MODE, 1, ((CommandXferMode >> 8) & 0xFF));
    NV_SDMMC_WRITE_08(CMD_XFER_MODE, 2, ((CommandXferMode >> 16) & 0xFF));
    // Wait till Controller is ready.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitCmd());
    // Wait till busy line is deasserted.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitData());
    // Setup Boot Control reg.
    BootControl = NV_DRF_DEF(SDMMC, VENDOR_BOOT_CNTRL, BOOT_ACK, ENABLE) |
                  NV_DRF_DEF(SDMMC, VENDOR_BOOT_CNTRL, BOOT, ENABLE);
    NV_SDMMC_WRITE(VENDOR_BOOT_CNTRL, BootControl);
    // Wait for data receive.
    s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_ReadInProgress;
    s_SdmmcContext->ReadStartTime = NvBootUtilGetTimeUS();
    s_SdmmcContext->BootModeReadInProgress = NV_TRUE;
    do
    {
        DevStatus = NvBootSdmmcQueryStatus();
    } while ( (DevStatus != NvBootDeviceStatus_Idle) &&
              (DevStatus == NvBootDeviceStatus_ReadInProgress) );
    s_SdmmcContext->BootModeReadInProgress = NV_FALSE;
    if (DevStatus != NvBootDeviceStatus_Idle)
    {
        while (TimeOut)
        {
            // Disable Boot mode.
            BootControl = NV_DRF_DEF(SDMMC, VENDOR_BOOT_CNTRL, BOOT_ACK,
                            DISABLE) |
                          NV_DRF_DEF(SDMMC, VENDOR_BOOT_CNTRL, BOOT, DISABLE);
            NV_SDMMC_WRITE(VENDOR_BOOT_CNTRL, BootControl);
            NV_SDMMC_READ(VENDOR_BOOT_CNTRL, BootControl);
            if (!BootControl)
                break;
            TimeOut--;
        }
        // Clear Status bits what ever is set.
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatus);
        NV_SDMMC_WRITE(INTERRUPT_STATUS, InterruptStatus);
        return NvBootError_DeviceError;
    }
    // Boot mode is succesful. Don't try to read in boot mode again.
    s_IsBootModeDataValid = NV_TRUE;
    return e;
}

static NvBool SdmmcIsCardInTransferState(void)
{
    NvBootError e;
    NvU32 CardState;
    NvU32* pResp = &s_SdmmcContext->SdmmcResponse[0];

    // Send SEND_STATUS(CMD13) Command.
    NV_BOOT_CHECK_ERROR_CLEANUP(HwSdmmcSendCommand(SdmmcCommand_SendStatus,
        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE));
    // Extract the Card State from the Response.
    CardState = NV_DRF_VAL(SDMMC, CS, CURRENT_STATE, pResp[0]);
    if (CardState == SdmmcState_Tran)
        return NV_TRUE;
fail:
    return NV_FALSE;
}

static NvBootError EmmcGetOpConditions(void)
{
    NvBootError e;
    NvU32 StartTime;
    NvU32 OCRRegister = 0;
    NvU32 ElapsedTime = 0;
    NvU32* pSdmmcResponse = &s_SdmmcContext->SdmmcResponse[0];
    NvU32 Cmd1Arg = s_OcrVoltageRange[s_FuseInfo.VoltageRange];

    if (Cmd1Arg != EmmcOcrVoltageRange_QueryVoltage)
        Cmd1Arg |= SDMMC_CARD_CAPACITY_MASK;
    StartTime = NvBootUtilGetTimeUS();
    // Send SEND_OP_COND(CMD1) Command.
    while (ElapsedTime <= SDMMC_OP_COND_TIMEOUT_IN_US)
    {
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(
            SdmmcCommand_EmmcSendOperatingConditions, Cmd1Arg,
            SdmmcResponseType_R3, NV_FALSE));
        // Extract OCR from Response.
        OCRRegister = pSdmmcResponse[SDMMC_OCR_RESPONSE_WORD];
        // Check for Card Ready.
        if (OCRRegister & SDMMC_OCR_READY_MASK)
            break;
        if (Cmd1Arg == EmmcOcrVoltageRange_QueryVoltage)
        {
            if (OCRRegister & EmmcOcrVoltageRange_HighVoltage)
            {
                Cmd1Arg = EmmcOcrVoltageRange_HighVoltage;
                s_SdmmcContext->IsHighVoltageRange = NV_TRUE;
                s_SdmmcBitInfo->DiscoveredVoltageRange =
                    EmmcOcrVoltageRange_HighVoltage;
            }
            else if (OCRRegister & EmmcOcrVoltageRange_LowVoltage)
            {
                Cmd1Arg = EmmcOcrVoltageRange_LowVoltage;
                s_SdmmcContext->IsHighVoltageRange = NV_FALSE;
                s_SdmmcBitInfo->DiscoveredVoltageRange =
                    EmmcOcrVoltageRange_LowVoltage;
            }
            else
            {
                ElapsedTime = NvBootUtilElapsedTimeUS(StartTime);
                continue;
            }
            Cmd1Arg |= SDMMC_CARD_CAPACITY_MASK;
            StartTime = NvBootUtilGetTimeUS();
            continue;
        }
        #if DEBUG_SDMMC
        ElapsedTime += 10000;
        #else
        ElapsedTime = NvBootUtilElapsedTimeUS(StartTime);
        #endif
        // Wait for ten milliseconds between commands. This to avoid
        // sending cmd1 too many times.
        NvBootUtilWaitUS(10000);
    }
    if (ElapsedTime > SDMMC_OP_COND_TIMEOUT_IN_US)
    {
        PRINT_SDMMC_ERRORS("\r\nTimeout during CMD1");
        return NvBootError_HwTimeOut;
    }
    PRINT_SDMMC_MESSAGES("\r\nTimeTaken for CMD1=%dus", ElapsedTime);
    s_SdmmcContext->IsHighCapacityCard = ( (OCRRegister &
                                           SDMMC_CARD_CAPACITY_MASK) ?
                                           NV_TRUE : NV_FALSE );
    return e;
}


static NvBootError SdmmcGetCsd(void)
{
    NvU32 Mult;
    NvU32 CSize;
    NvBootError e;
    NvU32 CSizeMulti;
    NvU32* pResp = &s_SdmmcContext->SdmmcResponse[0];

    // Send SEND_CSD(CMD9) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SendCsd,
        s_SdmmcContext->CardRca, SdmmcResponseType_R2, NV_FALSE));
    // Extract the page size log2 from Response data.
    s_SdmmcContext->PageSizeLog2 = NV_DRF_VAL(EMMC, CSD, READ_BL_LEN, pResp[2]);
    s_SdmmcContext->PageSizeLog2ForCapacity = s_SdmmcContext->PageSizeLog2;
    s_SdmmcContext->BlockSizeLog2 = NVBOOT_SDMMC_BLOCK_SIZE_LOG2;
    /*
     * The page size can be 512, 1024, 2048 or 4096. this size can only be used
     * for Card capacity calculation. For Read/Write operations, We must use
     * 512 byte page size only.
     */
    // Restrict the reads to (1 << EMMC_MAX_PAGE_SIZE_LOG_2) byte reads.
    if (s_SdmmcContext->PageSizeLog2 > SDMMC_MAX_PAGE_SIZE_LOG_2)
        s_SdmmcContext->PageSizeLog2 = SDMMC_MAX_PAGE_SIZE_LOG_2;
    if (s_SdmmcContext->PageSizeLog2 == 0)
        return NvBootError_DeviceError;
    s_SdmmcContext->PagesPerBlockLog2 = (s_SdmmcContext->BlockSizeLog2 -
                                           s_SdmmcContext->PageSizeLog2);
    // Extract the Spec Version from Response data.
    s_SdmmcContext->SpecVersion = NV_DRF_VAL(EMMC, CSD, SPEC_VERS, pResp[3]);
    s_SdmmcContext->taac = NV_DRF_VAL(EMMC, CSD, TAAC, pResp[3]);
    s_SdmmcContext->nsac = NV_DRF_VAL(EMMC, CSD, NSAC, pResp[3]);
    s_SdmmcContext->TranSpeed = NV_DRF_VAL(EMMC, CSD, TRAN_SPEED, pResp[2]);

    // For <= Emmc v4.0, v4.1, v4.2 and < Esd v1.10.
    s_SdmmcContext->TranSpeedInMHz = 20;
    // For Emmc v4.3, v4.4 and Esd 1.10 onwards.
    if (s_SdmmcContext->TranSpeed == EMMC_CSD_V4_3_TRAN_SPEED)
    {
        // For Emmc, it is 26MHz.
        s_SdmmcContext->TranSpeedInMHz = 26;
    }

    if (s_SdmmcContext->SpecVersion >= 4)
        s_SdmmcContext->CardSupportsHighSpeedMode = NV_TRUE;
    // Fund out number of blocks in card.
    CSize = NV_DRF_VAL(EMMC, CSD, C_SIZE_0, pResp[1]);
    CSize |= (NV_DRF_VAL(EMMC, CSD, C_SIZE_1, pResp[2]) <<
             EMMC_CSD_C_SIZE_1_LEFT_SHIFT_OFFSET);
    CSizeMulti = NV_DRF_VAL(EMMC, CSD, C_SIZE_MULTI, pResp[1]);
    if ( (CSize == EMMC_CSD_MAX_C_SIZE) &&
         (CSizeMulti == EMMC_CSD_MAX_C_SIZE_MULTI) )
    {
        // Capacity is > 2GB and should be calculated from ECSD fields,
        // which is done in EmmcGetExtCSD() method.
        PRINT_SDMMC_MESSAGES("\r\nSdmmcGetCsd:Capacity is > 2GB")
    }
    else
    {
        Mult = 1 << (CSizeMulti + 2);
        s_SdmmcContext->NumOfBlocks = (CSize + 1) * Mult *
                               (1 << (s_SdmmcContext->PageSizeLog2ForCapacity -
                                s_SdmmcContext->PageSizeLog2));
        PRINT_SDMMC_MESSAGES("\r\nCsd NumOfBlocks=%d",
            s_SdmmcContext->NumOfBlocks);
    }

    PRINT_SDMMC_MESSAGES("\r\nPage size from Card=0x%x, 0x%x",
        s_SdmmcContext->PageSizeLog2, (1 << s_SdmmcContext->PageSizeLog2));
    PRINT_SDMMC_MESSAGES("\r\nEmmc SpecVersion=0x%x", s_SdmmcContext->SpecVersion);
    PRINT_SDMMC_MESSAGES("\r\ntaac=0x%x", s_SdmmcContext->taac);
    PRINT_SDMMC_MESSAGES("\r\nnsac=0x%x", s_SdmmcContext->nsac);
    PRINT_SDMMC_MESSAGES("\r\nTranSpeed=0x%x", s_SdmmcContext->TranSpeed);
    PRINT_SDMMC_MESSAGES("\r\nTranSpeedInMHz=%d", s_SdmmcContext->TranSpeedInMHz);
    PRINT_SDMMC_MESSAGES("\r\nCardCommandClasses=0x%x", NV_DRF_VAL(EMMC, CSD,
        CCC, pResp[2]));
    PRINT_SDMMC_MESSAGES("\r\nCSize=0x%x, CSizeMulti=0x%x", CSize, CSizeMulti);
    return e;
}

static
NvBootError
EmmcVerifyResponse(
    SdmmcCommand command,
    NvBool AfterCmdExecution)
{
    NvU32* pResp = &s_SdmmcContext->SdmmcResponse[0];
    NvU32 AddressOutOfRange = NV_DRF_VAL(SDMMC, CS, ADDRESS_OUT_OF_RANGE, pResp[0]);
    NvU32 AddressMisalign = NV_DRF_VAL(SDMMC, CS, ADDRESS_MISALIGN, pResp[0]);
    NvU32 BlockLengthError = NV_DRF_VAL(SDMMC, CS, BLOCK_LEN_ERROR, pResp[0]);
    NvU32 CommandCrcError = NV_DRF_VAL(SDMMC, CS, COM_CRC_ERROR, pResp[0]);
    // For illegal commands, card does not respond. It can
    // be known only through CMD13.
    NvU32 IllegalCommand = NV_DRF_VAL(SDMMC, CS, ILLEGAL_CMD, pResp[0]);
    NvU32 CardInternalError = NV_DRF_VAL(SDMMC, CS, CC_ERROR, pResp[0]);
    NvU32 CardEccError = NV_DRF_VAL(SDMMC, CS, CARD_ECC_FAILED, pResp[0]);
    NvU32 SwitchError = NV_DRF_VAL(SDMMC, CS, SWITCH_ERROR, pResp[0]);
    NvBool BeforeCommandExecution = (AfterCmdExecution ? NV_FALSE : NV_TRUE);

    if ((command == SdmmcCommand_ReadSingle) || (command == SdmmcCommand_ReadMulti))
    {
        if (BeforeCommandExecution)
        {
            // This is during response time.
            if ( AddressOutOfRange || AddressMisalign || BlockLengthError ||
                 CardInternalError )
            {
                PRINT_SDMMC_ERRORS("\r\nReadSingle/Multi Operation failed.");
                return NvBootError_DeviceResponseError;
            }
        }
        else if (CommandCrcError || IllegalCommand || CardEccError)
        {
            return NvBootError_DeviceReadError;
        }
    }
    else if (command == SdmmcCommand_SetBlockLength)
    {
        if ( BeforeCommandExecution && (BlockLengthError || CardInternalError) )
        {
            // Either the argument of a SET_BLOCKLEN command exceeds the
            // maximum value allowed for the card, or the previously defined
            // block length is illegal for the current command
            PRINT_SDMMC_ERRORS("\r\nSetBlockLength Operation failed.");
            return NvBootError_DeviceResponseError;
        }
    }
    else if (command == SdmmcCommand_Switch)
    {
        if ( AfterCmdExecution && (SwitchError || CommandCrcError) )
        {
            // If set, the card did not switch to the expected mode as
            // requested by the SWITCH command.
            PRINT_SDMMC_ERRORS("\r\nSwitch Operation failed.");
            return NvBootError_DeviceResponseError;
        }
    }
    else if (command == SdmmcCommand_EmmcSendExtendedCsd)
    {
        if (BeforeCommandExecution && CardInternalError)
        {
            PRINT_SDMMC_ERRORS("\r\nSend Extneded CSD Operation failed.");
            return NvBootError_DeviceResponseError;
        }
    }
    return NvBootError_Success;
}

static NvBootError EmmcSendSwitchCommand(NvU32 CmdArg)
{
    NvBootError e;
    SdmmcResponseType Response = SdmmcResponseType_R1B;

    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_Switch,
        CmdArg, Response, NV_FALSE));
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SendStatus,
        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE));
    NV_BOOT_CHECK_ERROR(EmmcVerifyResponse(SdmmcCommand_Switch, NV_TRUE));
    return e;
}

static NvBootError EmmcSelectAccessRegion(SdmmcAccessRegion region)
{
    NvU32 CmdArg;
    NvBootError e;
    NV_ASSERT(region < SdmmcAccessRegion_Num);

    CmdArg = s_SdmmcContext->BootConfig & (~EMMC_SWITCH_SELECT_PARTITION_MASK);
    CmdArg |= region;
    CmdArg <<= EMMC_SWITCH_SELECT_PARTITION_OFFSET;
    CmdArg |= EMMC_SWITCH_SELECT_PARTITION_ARG;
    NV_BOOT_CHECK_ERROR(EmmcSendSwitchCommand(CmdArg));
    s_SdmmcContext->CurrentAccessRegion = region;
    PRINT_SDMMC_MESSAGES("\r\n\r\nSelected Region=%d(1->BP1, 2->BP2, 0->User)",
        region);
    return e;
}

static NvBootError SdmmcSelectAccessRegion(NvU32* Block, NvU32* Page)
{
    NvBootError e = NvBootError_Success;
    SdmmcAccessRegion region;
    NvU32 BlocksPerPartition = s_SdmmcContext->EmmcBootPartitionSize >>
                               s_SdmmcContext->BlockSizeLog2;

    // If boot partition size is zero, then the card is either eSD or
    // eMMC version is < 4.3.
    if (s_SdmmcContext->EmmcBootPartitionSize == 0)
    {
        s_SdmmcContext->CurrentAccessRegion = SdmmcAccessRegion_UserArea;
        return e;
    }
    // This will not work always, if the request is a multipage one.
    // But this driver never gets multipage requests.
    if ( (*Block) < BlocksPerPartition )
    {
        region = SdmmcAccessRegion_BootPartition1;
    }
    else if ( (*Block) < (BlocksPerPartition << 1) )
    {
        region = SdmmcAccessRegion_BootPartition2;
        *Block = (*Block) - BlocksPerPartition;
    }
    else
    {
        region = SdmmcAccessRegion_UserArea;
        *Block = (*Block) - (BlocksPerPartition << 1);
    }

    if (region != s_SdmmcContext->CurrentAccessRegion)
        NV_BOOT_CHECK_ERROR(EmmcSelectAccessRegion(region));
    return e;
}

static NvBootError EmmcGetExtCsd(void)
{
    NvBootError e;
    NvBootDeviceStatus DevStatus;
    NvU8* pBuffer = (NvU8*)&s_SdmmcContext->SdmmcInternalBuffer[0];

    // Set num of blocks to read to 1.
    HwSdmmcSetNumOfBlocks((1 << s_SdmmcContext->PageSizeLog2), 1);
    // Setup Dma.
    HwSdmmcSetupDma((NvU8*)s_SdmmcContext->SdmmcInternalBuffer,
        (1 << s_SdmmcContext->PageSizeLog2));
    // Send SEND_EXT_CSD(CMD8) command to get boot partition size.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_EmmcSendExtendedCsd,
        0, SdmmcResponseType_R1, NV_TRUE));
    // If response fails, return error. Nothing to clean up.
    NV_BOOT_CHECK_ERROR(EmmcVerifyResponse(SdmmcCommand_EmmcSendExtendedCsd,
        NV_FALSE));
    s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_ReadInProgress;
    s_SdmmcContext->ReadStartTime = NvBootUtilGetTimeUS();
    do
    {
        DevStatus = NvBootSdmmcQueryStatus();
    } while ( (DevStatus != NvBootDeviceStatus_Idle) &&
              (DevStatus == NvBootDeviceStatus_ReadInProgress) );
    if (DevStatus != NvBootDeviceStatus_Idle)
        return NvBootError_DeviceError;
    s_SdmmcContext->EmmcBootPartitionSize =
        // The partition size comes in 128KB units.
        // Left shift it by 17 to get it multiplied by 128KB.
        (pBuffer[EMMC_ECSD_BOOT_PARTITION_SIZE_OFFSET] << 17);
    s_SdmmcContext->PowerClass26MHz360V = pBuffer[EMMC_ECSD_POWER_CL_26_360_OFFSET];
    s_SdmmcContext->PowerClass52MHz360V = pBuffer[EMMC_ECSD_POWER_CL_52_360_OFFSET];
    s_SdmmcContext->PowerClass26MHz195V = pBuffer[EMMC_ECSD_POWER_CL_26_195_OFFSET];
    s_SdmmcContext->PowerClass52MHz195V = pBuffer[EMMC_ECSD_POWER_CL_52_195_OFFSET];
    s_SdmmcContext->PowerClass52MHzDdr360V = pBuffer[EMMC_ECSD_POWER_CL_DDR_52_360_OFFSET];
    s_SdmmcContext->PowerClass52MHzDdr195V = pBuffer[EMMC_ECSD_POWER_CL_DDR_52_195_OFFSET];
    s_SdmmcContext->BootConfig = pBuffer[EMMC_ECSD_BOOT_CONFIG_OFFSET];
    if (s_SdmmcContext->IsHighCapacityCard)
    {
        s_SdmmcContext->NumOfBlocks = (pBuffer[EMMC_ECSD_SECTOR_COUNT_0_OFFSET] |
                                      (pBuffer[EMMC_ECSD_SECTOR_COUNT_1_OFFSET] << 8) |
                                      (pBuffer[EMMC_ECSD_SECTOR_COUNT_2_OFFSET] << 16) |
                                      (pBuffer[EMMC_ECSD_SECTOR_COUNT_3_OFFSET] << 24));
        PRINT_SDMMC_MESSAGES("\r\nEcsd NumOfBlocks=%d", s_SdmmcContext->NumOfBlocks);
    }

    PRINT_SDMMC_MESSAGES("\r\nBootPartition Size=%d",
        s_SdmmcContext->EmmcBootPartitionSize);
    PRINT_SDMMC_MESSAGES("\r\nPowerClass26MHz360V=%d, PowerClass52MHz360V=%d, "
        "PowerClass26MHz195V=%d, PowerClass52MHz195V=%d",
        s_SdmmcContext->PowerClass26MHz360V, s_SdmmcContext->PowerClass52MHz360V,
        s_SdmmcContext->PowerClass26MHz195V, s_SdmmcContext->PowerClass52MHz195V);
    PRINT_SDMMC_MESSAGES("\r\nCurrentPowerClass=%d, CardType=%d",
        pBuffer[EMMC_ECSD_POWER_CLASS_OFFSET], pBuffer[EMMC_ECSD_CARD_TYPE_OFFSET]);

    s_SdmmcContext->CardSupportSpeed = pBuffer[EMMC_ECSD_CARD_TYPE_OFFSET];
    s_SdmmcContext->CardBusWidth = pBuffer[EMMC_ECSD_BUS_WIDTH];
    return e;
}

static NvU32 EmmcGetPowerClass(void)
{
    NvU32 PowerClass;

    // set Ddr mode from getextcsd data & width supported
    if(s_SdmmcContext->IsDdrMode)
        PowerClass = s_SdmmcContext->IsHighVoltageRange ?
                     s_SdmmcContext->PowerClass52MHzDdr360V :
                     s_SdmmcContext->PowerClass52MHzDdr195V;
    else if (s_SdmmcContext->IsHighVoltageRange)
        PowerClass = s_SdmmcContext->HighSpeedMode ?
                     s_SdmmcContext->PowerClass52MHz360V :
                     s_SdmmcContext->PowerClass26MHz360V;
    else
        PowerClass = s_SdmmcContext->HighSpeedMode ?
                     s_SdmmcContext->PowerClass52MHz195V :
                     s_SdmmcContext->PowerClass26MHz195V;
    /*
     * In the above power class, lower 4 bits give power class requirement for
     * for 4-bit data width and upper 4 bits give power class requirement for
     * for 8-bit data width.
     */
    if ((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_4Bit) ||
        (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit))
        PowerClass = (PowerClass >> EMMC_ECSD_POWER_CLASS_4_BIT_OFFSET) &
                     EMMC_ECSD_POWER_CLASS_MASK;
    else if ((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_8Bit) ||
        (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit))
        PowerClass = (PowerClass >> EMMC_ECSD_POWER_CLASS_8_BIT_OFFSET) &
                     EMMC_ECSD_POWER_CLASS_MASK;
    else //if (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_1Bit)
        PowerClass = 0;
    return PowerClass;
}

static NvBootError EmmcSetPowerClass(void)
{
    NvU32 CmdArg;
    NvU32 PowerClassToSet;
    NvBootError e = NvBootError_Success;

    PowerClassToSet = EmmcGetPowerClass();
    // Select best possible configuration here.
    while (PowerClassToSet > s_SdmmcContext->MaxPowerClassSupported)
    {
        if (s_SdmmcContext->HighSpeedMode)
        {
            // Disable high speed and see, if it can be supported.
            s_SdmmcContext->CardSupportsHighSpeedMode = NV_FALSE;
            // Find out clock divider for card clock again for normal speed.
            HwSdmmcCalculateCardClockDivisor();
        }
        else if ((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_8Bit) ||
        (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit))
            s_SdmmcContext->DataWidth = NvBootSdmmcDataWidth_4Bit;
        else if ((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_4Bit) ||
        (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit))
            s_SdmmcContext->DataWidth = NvBootSdmmcDataWidth_1Bit;
        PowerClassToSet = EmmcGetPowerClass();
    }
    if (PowerClassToSet)
    {
        PRINT_SDMMC_MESSAGES("\r\nSet Power Class to %d", PowerClassToSet);
        CmdArg = EMMC_SWITCH_SELECT_POWER_CLASS_ARG |
                 (PowerClassToSet << EMMC_SWITCH_SELECT_POWER_CLASS_OFFSET);
        NV_BOOT_CHECK_ERROR(EmmcSendSwitchCommand(CmdArg));
    }
    s_SdmmcBitInfo->PowerClassUnderUse = PowerClassToSet;
    return e;
}

static void EmmcEnableHighSpeed(void)
{
    NvBootError e;
    NvU8* pBuffer = (NvU8*)&s_SdmmcContext->SdmmcInternalBuffer[0];

    // Clear controller's high speed bit.
    HwSdmmcEnableHighSpeed(NV_FALSE);
    // Enable the High Speed Mode, if required.
    if (s_SdmmcContext->HighSpeedMode)
    {
        PRINT_SDMMC_MESSAGES("\r\n\r\nSet High speed to %d",
            s_SdmmcContext->HighSpeedMode);
        NV_BOOT_CHECK_ERROR_CLEANUP(EmmcSendSwitchCommand(
            EMMC_SWITCH_HIGH_SPEED_ENABLE_ARG));
        // Set the clock for data transfer.
        HwSdmmcSetCardClock(NvBootSdmmcCardClock_DataTransfer);
        // Validate high speed mode bit from card here.
        NV_BOOT_CHECK_ERROR_CLEANUP(EmmcGetExtCsd());
        if (pBuffer[EMMC_ECSD_HS_TIMING_OFFSET])
        {
            // As per Hw team, it should not be enabled. See Hw bug number
            // AP15#353684/AP20#478599.
            //HwSdmmcEnableHighSpeed(NV_TRUE);
            return;
        }
    fail:
        // If enable high speed fails, run in normal speed.
        PRINT_SDMMC_ERRORS("\r\nEmmcEnableHighSpeed Failed");
        s_SdmmcContext->CardSupportsHighSpeedMode = NV_FALSE;
        // Find out clock divider for card clock again.
        HwSdmmcCalculateCardClockDivisor();
    }
}

static NvBootError EmmcSetBusWidth(void)
{
    NvU32 CmdArg;
    NvBootError e = NvBootError_Success;

    // Send SWITCH(CMD6) Command to select bus width.
    PRINT_SDMMC_MESSAGES("\r\n\r\nChange Data width to %d(0->1bit, 1->4bit,"
        " 2->8-bit, 5->4-bit DDR, 6->8-bit DDR)", s_SdmmcContext->DataWidth);
    CmdArg = EMMC_SWITCH_BUS_WIDTH_ARG |
             (s_SdmmcContext->DataWidth << EMMC_SWITCH_BUS_WIDTH_OFFSET);
    NV_BOOT_CHECK_ERROR(EmmcSendSwitchCommand(CmdArg));
    HwSdmmcSetDataWidth(s_SdmmcContext->DataWidth);
    return e;
}
static NvBootError EmmcEnableDDRSupport(void)
{
    NvU32 HostControl2Reg=0;
    NvU32 StcReg;
    NvU32 CapabilityReg;
    NvBootError e = NvBootError_Success;

    if((s_SdmmcContext->CardSupportSpeed & EMMC_ECSD_CT_HS_DDR_52_180_300_MASK )
            == EMMC_ECSD_CT_HS_DDR_52_180_300)
    {
        // card operates @ eiher 1.8v or 3.0v I/O
        // Power class selection
        // eMMC ddr works @ 1.8v and 3v
        // change the bus speed mode.
        // switch command to select card type -- ddr hs 1.8/3v

        // set HS_TIMING to 1 before setting the ddr mode data width..
        #if NVBOOT_TARGET_FPGA
            if(!s_SdmmcContext->HighSpeedMode)
            {
                s_SdmmcContext->HighSpeedMode = NV_TRUE;
                EmmcEnableHighSpeed();
                s_SdmmcContext->HighSpeedMode = NV_FALSE;
            }
        #endif

        NV_BOOT_CHECK_ERROR(EmmcSetBusWidth());
        // for t114 DDR 50 is not advertised.
        // check bug # http://nvbugs/832177 for more details.
        //set the ddr mode in Host controller and other misc things..
        // DDR support is available by fuse bit
        // check capabilities register for Ddr support
        // read capabilities and capabilities higher reg
        NV_SDMMC_READ(CAPABILITIES, CapabilityReg);

        if NV_DRF_VAL(SDMMC, CAPABILITIES, HIGH_SPEED_SUPPORT, CapabilityReg)
        {
            // reset SD clock enable
            NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
            StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                        SD_CLOCK_EN, DISABLE, StcReg);
            NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);

            // set DDR50 UHS Mode
            NV_SDMMC_READ(AUTO_CMD12_ERR_STATUS, HostControl2Reg);
            HostControl2Reg |= NV_DRF_DEF(SDMMC, AUTO_CMD12_ERR_STATUS,
                                    UHS_MODE_SEL, DDR50);
            NV_SDMMC_WRITE(AUTO_CMD12_ERR_STATUS, HostControl2Reg);

            // set enable SD clock
            StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                        SD_CLOCK_EN, ENABLE, StcReg);
            NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);

        }

    }
    else if((s_SdmmcContext->CardSupportSpeed & EMMC_ECSD_CT_HS_DDR_52_120_MASK)
            == EMMC_ECSD_CT_HS_DDR_52_120)
    {
        // NOT supported
        e = NvBootError_Unimplemented;
    }
    else
    {
        // should not be here!!
        // ASSERT
        e = NvBootError_InvalidParameter;
    }
    return e;
}

static NvBootError EmmcIdentifyCard(void)
{
    NvBootError e = NvBootError_Success;
    SdmmcAccessRegion NvBootPartitionEn = SdmmcAccessRegion_UserArea;
    NvU8 CardSpeed = 0;

    // Set Clock rate to 375KHz for identification of card.
    HwSdmmcSetCardClock(NvBootSdmmcCardClock_Identification);
    // Send GO_IDLE_STATE(CMD0) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_GoIdleState,
        0, SdmmcResponseType_NoResponse, NV_FALSE));
    // This sends SEND_OP_COND(CMD1) Command and finds out address mode and
    // capacity status.
    NV_BOOT_CHECK_ERROR(EmmcGetOpConditions());
    // Send ALL_SEND_CID(CMD2) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_AllSendCid,
        0, SdmmcResponseType_R2, NV_FALSE));
    // Copy card identification data to sdmmc bit info.
    NvBootUtilMemcpy(s_SdmmcBitInfo->Cid, s_SdmmcContext->SdmmcResponse,
        sizeof(s_SdmmcBitInfo->Cid));
    // Set RCA to Card Here. It should be greater than 1 as JEDSD spec.
    s_SdmmcContext->CardRca = (2 << SDMMC_RCA_OFFSET);
    // Send SET_RELATIVE_ADDR(CMD3) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_EmmcSetRelativeAddress,
        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE));
    // Get Card specific data. We can get it at this stage of identification.
    NV_BOOT_CHECK_ERROR(SdmmcGetCsd());
    // Send SELECT/DESELECT_CARD(CMD7) Command to place the card in tran state.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SelectDeselectCard,
        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE));
    // Card should be in transfer state now. Confirm it.
    if (SdmmcIsCardInTransferState() == NV_FALSE)
        return NvBootError_DeviceError;
    PRINT_SDMMC_MESSAGES("\r\nCard is identification is  Successful");
    PRINT_SDMMC_MESSAGES("\r\nConfig Card");
    // Find out clock divider for card clock and high speed mode requirement.
    HwSdmmcCalculateCardClockDivisor();

    if (s_SdmmcContext->SpecVersion >= 4) // v4.xx
    {
        // Bus width can only be changed to 4-bit/8-bit after required power class
        // is set. To get Power class, we need to read Ext CSD. As we don't know
        // the power class required for 4-bit/8-bit, we need to read Ext CSD
        // with 1-bit data width.
        PRINT_SDMMC_MESSAGES("\r\n\r\nSet Data width to 1-bit for ECSD");
        HwSdmmcSetDataWidth(NvBootSdmmcDataWidth_1Bit);
        // Set data clock rate to 20MHz.
        HwSdmmcSetCardClock(NvBootSdmmcCardClock_20MHz);
        // It is valid for v4.xx and above cards only.
        // EmmcGetExtCsd() Finds out boot partition size also.
        NV_BOOT_CHECK_ERROR(EmmcGetExtCsd());
        //
        // Powerclass setting controlled via MaxPowerClassSupported param
        // Select the power class now.
        if(s_SdmmcContext->MaxPowerClassSupported)
            NV_BOOT_CHECK_ERROR(EmmcSetPowerClass());
        // Enable the High Speed Mode, if required.
        EmmcEnableHighSpeed();
    }

    // Set the clock for data transfer.
    HwSdmmcSetCardClock(NvBootSdmmcCardClock_DataTransfer);

    // v4.4 and ddr supported enable ddr
    if ((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit) ||
        (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit))
    {
        // v4.4 and ddr supported enable ddr
        CardSpeed = (s_SdmmcContext->CardSupportSpeed &
                                EMMC_ECSD_CT_HS_DDR_MASK);
        if((CardSpeed & EMMC_ECSD_CT_HS_DDR_52_120) ||
            (CardSpeed & EMMC_ECSD_CT_HS_DDR_52_180_300))
        {
            NV_BOOT_CHECK_ERROR(EmmcEnableDDRSupport());
        }
        else
        {
            // set the bus width since the card does not support the requested mode data width..
            if (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit)
                s_SdmmcContext->DataWidth = NvBootSdmmcDataWidth_4Bit;
            else if(s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit)
                s_SdmmcContext->DataWidth = NvBootSdmmcDataWidth_8Bit;

            NV_BOOT_CHECK_ERROR(EmmcSetBusWidth());
        }
    }
    else
    {
        NV_BOOT_CHECK_ERROR(EmmcSetBusWidth());
    }

    // Select boot partition1 here.
    // also the boot partition should be checked (enabled) for access the partition.
    NvBootPartitionEn = (SdmmcAccessRegion)((s_SdmmcContext->BootConfig >>
                                        EMMC_ECSD_BC_BPE_OFFSET)
                                    &&  EMMC_ECSD_BC_BPE_MASK);

    if (s_SdmmcContext->EmmcBootPartitionSize != 0 &&
        ((NvBootPartitionEn == EMMC_ECSD_BC_BPE_BAP1) ||
        (NvBootPartitionEn == EMMC_ECSD_BC_BPE_BAP2)))
        NV_BOOT_CHECK_ERROR(EmmcSelectAccessRegion(NvBootPartitionEn));

    s_SdmmcBitInfo->BootFromBootPartition =
                                  s_SdmmcContext->EmmcBootPartitionSize ? 1 : 0;
    return e;
}

/* Public Function Definitions. */
void
NvBootSdmmcGetParams(
    const NvU32 ParamIndex,
    NvBootSdmmcParams **Params)
{
    NvU32 Index, ClockDivider20Mhz;
    NV_ASSERT(Params != NULL);

    // Extract Data width from Param Index, which comes from fuses.
    Index = NV_DRF_VAL(SDMMC_DEVICE, CONFIG, DATA_WIDTH, ParamIndex);
    /*
     * One Fuse bit is used for Data width. The value starting from
     * zero to one, corresponds to data widths 4 and 8 bits.
     * The enum value for 4bit data width is 1 and 2 for 8-bit data width enum.
     */
    s_DefaultSdmmcParams.DataWidth = (NvBootSdmmcDataWidth)(Index + 1);
    /*
     * Extract the Ddr selection from Param Index, which comes from fuses.
     * One Fuse bit is used for Ddr mode selection. The value
     * starting from zero to one, corresponds to normal and DDR mode.
     */
    s_FuseInfo.DdrMode = NV_DRF_VAL(SDMMC_DEVICE, CONFIG,
                                    DDR_MODE, ParamIndex);
    /*
     * Extract the voltage range to use from Param Index, which comes from fuses.
     * Two Fuse bits are used for voltage range selection. The value starting
     * from zero to thress, corresponds to query, high, dual and low voltage ranges.
     */
    s_FuseInfo.VoltageRange = (NvBootSdmmcVoltageRange)NV_DRF_VAL(SDMMC_DEVICE,
                                CONFIG, VOLTAGE_RANGE, ParamIndex);
    /*
     * Extract the Boot mode support from Param Index, which comes from fuses.
     * One Fuse bit is used for Boot mode support disable/enable. The value
     * starting from zero to one, corresponds to enable and disable.
     */
    s_FuseInfo.DisableBootMode = NV_DRF_VAL(SDMMC_DEVICE, CONFIG,
                                    DISABLE_BOOT_MODE, ParamIndex);
    /*
     * SDMMC4 pads voltage
     */
    s_FuseInfo.Sdmmc4PadsVoltage = NV_DRF_VAL(SDMMC_DEVICE, CONFIG,
                                    SDMMC4_PADS_VOLTAGE, ParamIndex);
    /*
     * SDMMC4 Clock Divider
     */
    s_FuseInfo.Sdmmc4ClkDivider = NV_DRF_VAL(SDMMC_DEVICE, CONFIG,
                                    SDMMC4_CLOCK_DIVIDER, ParamIndex);

    /*
     * SDMMC4 Multi Page
     */
    s_FuseInfo.SdmmcMultiPageSupport = NV_DRF_VAL(SDMMC_DEVICE, CONFIG,
                                SDMMC4_MULTI_PAGE_READ, ParamIndex);

    /* Set the SDMMC clock source frquency as 20MHz before BCT is read. This
     * is for backward compatibility of cards, which don't support high speed mode.
     * The Clock Source to eMMC is PLLP and which is operating at 408MHz.
     * To find out clock divisor value, divide it by 21 (408/21 =19.43) and
     * convert it to ceiling value, which is 21. 408/21 = 19.43MHz. The frequency
     * should not be more than 20MHz to run in normal speed for EMMC cards of
     * version less than 4.3.
     * T124 decreases the calculated divider (for 20 Mhz) by *twice* the fuse value to achieve
     * higher frequency
     */
    
    ClockDivider20Mhz = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, 20);
    if(ClockDivider20Mhz > (s_FuseInfo.Sdmmc4ClkDivider))
        s_DefaultSdmmcParams.ClockDivider = ClockDivider20Mhz - (s_FuseInfo.Sdmmc4ClkDivider);
    else
        s_DefaultSdmmcParams.ClockDivider = ClockDivider20Mhz;
    /*
     * Max Power class supported by target board is unknown. Bct would give us
     * the Max power class supported. So, Till that time, Let it be 0 and work
     * with power calss 0..Target board must support power class 0.
     */
    s_DefaultSdmmcParams.MaxPowerClassSupported = 0;

    s_DefaultSdmmcParams.MultiPageSupport = s_FuseInfo.SdmmcMultiPageSupport;

    s_SdmmcBitInfo->FuseDataWidth = s_DefaultSdmmcParams.DataWidth;
    s_SdmmcBitInfo->FuseVoltageRange = s_FuseInfo.VoltageRange;
    s_SdmmcBitInfo->FuseDisableBootMode = s_FuseInfo.DisableBootMode;
    s_SdmmcBitInfo->FuseDdrMode = s_FuseInfo.DdrMode;

    /* set the data width param depending upon speed mode setting
    */
    if(s_FuseInfo.DdrMode)
    {
        if(s_DefaultSdmmcParams.DataWidth == NvBootSdmmcDataWidth_4Bit)
            s_DefaultSdmmcParams.DataWidth = NvBootSdmmcDataWidth_Ddr_4Bit;
        else if(s_DefaultSdmmcParams.DataWidth == NvBootSdmmcDataWidth_8Bit)
            s_DefaultSdmmcParams.DataWidth = NvBootSdmmcDataWidth_Ddr_8Bit;
    }

    *Params = (NvBootSdmmcParams*)&s_DefaultSdmmcParams;
    PRINT_SDMMC_MESSAGES("\r\nParamIndex=0x%x, DataWidth=%d (1->4bit, 2->8bit), "
        "CardType=%d (0->EMMC), VoltageRange=%d(0->query, 1->high, "
        "2-> dual, 3->low), DisableBootMode=%d, ClockDivider=%d, MaxPowerClass"
        "Supported=%d", ParamIndex, s_DefaultSdmmcParams.DataWidth,
        NvBootSdmmcCardType_Emmc, s_FuseInfo.VoltageRange,
        s_FuseInfo.DisableBootMode, s_DefaultSdmmcParams.ClockDivider,
        s_DefaultSdmmcParams.MaxPowerClassSupported);
}

NvBool
NvBootSdmmcValidateParams(
    const NvBootSdmmcParams *Params)
{
    NV_ASSERT(Params != NULL);
    PRINT_SDMMC_MESSAGES("\r\nValidateParams, DataWidth=%d (1->4bit, 2->8bit), "
        "CardType=%d (0->EMMC), VoltageRange=%d(0->query, 1->high,2-> dual,"
        "3->low), DisableBootMode=%d, ClockDivider=%d, MaxPowerClassSupp"
        "orted=%d", Params->DataWidth, NvBootSdmmcCardType_Emmc,
        s_FuseInfo.VoltageRange, s_FuseInfo.DisableBootMode,
        Params->ClockDivider, Params->MaxPowerClassSupported);

    if ( (Params->ClockDivider < SDMMC_MIN_CLOCK_DIVIDER_SUPPORTED) ||
         (Params->ClockDivider > SDMMC_MAX_CLOCK_DIVIDER_SUPPORTED) )
        return NV_FALSE;
    // Multipage Support
    if(Params->MultiPageSupport > SDMMC_MAX_BLOCK_SIZE_LOG2)
        return NV_FALSE;
    // change this to support DDR too
    if ( (Params->DataWidth != NvBootSdmmcDataWidth_4Bit) &&
         (Params->DataWidth != NvBootSdmmcDataWidth_8Bit) &&
         (Params->DataWidth != NvBootSdmmcDataWidth_Ddr_4Bit) &&
         (Params->DataWidth != NvBootSdmmcDataWidth_Ddr_8Bit))
        return NV_FALSE;
    if (Params->MaxPowerClassSupported > SDMMC_MAX_POWER_CLASS_SUPPORTED)
        return NV_FALSE;
    return NV_TRUE;
}

void
NvBootSdmmcGetBlockSizes(
    const NvBootSdmmcParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);
    NV_ASSERT(s_SdmmcContext != NULL);

    *BlockSizeLog2 = s_SdmmcContext->BlockSizeLog2;
    *PageSizeLog2 = ( s_SdmmcContext->PageSizeLog2 + s_SdmmcContext->MultiPageSupported );
    PRINT_SDMMC_MESSAGES("\r\nBlockSize=%d, PageSize=%d, PagesPerBlock=%d",
        (1 << s_SdmmcContext->BlockSizeLog2),(1 << s_SdmmcContext->PageSizeLog2),
        (1 << s_SdmmcContext->PagesPerBlockLog2));
}

NvBootError
NvBootSdmmcInit(
    const NvBootSdmmcParams *Params,
    NvBootSdmmcContext *Context)
{
    NvBootError e = NvBootError_Success;
    NvU32 PageSize = 0;
    NV_ASSERT(Params != NULL);
    NV_ASSERT(Context != NULL);
    NV_ASSERT(Params->ClockDivider >= SDMMC_MIN_CLOCK_DIVIDER_SUPPORTED &&
        Params->ClockDivider <= SDMMC_MAX_CLOCK_DIVIDER_SUPPORTED);

    NV_ASSERT( (Params->DataWidth == NvBootSdmmcDataWidth_4Bit) ||
        (Params->DataWidth == NvBootSdmmcDataWidth_8Bit) ||
        (Params->DataWidth == NvBootSdmmcDataWidth_Ddr_4Bit) ||
        (Params->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit) );

    // Stash the pointer to the context structure.
    s_SdmmcContext = Context;
    s_SdmmcContext->taac = 0;
    s_SdmmcContext->nsac = 0;
    s_SdmmcContext->ClockDivisor = Params->ClockDivider;
    s_SdmmcContext->DataWidth = Params->DataWidth;
    s_SdmmcContext->MaxPowerClassSupported = Params->MaxPowerClassSupported;
    s_SdmmcContext->MultiPageSupported =  Params->MultiPageSupport;
    s_SdmmcContext->CardSupportsHighSpeedMode = NV_FALSE;
    s_SdmmcContext->ReadTimeOutInUs = SDMMC_READ_TIMEOUT_IN_US;
    s_SdmmcContext->EmmcBootPartitionSize = 0;
    s_SdmmcContext->CurrentClockRate = NvBootSdmmcCardClock_Identification;
    s_SdmmcContext->CurrentAccessRegion = SdmmcAccessRegion_Unknown;
    s_SdmmcContext->BootModeReadInProgress = NV_FALSE;

    // Initialize the Hsmmc Hw controller.
    NV_BOOT_CHECK_ERROR(HwSdmmcInitController());

    // Check for card is present. NOT required
    // Bug # 826908

    if (s_FuseInfo.DisableBootMode == NV_FALSE)
    {
        PRINT_SDMMC_MESSAGES("\r\nBootMode Enabled");
        e = EmmcReadDataInBootMode(&s_SdmmcContext->SdmmcBootModeBuffer[0], 1);
        if (e != NvBootError_Success)
        {
            // Reset data line.
            NV_BOOT_CHECK_ERROR(HwSdmmcInitController());
        }
    }
    // only eMMC is supported as boot media
    e = EmmcIdentifyCard();
    if (e != NvBootError_Success)
    {
        return e;
    }
    s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_Idle;
    s_SdmmcBitInfo->DiscoveredCardType = NvBootSdmmcCardType_Emmc;
    s_SdmmcBitInfo->BootModeReadSuccessful = s_IsBootModeDataValid;

    // enable block length setting, if CYA is cleared.
    if((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_4Bit) ||
            (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_8Bit))
    {
        PageSize = (1 << s_SdmmcContext->PageSizeLog2);
        // Send SET_BLOCKLEN(CMD16) Command.
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SetBlockLength,
            PageSize, SdmmcResponseType_R1, NV_FALSE));
        NV_BOOT_CHECK_ERROR(EmmcVerifyResponse(SdmmcCommand_SetBlockLength,
            NV_FALSE));
    }

    return e;
}

NvBootError
NvBootSdmmcReadPage(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *pBuffer)
{
    NvBootError e;
    NvU32 CommandArg;
    NvU32 ActualBlockToRead;
    NvU32 Page2Access = Page * (1 << s_SdmmcContext->MultiPageSupported);//PageToRead;
    NvU32 Block2Access = Block;
    NvU32 PageSize = (1 << s_SdmmcContext->PageSizeLog2);

    NV_ASSERT(Page < (1 << s_SdmmcContext->PagesPerBlockLog2));
    NV_ASSERT(pBuffer != NULL);
    PRINT_SDMMC_MESSAGES("\r\nRead Block=%d, Page=%d", Block, Page);

    if(s_SdmmcContext->MultiPageSupported)
        s_SdmmcBitInfo->NumPagesRead += (1 << s_SdmmcContext->MultiPageSupported);// define PageToRead to verify
    else
        s_SdmmcBitInfo->NumPagesRead++;
    if ( (s_IsBootModeDataValid == NV_TRUE) && (Block == 0) && (Page == 0) && (!s_SdmmcContext->MultiPageSupported))
    {
        // the 0th page of 0th block will read in boot mode, if boot mode is
        // enabled. So, give it back from buffer.
        NvBootUtilMemcpy(pBuffer, &s_SdmmcContext->SdmmcBootModeBuffer[0],
            PageSize);
        return NvBootError_Success;
    }

    // If data line ready times out, try to recover from errors.
    if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
        NV_BOOT_CHECK_ERROR(HwSdmmcRecoverControllerFromErrors(NV_TRUE));
    // Select access region.This will intern changes block and page addresses
    // based on the region the request falls in.
    NV_BOOT_CHECK_ERROR(SdmmcSelectAccessRegion(&Block2Access, &Page2Access));
    PRINT_SDMMC_MESSAGES("\r\nRegion=%d(1->BP1, 2->BP2, 0->UP)Block2Access=%d, "
        "Page2Access=%d", s_SdmmcContext->CurrentAccessRegion, Block2Access,
        Page2Access);

    // Find out the Block to read from MoviNand.
    ActualBlockToRead = (Block2Access << s_SdmmcContext->PagesPerBlockLog2) +
                        Page2Access;
    /*
     * If block to read is beyond card's capacity, then some Emmc cards are
     * responding with error back and continue to work. Some are not responding
     * for this and for subsequent valid operations also.
     */
    //if (ActualBlockToRead >= s_SdmmcContext->NumOfBlocks)
    //    return NvBootError_IllegalParameter;
    // Set number of blocks to read to 1.
    if(s_SdmmcContext->MultiPageSupported)
        HwSdmmcSetNumOfBlocks(PageSize, (1 << s_SdmmcContext->MultiPageSupported));
    else
        HwSdmmcSetNumOfBlocks(PageSize, 1);
    // Set up command arg.
    if (s_SdmmcContext->IsHighCapacityCard)
        CommandArg = ActualBlockToRead;
    else
        CommandArg = (ActualBlockToRead << s_SdmmcContext->PageSizeLog2);
    PRINT_SDMMC_MESSAGES("\r\nActualBlockToRead=%d, CommandArg=%d",
        ActualBlockToRead, CommandArg);
    // Store address of pBuffer in sdmmc context
    s_SdmmcContext->CurrentReadBufferAddress = pBuffer;
    // Setup Dma.
    HwSdmmcSetupDma(pBuffer, PageSize);

    if(s_SdmmcContext->MultiPageSupported)
    {
        // settingthe block count command
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SetBlockCount,
        (1 << s_SdmmcContext->MultiPageSupported), SdmmcResponseType_R1, NV_FALSE));

        // Send command to card.
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_ReadMulti,
                                CommandArg, SdmmcResponseType_R1, NV_TRUE));
    }
    else
    {
        // Send command to card.
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_ReadSingle,
                                CommandArg, SdmmcResponseType_R1, NV_TRUE));
    }
    // If response fails, return error. Nothing to clean up.
    NV_BOOT_CHECK_ERROR_CLEANUP(EmmcVerifyResponse(SdmmcCommand_ReadSingle,
        NV_FALSE));
    s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_ReadInProgress;
    s_SdmmcContext->ReadStartTime = NvBootUtilGetTimeUS();
    return e;
fail:
    HwSdmmcAbortDataRead();
    return e;
}

NvBootDeviceStatus NvBootSdmmcQueryStatus(void)
{
    NvBootError e;
    NvU32 SdmaAddress;
    NvU32 TransferDone = 0;
    NvU32 InterruptStatusReg;
    NvU32 ErrorMask =
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_END_BIT_ERR, ERR) |
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_CRC_ERR, ERR) |
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_TIMEOUT_ERR,
                TIMEOUT) |
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_INDEX_ERR,
                ERR) |
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_END_BIT_ERR,
                END_BIT_ERR_GENERATED) |
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_CRC_ERR,
                CRC_ERR_GENERATED) |
              NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_TIMEOUT_ERR,
                TIMEOUT);
    NvU32 DmaBoundaryInterrupt = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS,
                                    DMA_INTERRUPT,GEN_INT);
    NvU32 DataTimeOutError =
                NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_TIMEOUT_ERR, TIMEOUT);

    if (s_SdmmcContext->DeviceStatus == NvBootDeviceStatus_ReadInProgress)
    {
        // Check whether Transfer is done.
        NV_SDMMC_READ(INTERRUPT_STATUS, InterruptStatusReg);
        TransferDone = NV_DRF_VAL(SDMMC, INTERRUPT_STATUS, XFER_COMPLETE,
                            InterruptStatusReg);
        // Check whether there are any errors.
        if (InterruptStatusReg & ErrorMask)
        {
            if ( (InterruptStatusReg & ErrorMask) == DataTimeOutError)
                s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_DataTimeout;
            else
            {
                s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_CrcFailure;
                s_SdmmcBitInfo->NumCrcErrors++;
            }
            // Recover from errors here.
            (void)HwSdmmcRecoverControllerFromErrors(NV_TRUE);
        }
        else if (InterruptStatusReg & DmaBoundaryInterrupt)
        {
            // Need to clear this DMA boundary interrupt and write SDMA address
            // again. Otherwise controller doesn't go ahead.
            NV_SDMMC_WRITE(INTERRUPT_STATUS, DmaBoundaryInterrupt);
            NV_SDMMC_READ(SYSTEM_ADDRESS, SdmaAddress);
            NV_SDMMC_WRITE(SYSTEM_ADDRESS, SdmaAddress);
        }
        else if (TransferDone)
        {
            s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_Idle;
            NV_SDMMC_WRITE(INTERRUPT_STATUS, InterruptStatusReg);
            if (s_SdmmcContext->BootModeReadInProgress == NV_FALSE)
            {
                // Check Whether there is any read ecc error.
                e = HwSdmmcSendCommand(SdmmcCommand_SendStatus,
                        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE);
                if (e == NvBootError_Success)
                {
                    e = EmmcVerifyResponse(SdmmcCommand_ReadSingle, NV_TRUE);
                }
                if (e != NvBootError_Success)
                    s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
            }
        }
        else if (NvBootUtilElapsedTimeUS(s_SdmmcContext->ReadStartTime) >
                 s_SdmmcContext->ReadTimeOutInUs)
        {
            s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
        }
    }
    return s_SdmmcContext->DeviceStatus;
}

void NvBootSdmmcShutdown(void)
{
    NvU32 StcReg;
    NvU32 PowerControlHostReg;

    // Stop the clock to SDMMC card.
    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    StcReg = NV_FLD_SET_DRF_DEF(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SD_CLOCK_EN, DISABLE, StcReg);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Disable the bus power.
    NV_SDMMC_READ(POWER_CONTROL_HOST, PowerControlHostReg);
    PowerControlHostReg = NV_FLD_SET_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                            SD_BUS_POWER, POWER_OFF, PowerControlHostReg);
    NV_SDMMC_WRITE(POWER_CONTROL_HOST, PowerControlHostReg);

    // Keep the controller in reset and disable the clock.
    NvBootResetSetEnable(NvBootResetDeviceId_SdmmcId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_SdmmcId, NV_FALSE);
    s_SdmmcContext = NULL;
}

NvBootError NvBootSdmmcGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
