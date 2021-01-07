/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
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
#include "arsdmmc.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_config.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_sdmmc_context.h"
#include "nvboot_sdmmc_int.h"
#include "nvboot_sdmmc_param.h"
#include "nvboot_util_int.h"
#include "nvboot_car_int.h"
#include "nvboot_dispatcher_int.h"
#include "nvboot_fuse_int.h"

//t214 specific
#include "project.h"
#include "nvboot_arc_int.h"
#include "arapb_misc_gp.h"
#include "nvboot_ahb_int.h"
#include "nvboot_platform_int.h"

#define DEBUG_SDMMC 0

#if DEBUG_SDMMC
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

NV_CT_ASSERT( (NvBootSdmmcDataWidth_8Bit == 2) );

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

#define QUOTIENT_CEILING(dividend, divisor) \
    ((dividend + divisor - 1) / divisor)


extern NvBootSdmmcParams s_DefaultSdmmcParams;
extern NvBootSdmmcContext *s_SdmmcContext;
extern NvBootSdmmcStatus* s_SdmmcBitInfo;

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

// Trim and Tap values for input and output data paths for SDR and DDR modes.
// For DDR52 mode and SDR mode
// SDMMC_VENDOR_CLOCK_CNTRL_0_TRIM_VAL : OB trimmer tap value = 13
// SDMMC_VENDOR_CLOCK_CNTRL_0_TAP_VAL : IB trimmer tap value = 9.
static const NvBootSdmmcVendorClkCtrl s_VendorClockCtrl = 
          { NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TAP_VAL, 9) |   //sdr
            NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TRIM_VAL, 13) ,
            NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TAP_VAL, 9) |   //ddr
            NV_DRF_NUM(SDMMCBOOT, VENDOR_CLOCK, TRIM_VAL, 13)
          };


// fwd declartion 
void NvBootSdmmcClockTable(void **SdmmcClockTable, ClockTableType *Id);

// Primary clock table for Sdmmc controller .
ClockInst s_SdmmcClkTable_Init[] = 
{
    // Assert Reset to Device, SS, Host (in case of Host boot), PadCtl
    Instc(Clk_W1b,  RST_DEV_L_SET, SET_SDMMC4_RST),

    // Configure the clock source with divider 17, which gives 24MHz.
    // (Pllp_out0@408M/17)
    Instc(Clk_Src,  CLK_SOURCE_SDMMC4, SDMMC4_CLK_SRC, PLLP_OUT0,
                                              SDMMC4_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(17, 0)),
    // Enable the clock.
    Instc(Clk_W1b,  CLK_ENB_L_SET, SET_CLK_ENB_SDMMC4),

    // Remove the controller from Reset.
    Instc(Clk_W1b,  RST_DEV_L_CLR, CLR_SDMMC4_RST),

    {0} // Null terminated.

};

// clock Divisor for Sdmmc controller 20Mhz IO clock.
const ClockInst s_SdmmcClkDiv_20[] = 
{
    // Configure the clock source with divider 21, which gives 19.42MHz.
    // (Pllp_out0@408M/21)
    Instc(Clk_Src,  CLK_SOURCE_SDMMC4, SDMMC4_CLK_SRC, PLLP_OUT0,
                                              SDMMC4_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(21, 0)),
    {0} // Null terminated.
};

// clock Divisor for Sdmmc controller 24Mhz IO clock.
const ClockInst s_SdmmcClkDiv_24[] = 
{
    // Configure the clock source with divider 17, which gives 24MHz.
    // (Pllp_out0@408M/17)
    Instc(Clk_Src,  CLK_SOURCE_SDMMC4, SDMMC4_CLK_SRC, PLLP_OUT0,
                                              SDMMC4_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(17, 0)),
    {0} // Null terminated.
};


// clock Divisor for Sdmmc controller 51Mhz IO clock.
const ClockInst s_SdmmcClkDiv_51[] = 
{
    // Configure the clock source with divider 14, which gives 51MHz.
    // (Pllp_out0@408M/8)
    Instc(Clk_Src,  CLK_SOURCE_SDMMC4, SDMMC4_CLK_SRC, PLLP_OUT0,
                                              SDMMC4_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(8, 0)),
    {0} // Null terminated.
};

// clock Divisor for Sdmmc controller 102Mhz IO clock.
const ClockInst s_SdmmcClkDiv_102[] = 
{
    // Configure the clock source with divider 6, which gives 51MHz.
    // (Pllp_out0@408M/4)
    Instc(Clk_Src,  CLK_SOURCE_SDMMC4, SDMMC4_CLK_SRC, PLLP_OUT0,
                                              SDMMC4_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(4, 0)),
    {0} // Null terminated.
};

void NvBootSdmmcClockTable(void **SdmmcClockTable, ClockTableType *Id)
{
    // Passing in list of tables.
    *SdmmcClockTable= s_SdmmcClkTable_Init;
    *Id = TYPE_SINGLE_TABLE;
}

static NvBootError HwSdmmcWaitForClkStable(void)
{
    uint32_t StcReg;
    uint32_t ClockReady;
    uint32_t TimeOut = SDMMC_TIME_OUT_IN_US;
    
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
    

static void HwSdmmcAutoCalibrate()
{
    NvU32 StcReg = 0;
    uint32_t ActiveInProgress = 0;
    uint32_t TimeOut = SDMMC_TIME_OUT_IN_US;

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
                                        10,
                                        StcReg);
            StcReg = NV_FLD_SET_DRF_NUM(APB_MISC_GP,
                                        EMMC4_PAD_CFGPADCTRL,
                                        CFG2TMC_EMMC4_PAD_DRVDN_COMP,
                                        10,
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

static void HwSdmmcSetDataWidth(NvBootSdmmcDataWidth DataWidth, uint8_t *DataWidthUnderUse)
{
    uint32_t PowerControlHostReg = 0;

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
    *DataWidthUnderUse = DataWidth;
}

NvBootError HwSdmmcWaitForDataLineReady(void)
{
    uint32_t PresentState;
    uint32_t DataLineActive;
    uint32_t TimeOut = s_SdmmcContext->ReadTimeOutInUs;

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
    uint32_t PresentState;
    uint32_t CmdInhibitData;
    uint32_t TimeOut = s_SdmmcContext->ReadTimeOutInUs;

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
    uint32_t PresentState;
    uint32_t CmdInhibitCmd;
    uint32_t TimeOut = SDMMC_COMMAND_TIMEOUT_IN_US;

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

static void
HwSdmmcReadResponse(
    SdmmcResponseType ResponseType,
    uint32_t* pRespBuffer)
{
    uint32_t* pTemp = pRespBuffer;

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


static NvBootError HwSdmmcWaitForCommandComplete(void)
{
    uint32_t CommandDone;
    uint32_t InterruptStatus;
    uint32_t TimeOutCounter = SDMMC_COMMAND_TIMEOUT_IN_US;
    uint32_t ErrorMask = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_INDEX_ERR,
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
    uint32_t retries = 2;
    uint32_t CommandXferMode;
    uint32_t InterruptStatus;
    uint32_t* pSdmmcResponse = &s_SdmmcContext->SdmmcResponse[0];

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

static void HwSdmmcEnableHighSpeed(NvBool Enable)
{
    uint32_t PowerControlHostReg = 0;

    NV_SDMMC_READ(POWER_CONTROL_HOST, PowerControlHostReg);
    PowerControlHostReg = NV_FLD_SET_DRF_NUM(SDMMC, POWER_CONTROL_HOST,
                            HIGH_SPEED_EN, ((Enable == NV_TRUE) ? 1 : 0),
                            PowerControlHostReg);
    NV_SDMMC_WRITE(POWER_CONTROL_HOST, PowerControlHostReg);
}


NvBootError HwSdmmcRecoverControllerFromErrors(NvBool IsDataCmd)
{
    uint32_t StcReg;
    uint32_t PresentState;
    uint32_t ResetInProgress;
    uint32_t InterruptStatus;
    uint32_t TimeOut = SDMMC_TIME_OUT_IN_US;
    uint32_t CommandError = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_INDEX_ERR,
                            ERR) |
                         NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_END_BIT_ERR,
                            END_BIT_ERR_GENERATED) |
                         NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_CRC_ERR,
                            CRC_ERR_GENERATED) |
                         NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, COMMAND_TIMEOUT_ERR,
                            TIMEOUT);
    uint32_t DataError = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_END_BIT_ERR,
                        ERR) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_CRC_ERR,
                        ERR) |
                      NV_DRF_DEF(SDMMC, INTERRUPT_STATUS, DATA_TIMEOUT_ERR,
                        TIMEOUT);
    uint32_t DataStateMask = NV_DRF_DEF(SDMMC, PRESENT_STATE, DAT_3_0_LINE_LEVEL,
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

void HwSdmmcAbortDataRead(void)
{
    uint32_t StcReg;
    uint32_t PresentState;
    uint32_t ResetInProgress;
    uint32_t TimeOut = SDMMC_TIME_OUT_IN_US;
    uint32_t DataStateMask = NV_DRF_DEF(SDMMC, PRESENT_STATE,
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



 NvBootError HwSdmmcInitController(void)
{
    uint32_t StcReg, RegData;
    NvBootError e;
    uint32_t CapabilityReg;
    uint32_t IntStatusEnableReg;
    uint32_t PowerControlHostReg;
    const NvBootSdmmcVendorClkCtrl *pVendorClockCtrl;


    // Configure the clock source with divider 17, which gives 24MHz.
    NV_BOOT_CHECK_ERROR(NvBootClocksEngine(s_SdmmcClkTable_Init, TYPE_SINGLE_TABLE));

    /// Update Clk source for DevStatus
    s_SdmmcBitInfo->ModuleClkSource    = CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4_0_SDMMC4_CLK_SRC_PLLP_OUT0;
    /// Update Clk enable for DevStatus
    s_SdmmcBitInfo->ModuleClkEnable    = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_SDMMC4_ENABLE;
    /// Update Clk rst status for DevStatus
    s_SdmmcBitInfo->ModuleClkRstStatus    = CLK_RST_CONTROLLER_RST_DEVICES_L_0_SWR_SDMMC4_RST_DISABLE;

    // Autocalibrate Bug # 839636
    HwSdmmcAutoCalibrate();

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

    if (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_Ddr_8Bit)
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

    // Find out what voltage is supported.
    NV_SDMMC_READ(CAPABILITIES, CapabilityReg);
    PowerControlHostReg = 0;

    // voltage setting from low to high
        if (NV_DRF_VAL(SDMMC, CAPABILITIES,\
        VOLTAGE_SUPPORT_1_8_V, CapabilityReg))
        {
            PowerControlHostReg |= NV_DRF_DEF(SDMMC, POWER_CONTROL_HOST,
                                SD_BUS_VOLTAGE_SELECT, V1_8);
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
                                SD_BUS_VOLTAGE_SELECT, V3_3);
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


NvBootError
HwSdmmcSendCommand(
    SdmmcCommand CommandIndex,
    uint32_t CommandArg,
    SdmmcResponseType ResponseType,
    NvBool IsDataCmd)
{
    NvBootError e;
    uint32_t retries = 3;
    uint32_t CommandXferMode;
    uint32_t InterruptStatus;
    uint32_t* pSdmmcResponse = &s_SdmmcContext->SdmmcResponse[0];
    NvBool DataXferDir = IsDataCmd;
    NvBool AutoCmdSel = NV_FALSE;
    NV_ASSERT(ResponseType < SdmmcResponseType_Num);
    PRINT_SDMMC_MESSAGES("\r\n\r\n     Sending CMD%d", CommandIndex);
    PRINT_SDMMC_MESSAGES("\r\nCmd Index=0x%x, Arg=0x%x, RespType=%d, data=%d",
        CommandIndex, CommandArg, ResponseType, IsDataCmd);
    // Wait till Controller is ready.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForCmdInhibitCmd());

    if (IsDataCmd)
    {
        switch (CommandIndex)
        {
            case SdmmcCommand_WriteMulti:
                 AutoCmdSel = 1;
                 DataXferDir = 0;
                 break;
            case SdmmcCommand_WriteSingle:
                 DataXferDir = 0;
                 break;
            case SdmmcCommand_ReadMulti:
            case SdmmcCommand_ReadSingle:
                 DataXferDir = 1;
                 break;
            default:
                 break;

        }
    }
        
    CommandXferMode =
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, COMMAND_INDEX, CommandIndex) | 
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_PRESENT_SELECT, IsDataCmd) | 
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DATA_XFER_DIR_SEL, DataXferDir) | 
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, AUTO_CMD12_EN, AutoCmdSel) | 
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, BLOCK_COUNT_EN, IsDataCmd) | 
        NV_DRF_NUM(SDMMC, CMD_XFER_MODE, DMA_EN, IsDataCmd);

    if((CommandIndex == SdmmcCommand_ReadMulti) || 
        (CommandIndex == SdmmcCommand_WriteMulti))
        {
       CommandXferMode |= 
            NV_DRF_DEF(SDMMC, CMD_XFER_MODE,MULTI_BLOCK_SELECT , ENABLE);
        }
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
        {
            break;
        }
        // Recover Controller from Errors.
        e = HwSdmmcRecoverControllerFromErrors(IsDataCmd);
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

void HwSdmmcSetNumOfBlocks(uint32_t BlockLength, uint32_t NumOfBlocks)
{
    uint32_t BlockReg;

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

 void HwSdmmcSetupDma(uint8_t *pBuffer)
{
    // Program Single DMA base address.
    NV_SDMMC_WRITE(SYSTEM_ADDRESS, (uint32_t)(pBuffer));
}

NvBootError HwSdmmcSetCardClock(NvBootSdmmcCardClock ClockRate)
{
    uint32_t taac;
    uint32_t nsac;
    uint32_t StcReg;
    NvBootError e;
    uint32_t CardClockInMHz;
    uint32_t CardClockDivisor;
    uint32_t TimeOutCounter = 0;
    uint32_t ClockCyclesRequired = 0;
    uint32_t ControllerClockInMHz;
    uint32_t CardCycleTimeInNanoSec;
    uint32_t ControllerClockDivisor;
    uint32_t ContollerCycleTimeInNanoSec;
    void *SdmmcClockTable;


    // These array values are as per Emmc/Esd Spec's.
    const uint32_t TaacTimeUnitArray[] = {1, 10, 100, 1000, 10000, 100000, 1000000,
                                       10000000};
    const uint32_t TaacMultiplierArray[] = {10, 10, 12, 13, 15, 20, 25, 30, 35, 40,
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
        /* If PLLP fallthrough scheme is used by setting FUSE_SKU_DIRECT_CONFIG[1:1],
         * then, assuming osc freq = 38.4MHz, Controller clock is 38.4/2 = 19.2MHz
         * and Controller clock is divided down further to get identification clock
         * of 300KHz.
         */
        ControllerClockDivisor = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, 24);
        CardClockDivisor = 64;
        SdmmcClockTable = (void *)(s_SdmmcClkDiv_24);
    }
    else if (ClockRate == NvBootSdmmcCardClock_DataTransfer)
    {

        // Default setting for  ( SDR_25.5Mhz_ReadMultipage , SDR_25.5Mhz_ReadSinglepage,  DDR_25.5Mhz_ReadMultipage)
        SdmmcClockTable = (void *)(s_SdmmcClkDiv_51);
        ControllerClockDivisor = s_SdmmcContext->ClockDivisor;//8;

        if(NvBootIsPlatformFpga())
        {
            CardClockDivisor = SDMMC_SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL_0_SDCLK_FREQUENCYSELECT_DIV2 << 1;
        }
        else
        {
            CardClockDivisor = s_SdmmcContext->CardClockDivisor; /// 2
        }
        if(s_SdmmcContext->ConfigOption == Sdmmc_Config_3)
        {
            SdmmcClockTable = (void *)(s_SdmmcClkDiv_102);
        }
        /// Update Clk divisor for DevStatus
        s_SdmmcBitInfo->ModuleClkDivisor = ControllerClockDivisor;

    }
    else //if (ClockRate == NvBootSdmmcCardClock_20MHz)
    {
        ControllerClockDivisor = QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, 20);
        CardClockDivisor = SDMMC_IO_CLOCK_DIVISOR_NONE;///1;
        SdmmcClockTable = s_SdmmcClkDiv_20;
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

    // Osc freq with a divisor of 2 should be used here
    // With 38.4MHz, it will give 19.2MHz (guaranteed mode of operation
    // On FPGA, not programming divisor doesn't cause an issue because
    // oscillator frequency is 19.2MHz on FPGA
/*        NvBootClocksConfigureClock(NvBootClocksClockId_SdmmcId,
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(ControllerClockDivisor, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4_0_SDMMC4_CLK_SRC_CLK_M);
*/
    NV_BOOT_CHECK_ERROR(NvBootClocksEngine(SdmmcClockTable, TYPE_SINGLE_TABLE));

    // If the card clock divisor is 64, the register should be written with 32.
    StcReg = NV_FLD_SET_DRF_NUM(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL,
                SDCLK_FREQUENCYSELECT, (CardClockDivisor >> 1), StcReg);
    NV_SDMMC_WRITE(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);
    // Wait till clock is stable.
    NV_BOOT_CHECK_ERROR(HwSdmmcWaitForClkStable());
    // Reload reg value after clock is stable.
    NV_SDMMC_READ(SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, StcReg);

    /// Update Internal Clk divisor for DevStatus
    CardClockDivisor = NV_DRF_VAL(SDMMC, SW_RESET_TIMEOUT_CTRL_CLOCK_CONTROL, SDCLK_FREQUENCYSELECT, StcReg);
    s_SdmmcBitInfo->SdmmcIntrlClkDivisor = CardClockDivisor;

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
    while ( ClockCyclesRequired > (uint32_t)(1 << (13 + TimeOutCounter)) )
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

void HwSdmmcCalculateCardClockDivisor(void)
{
    uint32_t TotalClockDivisor = s_SdmmcContext->ClockDivisor;

//    s_SdmmcContext->CardClockDivisor = 1;
    s_SdmmcContext->HighSpeedMode = NV_FALSE;
    if ( (s_SdmmcContext->HostSupportsHighSpeedMode == NV_FALSE) ||
         (s_SdmmcContext->CardSupportsHighSpeedMode == NV_FALSE) ||
         (s_SdmmcContext->SpecVersion < 4) )
    {
        // Either card or host doesn't support high speed. So reduce the clock
        // frequency if required.
        // fall back option to config Sdmmc_Config_0 SDR_25.5Mhz_ReadMultipage

        // change the config to Sdmmc_Config_0 
        s_SdmmcContext->ConfigOption = Sdmmc_Config_0;// fall back option..
        s_SdmmcContext->ClockDivisor = SDMMC_CNTL_CLOCK_DIVISOR_51MHZ;
        s_SdmmcContext->CardClockDivisor = SDMMC_IO_CLOCK_DIVISOR_BY_HALF;
        /*
        if (QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ,
            s_SdmmcContext->ClockDivisor) >
            s_SdmmcContext->TranSpeedInMHz)
            s_SdmmcContext->ClockDivisor =
            QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ,
            s_SdmmcContext->TranSpeedInMHz);
            */
    }
    else
    {
        /*
        while (QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, TotalClockDivisor) >
                SDMMC_MAX_CLOCK_FREQUENCY_IN_MHZ)
        {
            s_SdmmcContext->CardClockDivisor <<= 1;
            TotalClockDivisor <<= 1;
        }
        */
        if ( QUOTIENT_CEILING(SDMMC_PLL_FREQ_IN_MHZ, TotalClockDivisor) > s_SdmmcContext->TranSpeedInMHz)
        {
            s_SdmmcContext->HighSpeedMode = NV_TRUE;
        }
    }
    PRINT_SDMMC_MESSAGES("\r\nClockDivisor=%d, CardClockDivisor=%d, "
        "HighSpeedMode=%d", s_SdmmcContext->ClockDivisor,
        s_SdmmcContext->CardClockDivisor, s_SdmmcContext->HighSpeedMode);
}

static NvBool SdmmcIsCardInTransferState(void)
{
    NvBootError e;
    uint32_t CardState;
    uint32_t* pResp = &s_SdmmcContext->SdmmcResponse[0];

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

NvBootError
EmmcVerifyResponse(
    SdmmcCommand command,
    NvBool AfterCmdExecution)
{
    uint32_t* pResp = &s_SdmmcContext->SdmmcResponse[0];
    uint32_t AddressOutOfRange = NV_DRF_VAL(SDMMC, CS, ADDRESS_OUT_OF_RANGE, pResp[0]);
    uint32_t AddressMisalign = NV_DRF_VAL(SDMMC, CS, ADDRESS_MISALIGN, pResp[0]);
    uint32_t BlockLengthError = NV_DRF_VAL(SDMMC, CS, BLOCK_LEN_ERROR, pResp[0]);
    uint32_t CommandCrcError = NV_DRF_VAL(SDMMC, CS, COM_CRC_ERROR, pResp[0]);
    // For illegal commands, card does not respond. It can
    // be known only through CMD13.
    uint32_t IllegalCommand = NV_DRF_VAL(SDMMC, CS, ILLEGAL_CMD, pResp[0]);
    uint32_t CardInternalError = NV_DRF_VAL(SDMMC, CS, CC_ERROR, pResp[0]);
    uint32_t CardEccError = NV_DRF_VAL(SDMMC, CS, CARD_ECC_FAILED, pResp[0]);
    uint32_t SwitchError = NV_DRF_VAL(SDMMC, CS, SWITCH_ERROR, pResp[0]);
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

NvBootError EmmcSendSwitchCommand(uint32_t CmdArg)
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

NvBootError EmmcSelectAccessRegion(SdmmcAccessRegion region)
{
    uint32_t CmdArg;
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

NvBootError SdmmcSelectAccessRegion(uint32_t* Block)
{
    NvBootError e = NvBootError_Success;
    SdmmcAccessRegion region;
    uint32_t BlocksPerPartition = s_SdmmcContext->EmmcBootPartitionSize >>
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
    uint8_t* pBuffer = (uint8_t*)&s_SdmmcContext->SdmmcInternalBuffer[0];

    // Set num of blocks to read to 1.
    HwSdmmcSetNumOfBlocks((1 << s_SdmmcContext->PageSizeLog2), 1);
    // Setup Dma.
    HwSdmmcSetupDma((uint8_t*)s_SdmmcContext->SdmmcInternalBuffer);
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
        DevStatus = HwSdmmcQueryStatus();
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

static uint32_t EmmcGetPowerClass(void)
{
    uint32_t PowerClass;

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
    uint32_t CmdArg;
    uint32_t PowerClassToSet;
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
    uint8_t* pBuffer = (uint8_t*)&s_SdmmcContext->SdmmcInternalBuffer[0];

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
    uint32_t CmdArg;
    NvBootError e = NvBootError_Success;

    // Send SWITCH(CMD6) Command to select bus width.
    PRINT_SDMMC_MESSAGES("\r\n\r\nChange Data width to %d(0->1bit, 1->4bit,"
        " 2->8-bit, 5->4-bit DDR, 6->8-bit DDR)", s_SdmmcContext->DataWidth);
    CmdArg = EMMC_SWITCH_BUS_WIDTH_ARG |
             (s_SdmmcContext->DataWidth << EMMC_SWITCH_BUS_WIDTH_OFFSET);
    NV_BOOT_CHECK_ERROR(EmmcSendSwitchCommand(CmdArg));
    HwSdmmcSetDataWidth(s_SdmmcContext->DataWidth, &(s_SdmmcBitInfo->DataWidthUnderUse));
    return e;
}
static NvBootError EmmcEnableDDRSupport(void)
{
    uint32_t HostControl2Reg=0;
    uint32_t StcReg;
    uint32_t CapabilityReg;
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
        if(NvBootIsPlatformFpga())
        {
            if(!s_SdmmcContext->HighSpeedMode)
            {
                s_SdmmcContext->HighSpeedMode = NV_TRUE;
                EmmcEnableHighSpeed();
                s_SdmmcContext->HighSpeedMode = NV_FALSE;
            }
        }

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

NvBootError EmmcGetOpConditions(void)
{
    NvBootError e;
    uint32_t StartTime;
    uint32_t OCRRegister = 0;
    uint32_t ElapsedTime = 0;
    uint32_t* pSdmmcResponse = &s_SdmmcContext->SdmmcResponse[0];
    uint32_t Cmd1Arg = EmmcOcrVoltageRange_QueryVoltage;

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
            if (OCRRegister & EmmcOcrVoltageRange_LowVoltage)
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


NvBootError SdmmcGetCsd(void)
{
    uint32_t Mult;
    uint32_t CSize;
    NvBootError e;
    uint32_t CSizeMulti;
    uint32_t* pResp = &s_SdmmcContext->SdmmcResponse[0];

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
     * 512 byte page size only. Transfer size for all SD/eMMC devices is fixed at 512 bytes.
     * although the logical sector size may differ.
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


NvBootError EmmcIdentifyCard(void)
{
    NvBootError e = NvBootError_Success;
    SdmmcAccessRegion NvBootPartitionEn = SdmmcAccessRegion_UserArea;
    uint8_t CardSpeed = 0;
    unsigned long funcStartTick;
    uint32_t SkipDelay = 0;

    // Set Clock rate to 375KHz for identification of card.
    HwSdmmcSetCardClock(NvBootSdmmcCardClock_Identification);

    NvBootFuseSkipDelaySeq(&SkipDelay);
    if(!SkipDelay)
    {
        NvBootUtilWaitUS(1000); // send clocks for max (1ms)
        NvBootUtilWaitUS(3);    // send clocks for 74 clocks @ 375Khz
    }

    // Send GO_IDLE_STATE(CMD0) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_GoIdleState,
        0, SdmmcResponseType_NoResponse, NV_FALSE));
    // This sends SEND_OP_COND(CMD1) Command and finds out address mode and
    // capacity status.

    // time Init
    funcStartTick = NvBootUtilGetTimeUS();

    NV_BOOT_CHECK_ERROR(EmmcGetOpConditions());

    /// Update Cmd1 response timestamp
    s_SdmmcBitInfo->eMMCCmd1= NvBootUtilGetTimeUS() - funcStartTick;

    // Send ALL_SEND_CID(CMD2) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_AllSendCid,
        0, SdmmcResponseType_R2, NV_FALSE));

    /// Update Send CID response from Cmd1 timestamp
    s_SdmmcBitInfo->eMMC_ALL_SEND_CID = NvBootUtilGetTimeUS() - s_SdmmcBitInfo->eMMCCmd1;

    // Copy card identification data to sdmmc bit info.
    memcpy(s_SdmmcBitInfo->Cid, s_SdmmcContext->SdmmcResponse,
        sizeof(s_SdmmcBitInfo->Cid));

    // Set RCA to Card Here. It should be greater than 1 as JEDSD spec.
    s_SdmmcContext->CardRca = (2 << SDMMC_RCA_OFFSET);
    // Send SET_RELATIVE_ADDR(CMD3) Command.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_EmmcSetRelativeAddress,
        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE));

    // time for CSD.
    funcStartTick = NvBootUtilGetTimeUS();

    // Get Card specific data. We can get it at this stage of identification.
    NV_BOOT_CHECK_ERROR(SdmmcGetCsd());
    /// Update CSD retrieve timestamp
    s_SdmmcBitInfo->eMMC_CSD = NvBootUtilGetTimeUS() - funcStartTick;

    // Send SELECT/DESELECT_CARD(CMD7) Command to place the card in tran state.
    NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SelectDeselectCard,
        s_SdmmcContext->CardRca, SdmmcResponseType_R1, NV_FALSE));
    // Card should be in transfer state now. Confirm it.
    if (SdmmcIsCardInTransferState() == NV_FALSE)
        return NvBootError_DeviceError;

    /// Update transition timestamp from identification to transfer mode.
    s_SdmmcBitInfo->eMMC_transfer_mode = NvBootUtilGetTimeUS() - s_SdmmcBitInfo->eMMC_CSD;

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

        // time for ExtCSD.
        funcStartTick = NvBootUtilGetTimeUS();

        HwSdmmcSetDataWidth(NvBootSdmmcDataWidth_1Bit, &(s_SdmmcBitInfo->DataWidthUnderUse));
        // Set data clock rate to 20MHz.
        HwSdmmcSetCardClock(NvBootSdmmcCardClock_20MHz);
        // It is valid for v4.xx and above cards only.
        // EmmcGetExtCsd() Finds out boot partition size also.
        NV_BOOT_CHECK_ERROR(EmmcGetExtCsd());

        /// Update Ext CSD retrieve timestamp
        s_SdmmcBitInfo->eMMC_ExtCsd = NvBootUtilGetTimeUS() - funcStartTick;
        //
        // Powerclass setting controlled via MaxPowerClassSupported param
        // Select the power class now.
        if(s_SdmmcContext->MaxPowerClassSupported)
        {
            NV_BOOT_CHECK_ERROR(EmmcSetPowerClass());
            /// Update power class setup timestamp from ext csd
            s_SdmmcBitInfo->eMMC_PowerClass = NvBootUtilGetTimeUS() - s_SdmmcBitInfo->eMMC_ExtCsd;
        }
        // Enable the High Speed Mode, if required.
        EmmcEnableHighSpeed();
    }

    // Set the clock for data transfer.
    HwSdmmcSetCardClock(NvBootSdmmcCardClock_DataTransfer);

    // time for buswidth/partition.
    funcStartTick = NvBootUtilGetTimeUS();

    // v4.4 and ddr supported enable ddr
    if ((s_SdmmcContext->ConfigOption == Sdmmc_Config_3) ||
        (s_SdmmcContext->ConfigOption == Sdmmc_Config_4))
    {
        // v4.4 and ddr supported enable ddr
        CardSpeed = (s_SdmmcContext->CardSupportSpeed &
                                EMMC_ECSD_CT_HS_DDR_MASK);
        if((CardSpeed & EMMC_ECSD_CT_HS_DDR_52_120) ||
            (CardSpeed & EMMC_ECSD_CT_HS_DDR_52_180_300))
        {
            NV_BOOT_CHECK_ERROR(EmmcEnableDDRSupport());

            /// Update Data transfer modefor DevStatus
            s_SdmmcBitInfo->SdmmcDataMode = SDMMC_AUTO_CMD12_ERR_STATUS_0_UHS_MODE_SEL_DDR50;
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
                                    &  EMMC_ECSD_BC_BPE_MASK);

    if (s_SdmmcContext->EmmcBootPartitionSize != 0 &&
        ((NvBootPartitionEn == EMMC_ECSD_BC_BPE_BAP1) ||
        (NvBootPartitionEn == EMMC_ECSD_BC_BPE_BAP2)))
        NV_BOOT_CHECK_ERROR(EmmcSelectAccessRegion(NvBootPartitionEn));

    /// Update Buswidth setup time .
    s_SdmmcBitInfo->eMMC_WidthPartitionSetup = NvBootUtilGetTimeUS() - funcStartTick;

    s_SdmmcBitInfo->BootFromBootPartition =
                                  s_SdmmcContext->EmmcBootPartitionSize ? 1 : 0;
    return e;
}


NvBootDeviceStatus HwSdmmcQueryStatus(void)
{
    NvBootError e;
    uint32_t SdmaAddress;
    uint32_t TransferDone = 0;
    uint32_t InterruptStatusReg;
    uint32_t ErrorMask =
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
    uint32_t DmaBoundaryInterrupt = NV_DRF_DEF(SDMMC, INTERRUPT_STATUS,
                                    DMA_INTERRUPT,GEN_INT);
    uint32_t DataTimeOutError =
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

void HwSdmmcShutdown(void)
{
    uint32_t StcReg;
    uint32_t PowerControlHostReg;

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

