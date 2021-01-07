/*
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_usbf_hw.h"
#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "arusb.h"
#include "arfuse.h"
#include "arclk_rst.h"
#include "arapbpm.h"
#include "nvboot_clocks_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_usbf_int.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_irom_patch_int.h"

//---------------------Global Variables Decleration----------------------------


/**
 * Usbf context record and its pointer used locally by the driver
 * Note that the pointer is NULL until NvBootUsbfInit has been called.
 * The usbf context record used by usbf driver is being referenced here.
 */
NvBootUsbfContext s_UsbfContext;
NvBootUsbfContext *s_pUsbfCtxt = NULL;

//---------------------Private Variables Decleration---------------------------


// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
///////////////////////////////////////////////////////////////////////////////
//  PLLU configuration information (reference clock is osc/clk_m and PLLU-FOs are fixed at
//  12MHz/60MHz/480MHz).
//
//  reference frequency
//               13.0MHz       19.2MHz      12.0MHz      26.0MHz      16.8MHz....  38.4MHz  48MHz
//  -----------------------------------------------------------------------------------------------
// DIVN      37 (025h)     25 (019h)      40 (028h)    37 (025h)    028 (01Ch)   25 (019h)  20(014h)
// DIVM      1  ( 01h)      1 ( 01h)      1 ( 01h)      2 ( 02h)      1 (01h)      2 (02h)   2 (02h)
// http://nvbugs/1404142 
// PLLU does not have divided reference branches anymore
// Also Post Divider is 5 bits now.
// CLKIN/M must be min 5M and max 20M.
static const UsbPllClockParams s_UsbPllBaseInfo[NvBootClocksOscFreq_MaxVal] = 
{
    //DivN, DivM, DivP
    {0x025, 0x01, 0x1}, // For NvBootClocksOscFreq_13,
    {0x01C,  0x1, 0x1}, // For NvBootClocksOscFreq_16_8
    {    0,    0,   0}, // dummy field
    {    0,    0,   0}, // dummy field
    {0x019, 0x01, 0x1}, // For NvBootClocksOscFreq_19_2
    {0x019, 0x02, 0x1}, // For NvBootClocksOscFreq_38_4,
    {    0,    0,   0}, // dummy field
    {    0,    0,   0}, // dummy field
    {0x028, 0x01, 0x1}, // For NvBootClocksOscFreq_12
    {0x028, 0x04, 0x1}, // For NvBootClocksOscFreq_48,
    {    0,    0,   0}, // dummy field
    {    0,    0,   0}, // dummy field
    {0x025, 0x02, 0x1}  // NvBootClocksOscFreq_26
};

// UTMI PLL needs output = 960 Mhz with osc input
// Bug 1398118
// Note: CLKIN/M ratio should between 12 and 38.4 Mhz
static const UtmiPllClockParams s_UtmiPllBaseInfo[NvBootClocksOscFreq_MaxVal] = 
{
    //DivN, DivM
    {0x04A, 0x01}, // For NvBootClocksOscFreq_13, // Not P0.
    {0x039,  0x1}, // For NvBootClocksOscFreq_16_8 // Not P0.
    {    0,    0}, // dummy field
    {    0,    0}, // dummy field
    {0x032, 0x01}, // For NvBootClocksOscFreq_19_2 // Not P0.
    {0x019, 0x01}, // For NvBootClocksOscFreq_38_4,
    {    0,    0}, // dummy field
    {    0,    0}, // dummy field
    {0x050, 0x01}, // For NvBootClocksOscFreq_12
    {0x028, 0x02}, // For NvBootClocksOscFreq_48, // Not P0.
    {    0,    0}, // dummy field
    {    0,    0}, // dummy field
    {0x04A, 0x02}  // NvBootClocksOscFreq_26 // Not P0.
};

// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
///////////////////////////////////////////////////////////////////////////////
// PLL CONFIGURATION & PARAMETERS: refer to the arapb_misc_utmip.spec file.
///////////////////////////////////////////////////////////////////////////////
// PLL CONFIGURATION & PARAMETERS for different clock generators:
//-----------------------------------------------------------------------------
// Reference frequency            13.0MHz       19.2MHz       12.0MHz      26.0MHz 
// ----------------------------------------------------------------------------
// PLLU_ENABLE_DLY_COUNT   02 (02h)       03 (03h)       02 (02h)     04 (04h)
// PLLU_STABLE_COUNT          51 (33h)       75 (4Bh)       47 (2Fh)     102 (66h)
// PLL_ACTIVE_DLY_COUNT     09 (09h)       12 (0C)        08 (08h)     17 (11h)
// XTAL_FREQ_COUNT             118 (76h)     188 (BCh)     118 (76h)   254 (FEh)
//-----------------------------------------------------------------------------
// Reference frequency            16.8MHz        38.4MHz         48MHz 
// ----------------------------------------------------------------------------
// PLLU_ENABLE_DLY_COUNT   03 (03h)        05 (05h)         06 (06h)
// PLLU_STABLE_COUNT          66 (42h)        150 (96h)       188 (BCh)
// PLL_ACTIVE_DLY_COUNT     11 (0Bh)       24 (18h)         30 (1Eh)
// XTAL_FREQ_COUNT             165 (A5h)      375 (177h)     469 (1D5h)
///////////////////////////////////////////////////////////////////////////////
static const UsbPllDelayParams s_UsbPllDelayParams[NvBootClocksOscFreq_MaxVal] =
{
    //ENABLE_DLY,  STABLE_CNT,  ACTIVE_DLY,  XTAL_FREQ_CNT
    {0x02,         0x33,        0x09,       0x7F}, // For NvBootClocksOscFreq_13,
    {0x03,         0x42,        0x0B,       0xA5}, // For NvBootClocksOscFreq_16_8
    {     0,              0,             0,             0}, // dummy field
    {     0,              0,             0,             0}, // dummy field
    {0x03,         0x4B,       0x0C,        0xBC}, // For NvBootClocksOscFreq_19_2
    {0x05,         0x96,       0x18,      0x177},  //For NvBootClocksOscFreq_38_4
    {     0,              0,             0,             0}, // dummy field
    {     0,             0,              0,             0}, // dummy field
    {0x02,         0x2F,       0x08,        0x76}, // For NvBootClocksOscFreq_12
    {0x06,         0xBC,       0X1F,      0x1D5}, // For NvBootClocksOscFreq_48
    {     0,              0,            0,              0}, // dummy field
    {     0,              0,            0,              0}, // dummy field
    {0x04,         0x66,        0x11,       0xFE}  // For NvBootClocksOscFreq_26
};

// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
///////////////////////////////////////////////////////////////////////////////
// Debounce values IdDig, Avalid, Bvalid, VbusValid, VbusWakeUp, and SessEnd. 
// Each of these signals have their own debouncer and for each of those one out
// of 2 debouncing times can be chosen (BIAS_DEBOUNCE_A or BIAS_DEBOUNCE_B.)
//
// The values of DEBOUNCE_A and DEBOUNCE_B are calculated as follows:
// 0xffff -> No debouncing at all
// <n> ms = <n> *1000 / (1/19.2MHz) / 4
// So to program a 1 ms debounce for BIAS_DEBOUNCE_A, we have:
// BIAS_DEBOUNCE_A[15:0] = 1000 * 19.2 / 4  = 4800 = 0x12c0
// We need to use only DebounceA for BOOTROM. We don’t need the DebounceB 
// values, so we can keep those to default.
///////////////////////////////////////////////////////////////////////////////
static const NvU32 s_UsbBiasDebounceATime[NvBootClocksOscFreq_MaxVal] =
{
    /* Ten milli second delay for BIAS_DEBOUNCE_A */
    0x7EF4,  // For NvBootClocksOscFreq_13,
    0XA410,  //  For NvBootClocksOscFreq_16_8
             0,  // dummy field
             0,  // dummy field
    0xBB80,  // For NvBootClocksOscFreq_19_2
    0xBB80,  //For NvBootClocksOscFreq_38_4,
             0,  // dummy field
             0,  // dummy field
    0x7530,  // For NvBootClocksOscFreq_12
    0xEA60,  // For NvBootClocksOscFreq_48,
             0,  // dummy field
             0,  // dummy field
    0xFDE8   // For NvBootClocksOscFreq_26
};

////////////////////////////////////////////////////////////////////////////////
// The following arapb_misc_utmip.spec fields need to be programmed to ensure 
// correct operation of the UTMIP block:
// Production settings : 
//        'HS_SYNC_START_DLY' : 9,
//        'IDLE_WAIT'         : 17,
//        'ELASTIC_LIMIT'     : 16,
// All other fields can use the default reset values.
// Setting the fields above, together with default values of the other fields, 
// results in programming the registers below as follows:
//         UTMIP_HSRX_CFG0 = 0x9168c000
//         UTMIP_HSRX_CFG1 = 0x13
////////////////////////////////////////////////////////////////////////////////
//UTMIP Idle Wait Delay
static const NvU8 s_UtmipIdleWaitDelay    = 17;
//UTMIP Elastic limit
static const NvU8 s_UtmipElasticLimit     = 16;
//UTMIP High Speed Sync Start Delay
static const NvU8 s_UtmipHsSyncStartDelay = 9;

// Divisor for USB HSIC TRK clock
// This is supposed to be static but made global for xusb_dev sake.
const NvU32 s_UsbHsicTrkDiv[NvBootClocksOscFreq_MaxVal] =
{
    // The frequency for the tracking circuit should be between 1 to 10 MHz. 
    // osc_clk frequency is between 10 to 20 MHz, the clock divisor should be set to 0x2; 
    // osc_clk frequency is between 20 to 30 MHz, the clock divisor should be set to 0x4; 
    // osc_clk frequency is between 30 to 40 MHz, the clock divisor should be set to 0x6;
    2,  // For NvBootClocksOscFreq_13,
    2,  //  For NvBootClocksOscFreq_16_8
    0,  // dummy field
    0,  // dummy field
    2,  // For NvBootClocksOscFreq_19_2
    6,  //For NvBootClocksOscFreq_38_4,
    0,  // dummy field
    0,  // dummy field
    2,  // For NvBootClocksOscFreq_12
    6,  // For NvBootClocksOscFreq_48,
    0,  // dummy field
    0,  // dummy field
    4   // For NvBootClocksOscFreq_26
};

//---------------------Private Functions Decleration---------------------------

/**
 * Intializes the USB controller H/W by enabling the clocks and 
 * configuring the TDs and Queue Heads required for the USB.
 */
static NvBootError 
NvBootUsbfHwIntializeController(
    NvBootUsbfContext *pUsbFuncCtxt);
/*
 * Initialize the internal UsbfContext and its pointer.
 */
static NvBootUsbfInitContext(void);


//---------------------Public Functions Definitions----------------------------

static void NvBootUsbfResetController(NvU32 UsbBase)
{
    NvU32 RegVal = 0;

    // Enable clock to the USB controller
    NvBootClocksSetEnable(NvBootClocksClockId_UsbId, NV_TRUE);
    // Reset the USB controller
    NvBootResetSetEnable(NvBootResetDeviceId_UsbId, NV_TRUE);
    NvBootResetSetEnable(NvBootResetDeviceId_UsbId, NV_FALSE);

    // deassert reset to XUSB PAD Control block.
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbId, NV_FALSE);

    /*
     * Assert UTMIP_RESET in USB1_IF_USB_SUSP_CTRL register to put
     * UTMIP1 in reset.
     */
    USBIF_REG_UPDATE_DEF(UsbBase, USB_SUSP_CTRL, UTMIP_RESET, ENABLE);

    /*
    * Set USB1 to use UTMIP PHY by setting
    * USB1_IF_USB_SUSP_CTRL.UTMIP_PHY_ENB  register to 1.
    */
    RegVal = NV_READ32(UsbBase + USB1_IF_USB_SUSP_CTRL_0);

    RegVal = NV_FLD_SET_DRF_DEF(USB1_IF, USB_SUSP_CTRL, UTMIP_PHY_ENB, ENABLE, RegVal);

    NV_WRITE32(UsbBase + USB1_IF_USB_SUSP_CTRL_0, RegVal);

}

static void NvbootUsbfPllManualPgramming(void)
{
    NvU32 RegVal = 0;

    //wait for pllu enable period (1us)
    NvBootUtilWaitUS(1);

    // Set PLL enable delay count and Crystal frequency count ( clock reset domain)
    RegVal = UTMIP_PLL_RD(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1);
    // Remove power down for PLLU
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_FORCE_PLLU_POWERDOWN,
                        0x0,
                        RegVal);
    // Enable power up for PLLU
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_FORCE_PLLU_POWERUP,
                        0x1,
                        RegVal);

    UTMIP_PLL_WR(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1, RegVal);

    //wait for pllustable period (1ms)
    NvBootUtilWaitUS(1000);

    // Remove power down for PLL_ENABLE
    RegVal = UTMIP_PLL_RD(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_FORCE_PLL_ENABLE_POWERDOWN,
                        0x0,
                        RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_FORCE_PLL_ENABLE_POWERUP,
                        0x1,
                        RegVal);

    UTMIP_PLL_WR(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1, RegVal);

    //wait for pllactive delay count (10us)
    NvBootUtilWaitUS(10);

    // Remove power down for PLL_ACTIVE
    RegVal = UTMIP_PLL_RD(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1,
                        UTMIP_FORCE_PLL_ACTIVE_POWERDOWN,
                        0x0,
                        RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1,
                        UTMIP_FORCE_PLL_ACTIVE_POWERUP,
                        0x1,
                        RegVal);

    UTMIP_PLL_WR(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1, RegVal);
}

NvBootError NvBootUsbfInit(void)
{
    NvU32 RegVal = 0, PlluInputForUtmi = 0, LockTime;
    NvU32 HSTermRangeAdj, Setup, RpdCtrl;
    NvU32 PlluStableTime = 0, Misc1, Misc2;
    // Configure the USB phy stabilization delay setting.
    NvBootClocksOscFreq OscFreq;
    NvU32 UsbBase = NV_ADDRESS_MAP_USB_BASE;
    NvBootError BootError = NvBootError_Success;

    // Get the Oscillator frequency
    OscFreq = NvBootClocksGetOscFreq();

    // LFCON, CPCON removed for PLLU
    Misc1 = 0;
    Misc2 = 0;

    // Enable PLL U for USB
    NvBootClocksStartPll(NvBootClocksPllId_PllU,
                         s_UsbPllBaseInfo[OscFreq].M,
                         s_UsbPllBaseInfo[OscFreq].N,
                         s_UsbPllBaseInfo[OscFreq].P,
                         Misc1,
                         Misc2,
                         &PlluStableTime);


    // Initialize USB1 controller.

    // Reset the usb controller.
    NvBootUsbfResetController(UsbBase);

    // UTMIPLL_IDDQ_OVERRIDE_VALUE=0 and UTMIPLL_IDDQ_SWCTL=1 should be 
    // programmed to power up the UTMIPLL and they should not be touched after that.
    // Write UTMIPLL_SEQ_RESET_INPUT_VALUE=1, UTMIPLL_SEQ_IN_SWCTL=1
    // and UTMIPLL_SEQ_START_STATE=0.
    // Write UTMIPLL_SEQ_ENABLE=1.
    RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    UTMIPLL_HW_PWRDN_CFG0,
                    UTMIPLL_IDDQ_OVERRIDE_VALUE,
                    0x0, RegVal);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0, RegVal);

    // Wait 10us as per UTMIP spec.
    NvBootUtilWaitUS(10);

    // Stop crystal clock by setting UTMIP_PHY_XTAL_CLOCKEN low
    UTMIP_REG_UPDATE_DEF(UsbBase,
                         MISC_CFG1,
                         UTMIP_PHY_XTAL_CLOCKEN,
                         SW_DEFAULT);

    UTMIP_PLL_UPDATE_DEF(NV_ADDRESS_MAP_CLK_RST_BASE,
                         PLL_CFG2,
                         UTMIP_PHY_XTAL_CLOCKEN,
                         SW_DEFAULT);

    // Follow the crystal clock disable by >100ns delay.
    NVBOOT_UTIL_WAIT_100NS_OSC();
    /*
     * B_SESS_VLD_SW_VALUE, B_SESS_VLD_SW_EN bits of 
     * USB1_IF_USB_PHY_VBUS_SENSORS_0 register need to be set to enable 
     * SW overrides on VBUS sensors.
     */
    USBIF_REG_UPDATE_DEF(UsbBase,
                         USB_PHY_VBUS_SENSORS,
                         B_SESS_VLD_SW_VALUE, 
                         SET);
    USBIF_REG_UPDATE_DEF(UsbBase,
                         USB_PHY_VBUS_SENSORS,
                         B_SESS_VLD_SW_EN,
                         ENABLE);

    // Set clock source for UTMIP. Default is osc_clk.
    // Based on CYA, choose PLLU as input.
    PlluInputForUtmi = NvBootGetSwCYA() & NVBOOT_SW_CYA_USE_PLLU_SRC_UTMIP;
    if (PlluInputForUtmi)
    {
        RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG3_0);
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                        PLL_CFG3, 
                        UTMIP_PLL_REF_SRC_SEL,
                        1, // PLLU
                        RegVal);
        NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG3_0, RegVal);
        // Configure UTMI PLL dividers.
        // PLLU 480 Mhz VCO output is src for UTMI.
        // Set N=2, M=1 so that UTMI PLL VCO is 960 Mhz.
        RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_NDIV,
                            30,
                            RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_MDIV,
                            15,
                            RegVal);
        NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0, RegVal);
    }
    else
    {
        // Configure UTMI PLL dividers based on oscillator frequency.
        RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_NDIV,
                            s_UtmiPllBaseInfo[OscFreq].N,
                            RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_MDIV,
                            s_UtmiPllBaseInfo[OscFreq].M,
                            RegVal);
        NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0, RegVal);
    }
    // PLL Delay CONFIGURATION settings
    // The following parameters control the bring up of the plls:
    RegVal = UTMIP_PLL_RD(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG2);
    if (PlluInputForUtmi)
    {
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, 
                        PLL_CFG2, 
                        UTMIP_PLLU_STABLE_COUNT,
                        s_UsbPllDelayParams[OscFreq].StableCount,
                        RegVal);
    }
    else
    {
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, 
                        PLL_CFG2, 
                        UTMIP_PLLU_STABLE_COUNT,
                        0, // Need not wait for PLLU to be stable if using Osc_Clk as input
                        RegVal);
    }
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                        UTMIP_PLL_ACTIVE_DLY_COUNT,
                        s_UsbPllDelayParams[OscFreq].ActiveDelayCount,
                        RegVal);

    UTMIP_PLL_WR(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG2, RegVal);

    // Set PLL enable delay count and Crystal frequency count ( clock reset domain)
    RegVal = UTMIP_PLL_RD(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1);
    if(PlluInputForUtmi)
    {
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_PLLU_ENABLE_DLY_COUNT,
                        s_UsbPllDelayParams[OscFreq].EnableDelayCount,
                        RegVal);
        // Remove power down for PLLU
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_FORCE_PLLU_POWERDOWN,
                        0x0,
                        RegVal);
    }
    else
    {
        RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_PLLU_ENABLE_DLY_COUNT,
                        0, // Need not wait for PLLU if input is OSC_CLK
                        RegVal);
    }
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                    UTMIP_XTAL_FREQ_COUNT,
                    s_UsbPllDelayParams[OscFreq].XtalFreqCount,
                    RegVal);
    // power-up for UTMIP PLL ENABLE
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                    UTMIP_FORCE_PLL_ENABLE_POWERUP,
                    0x1,
                    RegVal);

    // Remove power down for UTMIP PLL ENABLE
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                    UTMIP_FORCE_PLL_ENABLE_POWERDOWN,
                    0x0,
                    RegVal);
    // Remove power down for PLL_ACTIVE
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                    UTMIP_FORCE_PLL_ACTIVE_POWERDOWN,
                    0x0,
                    RegVal);


    UTMIP_PLL_WR(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG1, RegVal);

    // TODO: PLL_CFG1 & 2  regs are in clk base,need to move to clk driver??.

    // Add manual PLL programming routine as a part of IROM patch
    if (NvBootGetSwCYA() & NVBOOT_SW_CYA_USBF_PLL_MANUAL_ENABLE)
    {
        NvbootUsbfPllManualPgramming();
    }

    LockTime = 10; // 10 us
    while(LockTime)
    {
        RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);
        if(NV_DRF_VAL(CLK_RST_CONTROLLER,
                      UTMIPLL_HW_PWRDN_CFG0,
                      UTMIPLL_LOCK,
                      RegVal))
        break;
        NvBootUtilWaitUS(1);
        LockTime--;
    }

    //Static params come from FUSE_USB_CALIB register.
    RegVal = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_0);

    HSTermRangeAdj = NV_DRF_VAL(FUSE, USB_CALIB, TERM_RANGE_ADJ, RegVal); //HS_TERM_RANGE_ADJ
    Setup = NV_DRF_VAL(FUSE, USB_CALIB, SETUP, RegVal); //SETUP

    RegVal = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_EXT_0);
    RpdCtrl = NV_DRF_VAL(FUSE, USB_CALIB_EXT, RPD_CTRL, RegVal); //RPD_CTRL

    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG0);
    // Set Static Param SETUP
	RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
					XCVR_CFG0,
					UTMIP_XCVR_SETUP,
					Setup & 0xf, // Program 4 bits LSB here.
					RegVal);
	RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
					XCVR_CFG0,
					UTMIP_XCVR_SETUP_MSB,
					(Setup >> 4) & 0x7, // Program remaining 3 bits MSB here.
					RegVal);
    UTMIP_REG_WR(UsbBase, XCVR_CFG0, RegVal);

    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG1);

    // Set Static Param TERM_RANGE_ADJ
	RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
					XCVR_CFG1,
					UTMIP_XCVR_TERM_RANGE_ADJ,
					HSTermRangeAdj,
					RegVal);
    UTMIP_REG_WR(UsbBase, XCVR_CFG1, RegVal);

    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG3);

    // Set Static Param RPD_CTRL
	RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
					XCVR_CFG3,
					UTMIP_XCVR_RPD_CTRL,
					RpdCtrl,
					RegVal);
    UTMIP_REG_WR(UsbBase, XCVR_CFG3, RegVal);

    // Set UTMIP_XCVR_LSBIAS_SEL to 0
    // Read XCVR_CFG0 reg
    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG0);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    XCVR_CFG0,
                    UTMIP_XCVR_LSBIAS_SEL,
                    0x0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, XCVR_CFG0, RegVal);

    // Bug 1545608. Set HS_SQUELCH_LEVEL_NEW = 2 based on characterization.
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG2);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG2,
                    UTMIP_HSSQUELCH_LEVEL_NEW,
                    0x2,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG2, RegVal);

    // In 20 nm soc, pad voltage is 1.5V so enable pad protection circuit against
    // 3.3V. Bug 1500052
    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG2);

	RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
					XCVR_CFG2,
					UTMIP_XCVR_VREG_FIX18,
					0, // 0 = Internal voltage regulator control,to generate some divided voltage
					RegVal);
	RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
					XCVR_CFG2,
					UTMIP_XCVR_VREG_LEV,
					1, // 1 = Generate 1.5 V
					RegVal);

    UTMIP_REG_WR(UsbBase, XCVR_CFG2, RegVal);

    NvBootUtilWaitUS(1);

    // Program 1ms Debounce time for VBUS to become valid.
    UTMIP_REG_UPDATE_NUM(UsbBase, DEBOUNCE_CFG0,
                UTMIP_BIAS_DEBOUNCE_A, 
                s_UsbBiasDebounceATime[OscFreq]);

    /* Actual value required for Bias Debounce A for clock 38.4 is 0x17700 
    and 48 Mhz is 0x1D4C0. But we have 16 bits for this, as per above calculations we need a 17th bit.
    For this, we need to use below config and set the timescale on bias debounce.
    */
    if ((OscFreq == NvBootClocksOscFreq_38_4) || (OscFreq == NvBootClocksOscFreq_48))
    {
        // Read BIAS cfg1 reg
        RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
        // Remove power down on PDTRK
        RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                        BIAS_CFG1,
                        UTMIP_BIAS_DEBOUNCE_TIMESCALE,
                        1,
                        RegVal);
        UTMIP_REG_WR(UsbBase, BIAS_CFG1, RegVal);
    }

    // Clear bit 3, 4, 8 of UTMIP_SPARE_CFG0 so that config register values
    // are used for Setup, TermRangeAdj and RpdCtrl.
    // These values are manually parsed from fuses.
    // Bug 1545608. Also clear bit 7 to apply HS_SQUELCH_LEVEL_NEW=0x2.
    RegVal = UTMIP_REG_RD(UsbBase, SPARE_CFG0);
    RegVal &= ~(0x1 << 3 | 0x1 << 4 | 0x1 << 7 | 0x1 << 8);
    UTMIP_REG_WR(UsbBase, SPARE_CFG0, RegVal);

    // Set UTMIP_FS_PREAMBLE_J to 1
    UTMIP_REG_UPDATE_NUM(UsbBase, TX_CFG0, UTMIP_FS_PREAMBLE_J, 0x1);

    /* Configure the UTMIP_IDLE_WAIT and UTMIP_ELASTIC_LIMIT
     * Setting these fields, together with default values of the other
     * fields, results in programming the registers below as follows:
     *         UTMIP_HSRX_CFG0 = 0x9168c000
     *         UTMIP_HSRX_CFG1 = 0x13
     */

    // Set PLL enable delay count and Crystal frequency count
    RegVal = UTMIP_REG_RD(UsbBase, HSRX_CFG0);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    HSRX_CFG0, 
                    UTMIP_IDLE_WAIT,
                    s_UtmipIdleWaitDelay, 
                    RegVal);

    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    HSRX_CFG0, 
                    UTMIP_ELASTIC_LIMIT,
                    s_UtmipElasticLimit, 
                    RegVal);

    UTMIP_REG_WR(UsbBase, HSRX_CFG0, RegVal);

    // Configure the UTMIP_HS_SYNC_START_DLY
    UTMIP_REG_UPDATE_NUM(UsbBase, HSRX_CFG1, 
                UTMIP_HS_SYNC_START_DLY, 
                s_UtmipHsSyncStartDelay);

    // Proceed  the crystal clock enable  by >100ns delay.
    NVBOOT_UTIL_WAIT_100NS_OSC();

    // Resuscitate  crystal clock by setting UTMIP_PHY_XTAL_CLOCKEN 
    UTMIP_REG_UPDATE_DEF(UsbBase,
                         MISC_CFG1,
                         UTMIP_PHY_XTAL_CLOCKEN,
                         DEFAULT);
    UTMIP_PLL_UPDATE_DEF(NV_ADDRESS_MAP_CLK_RST_BASE,
                         PLL_CFG2,
                         UTMIP_PHY_XTAL_CLOCKEN,
                         DEFAULT);

    /* Perform tracking Bug 1440206 */
    // Enable clock to tracking unit
    RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
					CLK_OUT_ENB_Y,
					CLK_ENB_USB2_TRK,
					0x1,
					RegVal);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0, RegVal);

    // The frequency for the tracking circuit should be between 1 to 10 MHz. 
    // osc_clk frequency is between 10 to 20 MHz, the clock divisor should be set to 0x2; 
    // osc_clk frequency is between 20 to 30 MHz, the clock divisor should be set to 0x4; 
    // osc_clk frequency is between 30 to 40 MHz, the clock divisor should be set to 0x6.

    RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_USB2_HSIC_TRK_0);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
					CLK_SOURCE_USB2_HSIC_TRK,
					USB2_HSIC_TRK_CLK_DIVISOR,
					s_UsbHsicTrkDiv[OscFreq],
					RegVal);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_USB2_HSIC_TRK_0, RegVal); 


    // Read BIAS cfg1 reg
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
    // Note: These timings are not dependent on oscillator frequency any more
    // as we adjust tracking clock div to fixed freq based on osc input.

    // Setting TRK_DONE to PD_TRK assertion/TRK_START de-assertion time.
    // 10 TRK clock cycles
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_BIAS_PDTRK_COUNT,
                    0xA,
                    RegVal);

    // Setting PD_TRK de-assertion to TRK_START. 30 TRK clock cycles
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_BIAS_TRK_START_COUNT,
                    0x1E,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG1, RegVal);

    // Read BIAS cfg0 reg
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG0);
    // Remove power down on BIAS PD
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG0,
                    UTMIP_BIASPD,
                    0x0,
                    RegVal);

    UTMIP_REG_WR(UsbBase, BIAS_CFG0, RegVal);

    NvBootUtilWaitUS(1);

    // TRK_START will be asserted after TRK_START_COUNT once we remove PDTRK
    // Read BIAS cfg1 reg
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
    // Remove power down on PDTRK
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_FORCE_PDTRK_POWERDOWN,
                    0,
                    RegVal);
    // Power up PD TRK
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_FORCE_PDTRK_POWERUP,
                    1,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG1, RegVal);

    // Poll for TRK_DONE to assert. Break if not set by 1ms.
    // It was decided during review meeting to just wait for 100us
    // as it is possible to miss TRK_DONE assertion
    // PollTrkDoneTime = 1000;
    // while(PollTrkDoneTime)
    // {
        // RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
        // if(NV_DRF_VAL(USB1_UTMIP,
                      // BIAS_CFG1,
                      // UTMIP_BIAS_TRK_DONE,
                      // RegVal))
            // break;
        // NvBootUtilWaitUS(1);
        // PollTrkDoneTime--;
    // }

    // Wait for 100 us for tracking to be done
    NvBootUtilWaitUS(100);

    // Power down PDTRK and clear TRK_DONE
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_FORCE_PDTRK_POWERDOWN,
                    1,
                    RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_BIAS_TRK_DONE,
                    0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG1, RegVal);

    // Giving some time for powerdown.(30 trk clk cycles @0.1us)
    NvBootUtilWaitUS(3);

    // Bug 200006851. Do another round of tracking.

    // Power up PDTRK
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_FORCE_PDTRK_POWERDOWN,
                    0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG1, RegVal);

    // Wait for 100 us for tracking to be done
    NvBootUtilWaitUS(100);

    // Power down PDTRK and clear TRK_DONE
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_FORCE_PDTRK_POWERDOWN,
                    1,
                    RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG1,
                    UTMIP_BIAS_TRK_DONE,
                    0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG1, RegVal);
    // PDTRK should be asserted by HW State machine after PDTRK_COUNT (10 TRK cycles)
    // 1 TRK cycle is approximately 0.1us. But we do it anyway

    // Disable clock to tracking unit
    RegVal = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
					CLK_OUT_ENB_Y,
					CLK_ENB_USB2_TRK,
					0x0,
					RegVal);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0, RegVal);

    // Remove various power downs one by one

    // Set PLL enable delay count and Crystal frequency count ( clock reset domain)
    RegVal = UTMIP_PLL_RD(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG2);
    // Remove power down for PLLU_ENABLE
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_A_POWERDOWN,
                    0x0,
                    RegVal);

    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_A_POWERUP,
                    0x1,
                    RegVal);

    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_C_POWERDOWN,
                    0x0,
                    RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_C_POWERUP,
                    0x1,
                    RegVal);

    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_B_POWERUP,
                    0x1,
                    RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_B_POWERDOWN,
                    0x0,
                    RegVal);

    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_D_POWERUP,
                    0x1,
                    RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_D_POWERDOWN,
                    0x0,
                    RegVal);

    UTMIP_PLL_WR(NV_ADDRESS_MAP_CLK_RST_BASE, PLL_CFG2, RegVal);

    NvBootUtilWaitUS(1);

    // Read BIAS cfg0 reg and remove OTG power down
    RegVal = UTMIP_REG_RD(UsbBase, BIAS_CFG0);
    // Remove power down on OTGPD
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG0,
                    UTMIP_OTGPD,
                    0x0,
                    RegVal);
    // Set 0 to UTMIP_IDPD_SEL
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP,
                    BIAS_CFG0,
                    UTMIP_IDPD_SEL,
                    0x0,
                    RegVal);
    // Set 0 to UTMIP_IDPD_VAL
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    BIAS_CFG0,
                    UTMIP_IDPD_VAL,
                    0x0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, BIAS_CFG0, RegVal);

    NvBootUtilWaitUS(1);

    // Remove Power Down for VBUS detectors from PMC 
    // registers.
    RegVal = NV_READ32(NV_ADDRESS_MAP_APB_PMC_BASE + APBDEV_PMC_USB_AO_0);

    // Remove power down on VBUS WAKEUP PD
    RegVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC, 
                    USB_AO,
                    VBUS_WAKEUP_PD_P0,
                    0x0,
                    RegVal);

    // Remove Power Down ID Wake up for UTMIP P0 to overcome the pad modelling issue
    RegVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC, 
                    USB_AO,
                    ID_PD_P0,
                    0x0,
                    RegVal);

    NV_WRITE32(NV_ADDRESS_MAP_APB_PMC_BASE + APBDEV_PMC_USB_AO_0, RegVal);

    NvBootUtilWaitUS(1);

    // Remove power down on PD
    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG0);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    XCVR_CFG0,
                    UTMIP_FORCE_PD_POWERDOWN,
                    0x0,
                    RegVal);

    UTMIP_REG_WR(UsbBase, XCVR_CFG0, RegVal);

    NvBootUtilWaitUS(1);

    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG0);

    // Remove power down on PD2
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    XCVR_CFG0,
                    UTMIP_FORCE_PD2_POWERDOWN,
                    0x0,
                    RegVal);

    UTMIP_REG_WR(UsbBase, XCVR_CFG0, RegVal);

    NvBootUtilWaitUS(1);

    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG0);

    // Remove power down on PDZI
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    XCVR_CFG0,
                    UTMIP_FORCE_PDZI_POWERDOWN,
                    0x0,
                    RegVal);

    UTMIP_REG_WR(UsbBase, XCVR_CFG0, RegVal);

    NvBootUtilWaitUS(1);

    // Remove power down on PDCHRP

    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    XCVR_CFG1,
                    UTMIP_FORCE_PDCHRP_POWERDOWN,
                    0x0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, XCVR_CFG1, RegVal);

    NvBootUtilWaitUS(1);

    // Remove power down on PDDR
    RegVal = UTMIP_REG_RD(UsbBase, XCVR_CFG1);
    RegVal = NV_FLD_SET_DRF_NUM(USB1_UTMIP, 
                    XCVR_CFG1,
                    UTMIP_FORCE_PDDR_POWERDOWN,
                    0x0,
                    RegVal);
    UTMIP_REG_WR(UsbBase, XCVR_CFG1, RegVal);

    NvBootUtilWaitUS(1);
    if (!s_pUsbfCtxt)
    {
        NvBootUsbfInitContext();
    }
    // Initialize the USB controller
    BootError = NvBootUsbfHwIntializeController(s_pUsbfCtxt);

    s_pUsbfCtxt->EnumerationDone = NV_FALSE;
    // Better Error handling required in this function
    return BootError;
}

/*
 * Initialize the internal UsbfContext and its pointer.
 *
 * NOTE: This must be called _after_ setting up the C runtime environment,
 *       which precludes its incorporation in NvBootUsbfInit().
 */
static NvBootUsbfInitContext(void)
{
    // Store the context for future use.
    s_pUsbfCtxt = &s_UsbfContext;

    // Return the base address to USB_BASE, as only USB1 can be used for
    // recovery mode.
    s_pUsbfCtxt->UsbBaseAddr = NV_ADDRESS_MAP_USB_BASE;

    // Set controller enabled to NV_FALSE until we start the controller.
    s_pUsbfCtxt->UsbControllerEnabled = NV_FALSE;

}

NvBool NvBootUsbfIsCableConnected(void)
{
    if (!s_pUsbfCtxt)
    {
        NvBootUsbfInitContext();
    }

    /* Return the B Session valid status for cable detection
    ** B session valid is "1" = NV_TRUE means cable is present.
    ** B session valid is "0" = NV_FALSE means cable is not present 
    **/
    return (USBIF_DRF_VAL(USB_PHY_VBUS_SENSORS,
                B_SESS_VLD_STS, 
                USBIF_REG_RD(s_pUsbfCtxt->UsbBaseAddr,
                USB_PHY_VBUS_SENSORS)));
}

static NvBootError
NvBootUsbfHwIntializeController(
    NvBootUsbfContext *pUsbFuncCtxt)
{
    NvU32 TimeOut = CONTROLLER_HW_TIMEOUT_US;
    NvU32 PhyClkValid = 0;
    NvU32 regValue = 0;

    // USB controller is not enabled yet, set to FALSE
    pUsbFuncCtxt->UsbControllerEnabled = NV_FALSE;


    //Bring respective USB and PHY out of reset by writing 0 to UTMIP_RESET    
    USBIF_REG_UPDATE_DEF(s_pUsbfCtxt->UsbBaseAddr,
                         USB_SUSP_CTRL,
                         UTMIP_RESET,
                         DISABLE);

    TimeOut = CONTROLLER_HW_TIMEOUT_US;
    do {
        //wait for the phy clock to become valid
        PhyClkValid = USBIF_REG_READ_VAL(s_pUsbfCtxt->UsbBaseAddr,
                                         USB_SUSP_CTRL,
                                         USB_PHY_CLK_VALID);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (!PhyClkValid);

    //By now USB controller is enabled set to TRUE
    pUsbFuncCtxt->UsbControllerEnabled = NV_TRUE;

    // Clear device address
    USB_REG_WR(PERIODICLISTBASE, USB_DEF_RESET_VAL(PERIODICLISTBASE));
    // Clear setup token, by reading and wrting back the same value
    USB_REG_WR(ENDPTSETUPSTAT, USB_REG_RD(ENDPTSETUPSTAT));
    // Clear endpoint complete status bits.by reading and writing back
    USB_REG_WR(ENDPTCOMPLETE, USB_REG_RD(ENDPTCOMPLETE));

    // STOP the USB controller
    USB_REG_UPDATE_DEF(USBCMD, RS, STOP);
    // Set the USB mode to the IDLE before reset
    USB_REG_UPDATE_DEF(USBMODE, CM, IDLE);
    // Reset the controller
    USB_REG_UPDATE_DEF(USBCMD, RST, SET);

    do {
        // Wait till  reset clears.
        regValue = USB_REG_READ_VAL(USBCMD, RST);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (regValue);

    TimeOut = CONTROLLER_HW_TIMEOUT_US;
    do {
        //wait for the phy clock to become valid
        PhyClkValid = USBIF_REG_READ_VAL(s_pUsbfCtxt->UsbBaseAddr,
                                         USB_SUSP_CTRL,
                                         USB_PHY_CLK_VALID);

        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (!PhyClkValid);

    // set the controller to device controller mode
    USB_REG_UPDATE_DEF( USBMODE, CM, DEVICE_MODE);
    TimeOut = CONTROLLER_HW_TIMEOUT_US;
    do {
        //wait till device mode change is finished.
        regValue = USB_REG_READ_VAL(USBMODE, CM);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (regValue != USB_DRF_DEF_VAL(USBMODE, CM, DEVICE_MODE));

    // Disable all USB interrupts
    USB_REG_WR(USBINTR, USB_DEF_RESET_VAL(USBINTR));
    USB_REG_WR(OTGSC, USB_DEF_RESET_VAL(OTGSC));

    // clear all pending interrupts, if any
    regValue = USB_DRF_DEF(USBSTS, SLI, SUSPENDED) |
               USB_DRF_DEF(USBSTS, SRI, SOF_RCVD) |
               USB_DRF_DEF(USBSTS, URI, USB_RESET) |
               USB_DRF_DEF(USBSTS, AAI, ADVANCED) |
               USB_DRF_DEF(USBSTS, SEI, ERROR) |
               USB_DRF_DEF(USBSTS, FRI, ROLLOVER) |
               USB_DRF_DEF(USBSTS, PCI, PORT_CHANGE) |
               USB_DRF_DEF(USBSTS, UEI, ERROR) |
               USB_DRF_DEF(USBSTS, UI, INT);
    USB_REG_WR(USBSTS, regValue);

    regValue = USB_DRF_DEF(OTGSC, DPIS, INT_SET) |
               USB_DRF_DEF(OTGSC, ONEMSS, INT_SET) |
               USB_DRF_DEF(OTGSC, BSEIS, INT_SET) |
               USB_DRF_DEF(OTGSC, BSVIS, INT_SET) |
               USB_DRF_DEF(OTGSC, ASVIS, INT_SET) |
               USB_DRF_DEF(OTGSC, AVVIS, INT_SET) |
               USB_DRF_DEF(OTGSC, IDIS, INT_SET);
    USB_REG_WR(OTGSC, regValue);

    regValue = USB_DRF_DEF(ENDPTSETUPSTAT, ENDPTSETUPSTAT0, SETUP_RCVD) |
               USB_DRF_DEF(ENDPTSETUPSTAT, ENDPTSETUPSTAT1, SETUP_RCVD) |
               USB_DRF_DEF(ENDPTSETUPSTAT, ENDPTSETUPSTAT2, SETUP_RCVD);
    USB_REG_WR(ENDPTSETUPSTAT, regValue);

    USB_REG_UPDATE_DEF(USBCMD, ITC, IMMEDIATE);

    return NvBootError_Success;
}
