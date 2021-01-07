/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */


#include "address_map_new.h"
#include "nvtypes.h"
#include "nvrm_drf.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "aruphy_lane.h"
#include "aruphy_pll.h"
#include "arclk_rst.h"
#include "nvboot_fuse_int.h"
#include "nvboot_util_int.h"
#include "nvboot_uphy_tables.h"

#define UPHY_LANE_RMW_FLD_NUM(r, f, v, b) \
do { \
    NvU32 rval = NV_READ32(b + UPHY_LANE_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(UPHY_LANE, r, f, v, rval); \
    NV_WRITE32(b + UPHY_LANE_##r##_0, rval); \
} while(0)

static void uphyLane5CfgWrite(uint8_t addr, NvU32 data)
{
    NvU32 rdata = NV_READ32(NV_ADDRESS_MAP_UPHY_LANE5_BASE + UPHY_LANE_DIRECT_CTL_2_0);
    rdata = NV_FLD_SET_DRF_NUM(UPHY, LANE_DIRECT_CTL_2, CFG_ADDR, addr, rdata);
    rdata = NV_FLD_SET_DRF_NUM(UPHY, LANE_DIRECT_CTL_2, CFG_WDATA, data, rdata);
    rdata = NV_FLD_SET_DRF_NUM(UPHY, LANE_DIRECT_CTL_2, CFG_WDS, 0x1, rdata);
    NV_WRITE32(NV_ADDRESS_MAP_UPHY_LANE5_BASE + UPHY_LANE_DIRECT_CTL_2_0, rdata);
}

static void uphyPllCfgWrite(NvU32 base, uint8_t addr, NvU32 data)
{
    NvU32 rdata = NV_READ32(base + UPHY_PLL_CTL_4_0);
    rdata = NV_FLD_SET_DRF_NUM(UPHY, PLL_CTL_4, CFG_ADDR, addr, rdata);
    rdata = NV_FLD_SET_DRF_NUM(UPHY, PLL_CTL_4, CFG_WDATA, data, rdata);
    rdata = NV_FLD_SET_DRF_NUM(UPHY, PLL_CTL_4, CFG_WDS, 0x1, rdata);
    NV_WRITE32(base + UPHY_PLL_CTL_4_0, rdata);
}

// This is taken slightly modified from NvBootPollField in Ufs driver, like most of
// the UPHY driver. I think this should be added to NvBootUtil, its a useful tool
// but the t186 BR is too late already to make changes like that.
static NvU32 UphyPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout)
{
    NV_ASSERT(Timeout != 0);
    NvU32 RegData;
    do {
        RegData = NV_READ32(RegAddr);
        if((RegData & Mask) == ExpectedValue)
            return NvBootError_Success;
        // waits this small are not great, Long Timeouts will be innaccurate.
        // if we added this routine to nvboot_util it
        // should be larger, but I'll leave it be given T186 BootROM's schedule
        NvBootUtilWaitUS(1);
        Timeout--;
    } while(Timeout);
    return NvBootError_HwTimeOut;
}

static void PllParamsSetup(NvU32 Pll)
{
    NvU32 RegData, PllBase;
    if(Pll == 0)
        PllBase = NV_ADDRESS_MAP_UPHY_PLL0_BASE;
    else
        PllBase = NV_ADDRESS_MAP_UPHY_PLL1_BASE;

    // PWR_OVRD
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, PWR_OVRD, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);//PLL0-CTL0

    // RCAL,CAL OVRD.
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, CAL_OVRD, 1,RegData);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_OVRD, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);//PLL0-CTL0

    NvBootUtilWaitUS(1);

    for(NvU32 i = 0; i < PLL_PARAMS_LENGTH; i++) {
        if (NvUphyClm_SataG1G2G3_init[i].en) {
            uphyPllCfgWrite(PllBase,
                            NvUphyClm_SataG1G2G3_init[i].addr,
                            NvUphyClm_SataG1G2G3_init[i].wdata);
        }
    }
}

static NvBootError PllFullInit(NvU32 Pll)
{
    NvBootError e = NvBootError_Success;
    NvU32 RegData, PllBase;
    NvU32 ShiftExpectedValue, ShiftMask;
    if(Pll == 0)
        PllBase = NV_ADDRESS_MAP_UPHY_PLL0_BASE;
    else
        PllBase = NV_ADDRESS_MAP_UPHY_PLL1_BASE;

    // NV_WRITE32(PllBase + UPHY_PLL_CTL_1_0,0x00000044); //PLL0 - CTL0
    // NV_WRITE32(PllBase + UPHY_PLL_CTL_2_0,0x00870000); //PLL0 - CTL2
    // IDDQ = 0, SLEEP=0 PLL1, PWR_OVR=1
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, IDDQ, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);//PLL0-CTL0
    NvBootUtilWaitUS(1);

    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, SLEEP, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);//PLL0-CTL0
    NvBootUtilWaitUS(1);

    // Perform Calibration.
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, CAL_EN, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);//PLL0-CTL0

    // TODO: This is supposed to be 200 us. Also do for rate A.
    ShiftMask = ShiftExpectedValue = UPHY_PLL_CTL_2_0_CAL_DONE_FIELD;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 200);

    // TODO Steps if calibration fails.

    // Turn off calibration and poll till CAL_DONE goes to 0.
    // Perform Calibration.
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, CAL_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);//PLL0-CTL0


    // What is the time out for this?
    ShiftMask =  UPHY_PLL_CTL_2_0_CAL_DONE_FIELD;
    ShiftExpectedValue = 0;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 200);

    // TODO. Exit if fail
    // Enable the PLL and wait for lock. (actual lock time 20us)
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, ENABLE, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);


    ShiftMask = ShiftExpectedValue = UPHY_PLL_CTL_1_0_LOCKDET_STATUS_FIELD;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_1_0,
                            ShiftMask, ShiftExpectedValue, 20);

    // TODO: exit if fail
    // Resistor calibration
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_EN, 1,RegData);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_CLK_EN, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    // Poll for calibration done
    ShiftMask = ShiftExpectedValue = UPHY_PLL_CTL_2_0_RCAL_DONE_FIELD;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 10);

    // TURN OFF CALIBRATION
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    ShiftMask = UPHY_PLL_CTL_2_0_RCAL_DONE_FIELD;
    ShiftExpectedValue = 0;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 5);
    // Turn off RCAL clk.
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_CLK_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    return e;
}

static NvBootError PllPartialInit(NvU32 Pll)
{
    NvBootError e = NvBootError_Success;
    NvU32 RegData, PllBase;
    NvU32 ShiftExpectedValue, ShiftMask;
    if(Pll == 0)
        PllBase = NV_ADDRESS_MAP_UPHY_PLL0_BASE;
    else
        PllBase = NV_ADDRESS_MAP_UPHY_PLL1_BASE;

    // NV_WRITE32(PllBase + UPHY_PLL_CTL_1_0,0x00000044); //PLL0 - CTL0
    // NV_WRITE32(PllBase + UPHY_PLL_CTL_2_0,0x00870000); //PLL0 - CTL2
    // IDDQ = 0, SLEEP=0 PLL1, PWR_OVR=1
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, IDDQ, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);//PLL0-CTL0

    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, SLEEP, 2,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);//PLL0-CTL0
    NvBootUtilWaitUS(1);

    // TODO: exit if fail
    // Resistor calibration
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_EN, 1,RegData);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_CLK_EN, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    // Poll for calibration done
    ShiftMask = ShiftExpectedValue = UPHY_PLL_CTL_2_0_RCAL_DONE_FIELD;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 10);

    // TURN OFF CALIBRATION
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    ShiftMask = UPHY_PLL_CTL_2_0_RCAL_DONE_FIELD;
    ShiftExpectedValue = 0;
    e = UphyPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 5);
    // Turn off RCAL clk.
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_CLK_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    return e;
}

static void LaneSetup()
{
    for(NvU32 i = 0; i < LANE_PARAMS_LENGTH; i++) {
        if (NvUphyDlm_SataG1G2_init[i].en) {
            uphyLane5CfgWrite(NvUphyDlm_SataG1G2_init[i].addr,
                              NvUphyDlm_SataG1G2_init[i].wdata);
        }
    }
}

static NvU32 UphyClkEnableResetDisable()
{
    NvU32 RegData, LaneBase;

    // UPHY Pad Macro reset disable
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                             RST_DEV_UPHY,
                             SWR_UPHY_RST,
                             DISABLE,
                             RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
           CLK_RST_CONTROLLER_RST_DEV_UPHY_0, RegData);

    NvBootUtilWaitUS(1);

    // Lane 5
    LaneBase = NV_ADDRESS_MAP_UPHY_LANE5_BASE;

    // Select SATA
    UPHY_LANE_RMW_FLD_NUM(MUX, SEL, 0x2, LaneBase);


    // MGMT Clks --> This is derived from PLL0. So PLL0 should be out of IDDQ ??? TODO 
    // Enable MGMT CLK to UPHY Lanes. MGMT clock to all UPHY lanes come from PLL0 MGMT CLK
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT,
                                 PEX_USB_PAD_PLL0_MGMT_CLK_CE,
                                 0x1,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT,
                                 PEX_USB_PAD_PLL1_MGMT_CLK_CE,
                                 0x1,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT_0, RegData);

    // Rx Byp RefClk. To bias PWM detector logic, UPHY uses RX_BYP_REFCLK. 
    // As M-PHY configuration of UPHY uses PWM detector,  RX_BYP_REFCLK should be enabled
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_SATA_USB_RX_BYP_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_PEX_SATA_USB_RX_BYP,
                                 PEX_USB_PAD_RX_BYP_REFCLK_CE,
                                 0x1,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_SATA_USB_RX_BYP_0, RegData);

    NvBootUtilWaitUS(1);


    // Remove global master  reset (global is misleading here!)
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                             RST_DEV_PEX_USB_UPHY,
                             SWR_PEX_USB_UPHY_RST,
                             DISABLE,
                             RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
           CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0, RegData);

    NvBootUtilWaitUS(1);

    // Remove Reset for UPHY PLL0 and PLL1
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                             RST_DEV_PEX_USB_UPHY,
                             SWR_PEX_USB_UPHY_PLL1_RST,
                             DISABLE,
                             RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                             RST_DEV_PEX_USB_UPHY,
                             SWR_PEX_USB_UPHY_PLL0_RST,
                             DISABLE,
                             RegData);

    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
           CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0, RegData);

    NvBootUtilWaitUS(1);


    // Per bug 200083804 program UPHY PLL0 IDDQ = 0 whenever M-PHY is enabled
    // Keep PLL0 in P3 state (IDDQ = 0 but SLEEP state = 3) to enable MGMT CLK to PLL1.
//TODO commented out
/*    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                             XUSBIO_PLL_CFG0,
                             XUSBIO_PADPLL_SLEEP_IDDQ,
                             0,
                             RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0, RegData);

    NvBootUtilWaitUS(1);
*/
    // De-assert reset on Lanes. Platform specific.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0);
//Changed
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                             RST_DEV_PEX_USB_UPHY,
                             SWR_PEX_USB_UPHY_L5_RST,
                             DISABLE,
                             RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
           CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0, RegData);

    NvBootUtilWaitUS(1);

    return NvBootError_Success;
}

/** Remove Clamps and IDDQ as a last step.
 *  Bug 1538216 Comment 13 http://nvbugs/1538216/13
 */
static void LaneIddqClampRelease(NvU32 Lane)
{
    NvU32 LaneBase;

    if(Lane == 0) // Lane 4
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE4_BASE;
    else // Lane 5
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE5_BASE;

    // Disable IDDQ =1 and CLAMP_EN_EARLY=0
    // FORCE_IDDQ_DISABLE
    UPHY_LANE_RMW_FLD_NUM(MUX, FORCE_IDDQ_DISABLE, 1, LaneBase);

    NvBootUtilWaitUS(1);

    // CLAMP_EN_EARLY
    UPHY_LANE_RMW_FLD_NUM(MUX, CLAMP_EN_EARLY, 0, LaneBase);
}

struct DynCtlVals {
    uint8_t TxDrvAmpSel0;
    uint8_t TxDrvAmpSel1;
    uint8_t TxDrvPostSel0;
    uint8_t TxDrvPostSel1;
};
static void SataUphyCalibPadCntrl(void)
{
    NvU32 LaneBase = NV_ADDRESS_MAP_UPHY_LANE5_BASE;

    //Tables have static storage class for easy patchability, maybe not needed, doesn't hurt
    //Index with Sata NV Calib
    static const uint8_t AuxRxIdleVals[] = {0, 1, 2, 3};
    //Index with Sata MPHY ODM Calib[1:0]
    static const struct DynCtlVals DynCtlValTbl[] =
        {{.TxDrvAmpSel0 = 0x1B, .TxDrvAmpSel1 = 0x1F, .TxDrvPostSel0 = 0x7, .TxDrvPostSel1 = 0xA},
         {.TxDrvAmpSel0 = 0x17, .TxDrvAmpSel1 = 0x1B, .TxDrvPostSel0 = 0x5, .TxDrvPostSel1 = 0xA},
         {.TxDrvAmpSel0 = 0x13, .TxDrvAmpSel1 = 0x17, .TxDrvPostSel0 = 0x4, .TxDrvPostSel1 = 0xA},
         {.TxDrvAmpSel0 = 0x1f, .TxDrvAmpSel1 = 0x23, .TxDrvPostSel0 = 0xA, .TxDrvPostSel1 = 0xE}};
    //Index with Sata MPHY ODM Calib[3:2]
    // No values for these in UPHY programming guide

    NvU32 SataNvCalib = NvBootFuseGetSataNvCalib();

    // Only want the Bottom 2 bits, sata_nv_calib[1:0]
    UPHY_LANE_RMW_FLD_NUM(AUX_CTL_1, AUX_RX_IDLE_TH, AuxRxIdleVals[SataNvCalib & 0x3], LaneBase);

    NvU32 SataMpyOdmCalib = NvBootFuseGetSataMpyOdmCalib();

    // Again ony want sata_mphy_odm_calib[1:0]
    struct DynCtlVals dvals = DynCtlValTbl[SataMpyOdmCalib & 0x3];
    UPHY_LANE_RMW_FLD_NUM(DYN_CTL_1, TX_DRV_AMP_SEL0, dvals.TxDrvAmpSel0, LaneBase);
    UPHY_LANE_RMW_FLD_NUM(DYN_CTL_1, TX_DRV_AMP_SEL1, dvals.TxDrvAmpSel1, LaneBase);
    UPHY_LANE_RMW_FLD_NUM(DYN_CTL_4, TX_DRV_POST_SEL0, dvals.TxDrvPostSel0, LaneBase);
    UPHY_LANE_RMW_FLD_NUM(DYN_CTL_4, TX_DRV_POST_SEL1, dvals.TxDrvPostSel1, LaneBase);
}

// Only function to be called from this file
NvBootError SataUPhyInit()
{
    //PLLP, PLLE, and PLLREFE should be running and set up before calling

    // Step 3 Release Reset for UPhy Pad macros
    UphyClkEnableResetDisable();

    PllParamsSetup(1);

    SataUphyCalibPadCntrl();

    LaneSetup();

    PllPartialInit(0);
    PllFullInit(1);

    NvBootUtilWaitUS(20);

    LaneIddqClampRelease(1);

    return NvBootError_Success;
}

