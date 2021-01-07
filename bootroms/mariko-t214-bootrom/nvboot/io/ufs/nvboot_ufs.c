/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_ufs_int.h"
#include "nvboot_util_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_xusb_dev_int.h"
#include "nvboot_ufs_hci.h"
#include "nvboot_ufs_param.h"
#include "nvboot_irom_patch_int.h"
//#include "nvboot_ufs_hc_reg.h"
#include "nvrm_drf.h"
#include "nvboot_error.h"
#include "nvboot_config.h"
#include "DWC_ufshc_header.h"
#include "address_map_new.h"
#include "aruphy_lane.h"
#include "aruphy_pll.h"
#include "arclk_rst.h"
#include "arufshc_aux.h"
#include "armphy_rx_apb.h"
#include "armphy_tx_apb.h"
#include "arpmc_impl.h"
#include "arpadctl_UFS.h"
#include "nvboot_config.h"
#include "arfuse.h"

/** READ/WRITE MACROS **/
#define BIT_MASK(REGFLD)  ((1 << REGFLD##_RegisterSize) - 1)
#define SHIFT(REGFLD) (REGFLD##_BitAddressOffset)
#define SHIFT_MASK(REGFLD)    (BIT_MASK(REGFLD) << SHIFT(REGFLD))
#define SET_FLD(REGFLD, VAL, REGDATA) ((REGDATA & ~SHIFT_MASK(REGFLD)) | (VAL << SHIFT(REGFLD)))
#define READ_FLD(REGFLD, REGDATA)  ((REGDATA & SHIFT_MASK(REGFLD)) >> SHIFT(REGFLD))

#define UFS_READ32(REG) NV_READ32(REG)

#define NV_READ32(ADDR) (*(volatile unsigned int*)(ADDR))

#define UFS_WRITE32(REG, VALUE) NV_WRITE32(REG, VALUE)

#define NV_WRITE32(ADDR, VALUE) (*(volatile unsigned int*)(ADDR) = VALUE)
#define TRUE 1
#define FALSE 0

/** Macros to convert endianness
 */
#define BYTE_SWAP32(a)   \
        ((((a)&0xff) << 24) | (((a)&0xff00)<< 8) | \
        (((a)&0xff0000) >> 8) | (((a)&0xff000000) >> 24))
#define BYTE_SWAP16(a)   \
        ( (((a)&0xff)<< 8) | (((a)&0xff00) >> 8))
/* TODO Find HCE SET wait time */

/* Align macros */
#define CEIL_PAGE(LEN, PAGE_SIZE)  (((LEN)+(PAGE_SIZE)-1)/(PAGE_SIZE))
#define ALIGN_LEN(LEN, BYTES) ((((LEN)+(BYTES)-1)/(BYTES)) * (BYTES))

// Setting large timeouts.
#define HCE_SET_TIMEOUT             500000
#define UTRLRDY_SET_TIMEOUT         500000
#define UTMRLRDY_SET_TIMEOUT        500000
#define IS_UCCS_TIMEOUT             500000
#define IS_UPMS_TIMEOUT             500000
#define NOP_TIMEOUT                 500000
#define QUERY_REQ_DESC_TIMEOUT      500000
#define QUERY_REQ_FLAG_TIMEOUT      500000
#define SCSI_REQ_READ_TIMEOUT      1000000
#define QUERY_REQ_ATTRB_TIMEOUT     500000
#define REQUEST_SENSE_TIMEOUT       500000


/** Static structure of TRDs in system memory aligned to 1KB boundary 
 */
#define NEXT_TRD_IDX(idx) (((idx) == ((MAX_TRD_NUM) - 1)) ? 0 : ((idx) + 1))

#define MAX_CMD_DESC_NUM    2
#define NEXT_CD_IDX(idx) (((idx) == ((MAX_CMD_DESC_NUM) - 1)) ? 0 : ((idx) + 1))

#define TX_REQ_DESC_START   (ALIGN_ADDR(NVBOOT_DEV_DS_START, 1024))
#define TX_REQ_DESC_SIZE    (MAX_TRD_NUM * sizeof(TransferRequestDescriptor_T))
#define TM_DESC_START       (ALIGN_ADDR(TX_REQ_DESC_START+TX_REQ_DESC_SIZE, 1024))
#define TM_DESC_SIZE        (MAX_TMD_NUM * sizeof(TaskMgmtRequestDescriptor_T))
#define CMD_DESC_START       (ALIGN_ADDR(TM_DESC_START+TM_DESC_SIZE, 128))
#define CMD_DESC_SIZE        (MAX_CMD_DESC_NUM * sizeof(CmdDescriptor_T))
#define DESC_BUFFER_START   (ALIGN_ADDR(CMD_DESC_START+CMD_DESC_SIZE, 4))
#define DESC_BUFFER_SIZE     (1024)
#define SYSRAM_DIFFERENCE   0x10000000
static TransferRequestDescriptor_T *TxReqDesc = 
                                (TransferRequestDescriptor_T*)TX_REQ_DESC_START;
static TaskMgmtRequestDescriptor_T *TaskMgmntDesc = 
                                (TaskMgmtRequestDescriptor_T *)TM_DESC_START;

static CmdDescriptor_T *CmdDescriptors = (CmdDescriptor_T *)CMD_DESC_START;


static NvBootUfsContext* pUfsContext;


static NvBootUfsParams s_UfsParams;

static UfsInternalParams_T s_UfsInternalParams;

static ClockInst s_UfsClkTable[] = 
{
    // Enable clocks for Ufs Host controller and device reference clock.
    Instc(Clk_W1b,  CLK_OUT_ENB_UFS_SET, SET_CLK_ENB_UFSDEV_REF, 
                                         SET_CLK_ENB_UFSHC),
    // Set Ufs clock source to 51 Mhz from PLLP.
    Instc(Clk_Src,  CLK_SOURCE_UFSHC_CG_SYS, UFSHC_CG_SYS_CLK_SRC, PLLP_OUT0,
                                             UFSHC_CG_SYS_CLK_DIVISOR, 0xE),
    // Set device reference clock source to osc (CLK_M, 38.4M)
    Instc(Clk_Src,  CLK_SOURCE_UFSDEV_REF, UFSDEV_REF_CLK_SRC, CLK_M,
                                           UFSDEV_REF_CLK_DIVISOR, 0),
    // Enable MPHY clocks
    Instc(Clk_W1b,  CLK_OUT_ENB_MPHY_SET, SET_CLK_ENB_MPHY_CORE_PLL_FIXED,
                                          SET_CLK_ENB_MPHY_TX_1MHZ_REF,
                                          SET_CLK_ENB_MPHY_IOBIST,
                                          SET_CLK_ENB_MPHY_L0_TX_LS_3XBIT),
    Instc(Clk_W1b,  CLK_OUT_ENB_MPHY_SET, SET_CLK_ENB_MPHY_L0_TX_SYMB, 
                                          SET_CLK_ENB_MPHY_L0_RX_LS_BIT,
                                          SET_CLK_ENB_MPHY_L0_RX_SYMB),
    Instc(Clk_W1b,  CLK_OUT_ENB_MPHY_SET, SET_CLK_ENB_MPHY_L0_RX_ANA,
                                          SET_CLK_ENB_MPHY_L1_RX_ANA),
    // Set MPHY core PLL to be 625/3=208 Mhz
    Instc(Clk_Rmw_Num,  CLK_SOURCE_MPHY_CORE_PLL_FIXED, MPHY_CORE_PLL_FIXED_CLK_DIVISOR, 4),
    // Set the reference for 1Mhz (in this case, 38.4/38 ~ 1.01 Mhz)
    Instc(Clk_Rmw_Num,  CLK_SOURCE_MPHY_TX_1MHZ_REF, MPHY_TX_1MHZ_REF_CLK_DIVISOR, 0x4A),
    // Reset Disable Mphy
    Instc(Clk_W1b,  RST_DEV_MPHY_CLR, CLR_SWR_MPHY_CLK_CTL_RST),
    Instc(Clk_W1b,  RST_DEV_MPHY_CLR, CLR_SWR_MPHY_L1_RX_RST,
                                      CLR_SWR_MPHY_L1_TX_RST,
                                      CLR_SWR_MPHY_L0_RX_RST,
                                      CLR_SWR_MPHY_L0_TX_RST),
    // Reset Disable Ufs
    Instc(Clk_W1b,  RST_DEV_UFS_AON_CLR, CLR_SWR_UFSHC_RST),
    Instc(Clk_W1b,  RST_DEV_UFS_CLR, CLR_SWR_UFSHC_AXI_M_RST,
                                     CLR_SWR_UFSHC_LP_RST),
    {0} // Null Terminated
};

static uint8_t UFS_Granularity_us[]=
{
    0, // Undefined.
    1, // 1 = 1us
    4, // 2 = 4 us
    8, // 3 = 8 us
    16,// 4 = 16 us
    32,// 5 = 32 us
    100, // 6 = 100 us
};

static uint8_t UFS_Tactivate_64us[]=
{
    0, // Undefined
    64, // 1 * 64
    16, // 4 * 16
    8, //  8 * 8
    4, //  16* 4
    2, //  32* 2
    1, // 100 * 1
};

/** 4 sets of timing/threshold values for PWM G1 ~ G4 */
static uint16_t DME_AFC0ReqTimeOutVal_x1[] = 
{
    1533, // PWM G1
    766,  // PWM G2
    383,  // PWM G3
    191   // PWM G4
};

static uint16_t DME_AFC0ReqTimeOutVal_x2[] = 
{
    833, // PWM G1
    441, // PWM G2
    220, // PWM G3
    110  // PWM G4
};

static uint16_t DME_TC0ReplayTimeOutVal_x1[] = 
{
    2986, // PWM G1
    1493, // PWM G2
    746, // PWM G3
    373  // PWM G4
};

static uint16_t DME_TC0ReplayTimeOutVal_x2[] = 
{
    1746, // PWM G1
    873,  // PWM G2
    436,  // PWM G3
    218   // PWM G4
};

static uint16_t DME_FC0ProtectionTimeOutVal_x1[] = 
{
    2786, // PWM G1
    1393, // PWM G2
    696, // PWM G3
    348  // PWM G4
};

static uint16_t DME_FC0ProtectionTimeOutVal_x2[] = 
{
    1620, // PWM G1
    810, // PWM G2
    405, // PWM G3
    202  // PWM G4
};
 
NvU32 NvBootUfsGetCmdDescriptor(NvU32 *pCmdDescIndex)
{
    NvU32 NextCmdIndex;
    

    if(pUfsContext->CmdDescInUse < MAX_CMD_DESC_NUM)
    {
        NextCmdIndex = NEXT_CD_IDX(pUfsContext->LastCmdDescIndex);
        pUfsContext->LastCmdDescIndex = *pCmdDescIndex = NextCmdIndex;
        pUfsContext->CmdDescInUse++;
        return NvBootError_Success;
    }
    else 
        return NvBootError_UFSResourceMax;
}

NvU32 NvBootPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout)
{
    NvU32 RegData;
    do {
    RegData = NV_READ32(RegAddr);
    if((RegData & Mask) == ExpectedValue)
        return NvBootError_Success;
    NvBootUtilWaitUS(1);
    Timeout--;
    } while(Timeout);
    return NvBootError_HwTimeOut;
}

NvU32 NvBootUfsLinkUphySetup()
{
    // No params till we patch. CLN default is good.
    NvBootUfsLinkUphyPllParamsSetup(0); // Pll 0
    NvBootUfsLinkUphyPllParamsSetup(1); // Pll 1

    // RCAL, PLL enable.
    NvBootUfsLinkUphyPllSetup(0); // Pll 0
    NvBootUfsLinkUphyPllSetup(1); // Pll 1

    NvBootUtilWaitUS(20);
    if(pUfsContext->NumLanes == 2)
    {
        NvBootUfsLinkUphyLaneSetup(0); // Lane 4
    }
    NvBootUfsLinkUphyLaneSetup(1); // Lane 5

    if(pUfsContext->NumLanes == 2)
    {
        NvBootUphyLaneIddqClampRelease(0); // Lane 4
    }
    NvBootUphyLaneIddqClampRelease(1); // Lane 5
    return NvBootError_Success;
}

NvU32 NvBootUfsLinkUphyPllSetup(NvU32 Pll)
{
    NvBootError e;
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
    e = NvBootPollField(PllBase+ UPHY_PLL_CTL_2_0,
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
    e = NvBootPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 200);

    // TODO. Exit if fail
    // Enable the PLL and wait for lock. (actual lock time 20us)
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_1, ENABLE, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_1_0,RegData);


    ShiftMask = ShiftExpectedValue = UPHY_PLL_CTL_1_0_LOCKDET_STATUS_FIELD;
    e = NvBootPollField(PllBase+ UPHY_PLL_CTL_1_0,
                            ShiftMask, ShiftExpectedValue, 20);


    // TODO: exit if fail
    // Resistor calibration
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_EN, 1,RegData);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_CLK_EN, 1,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    // Poll for calibration done
    ShiftMask = ShiftExpectedValue = UPHY_PLL_CTL_2_0_RCAL_DONE_FIELD;
    e = NvBootPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 10);

    // TURN OFF CALIBRATION
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    ShiftMask = UPHY_PLL_CTL_2_0_RCAL_DONE_FIELD;
    ShiftExpectedValue = 0;
    e = NvBootPollField(PllBase+ UPHY_PLL_CTL_2_0,
                            ShiftMask, ShiftExpectedValue, 5);
    // Turn off RCAL clk.
    RegData = NV_READ32(PllBase+ UPHY_PLL_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY,PLL_CTL_2, RCAL_CLK_EN, 0,RegData);
    NV_WRITE32(PllBase+ UPHY_PLL_CTL_2_0,RegData);

    return e;
}

NvU32 NvBootUfsLinkUphyLaneSetup(NvU32 Lane)
{
    NvU32 RegData, LaneBase, AuxRxIdleTh, RxPwmCtrl;

    if(Lane == 0) // Lane 4
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE4_BASE;
    else // Lane 5
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE5_BASE;

    // During boot, BootROM should read FUSE_MPHY_NV_CALIB_0 fuse register and 
    // check the fuse value of idle detector configuration (i.e. FUSE_MPHY_NV_CALIB_0[1:0])
    // and program      AUX_RX_IDLE_TH field of AUX_CTL_1 register as follows:
    // Register Name	Field Name	Based on FUSE_MPHY_NV_CALIB_0[1:0] value
	                            // 2’b00	2’b01	2’b10	2’b11
    // AUX_CTL_1	AUX_RX_IDLE_TH	0h	     1h	      2h	   3h

    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_MPHY_NV_CALIB_0);
    AuxRxIdleTh = NV_DRF_VAL(FUSE, MPHY_NV_CALIB, IDLE_DETECTOR, RegData);

    // Bits 15:5 = 0
    // Bits 4:3  = calib[3:2]
    // Bits 2:1  = calib[3:2]
    // Bit 0     = calib[4]

    RxPwmCtrl =  NV_DRF_VAL(FUSE, MPHY_NV_CALIB, PWM_DETECTOR_4, RegData);
    RxPwmCtrl |= (NV_DRF_VAL(FUSE, MPHY_NV_CALIB, PWM_DETECTOR_3_2, RegData) << 1);
    RxPwmCtrl |= (NV_DRF_VAL(FUSE, MPHY_NV_CALIB, PWM_DETECTOR_3_2, RegData) << 3);

    // Idle detector
    RegData = NV_READ32(LaneBase + UPHY_LANE_AUX_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY_LANE, AUX_CTL_1, AUX_RX_IDLE_TH , AuxRxIdleTh,RegData);
    NV_WRITE32(LaneBase + UPHY_LANE_AUX_CTL_1_0, RegData);

    // TX_DRV_AMP_SEL0,SEL1,SEL2,SEL3=0 for Lane 4
    NV_WRITE32(LaneBase + UPHY_LANE_DYN_CTL_1_0   ,0x11081108); //Lane4 - DYN_CTL_1
    NV_WRITE32(LaneBase + UPHY_LANE_DYN_CTL_2_0   ,0x11081108); //Lane4 - DYN_CTL_1
    NV_WRITE32(LaneBase + UPHY_LANE_DYN_CTL_3_0   ,0x00001108); //Lane4 - DYN_CTL_1
    NV_WRITE32(LaneBase + UPHY_LANE_DYN_CTL_4_0   ,0x000A0000); //Lane4 - DYN_CTL_1
    NV_WRITE32(LaneBase + UPHY_LANE_DYN_CTL_5_0   ,0x0808000F); //Lane4 - DYN_CTL_1 

    // From UPHY programming guide.
    // Register	CFG_ADDR	CFG_WDATA
    // DIRECT_CTL_2	8’d1: DLN_CFG_ID_CTRL	16'h0004
    // DIRECT_CTL_2	8’d3: MGMT_DLN_MISC_CTRL0	16'h0002
    // DIRECT_CTL_2	8’d10: MGMT_TX_CTRL	16'h0227
    // DIRECT_CTL_2	8’d11: MGMT_RX_CTRL	16'h0009
    // DIRECT_CTL_2	8’d30: MGMT_TXEQTERM_MISC_CTRL	16'h0002
    // DIRECT_CTL_2	8’d150: DLN_MISC_CTRL	16'h0001

     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x09010004); //LANE4 - CFG   
     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x09030002); //LANE4 - CFG   
     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x090A0227); //LANE4 - CFG
     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x090B0009); //LANE4 - CFG
     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x091E0002); //LANE4 - CFG
     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x09960001); //LANE4 - CFG

     if(NvBootGetSwCYA() & NVBOOT_SW_CYA_UFS_LANE)
     {
         NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x09040000); //LANE4 - CFG
         NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,0x09090011); //LANE4 - CFG
     }
    // DIRECT_CTL_2	8’d35: RX_PWM_CTRL
    // Bits 15:5 = 0
    // Bits 4:3 = calib[3:2]
    // Bits 2:1 = calib[3:2]
    // Bit 0 = calib[4]
     // RX PWM CTRL
     NV_WRITE32(LaneBase + UPHY_LANE_DIRECT_CTL_2_0,(0x09230000 | RxPwmCtrl)); // PWM Detector

     // RX_RATE_PDIV=1 (chooses 20 bit parallel interface). RX_BYP_MODE =2(PWM differential data) for Lane 4
     NV_WRITE32(LaneBase + UPHY_LANE_MPHY_CTL_2_0,0x00000022); //Lane4 - MPHY_CTL_2  
     // TX_RATE_PDIV=1. TX_BYP_MODE =0 (Differential output data) for Lane 4
     NV_WRITE32(LaneBase + UPHY_LANE_MPHY_CTL_1_0,0x00000002); //Lane4 - MPHY_CTL_1

    // Needed for biasing PWM detector
    RegData = NV_READ32(LaneBase + UPHY_LANE_MISC_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM(UPHY_LANE, MISC_CTL_2, RX_BYP_REFCLK_EN, 1,RegData);
    NV_WRITE32(LaneBase + UPHY_LANE_MISC_CTL_2_0,RegData ); //Lane4 - UPHY_LANE_MISC_CTL_2_0
    return NvBootError_Success;
}    

NvU32 NvBootUfsLinkUphyPllParamsSetup(NvU32 Pll)
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

 
     // M,N,P programming for rate A and rate B.
     // Program M,N,P for Rate A. 
     // Input clock = 208 Mhz from PLLREFE. , Out=2.496 Ghz Rate A
     // 2.496 DDR - 4.992 Gbps (Max HS Gear supported). 
     // 4.992/(20 bit ||l interface) ~ 250 Mhz. TxRefClk.
     // N=24 [7:0], M=1 (div by 2) [9:8], P=0 (div by 1) [11:10]
     // NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x09000118); //PLL0 - CFG   
     // // Input clock = 208 Mhz from PLLE. , Out=2.912 Ghz Rate B
     // // N=28 [7:0], M=1 (div by 2) [9:8], P=0 (div by 1) [11:10]
     // NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x0901011C); //PLL0 - CFG   
     // NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x09020000); //PLL0 - CFG   
     // // txclkref_en [0]=1, txclkref_sel=DIV_TX_BY_10 [5:4]
     // // txclkref = 250 Mhz.
     // // NV_WRITE32(NV_ADDRESS_MAP_UPHY_PLL1_BASE + UPHY_PLL_CTL_4_0,0x09037121); //PLL0 - CFG   
     // // NV_WRITE32(NV_ADDRESS_MAP_UPHY_PLL1_BASE + UPHY_PLL_CTL_4_0,0x09047121); //PLL0 - CFG

     if((NvBootGetSwCYA() & NVBOOT_SW_CYA_UFS_PLL0) && (PllBase == NV_ADDRESS_MAP_UPHY_PLL0_BASE))
     {
         NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x09000199); //PLL - CFG
         NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x090101c4); //PLL - CFG
     }

     if((NvBootGetSwCYA() & NVBOOT_SW_CYA_UFS_PLL1) && (PllBase == NV_ADDRESS_MAP_UPHY_PLL1_BASE))
     {
         NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x09000118); //PLL - CFG
         NV_WRITE32(PllBase + UPHY_PLL_CTL_4_0,0x0901011C); //PLL - CFG
     }
         
 
    return NvBootError_Success;
}


NvBootError NvBootUfsSetActivateTime()
{
    NvBootError e;
    NvU32 Data;
    NvU32 Local_Granularity;
    NvU32 Local_TActivate;
    NvU32 TActivate;

    // Dont need this as our MPHY is UniPro controlled.

    // // TX_Min_ActivateTime = 1 (100 us)
    // RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_30_33_0);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_30_33, TX_Min_ActivateTime, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_30_33_0, RegData);


    // // TX_Min_ActivateTime = 1 (100 us)
    // RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_30_33_0);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_30_33, TX_Min_ActivateTime, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_30_33_0, RegData);

    // // TX_Advanced_Granularity = 8 steps (64 us)
    // // TX_Advanced_Granularity_Setting = 1 (8 us)
    // RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity, 0x8, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity_Setting, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0, RegData);

    // RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity, 0x8, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity_Setting, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0, RegData);


    // // XFER_GO !!
    // RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR0, REG_XFER_GO, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0, RegData);

    // RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR0, REG_XFER_GO, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0, RegData);

    // Set DME Attribute for local.
    // Granularity 0x3=8us
    
    Local_Granularity = 0x0; e = NvBootSetDMECommand(DME_GET, 0, PA_Granularity, &Local_Granularity); // GRANULARITY 0x15aa
    if(e != NvBootError_Success)
        return e;

    
    Local_TActivate = 0x0; e = NvBootSetDMECommand(DME_GET, 0, PA_TActivate, &Local_TActivate); // 0x15a8
    if(e != NvBootError_Success)
        return e;

    TActivate = UFS_Granularity_us[Local_Granularity]*Local_TActivate;
    if(TActivate < 64)
    {
        
        Data = UFS_Tactivate_64us[Local_Granularity]&0xff; e = NvBootSetDMECommand(DME_SET, 0, PA_TActivate, &Data); // 0x15a8
        if(e != NvBootError_Success)
            return e;
    }
    return NvBootError_Success;
}
NvU32 NvBootUfsLinkMphySetup()
{
    NvBootError e;
    NvU32 RegData, Data;
    // Set Unipro timer for 1us.
    // Data = 0x33; NvBootSetDMECommand(DME_SET, 0x0, 0xFC, &Data);
    // wait a bit
    // NvBootUtilWaitUS(1);

#if NVBOOT_TARGET_RTL    
    //Update MPHY Attributes
    // MPhy Tx
    // LCC Disable

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0);
    // if(lane_polarity_reversal)RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_2C_2F, TX_DRIVER_POLARITY, 0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_2C_2F, TX_LCC_Enable, 0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0);
    // if(lane_polarity_reversal)RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_2C_2F, TX_DRIVER_POLARITY, 0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_2C_2F, TX_LCC_Enable, 0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0, RegData);

    // Adv.Granularity SyncLen
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity, 0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity_Setting, 0x1, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity_Setting, 0x5, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity, 0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity_Setting, 0x1, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_34_37, TX_Advanced_Granularity_Setting, 0x5, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_34_37_0, RegData);

    // LINE_RESET_TIME[original 0xc80 (3.2ms)]
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR3_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR3, LINE_RESET_TIME, 0x4c, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR3_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR3_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR3, LINE_RESET_TIME, 0x4c, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR3_0, RegData);

    // XFER_GO !!
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR0, REG_XFER_GO, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR0, REG_XFER_GO, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0, RegData);

     Data = 0x0; NvBootSetDMECommand(DME_SET, 0, PA_Local_TX_LCC_Enable, &Data); // DISABLE LCC Local 0x155e
     Data = 0x0; NvBootSetDMECommand(DME_SET, 0, PA_Peer_TX_LCC_Enable, &Data); // DISABLE LCC Peer 0x155f
     Data = 0x20; NvBootSetDMECommand(DME_SET, 0, PA_TxTrailingClocks, &Data); // PA_TxTrailingClocks 0x1564

     Data = 0x20; NvBootSetDMECommand(DME_SET, 0, PA_Hibern8Time, &Data); // T_HIBERN8 0x15a7
    // read back
     Data = 0;
     NvBootSetDMECommand(DME_GET, 0, PA_Hibern8Time, &Data); // T_HIBERN8 0x15a7
     // Data = 0xf; NvBootSetDMECommand(DME_SET, 0, PA_TActivate, &Data); // T_Activate 0x15a8
    // Sanjay asked to try this with default
     Data = 0x1; NvBootSetDMECommand(DME_SET, 0, PA_Granularity, &Data); // GRANULARITY 0x15aa
     Data = 0x20; NvBootSetDMECommand(DME_SET, 0, 0x2044, &Data); // DL AFC Credit Threshold 0x2044
     Data = 0x4; NvBootSetDMECommand(DME_SET, 0, 0x2045, &Data); // DL TC Out Ack Threshold 0x2045

    // This is for BFM to come up.
    NvBootUtilWaitUS(100);
#endif
    //  RX_Min_ActivateTime_Capability = 1 (100 us)
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_CAPABILITY_8C_8F_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, CAPABILITY_8C_8F, RX_Min_ActivateTime_Capability, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_CAPABILITY_8C_8F_0, RegData);

    //  RX_Min_ActivateTime_Capability = 1 (100 us)
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_CAPABILITY_8C_8F_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, CAPABILITY_8C_8F, RX_Min_ActivateTime_Capability, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_CAPABILITY_8C_8F_0, RegData);

    //  RX_Advanced_Step_Size_Granularity_Capability = 1 (8us)
    //  RX_Advanced_Min_ActivateTime_Capability = 8 steps of 1 us (64 us)
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_CAPABILITY_98_9B_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, CAPABILITY_98_9B, RX_Advanced_Min_ActivateTime_Capability, 0x8, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, CAPABILITY_98_9B, RX_Advanced_Step_Size_Granularity_Capability, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_CAPABILITY_98_9B_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_CAPABILITY_98_9B_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, CAPABILITY_98_9B, RX_Advanced_Min_ActivateTime_Capability, 0x8, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, CAPABILITY_98_9B, RX_Advanced_Step_Size_Granularity_Capability, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_CAPABILITY_98_9B_0, RegData);

    // REG UPDATE
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR2_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, VENDOR2, REGS_UPDATE, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR2_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR2_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX_APB, VENDOR2, REGS_UPDATE, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR2_0, RegData);
    
#if NVBOOT_TARGET_FPGA
    // prog Mphy attributes
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_2C_2F, TX_LCC_Enable, 0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_ATTRIBUTE_2C_2F, TX_LCC_Enable, 0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_ATTRIBUTE_2C_2F_0, RegData);

    // XFER_GO !!
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR0, REG_XFER_GO, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_VENDOR0, REG_XFER_GO, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0, RegData);

     Data = 0x0; NvBootSetDMECommand(DME_SET, 0, PA_Local_TX_LCC_Enable, &Data); // DISABLE LCC Local 0x155e
     Data = 0x0; NvBootSetDMECommand(DME_SET, 0, PA_Peer_TX_LCC_Enable, &Data); // DISABLE LCC Peer 0x155f

     // Device Clocks and Rst prog
    // Disable reference clock and enable reset to Device. OE pin on UFS GPIO pad
    RegData = NV_READ32(UFSHC_AUX_UFSHC_DEV_CTRL_0);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX, UFSHC_DEV_CTRL, UFSHC_DEV_CLK_EN, 0, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX, UFSHC_DEV_CTRL, UFSHC_DEV_RESET, 0, RegData);
    NV_WRITE32(UFSHC_AUX_UFSHC_DEV_CTRL_0, RegData);

    // Enable ref clock and disable reset.
    RegData = NV_READ32(UFSHC_AUX_UFSHC_DEV_CTRL_0);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX, UFSHC_DEV_CTRL, UFSHC_DEV_CLK_EN, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX, UFSHC_DEV_CTRL, UFSHC_DEV_RESET, 1, RegData);
    NV_WRITE32(UFSHC_AUX_UFSHC_DEV_CTRL_0, RegData);
    
    // MPhy clk divisors
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR22_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G1_BIT_CLK_DIVISOR, 0x34, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G2_BIT_CLK_DIVISOR, 0x1E, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G3_BIT_CLK_DIVISOR, 0xF, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G4_BIT_CLK_DIVISOR, 0x8, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR22_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR24_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR24, HS_G1_SYMB_CLK_DIVISOR, 0x8, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR24_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR22_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G1_BIT_CLK_DIVISOR, 0x34, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G2_BIT_CLK_DIVISOR, 0x1E, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G3_BIT_CLK_DIVISOR, 0xF, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR22, PWM_G4_BIT_CLK_DIVISOR, 0x8, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR22_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR24_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR24, HS_G1_SYMB_CLK_DIVISOR, 0x8, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR24_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR2_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR2, REGS_UPDATE, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_RX_APB_VENDOR2_0, RegData);
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR2_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_RX, APB_VENDOR2, REGS_UPDATE, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_RX_APB_VENDOR2_0, 0x1);


    // MPhy Tx
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_CLK_CTRL0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G1_3XBIT_CLK_DIV, 0x40, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G2_3XBIT_CLK_DIV, 0x20, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G3_3XBIT_CLK_DIV, 0x10, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G4_3XBIT_CLK_DIV, 0x8, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_CLK_CTRL0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_CLK_CTRL2_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL2, HS_G1_SYMB_CLK_DIV, 0x10, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_CLK_CTRL2_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_CLK_CTRL0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G1_3XBIT_CLK_DIV, 0x40, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G2_3XBIT_CLK_DIV, 0x20, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G3_3XBIT_CLK_DIV, 0x10, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL0, LS_G4_3XBIT_CLK_DIV, 0x8, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_CLK_CTRL0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_CLK_CTRL2_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX, APB_TX_CLK_CTRL2, HS_G1_SYMB_CLK_DIV, 0x10, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_CLK_CTRL2_0, RegData);
    
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_VENDOR0_0, 0x1);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_VENDOR0_0, 0x1);
#endif
    Data = 0x0; e = NvBootSetDMECommand(DME_LINKSTARTUP, 0, 0, &Data); // DME_LINKSTARTUP
    if(e != NvBootError_Success)
        return e;

    // HCI-Init-2
    RegData = 0;
    while( RegData != 0x0000000f) {
        // Read the Host Controller Status until the following bits are set to 1'b1 
        // DP, UTRLRDY,  UTMRLRDY,  UCRDY
        RegData = UFS_READ32(HCS);
    }
    return NvBootError_Success;
}

NvU32 NvBootSetDMECommand(uint8_t CmdOp, uint16_t GenSelIdx, uint16_t MIBAttr, NvU32* Data)
{
    DMECommand_T DmeCmd, *pDmeCmd;
    pDmeCmd = &DmeCmd;
    // TODO: AttrSetType
    NvU32 ShiftExpectedValue;
    NvBootError Error;

    NvBootUtilMemset((void*)pDmeCmd, 0, sizeof(DMECommand_T));
    pDmeCmd->UICCmd.CmdOp = CmdOp;
    pDmeCmd->UICCmdArg1.MIBAttribute = MIBAttr;
    pDmeCmd->UICCmdArg1.GenSelectorIndex = GenSelIdx;
    pDmeCmd->ReadWriteValue = *Data;

    // First set all the arguments and then write the command
    // Set MIB Attribute(Layer address:field), GenSelector (Lane/Cport/TestFeature)
    UFS_WRITE32(UICCMDARG1, pDmeCmd->UICCmdArg1.DW);

    // Write the AttrSetType only for these DME_SET and DME_PEER_SET
    if(pDmeCmd->UICCmd.CmdOp == DME_SET || pDmeCmd->UICCmd.CmdOp == DME_PEER_SET)
    {
        UFS_WRITE32(UICCMDARG2, pDmeCmd->UICCmdArg2.DW);
        UFS_WRITE32(UICCMDARG3, pDmeCmd->ReadWriteValue);
    }

    // Write the command now 
    UFS_WRITE32(UICCMD, pDmeCmd->UICCmd.DW);

    // Only for power mode, poll UIC Power Mode Status (UPMS),
    // otherwise UCCS UIC Command Completion Status (UCCS)
    if(MIBAttr==PA_PWRMode)
    {
        ShiftExpectedValue = 1 << SHIFT(IS_UPMS);
        Error = NvBootPollField(IS, SHIFT_MASK(IS_UPMS), ShiftExpectedValue, IS_UPMS_TIMEOUT);
        // TODO FIXME REMOVE BELOW STUFF
        pDmeCmd->ReadWriteValue = UFS_READ32(HCS);
    }
    else
    {
        // Poll for command complete.
        ShiftExpectedValue = 1 << SHIFT(IS_UCCS);
        Error = NvBootPollField(IS, SHIFT_MASK(IS_UCCS), ShiftExpectedValue, IS_UCCS_TIMEOUT);
    }
    if(Error != NvBootError_Success)
        return Error;
    // Clear IS UCCS bit
    UFS_WRITE32(IS, ShiftExpectedValue);

    // Check error code.
    // TODO more granular error handling.
    pDmeCmd->UICCmdArg2.DW = UFS_READ32(UICCMDARG2);
    
    if(pDmeCmd->UICCmdArg2.ConfigErrorCode)
        return NvBootError_UFSBootDMECmdError;

    // Get the read value in case of DME_GET
    if((pDmeCmd->UICCmd.CmdOp == DME_GET) || (pDmeCmd->UICCmd.CmdOp == DME_PEER_GET))
        *Data = pDmeCmd->ReadWriteValue = UFS_READ32(UICCMDARG3);

    return NvBootError_Success;
}

NvBootError NvBootUfsClockEnable(void)
{
    NvU32 RegData, LockTimeout;
    NvBootClocksOscFreq OscFreq;

    OscFreq = NvBootClocksGetOscFreq();

    NvBootXusbDeviceSetupSWControlUTMIPll(OscFreq);

    // Bring up PLLREFE. Clk input is UTMIP branch @60 Mhz. (UTMIP VCO is still @960 Mhz)
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLREFE_MISC_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 PLLREFE_MISC,
                                 PLLREFE_IDDQ,
                                 OFF,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_PLLREFE_MISC_0, RegData);

    NvBootUtilWaitUS(5);

    // We need PLLREFE @625 Mhz which is fed to M-PHY RX and TX blocks.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLREFE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_DIVM,
                                 0xc,
                                 RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_DIVN,
                                 0x7d,
                                 RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_DIVP,
                                 0x2,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_PLLREFE_BASE_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLREFE_BASE_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 PLLREFE_BASE,
                                 PLLREFE_ENABLE,
                                 ENABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_PLLREFE_BASE_0, RegData);

    NvBootUtilWaitUS(2);

    // Poll for lock
    LockTimeout = 1000; // 1ms
    while(LockTimeout)
    {
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_MISC_0);
        if(NV_DRF_VAL(CLK_RST_CONTROLLER, PLLREFE_MISC, PLLREFE_LOCK, RegData))
            break;
        NvBootUtilWaitUS(1);
        LockTimeout--;
    }

    // Enable clks for UFSHC and UFSDEV_REF
		#ifdef NV_CAR_HAS_UFS
			RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_CLK_OUT_ENB_UFS_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_OUT_ENB_UFS,
                                 CLK_ENB_UFSHC,
                                 ENABLE,
                                 RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_OUT_ENB_UFS,
                                 CLK_ENB_UFSDEV_REF,
                                 ENABLE,
                                 RegData);
	    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_UFS_0, RegData);
		#endif

    NvBootUtilWaitUS(1);

    // Set UFSHC clock src to PLLP
		#ifdef NV_CAR_HAS_UFS
	    RegData =  NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_UFSHC_CG_SYS_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_UFSHC_CG_SYS,
                                 UFSHC_CG_SYS_CLK_SRC,
                                 PLLP_OUT0,
                                 RegData);
    // Target Frequency = 51 Mhz from PLLP.
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_UFSHC_CG_SYS,
                                 UFSHC_CG_SYS_CLK_DIVISOR,
                                 0xE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_UFSHC_CG_SYS_0, RegData);
		#endif

    NvBootUtilWaitUS(1);

    // Set Device reference clock source to osc (CLK_M, 38.4 Mhz)
		#ifdef NV_CAR_HAS_UFS
    RegData =  NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_UFSDEV_REF_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_UFSDEV_REF,
                                 UFSDEV_REF_CLK_SRC,
                                 CLK_M,
                                 RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_UFSDEV_REF,
                                 UFSDEV_REF_CLK_DIVISOR,
                                 0,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_UFSDEV_REF_0, RegData);
		#endif
    NvBootUtilWaitUS(1);

    // Enable MPHY Clocks.
		#ifdef NV_CAR_HAS_MPHY
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_MPHY_0);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_CORE_PLL_FIXED, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_TX_1MHZ_REF, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_IOBIST, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_L0_TX_LS_3XBIT, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_L0_TX_SYMB, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_L0_RX_LS_BIT, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_L0_RX_SYMB, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_L0_RX_ANA, ENABLE, RegData);

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_OUT_ENB_MPHY,
                                 CLK_ENB_MPHY_L1_RX_ANA, ENABLE, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_MPHY_0, RegData);
		#endif
    NvBootUtilWaitUS(1);

    // Set MPHY core PLL to be 625/3=208 Mhz
		#ifdef NV_CAR_HAS_MPHY
    RegData =  NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_CORE_PLL_FIXED_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_MPHY_CORE_PLL_FIXED,
                                 MPHY_CORE_PLL_FIXED_CLK_DIVISOR,
                                 0x4,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_CORE_PLL_FIXED_0, RegData);

    // TODO This should come from a table of dividers for different oscillators.
    // Following divider assumes 38.4 Mhz Osc.
    // Set the reference for 1Mhz (in this case, 38.4/38 ~ 1.01 Mhz)
    RegData =  NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_TX_1MHZ_REF_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_MPHY_TX_1MHZ_REF,
                                 MPHY_TX_1MHZ_REF_CLK_DIVISOR,
                                 0x4A,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_TX_1MHZ_REF_0, RegData);
		#endif
    NvBootUtilWaitUS(1);

    return NvBootError_Success;
}
NvBootError NvBootUfsResetDisable()
{
    NvU32 RegData;

    // MPHY RESETS Disable
		#ifdef NV_CAR_HAS_MPHY
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_MPHY_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_MPHY,
                                 SWR_MPHY_CLK_CTL_RST, // Is this needed? Not in PG
                                 DISABLE,
                                 RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_MPHY,
                                 SWR_MPHY_L1_RX_RST,
                                 DISABLE,
                                 RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_MPHY,
                                 SWR_MPHY_L1_TX_RST,
                                 DISABLE,
                                 RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_MPHY,
                                 SWR_MPHY_L0_RX_RST,
                                 DISABLE,
                                 RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_MPHY,
                                 SWR_MPHY_L0_TX_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_RST_DEV_MPHY_0, RegData);
		#endif
    NvBootUtilWaitUS(1);

    // De-Assert UFS resets.

		#ifdef NV_CAR_HAS_UFS
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_UFS_AON_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_UFS_AON,
                                 SWR_UFSHC_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_RST_DEV_UFS_AON_0, RegData);

    NvBootUtilWaitUS(1);
    
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_UFS_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_UFS,
                                 SWR_UFSHC_AXI_M_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_RST_DEV_UFS_0, RegData);

    NvBootUtilWaitUS(1);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_UFS_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_UFS,
                                 SWR_UFSHC_LP_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_RST_DEV_UFS_0, RegData);
		#endif
    NvBootUtilWaitUS(1);

    // Set the following PMC register bits to ‘0’ to remove isolation between UFSHC AO logic inputs coming from PSW domain
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_UFSHC_PWR_CNTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, UFSHC_PWR_CNTRL,LP_ISOL_EN, DISABLE, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_UFSHC_PWR_CNTRL_0, RegData);

#if !NVBOOT_TARGET_FPGA
    // Enable reference clock to Device. OE pin on UFS GPIO pad
    RegData = NV_READ32(UFSHC_AUX_UFSHC_DEV_CTRL_0);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX, UFSHC_DEV_CTRL, UFSHC_DEV_CLK_EN, 1, RegData);
    NV_WRITE32(UFSHC_AUX_UFSHC_DEV_CTRL_0, RegData);

    // Release reset to device.
    RegData = NV_READ32(UFSHC_AUX_UFSHC_DEV_CTRL_0);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX, UFSHC_DEV_CTRL, UFSHC_DEV_RESET, 1, RegData);
    NV_WRITE32(UFSHC_AUX_UFSHC_DEV_CTRL_0, RegData);
#endif
    return NvBootError_Success;
}

NvU32 NvBootUfsUphyClkEnableResetDisable()
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

    if(pUfsContext->NumLanes == 2)
    {
        // Lane 4
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE4_BASE;

        // Select MPHY
        RegData = NV_READ32(LaneBase + UPHY_LANE_MUX_0);//L4 - Lane MUX
        RegData = NV_FLD_SET_DRF_NUM(UPHY_LANE, MUX, SEL, 0x3, RegData);
        NV_WRITE32(LaneBase + UPHY_LANE_MUX_0 ,RegData);//L4 - Lane MUX
    }
    // Lane 5
    LaneBase = NV_ADDRESS_MAP_UPHY_LANE5_BASE;
    
    // Select MPHY
    RegData = NV_READ32(LaneBase + UPHY_LANE_MUX_0);//L5 - Lane MUX
    RegData = NV_FLD_SET_DRF_NUM(UPHY_LANE, MUX, SEL, 0x3, RegData);
    NV_WRITE32(LaneBase + UPHY_LANE_MUX_0 ,RegData);//L5 - Lane MUX


    // MGMT Clks --> This is derived from PLL0. So PLL0 should be out of IDDQ ??? TODO 
    // Enable MGMT CLK to UPHY Lanes. MGMT clock to all UPHY lanes come from PLL0 MGMT CLK
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT, PEX_USB_PAD_PLL0_MGMT_CLK_CE, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT, PEX_USB_PAD_PLL1_MGMT_CLK_CE, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT_0, RegData);

    // Rx Byp RefClk. To bias PWM detector logic, UPHY uses RX_BYP_REFCLK. 
    // As M-PHY configuration of UPHY uses PWM detector,  RX_BYP_REFCLK should be enabled
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_PEX_SATA_USB_RX_BYP_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_PEX_SATA_USB_RX_BYP, PEX_USB_PAD_RX_BYP_REFCLK_CE, 0x1, RegData);
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
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                             XUSBIO_PLL_CFG0,
                             XUSBIO_PADPLL_SLEEP_IDDQ,
                             0,
                             RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0, RegData);

    NvBootUtilWaitUS(1);

    // De-assert reset on Lanes. Platform specific.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0);

    if(pUfsContext->NumLanes == 2)
    {
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEV_PEX_USB_UPHY,
                                 SWR_PEX_USB_UPHY_L4_RST,
                                 DISABLE,
                                 RegData);
    }
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

NvU32 NvBootUfsDisClkGate()
{
    // Remove ufs controller clock gates
    NvU32 RegData;
    RegData = NV_READ32(UFSHC_AUX_UFSHC_SW_EN_CLK_SLCG_0);

    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_PCLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_RX_SYMBOLCLKSELECTED_CLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_TX_SYMBOL_CLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_CG_SYS_CLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_CLK_T_CLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_LP_CLK_T_CLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_HCLK_OVR_ON, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFSHC_AUX,UFSHC_SW_EN_CLK_SLCG, 
                                 UFSHC_CLK_OVR_ON, 1, RegData);

    NV_WRITE32(UFSHC_AUX_UFSHC_SW_EN_CLK_SLCG_0, RegData);

    NvBootUtilWaitUS(1);

    // TODO: Double check this.
    // Remove Mphy lane clock gates
    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_CG_OVR0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_SYMB, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_CFG, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_3X, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_FIXED, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_SLOW, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_OVR_ON, 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L0_BASE + MPHY_TX_APB_TX_CG_OVR0_0, RegData);

    NvBootUtilWaitUS(1);

    RegData = NV_READ32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_CG_OVR0_0);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_SYMB, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_CFG, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_3X, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_FIXED, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_EN_SLOW, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(MPHY_TX_APB, TX_CG_OVR0, MPHY_TX_CLK_OVR_ON, 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_MPHY_L1_BASE + MPHY_TX_APB_TX_CG_OVR0_0, RegData);

    NvBootUtilWaitUS(1);
    return NvBootError_Success;
}

/** Remove Clamps and IDDQ as a last step. 
 *  Bug 1538216 Comment 13 http://nvbugs/1538216/13
 */
void NvBootUphyLaneIddqClampRelease(NvU32 Lane)
{
    NvU32 LaneBase, RegData;

    if(Lane == 0) // Lane 4
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE4_BASE;
    else // Lane 5
        LaneBase = NV_ADDRESS_MAP_UPHY_LANE5_BASE;

        // Disable IDDQ =1 and CLAMP_EN_EARLY=0
        // FORCE_IDDQ_DISABLE
        RegData = NV_READ32(LaneBase + UPHY_LANE_MUX_0);//L4/L5 - Lane MUX
        RegData = NV_FLD_SET_DRF_NUM(UPHY_LANE, MUX, FORCE_IDDQ_DISABLE, 1, RegData);
        NV_WRITE32(LaneBase + UPHY_LANE_MUX_0 ,RegData);//L4/L5 - Lane MUX

        NvBootUtilWaitUS(1);

        // CLAMP_EN_EARLY
        RegData = NV_READ32(LaneBase + UPHY_LANE_MUX_0);//L4/L5 - Lane MUX
        RegData = NV_FLD_SET_DRF_NUM(UPHY_LANE, MUX, CLAMP_EN_EARLY, 0, RegData);
        NV_WRITE32(LaneBase + UPHY_LANE_MUX_0 ,RegData);//L4/L5 - Lane MUX

}

/** Clocks, Resets, UFS Link bringup and Host controller setup.
     */
NvBootError NvBootUfsHwInit()
{
    NvU32 RegData, Error, ShiftExpectedValue=0;

    // Clock Enable - MPHY, UFSHC
    NvBootUfsClockEnable();

    // Reset disable - MPHY, UFSHC
    NvBootUfsResetDisable();

    // Disable SLCGs with Overrides. MPHY, UFSHC
    NvBootUfsDisClkGate();

    // Enable MGMT clocks, UPHY Pad Macros, UPHY PLL0/1, UPHY lane clocks
    NvBootUfsUphyClkEnableResetDisable();

    // Bring up UPHY PLLs and lanes.
#if !NVBOOT_TARGET_FPGA
    NvBootUfsLinkUphySetup();
#endif

    /* Write Enable to UFS HCE */
    RegData = UFS_READ32(HCE);
    RegData = SET_FLD(HCE_HCE,1,RegData);
    UFS_WRITE32(HCE, RegData);

    /* Poll for Initialization sequence to be complete */
    /* At the end of this sequence, DME RESET and DME LAYER ENABLE should have been
       completed. In a nutshell, Unipro layer is ready and is in link down state.
    */
    ShiftExpectedValue = 1 << SHIFT(HCE_HCE);
    Error = NvBootPollField(HCE, SHIFT_MASK(HCE_HCE), ShiftExpectedValue, HCE_SET_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    /* This is also done using a DME command with attribute 0xFC 
       Set HCLKDIV for 1us. */
    RegData = UFS_READ32(HCLKDIV);
    RegData = SET_FLD(HCLKDIV_HCLKDIV ,0x33,RegData);
    UFS_WRITE32(HCLKDIV, RegData);

    /* A lot of this is RTL stuff. Si stuff is only DME LINKSTARTUP and poll for
      Device Present (DP). */
    NvBootUfsLinkMphySetup();


    /* Setup buffers for Transfer Request and Task Managment engines*/
    NvBootUfsSetupTRDTMLists();
    /* Start TM and TR processing engines */
    NvBootUfsStartTMTREngines();

    if(!(NvBootGetSwCYA() & NVBOOT_SW_SKIP_ACTIVATE))
    {
        Error = NvBootUfsSetActivateTime();
        if(Error != NvBootError_Success)
            return Error;
    }

    /* Check if Device is Ready to receive UPIUs. Send NOP OUT till we get back NOP IN*/
    Error = NvBootUfsChkIfDevRdy2RcvDesc();
    if(Error != NvBootError_Success)
        return Error;

    if(!pUfsContext->BootEnabled)
    {
        Error = NvBootUfsCompleteInit();
        if(Error != NvBootError_Success)
            return Error;

        // Should query attribute and change power mode accordingly
        // Error = NvBootUfsGetAttribute(&currentPowerMode, QUERY_ATTRB_CURR_POWER_MODE, 0);
    }

    return NvBootError_Success;
}

/** Device Manager Callback for init and re-init.
*/
NvBootError
NvBootUfsInit(
    const NvBootUfsParams *Params,
    NvBootUfsContext *Context)
{
    NvBootError e;
    // This should be done only once.
    if(!Context->InitDone)
    {
        // Save Context obtained from device manager.
        pUfsContext = Context;
        
        // These cannot be changed through re-init through BCT.
        if(s_UfsInternalParams.BootEnabled)
        {
            pUfsContext->BootEnabled = 1;
            // 0xD0 is Boot W-LUN
            pUfsContext->BootLun = 0xB0; // 0x30 Actually. Bit7=1: to indicate W-LUN
        }
        else
        {
            pUfsContext->BootEnabled = 0;
            pUfsContext->BootLun = s_UfsInternalParams.BootLun;
        }

        // Currently 4096 * pow(2,fuse option). Default 4096 bytes.
        pUfsContext->PageSizeLog2 = s_UfsInternalParams.PageSizeLog2;
        pUfsContext->BlockSizeLog2 = UFS_BLOCK_SIZE_LOG2;
        // Even if Platform configuration is 0x2, UFS will come up with 1 lane in
        // Hibernate and we should explicitly change number of lanes.
        pUfsContext->ActiveLanes = 1; // Default
        // This is number of lanes as per platform config.
        pUfsContext->NumLanes = s_UfsInternalParams.NumLanes;

        e =  NvBootUfsHwInit();
        if(e != NvBootError_Success)
            return e;
        
        pUfsContext->CurrentPWMGear = 1;
        Context->InitDone = 1;
    }

    // Check if we have to change number of lanes because of fuse or reinit through BCT
    if(pUfsContext->ActiveLanes != Params->ActiveLanes)
    {
        e = NvBootUfsChangeNumLanes(Params->ActiveLanes);
        if(e != NvBootError_Success)
            return e;
        pUfsContext->ActiveLanes = Params->ActiveLanes;
    }
    
    // Check if we have to change gear because of fuse or reinit through BCT
    if(pUfsContext->CurrentPWMGear != Params->PWMGear)
    {
        e = NvBootUfsChangeGear(Params->PWMGear);
        if(e != NvBootError_Success)
            return e;
        pUfsContext->CurrentPWMGear = Params->PWMGear;
    }
    return NvBootError_Success;
}

/** Start Engines for processing Task Management
 * and TRD List. UTMRLRDY and UTRLRDY should be set
 */
NvU32 NvBootUfsStartTMTREngines(void)
{
    NvU32 ShiftExpectedValue, Error, RegData;
    /** Start Processing Engine for TM List and TRD list. 
     * TODO Define UTMRLRDY and UTRLRDY SET TIMEOUT
     */

    RegData = UFS_READ32(UTMRLRSR);
    RegData = SET_FLD(UTMRLRSR_UTMRLRSR,1,RegData);
    UFS_WRITE32(UTMRLRSR, RegData);
    
    RegData = UFS_READ32(UTRLRSR);
    RegData = SET_FLD(UTRLRSR_UTRLRSR,1,RegData);
    UFS_WRITE32(UTRLRSR, RegData);
    
    ShiftExpectedValue = 1 << SHIFT(HCS_UTMRLRDY);
    Error = NvBootPollField(HCS, SHIFT_MASK(HCS_UTMRLRDY), ShiftExpectedValue, UTMRLRDY_SET_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;
    ShiftExpectedValue = 1 << SHIFT(HCS_UTRLRDY);
    Error = NvBootPollField(HCS, SHIFT_MASK(HCS_UTRLRDY), ShiftExpectedValue, UTRLRDY_SET_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;
    return NvBootError_Success;
}
void NvBootUfsSetupTRDTMLists()
{
    /* Write Transfer Request List Lower and Upper Base Address */
    UFS_WRITE32(UTRLBA, (NvU32)&TxReqDesc[0]-SYSRAM_DIFFERENCE);
    UFS_WRITE32(UTRLBAU, 0);
    
    /* Write Task Management Request List Lower and Upper Base Address */
    UFS_WRITE32(UTMRLBA, (NvU32)&TaskMgmntDesc[0]-SYSRAM_DIFFERENCE);
    UFS_WRITE32(UTMRLBAU, 0);
}

/** Get a TRD slot from TRD list if available else
 *  return error.
 */
NvU32 NvBootUfsGetTxReqDescriptor(NvU32 *pTRDIndex)
{
    NvU32 TRDIndex, RegData;
    
    if(pUfsContext->TxReqDescInUse < MAX_TRD_NUM)
    {
        TRDIndex = NEXT_TRD_IDX(pUfsContext->LastTRDIndex);
        RegData = UFS_READ32(UTRLDBR);
        if(RegData & (1 << TRDIndex))
            return NvBootError_UFSFatalError;
        *pTRDIndex= TRDIndex;
        pUfsContext->LastTRDIndex = TRDIndex;
        pUfsContext->TxReqDescInUse++;
        return NvBootError_Success;
    }
    else
        return NvBootError_UFSResourceMax;
}
/** Creates TRD from given GenericCmdDescriptor (if slot available in TRD List)
 *  and returns success else returns UFSResourceMax Error.
 */

NvU32 NvBootUfsCreateTRD(NvU32 TRDIndex, NvU32 CmdDescIndex, NvU32 DataDir)
{
    TransferRequestDescriptor_T *pTxReqDesc;
    /* Get TRD */
    pTxReqDesc = &TxReqDesc[TRDIndex];
    NvBootUtilMemset((void*)pTxReqDesc, 0, sizeof(TransferRequestDescriptor_T));

    /* Command Type and Data Direction 
     * CmdType is always UFS Storage
     */
    pTxReqDesc->DW0.CT = UFS_TRD_DW0_0_CT_UFS; // 0x1: UFS Storage
    pTxReqDesc->DW0.DD = DataDir;

    /* Fill in Cmd Desc Lower and Upper Address. */
    pTxReqDesc->DW4.CTBA = ((NvU32)&CmdDescriptors[CmdDescIndex]-SYSRAM_DIFFERENCE) >> 7;

    /* Fill in Resp UPIU length and offset in Dwords*/
    pTxReqDesc->DW6.RUL = CMD_DESC_RESP_LENGTH/4;
    pTxReqDesc->DW6.RUO = CMD_DESC_REQ_LENGTH/4;
    
    if(DataDir != DATA_DIR_NIL) {
        pTxReqDesc->DW7.PRDTL = 1; // Bootrom will only use 1 PRDT entry.
        pTxReqDesc->DW7.PRDTO = (CMD_DESC_RESP_LENGTH+CMD_DESC_REQ_LENGTH)/4;
    }
    return NvBootError_Success;
}

NvU32 NvBootUfsQueueTRD(NvU32 TRDIndex, NvU32 TRDTimeout)
{
    NvU32 RegData;
    
    /** Confirm that slot *as calculated* is indeed empty
     *  else return error 
     */
    RegData = UFS_READ32(UTRLDBR);
    if(RegData & (1 << TRDIndex))
    {
        NvBootUfsFreeTRDCmdDesc();
        return NvBootError_UFSFatalError;
    }
    /* All slots except the required should be set to zero */
    RegData = 1 << TRDIndex;
    //RegData =  SET_FLD( UTRLDBR, UTRLDBR, 1, RegData);
    UFS_WRITE32(UTRLDBR, RegData);
    /* define TRD Info struct */
    NvBootUtilMemset((void*)&pUfsContext->TRDInfo[TRDIndex], 0, sizeof(TRDInfo_T));
    pUfsContext->TRDInfo[TRDIndex].TRDStartTime = NvBootUtilGetTimeUS();
    pUfsContext->TRDInfo[TRDIndex].TRDTimeout = TRDTimeout;
    return NvBootError_Success;
}

NvU32 NvBootUfsWaitTRDRequestComplete(NvU32 TRDIndex, NvU32 Timeout)
{
    NvU32 RegData, Error, ShiftExpectedValue, Mask;
    volatile TransferRequestDescriptor_T *pTxReqDesc;
    NvU32 *pResponse;
    CmdDescriptor_T *pCmdDescriptor;

    pTxReqDesc = &TxReqDesc[TRDIndex];
    pCmdDescriptor = (CmdDescriptor_T *)((pTxReqDesc->DW4.CTBA << 7)+SYSRAM_DIFFERENCE);
    Mask = 1 << TRDIndex;
    ShiftExpectedValue = 0;
    /* TODO TIMEOUT for DOORBELL SET */

    Error = NvBootPollField(UTRLDBR, Mask, ShiftExpectedValue, Timeout);
    if(Error!= NvBootError_Success)
        return Error;

    // First print out the response. REMOVE THIS> FIXME TODO
    pResponse = (NvU32*)&pCmdDescriptor->UCDGenericRespUPIU;
    
    RegData = NV_READ32(pResponse++); // DW0
    RegData = NV_READ32(pResponse++); // DW1
    RegData = NV_READ32(pResponse++); // DW2
    RegData = NV_READ32(pResponse++); // DW3
    RegData = NV_READ32(pResponse++); // DW4
    RegData = NV_READ32(pResponse++); // DW5
    RegData = NV_READ32(pResponse++); // DW6
    RegData = NV_READ32(pResponse++); // DW7

    if(pTxReqDesc->DW2.OCS != OCS_SUCCESS)
        return NvBootError_UFSFatalError;

    RegData = UFS_READ32(IS);
    RegData = UFS_READ32(HCS);
    RegData = UFS_READ32(UECPA);
    RegData = UFS_READ32(UECDL);
    RegData = UFS_READ32(UECN);
    RegData = UFS_READ32(UECT);
    RegData = UFS_READ32(UECDME);


    if(READ_FLD(IS_SBFES, RegData) || READ_FLD(IS_HCFES, RegData) \
       || READ_FLD(IS_UTPES, RegData) || READ_FLD(IS_DFES, RegData))
        return NvBootError_UFSFatalError;
        
    return NvBootError_Success;
}

void NvBootUfsFreeTRDCmdDesc(void)
{
    
    pUfsContext->TxReqDescInUse--;
    pUfsContext->CmdDescInUse--;
    /* Free associated command descriptor */
}

NvU32 NvBootUfsChkIfDevRdy2RcvDesc()
{
    /* Send NOPs OUT till you get NOP in */
    NOP_UPIU_T *pNOP_UPIU;
    CmdDescriptor_T *pCmdDescriptor;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success, NOPReceived =0;

    do
    {
        Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
        if(Error !=NvBootError_Success)
            return Error;

        Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
        if(Error != NvBootError_Success)
            return Error;

        pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
        NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

        /* Create NOP_OUT UPIU. Only Transaction Code needed */
        pNOP_UPIU = (NOP_UPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
        pNOP_UPIU->BasicHeader.TransCode = UPIU_NOP_OUT_TRANSACTION;
        // TODO Add task tag

        Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
        if(Error!= NvBootError_Success)
            return Error;
        // /* TODO Timeout for NOP */
        Error = NvBootUfsQueueTRD(TRDIndex, NOP_TIMEOUT);
        if(Error != NvBootError_Success)
            return Error;
        
        /* TODO Timeout for NOP */
        Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, NOP_TIMEOUT);
        if(Error!= NvBootError_Success)
            return Error;
        NOPReceived = NvBootUfsIsRespNOPIN(CmdDescIndex);
        NvBootUfsFreeTRDCmdDesc();
    }
    while(!NOPReceived);
    return NvBootError_Success;
    /* TODO HOW MANY TIMES SHOULD WE SEND NOP OUTs */
}

NvU32 NvBootUfsIsRespNOPIN(NvU32 CmdDescIndex)
{
    NOP_UPIU_T *pNOP_UPIU;
    CmdDescriptor_T *pCmdDescriptor;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    pNOP_UPIU = (NOP_UPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
    if(pNOP_UPIU->BasicHeader.TransCode == \
        UPIU_NOP_IN_TRANSACTION)
        return TRUE;
    else 
        return FALSE;
}
/** Get Descriptor to identify device characteristics
 */
NvU32 NvBootUfsGetDescriptor(uint8_t *pUFSDesc, NvU32 DescIDN, NvU32 DescIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    
    QueryReqRespUPIU_T *pQueryReqUPIU, *pQueryRespUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqUPIU->BasicHeader.TransCode = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_FUNC_STD_READ;
    pQueryReqUPIU->TSF.Opcode = TSF_OPCODE_READ_DESC;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = QUERY_DESC_DEVICE_DESC_IDN;
    pQueryReqUPIU->TSF.DescFields.DescIDN = DescIDN;
    /* Index */
    pQueryReqUPIU->TSF.DescFields.Index = DescIndex;
    /** TODO Find out device descriptor size. It is ok to simply specify max desc size
     * of 255 bytes as device will return actual descriptor size if lesser.
     */
    pQueryReqUPIU->TSF.DescFields.LengthBigE = BYTE_SWAP16(255);
    
    /* TODO Task Tags */
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ DESCRIPTOR */
    Error = NvBootUfsQueueTRD(TRDIndex, QUERY_REQ_DESC_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for NOP */
    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, QUERY_REQ_DESC_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    pQueryRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
    NvBootUtilMemcpy((void*)pUFSDesc, \
           (void*)&(pQueryRespUPIU->DataSegment[0]), \
            BYTE_SWAP16(pQueryRespUPIU->BasicHeader.DataSegLenBigE));    

    NvBootUfsFreeTRDCmdDesc();
    return NvBootError_Success;
}

/* This routine is called if BootDisable Fuse is not set.
 * It assumes BootLUN is available and enabled (else returns appropriate error)
 * This is done after Partial Initilization. If we find Boot is not enabled,
 * we have to exit and complete initialization.
 */
NvU32 NvBootUfsGetDevInfoPartialInit()
{
    NvU32 Error;
    uint8_t DescBuffer[UFS_DESC_MAX_SIZE];
    UFSDeviceDesc_T *pDevDesc = 0;
    
    

    /** First things first. Get Device descriptor, geometry?
     * Device Descriptor itself may not be accessible during partial initialzation.
     * So keep small timeout. TODO
     * Is device Bootable ?? If no, return error.
     * Following commands can be sent at this time
     * INQUIRY, TEST UNIT READY, READ.
     * Only device descriptor can be read at this time if allowed.
     */
    Error = NvBootUfsGetDescriptor(&DescBuffer[0], QUERY_DESC_DEVICE_DESC_IDN, 0);
    if(Error != NvBootError_Success)
        return Error;
    pDevDesc = (UFSDeviceDesc_T *)&DescBuffer[0];
    
    /* Get what we need from the device descriptor */
    pUfsContext->BootEnabled = pDevDesc->bBootEnable;
    pUfsContext->NumLUN = pDevDesc->bNumberLU;
    /* Do we need this ? */
    pUfsContext->NumWLU = pDevDesc->bNumberWLU;
    // TODO this should come from fuses and default will be 4K.
    // Num of blocks should not matter for boot enable as we will read BCT+BL 
    // from same logical unit.
    // It matters for boot not enabled. For that case, we might have to read num of 
    // blocks and handle the case where block num is out of bounds.
    pUfsContext->BootLUNBlockSize = 4096;
    pUfsContext->BootLUNNumBlocks = 0x100;
    // if(pUfsContext->BootEnabled)
    // {
        // /* BIG TODO ReadAttribute to get BootLunEn*/
        // /* Find out if Boot LUN A or B is mapped */
        // NvBootUfsGetAttribute(&BootLUNID, BOOT_LUN_EN_ATTRB, 0);
        // /* Find out who is actually Boot LUN A or B. 
         // * This step is only required to support GPT.
         // * i.e to find out total size of Boot Partition.
         // */
        // for(LUN=0; LUN < pUfsContext->NumLUN; LUN++)
        // {
            // Error = NvBootUfsGetDescriptor(&DescBuffer[0], UNIT_DESC_IDN, LUN);
            // if(Error != NvBootError_Success)
                // return Error;
            // pUnitDesc = (UFSUnitDesc_T *)&DescBuffer[0];
            // if(pUnitDesc->bBootLUNID == BootLUNID)
            // {   
                // FoundBootLUN = TRUE;
                // break;
            // }
        // }
        // if(!FoundBootLUN)
            // return NvBootError_UFSBootLUNNotFound;
        
        // /* Make sure LUN is enabled */
        // if(!pUnitDesc->bLUEnable)
            // return NvBootError_UFSBootLUNNotEnabled;

        // /* Extract useful info from Unit descriptor */
        // pUfsContext->BootLUNBlockSize = pUnitDesc->bLogicalBlockSize << UFS_MIN_BLOCK_SIZE_LOG2;
        // pUfsContext->BootLUNNumBlocks = pUnitDesc->qLogicalBlockCount;
        // return NvBootError_Success;
     // }
    // else
        // return NvBootError_UFSBootNotEnabled;
    if(!pUfsContext->BootEnabled)
    {
        return NvBootError_UFSBootLUNNotEnabled;
    }
    return NvBootError_Success;
}
NvU32 NvBootUfsGetAttribute(NvU32 *pUFSAttrb, NvU32 AttrbIDN, NvU32 AttrbIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    QueryReqRespUPIU_T *pQueryReqUPIU;
    
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqUPIU->BasicHeader.TransCode = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_FUNC_STD_READ;
    pQueryReqUPIU->TSF.Opcode = TSF_OPCODE_READ_ATTRB;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = QUERY_DESC_DEVICE_DESC_IDN;
    pQueryReqUPIU->TSF.AttrbFields.AttrbIDN = AttrbIDN;
    /* Index */
    pQueryReqUPIU->TSF.AttrbFields.Index = AttrbIndex;
    
    /* TODO Task Tags */
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, 0);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ ATTRB */
    Error = NvBootUfsQueueTRD(TRDIndex, QUERY_REQ_ATTRB_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for ATTRB
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, QUERY_REQ_ATTRB_TIMEOUT);
    // FIXME
    if(Error!= NvBootError_Success)
        return Error;
    
    pQueryReqUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
    *pUFSAttrb = BYTE_SWAP32(pQueryReqUPIU->TSF.AttrbFields.ValueBigE);

    NvBootUfsFreeTRDCmdDesc();
    return NvBootError_Success;

}
NvU32 NvBootUfsGetFlag(NvU32 *pUFSFlag, NvU32 FlagIDN, NvU32 FlagIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    QueryReqRespUPIU_T *pQueryReqUPIU, *pQueryRespUPIU;
    
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqUPIU->BasicHeader.TransCode = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_FUNC_STD_READ;
    pQueryReqUPIU->TSF.Opcode = TSF_OPCODE_READ_FLAG;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = QUERY_DESC_DEVICE_DESC_IDN;
    pQueryReqUPIU->TSF.FlagFields.FlagIDN = FlagIDN;
    /* Index */
    pQueryReqUPIU->TSF.FlagFields.Index = FlagIndex;
    
    /* TODO Task Tags */
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ FLAG */
    Error = NvBootUfsQueueTRD(TRDIndex, QUERY_REQ_FLAG_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for FLAG
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, QUERY_REQ_FLAG_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    pQueryRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
    *pUFSFlag = pQueryRespUPIU->TSF.FlagFields.FlagValue;

    NvBootUfsFreeTRDCmdDesc();
    
    return Error;

}

NvU32 NvBootUfsSetFlag(NvU32 FlagIDN, NvU32 FlagIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    QueryReqRespUPIU_T *pQueryReqUPIU, *pQueryRespUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;
    uint8_t FlagReadBack;
    

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqUPIU->BasicHeader.TransCode = UPIU_QUERY_REQUEST_TRANSACTION;
    /* TODO: Is SET FLAG a Read or Write Query function */
    pQueryReqUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_FUNC_STD_WRITE;
    pQueryReqUPIU->TSF.Opcode = TSF_OPCODE_SET_FLAG;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = QUERY_DESC_DEVICE_DESC_IDN;
    pQueryReqUPIU->TSF.FlagFields.FlagIDN = FlagIDN;
    /* Index */
    pQueryReqUPIU->TSF.FlagFields.Index = FlagIndex;
    
    /* TODO Task Tags */
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ FLAG */
    Error = NvBootUfsQueueTRD(TRDIndex, QUERY_REQ_FLAG_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for FLAG
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, QUERY_REQ_FLAG_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    pQueryRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
    /* OSF6 LSB (Big Endian) TODO contains the returned flag*/
    FlagReadBack = pQueryRespUPIU->TSF.FlagFields.FlagValue;
    if(FlagReadBack != 1)
        Error = NvBootError_UFSFlagSet;
    NvBootUfsFreeTRDCmdDesc();

    return Error;
}

NvU32 NvBootUfsCompleteInit(void)
{
    NvU32 e, FlagDeviceInit;
    /* Set fDeviceInit flag and poll for fDeviceInit flag to be reset
     * by device to 0. The device will move from Boot W-LU Ready to 
     * Device Initialization Complete stage 
     */
    e = NvBootUfsSetFlag(QUERY_FLAG_DEVICE_INIT_IDN, 0);
    if(e != NvBootError_Success)
        return e;

    /* TODO: Timeout waiting for deviceInit */
    do
    {
        e = NvBootUfsGetFlag(&FlagDeviceInit, QUERY_FLAG_DEVICE_INIT_IDN, 0);
        if(e != NvBootError_Success)
            return e;
    }
    while(FlagDeviceInit!=0);

    return NvBootError_Success;
}

/** Retrieves Device Descriptor and Unit descriptor for LUN from fuses.
 * Called only if Boot LUN is not enabled or Descriptor access failed 
 * during partial Init
 */
NvU32 NvBootUfsGetDevInfo(void)
{
    // NvU32 Error;
    // uint8_t DescBuffer[UFS_DESC_MAX_SIZE];
    // UFSDeviceDesc_T *pDevDesc;
    // UFSUnitDesc_T *pUnitDesc;
    

    // Error = NvBootUfsGetDescriptor(&DescBuffer[0], QUERY_DESC_DEVICE_DESC_IDN, 0);
    // if(Error != NvBootError_Success)
        // return Error;
    // pDevDesc = (UFSDeviceDesc_T *)&DescBuffer[0];
    
    // /* Get what we need from the device descriptor */
    // pUfsContext->BootEnabled = pDevDesc->bBootEnable;
    // pUfsContext->NumLUN = pDevDesc->bNumberLU;
    // /* Do we need this ? */
    // pUfsContext->NumWLU = pDevDesc->bNumberWLU;


    // Error = NvBootUfsGetDescriptor(&DescBuffer[0], UNIT_DESC_IDN, pUfsContext->FuseParamLUN);
    // if(Error != NvBootError_Success)
        // return Error;
    // pUnitDesc = (UFSUnitDesc_T *)&DescBuffer[0];
    // /* Make sure LUN is enabled */
    // if(!pUnitDesc->bLUEnable)
        // return NvBootError_UFSLUNNotEnabled;

    // /* Extract useful info from Unit descriptor */
    // pUfsContext->BootLUNBlockSize = pUnitDesc->bLUNBlockSize << UFS_MIN_BLOCK_SIZE_LOG2;
    // pUfsContext->BootLUNNumBlocks = pUnitDesc->qLUNBlockCount;
    return NvBootError_Success;

}

NvU32 NvBootUfsRead(const NvU32 Block, const NvU32 Page, const NvU32 Length, NvU32 *pBuffer)
{
    CmdDescriptor_T *pCmdDescriptor;
    CommandUPIU_T *pCommandUPIU;
    ResponseUPIU_T* pResponseUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;
    NvU32 LUNBlock, LUNReady = 0;
    uint16_t numBlocks;
    

    // First check that LUN is ready for the command.
    while(!LUNReady)
    {
        // Check LUN is ready.
        Error = NvBootUfsTestUnitReady(pUfsContext->BootLun);
        if(Error == NvBootError_Success)
        {
            LUNReady = 1;
        }
        // If LUN Busy, let's try again after a small delay.
        else if(Error == NvBootError_UFSLUNBusy)
        {
            NvBootUtilWaitUS(10);
        }
        else if(Error == NvBootError_UFSLUNCheckCondition)
        {
            // Issue Request Sense command to clear the Check Condition
            Error = NvBootUfsRequestSense(pUfsContext->BootLun);
            if(Error == NvBootError_UFSLUNBusy || Error == NvBootError_Success)
                continue;
            else // Even Check Condition is a fatal error for Request Sense
                return Error;
        }
        else // Fatal error
            return Error;
    }

    // UFS Logical Units has blocks which in bootrom context is pages.
    LUNBlock = ((Block << (pUfsContext->BlockSizeLog2 - pUfsContext->PageSizeLog2)) + Page );
    // Read whole number of blocks.
    numBlocks = CEIL_PAGE(Length,1<<pUfsContext->PageSizeLog2);

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

    pCommandUPIU = (CommandUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;

    /// Program UPIU Header
    pCommandUPIU->BasicHeader.TransCode = UPIU_COMMAND_TRANSACTION;
    // The flag indicates the direction of DATA UPIU. Only relevant for Command UPIU
    pCommandUPIU->BasicHeader.Flags = 1 << UFS_UPIU_FLAGS_R_SHIFT;
    pCommandUPIU->BasicHeader.LUN = pUfsContext->BootLun;
    pCommandUPIU->BasicHeader.CmdSetType = UPIU_COMMAND_SET_SCSI;
    pCommandUPIU->ExpectedDataTxLenBigE = BYTE_SWAP32( numBlocks*(1<<pUfsContext->PageSizeLog2));

    /* Construct CDB for READ(10) command */
    pCommandUPIU->CDB[0] = 0x28;
    /* Byte 1-3 are used for LBA in Big endian format */
    pCommandUPIU->CDB[5] = (LUNBlock & 0xFF); // LSB
    pCommandUPIU->CDB[4] = (LUNBlock >> 8) & 0xFF;
    pCommandUPIU->CDB[3] = (LUNBlock >> 16) & 0xFF;
    pCommandUPIU->CDB[2] = (LUNBlock >> 24) & 0xFF; //MSB

    /* Fill in transfer length in num of blocks*/
    pCommandUPIU->CDB[7] = (numBlocks >> 8) & 0xFF; // MSB
    pCommandUPIU->CDB[8] = numBlocks &0xff; // LSB
    /* Fill in control = 0x00 */
    pCommandUPIU->CDB[9] = 0;

     /*
     * DW0 Data Base Address Lower bits DWORD aligned.
     * DW1 Data Base Address Upper bits: set to 0. 64 bit Not supported.
     * DW3 Data byte count. 0 based value. Minimum 4 bytes i.e. 3
     */
    pCmdDescriptor->PRDT.DW0 = (NvU32)pBuffer & ~(0x3);
    pCmdDescriptor->PRDT.DW1 = 0;
    // Make sure length is aligned to num_pages*page_size.
    pCmdDescriptor->PRDT.DW3 = (numBlocks*(1<<pUfsContext->PageSizeLog2))-1;
    
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_D2H);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for SCSI READ*/
    Error = NvBootUfsQueueTRD(TRDIndex, SCSI_REQ_READ_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;

    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, SCSI_REQ_READ_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    // Analyze Response to see if read is ok.
    pResponseUPIU = (ResponseUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
#define TARGET_SUCCESS 0x0
#define TARGET_FAILURE 0x1

    // Check UFS Response code first
    if(pResponseUPIU->BasicHeader.Response != TARGET_SUCCESS)
    {
        return NvBootError_DeviceReadError;
    }

    // Check SCSI Response code
    if(pResponseUPIU->BasicHeader.Status != SCSI_STATUS_GOOD)
    {
        return NvBootError_DeviceReadError;
    }

    NvBootUfsFreeTRDCmdDesc();

    return NvBootError_Success;
}

NvU32 NvBootUfsWrite(const NvU32 Block, const NvU32 Page, const NvU32 Length, NvU32 *pBuffer)
{
    CmdDescriptor_T *pCmdDescriptor;
    CommandUPIU_T *pCommandUPIU;
    ResponseUPIU_T* pResponseUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;
    NvU32 LUNBlock, LUNReady = 0;
    uint16_t numBlocks;

    // First check that LUN is ready for the command.
    while(!LUNReady)
    {
        // Check LUN is ready.
        Error = NvBootUfsTestUnitReady(pUfsContext->BootLun);
        if(Error == NvBootError_Success)
        {
            LUNReady = 1;
        }
        // If LUN Busy, let's try again after a small delay.
        else if(Error == NvBootError_UFSLUNBusy)
        {
            NvBootUtilWaitUS(10);
        }
        else if(Error == NvBootError_UFSLUNCheckCondition)
        {
            // Issue Request Sense command to clear the Check Condition
            Error = NvBootUfsRequestSense(pUfsContext->BootLun);
            if(Error == NvBootError_UFSLUNBusy || Error == NvBootError_Success)
                continue;
            else // Even Check Condition is a fatal error for Request Sense
                return Error;
        }
        else // Fatal error
            return Error;
    }

    // UFS Logical Units has blocks which in bootrom context is pages.
    LUNBlock = ((Block << (pUfsContext->BlockSizeLog2 - pUfsContext->PageSizeLog2)) + Page );
    // Read whole number of blocks.
    numBlocks = CEIL_PAGE(Length,1<<pUfsContext->PageSizeLog2);

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

    pCommandUPIU = (CommandUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;

    /// Program UPIU Header
    pCommandUPIU->BasicHeader.TransCode = UPIU_COMMAND_TRANSACTION;
    // The flag indicates the direction of DATA UPIU. Only relevant for Command UPIU
    pCommandUPIU->BasicHeader.Flags = 1 << UFS_UPIU_FLAGS_W_SHIFT;
    pCommandUPIU->BasicHeader.CmdSetType = UPIU_COMMAND_SET_SCSI;
    pCommandUPIU->BasicHeader.LUN = pUfsContext->BootLun;
    pCommandUPIU->ExpectedDataTxLenBigE = BYTE_SWAP32( numBlocks*(1<<pUfsContext->PageSizeLog2));
    
    /* Construct CDB for WRITE(10) command */
    pCommandUPIU->CDB[0] = 0x2A;
    /* Byte 1-3 are used for LBA in Big endian format */
    pCommandUPIU->CDB[5] = (LUNBlock & 0xFF); // LSB
    pCommandUPIU->CDB[4] = (LUNBlock >> 8) & 0xFF;
    pCommandUPIU->CDB[3] = (LUNBlock >> 16) & 0xFF;
    pCommandUPIU->CDB[2] = (LUNBlock >> 24) & 0xFF; //MSB
    /* Fill in transfer length in num of blocks*/
    pCommandUPIU->CDB[7] = (numBlocks >> 8) & 0xFF; // MSB
    pCommandUPIU->CDB[8] = numBlocks &0xff; // LSB
    /* Fill in control = 0x00 */
    pCommandUPIU->CDB[9] = 0;

    /* 
     * DW0 Data Base Address Lower bits DWORD aligned.
     * DW1 Data Base Address Upper bits: set to 0. 64 bit Not supported.
     * DW3 Data byte count. 0 based value. Minimum 4 bytes i.e. 3
     */
    pCmdDescriptor->PRDT.DW0 = (NvU32)pBuffer & ~(0x3);
    pCmdDescriptor->PRDT.DW1 = 0;
    // LSB[1:0] are always 0x3 to indicate dword granularity
    pCmdDescriptor->PRDT.DW3 = (numBlocks*(1<<pUfsContext->PageSizeLog2))-1;
    
    
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_H2D);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for SCSI READ*/
    Error = NvBootUfsQueueTRD(TRDIndex, SCSI_REQ_READ_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, SCSI_REQ_READ_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    // Analyze Response to see if read is ok.
    pResponseUPIU = (ResponseUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;
#define TARGET_SUCCESS 0x0
#define TARGET_FAILURE 0x1

    // Check UFS Response code first
    if(pResponseUPIU->BasicHeader.Response != TARGET_SUCCESS)
    {
        return NvBootError_DeviceReadError;
    }

    // Check SCSI Response code
    if(pResponseUPIU->BasicHeader.Status != SCSI_STATUS_GOOD)
    {
        return NvBootError_DeviceReadError;
    }


    NvBootUfsFreeTRDCmdDesc();

    return NvBootError_Success;
}

void
NvBootUfsGetParams(
    const NvU32 ParamIndex,
    NvBootUfsParams **Params)
{
    // Active Lanes: 0 based fuses. Adjust accordingly. No Need to adjust params from BCT
    s_UfsParams.ActiveLanes = NV_DRF_VAL(UFS, FUSE_PARAMS, ACTIVE_LANES, ParamIndex)+1; 
    // Gear: 0 based gear. Adjust accordingly. No Need to adjust params from BCT
    s_UfsParams.PWMGear = NV_DRF_VAL(UFS, FUSE_PARAMS, SPEED, ParamIndex)+1;

    // These params can only be set from fuse.
    s_UfsInternalParams.PageSizeLog2 = UFS_PAGE_SIZE_LOG2 + NV_DRF_VAL(UFS, FUSE_PARAMS, PAGE_SIZE, ParamIndex);
    s_UfsInternalParams.BootLun = NV_DRF_VAL(UFS, FUSE_PARAMS, LUN, ParamIndex);
    s_UfsInternalParams.BootEnabled = NV_DRF_VAL(UFS, FUSE_PARAMS, BOOT_ENABLE, ParamIndex);
    s_UfsInternalParams.NumLanes = NV_DRF_VAL(UFS, FUSE_PARAMS, NUM_LANES, ParamIndex)+1;

    *Params = &s_UfsParams;
}


NvBool
NvBootUfsValidateParams(
    const NvBootUfsParams *Params)
{
    // Check num Lanes. 1 <= ActiveLanes <=2
    if(Params->ActiveLanes > 2 || Params->ActiveLanes < 1)
        return NV_FALSE;

    // Check gears 1 <= Gears <=4
    if(Params->PWMGear > 4 || Params->PWMGear < 1)
        return NV_FALSE;

    return NV_TRUE;
}

void
NvBootUfsGetBlockSizes(
    const NvBootUfsParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    (void)Params;
    *BlockSizeLog2 = pUfsContext->BlockSizeLog2; // Hardcoded. 128 KB
    *PageSizeLog2 = pUfsContext->PageSizeLog2; // From fuse. Default 4K
}

NvBootDeviceStatus NvBootUfsQueryStatus(void)
{
    // UFS reads are blocking reads so this function always returns idle.
    return NvBootDeviceStatus_Idle;
}

NvBootError NvBootUfsGetReaderBuffersBase(uint8_t** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    // WHY!!!
    (void)ReaderBuffersBase;
    (void)Alignment;
    (void)Bytes;

    return NvBootError_Success;
}

void NvBootUfsShutdown(void)
{
    // Implement
}

NvBootDeviceStatus NvBootUfsPinMuxInit(const void *Params)
{
    NvU32 RegData;

    (void)Params;
    // Default state of these sideband signals are PASSTHROUGH.
    // But Siva recommends to configure it anyway.
    RegData = NV_READ32(NV_ADDRESS_MAP_PADCTL_A17_BASE + PADCTL_UFS_UFS0_REF_CLK_0);
    RegData = NV_FLD_SET_DRF_DEF(PADCTL_UFS, UFS0_REF_CLK, TRISTATE, PASSTHROUGH, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A17_BASE + PADCTL_UFS_UFS0_REF_CLK_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_PADCTL_A17_BASE + PADCTL_UFS_UFS0_RST_0);
    RegData = NV_FLD_SET_DRF_DEF(PADCTL_UFS, UFS0_RST, TRISTATE, PASSTHROUGH, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A17_BASE + PADCTL_UFS_UFS0_RST_0, RegData);

    return NvBootError_Success;
}

NvBootError NvBootUfsChangeNumLanes(NvU32 ActiveLanes)
{
    NvU32 Data;
    NvBootError e = NvBootError_Success;
    // Change Active Data Lanes
    Data = ActiveLanes; e = NvBootSetDMECommand(DME_SET, 0, PA_ActiveTxDataLanes, &Data); // 0x1560
    if(e != NvBootError_Success)
        return e;

    Data = ActiveLanes; e= NvBootSetDMECommand(DME_SET, 0, PA_ActiveRxDataLanes, &Data); // 0x1580
    if(e!= NvBootError_Success)
        return e;

    // Do slow mode. Todo OR SLOW AUTO MODE?   (PMRX 7:4 PMTX 3:0)
    Data =  ((PWRMODE_SLOW_MODE<<4)|PWRMODE_SLOW_MODE); NvBootSetDMECommand(DME_SET, 0, PA_PWRMode, &Data); // 0x1571
    if(e!= NvBootError_Success)
        return e;

    return e;
}

NvBootError NvBootUfsChangeGear(NvU32 Gear)
{
    NvU32 Data;
    NvBootError e = NvBootError_Success;

    // Change Gears
    // Do we need to add lanes for DME commands. i.e. gensel
    Data = Gear; e = NvBootSetDMECommand(DME_SET, 0, PA_TxGear, &Data); // 0x1568
    if(e!= NvBootError_Success)
        return e;

    Data = Gear; e = NvBootSetDMECommand(DME_SET, 0, PA_RxGear, &Data); // 0x1583
    if(e!= NvBootError_Success)
        return e;

    // After initializing Layer 1.5, the Layer 2 timer values for the local device and the peer device applied after the
    // successful power mode change need to be programmed by accessing:
    NvBootUfsSetTimerThreshold(Gear, pUfsContext->ActiveLanes);

    // Do slow mode. Todo OR SLOW AUTO MODE?   (PMRX 7:4 PMTX 3:0)
    Data =  ((PWRMODE_SLOW_MODE<<4)|PWRMODE_SLOW_MODE); NvBootSetDMECommand(DME_SET, 0, PA_PWRMode, &Data); // 0x1571
    if(e!= NvBootError_Success)
        return e;

    return e;
}

// Set timer and threshold values for different gears.
NvBootError NvBootUfsSetTimerThreshold(NvU32 Gear, NvU32 ActiveLanes)
{
    NvBootError e;
    NvU32 Data;
    // Adjust gear for 0 based index.
    // Note: We always keep Rx and Tx lanes at same gear so same timer threshold
    // values can be applied for both local and peer.
    if(ActiveLanes == 2)
    {
        // FC0ProtectionTimeOutVal Local followed by Peer
        Data = DME_FC0ProtectionTimeOutVal_x2[Gear-1]; e = NvBootSetDMECommand(DME_SET, 0, DME_FC0ProtectionTimeOutVal, &Data);
        if(e != NvBootError_Success)
            return e;
        e = NvBootSetDMECommand(DME_SET, 0, PWRModeUserData0, &Data);
        if(e != NvBootError_Success)
            return e;

        // TC0ReplayTimeOutVal Local followed by Peer
        Data = DME_TC0ReplayTimeOutVal_x2[Gear-1]; e = NvBootSetDMECommand(DME_SET, 0, DME_TC0ReplayTimeOutVal, &Data);
        if(e!= NvBootError_Success)
            return e;
        e = NvBootSetDMECommand(DME_SET, 0, PWRModeUserData1, &Data);
        if(e != NvBootError_Success)
            return e;

        // AFC0ReqTimeOutVal Local followed by Peer
        Data = DME_AFC0ReqTimeOutVal_x2[Gear-1]; e = NvBootSetDMECommand(DME_SET, 0, DME_AFC0ReqTimeOutVal, &Data);
        if(e != NvBootError_Success)
            return e;
        e = NvBootSetDMECommand(DME_SET, 0, PWRModeUserData2, &Data);
        if(e != NvBootError_Success)
            return e;
    }
    else
    {
        // FC0ProtectionTimeOutVal Local followed by Peer
        Data = DME_FC0ProtectionTimeOutVal_x1[Gear-1]; e = NvBootSetDMECommand(DME_SET, 0, DME_FC0ProtectionTimeOutVal, &Data);
        if(e != NvBootError_Success)
            return e;
        e = NvBootSetDMECommand(DME_SET, 0, PWRModeUserData0, &Data);
        if(e != NvBootError_Success)
            return e;

        // TC0ReplayTimeOutVal Local followed by Peer
        Data = DME_TC0ReplayTimeOutVal_x1[Gear-1]; e = NvBootSetDMECommand(DME_SET, 0, DME_TC0ReplayTimeOutVal, &Data);
        if(e!= NvBootError_Success)
            return e;
        e = NvBootSetDMECommand(DME_SET, 0, PWRModeUserData1, &Data);
        if(e != NvBootError_Success)
            return e;

        // AFC0ReqTimeOutVal Local followed by Peer
        Data = DME_AFC0ReqTimeOutVal_x1[Gear-1]; e = NvBootSetDMECommand(DME_SET, 0, DME_AFC0ReqTimeOutVal, &Data);
        if(e != NvBootError_Success)
            return e;
        e = NvBootSetDMECommand(DME_SET, 0, PWRModeUserData2, &Data);
        if(e != NvBootError_Success)
            return e;
    }
    return NvBootError_Success;
}

/* This is used to test if the logical unit is ready before issuing a read/write command */
NvBootError NvBootUfsTestUnitReady(NvU32 LUN)
{
    CmdDescriptor_T *pCmdDescriptor;
    CommandUPIU_T *pCommandUPIU;
    ResponseUPIU_T* pResponseUPIU;

    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;
    Error = NvBootError_Success;

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

    pCommandUPIU = (CommandUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;

    /// Program UPIU Header
    pCommandUPIU->BasicHeader.TransCode = UPIU_COMMAND_TRANSACTION;
    // Flag is not applicable as there is no DATA UPIU involved.
    pCommandUPIU->BasicHeader.LUN = LUN;
    pCommandUPIU->BasicHeader.CmdSetType = UPIU_COMMAND_SET_SCSI;
    // Data segment = 0  as there is no data segment in the UPIU.
    // Expected Data length = 0 as there is no DATA UPIU involved.
    pCommandUPIU->ExpectedDataTxLenBigE = 0;

    /* Construct CDB for Test Unit Ready command */
    pCommandUPIU->CDB[0] = 0x0;
    
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for SCSI Cmds*/
    Error = NvBootUfsQueueTRD(TRDIndex, REQUEST_SENSE_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;

    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, REQUEST_SENSE_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    // Check if command succeeded.
    pResponseUPIU = (ResponseUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;

    if(pResponseUPIU->BasicHeader.Status == SCSI_STATUS_GOOD)
    {
        Error = NvBootError_Success;
    }
    else if(pResponseUPIU->BasicHeader.Status == SCSI_STATUS_BUSY)
    {
        Error = NvBootError_UFSLUNBusy;
    }
    else if(pResponseUPIU->BasicHeader.Status == SCSI_STATUS_CHECK_CONDITION)
    {
        Error = NvBootError_UFSLUNCheckCondition;
    }
    else
        Error = NvBootError_UFSUnknownSCSIStatus;
    
    NvBootUfsFreeTRDCmdDesc();

    return Error;
}

/* This is used to request sense data. */
NvBootError NvBootUfsRequestSense(NvU32 LUN)
{
    CmdDescriptor_T *pCmdDescriptor;
    CommandUPIU_T *pCommandUPIU;
    ResponseUPIU_T* pResponseUPIU;

    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;
    Error = NvBootError_Success;

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

    pCommandUPIU = (CommandUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;

    /// Program UPIU Header
    pCommandUPIU->BasicHeader.TransCode = UPIU_COMMAND_TRANSACTION;
    // Flag is not applicable as there is no DATA UPIU involved.
    pCommandUPIU->BasicHeader.LUN = LUN;
    pCommandUPIU->BasicHeader.CmdSetType = UPIU_COMMAND_SET_SCSI;
    // Data segment = 0  as there is no data segment in the UPIU.
    // Expected Data length = 0 as there is no DATA UPIU involved.
    pCommandUPIU->ExpectedDataTxLenBigE = 0;

    /* Construct CDB for Request Sense command */
    pCommandUPIU->CDB[0] = 0x3;
    // Allocation length is left 0 as we don't need the sense data and this is 
    // not considered an error.
    pCommandUPIU->CDB[4] = 0x0;
    
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for SCSI Cmds*/
    Error = NvBootUfsQueueTRD(TRDIndex, REQUEST_SENSE_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;

    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, REQUEST_SENSE_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    // Check if command succeeded.
    pResponseUPIU = (ResponseUPIU_T*)&pCmdDescriptor->UCDGenericRespUPIU;

    if(pResponseUPIU->BasicHeader.Status == SCSI_STATUS_GOOD)
    {
        Error = NvBootError_Success;
    }
    else if(pResponseUPIU->BasicHeader.Status == SCSI_STATUS_BUSY)
    {
        Error = NvBootError_UFSLUNBusy;
    }
    else if(pResponseUPIU->BasicHeader.Status == SCSI_STATUS_CHECK_CONDITION)
    {
        Error = NvBootError_UFSLUNCheckCondition;
    }
    else
        Error = NvBootError_UFSUnknownSCSIStatus;
    
    NvBootUfsFreeTRDCmdDesc();

    return Error;
}

/** Get Descriptor to identify device characteristics
 */
NvU32 NvBootUfsWriteDescriptor(uint8_t *pUFSDesc, NvU32 DescIDN, NvU32 DescIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    
    QueryReqRespUPIU_T *pQueryReqUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUfsGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUfsGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    NvBootUtilMemset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqUPIU->BasicHeader.TransCode = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_FUNC_STD_WRITE;
    pQueryReqUPIU->BasicHeader.DataSegLenBigE = BYTE_SWAP16(pUFSDesc[0]);
    pQueryReqUPIU->TSF.Opcode = TSF_OPCODE_WRITE_DESC;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = QUERY_DESC_DEVICE_DESC_IDN;
    pQueryReqUPIU->TSF.DescFields.DescIDN = DescIDN;
    /* Index */
    pQueryReqUPIU->TSF.DescFields.Index = DescIndex;
    /** TODO Find out device descriptor size. It is ok to simply specify max desc size
     * of 255 bytes as device will return actual descriptor size if lesser.
     */
    pQueryReqUPIU->TSF.DescFields.LengthBigE = BYTE_SWAP16(pUFSDesc[0]);

    NvBootUtilMemcpy((void*)&(pQueryReqUPIU->DataSegment[0]), \
            (void*)pUFSDesc, \
            pUFSDesc[0]); // Byte 0 contains length 
    
    /* TODO Task Tags */
    Error = NvBootUfsCreateTRD(TRDIndex, CmdDescIndex, DATA_DIR_NIL);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ DESCRIPTOR */
    Error = NvBootUfsQueueTRD(TRDIndex, QUERY_REQ_DESC_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for NOP */
    Error = NvBootUfsWaitTRDRequestComplete(TRDIndex, QUERY_REQ_DESC_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;   

    NvBootUfsFreeTRDCmdDesc();
    return NvBootError_Success;
}

void NvBootUfsGetClockTable(void **UfsClockTable, ClockTableType *TableType)
{
    *UfsClockTable= &s_UfsClkTable[0];
    *TableType = TYPE_LIST_TABLES;
}
