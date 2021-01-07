/*
 * Copyright (c) 2006 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

// Don't integrate these to SW folder
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arapb_misc.h"
#include "arapbpm.h"
#include "aremc.h"
#include "armc.h"
#include "nvboot_clocks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_sdram_int.h"
#include "nvboot_sdram_param.h"
#include "nvboot_util_int.h"
#include "project.h"

#define EMC_SPARE_BLACKLIST(_) \
          _(EMC_CCFIFO_ADDR_0) \
          _(EMC_CMD_MAPPING_CMD0_0_0) \
          _(EMC_CMD_MAPPING_CMD0_1_0) \
          _(EMC_CMD_MAPPING_CMD0_2_0) \
          _(EMC_CMD_MAPPING_CMD1_0_0) \
          _(EMC_CMD_MAPPING_CMD1_1_0) \
          _(EMC_CMD_MAPPING_CMD1_2_0) \
          _(EMC_CMD_MAPPING_CMD2_0_0) \
          _(EMC_CMD_MAPPING_CMD2_1_0) \
          _(EMC_CMD_MAPPING_CMD2_2_0) \
          _(EMC_CMD_MAPPING_CMD3_0_0) \
          _(EMC_CMD_MAPPING_CMD3_1_0) \
          _(EMC_CMD_MAPPING_CMD3_2_0) \
          _(EMC_CMD_MAPPING_BYTE_0) \
          _(EMC_ADR_CFG_0) \
          _(EMC_FBIO_CFG8_0) \
          _(EMC_FBIO_SPARE_0) \
          _(EMC_PMACRO_BRICK_MAPPING_0_0) \
          _(EMC_PMACRO_BRICK_MAPPING_1_0) \
          _(EMC_PMACRO_BRICK_MAPPING_2_0)
    
#define MC_SPARE_BLACKLIST(_) \
      _(MC_EMEM_CFG_0) \
      _(MC_EMEM_CFG_ACCESS_CTRL_0) \
      _(MC_EMEM_ADR_CFG_0) \
      _(MC_EMEM_ADR_CFG_DEV0_0) \
      _(MC_EMEM_ADR_CFG_DEV1_0) \
      _(MC_EMEM_ADR_CFG_CHANNEL_MASK_0) \
      _(MC_EMEM_ADR_CFG_BANK_MASK_0_0) \
      _(MC_EMEM_ADR_CFG_BANK_MASK_1_0) \
      _(MC_EMEM_ADR_CFG_BANK_MASK_2_0) \
      _(MC_VIDEO_PROTECT_BOM_0) \
      _(MC_VIDEO_PROTECT_BOM_ADR_HI_0) \
      _(MC_VIDEO_PROTECT_SIZE_MB_0) \
      _(MC_VIDEO_PROTECT_VPR_OVERRIDE_0) \
      _(MC_VIDEO_PROTECT_VPR_OVERRIDE1_0) \
      _(MC_VIDEO_PROTECT_GPU_OVERRIDE_0_0) \
      _(MC_VIDEO_PROTECT_GPU_OVERRIDE_1_0) \
      _(MC_VIDEO_PROTECT_REG_CTRL_0) \
      _(MC_IRAM_BOM_0) \
      _(MC_IRAM_TOM_0) \
      _(MC_IRAM_ADR_HI_0) \
      _(MC_IRAM_REG_CTRL_0) \
      _(MC_TZ_SECURITY_CTRL_0) \
      _(MC_MTS_CARVEOUT_BOM_0) \
      _(MC_MTS_CARVEOUT_ADR_HI_0) \
      _(MC_MTS_CARVEOUT_SIZE_MB_0) \
      _(MC_MTS_CARVEOUT_REG_CTRL_0) \
      _(MC_SEC_CARVEOUT_BOM_0) \
      _(MC_SEC_CARVEOUT_SIZE_MB_0) \
      _(MC_SEC_CARVEOUT_ADR_HI_0) \
      _(MC_SEC_CARVEOUT_REG_CTRL_0) \
      _(MC_SECURITY_CARVEOUT1_CFG0_0) \
      _(MC_SECURITY_CARVEOUT2_CFG0_0) \
      _(MC_SECURITY_CARVEOUT3_CFG0_0) \
      _(MC_SECURITY_CARVEOUT4_CFG0_0) \
      _(MC_SECURITY_CARVEOUT5_CFG0_0) \
      _(MC_UNTRANSLATED_REGION_CHECK_0)

#define CAR_SPARE_WHITELIST(_) \
      _(CLK_RST_CONTROLLER_RST_DEV_H_CLR_0) \
      _(CLK_RST_CONTROLLER_PLLM_BASE_0) \
      _(CLK_RST_CONTROLLER_PLLM_MISC1_0) \
      _(CLK_RST_CONTROLLER_PLLM_MISC2_0) \
      _(CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0) \
      _(CLK_RST_CONTROLLER_CLK_SOURCE_EMC_DLL_0) \
      _(CLK_RST_CONTROLLER_CLK_ENB_W_CLR_0) \
      _(CLK_RST_CONTROLLER_CLK_ENB_W_SET_0) \
      _(CLK_RST_CONTROLLER_CLK_ENB_H_SET_0) \
      _(CLK_RST_CONTROLLER_CLK_ENB_X_SET_0) 

#define PMC_SPARE_WHITELIST(_) \
        _(APBDEV_PMC_VDDP_SEL_0) \
        _(APBDEV_PMC_DDR_CFG_0) \
        _(APBDEV_PMC_IO_DPD3_REQ_0) \
        _(APBDEV_PMC_IO_DPD4_REQ_0) \
        _(APBDEV_PMC_NO_IOPOWER_0) \
        _(APBDEV_PMC_WEAK_BIAS_0) \
        _(APBDEV_PMC_DDR_CNTRL_0) \
        _(APBDEV_PMC_REG_SHORT_0) 


#define MSS_LIST_CASE(r)  case( r ):

// Wrapper Macros for writting/reading into/from MC and EMC registers

/** 
 * HW_REGR - Read a hardware register.
 *
 *   @param d register domain (hardware block)
 *   @param r register name
 */

#define HW_REGR(d, p, r)  NV_READ32(NV_ADDRESS_MAP_##d##_BASE + p##_##r##_0)


/** 
 * HW_REGW - Write a hardware register.
 *
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param v defined value for the register
 */
#ifdef TEMPORARY_DONT_INTEGRATE
#define HW_REGW(b, p, r, v)                             \
do{                                                     \
    if (b & 0x80000000)                                 \
    {                                                   \
        NV_WRITE32((b & 0x7FFFFFFF) + p##_##r##_0, v);  \
        NV_WRITE32((b & 0x7FFFFFFF) -                   \
                   NV_ADDRESS_MAP_##p##0_BASE +         \
                   NV_ADDRESS_MAP_##p##1_BASE +         \
                   p##_##r##_0, v);                     \
    } else                                              \
    {                                                   \
        NV_WRITE32(b + p##_##r##_0, v);                 \
    }                                                   \
} while(0)
#else
#define HW_REGW(b, p, r, v) NV_WRITE32(b + p##_##r##_0, v)
#endif

/**
 * ISSUE_AUTO_REFRESH - Issue an autorefresh command to the SDRAM.
 */
#define ISSUE_AUTO_REFRESH()                                                  \
do {                                                                          \
    HW_REGW(EmcBase, EMC, REF,                                                \
            NV_DRF_NUM(EMC, REF, REF_CMD,    1) | /* Trigger bit */           \
            NV_DRF_NUM(EMC, REF, REF_NORMAL, 0) | /* Send immediately */      \
            NV_DRF_NUM(EMC, REF, REF_NUM,    0) | /* Issue 1 refresh */       \
            NV_DRF_NUM(EMC, REF, REF_DEV_SELECTN, pData->EmcDevSelect));      \
} while(0)

/**
 * PRECHARGE_ALL_BANKS_ALL_DEVICES - Trigger a precharge on all SDRAM devices.
 */
#define PRECHARGE_ALL_BANKS_ALL_DEVICES()                                     \
do {                                                                          \
    /* Precharge all banks. DEV_SELECTN = 0 => Select all devices   */        \
    HW_REGW(EmcBase, EMC, PRE,                                                \
            NV_DRF_NUM(EMC, PRE, PRE_CMD,         1) | /* Trigger precharge*/ \
            NV_DRF_NUM(EMC, PRE, PRE_DEV_SELECTN, pData->EmcDevSelect));      \
} while(0)

void NvBootMssSpareWrite(NvU32 addr, NvU32 data)
{
    NvU32 base;
    NvU32 offset;
    if (!(addr))
       return;
    if ((addr >= NV_ADDRESS_MAP_EMC_BASE) && (addr <= NV_ADDRESS_MAP_EMC_LIMIT)) {
       base = NV_ADDRESS_MAP_EMC_BASE;
       offset = addr - base;
       switch (offset) {
          EMC_SPARE_BLACKLIST(MSS_LIST_CASE)
            return;
            break;
          default:
            NV_WRITE32(addr, data);
            break;
       }
    }
    if ((addr >= NV_ADDRESS_MAP_MC_BASE) && (addr <= NV_ADDRESS_MAP_MC_LIMIT)) {
       base = NV_ADDRESS_MAP_MC_BASE;
       offset = addr - base;
       switch (offset) {
          MC_SPARE_BLACKLIST(MSS_LIST_CASE)
            return;
            break;
          default:
            NV_WRITE32(addr, data);
            break;
       }
    }
    if ((addr >= NV_ADDRESS_MAP_APB_PMC_BASE)  && (addr <= NV_ADDRESS_MAP_APB_PMC_LIMIT)) {
       base = NV_ADDRESS_MAP_APB_PMC_BASE;
       offset = addr - base;
       switch (offset) {
          PMC_SPARE_WHITELIST(MSS_LIST_CASE)
            NV_WRITE32(addr, data);
            break;
          default:
            return;
            break;
       }
    }
    if ((addr >= NV_ADDRESS_MAP_CLK_RST_BASE)  && (addr <= NV_ADDRESS_MAP_CLK_RST_LIMIT)) {
       base = NV_ADDRESS_MAP_APB_PMC_BASE;
       offset = addr - base;
       switch (offset) {
          CAR_SPARE_WHITELIST(MSS_LIST_CASE)
            NV_WRITE32(addr, data);
            break;
          default:
            return;
            break;
       }
    }
}

#ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
static void InitDdr(const NvBootSdramParams *pData, NvU32 EmcBase)
{
    PRECHARGE_ALL_BANKS_ALL_DEVICES();
    
    // Issue an Extended Mode Register Set (EMRS) command to intialize the mode
    // register.
    HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);

    // Issue a Mode Register Set (MRS) command to intialize the mode resgister.
    HW_REGW(EmcBase, EMC, MRS, pData->EmcMrsResetDll);

    // Wait 200 clock cycles after resetting DLL.  The lowest speed grade for
    // ddr is 100MHz, so 3us gives adequate margin.
    NvBootUtilWaitUS(pData->EmcMrsResetDllWait);
    
    PRECHARGE_ALL_BANKS_ALL_DEVICES();
    
    // Issue Auto Refresh commands. Must be done twice.
    ISSUE_AUTO_REFRESH();
    ISSUE_AUTO_REFRESH();

    // Issue MRS command to intialize the mode register.
    HW_REGW(EmcBase, EMC, MRS, pData->EmcMrs);
    // Patch 6 using BCT Spare Variables
    if(pData->EmcBctSpare10)
       NvBootMssSpareWrite(pData->EmcBctSpare10, pData->EmcBctSpare11); 
}

static void InitLpDdr(const NvBootSdramParams *pData, NvU32 EmcBase)
{
    PRECHARGE_ALL_BANKS_ALL_DEVICES();
    
    // Issue Auto Refresh commands. Must be done twice.
    ISSUE_AUTO_REFRESH();
    ISSUE_AUTO_REFRESH();
    
    // Issue MRS and EMRS commands to intialize the mode resgisters.
    HW_REGW(EmcBase, EMC, MRS,  pData->EmcMrs );
    HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);
    // Patch 6 using BCT Spare Variables
    if(pData->EmcBctSpare10)
       NvBootMssSpareWrite(pData->EmcBctSpare10, pData->EmcBctSpare11); 
}
#endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES

// Note - InitLpDdr2 is also used for LPDDR3
static void InitLpDdr2(const NvBootSdramParams *pData, NvU32 EmcBase)
{
    // Assume that voltage and clocks are supplied to EMC and all are stable.
    
    PRECHARGE_ALL_BANKS_ALL_DEVICES();

    // Send Reset MRW command.
    HW_REGW(EmcBase, EMC, MRW, pData->EmcMrwResetCommand);
    NvBootUtilWaitUS(pData->EmcMrwResetNInitWait);

    if (NV_DRF_VAL(EMCZCAL, BOOT_ENABLE, COLDBOOT,pData->EmcZcalWarmColdBootEnables)) { //    if (pData->EmcZcalColdBootEnable) {
        // ZQ Initialization for device 0
        HW_REGW(EmcBase, EMC, MRW, pData->EmcZcalInitDev0);
        NvBootUtilWaitUS(pData->EmcZcalInitWait);

        if ((pData->EmcDevSelect & 2) == 0)
        {
            // ZQ Initialization for device 1
            HW_REGW(EmcBase, EMC, MRW, pData->EmcZcalInitDev1);
            NvBootUtilWaitUS(pData->EmcZcalInitWait);
        }
    }

    // Program mode registers
    HW_REGW(EmcBase, EMC, MRW2, pData->EmcMrw2);
    HW_REGW(EmcBase, EMC, MRW, pData->EmcMrw1);
    HW_REGW(EmcBase, EMC, MRW3, pData->EmcMrw3);
    HW_REGW(EmcBase, EMC, MRW4, pData->EmcMrw4);
    // Patch 6 using BCT Spare Variables
    if(pData->EmcBctSpare10)
       NvBootMssSpareWrite(pData->EmcBctSpare10, pData->EmcBctSpare11); 

    if (pData->EmcExtraModeRegWriteEnable)
    {
        HW_REGW(EmcBase, EMC, MRW, pData->EmcMrwExtra);
    }
    // For LPDDR3, CA training is done in microboot
}

#ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
static void InitDdr2(const NvBootSdramParams *pData, NvU32 EmcBase)
{
    NvBootUtilWaitUS(1);    // The specified delay is 400ns.

    PRECHARGE_ALL_BANKS_ALL_DEVICES();

    // Issue EMRS commands to intialize the mode registers
    HW_REGW(EmcBase, EMC, EMRS2, pData->EmcEmrs2);
    HW_REGW(EmcBase, EMC, EMRS3, pData->EmcEmrs3);
    HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrsDdr2DllEnable);

    // Issue MRS command to intialize the mode register
    HW_REGW(EmcBase, EMC, MRS,  pData->EmcMrsDdr2DllReset);

    PRECHARGE_ALL_BANKS_ALL_DEVICES();
    
    // Issue Auto Refresh commands. Must be done twice.
    ISSUE_AUTO_REFRESH();
    ISSUE_AUTO_REFRESH();
    
    // Issue MRS command to intialize the mode register
    HW_REGW(EmcBase, EMC, MRS, pData->EmcMrs);

    // Add a delay to space out EMC_CFG which enables the active-powerdown
    // that may cause CKE to go low. The datasheet says CKE must be high for
    // at least 200 cycles after Rest-dll. Since ddr2 runs faster than 100MHz,
    // 2us is adequate.
    NvBootUtilWaitUS(pData->EmcDdr2Wait);

    // Enter default OCD calibration 
    HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrsDdr2OcdCalib);

    // Exit OCD calibration
    HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);
    // Patch 6 using BCT Spare Variables
    if(pData->EmcBctSpare10)
       NvBootMssSpareWrite(pData->EmcBctSpare10, pData->EmcBctSpare11); 
}
#endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES

static void InitDdr3(const NvBootSdramParams *pData, NvU32 EmcBase)
{
    HW_REGW(EmcBase, EMC, EMRS2, pData->EmcEmrs2);
    HW_REGW(EmcBase, EMC, EMRS3, pData->EmcEmrs3);
    HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);
    HW_REGW(EmcBase, EMC, MRS, pData->EmcMrs); // resets DLL

    // Patch 6 using BCT Spare Variables
    if(pData->EmcBctSpare10)
       NvBootMssSpareWrite(pData->EmcBctSpare10, pData->EmcBctSpare11); 

    if (pData->EmcExtraModeRegWriteEnable)
    {
        HW_REGW(EmcBase, EMC, MRS, pData->EmcMrsExtra);
    }

    if(NV_DRF_VAL(EMCZCAL, BOOT_ENABLE, COLDBOOT,pData->EmcZcalWarmColdBootEnables))  // if (pData->EmcZcalColdBootEnable)
    {
        HW_REGW(EmcBase, EMC, ZQ_CAL, pData->EmcZcalInitDev0);
        NvBootUtilWaitUS(pData->EmcZcalInitWait);
    
        if ((pData->EmcDevSelect & 2) == 0)
        {
            HW_REGW(EmcBase, EMC, ZQ_CAL, pData->EmcZcalInitDev1);
            NvBootUtilWaitUS(pData->EmcZcalInitWait);
        }
    } else {
        // EmcZcalInitWait is used for DLL stablization time even without ZCAL
        NvBootUtilWaitUS(pData->EmcZcalInitWait);
    }
}

static void InitLpDdr4(const NvBootSdramParams *pData, NvU32 EmcBase)
{
    // Patch 6 using BCT Spare Variables
    if(pData->EmcBctSpare10)
       NvBootMssSpareWrite(pData->EmcBctSpare10, pData->EmcBctSpare11); 
    HW_REGW(EmcBase, EMC, MRW2, pData->EmcMrw2);
    HW_REGW(EmcBase, EMC, MRW, pData->EmcMrw1);
    HW_REGW(EmcBase, EMC, MRW3, pData->EmcMrw3);
    HW_REGW(EmcBase, EMC, MRW4, pData->EmcMrw4);
    HW_REGW(EmcBase, EMC, MRW6, pData->EmcMrw6);
    HW_REGW(EmcBase, EMC, MRW14, pData->EmcMrw14);


    HW_REGW(EmcBase, EMC, MRW8, pData->EmcMrw8);
    HW_REGW(EmcBase, EMC, MRW12, pData->EmcMrw12);
    HW_REGW(EmcBase, EMC, MRW9, pData->EmcMrw9);
    HW_REGW(EmcBase, EMC, MRW13, pData->EmcMrw13);
    
    if(NV_DRF_VAL(EMCZCAL, BOOT_ENABLE, COLDBOOT,pData->EmcZcalWarmColdBootEnables))  // if (pData->EmcZcalColdBootEnable)
    {
        HW_REGW(EmcBase, EMC, ZQ_CAL, pData->EmcZcalInitDev0); // issue ZQCAL start
        NvBootUtilWaitUS(pData->EmcZcalInitWait);
        HW_REGW(EmcBase, EMC, ZQ_CAL, (pData->EmcZcalInitDev0 ^ 0x3)); //issue ZQCAL latch 
    
        if ((pData->EmcDevSelect & 2) == 0)
        {
            HW_REGW(EmcBase, EMC, ZQ_CAL, pData->EmcZcalInitDev1);
            NvBootUtilWaitUS(pData->EmcZcalInitWait);
            HW_REGW(EmcBase, EMC, ZQ_CAL, (pData->EmcZcalInitDev1 ^ 0x3)); //issue ZQCAL latch 
        }
    }

 
}



void NvBootEnableMemClk (const NvBootSdramParams *pData, NvBool preserveDram)
{
    // Function: 
    //  -Brings IOBRICK's out of DPD
    //  -Enables all MSS clocks and ROC clock
    //  -deasserts MSS resets

    // Prerequisites:
    // IO rails should be stable before this function is called
    // For warmBoot (preserveDram ==1):
    //  -*pData: unpacking of bct from PMC space needs to have been executed
    //  -IOBRICK's and CMD/RESET pads held in DPD, either via DPD_REQ or RAMDUMP FSM
    // For other boot variants (preserveDram == 0)
    //   -no assumptions made on pad DPD state.  proper DPD exit sequence will be followed
    //   *pData not required

    // preserveDram == 1: requires *pData pointer to BCT settings unpacked from PMC.  Required for warmBoot
    // preserveDram == 0: *pData is not used.  DRAM contents lost 

    NvU32 dpd4_val;
    NvU32 dpd4_val_e_dpd;
    NvU32 dpd3_val;
    NvU32 dpd3_wait;
    NvU32 dpd4_wait;
    NvU32 wb_val;
    dpd3_wait   = 1;
    dpd4_wait   = 2;

    //BCT data is only available if stored in PMC
    if (preserveDram) {
        //NV_ASSERT(pData);
        dpd4_wait   = pData->PmcIoDpd4ReqWait;
        dpd3_wait   = pData->PmcIoDpd3ReqWait;
        //ASSERT SEL_DPD_CMD and E_DPD for DDR_RESET (to properly control levels during SELF-REF EXIT
        dpd4_val    = (pData->EmcPmcScratch2 & 0x3FFFFFFF) | NV_DRF_DEF(APBDEV_PMC, IO_DPD4_REQ, CODE, DPD_OFF);
        dpd3_val    = (pData->EmcPmcScratch1 & 0x3FFFFFFF) | NV_DRF_DEF(APBDEV_PMC, IO_DPD3_REQ, CODE, DPD_OFF);

        wb_val      = ((pData->EmcPmcScratch1 ^ 0x00000FFF) & 0x00000FFF) << 18 ; // setting WB only for bricks
        wb_val =      wb_val      | ((0x00001000 & ~pData->EmcPmcScratch1)<< 19) | ((0x00008000 & ~pData->EmcPmcScratch1)<< 15);  // setting WB for comp and dll if side-A is not used
    } else {
        //For L1 reset case (non-ramdump), assert SEL_DPD_CMD and RESET E_DPD/SEL_DPD
        dpd3_val    = 0x3FFF0000 & NV_RESETVAL(APBDEV_PMC, IO_DPD3_REQ);
        dpd3_val   |= NV_DRF_DEF(APBDEV_PMC, IO_DPD3_REQ, CODE, DPD_ON);

        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD3_REQ_0, dpd3_val);

        dpd3_val = 0x0000FFFF |
                   NV_DRF_DEF(APBDEV_PMC, IO_DPD3_REQ, CODE, DPD_OFF);

        dpd4_val = 0x3FFFFFFF |  NV_DRF_DEF(APBDEV_PMC, IO_DPD4_REQ, CODE, DPD_OFF);
        wb_val      = 0;
        
    }

    // disable e_dpd_bg 
    dpd4_val_e_dpd = dpd4_val & 0xC000FFFF;
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD4_REQ_0, dpd4_val_e_dpd);
    NvBootPmcUtilWaitUS(dpd4_wait);

    // disable e_dpd_vttgen
    dpd4_val_e_dpd = dpd4_val & 0xFFFF0000;
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD4_REQ_0, dpd4_val_e_dpd);
    NvBootPmcUtilWaitUS(dpd4_wait);

    // disable sel_dpd
    dpd3_val &= 0xC000FFFF;
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD3_REQ_0, dpd3_val);
    NvBootPmcUtilWaitUS(dpd3_wait);

    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_WEAK_BIAS_0, wb_val);

    NvBootPmcUtilWaitUS(1);

    // Enable the clocks for EMC, MC, and ROCj
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_H_SET_0,
          NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_ENB_H_SET, SET_CLK_ENB_MEM, 1) |
          NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_ENB_H_SET, SET_CLK_ENB_EMC, 1));
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_W_SET_0,
          NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_ENB_W_SET, SET_CLK_ENB_MC1, 1));

    NvBootPmcUtilWaitUS(1); // add a wait to make sure we give enough time for reset deassert to propagate

    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_H_CLR_0,CLK_RST_CONTROLLER_RST_DEV_H_CLR_0_CLR_EMC_RST_FIELD | CLK_RST_CONTROLLER_RST_DEV_H_CLR_0_CLR_MEM_RST_FIELD);

    NvBootPmcUtilWaitUS(5); // add a wait to make sure we give enough time for clk_switch to take place. 

}


static void DoSdramInit(const NvBootSdramParams *pData, NvBool IsWarmboot)
{
    // PLLM should be stable before this function is called
    NvU32 RegData;
    NvU32 TempData;
    NvU32 wb_enables; 
    NvU32 ddrcntrl;
    NvU32 dpd4_val;
    NvU32 dpd3_val;
    NvU32 dpd3_val_sel_dpd;
    NvU32 Reg, Fld;
    const NvU32 EmcBase =
        NV_ADDRESS_MAP_EMC_BASE;

    const NvU32 McBase =
        NV_ADDRESS_MAP_MC_BASE;

    NV_ASSERT(pData);

    if(!IsWarmboot) {
        // Set VDDP select
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_VDDP_SEL_0, pData->PmcVddpSel);
        NvBootUtilWaitUS(pData->PmcVddpSelWait);

        #define COPY_PMC_FLD(reg, fld, var) \
        do {\
            Fld = NV_DRF_VAL(APBDEV_PMC, reg, fld, pData->var); \
            Reg = NV_FLD_SET_DRF_NUM(APBDEV_PMC, reg, fld, Fld, Reg); \
        } while(0)

        // Turn on MEM IO power
        Reg = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_NO_IOPOWER_0);
        COPY_PMC_FLD(NO_IOPOWER, MEM, PmcNoIoPower); // make sure mem power is on
        COPY_PMC_FLD(NO_IOPOWER, MEM_COMP, PmcNoIoPower); // make sure mem power is on
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_NO_IOPOWER_0, Reg);

        //Clear TX_FORCE_TRISTATE
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_CNTRL_0, pData->PmcDdrCntrl);
    }

    // Patch 1 using BCT Spare Variables
    if(pData->EmcBctSpare0)
      NvBootMssSpareWrite(pData->EmcBctSpare0 , pData->EmcBctSpare1);

    if (pData->ClkRstControllerPllmMisc2OverrideEnable) {
      NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLM_MISC2_0, pData->ClkRstControllerPllmMisc2Override);
    }


    // use this to enable the E_WB on powered down BRICKS based of the sel_dpd config
    wb_enables = ((pData->EmcPmcScratch1 ^ 0x00000FFF) & 0x00000FFF) << 18 ; // setting WB only for bricks

    wb_enables = wb_enables | ((0x00001000 & ~pData->EmcPmcScratch1)<< 19) | ((0x00008000 & ~pData->EmcPmcScratch1)<< 15); // setting WB for comp and dll if side-A is not used
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_WEAK_BIAS_0, wb_enables);

    // put unused bricks into DPD
    dpd3_val = NV_FLD_SET_DRF_DEF(APBDEV_PMC, IO_DPD3_REQ, CODE, DPD_ON,  (0x00009FFF & ~pData->EmcPmcScratch1));
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD3_REQ_0, dpd3_val);

    NvBootPmcUtilWaitUS(pData->PmcIoDpd3ReqWait);

    // Set e_dpd_vttgen on all unused bricks 
    dpd4_val = NV_FLD_SET_DRF_DEF(APBDEV_PMC, IO_DPD4_REQ, CODE, DPD_ON,  (0x3FFF0000 & ~pData->EmcPmcScratch2));
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD4_REQ_0, dpd4_val);

    NvBootPmcUtilWaitUS(pData->PmcIoDpd4ReqWait);
    // Set e_dpd_bg on unused Side
    dpd4_val = NV_FLD_SET_DRF_DEF(APBDEV_PMC, IO_DPD4_REQ, CODE, DPD_ON,  (0x00001FFF & ~pData->EmcPmcScratch2));
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD4_REQ_0, dpd4_val);

    NvBootUtilWaitUS(1);

    // Enable the clocks for EMC and MC
    // T214 - MH - shouldn't be needed, but leave in case not all TB's use EnableMemClk -- HAS NO IMPACT
    NvBootClocksSetEnable(NvBootClocksClockId_EmcId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_McId,  NV_TRUE);

    if (NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_EMC, EMC_2X_CLK_SRC, pData->EmcClockSource) != CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_UD) {
      NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_X_SET_0, CLK_RST_CONTROLLER_CLK_ENB_X_SET_0_SET_CLK_ENB_EMC_DLL_FIELD);
    }

    // Disable clk to unused channel
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_W_CLR_0, NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_ENB_W_CLR, CLR_CLK_ENB_MC1, pData->ClearClk2Mc1));

    // Remove the EMC and MC controllers from Reset
    // T214 - MH - shouldn't be needed, but leave in case not all TB's use EnableMemClk -- HAS NO IMPACT
    NvBootResetSetEnable(NvBootResetDeviceId_EmcId, NV_FALSE);
    NvBootResetSetEnable(NvBootResetDeviceId_McId,  NV_FALSE);

    //   NvBootResetSetEnable(NvBootResetDeviceId_EmcId, NV_FALSE);
    //   NvBootResetSetEnable(NvBootResetDeviceId_McId,  NV_FALSE);
    // Ideally that code needs to be converted to NV_WRITE32 or converted to be parsed by the warmboot code test
    //NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_H_CLR_0,CLK_RST_CONTROLLER_RST_DEV_H_CLR_0_CLR_EMC_RST_FIELD | CLK_RST_CONTROLLER_RST_DEV_H_CLR_0_CLR_MEM_RST_FIELD);

    //  T214 - this was bug in T186 - cmd/brick mapping must be done before timing_update
    // Program CMD mapping.  Required to be done before brick mapping, otherwise
    //  We can't gurantee CK will be differential at all times
    HW_REGW(EmcBase, EMC, FBIO_CFG7, pData->EmcFbioCfg7);

    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD0_0, pData->EmcCmdMappingCmd0_0);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD0_1, pData->EmcCmdMappingCmd0_1);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD0_2, pData->EmcCmdMappingCmd0_2);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD1_0, pData->EmcCmdMappingCmd1_0);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD1_1, pData->EmcCmdMappingCmd1_1);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD1_2, pData->EmcCmdMappingCmd1_2);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD2_0, pData->EmcCmdMappingCmd2_0);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD2_1, pData->EmcCmdMappingCmd2_1);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD2_2, pData->EmcCmdMappingCmd2_2);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD3_0, pData->EmcCmdMappingCmd3_0);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD3_1, pData->EmcCmdMappingCmd3_1);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_CMD3_2, pData->EmcCmdMappingCmd3_2);
    HW_REGW(EmcBase, EMC, CMD_MAPPING_BYTE,   pData->EmcCmdMappingByte);

    HW_REGW(EmcBase, EMC, PMACRO_BRICK_MAPPING_0, pData->EmcPmacroBrickMapping0);
    HW_REGW(EmcBase, EMC, PMACRO_BRICK_MAPPING_1, pData->EmcPmacroBrickMapping1);
    HW_REGW(EmcBase, EMC, PMACRO_BRICK_MAPPING_2, pData->EmcPmacroBrickMapping2);

    //Update regulator settings simultaneously so we don't create multi-voltage timing paths
    HW_REGW(EmcBase, EMC, PMACRO_VTTGEN_CTRL_0, pData->EmcPmacroVttgenCtrl0);
    HW_REGW(EmcBase, EMC, PMACRO_VTTGEN_CTRL_1, pData->EmcPmacroVttgenCtrl1);
    HW_REGW(EmcBase, EMC, PMACRO_VTTGEN_CTRL_2, pData->EmcPmacroVttgenCtrl2);
    HW_REGW(EmcBase, EMC, PMACRO_BG_BIAS_CTRL_0, pData->EmcPmacroBgBiasCtrl0);

    if(pData->EmcBctSpareSecure0)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure0, pData->EmcBctSpareSecure1);
    if(pData->EmcBctSpareSecure2)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure2, pData->EmcBctSpareSecure3);
    if(pData->EmcBctSpareSecure4)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure4, pData->EmcBctSpareSecure5);

    HW_REGW(EmcBase, EMC, TIMING_CONTROL, 1); // Trigger timing update so above take effect
    NvBootUtilWaitUS(2 + pData->PmcVddpSelWait); // add a wait to make sure we give enough time for regulators to settle

    // Update emc clock source register from pData ->EmcClockSource
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, pData ->EmcClockSource);

    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_EMC_DLL_0, pData->EmcClockSourceDll);

    NvBootUtilWaitUS(5); // add a wait to make sure we give enough time for clk_switch to take place. 


    // this line is used on fakeboot/wb_code test and all executions through emc_reg_calc (e.g Simfront). The command does not need to get into the bootrom
    // as it's already covered in the code above:  
    // Remove the EMC and MC controllers from Reset

    HW_REGW(EmcBase, EMC, DBG, pData->EmcDbg | NV_DRF_NUM(EMC,DBG, WRITE_MUX, pData->EmcDbgWriteMux));
    

    // Patch 2 using BCT Spare Variables
    if(pData->EmcBctSpare2)
      NvBootMssSpareWrite(pData->EmcBctSpare2, pData->EmcBctSpare3);

    // This is required to do any reads from the pad macros
    HW_REGW(EmcBase, EMC, CONFIG_SAMPLE_DELAY,    pData->EmcConfigSampleDelay);

    // During WARM_BOOT, need to change PIN_RESET value to INACTIVE
    if ( IsWarmboot ){
        // Set reset pin == 1
        HW_REGW(EmcBase, EMC, PIN, 
                    (NV_DRF_NUM(EMC, PIN, PIN_GPIO_EN, pData->EmcPinGpioEn) |
                    NV_DRF_NUM(EMC, PIN, PIN_GPIO,    pData->EmcPinGpio) |
                    NV_DRF_DEF(EMC, PIN, PIN_RESET, INACTIVE)));

//                    NV_FLD_SET_DRF_DEF(EMC, PIN, PIN_RESET, INACTIVE, pData->EmcPin));
        NvBootUtilWaitUS(pData->EmcPinProgramWait);
    }

    HW_REGW(EmcBase, EMC, FBIO_CFG8, pData->EmcFbioCfg8);

    //Change the programming sequence for swizzle registers before programming any of the other EMC registers.
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK0_BYTE0, pData->EmcSwizzleRank0Byte0);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK0_BYTE1, pData->EmcSwizzleRank0Byte1);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK0_BYTE2, pData->EmcSwizzleRank0Byte2);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK0_BYTE3, pData->EmcSwizzleRank0Byte3);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK1_BYTE0, pData->EmcSwizzleRank1Byte0);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK1_BYTE1, pData->EmcSwizzleRank1Byte1);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK1_BYTE2, pData->EmcSwizzleRank1Byte2);
    HW_REGW(EmcBase, EMC, SWIZZLE_RANK1_BYTE3, pData->EmcSwizzleRank1Byte3);
    

    // Patch 4 using BCT Spare Variables
    if(pData->EmcBctSpare6)
      NvBootMssSpareWrite(pData->EmcBctSpare6, pData->EmcBctSpare7);
    
    // Program the pad controls
    HW_REGW(EmcBase, EMC, XM2COMPPADCTRL, pData->EmcXm2CompPadCtrl);
    HW_REGW(EmcBase, EMC, XM2COMPPADCTRL2, pData->EmcXm2CompPadCtrl2);
    HW_REGW(EmcBase, EMC, XM2COMPPADCTRL3, pData->EmcXm2CompPadCtrl3);

    // Program Autocal controls with shadowed register fields
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG2,          pData->EmcAutoCalConfig2);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG3,          pData->EmcAutoCalConfig3);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG4,          pData->EmcAutoCalConfig4);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG5,          pData->EmcAutoCalConfig5);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG6,          pData->EmcAutoCalConfig6);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG7,          pData->EmcAutoCalConfig7);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG8,          pData->EmcAutoCalConfig8);

    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_RX_TERM,            pData->EmcPmacroRxTerm);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_DQ_TX_DRV,          pData->EmcPmacroDqTxDrv);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_CA_TX_DRV,          pData->EmcPmacroCaTxDrv);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_CMD_TX_DRV,         pData->EmcPmacroCmdTxDrv);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_AUTOCAL_CFG_COMMON, pData->EmcPmacroAutocalCfgCommon);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CHANNEL,          pData->EmcAutoCalChannel);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_ZCTRL,              pData->EmcPmacroZctrl);


    HW_REGW(EmcBase, EMC, PMACRO_DLL_CFG_0, pData->EmcPmacroDllCfg0);
    HW_REGW(EmcBase, EMC, PMACRO_DLL_CFG_1, pData->EmcPmacroDllCfg1);
    HW_REGW(EmcBase, EMC, CFG_DIG_DLL_1, pData->EmcCfgDigDll_1);

    HW_REGW(EmcBase, EMC, DATA_BRLSHFT_0, pData->EmcDataBrlshft0);
    HW_REGW(EmcBase, EMC, DATA_BRLSHFT_1, pData->EmcDataBrlshft1);
    HW_REGW(EmcBase, EMC, DQS_BRLSHFT_0, pData->EmcDqsBrlshft0);
    HW_REGW(EmcBase, EMC, DQS_BRLSHFT_1, pData->EmcDqsBrlshft1);
    HW_REGW(EmcBase, EMC, CMD_BRLSHFT_0, pData->EmcCmdBrlshft0);
    HW_REGW(EmcBase, EMC, CMD_BRLSHFT_1, pData->EmcCmdBrlshft1);
    HW_REGW(EmcBase, EMC, CMD_BRLSHFT_2, pData->EmcCmdBrlshft2);
    HW_REGW(EmcBase, EMC, CMD_BRLSHFT_3, pData->EmcCmdBrlshft3);
    HW_REGW(EmcBase, EMC, QUSE_BRLSHFT_0, pData->EmcQuseBrlshft0);
    HW_REGW(EmcBase, EMC, QUSE_BRLSHFT_1, pData->EmcQuseBrlshft1);
    HW_REGW(EmcBase, EMC, QUSE_BRLSHFT_2, pData->EmcQuseBrlshft2);
    HW_REGW(EmcBase, EMC, QUSE_BRLSHFT_3, pData->EmcQuseBrlshft3);

    HW_REGW(EmcBase, EMC, PMACRO_BRICK_CTRL_RFU1, pData->EmcPmacroBrickCtrlRfu1);

    HW_REGW(EmcBase, EMC, PMACRO_PAD_CFG_CTRL, pData->EmcPmacroPadCfgCtrl);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_BRICK_CTRL_FDPD,    pData->EmcPmacroCmdBrickCtrlFdpd);
    HW_REGW(EmcBase, EMC, PMACRO_BRICK_CTRL_RFU2,    pData->EmcPmacroBrickCtrlRfu2);
    HW_REGW(EmcBase, EMC, PMACRO_DATA_BRICK_CTRL_FDPD,    pData->EmcPmacroDataBrickCtrlFdpd);
    HW_REGW(EmcBase, EMC, PMACRO_DATA_PAD_RX_CTRL, pData->EmcPmacroDataPadRxCtrl);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_PAD_RX_CTRL, pData->EmcPmacroCmdPadRxCtrl);
    HW_REGW(EmcBase, EMC, PMACRO_DATA_PAD_TX_CTRL, pData->EmcPmacroDataPadTxCtrl);
    HW_REGW(EmcBase, EMC, PMACRO_DATA_RX_TERM_MODE, pData->EmcPmacroDataRxTermMode);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_RX_TERM_MODE, pData->EmcPmacroCmdRxTermMode);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_PAD_TX_CTRL, pData->EmcPmacroCmdPadTxCtrl & 0xEFFFFFFF); //MH - T214 review - masks CMD_DQS_TX_DISABLE_CAL_UPDATE for initial autocal cycle

    HW_REGW(EmcBase, EMC, CFG_3, pData->EmcCfg3);
    HW_REGW(EmcBase, EMC, PMACRO_TX_PWRD_0, pData->EmcPmacroTxPwrd0);
    HW_REGW(EmcBase, EMC, PMACRO_TX_PWRD_1, pData->EmcPmacroTxPwrd1);
    HW_REGW(EmcBase, EMC, PMACRO_TX_PWRD_2, pData->EmcPmacroTxPwrd2);
    HW_REGW(EmcBase, EMC, PMACRO_TX_PWRD_3, pData->EmcPmacroTxPwrd3);
    HW_REGW(EmcBase, EMC, PMACRO_TX_PWRD_4, pData->EmcPmacroTxPwrd4);
    HW_REGW(EmcBase, EMC, PMACRO_TX_PWRD_5, pData->EmcPmacroTxPwrd5);
    HW_REGW(EmcBase, EMC, PMACRO_TX_SEL_CLK_SRC_0, pData->EmcPmacroTxSelClkSrc0);
    HW_REGW(EmcBase, EMC, PMACRO_TX_SEL_CLK_SRC_1, pData->EmcPmacroTxSelClkSrc1);
    HW_REGW(EmcBase, EMC, PMACRO_TX_SEL_CLK_SRC_2, pData->EmcPmacroTxSelClkSrc2);
    HW_REGW(EmcBase, EMC, PMACRO_TX_SEL_CLK_SRC_3, pData->EmcPmacroTxSelClkSrc3);
    HW_REGW(EmcBase, EMC, PMACRO_TX_SEL_CLK_SRC_4, pData->EmcPmacroTxSelClkSrc4);
    HW_REGW(EmcBase, EMC, PMACRO_TX_SEL_CLK_SRC_5, pData->EmcPmacroTxSelClkSrc5);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_FGCG_CTRL_0, pData->EmcPmacroPerbitFgcgCtrl0);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_FGCG_CTRL_1, pData->EmcPmacroPerbitFgcgCtrl1);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_FGCG_CTRL_2, pData->EmcPmacroPerbitFgcgCtrl2);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_FGCG_CTRL_3, pData->EmcPmacroPerbitFgcgCtrl3);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_FGCG_CTRL_4, pData->EmcPmacroPerbitFgcgCtrl4);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_FGCG_CTRL_5, pData->EmcPmacroPerbitFgcgCtrl5);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU_CTRL_0, pData->EmcPmacroPerbitRfuCtrl0);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU_CTRL_1, pData->EmcPmacroPerbitRfuCtrl1);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU_CTRL_2, pData->EmcPmacroPerbitRfuCtrl2);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU_CTRL_3, pData->EmcPmacroPerbitRfuCtrl3);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU_CTRL_4, pData->EmcPmacroPerbitRfuCtrl4);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU_CTRL_5, pData->EmcPmacroPerbitRfuCtrl5);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU1_CTRL_0, pData->EmcPmacroPerbitRfu1Ctrl0);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU1_CTRL_1, pData->EmcPmacroPerbitRfu1Ctrl1);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU1_CTRL_2, pData->EmcPmacroPerbitRfu1Ctrl2);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU1_CTRL_3, pData->EmcPmacroPerbitRfu1Ctrl3);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU1_CTRL_4, pData->EmcPmacroPerbitRfu1Ctrl4);
    HW_REGW(EmcBase, EMC, PMACRO_PERBIT_RFU1_CTRL_5, pData->EmcPmacroPerbitRfu1Ctrl5);
    HW_REGW(EmcBase, EMC, PMACRO_DATA_PI_CTRL, pData->EmcPmacroDataPiCtrl);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_PI_CTRL, pData->EmcPmacroCmdPiCtrl);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_BYPASS,      pData->EmcPmacroDdllBypass);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_PWRD_0,      pData->EmcPmacroDdllPwrd0);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_PWRD_1,      pData->EmcPmacroDdllPwrd1);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_PWRD_2,      pData->EmcPmacroDdllPwrd2);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_CTRL_0,       pData->EmcPmacroCmdCtrl0);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_CTRL_1,       pData->EmcPmacroCmdCtrl1);
    HW_REGW(EmcBase, EMC, PMACRO_CMD_CTRL_2,       pData->EmcPmacroCmdCtrl2);
    HW_REGW(EmcBase, EMC, PMACRO_IB_VREF_DQ_0,      pData->EmcPmacroIbVrefDq_0);
    HW_REGW(EmcBase, EMC, PMACRO_IB_VREF_DQ_1,      pData->EmcPmacroIbVrefDq_1);
    HW_REGW(EmcBase, EMC, PMACRO_IB_VREF_DQS_0,     pData->EmcPmacroIbVrefDqs_0);
    HW_REGW(EmcBase, EMC, PMACRO_IB_VREF_DQS_1,     pData->EmcPmacroIbVrefDqs_1);
    HW_REGW(EmcBase, EMC, PMACRO_IB_RXRT,           pData->EmcPmacroIbRxrt);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK0_0, pData->EmcPmacroQuseDdllRank0_0);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK0_1, pData->EmcPmacroQuseDdllRank0_1);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK0_2, pData->EmcPmacroQuseDdllRank0_2);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK0_3, pData->EmcPmacroQuseDdllRank0_3);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK0_4, pData->EmcPmacroQuseDdllRank0_4);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK0_5, pData->EmcPmacroQuseDdllRank0_5);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK1_0, pData->EmcPmacroQuseDdllRank1_0);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK1_1, pData->EmcPmacroQuseDdllRank1_1);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK1_2, pData->EmcPmacroQuseDdllRank1_2);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK1_3, pData->EmcPmacroQuseDdllRank1_3);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK1_4, pData->EmcPmacroQuseDdllRank1_4);
    HW_REGW(EmcBase, EMC, PMACRO_QUSE_DDLL_RANK1_5, pData->EmcPmacroQuseDdllRank1_5);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK0_0, pData->EmcPmacroObDdllLongDqRank0_0);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK0_1, pData->EmcPmacroObDdllLongDqRank0_1);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK0_2, pData->EmcPmacroObDdllLongDqRank0_2);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK0_3, pData->EmcPmacroObDdllLongDqRank0_3);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK0_4, pData->EmcPmacroObDdllLongDqRank0_4);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK0_5, pData->EmcPmacroObDdllLongDqRank0_5);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK1_0, pData->EmcPmacroObDdllLongDqRank1_0);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK1_1, pData->EmcPmacroObDdllLongDqRank1_1);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK1_2, pData->EmcPmacroObDdllLongDqRank1_2);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK1_3, pData->EmcPmacroObDdllLongDqRank1_3);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK1_4, pData->EmcPmacroObDdllLongDqRank1_4);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQ_RANK1_5, pData->EmcPmacroObDdllLongDqRank1_5);
    
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK0_0, pData->EmcPmacroObDdllLongDqsRank0_0);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK0_1, pData->EmcPmacroObDdllLongDqsRank0_1);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK0_2, pData->EmcPmacroObDdllLongDqsRank0_2);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK0_3, pData->EmcPmacroObDdllLongDqsRank0_3);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK0_4, pData->EmcPmacroObDdllLongDqsRank0_4);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK0_5, pData->EmcPmacroObDdllLongDqsRank0_5);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK1_0, pData->EmcPmacroObDdllLongDqsRank1_0);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK1_1, pData->EmcPmacroObDdllLongDqsRank1_1);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK1_2, pData->EmcPmacroObDdllLongDqsRank1_2);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK1_3, pData->EmcPmacroObDdllLongDqsRank1_3);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK1_4, pData->EmcPmacroObDdllLongDqsRank1_4);
    HW_REGW(EmcBase, EMC, PMACRO_OB_DDLL_LONG_DQS_RANK1_5, pData->EmcPmacroObDdllLongDqsRank1_5);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK0_0, pData->EmcPmacroIbDdllLongDqsRank0_0);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK0_1, pData->EmcPmacroIbDdllLongDqsRank0_1);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK0_2, pData->EmcPmacroIbDdllLongDqsRank0_2);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK0_3, pData->EmcPmacroIbDdllLongDqsRank0_3);
    //HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK0_4, pData->EmcPmacroIbDdllLongDqsRank0_4);
    //HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK0_5, pData->EmcPmacroIbDdllLongDqsRank0_5);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK1_0, pData->EmcPmacroIbDdllLongDqsRank1_0);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK1_1, pData->EmcPmacroIbDdllLongDqsRank1_1);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK1_2, pData->EmcPmacroIbDdllLongDqsRank1_2);
    HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK1_3, pData->EmcPmacroIbDdllLongDqsRank1_3);
    //HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK1_4, pData->EmcPmacroIbDdllLongDqsRank1_4);
    //HW_REGW(EmcBase, EMC, PMACRO_IB_DDLL_LONG_DQS_RANK1_5, pData->EmcPmacroIbDdllLongDqsRank1_5);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_LONG_CMD_0, pData->EmcPmacroDdllLongCmd_0);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_LONG_CMD_1, pData->EmcPmacroDdllLongCmd_1);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_LONG_CMD_2, pData->EmcPmacroDdllLongCmd_2);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_LONG_CMD_3, pData->EmcPmacroDdllLongCmd_3);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_LONG_CMD_4, pData->EmcPmacroDdllLongCmd_4);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_SHORT_CMD_0, pData->EmcPmacroDdllShortCmd_0);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_SHORT_CMD_1, pData->EmcPmacroDdllShortCmd_1);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_SHORT_CMD_2, pData->EmcPmacroDdllShortCmd_2);
    HW_REGW(EmcBase, EMC, PMACRO_DDLL_PERIODIC_OFFSET, pData->EmcPmacroDdllPeriodicOffset);
    //HW_REGW(EmcBase, EMC, PMACRO_COMMON_PAD_TX_CTRL, common_pad_macro_step1);

    // Patch 3 using BCT Spare Variables
    if(pData->EmcBctSpare4)
      NvBootMssSpareWrite(pData->EmcBctSpare4, pData->EmcBctSpare5);
    if(pData->EmcBctSpareSecure6)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure6, pData->EmcBctSpareSecure7);
    if(pData->EmcBctSpareSecure8)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure8, pData->EmcBctSpareSecure9);
    if(pData->EmcBctSpareSecure10)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure10, pData->EmcBctSpareSecure11);

    HW_REGW(EmcBase, EMC, TIMING_CONTROL, 1); // Trigger timing update after padctrl. Otherwise shadowed values will not be enabled until AFTER the first AUTO_CAL run, mis-calibrating the pads based on their reset defaults.

    // MC sequence
    
    HW_REGW(McBase, MC, VIDEO_PROTECT_BOM         ,pData->McVideoProtectBom);
    HW_REGW(McBase, MC, VIDEO_PROTECT_BOM_ADR_HI  ,pData->McVideoProtectBomAdrHi);
    HW_REGW(McBase, MC, VIDEO_PROTECT_SIZE_MB     ,pData->McVideoProtectSizeMb);
    HW_REGW(McBase, MC, VIDEO_PROTECT_VPR_OVERRIDE,pData->McVideoProtectVprOverride);
    HW_REGW(McBase, MC, VIDEO_PROTECT_VPR_OVERRIDE1,pData->McVideoProtectVprOverride1);
    HW_REGW(McBase, MC, VIDEO_PROTECT_GPU_OVERRIDE_0,pData->McVideoProtectGpuOverride0);
    HW_REGW(McBase, MC, VIDEO_PROTECT_GPU_OVERRIDE_1,pData->McVideoProtectGpuOverride1);
    // Moving sticky bit McVideoProtectWriteAccess to end of sequence. 

    
    HW_REGW(McBase, MC, EMEM_ADR_CFG, pData->McEmemAdrCfg);
    HW_REGW(McBase, MC, EMEM_ADR_CFG_DEV0, pData->McEmemAdrCfgDev0);
    HW_REGW(McBase, MC, EMEM_ADR_CFG_DEV1, pData->McEmemAdrCfgDev1);
    HW_REGW(McBase, MC, EMEM_ADR_CFG_CHANNEL_MASK, pData->McEmemAdrCfgChannelMask);
    HW_REGW(McBase, MC, EMEM_ADR_CFG_BANK_MASK_0, pData->McEmemAdrCfgBankMask0);
    HW_REGW(McBase, MC, EMEM_ADR_CFG_BANK_MASK_1, pData->McEmemAdrCfgBankMask1);
    HW_REGW(McBase, MC, EMEM_ADR_CFG_BANK_MASK_2, pData->McEmemAdrCfgBankMask2);
    HW_REGW(McBase, MC, EMEM_CFG, pData->McEmemCfg);

    // Move sticky bit EMEM_CFG_ACCESS_CTRL to end of sequence
    HW_REGW(McBase, MC, SEC_CARVEOUT_BOM          ,pData->McSecCarveoutBom);
    HW_REGW(McBase, MC, SEC_CARVEOUT_ADR_HI       ,pData->McSecCarveoutAdrHi);
    HW_REGW(McBase, MC, SEC_CARVEOUT_SIZE_MB      ,pData->McSecCarveoutSizeMb);
    // Moving sticky bit McSecCarveoutProtectWriteAccess to end of sequence. 
    HW_REGW(McBase, MC, MTS_CARVEOUT_BOM          ,pData->McMtsCarveoutBom);
    HW_REGW(McBase, MC, MTS_CARVEOUT_ADR_HI       ,pData->McMtsCarveoutAdrHi);
    HW_REGW(McBase, MC, MTS_CARVEOUT_SIZE_MB      ,pData->McMtsCarveoutSizeMb);
    // Moving sticky bit McMtsCarveoutRegCtrl to end of sequence. 

    HW_REGW(McBase, MC, EMEM_ARB_CFG,             pData->McEmemArbCfg);
    HW_REGW(McBase, MC, EMEM_ARB_OUTSTANDING_REQ, pData->McEmemArbOutstandingReq);
    HW_REGW(McBase, MC, EMEM_ARB_REFPB_HP_CTRL,   pData->McEmemArbRefpbHpCtrl);
    HW_REGW(McBase, MC, EMEM_ARB_REFPB_BANK_CTRL, pData->McEmemArbRefpbBankCtrl);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RCD,      pData->McEmemArbTimingRcd);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RP,       pData->McEmemArbTimingRp);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RC,       pData->McEmemArbTimingRc);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RAS,      pData->McEmemArbTimingRas);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_FAW,      pData->McEmemArbTimingFaw);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RRD,      pData->McEmemArbTimingRrd);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RAP2PRE,  pData->McEmemArbTimingRap2Pre);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_WAP2PRE,  pData->McEmemArbTimingWap2Pre);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_R2R,      pData->McEmemArbTimingR2R);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_W2W,      pData->McEmemArbTimingW2W);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_CCDMW,    pData->McEmemArbTimingCcdmw);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_R2W,      pData->McEmemArbTimingR2W);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_W2R,      pData->McEmemArbTimingW2R);
    HW_REGW(McBase, MC, EMEM_ARB_TIMING_RFCPB,    pData->McEmemArbTimingRFCPB);
    HW_REGW(McBase, MC, EMEM_ARB_DA_TURNS,        pData->McEmemArbDaTurns);
    HW_REGW(McBase, MC, EMEM_ARB_DA_COVERS,       pData->McEmemArbDaCovers);
    HW_REGW(McBase, MC, EMEM_ARB_MISC0,           pData->McEmemArbMisc0);
    HW_REGW(McBase, MC, EMEM_ARB_MISC1,           pData->McEmemArbMisc1);
    HW_REGW(McBase, MC, EMEM_ARB_MISC2,           pData->McEmemArbMisc2);
    HW_REGW(McBase, MC, EMEM_ARB_RING1_THROTTLE,  pData->McEmemArbRing1Throttle);
    HW_REGW(McBase, MC, EMEM_ARB_OVERRIDE,        pData->McEmemArbOverride);
    HW_REGW(McBase, MC, EMEM_ARB_OVERRIDE_1,      pData->McEmemArbOverride1);
    HW_REGW(McBase, MC, EMEM_ARB_RSV,             pData->McEmemArbRsv);
    HW_REGW(McBase, MC, DA_CONFIG0,               pData->McDaCfg0);
    HW_REGW(McBase, MC, TIMING_CONTROL, 1); // Trigger - just needs non-zero arg


    HW_REGW(McBase, MC, CLKEN_OVERRIDE, pData->McClkenOverride);
    HW_REGW(McBase, MC, STAT_CONTROL, pData->McStatControl);
    // EMC sequence
    HW_REGW(EmcBase, EMC, ADR_CFG, pData->EmcAdrCfg);
    HW_REGW(EmcBase, EMC, CLKEN_OVERRIDE, pData->EmcClkenOverride);

    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_AUTOCAL_CFG_0,      pData->EmcPmacroAutocalCfg0);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_AUTOCAL_CFG_1,      pData->EmcPmacroAutocalCfg1);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, PMACRO_AUTOCAL_CFG_2,      pData->EmcPmacroAutocalCfg2);

    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_VREF_SEL_0,       pData->EmcAutoCalVrefSel0);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_VREF_SEL_1,       pData->EmcAutoCalVrefSel1);
    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_INTERVAL,         pData->EmcAutoCalInterval);

    HW_REGW(EmcBase, EMC, AUTO_CAL_CONFIG, pData->EmcAutoCalConfig);
    NvBootUtilWaitUS(pData->EmcAutoCalWait);

    // Patch 5 using BCT Spare Variables
    if(pData->EmcBctSpare8)
      NvBootMssSpareWrite(pData->EmcBctSpare8, pData->EmcBctSpare9);

    HW_REGW(NV_ADDRESS_MAP_EMC_BASE, EMC, AUTO_CAL_CONFIG9, pData->EmcAutoCalConfig9);
    HW_REGW(EmcBase, EMC, CFG_2, pData->EmcCfg2);
    HW_REGW(EmcBase, EMC, CFG_PIPE, pData->EmcCfgPipe);
    HW_REGW(EmcBase, EMC, CFG_PIPE_1, pData->EmcCfgPipe1);
    HW_REGW(EmcBase, EMC, CFG_PIPE_2, pData->EmcCfgPipe2);
    HW_REGW(EmcBase, EMC, CMDQ, pData->EmcCmdQ);
    HW_REGW(EmcBase, EMC, MC2EMCQ, pData->EmcMc2EmcQ);
    HW_REGW(EmcBase, EMC, MRS_WAIT_CNT, pData->EmcMrsWaitCnt);
    HW_REGW(EmcBase, EMC, MRS_WAIT_CNT2, pData->EmcMrsWaitCnt2);
    HW_REGW(EmcBase, EMC, FBIO_CFG5, pData->EmcFbioCfg5);
    HW_REGW(EmcBase, EMC, RC, pData->EmcRc);
    HW_REGW(EmcBase, EMC, RFC, pData->EmcRfc);
    HW_REGW(EmcBase, EMC, RFCPB, pData->EmcRfcPb);
    HW_REGW(EmcBase, EMC, REFCTRL2, pData->EmcRefctrl2);
    HW_REGW(EmcBase, EMC, RFC_SLR, pData->EmcRfcSlr);
    HW_REGW(EmcBase, EMC, RAS, pData->EmcRas);
    HW_REGW(EmcBase, EMC, RP, pData->EmcRp);
    HW_REGW(EmcBase, EMC, TPPD, pData->EmcTppd);
    HW_REGW(EmcBase, EMC, TRTM, pData->EmcTrtm);
    HW_REGW(EmcBase, EMC, TWTM, pData->EmcTwtm);
    HW_REGW(EmcBase, EMC, TRATM, pData->EmcTratm);
    HW_REGW(EmcBase, EMC, TWATM, pData->EmcTwatm);
    HW_REGW(EmcBase, EMC, TR2REF, pData->EmcTr2ref);
    HW_REGW(EmcBase, EMC, R2R, pData->EmcR2r);
    HW_REGW(EmcBase, EMC, W2W, pData->EmcW2w);
    HW_REGW(EmcBase, EMC, R2W, pData->EmcR2w);
    HW_REGW(EmcBase, EMC, W2R, pData->EmcW2r);
    HW_REGW(EmcBase, EMC, R2P, pData->EmcR2p);
    HW_REGW(EmcBase, EMC, W2P, pData->EmcW2p);
    HW_REGW(EmcBase, EMC, CCDMW, pData->EmcCcdmw);
    HW_REGW(EmcBase, EMC, RD_RCD, pData->EmcRdRcd);
    HW_REGW(EmcBase, EMC, WR_RCD, pData->EmcWrRcd);
    HW_REGW(EmcBase, EMC, RRD, pData->EmcRrd);
    HW_REGW(EmcBase, EMC, REXT, pData->EmcRext);
    HW_REGW(EmcBase, EMC, WEXT, pData->EmcWext);
    HW_REGW(EmcBase, EMC, WDV, pData->EmcWdv);
    HW_REGW(EmcBase, EMC, WDV_CHK, pData->EmcWdvChk);
    HW_REGW(EmcBase, EMC, WSV, pData->EmcWsv);
    HW_REGW(EmcBase, EMC, WEV, pData->EmcWev);
    HW_REGW(EmcBase, EMC, WDV_MASK, pData->EmcWdvMask);
    HW_REGW(EmcBase, EMC, WS_DURATION, pData->EmcWsDuration);
    HW_REGW(EmcBase, EMC, WE_DURATION, pData->EmcWeDuration);
    HW_REGW(EmcBase, EMC, QUSE, pData->EmcQUse);
    HW_REGW(EmcBase, EMC, QUSE_WIDTH, pData->EmcQuseWidth);
    HW_REGW(EmcBase, EMC, IBDLY, pData->EmcIbdly);
    HW_REGW(EmcBase, EMC, OBDLY, pData->EmcObdly);
    HW_REGW(EmcBase, EMC, EINPUT, pData->EmcEInput);
    HW_REGW(EmcBase, EMC, EINPUT_DURATION, pData->EmcEInputDuration);
    HW_REGW(EmcBase, EMC, PUTERM_EXTRA, pData->EmcPutermExtra);
    HW_REGW(EmcBase, EMC, PUTERM_WIDTH, pData->EmcPutermWidth);
    
    //HW_REGW(EmcBase, EMC, PMACRO_COMMON_PAD_TX_CTRL, pData->EmcPmacroCommonPadTxCtrl);
    HW_REGW(EmcBase, EMC, DBG, pData->EmcDbg);
    HW_REGW(EmcBase, EMC, QRST, pData->EmcQRst);
    HW_REGW(EmcBase, EMC, ISSUE_QRST, 0x1);
    HW_REGW(EmcBase, EMC, ISSUE_QRST, 0x0);
    HW_REGW(EmcBase, EMC, QSAFE, pData->EmcQSafe);
    HW_REGW(EmcBase, EMC, RDV, pData->EmcRdv);
    HW_REGW(EmcBase, EMC, RDV_MASK, pData->EmcRdvMask);
    HW_REGW(EmcBase, EMC, RDV_EARLY, pData->EmcRdvEarly);
    HW_REGW(EmcBase, EMC, RDV_EARLY_MASK, pData->EmcRdvEarlyMask);
    HW_REGW(EmcBase, EMC, QPOP, pData->EmcQpop);
    HW_REGW(EmcBase, EMC, REFRESH, pData->EmcRefresh);
    HW_REGW(EmcBase, EMC, BURST_REFRESH_NUM, pData->EmcBurstRefreshNum);
    HW_REGW(EmcBase, EMC, PRE_REFRESH_REQ_CNT, pData->EmcPreRefreshReqCnt);
    HW_REGW(EmcBase, EMC, PDEX2WR, pData->EmcPdEx2Wr);
    HW_REGW(EmcBase, EMC, PDEX2RD, pData->EmcPdEx2Rd);
    HW_REGW(EmcBase, EMC, PCHG2PDEN, pData->EmcPChg2Pden);
    HW_REGW(EmcBase, EMC, ACT2PDEN, pData->EmcAct2Pden);
    HW_REGW(EmcBase, EMC, AR2PDEN, pData->EmcAr2Pden);
    HW_REGW(EmcBase, EMC, RW2PDEN, pData->EmcRw2Pden);
    HW_REGW(EmcBase, EMC, CKE2PDEN, pData->EmcCke2Pden);
    HW_REGW(EmcBase, EMC, PDEX2CKE, pData->EmcPdex2Cke);
    HW_REGW(EmcBase, EMC, PDEX2MRR, pData->EmcPdex2Mrr);
    HW_REGW(EmcBase, EMC, TXSR, pData->EmcTxsr);
    HW_REGW(EmcBase, EMC, TXSRDLL, pData->EmcTxsrDll);
    HW_REGW(EmcBase, EMC, TCKE, pData->EmcTcke);
    HW_REGW(EmcBase, EMC, TCKESR, pData->EmcTckesr);
    HW_REGW(EmcBase, EMC, TPD, pData->EmcTpd);
    HW_REGW(EmcBase, EMC, TFAW, pData->EmcTfaw);
    HW_REGW(EmcBase, EMC, TRPAB, pData->EmcTrpab);
    HW_REGW(EmcBase, EMC, TCLKSTABLE, pData->EmcTClkStable);
    HW_REGW(EmcBase, EMC, TCLKSTOP, pData->EmcTClkStop);
    HW_REGW(EmcBase, EMC, TREFBW, pData->EmcTRefBw);
    HW_REGW(EmcBase, EMC, ODT_WRITE, pData->EmcOdtWrite);
    HW_REGW(EmcBase, EMC, CFG_DIG_DLL, pData->EmcCfgDigDll);
    HW_REGW(EmcBase, EMC, CFG_DIG_DLL_PERIOD, pData->EmcCfgDigDllPeriod);
    HW_REGW(EmcBase, EMC, FBIO_SPARE, pData->EmcFbioSpare & 0xfffffffd);// don't write bit 1 -- addr swizzle lock bit. written at end of sequence.  
    HW_REGW(EmcBase, EMC, CFG_RSV, pData->EmcCfgRsv);
    HW_REGW(EmcBase, EMC, PMC_SCRATCH1, pData->EmcPmcScratch1);
    HW_REGW(EmcBase, EMC, PMC_SCRATCH2, pData->EmcPmcScratch2);
    HW_REGW(EmcBase, EMC, PMC_SCRATCH3, pData->EmcPmcScratch3);
    HW_REGW(EmcBase, EMC, ACPD_CONTROL, pData->EmcAcpdControl);
    HW_REGW(EmcBase, EMC, TXDSRVTTGEN, pData->EmcTxdsrvttgen);
    HW_REGW(EmcBase, EMC, PMACRO_DSR_VTTGEN_CTRL_0, pData->EmcPmacroDsrVttgenCtrl0);


    // Set appropriate pipe_enables in EMC_CFG before sending any DRAM cmds. 
    // Other bits in EMC_CFG must come after REFCTRL
    RegData = NV_RESETVAL (EMC, CFG); 
    TempData = NV_DRF_VAL(EMC, CFG, EMC2PMACRO_CFG_BYPASS_DATAPIPE1, pData->EmcCfg);
    RegData = NV_FLD_SET_DRF_NUM ( EMC, CFG, EMC2PMACRO_CFG_BYPASS_DATAPIPE1, TempData, RegData); 
    TempData = NV_DRF_VAL(EMC, CFG, EMC2PMACRO_CFG_BYPASS_DATAPIPE2, pData->EmcCfg);
    RegData = NV_FLD_SET_DRF_NUM ( EMC, CFG, EMC2PMACRO_CFG_BYPASS_DATAPIPE2, TempData, RegData); 
    TempData = NV_DRF_VAL(EMC, CFG, EMC2PMACRO_CFG_BYPASS_ADDRPIPE, pData->EmcCfg);
    RegData = NV_FLD_SET_DRF_NUM ( EMC, CFG, EMC2PMACRO_CFG_BYPASS_ADDRPIPE, TempData, RegData); 

    HW_REGW (EmcBase, EMC, CFG, RegData); 

    // the following allows a patch of a single register that we may have forgotten
    // the address range supported is 0x7000_0000 to 0x7001_FFFF
    if (pData->BootRomPatchControl) {
      NvBootMssSpareWrite(pData->BootRomPatchControl, pData->BootRomPatchData);
      HW_REGW(McBase, MC, TIMING_CONTROL, 1);  // trigger MC just in case the patch needs it
    }

    if(pData->EmcBctSpareSecure12)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure12, pData->EmcBctSpareSecure13);
    if(pData->EmcBctSpareSecure14)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure14, pData->EmcBctSpareSecure15);
    if(pData->EmcBctSpareSecure16)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure16, pData->EmcBctSpareSecure17);

    // Release SEL_DPD_CMD
	
    dpd3_val = (pData->EmcPmcScratch1 & 0x3FFFFFFF) | 
                 NV_DRF_DEF(APBDEV_PMC, IO_DPD3_REQ, CODE, DPD_OFF);
    dpd3_val_sel_dpd = dpd3_val & 0xCFFF0000;

    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_IO_DPD3_REQ_0, dpd3_val_sel_dpd);
        
    NvBootPmcUtilWaitUS(pData->PmcIoDpd3ReqWait);

    HW_REGW(EmcBase, EMC, PMACRO_CMD_PAD_TX_CTRL, pData->EmcPmacroCmdPadTxCtrl);//MH - T214 review - sets final value -- bits masked above

    // ZQ CAL setup, not actually issuing ZQ CAL now.
    if (!IsWarmboot && NV_DRF_VAL(EMCZCAL, BOOT_ENABLE, COLDBOOT, pData->EmcZcalWarmColdBootEnables)) {
        if (pData->MemoryType == NvBootMemoryType_Ddr3)
        	HW_REGW(EmcBase, EMC, ZCAL_WAIT_CNT, (pData->EmcZcalWaitCnt << 3));
        else if (pData->MemoryType == NvBootMemoryType_LpDdr4) {
        	HW_REGW(EmcBase, EMC, ZCAL_WAIT_CNT, (pData->EmcZcalWaitCnt));
        	HW_REGW(EmcBase, EMC, ZCAL_MRW_CMD, (pData->EmcZcalMrwCmd));
        }
    }
    if (IsWarmboot && NV_DRF_VAL(EMCZCAL, BOOT_ENABLE, WARMBOOT, pData->EmcZcalWarmColdBootEnables)) {
        if (pData->MemoryType == NvBootMemoryType_Ddr3)
	        HW_REGW(EmcBase, EMC, ZCAL_WAIT_CNT, (pData->EmcZcalWaitCnt << 2));
        else if (pData->MemoryType == NvBootMemoryType_LpDdr4) {
        	HW_REGW(EmcBase, EMC, ZCAL_WAIT_CNT, (pData->EmcZcalWaitCnt));
        	HW_REGW(EmcBase, EMC, ZCAL_MRW_CMD, (pData->EmcZcalMrwCmd));
        }
    }

    HW_REGW(EmcBase, EMC, TIMING_CONTROL, 1); // Trigger - just needs non-zero arg

    // Wait for clock etc., if affected by shadow programming, to stablize
    NvBootUtilWaitUS(pData->EmcTimingControlWait);


    // Deassert HOLD_CKE_LOW
    ddrcntrl = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_CNTRL_0);
    //ddrcntrl = pData->PmcDdrCntrl;
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, TX_FORCE_TRISTATE,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR0,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR1,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR2,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR3,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR4,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR5,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR6,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR7,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR8,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR9,  0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR10, 0, ddrcntrl);
    ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, CMD_HOLD_LOW_BR11, 0, ddrcntrl);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_CNTRL_0, ddrcntrl);
    NvBootUtilWaitUS(pData->PmcDdrCntrlWait); //pupreti

    if (!IsWarmboot && (pData->MemoryType == NvBootMemoryType_Ddr3))
    {
        // Apply reset on DDR3
        HW_REGW(EmcBase, EMC, PIN,
                NV_DRF_NUM(EMC, PIN, PIN_GPIO_EN, pData->EmcPinGpioEn) |
                NV_DRF_NUM(EMC, PIN, PIN_GPIO,    pData->EmcPinGpio) |
                NV_DRF_DEF(EMC, PIN, PIN_RESET, ACTIVE    ) |
                NV_DRF_DEF(EMC, PIN, PIN_CKE,   POWERDOWN ) |
                NV_DRF_DEF(EMC, PIN, PIN_DQM,   NORMAL    ));

    // Bug 962057, assert dummy read of PIN register to ensure above write to PIN register went through. 
    // This looks applicable to t124, leaving the code in here, too. Instanced bug 994542. 
    RegData = HW_REGR (EMC, EMC, PIN);
        NvBootUtilWaitUS(200 + pData->EmcPinExtraWait);

        // Deassert reset on DDR3
        HW_REGW(EmcBase, EMC, PIN,
                NV_DRF_NUM(EMC, PIN, PIN_GPIO_EN, pData->EmcPinGpioEn) |
                NV_DRF_NUM(EMC, PIN, PIN_GPIO,    pData->EmcPinGpio) |
                NV_DRF_DEF(EMC, PIN, PIN_RESET, INACTIVE  ) |
                NV_DRF_DEF(EMC, PIN, PIN_CKE,   POWERDOWN ) |
                NV_DRF_DEF(EMC, PIN, PIN_DQM,   NORMAL    ));

    // Bug 962057, assert dummy read of PIN register to ensure above write to PIN register went through. 
    RegData = HW_REGR (EMC, EMC, PIN);
        NvBootUtilWaitUS(500 + pData->EmcPinExtraWait);
    }

    if (!IsWarmboot && (pData->MemoryType == NvBootMemoryType_LpDdr4))
    {
        // Apply reset on LPDDR4 
        HW_REGW(EmcBase, EMC, PIN,
                NV_DRF_NUM(EMC, PIN, PIN_GPIO_EN, pData->EmcPinGpioEn) |
                NV_DRF_NUM(EMC, PIN, PIN_GPIO,    pData->EmcPinGpio) |
                NV_DRF_DEF(EMC, PIN, PIN_RESET, ACTIVE    ) |
                NV_DRF_DEF(EMC, PIN, PIN_CKE,   POWERDOWN ) |
                NV_DRF_DEF(EMC, PIN, PIN_DQM,   NORMAL    ));

    // Bug 962057, assert dummy read of PIN register to ensure above write to PIN register went through. 
    // This looks applicable to t124, leaving the code in here, too. Instanced bug 994542. 
    RegData = HW_REGR (EMC, EMC, PIN);
        NvBootUtilWaitUS(200 + pData->EmcPinExtraWait);

        // Deassert reset on LPDDR4 
        HW_REGW(EmcBase, EMC, PIN,
                NV_DRF_NUM(EMC, PIN, PIN_GPIO_EN, pData->EmcPinGpioEn) |
                NV_DRF_NUM(EMC, PIN, PIN_GPIO,    pData->EmcPinGpio) |
                NV_DRF_DEF(EMC, PIN, PIN_RESET, INACTIVE  ) |
                NV_DRF_DEF(EMC, PIN, PIN_CKE,   POWERDOWN ) |
                NV_DRF_DEF(EMC, PIN, PIN_DQM,   NORMAL    ));

    // Bug 962057, assert dummy read of PIN register to ensure above write to PIN register went through. 
    RegData = HW_REGR (EMC, EMC, PIN);
        NvBootUtilWaitUS(2000 + pData->EmcPinExtraWait);
    }

    #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
    // On cold boot w/ DDR/DDR2: Wait 200usec to guarantee 200usec between
    // clock stable and CKE high.
    if (!IsWarmboot && ((pData->MemoryType == NvBootMemoryType_Ddr ) ||
                        (pData->MemoryType == NvBootMemoryType_Ddr2)))
    {
        NvBootUtilWaitUS(200 + pData->EmcPinExtraWait);
    }
    #endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES

    

    // Apply CKE and maintain stable clock by inputing NOP
    HW_REGW(EmcBase, EMC, PIN,
            NV_DRF_NUM(EMC, PIN, PIN_GPIO_EN, pData->EmcPinGpioEn) |
            NV_DRF_NUM(EMC, PIN, PIN_GPIO,    pData->EmcPinGpio) |
            NV_DRF_DEF(EMC, PIN, PIN_RESET, INACTIVE) |
            NV_DRF_DEF(EMC, PIN, PIN_CKE,   NORMAL  ) |
            NV_DRF_DEF(EMC, PIN, PIN_DQM,   NORMAL  ));

    // Bug 962057, assert dummy read of PIN register to ensure above write to PIN register went through. 
    RegData = HW_REGR (EMC, EMC, PIN);
    NvBootUtilWaitUS(pData->EmcPinProgramWait);
    if (IsWarmboot && pData->MemoryType == NvBootMemoryType_LpDdr4) {
        HW_REGW(EmcBase, EMC, SELF_REF, 0x0);
    }

    if (pData->MemoryType != NvBootMemoryType_LpDdr4)
        HW_REGW(EmcBase, EMC, NOP,
            NV_DRF_NUM(EMC, NOP, NOP_CMD, 1) | // Trigger - needs non-zero arg
            NV_DRF_NUM(EMC, NOP, NOP_DEV_SELECTN, pData->EmcDevSelect));
    
    // On cold boot w/ LPDDR and LPDDR2 and LPDDR3: Wait 200usec after asserting CKE high.
    if (!IsWarmboot) { 
        #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
        if (pData->MemoryType == NvBootMemoryType_LpDdr ) {
            NvBootUtilWaitUS(200 + pData->EmcPinExtraWait);
        }
        #endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES
        if (pData->MemoryType == NvBootMemoryType_LpDdr2) { 
            NvBootUtilWaitUS(200 + pData->EmcPinExtraWait);
        }
    }


    if (IsWarmboot)
    {
        NvBootUtilWaitUS(pData->WarmBootWait);

        // Mode register write
        if (pData->EmcMrsWarmBootEnable)
        {
            if (pData->MemoryType == NvBootMemoryType_LpDdr2)
            {
                HW_REGW(EmcBase, EMC, MRW2, pData->EmcMrw2);
                HW_REGW(EmcBase, EMC, MRW, pData->EmcMrw1);
                HW_REGW(EmcBase, EMC, MRW3, pData->EmcMrw3);
                HW_REGW(EmcBase, EMC, MRW4, pData->EmcMrw4);
                if (pData->EmcWarmBootExtraModeRegWriteEnable)
                {
                    HW_REGW(EmcBase, EMC, MRW, pData->EmcWarmBootMrwExtra);
                }
            }
            else if (pData->MemoryType == NvBootMemoryType_Ddr3)
            {
                HW_REGW(EmcBase, EMC, EMRS2, pData->EmcEmrs2);
                HW_REGW(EmcBase, EMC, EMRS3, pData->EmcEmrs3);
                HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);
                HW_REGW(EmcBase, EMC, MRS,  pData->EmcMrs);
                if (pData->EmcWarmBootExtraModeRegWriteEnable)
                {
                    HW_REGW(EmcBase, EMC, MRS, pData->EmcWarmBootMrsExtra);
                }
            } else if (pData->MemoryType == NvBootMemoryType_LpDdr4)
            {
		        if (pData->EmcWarmBootExtraModeRegWriteEnable) {
                    HW_REGW(EmcBase, EMC, MRW2, pData->EmcMrw2);
                    HW_REGW(EmcBase, EMC, MRW, pData->EmcMrw1);
                    HW_REGW(EmcBase, EMC, MRW3, pData->EmcMrw3);
                    HW_REGW(EmcBase, EMC, MRW4, pData->EmcMrw4);
    	            HW_REGW(EmcBase, EMC, MRW6, pData->EmcMrw6);
    		        HW_REGW(EmcBase, EMC, MRW14, pData->EmcMrw14);
                    HW_REGW(EmcBase, EMC, MRW, pData->EmcWarmBootMrwExtra);
                    HW_REGW(EmcBase, EMC, MRW8, pData->EmcMrw8);
                    HW_REGW(EmcBase, EMC, MRW12, pData->EmcMrw12);
                    HW_REGW(EmcBase, EMC, MRW9, pData->EmcMrw9);
                    HW_REGW(EmcBase, EMC, MRW13, pData->EmcMrw13);
                 }
	    }

            #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
            // No need to integrate these to SW tree
            if (pData->MemoryType == NvBootMemoryType_Ddr2)
            {
                HW_REGW(EmcBase, EMC, EMRS2, pData->EmcEmrs2);
                HW_REGW(EmcBase, EMC, EMRS3, pData->EmcEmrs3);
                HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);
                HW_REGW(EmcBase, EMC, MRS,  pData->EmcMrs);
            } else if (pData->MemoryType == NvBootMemoryType_LpDdr ||
                       pData->MemoryType == NvBootMemoryType_Ddr) {
                HW_REGW(EmcBase, EMC, EMRS, pData->EmcEmrs);
                HW_REGW(EmcBase, EMC, MRS,  pData->EmcMrs);
            }
            #endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES
        }

        // Insert a burst refreshes.
        if (pData->EmcExtraRefreshNum > 0)
        {
            HW_REGW(EmcBase, EMC, REF, 
                    NV_DRF_NUM(EMC, REF, REF_CMD,    1) | /* Trigger bit */ 
                    NV_DRF_NUM(EMC, REF, REF_NORMAL, 1) |
                    NV_DRF_NUM(EMC, REF, REF_NUM,
                               (1 << pData->EmcExtraRefreshNum) - 1)|
                    NV_DRF_NUM(EMC, REF, REF_DEV_SELECTN, pData->EmcDevSelect));
        }

        // Enable refresh on both devices.
        HW_REGW(EmcBase, EMC, REFCTRL,
                NV_DRF_NUM(EMC, REFCTRL, DEVICE_REFRESH_DISABLE,
                           pData->EmcDevSelect) |
                NV_DRF_DEF(EMC, REFCTRL, REF_VALID, ENABLED));


        // one time ZQ calibration
        
        if (NV_DRF_VAL(EMCZCAL, BOOT_ENABLE, WARMBOOT,pData->EmcZcalWarmColdBootEnables)) // if (pData->EmcZCalWarmBootEnable) 
        {
            if (pData->MemoryType == NvBootMemoryType_LpDdr2)
            {
                // ZQ calibration on device 0
                RegData = NV_FLD_SET_DRF_NUM(EMC, MRW, MRW_DEV_SELECTN, 2,
                                             pData->EmcMrwLpddr2ZcalWarmBoot);
                HW_REGW(EmcBase, EMC, MRW, RegData);

                NvBootUtilWaitUS(pData->EmcZcalWarmBootWait);

                if ((pData->EmcDevSelect & 2) == 0)
                {
                    // ZQ calibration on device 1
                    RegData =
                        NV_FLD_SET_DRF_NUM(EMC, MRW, MRW_DEV_SELECTN, 1,
                                           pData->EmcMrwLpddr2ZcalWarmBoot);
                    HW_REGW(EmcBase, EMC, MRW, RegData);
    
                    NvBootUtilWaitUS(pData->EmcZcalWarmBootWait);
                }
            }
            else if (pData->MemoryType == NvBootMemoryType_Ddr3)
            {
                // ZQ calibration on device 0
                RegData = NV_FLD_SET_DRF_NUM(EMC, ZQ_CAL, ZQ_CAL_DEV_SELECTN, 2,
                                             pData->EmcZqCalDdr3WarmBoot);
                HW_REGW(EmcBase, EMC, ZQ_CAL, RegData);

                NvBootUtilWaitUS(pData->EmcZcalWarmBootWait);

                if ((pData->EmcDevSelect & 2) == 0)
                {
                    // ZQ calibration on device 1
                    RegData = 
                        NV_FLD_SET_DRF_NUM(EMC, ZQ_CAL, ZQ_CAL_DEV_SELECTN, 1,
                                           pData->EmcZqCalDdr3WarmBoot);
                    HW_REGW(EmcBase, EMC, ZQ_CAL, RegData);
    
                    NvBootUtilWaitUS(pData->EmcZcalWarmBootWait);
                }
            }
            else if (pData->MemoryType == NvBootMemoryType_LpDdr4)
            {

                RegData = NV_FLD_SET_DRF_NUM(EMC, MRW, MRW_DEV_SELECTN, 2,
                                             pData->EmcZqCalLpDdr4WarmBoot);

                HW_REGW(EmcBase, EMC, ZQ_CAL, RegData); // issue ZQCAL start
                NvBootUtilWaitUS(pData->EmcZcalWarmBootWait);
                HW_REGW(EmcBase, EMC, ZQ_CAL, (RegData^ 0x3)); //issue ZQCAL latch 
            
                if ((pData->EmcDevSelect & 2) == 0)
                {

                    RegData = NV_FLD_SET_DRF_NUM(EMC, MRW, MRW_DEV_SELECTN, 1,
                                             pData->EmcZqCalLpDdr4WarmBoot);
                    HW_REGW(EmcBase, EMC, ZQ_CAL, RegData);
                    NvBootUtilWaitUS(pData->EmcZcalWarmBootWait);
                    HW_REGW(EmcBase, EMC, ZQ_CAL, (RegData ^ 0x3)); //issue ZQCAL latch 
                }
            }
        }
    }
    else
    {
        switch(pData->MemoryType)
        {
            #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
            case NvBootMemoryType_Ddr:
                InitDdr(pData, EmcBase);
                break;

            case NvBootMemoryType_LpDdr:
                InitLpDdr(pData, EmcBase);
                break;
                
            case NvBootMemoryType_Ddr2:
                InitDdr2(pData, EmcBase);
                break;
            #endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES
                
            case NvBootMemoryType_Ddr3:
                InitDdr3(pData, EmcBase);
                break;
                
            case NvBootMemoryType_LpDdr2:
                InitLpDdr2(pData, EmcBase);
                break;

            case NvBootMemoryType_LpDdr4:
                InitLpDdr4(pData, EmcBase);
                break;
                
            default:
                break;
        }
    }

    if(pData->EmcBctSpareSecure18)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure18, pData->EmcBctSpareSecure19);
    if(pData->EmcBctSpareSecure20)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure20, pData->EmcBctSpareSecure21);
    if(pData->EmcBctSpareSecure22)
      NvBootMssSpareWrite(pData->EmcBctSpareSecure22, pData->EmcBctSpareSecure23);

    if( !IsWarmboot ) {
      // Set package and DPD pad control
      NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_CFG_0, pData->PmcDdrCfg);
    }

    // EMC_ZCAL_* regs only meaningful for LPDDR2, LPDDR3, and DDR3
    if (pData->MemoryType == NvBootMemoryType_LpDdr2 ||
        pData->MemoryType == NvBootMemoryType_Ddr3   ||
        pData->MemoryType == NvBootMemoryType_LpDdr4) 
    {
        // Start periodic ZQ calibration if ZCAL_INTERVAL is non zero
        HW_REGW(EmcBase, EMC, ZCAL_INTERVAL, pData->EmcZcalInterval);
        HW_REGW(EmcBase, EMC, ZCAL_WAIT_CNT, pData->EmcZcalWaitCnt);

        HW_REGW(EmcBase, EMC, ZCAL_MRW_CMD, pData->EmcZcalMrwCmd);
    } 

    // Patch 7 using BCT Spare Variables
    if(pData->EmcBctSpare12)
      NvBootMssSpareWrite(pData->EmcBctSpare12, pData->EmcBctSpare13); 

    // Re-trigger the Timing value after writing ZCAL*
    HW_REGW(EmcBase, EMC, TIMING_CONTROL, 1); // Trigger - just needs non-zero arg

    if( !IsWarmboot ) {
        // Insert a burst refreshes.
        if (pData->EmcExtraRefreshNum > 0)
        {
            HW_REGW(EmcBase, EMC, REF, 
                    NV_DRF_NUM(EMC, REF, REF_CMD,    1) | /* Trigger bit */ 
                    NV_DRF_NUM(EMC, REF, REF_NORMAL, 1) |
                    NV_DRF_NUM(EMC, REF, REF_NUM,
                               (1 << pData->EmcExtraRefreshNum) - 1)|
                    NV_DRF_NUM(EMC, REF, REF_DEV_SELECTN, pData->EmcDevSelect));
        }

        // Enable refresh on both devices.
        HW_REGW(EmcBase, EMC, REFCTRL,
                NV_DRF_NUM(EMC, REFCTRL, DEVICE_REFRESH_DISABLE,
                           pData->EmcDevSelect) |
                NV_DRF_DEF(EMC, REFCTRL, REF_VALID, ENABLED));
    }
    // Note: Programming CFG must happen after REFCTRL to delay active power-
    //       down to after init. (DDR2 constraint).
    HW_REGW(EmcBase, EMC, DYN_SELF_REF_CONTROL,   pData->EmcDynSelfRefControl);
    HW_REGW(EmcBase, EMC, CFG,                    pData->EmcCfg);
	HW_REGW(EmcBase, EMC, FDPD_CTRL_DQ,  pData->EmcFdpdCtrlDq);
	HW_REGW(EmcBase, EMC, FDPD_CTRL_CMD,  pData->EmcFdpdCtrlCmd);
    HW_REGW(EmcBase, EMC, SEL_DPD_CTRL,           pData->EmcSelDpdCtrl);
    
    // rest of the register has been already been written before -- addr swizzle lock bit written at end of sequence. Since this register needs a timing update to apply, moved before the last timing update.
    HW_REGW(EmcBase, EMC, FBIO_SPARE, NV_FLD_SET_DRF_NUM(EMC, FBIO_SPARE, CFG_ADR_EN, 1,  pData->EmcFbioSpare));

    // Re-trigger the Timing value to latch power saving functions
    HW_REGW(EmcBase, EMC, TIMING_CONTROL, 1); // Trigger - just needs non-zero arg

    HW_REGW(EmcBase, EMC, CFG_UPDATE,             pData->EmcCfgUpdate);
    // Enable EMC pipe clock gating 
    HW_REGW(EmcBase, EMC, CFG_PIPE_CLK, pData->EmcCfgPipeClk);

    // Depending on frequency, enable CMD/CLK fdpd 
    HW_REGW(EmcBase, EMC, FDPD_CTRL_CMD_NO_RAMP, pData->EmcFdpdCtrlCmdNoRamp);

    // Write out all access ctrl lock bits at the end of the sequence
    HW_REGW(McBase, MC, UNTRANSLATED_REGION_CHECK, pData->McUntranslatedRegionCheck);
    HW_REGW(McBase, MC, VIDEO_PROTECT_REG_CTRL    ,pData->McVideoProtectWriteAccess);
    HW_REGW(McBase, MC, SEC_CARVEOUT_REG_CTRL      ,pData->McSecCarveoutProtectWriteAccess); 
    HW_REGW(McBase, MC, MTS_CARVEOUT_REG_CTRL      ,pData->McMtsCarveoutRegCtrl); 

    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_BOM                           ,pData->McGeneralizedCarveout1Bom                   );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_BOM_HI                        ,pData->McGeneralizedCarveout1BomHi                 );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_SIZE_128KB                    ,pData->McGeneralizedCarveout1Size128kb             );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_ACCESS0                ,pData->McGeneralizedCarveout1Access0               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_ACCESS1                ,pData->McGeneralizedCarveout1Access1               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_ACCESS2                ,pData->McGeneralizedCarveout1Access2               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_ACCESS3                ,pData->McGeneralizedCarveout1Access3               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_ACCESS4                ,pData->McGeneralizedCarveout1Access4               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS0 ,pData->McGeneralizedCarveout1ForceInternalAccess0  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS1 ,pData->McGeneralizedCarveout1ForceInternalAccess1  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS2 ,pData->McGeneralizedCarveout1ForceInternalAccess2  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS3 ,pData->McGeneralizedCarveout1ForceInternalAccess3  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS4 ,pData->McGeneralizedCarveout1ForceInternalAccess4  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT1_CFG0                          ,pData->McGeneralizedCarveout1Cfg0                  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_BOM                           ,pData->McGeneralizedCarveout2Bom                   );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_BOM_HI                        ,pData->McGeneralizedCarveout2BomHi                 );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_SIZE_128KB                    ,pData->McGeneralizedCarveout2Size128kb             );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_ACCESS0                ,pData->McGeneralizedCarveout2Access0               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_ACCESS1                ,pData->McGeneralizedCarveout2Access1               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_ACCESS2                ,pData->McGeneralizedCarveout2Access2               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_ACCESS3                ,pData->McGeneralizedCarveout2Access3               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_ACCESS4                ,pData->McGeneralizedCarveout2Access4               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS0 ,pData->McGeneralizedCarveout2ForceInternalAccess0  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS1 ,pData->McGeneralizedCarveout2ForceInternalAccess1  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS2 ,pData->McGeneralizedCarveout2ForceInternalAccess2  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS3 ,pData->McGeneralizedCarveout2ForceInternalAccess3  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS4 ,pData->McGeneralizedCarveout2ForceInternalAccess4  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT2_CFG0                          ,pData->McGeneralizedCarveout2Cfg0                  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_BOM                           ,pData->McGeneralizedCarveout3Bom                   );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_BOM_HI                        ,pData->McGeneralizedCarveout3BomHi                 );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_SIZE_128KB                    ,pData->McGeneralizedCarveout3Size128kb             );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_ACCESS0                ,pData->McGeneralizedCarveout3Access0               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_ACCESS1                ,pData->McGeneralizedCarveout3Access1               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_ACCESS2                ,pData->McGeneralizedCarveout3Access2               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_ACCESS3                ,pData->McGeneralizedCarveout3Access3               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_ACCESS4                ,pData->McGeneralizedCarveout3Access4               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS0 ,pData->McGeneralizedCarveout3ForceInternalAccess0  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS1 ,pData->McGeneralizedCarveout3ForceInternalAccess1  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS2 ,pData->McGeneralizedCarveout3ForceInternalAccess2  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS3 ,pData->McGeneralizedCarveout3ForceInternalAccess3  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS4 ,pData->McGeneralizedCarveout3ForceInternalAccess4  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT3_CFG0                          ,pData->McGeneralizedCarveout3Cfg0                  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_BOM                           ,pData->McGeneralizedCarveout4Bom                   );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_BOM_HI                        ,pData->McGeneralizedCarveout4BomHi                 );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_SIZE_128KB                    ,pData->McGeneralizedCarveout4Size128kb             );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_ACCESS0                ,pData->McGeneralizedCarveout4Access0               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_ACCESS1                ,pData->McGeneralizedCarveout4Access1               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_ACCESS2                ,pData->McGeneralizedCarveout4Access2               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_ACCESS3                ,pData->McGeneralizedCarveout4Access3               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_ACCESS4                ,pData->McGeneralizedCarveout4Access4               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS0 ,pData->McGeneralizedCarveout4ForceInternalAccess0  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS1 ,pData->McGeneralizedCarveout4ForceInternalAccess1  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS2 ,pData->McGeneralizedCarveout4ForceInternalAccess2  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS3 ,pData->McGeneralizedCarveout4ForceInternalAccess3  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS4 ,pData->McGeneralizedCarveout4ForceInternalAccess4  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT4_CFG0                          ,pData->McGeneralizedCarveout4Cfg0                  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_BOM                           ,pData->McGeneralizedCarveout5Bom                   );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_BOM_HI                        ,pData->McGeneralizedCarveout5BomHi                 );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_SIZE_128KB                    ,pData->McGeneralizedCarveout5Size128kb             );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_ACCESS0                ,pData->McGeneralizedCarveout5Access0               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_ACCESS1                ,pData->McGeneralizedCarveout5Access1               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_ACCESS2                ,pData->McGeneralizedCarveout5Access2               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_ACCESS3                ,pData->McGeneralizedCarveout5Access3               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_ACCESS4                ,pData->McGeneralizedCarveout5Access4               );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS0 ,pData->McGeneralizedCarveout5ForceInternalAccess0  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS1 ,pData->McGeneralizedCarveout5ForceInternalAccess1  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS2 ,pData->McGeneralizedCarveout5ForceInternalAccess2  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS3 ,pData->McGeneralizedCarveout5ForceInternalAccess3  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS4 ,pData->McGeneralizedCarveout5ForceInternalAccess4  );
    HW_REGW(McBase, MC, SECURITY_CARVEOUT5_CFG0                          ,pData->McGeneralizedCarveout5Cfg0                  );

    // This register must be written last since the wb0_code_test stops once it hits this register
    HW_REGW(McBase, MC, EMEM_CFG_ACCESS_CTRL, 1); // lock emem_cfg registers. (this differs from emc_reg_calc, we don't lock for sim testing purposes)

    // SW should set this bit to access the SDRAM after memory has been
    // initialized.
    RegData =
        NV_READ32(NV_ADDRESS_MAP_AHB_ARBC_BASE + AHB_ARBITRATION_XBAR_CTRL_0);
    RegData = 
        NV_FLD_SET_DRF_NUM(
            AHB,
            ARBITRATION_XBAR_CTRL,
            MEM_INIT_DONE,
            pData->AhbArbitrationXbarCtrlMemInitDone,
            RegData);

    NV_WRITE32(NV_ADDRESS_MAP_AHB_ARBC_BASE + AHB_ARBITRATION_XBAR_CTRL_0,
               RegData);

}

void NvBootSdramInit(const NvBootSdramParams *pData)
{

    // SDRAM initialization
    DoSdramInit(pData, NV_FALSE);
}

void NvBootSdramInitWarmBoot0(const NvBootSdramParams *pData)
{
    // Deassert CLKBUF's SEL_DPD.
    //NvU32 ddrcntrl = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_CNTRL_0);
    //ddrcntrl = NV_RESETVAL(APBDEV_PMC, DDR_CNTRL);
    //ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, MEM0_ADDR0_CLK_SEL_DPD, 0, ddrcntrl);
    //ddrcntrl = NV_FLD_SET_DRF_NUM(APBDEV_PMC, DDR_CNTRL, MEM0_ADDR1_CLK_SEL_DPD, 0, ddrcntrl);
    //NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DDR_CNTRL_0, ddrcntrl);
    //NvBootUtilWaitUS(pData->PmcDdrCntrlWait);

    DoSdramInit(pData, NV_TRUE);
}


//TODO review 34b_adr: size is now 64bit
NvU32 NvBootSdramQueryTotalMB(NvBool skip_rw_test)
{
    NvU32 MemorySizeInMB;
    volatile NvU32 *SdramLocation = (NvU32*)(NVBOOT_BL_SDRAM_START + 0x100);
    NvU32 Temp;
    NvU32 Signature=0xaa55aa55;

    // This code is also natively compiled by wb0_code_test.  We cannot access 
    // any address we wish on Linux systems.
    if (!skip_rw_test) {

        // Make sure memory is initialized and return accordingly.
        // Store the original data to a temp variable
        Temp = *SdramLocation;

        // Write the location with the signature.
        *SdramLocation = Signature;

        // Read back the location to verify the signature.
        // If it doesn't match, the SDRAM has not been initialized - return 0.
        if (*SdramLocation != Signature) {
            return 0;
        }

        // Get back the orginal content.
        *SdramLocation = Temp;
    }

    // MC_EMEM_CFG register was initialized with the memory size in MB.
    MemorySizeInMB =
        NV_DRF_VAL(MC, EMEM_CFG, EMEM_SIZE_MB, HW_REGR(MC, MC, EMEM_CFG)) ;
    // return Memorysize in MB
    return MemorySizeInMB;
}

NvU64 NvBootSdramQueryTotalSize(NvBool skip_rw_test)
{
    NvU32 MemorySizeInKB;
    volatile NvU32 *SdramLocation = (NvU32*)(NVBOOT_BL_SDRAM_START + 0x100);
    NvU32 Temp;
    NvU32 Signature=0xaa55aa55;

    // This code is also natively compiled by wb0_code_test.  We cannot access 
    // any address we wish on Linux systems.
    if (!skip_rw_test) {
        // Make sure memory is initialized and return accordingly.
        // Store the original data to a temp variable
        Temp = *SdramLocation;

        // Write the location with the signature.
        *SdramLocation = Signature;

        // Read back the location to verify the signature.
        // If it doesn't match, the SDRAM has not been initialized - return 0.
        if (*SdramLocation != Signature) {
            return 0;
        }

        // Get back the orginal content.
        *SdramLocation = Temp;
    }

    // MC_EMEM_CFG register was initialized with the memory size in MB.
    MemorySizeInKB =
        (NV_DRF_VAL(MC, EMEM_CFG, EMEM_SIZE_MB, HW_REGR(MC, MC, EMEM_CFG))) << 10;
    //TODO review 34b_adr: size is now 64bit
    if (((NvU64)MemorySizeInKB << 10) > (NvU64) NV_U64C(NV_ADDRESS_MAP_EMEM_MAX_SIZE))
		return (NvU64) NV_U64C(NV_ADDRESS_MAP_EMEM_MAX_SIZE);

    //TODO review 34b_adr: size is now 64bit
    // return Memorysize in bytes
    return (NvU64)MemorySizeInKB << 10;
}

#undef MSS_LIST_CASE
#undef EMC_SPARE_BLACKLIST
#undef MC_SPARE_BLACKLIST
#undef PMC_SPARE_WHITELIST
#undef CAR_SPARE_WHITELIST
