/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_pmc.c - Implementation of pmc support.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "arclk_rst.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_sdram_param.h"
#include "nvboot_util_int.h"
#include "project.h"

/*
 * Note that each bit is sticky, so one can call this routine
 * multiple times setting different bit(s) each time.
 * There is no need to Read-Modify-Write.
 */
void
NvBootPmcDisableAccess(NvU32 DisableMap)
{
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE_0, DisableMap);
}

void
NvBootPmcGetSskFromSecureScratch( NvU8 *pSsk) {
    NvU32 RegData;
    NvU32 i;
    NvU32 RegAddress;

    NV_ASSERT(pSsk != NULL) ;

    RegAddress = NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH0_0;
    for (i = 0, RegData = 0; i < NVBOOT_PMC_SSK_SIZE; i++) {
        if ( (i&3) == 0) {
            RegData = NV_READ32(RegAddress) ;
            RegAddress += 4 ;
        }
        pSsk[i] = RegData & 0xFF ;
        RegData >>= 8 ;
    }
}

void
NvBootPmcLoadSskIntoSecureScratch( NvU8 *pSsk) {
    NvU32 RegData;
    NvU32 RegAddress;
    NvU32 i;

    NV_ASSERT(pSsk != NULL);
    NV_ASSERT(NVBOOT_PMC_SSK_SIZE % sizeof(NvU32) == 0);

    RegAddress = NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH0_0;
    for (i=0; i<NVBOOT_PMC_SSK_SIZE; i+=4) {
        RegData = (pSsk[i+3]<<24) | (pSsk[i+2]<<16) | (pSsk[i+1]<<8) | pSsk[i];
        NV_WRITE32(RegAddress, RegData) ;
        RegAddress += 4 ;
    }
}

// trivial as the enum is the flag offset
NvBool
NvBootPmcQueryFlag(NvBootPmcFlagId FlagId) {
    NvU32 data;

    NVBOOT_PMC_CHECK_FLAGID(FlagId);

    data = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH0_0);

    // Ensure the return value is a valid NvBool value.
    return ((data & (NvU32)FlagId) ? NV_TRUE : NV_FALSE);
}

// trivial as the enum is the flag offset
void
NvBootPmcSetFlag(NvBootPmcFlagId FlagId, NvBool FlagValue) {
    NvU32 RegData ;

    NVBOOT_PMC_CHECK_FLAGID(FlagId) ;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH0_0) ;
    if (FlagValue) {
        RegData |=   (int) FlagId ;
    }
    else {
        RegData &= ~ (int) FlagId ;
    }
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH0_0, RegData);
}

NvBool
NvBootPmcGetAesDisable(void)
{
    return NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CRYPTO_OP_0);
}

void
NvBootPmcSetAesDisable(NvBool Disable)
{
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CRYPTO_OP_0, Disable);
}

// this code is mapped in the non secure part of the ROM
#ifdef __arm__
// NOTE: This pragma only makes sense when building for the ARM compilers.
#pragma arm section rodata = "PmcNonsecure", code = "PmcNonsecure"
#endif

/**
 * A utility function to query whether the PLLP override was enabled.
 *
 * It returns true if both the override function is enabled and PLLP
 * was enabled within the override register.
 *
 * @param none
 * @return NvBool Whether the PLLP override was enabled.
 */
NvBool
NvBootPmcIsPllpOverrideEnabled(void)
{
    NvU32 RegData;
    NvBool Enabled;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    Enabled =
      NV_DRF_VAL(APBDEV_PMC, PLLP_WB0_OVERRIDE, PLLP_OVERRIDE_ENABLE, RegData) &&
      NV_DRF_VAL(APBDEV_PMC, PLLP_WB0_OVERRIDE, PLLP_ENABLE,     RegData);

    return Enabled;
}

/**
 * Returns the oscillator frequency enum value stored in the PLLP override
 * register.
 *
 * @param NvBootClocksOscFreq The value of the stored oscillator frequency.
 * @return none
 */
NvBootClocksOscFreq
NvBootPmcGetPllpOverrideOscFreq(void)
{
    NvU32 RegData;
    NvBootClocksOscFreq OscFreq;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    OscFreq = (NvBootClocksOscFreq)NV_DRF_VAL(APBDEV_PMC,
                                              PLLP_WB0_OVERRIDE,
                                              OSC_FREQ,
                                              RegData);

    return OscFreq;
}

/**
 * Setup the Boot ROM side of the PLLP override.  The Boot ROM controls
 * the enable bit and stores the oscillator frequency.
 * Note that BL, OS, or LP0 entry code is responsible for setting the
 * override enable.
 *
 * @param NvBool Whether to enable PLLP if the override mechanism triggers.
 * @param NvBootClocksOscFreq The value of the measured oscillator frequency.
 * @retval none
 */
void
NvBootPmcSetPllpOverride(NvBool PllEnable, NvBootClocksOscFreq OscFreq)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                 PLLP_WB0_OVERRIDE,
                                 PLLP_ENABLE,
                                 PllEnable,
                                 RegData);

    // Add programming of PLLP_OVERRIDE_ENABLE to fully enable
    // PLLP WB0 auto restart but use a SW_CYA as well. 
    if(NvBootGetSwCYA() & NVBOOT_SW_CYA_PLLP_OVERRIDE_ENABLE)
    {
        RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                     PLLP_WB0_OVERRIDE,
                                     PLLP_OVERRIDE_ENABLE,
                                     1,
                                     RegData);
    }

    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                    PLLP_WB0_OVERRIDE,
                    PLL_REF_DIV,
                    CLK_RST_CONTROLLER_OSC_CTRL_0_PLL_REF_DIV_DIV1,
                    RegData);
    
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                 PLLP_WB0_OVERRIDE,
                                 OSC_FREQ,
                                 OscFreq,
                                 RegData);

    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLP_WB0_OVERRIDE_0,
               RegData);
}

/**
 * A utility function to query whether the PLLU override was enabled.
 *
 * It returns true if both the override function is enabled and PLLU
 * was enabled within the override register.
 *
 * @param none
 * @return NvBool Whether the PLLU override was enabled.
 */
NvBool
NvBootPmcIsPlluOverrideEnabled(void)
{
    NvU32 RegData;
    NvBool Enabled;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    Enabled =
      NV_DRF_VAL(APBDEV_PMC, PLLP_WB0_OVERRIDE, PLLU_OVERRIDE_ENABLE, RegData) &&
      NV_DRF_VAL(APBDEV_PMC, PLLP_WB0_OVERRIDE, PLLU_ENABLE,     RegData);

    return Enabled;
}

/**
 * Setup the Boot ROM side of the PLLU override.  The Boot ROM controls
 * the enable bit and stores the oscillator frequency.
 * Note that BL, OS, or LP0 entry code is responsible for setting the
 * override enable.
 *
 * @param NvBool Whether to enable PLLU if the override mechanism triggers.
 * @param NvBootClocksOscFreq The value of the measured oscillator frequency.
 * @retval none
 */
void
NvBootPmcSetPlluOverride(NvBool PllEnable, NvBootClocksOscFreq OscFreq)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                 PLLP_WB0_OVERRIDE,
                                 PLLU_ENABLE,
                                 PllEnable,
                                 RegData);

    // Add programming of PLLU_OVERRIDE_ENABLE to fully enable
    // PLLP WB0 auto restart but use a SW_CYA as well. 
    if(NvBootGetSwCYA() & NVBOOT_SW_CYA_PLLU_OVERRIDE_ENABLE)     
    {
        RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                     PLLP_WB0_OVERRIDE,
                                     PLLU_OVERRIDE_ENABLE,
                                     1,
                                     RegData);
    } 

    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                 PLLP_WB0_OVERRIDE,
                                 OSC_FREQ,
                                 OscFreq,
                                 RegData);

    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLP_WB0_OVERRIDE_0,
               RegData);
}

/**
 * A utility function to query whether the PLLM override was enabled.
 *
 * It returns true if both the override function is enabled and PLLM
 * was enabled within the override register.
 *
 * @param none
 * @return NvBool Whether the PLLM override was enabled.
 */
NvBool
NvBootPmcIsPllmOverrideEnabled(void)
{
    NvU32 RegData;
    NvBool Enabled;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    Enabled =
      NV_DRF_VAL(APBDEV_PMC, PLLP_WB0_OVERRIDE, PLLM_OVERRIDE_ENABLE, RegData) &&
      NV_DRF_VAL(APBDEV_PMC, PLLP_WB0_OVERRIDE, PLLM_ENABLE,     RegData);

    return Enabled;
}

/**
 * Setup the Boot ROM side of the PLLM override.  The Boot ROM controls
 * the enable bit and stores the oscillator frequency.
 * Note that BL, OS, or LP0 entry code is responsible for setting the
 * override enable.
 *
 * @param NvBool Whether to enable PLLM if the override mechanism triggers.
 * @param NvBootSdramParams A pointer to the BCT SDRAM parameters. 
 * @retval none
 */
void
NvBootPmcSetPllmOverride(NvBool PllEnable, NvBootSdramParams *Params)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                        APBDEV_PMC_PLLP_WB0_OVERRIDE_0);

    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                 PLLP_WB0_OVERRIDE,
                                 PLLM_ENABLE,
                                 PllEnable,
                                 RegData);

    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLP_WB0_OVERRIDE_0,
               RegData);

    // No need to program the OSC freq for PLLM, the individual PLL setting
    // is fully programmable via the PLLM_WB0_OVERRIDE_FREQ_0 register
    // FIXME: Review PLLM auto-restart programming for T114
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLM_WB0_OVERRIDE2_0);
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, PLLM_WB0_OVERRIDE2,
                SETUP,
                Params->PllMSetupControl,               // Setup
                RegData);
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, PLLM_WB0_OVERRIDE2,
                KCP,
                Params->PllMKCP,                        // KCP
                RegData);
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, PLLM_WB0_OVERRIDE2,
                KVCO,
                Params->PllMKVCO,                       // KVCO
                RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLM_WB0_OVERRIDE2_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLM_WB0_OVERRIDE_FREQ_0);
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, 
                                PLLM_WB0_OVERRIDE_FREQ, 
                                PLLM_DIVM,
                                Params->PllMInputDivider,           // M
                                RegData);
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, 
                                PLLM_WB0_OVERRIDE_FREQ, 
                                PLLM_DIVN,
                                Params->PllMFeedbackDivider,        // N
                                RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLM_WB0_OVERRIDE_FREQ_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLM_WB0_OVERRIDE2_0);
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                PLLM_WB0_OVERRIDE2,
                                DIVP,
                                Params->PllMPostDivider,            // P
                                RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLM_WB0_OVERRIDE2_0, RegData);
    
    // Add programming of PLLM_OVERRIDE_ENABLE to fully enable
    // PLLP WB0 auto restart but use a SW_CYA as well. 
    if(NvBootGetSwCYA() & NVBOOT_SW_CYA_PLLM_OVERRIDE_ENABLE)
    {
        RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                            APBDEV_PMC_PLLP_WB0_OVERRIDE_0);
        RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                     PLLP_WB0_OVERRIDE,
                                     PLLM_OVERRIDE_ENABLE,
                                     1,
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_PLLP_WB0_OVERRIDE_0,
                   RegData);
    }
    
    return;
}
#ifdef __arm__
// NOTE: This pragma only makes sense when building for the ARM compilers.
#pragma arm section
#endif
