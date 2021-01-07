/*
 * Copyright (c) 2006 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_pmc_int.h
 *
 * Support for PMC
 */

#ifndef INCLUDED_NVBOOT_PMC_INT_H
#define INCLUDED_NVBOOT_PMC_INT_H

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "nvboot_osc.h"
#include "nvboot_sdram_param.h"
#include "nvboot_pmc_scratch_map.h"

// Set of flags supported in Scratch 0
// The enum corresponds to the corresponding bit set
// This is a little bit tricky because only _RANGE is available
#define NVBOOT_PMC_FLAG_ID(f) (1 << (1?APBDEV_PMC_SCRATCH0_0_##f##_RANGE))

typedef enum
{  
    NvBootPmcFlagId_Wb0           = NVBOOT_PMC_FLAG_ID(WARM_BOOT0_FLAG),
    NvBootPmcFlagId_ForceRecovery = NVBOOT_PMC_FLAG_ID(FORCE_RECOVERY_FLAG),
    NvBootPmcFlagId_FailBack      = NVBOOT_PMC_FLAG_ID(BL_FAIL_BACK_FLAG),
    NvBootPmcFlagId_HaltAtWb0     = NVBOOT_PMC_FLAG_ID(BR_HALT_AT_WB0_FLAG),
    NvBootPmcFlagId_Force32       = 0x7fffffff
} NvBootPmcFlagId ;

#define NVBOOT_PMC_CHECK_FLAGID(FlagId) \
    NV_ASSERT( (FlagId == NvBootPmcFlagId_Wb0)           || \
               (FlagId == NvBootPmcFlagId_ForceRecovery) || \
               (FlagId == NvBootPmcFlagId_FailBack) || \
               (FlagId == NvBootPmcFlagId_HaltAtWb0) );

#if defined(__cplusplus)
extern "C"
{
#endif

// Disable read access to all secure scratch registers
#define NVBOOT_PMC_SECURE_SCRATCH_DISABLE_READ \
        (NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ,   ON))

// Disable write access to all secure scratch registers
#define NVBOOT_PMC_SECURE_SCRATCH_DISABLE_WRITE \
        (NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE,  ON))

// AP20 uses PMC secure scratch 0-3 for SSK
#define NVBOOT_PMC_SECURE_SCRATCH_DISABLE_READ_SSK \
      ( NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ0, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ1, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ2, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ3, ON) )

#define NVBOOT_PMC_SECURE_SCRATCH_DISABLE_WRITE_SSK \
      ( NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE0, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE1, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE2, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE3, ON) )

// SRK uses PMC secure scratch 4-7 for SRK
#define NVBOOT_PMC_SECURE_SCRATCH_DISABLE_READ_SRK \
      ( NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ4, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ5, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ6, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, READ7, ON) )

#define NVBOOT_PMC_SECURE_SCRATCH_DISABLE_WRITE_SRK \
      ( NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE4, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE5, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE6, ON) | \
        NV_DRF_DEF(APBDEV_PMC, SEC_DISABLE, WRITE7, ON) )

#define NVBOOT_PMC_BONDOUT_MIRROR_DISABLE_READ \
        (NV_DRF_DEF(APBDEV_PMC, BONDOUT_MIRROR_ACCESS, BREAD,  ON))

#define NVBOOT_PMC_BONDOUT_MIRROR_DISABLE_WRITE \
        (NV_DRF_DEF(APBDEV_PMC, BONDOUT_MIRROR_ACCESS, BWRITE, ON))

#define NVBOOT_PMC_SSK_SIZE (16)

/**
 * Control access to the PMC secure registers
 *
 * @param Bit map indicating read and write disable
 */
void
NvBootPmcDisableAccess(NvU32 DisableMap);

/**
 * Copy the SSK stored in the secure scratch registers to a specified buffer
 *
 * @param pKey pointer to the key as an array of bytes
 *
 * Trusted code, so no return value
 *
 */
void  
NvBootPmcGetSskFromSecureScratch( NvU8 *pSsk) ;    

/**
 * Copy the SSK stored in a specified buffer into the pmc secure registers
 *
 * @param pKey pointer to the key as an array of bytes
 *
 * Trusted code, so no return value
 *
 */
void  
NvBootPmcLoadSskIntoSecureScratch( NvU8 *pSsk) ;


/**
 * An utility function to check the value of flags stored in 
 * PMC scratch 0.
 *
 * @param enum identifying the flag to check
 *
 * @return the current flag value
 *
 */
NvBool 
NvBootPmcQueryFlag(NvBootPmcFlagId FlagId) ;

/**
 * An utility function to set the value of status flags stored in 
 * PMC scratch 0.
 *
 * @param enum identifying the flag to check
 *
 */
void
NvBootPmcSetFlag(NvBootPmcFlagId FlagId, NvBool FlagValue) ;

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
NvBootPmcIsPllpOverrideEnabled(void);

/**
 * Returns the oscillator frequency enum value stored in the PLLP override
 * register.
 *
 * @param NvBootClocksOscFreq The value of the stored oscillator frequency.
 * @return none
 */
NvBootClocksOscFreq 
NvBootPmcGetPllpOverrideOscFreq(void);

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
NvBootPmcSetPllpOverride(NvBool PllEnable, NvBootClocksOscFreq OscFreq);

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
NvBootPmcIsPlluOverrideEnabled(void);

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
NvBootPmcSetPlluOverride(NvBool PllEnable, NvBootClocksOscFreq OscFreq);

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
NvBootPmcIsPllmOverrideEnabled(void);

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
NvBootPmcSetPllmOverride(NvBool PllEnable, NvBootSdramParams *Params);

/**
 * Read the AES disable flag from the PMC registers.
 *
 * @retval Value of the AES disable flag.
 */
NvBool
NvBootPmcGetAesDisable(void);

/**
 * Write the AES disable flag from the PMC registers.
 *
 * @param NvBool Value to record in the AES disable flag
 */
void
NvBootPmcSetAesDisable(NvBool);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_PMC_INT_H
