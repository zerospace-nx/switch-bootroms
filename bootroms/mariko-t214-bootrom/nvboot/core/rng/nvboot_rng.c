/* Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 *  @file nvboot_rng.c
 *
 *  Random number generator.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "project.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "nvboot_rng_int.h"
#include "nvboot_platform_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_bpmp_int.h"
#include "nvboot_rng_hw.h"


/// RngState Default Disabled.
NvBootRngState RngState=RNG_DISABLED; 
 
/**
 *  @brief Generate Random Number in the range 0 - (Max-1)
 *  @param Max Upper limit of random number. Should be power of 2. Will be enforced by routine 
 *         regardless.
 *  @return Random Number 0 - (Max-1)
 */
NvU32 NvBootRngGenerateRandom(NvU32 Max)
{
    NvU32 Rand;
    
    if(RngState == RNG_DISABLED)
    {
        return 0;
    }
    
    // Minimum range is 0-1
    Max = NV_MAX(Max,2);
  
    // Enforce power of 2.
    if(Max&(Max-1)) // Not a power of 2.
        Max = (1 << NvBootUtilGetLog2Number(Max));
        
    Rand = NvBootSeRngGenerateRandom();
    
    return Rand &(Max-1);
}


/**
 *  @brief Wait Random
 *  @param MaxDelay Delay in Range 0 - MaxDelay-1
 *  @return void
 */
void NvBootRngWaitRandom(NvU32 MaxDelay)
{
    NvU32 Rand;
    
    if(RngState == RNG_DISABLED)
    {
        return;
    }
    // Can't afford to slow down rtl sims
    if(NvBootIsPlatformRtl())
    {
        Rand=0;
    }
    else
    {
        Rand = NvBootRngGenerateRandom(MaxDelay);
    }
    NvBootUtilWaitUS(Rand);
}

/**
 *  @brief Initialize RNG
 *  @return NvBootError
 */
NvBootError NvBootRngInit(void)
{
    volatile NvBootError e = NvBootError_NotInitialized;
    NvU32 CyaComplement;
    NvU32 Cya;
    Cya = NvBootGetSwCYA() & NVBOOT_SW_CYA_RNG_DISABLE;
    
    /// Entropy Ring Oscillators not synthesized on fpga.
    /// http://nvbugs/1859832
    if(Cya)
    {
        RngState = RNG_DISABLED;
        e = NvBootError_Success;

        /// RNG is disabled either because of a CYA or FPGA
        /// If not both, then there was a glitch.
        CyaComplement = (~NvBootGetSwCYA()) & NVBOOT_SW_CYA_RNG_DISABLE;
        if(CyaComplement)
        {
            e = NvBootError_NotInitialized;
        }
    }
    else
    {
        e = NvBootSeRngHwInit(&RngState);
    }
 
    return e;
}

/**
 *  @brief Wait Random
 *  @param MaxCycles in Range 0 - MaxCycles-1
 *  @return void
 */
void NvBootRngWaitRandomLoop(NvU32 MaxCycles)
{
    NvU32 Rand;
    volatile NvBootError e = NvBootError_Fault_Injection_Detection;
    
    if(RngState == RNG_DISABLED)
    {
        return;
    }
    // Can't afford to slow down rtl sims
    if(NvBootIsPlatformRtl())
    {
        Rand=0;
    }
    else
    {
        Rand = NvBootRngGenerateRandom(MaxCycles);
    }

    e = NvBootUtilInstrWait(Rand);
    
    // An error from above routine is a sign of glitching. Lock down and sit tight.
    if(e != NvBootError_Success)
        do_exception();

}