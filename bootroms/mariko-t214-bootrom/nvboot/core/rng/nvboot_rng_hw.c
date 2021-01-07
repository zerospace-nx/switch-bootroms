/* Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 *  @file nvboot_rng_hw.c
 *
 *  SE Random number generator h/w routines.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arse.h"
#include "project.h"
#include "nvboot_clocks_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_se_int.h"
#include "nvboot_pka_int.h"
#include "nvboot_ssk_int.h"
#include "nvboot_util_int.h"
#include "nvboot_crypto_sha_param.h"
#include "nvboot_rng_hw.h"
#include "nvboot_rng_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_platform_int.h"


/**
 *  @brief SE Random Number Generator Operations
 *  @param RngOp Instantitate, Generate or Reseed.
 *         Instantiate      Seeds RNG with Entropy/Nonce/Personalization String from ECID. Also returns random number
 *         Generate         Generate Random Number
 *         Force_reseed     Force_reseed. Will be forced if Reseed_Cntr_Exhausted
 *  @return 32 bit Random Number
 */
NvU32 NvBootSeRngOperation(NvBootSeRngOp RngOp)
{
    SingleSeLinkedList OutputLL;

    NvU32 RegData;
    NvU32   L[NVBOOT_SE_AES_BLOCK_LENGTH];

    /// If SE is disabled, return 0
    if(!NvBootSeInstanceIsEngineEnabled(NvBootSeInstance_Se1))
    {
        return 0;
    }
    
    /// Force reseed if counter is exhausted.
    RegData = NvBootGetSeReg(SE_INT_STATUS_0);
    if(NV_DRF_VAL(SE, INT_STATUS, RESEED_CNTR_EXHAUSTED, RegData))
    {
        RngOp = FORCE_RESEED;
        NvBootSetSeReg(SE_INT_STATUS_0, NV_DRF_DEF(SE, INT_STATUS, RESEED_CNTR_EXHAUSTED, SW_CLEAR));
    }
    
    /// Set configuration for RNG seeded with H/W Entropy and Random number destination=Memory
    RegData = 0;
    RegData = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, RNG, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, NOP, RegData);
    RegData = NV_FLD_SET_DRF_NUM(SE, CONFIG, ENC_MODE, SE_MODE_PKT_AESMODE_KEY128, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, MEMORY, RegData);
    NvBootSetSeReg(SE_CONFIG_0, RegData);
    
    RegData = 0;
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, XOR_POS, BYPASS, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, INPUT_SEL, RANDOM, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, HASH_ENB, DISABLE, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, CORE_SEL, ENCRYPT, RegData);
    NvBootSetSeReg(SE_CRYPTO_CONFIG_0, RegData);
    
    /// Setup the mode
    
    RegData = NvBootGetSeReg(SE_RNG_CONFIG_0);
    RegData = NV_FLD_SET_DRF_NUM(SE, RNG_CONFIG, MODE,  RngOp, RegData);
    NvBootSetSeReg(SE_RNG_CONFIG_0, RegData);
    
    OutputLL.LastBufferNumber = 0;
    OutputLL.LLElement.StartByteAddress = (NvU32) &L[0];
    OutputLL.LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
    
    NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &OutputLL);
    
    /// SE_CRYPTO_LAST_BLOCK_0 = Number of random vectors required - 1 
    NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, 0);
    
    /**
     * Issue START command in SE_OPERATION.OP
     */
     RegData = NV_DRF_DEF(SE, OPERATION, OP, START);
     NvBootSetSeReg(SE_OPERATION_0, RegData);

    // Poll for OP_DONE.
     while(NvBootSeIsEngineBusy((NvU8*)L))
         ;
    
    return L[0]; 
}


/**
 *  @brief Initialize SE RNG
 *  @param RngState Pass back RngState.
 *         It is possible for RngState to be disabled yet not an error (INT boards) 
 *  @return NvBootError
 */
NvBootError NvBootSeRngHwInit(NvBootRngState *RngState)
{
    NvU32 RegData, Divider, Src;
    NvBootError e = NvBootError_NotInitialized;
    
    /**
     *  FUSE_JTAG_SECUREID_VALID_0 needs to be set for SE RNG to use entropy.
     */
    if(NvBootFuseIsJtagSecureIdFuseSet() == NV_FALSE)
    {
        *RngState = RNG_DISABLED;
            return NvBootError_Success;
        }
    
    /** 
     *  SE clock might have been brought up already but just in case, enable and de-assert Reset.
     *  Do not assert Reset!
     */
    /// Enable clock to the controller
    NvBootClocksSetEnable(NvBootClocksClockId_SeId, NV_TRUE);

    /// De-assert Reset
    NvBootResetSetEnable(NvBootResetDeviceId_SeId, NV_FALSE);

    /// Get SE clock divider
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_SE_0);
    Divider = NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_SE, SE_CLK_DIVISOR, RegData);
    Src = NV_DRF_VAL(CLK_RST_CONTROLLER, CLK_SOURCE_SE, SE_CLK_SRC, RegData);
    /// Set Entropy clk to same frequency and source as SE.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_ENTROPY_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_ENTROPY, ENTROPY_CLK_DIVISOR, Divider, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_ENTROPY, ENTROPY_CLK_SRC, Src, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_ENTROPY_0, RegData);
    
    /// Enable Entropy source. (Not sticky)
    RegData = NvBootGetSeReg(SE_RNG_SRC_CONFIG_0);
    RegData = NV_FLD_SET_DRF_DEF(SE, RNG_SRC_CONFIG, RO_ENTROPY_SOURCE, ENABLE, RegData);
    NvBootSetSeReg(SE_RNG_SRC_CONFIG_0, RegData);

    /// Set configuration for RNG seeded with H/W Entropy and Random number destination=Memory
    RegData = 0;
    RegData = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, RNG, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, NOP, RegData);
    RegData = NV_FLD_SET_DRF_NUM(SE, CONFIG, ENC_MODE, SE_MODE_PKT_AESMODE_KEY128, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, MEMORY, RegData);
    NvBootSetSeReg(SE_CONFIG_0, RegData);
    
    RegData = 0;
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, XOR_POS, BYPASS, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, INPUT_SEL, RANDOM, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, HASH_ENB, DISABLE, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, CORE_SEL, ENCRYPT, RegData);
    NvBootSetSeReg(SE_CRYPTO_CONFIG_0, RegData);
    
    NvBootSetSeReg(SE_RNG_RESEED_INTERVAL_0, 0);
    
    // CYA in case Entropy is busted.
    NvU32 Cya = NvBootGetSwCYA() & NVBOOT_SW_CYA_RNG_SRC_LFSR;
    RegData = NvBootGetSeReg(SE_RNG_CONFIG_0);
    if(Cya)
    {
        RegData = NV_FLD_SET_DRF_DEF(SE, RNG_CONFIG, SRC, LFSR, RegData);
    }
    else
    {
    RegData = NV_FLD_SET_DRF_DEF(SE, RNG_CONFIG, SRC, ENTROPY, RegData);
    }

    NvBootSetSeReg(SE_RNG_CONFIG_0, RegData);
    
    /**
     *  Bypass VN filter on fpga. FPGA entropy quality is likely bad so the filter will discard and 
     *  RNG operation won't complete. 
     */
    if(NvBootIsPlatformFpga())
    {
        RegData = NvBootGetSeReg(SE_MISC_0);
        RegData = NV_FLD_SET_DRF_DEF(SE, MISC, ENTROPY_VN_BYPASS, ENABLE, RegData);
        NvBootSetSeReg(SE_MISC_0, RegData);
    }
    
    NvBootSeRngOperation(INSTANTIATE);
    
    e = NvBootError_Success;
    *RngState = RNG_ENABLED;

    return e;
}