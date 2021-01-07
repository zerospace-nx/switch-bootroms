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
 * nvboot_warmboot0.c - Implementation of WarmBoot0 functions.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arapb_misc.h"
#include "arclk_rst.h"
#include "aremc.h"
#include "armc.h"
#include "arrtc.h"
#include "arse.h"
#include "artimerus.h"
#include "nvboot_clocks_int.h"
#include "nvboot_config_int.h"
#include "nvboot_error.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_hash_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_reset_int.h"
#include "nvboot_sdram_int.h"
#include "nvboot_sdram_param.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_int.h"
#include "nvboot_se_rsa.h"
#include "nvboot_util_int.h"
#include "nvboot_warm_boot_0.h"
#include "nvboot_warm_boot_0_int.h"
#include "nvboot_bct.h"
#include "project.h"

/**
 * SCRATCHNAME - Compose the scratch field name
 *
 *   @param n scratch register number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 */
#define SCRATCHNAME(n, d, r, f) \
    APBDEV_PMC_SCRATCH##n##_0_##d##_##r##_0_##f##_RANGE

/**
 * PMC_TO_REG -  Set the register field from the pmc register field.
 *
 *   @param n scratch register number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 *   @param e element of SDRAM parameter structure
 *
 * 1. Extract field value from given source register value
 *       t = NV_DRF_VAL(PMC, SCRATCHn, d_r_0_f, RegVal);
 * 2. Set the specified field in the destination register
 *       pData->e = NV_FLD_SET_DRF_NUM(d, r, f, t, pData->e);
 */
#define PMC_TO_REG(n, d, r, f, e)                                           \
    do {                                                                    \
       pData->e = NV_FLD_SET_DRF_NUM(d, r, f,                               \
                                     NV_DRF_VAL(APBDEV_PMC, SCRATCH##n,     \
                                                d##_##r##_0_##f, RegVal),   \
                                     pData->e);                             \
    } while (0)

/* Shorthand for several sets of parameters, all build on PMC_TO_REG(). */

#define PMC_TO_FBIO(n, r, f, e) PMC_TO_REG(n, EMC, FBIO_##r, f, EmcFbio##e)

#define PMC_TO_EMC(n, r, f, e) PMC_TO_REG(n, EMC, r, f, Emc##e)

#define PMC_TO_MC(n, r, f, e)  PMC_TO_REG(n, MC,  r, f, Mc##e)

#define PMC_TO_PAD(n, rp, fs, e) \
    PMC_TO_REG(n, APB_MISC_GP, rp##PADCTRL, CFG2TMC_##rp##fs,  \
               ApbMiscGp##e##PadCtrl)

#define PMC_TO_PAD2(n, rp, fs, e) \
    PMC_TO_REG(n, APB_MISC_GP, rp##PADCTRL2, CFG2TMC_##rp##fs, \
               ApbMiscGp##e##PadCtrl2)

#define PMC_TO_PLL(n, p, f, e) \
    pData->Pll##p##e = NV_DRF_VAL(APBDEV_PMC, SCRATCH##n,      \
                                  CLK_RST_PLL##p##_##f, RegVal)

#define PMC_TO_TYPE(n, f, t, e) \
    pData->e = (t)NV_DRF_VAL(APBDEV_PMC, SCRATCH##n, f, RegVal)

#define PMC_TO_VAL(n, f, e) \
    pData->e = NV_DRF_VAL(APBDEV_PMC, SCRATCH##n, f, RegVal)

#pragma arm section zidata = "NonZIBuffer"
NV_ALIGN(16) NvBootWb0RecoveryHeader gs_Wb0Header;
// restoring the section mapping
#pragma arm section
static NvU32 gs_RecoveryCodeHeaderStart;
static NvBootObjectSignature gs_ComputedHash;
//structure holds data needed for MC/EMC initialization.
extern NvBootConfigTable BootConfigTable;
static NvBootSdramParams *s_sdRamParamData = (NvBootSdramParams*)(&(BootConfigTable.SdramParams[0]));
/*
 * Compile-time assertion that the header size is a multiple of 16 bytes.
 */
NV_CT_ASSERT((sizeof(NvBootWb0RecoveryHeader) & 0xf) == 0);

static void StartRecoveryCodeHash(void);
static NvBootError ValidateRecoveryCode(void);
static void StartRecoveryCodeDecrypt(void);
static NvBootError Wb0ReadRecoveryCodeHeader(void);
static NvBootError ProcessRecoveryCode(void);

static NvBootError
ValidateRecoveryCode(void)
{
    NvU32  i;
    NvU32  PaddingSize;
    NvU8  *PaddingStart;

    NvU32  *Src;
    NvU32   Offset; 
    NvU32   RecoveryCodeSizeToVerify;
    NvBootError e;
    NvBootFuseOperatingMode Mode;

    NvBootFuseGetOperatingMode(&Mode);

    if(Mode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
    {
        // Start address of the message to be verified. 
        Src = (NvU32*)(&((NvBootWb0RecoveryHeader*)gs_RecoveryCodeHeaderStart)->
                  RandomAesBlock);        

        // Offset to the start of the signed section of the whole recovery
        // code. 
        Offset = (NvU8*)&(gs_Wb0Header.RandomAesBlock) - (NvU8*)&gs_Wb0Header;

        // The total recovery code size including the signed section of the
        // recovery code header. 
        RecoveryCodeSizeToVerify = gs_Wb0Header.LengthInsecure - Offset;
    
        // Begin the RSA-PSS-VERIFY computation. The public key should have
        // been loaded into the SE in ProcessRecoveryCode(). 
        e = NvBootSeRsaPssSignatureVerify(SE_RSA_KEY_PKT_KEY_SLOT_ONE, 
                                          ARSE_RSA_MAX_EXPONENT_SIZE, 
                                          Src, 
                                          NULL,
                                          RecoveryCodeSizeToVerify,
                                          (NvU32 *) &gs_Wb0Header.Signature.RsaPssSig.Signature[0],
                                          SE_MODE_PKT_SHAMODE_SHA256,
                                          ARSE_SHA256_HASH_SIZE / 8);
        
        // Now handle any error from ProcessRecoveryCode().
        if(e)
        {
            return NvBootError_HashMismatch;
        }
    }
    else 
    {
        // Check the hash.
        for (i = 0; i < NVBOOT_SE_AES_BLOCK_LENGTH; i++)
        {
            if (gs_ComputedHash.CryptoHash.hash[i] != gs_Wb0Header.Signature.CryptoHash.hash[i])
            {
                return NvBootError_HashMismatch;
            }
        }
    }

    if (gs_Wb0Header.LengthInsecure != gs_Wb0Header.LengthSecure)
    {
        return NvBootError_ValidationFailure;
    }

    if (gs_Wb0Header.RecoveryCodeLength >
        (gs_Wb0Header.LengthSecure - sizeof(NvBootWb0RecoveryHeader)))
    {
        return NvBootError_ValidationFailure;
    }

    // Check Recovery code padding
    PaddingStart = (NvU8*)((gs_Wb0Header.Destination) +
                           gs_Wb0Header.RecoveryCodeLength);

    PaddingSize = gs_Wb0Header.LengthSecure - gs_Wb0Header.RecoveryCodeLength -
        sizeof(NvBootWb0RecoveryHeader);

    if (!NvBootUtilIsValidPadding(PaddingStart, PaddingSize))
    {
        return NvBootError_ValidationFailure;
    }
    if((gs_Wb0Header.EntryPoint < gs_Wb0Header.Destination) ||
       (gs_Wb0Header.EntryPoint >=
        (gs_Wb0Header.Destination + gs_Wb0Header.RecoveryCodeLength)))
    {
        return NvBootError_BootLoaderLoadFailure;
    }
    return NvBootError_Success;

}

/*
 * This function generates the subkeys and begins the hash operation.
 * It is upto the caller to wait till the hash is computed and read the hash
 * output.
 */
static void
StartRecoveryCodeHash(void)
{
    NvU32       TotalBlocks;
    NvU8       *Src;
    NvU32       Offset; // to the start of the hashed area of the message
    NvU32       K1[NVBOOT_SE_AES_BLOCK_LENGTH];
    NvU32       K2[NVBOOT_SE_AES_BLOCK_LENGTH];

    Src = (NvU8*)(&((NvBootWb0RecoveryHeader*)gs_RecoveryCodeHeaderStart)->
                  RandomAesBlock);

    Offset = (NvU8*)&(gs_Wb0Header.RandomAesBlock) - (NvU8*)&gs_Wb0Header;

    TotalBlocks = (gs_Wb0Header.LengthInsecure - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;

    NvBootSeAesCmacGenerateSubkey(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY128,
                                  &K1[0],
                                  &K2[0]);

    // Start the hash computation.  This is the only chunk to be hashed.
    // The code in ProcessRecoveryCode() waits for the engine to be idle
    // again before reading the computed hash.
    NvBootSeAesCmacHashBlocks(&K1[0],
                              &K2[0],
                              (NvU32 *) Src,
                              (NvU8 *) &gs_ComputedHash.CryptoHash,
                              NvBootSeAesKeySlot_SBK_AES_CMAC_Hash,
                              SE_MODE_PKT_AESMODE_KEY128,
                              TotalBlocks,
                              NV_TRUE,
                              NV_TRUE);
}

/*
 * This function decrypts the header, after which it launches the decryption
 * of the recovery code itself.  The calling function must handle waiting
 * for the recovery code decryption to complete.
 */
static void
StartRecoveryCodeDecrypt(void)
{
    NvU32       BlocksLeft;
    NvU32       HeaderBlocks;
    NvU8       *Src = (NvU8*)&(gs_Wb0Header.RandomAesBlock);
    NvU32       Offset; // to the start of the hashed area of the message

    Offset = (NvU8*)&(gs_Wb0Header.RandomAesBlock) - (NvU8*)&gs_Wb0Header;

    BlocksLeft   = (gs_Wb0Header.LengthInsecure - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;

    HeaderBlocks = (sizeof(NvBootWb0RecoveryHeader) - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;

    // Clear original IV.
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK_AES_Decrypt,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                              0);

    // Decrypt the header
    NvBootSeAesDecrypt(NvBootSeAesKeySlot_SBK_AES_Decrypt,
                       SE_MODE_PKT_AESMODE_KEY128,
                       NV_TRUE,
                       HeaderBlocks,
                       Src,
                       Src);

    while(NvBootSeIsEngineBusy())
        ;

    // Decrypt the data
    BlocksLeft -= HeaderBlocks;
    if (BlocksLeft > 0)
    {
        Src = (NvU8*)(gs_RecoveryCodeHeaderStart +
                      sizeof(NvBootWb0RecoveryHeader));

        NvBootSeAesDecrypt(NvBootSeAesKeySlot_SBK_AES_Decrypt,
                           SE_MODE_PKT_AESMODE_KEY128,
                           NV_FALSE,
                           BlocksLeft,
                           Src,
                           (NvU8 *) gs_Wb0Header.Destination);
    }
}

/*
 * This functions gets the recovery code header start for further use.
 */
static NvBootError
Wb0ReadRecoveryCodeHeader(void)
{
    NvU32 SdramStartInBytes;
    NvU32 SdramEndInBytes;

    // Read the recovery code header start and the header
    gs_RecoveryCodeHeaderStart = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +
                                           APBDEV_PMC_SCRATCH1_0);
    SdramStartInBytes = NVBOOT_BL_SDRAM_START;
    SdramEndInBytes = SdramStartInBytes + NvBootSdramQueryTotalSize();

    if (gs_RecoveryCodeHeaderStart & 0xF)
    {
        return NvBootError_MemoryNotAligned;
    }
    if ((gs_RecoveryCodeHeaderStart >= SdramEndInBytes) ||
        (gs_RecoveryCodeHeaderStart < SdramStartInBytes))
    {
        return NvBootError_DataCorrupted;
    }

    NvBootUtilMemcpy((void*)&gs_Wb0Header,
                     (void*)(gs_RecoveryCodeHeaderStart),
                     sizeof(gs_Wb0Header));

    return NvBootError_Success;
}

static NvBootError
ProcessRecoveryCode()
{
    // Declare word aligned storage for the public key to be loaded
    // into the SE. 
    NvBootSeRsaKey2048  PublicKey __attribute__ ((aligned (4)));
    NvU32               Sha256Digest[ARSE_SHA256_HASH_SIZE / 8 / 4];    
    NvU8                PublicKeyHash[ARSE_SHA256_HASH_SIZE / 8];    
    NvBootFuseOperatingMode Mode;

    NvBootFuseGetOperatingMode(&Mode);

    // Read recovery code header
    if(Wb0ReadRecoveryCodeHeader() != NvBootError_Success)
    {
        return NvBootError_WarmBoot0_Failure;
    }
    
    if(Mode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
    {
        /*
         * Verify the public key against the hash of the public key in fuses.
         * First compute the SHA-256 hash the public key modulus.
         * TODO: declare input data size used in a separate .h file which maps
         * to ARSE_RSA_MAX_MODULUS_SIZE so changes to the modulus size,
         * in the future only needs a change in the .h file.
         * TODO: move the public key verification code into its own
         * function? (since we use it more than once - here and in the 
         * reading of the BCT).
         */
        NvBootSeSHAHash(&gs_Wb0Header.PublicKey.Modulus[0], 
                        ARSE_RSA_MAX_MODULUS_SIZE / 8, 
                        NULL,
                        &Sha256Digest[0], 
                        SE_MODE_PKT_SHAMODE_SHA256);
        
        // Wait until the SE is done the operation.
        while(NvBootSeIsEngineBusy())
            ;
        
        // Get the SHA-256 hash of the public key modulus from fuses.
        NvBootFuseGetPublicKeyHash(&PublicKeyHash[0]);

        // Verify the SHA-256 hash of the public key modulus against the fuses.
        if(NvBootUtilCompareBytes((NvU8 *) &Sha256Digest, PublicKeyHash, ARSE_SHA256_HASH_SIZE / 8 ) == NV_FALSE)
               return NvBootError_FuseHashMismatch;
        
        /*
         * Set up the public key in a format expected by the SE (modulus
         * first, then exponent, in a contiguous buffer).
         */
        NvBootUtilMemcpy(&PublicKey.Modulus[0], &gs_Wb0Header.PublicKey.Modulus, sizeof(NvBootRsaKeyModulus));
        NvBootUtilMemset(&PublicKey.Exponent[0], 0, ARSE_RSA_MAX_EXPONENT_SIZE/8);
        PublicKey.Exponent[0] = NVBOOT_SE_RSA_PUBLIC_KEY_EXPONENT;
    
        // Load the public key from the recovery code header. 
        NvBootSeRsaWriteKey((NvU32 *) &PublicKey, 
                            ARSE_RSA_MAX_MODULUS_SIZE, 
                            ARSE_RSA_MAX_EXPONENT_SIZE, 
                            SE_RSA_KEY_PKT_KEY_SLOT_ONE);
        // Wait until the SE is done the operation.
        while(NvBootSeIsEngineBusy())
            ;

        // Copy the recovery code to the Destination
        NvBootUtilMemcpy((void*)gs_Wb0Header.Destination,
                         (void*)(gs_RecoveryCodeHeaderStart +
                                 sizeof(NvBootWb0RecoveryHeader)),
                         (gs_Wb0Header.LengthSecure -
                          sizeof(NvBootWb0RecoveryHeader)));
    } 
    else
    {
        if(NvBootFuseIsOdmProductionModeSecure())
        {
            StartRecoveryCodeDecrypt();

            // Wait for total decryption of recovery code.
            // Since we only have one SE engine instead of two
            // BSEA/BSEV engines to use, we cannot interleave operations.
            while(NvBootSeIsEngineBusy())
                ;
        }
        else
        {
            // Copy the recovery code to the Destination
            NvBootUtilMemcpy((void*)gs_Wb0Header.Destination,
                             (void*)(gs_RecoveryCodeHeaderStart +
                                     sizeof(NvBootWb0RecoveryHeader)),
                             (gs_Wb0Header.LengthSecure -
                              sizeof(NvBootWb0RecoveryHeader)));
        }

        // Begin computing the hash over the recovery code.
        // The AES-CMAC hash in ODMS mode is calculated from the encrypted recovery
        // code.
        StartRecoveryCodeHash();

        // Wait for hash completion, i.e., the engine is no longer busy.
        while(NvBootSeIsEngineBusy())
            ;
    }
    // Compare the hash in the recovery code header with the computed hash.
    if(ValidateRecoveryCode() != NvBootError_Success)
    {
        return NvBootError_WarmBoot0_Failure;
    }
    return NvBootError_Success;
}

NvBootError
NvBootWb0WarmBoot(NvU32 *BootRomExitTarget)
{
    NvU32 M, N, P, Misc1, Misc2;
    NvU32 RegVal;
    NvU32 pllXStabilizationDelay;
    NvU32 pllMStabilizationDelay;
    NvU32 pllXStableTime;
    NvU32 pllMStableTime;
    NvU32 pllXEnable = 0;
    NvBootError e;

    // Clear the warm boot flag so that if the warm boot fails, the iROM
    // will follow the cold boot path on the next reboot.
    NvBootPmcSetFlag(NvBootPmcFlagId_Wb0, NV_FALSE);

    // Initialize everything needed to restore SDRAM accesses.
/// TODO: PLLM is a different pll. Therefore, corresponding CAR registers and range constants
/// for these registers are also different and need to be fixed.
    // Extract the PLLM related parameters.
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH2_0);
    M = NV_DRF_VAL(APBDEV_PMC, SCRATCH2, CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVM, RegVal);
    N = NV_DRF_VAL(APBDEV_PMC, SCRATCH2, CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVN, RegVal);
    P = NV_DRF_VAL(APBDEV_PMC, SCRATCH2, CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVP, RegVal);
    // PLLM KVCO, KCP etc.
    Misc2 = NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KVCO, \
                NV_DRF_VAL(APBDEV_PMC, SCRATCH2, CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KVCO, RegVal)) | \
            NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KCP, \
                NV_DRF_VAL(APBDEV_PMC, SCRATCH2, CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KCP, RegVal));

    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH35_0);
    Misc1 = NV_DRF_NUM(MISC1, CLK_RST_CONTROLLER_PLLM_MISC1, PLLM_SETUP, \
                NV_DRF_VAL(APBDEV_PMC, SCRATCH35, CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_SETUP, RegVal));

    // Read the start time delays
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH4_0);
    pllXStabilizationDelay = NV_DRF_VAL(APBDEV_PMC, SCRATCH4, PLLX_STABLE_TIME,
                                       RegVal);
    pllMStabilizationDelay = NV_DRF_VAL(APBDEV_PMC, SCRATCH4, PLLM_STABLE_TIME,
                                       RegVal);

    // If PLLM auto-restart is enabled, skip the starting of PLLM.
    if(!(NvBootPmcIsPllmOverrideEnabled()))
    {
        // Start PLLM for EMC/MC
        ///TODO scratch is not updated for all the PLLM params
        NvBootClocksStartPll(NvBootClocksPllId_PllM, M, N, P, Misc1, Misc2, \
                             &pllMStableTime);
    }
    else
    {
        // This function *currently* does nothing as PLLM does not have external dividers.
        // 0 - Reset Enable, 1 - Reset Disable
        NvBootClocksPllDivRstCtrl(NvBootClocksPllId_PllM, 0);
    }

    // Overwrite the stable time returned by clocks API to use the supplied
    // delay.
    pllMStableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE +
                               TIMERUS_CNTR_1US_0) +
        pllMStabilizationDelay;

    s_sdRamParamData->PllMInputDivider = M ;
    s_sdRamParamData->PllMFeedbackDivider = N;
    s_sdRamParamData->PllMPostDivider = P;
    s_sdRamParamData->PllMKVCO = NV_DRF_VAL(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KVCO, Misc2);
    s_sdRamParamData->PllMKCP = NV_DRF_VAL(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KCP, Misc2);
    s_sdRamParamData->PllMSetupControl = NV_DRF_VAL(MISC1, CLK_RST_CONTROLLER_PLLM_MISC1, PLLM_SETUP, Misc1);
    s_sdRamParamData->PllMStableTime = pllMStableTime ;

    // start PLLX using data in PMC
    // extract the PLLX data from PMC
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH3_0);

    pllXEnable = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_ENABLE, RegVal);
    if(pllXEnable)
    {
        M = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVM, RegVal);
        N = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVN, RegVal);
        P = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVP, RegVal);

        Misc2 = NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLX_MISC_3, PLLX_KVCO, \
                    NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KVCO, RegVal)) | \
                NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLX_MISC_3, PLLX_KCP, \
                    NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KCP, RegVal));

        RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH36_0);
        Misc1 = NV_DRF_NUM(MISC1, CLK_RST_CONTROLLER_PLLX_MISC_1, PLLX_SETUP, \
                    NV_DRF_VAL(APBDEV_PMC, SCRATCH36, CLK_RST_CONTROLLER_PLLX_MISC_1_0_PLLX_SETUP, RegVal));

        // Start PLLX and record the default PLL start time in pllXStableTime.
        NvBootClocksStartPll(NvBootClocksPllId_PllX, M, N, P, Misc1, Misc2, \
                             &pllXStableTime);

        // Overwrite the stable time returned by clocks API to use the supplied
        // delay.
        pllXStableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE +
                                   TIMERUS_CNTR_1US_0) +
                                   pllXStabilizationDelay;

    }

    // Poll for PLLM lock bit and timeout on polling loop
    while (!(NvBootClocksIsPllStable(NvBootClocksPllId_PllM, pllMStableTime)));

    NV_BOOT_CHECK_ERROR(NvBootWb0UnpackSdramParams(s_sdRamParamData));

    // Call SDRAM Init routine for WB0
    NvBootSdramInitWarmBoot0(s_sdRamParamData);

    /* The recovery code processing is :
     * 1. Initiate decryption
     * 2. Initiate hash computation
     * 3. Wait till HASH_ENGINE is idle.
     * 4. Wait till DECRYPT_ENGINE is idle.
     * 5. Validate the recovery code
     * 6. Set the bootrom exit target to entry point of the recovery code.
     */
    e = ProcessRecoveryCode();

    // Now handle any error from ProcessRecoveryCode().
    if(e)
    {
        return NvBootError_WarmBoot0_Failure;
    }

    // Reaching this point indicates that the LP0 recovery code was successfully
    // validated and ok to use. Set the address where AVP should jump here first
    // before trying SE context restoration.
    *BootRomExitTarget = gs_Wb0Header.EntryPoint;

    // Poll for PLLX lock bit and timeout on polling loop
    // This loop has been postponed as long as possible to allow the rest
    // of the warmboot processing to occur in parallel with PLL stabilization.
    if(pllXEnable)
        while (!(NvBootClocksIsPllStable(NvBootClocksPllId_PllX, pllXStableTime)));

    // Do SE LP0 context restore last after everything. Since we need to use
    // the SE to do AES/RSA operations, we can't restore the SE context until
    // we are finished using any parts of the SE.
    // An error in the LP0 context restore process is non-blocking. The
    // BR will still continue the LP0 exit path with the above BootRomExitTarget,
    // but the SE will be disabled for use, and none of its context prior to the
    // LP0 cycle will be restored.
    NV_BOOT_CHECK_ERROR(NvBootLP0ContextRestore());

    // Reaching this point indicates that the full Warm Boot0 sequence
    // completed successfully.
    // The Boot ROM has nothing else to do here - CPU clock enable, power on,
    // and reset release will be handled outside of the BROM.

    return NvBootError_Success;
}
