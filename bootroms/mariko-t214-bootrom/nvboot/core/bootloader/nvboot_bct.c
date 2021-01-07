/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_bct.c - Implementation of Boot Config Table support.
 */

#include "nvboot_bct.h"
#include "nvboot_bct_int.h"
#include "nvboot_bit.h"
#include "nvboot_config.h"
#include "nvboot_context_int.h"
#include "nvboot_error.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_reader_int.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_se_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_util_int.h"
#include "nvboot_version_rom.h"
#include "nvboot_fuse_int.h"
#include "arse.h"
#include "arapbpm.h"
#include "nvboot_devmgr_int.h"
#include "nvboot_debug_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_rng_int.h"


/* Compile time assertions */
/*
 * The BCT should be a multiple of AES blocks.
 * If this fails, adjust NVBOOT_BCT_RESERVED_SIZE in nvboot_config.h to
 * change its size.
 */
NV_CT_ASSERT((sizeof(NvBootConfigTable) & 0xf) == 0);

/*
 * Verify  the end of a BCT will not fall within the last 16 bytes
 * of a page (Smallest page size is NVBOOT_MIN_PAGE_SIZE bytes).
 */
NV_CT_ASSERT(((sizeof(NvBootConfigTable)) % NVBOOT_MIN_PAGE_SIZE) <=
                                    (NVBOOT_MIN_PAGE_SIZE - 16));

/*
 * This assert is to make sure CustomerData field is always aligned to
 * a 4-byte boundary
 */
NV_CT_ASSERT((offsetof(NvBootConfigTable, CustomerData[0]) % 4) == 0);

/*
 * The unsigned section of the BCT must be a multiple of AES blocks (16 bytes)
 * to maintain compatibility with the nvboot_reader code. The nvboot_reader
 * code which does the AES CMAC-HASH and AES decryption using the BSEA/BSEV
 * engines must skip the unsigned section only in AES block multiples.
 */
NV_CT_ASSERT((offsetof(NvBootConfigTable, RandomAesBlock) % 16) == 0);

/*
 * BCT size should match with required size (NVBOOT_BCT_REQUIRED_SIZE).defined
 * If it fails need to adjust reserved filed,  customer data size or either
 * required size itself  for better performance.
 */
NV_CT_ASSERT(sizeof(NvBootConfigTable) == NVBOOT_BCT_REQUIRED_SIZE);

/*
 * Make sure MainBCT starts at SYSRAM location TBD.
 */
NV_CT_ASSERT(sizeof(NvBootInfoTable) >  0x80);
NV_CT_ASSERT(sizeof(NvBootInfoTable) <= 0x500);
NV_CT_ASSERT(sizeof(NvBootInfoTable) == NVBOOT_BIT_REQUIRED_SIZE);

/*
 * NcBootDevParams size is 64 bytes
 */
NV_CT_ASSERT(sizeof(NvBootDevParams) == 64);

/* Global data */
extern NvBootInfoTable   BootInfoTable;
extern NvBootConfigTable *pBootConfigTable;
extern NvBootCryptoMgrPublicBuf *pPublicCryptoBufR5;
extern int32_t FI_counter1;

/*
 * Static data
 */
/* TODO: Share this with the BL object descs */
static NvBootReaderObjDesc BctObjDesc;

/*
 * Static function declarations.
 */

// /**
 // * \brief   Validate and Decrypt the BCT.
 // *
 // * \note    BR will authenticate and decrypt the BCT using the following steps.
 // *          Validate the BCT using the OEM key (ECDSA, RSASSA-PSS, or AES-CMAC)
 // *          Decrypt (if enabled) the BCT using the OEM AES key.
 // *          Validate the BCT using the NVIDIA key (ECDSA only).
 // *          Decrypt the BCT using the NV key.
 // *
 // * \note    Decryption will happen in place.
 // */
// static NvBootError ValidateAndDecryptBct(NvBootContext *Context, NvBootConfigTable *Bct, uint8_t *BctDst);

// static NvBootError ValidateAndDecryptBct_SecureProvisioning(NvBootContext *Context, NvBootConfigTable *Bct, uint8_t *BctDst);

static NvBootError ValidateBct(NvBootContext *Context);

static NvBootError
ReadOneBct(
    NvBootContext *Context,
    const NvU32    Block,
    const NvU32    Page);

static void
RecordBctReadFailure(NvBootContext *Context, NvU32 Bit, NvBool IsBctSize, NvBootError e);

static void
RecordLastJournalReadStatus(NvBootContext *Context, NvBool IsBctSize, NvBootError e);

/*
static NvBootError
ValidateAndDecryptBct(NvBootContext *Context, NvBootConfigTable *Bct, uint8_t *BctDst)
{
    NvBootError e;
    TODO
    // void-ing this unused parameter. But is it needed?
    (void)Context;
    // If the bypass sysram fuse is set, skip all verification since usage of
    // SE requires SYSRAM/DRAM.
    if(NvBootFuseIsByPassSysramEnabled())
        return NvBootError_Success;

    // Validate the Pcp. Failure returns an error.
    e = NvBootCryptoMgrSetOemPcp(&Bct->Pcp);

    // If we are not in a PKC boot mode, NvBootCryptoMgrSetOemPcp will
    // return NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode.
    // Continue in this case because we will be using SBK.
    if( (e != NvBootError_Success) && (e != NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode))
        return e;

    // Validate the BCT using the OEM key.
    e = NvBootCryptoMgrVerify((uint8_t *)(BctDst + BctObjDesc.SignatureOffset),
                        &Bct->Signatures,
                        &pPublicCryptoBufR5->CalculatedSignaturesOemSigned,
                        BctObjDesc.Length - BctObjDesc.SignatureOffset,
                        Verify_w_OEM_Key);

    if(e != NvBootError_Success)
        return e;

    NvBootCryptoMgrDecrypt((uint8_t *) &(Bct->RandomAesBlock),
                           (uint8_t *) &(Bct->RandomAesBlock),
                           sizeof(NvBootConfigTable) - BctObjDesc.SignatureOffset,
                           Decrypt_w_OEM_Key);
    return e;
}
*/

// static NvBootError ValidateAndDecryptBct_SecureProvisioning(NvBootContext *Context, NvBootConfigTable *Bct, uint8_t *BctDst)
// {
    // NvBootError e;

    // // Entry into this function already has BCT read in.
    // // We already know we are provisionally in secure provisioning mode.
    // if(NvBootFuseGetSecureProvisioningIndexValidity() == NvBootError_Success)
    // {
        // Context->ProvisioningKeyNum = NvBootFuseGetSecureProvisionIndex();
    // }
    // else
    // {
        // // Else we use the BCT _insecure value. No need to worry about
        // // NvBootError_SecProvisioningInvalidAntiCloningKey, since we reached
        // // this path after secure provisioning is confirmed enabled.
        // Context->ProvisioningKeyNum = Bct->SecProvisioningKeyNum_Insecure;
    // }

    // // Load the prospective FSPK from secure IROM.
    // // If the ProvisioningKeyNum is the Key Wrap Key, decrypt it directly into the
    // // key slot.
    // if(Context->ProvisioningKeyNum == NvBootSeAesSecProvisioningKey_KeyWrapKey)
    // {
        // // Load the prospective FSPK from secure IROM into the KeyWrapKey_Decrypt
        // // key slot.
        // NvBootCryptoMgrLoadFSPK(Context->ProvisioningKeyNum, NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt);

        // // Decrypt the encrypted user-specified AES provisioning key directly into
        // // the SE key slots for FSPK.
        // NvBootSeAesDecryptKeyIntoKeySlot(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                                         // SE_MODE_PKT_AESMODE_KEY256,
                                         // NvBootSeAesKeySlot_Secure_Provisioning_Key,
                                         // SE_MODE_PKT_AESMODE_KEY256,
                                         // &(Bct->SecProvisioningKeyWrapKey.Key[0]));
    // }
    // else
    // {
        // // Load the prospective FSPK from secure IROM.
        // NvBootCryptoMgrLoadFSPK(Context->ProvisioningKeyNum, NvBootSeAesKeySlot_Secure_Provisioning_Key);
    // }

    // // Initialize SE key slot OriginalIv[127:0] to zero
    // NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key,
                      // SE_MODE_PKT_AESMODE_KEY128,
                      // SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                      // 0);

    // // Initialize SE slot UpdatedIV[127:0] to zero
    // NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key,
                      // SE_MODE_PKT_AESMODE_KEY128,
                      // SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      // 0);

    // // Once the key is loaded into a key slot, calculate the AES-CMAC hash of
    // // the payload.
    // e = NvBootCryptoMgrSecProvAuth((uint8_t *) (BctDst + BctObjDesc.SignatureOffset),
                                // &Bct->Signatures,
                                // &pPublicCryptoBufR5->CalculatedSignaturesOemSigned,
                                // BctObjDesc.Length - BctObjDesc.SignatureOffset);

    // if(e != NvBootError_Success)
        // return e;

    // // Decrypt in place and check against the Secure key number.
    // NvBootCryptoMgrSecProvDecrypt((uint8_t *) &(Bct->RandomAesBlock),
                // (uint8_t *) &(Bct->RandomAesBlock),
                // (sizeof(NvBootConfigTable) - BctObjDesc.SignatureOffset));

    // // Compare the Bct->SecProvisioningKeyNum_Insecure or the Anti-Cloning fuse value to the
    // // secure provisioningkey number after decryption. If _Insecure number or
    // // Anti-Cloning fuse value doesn't match the Secure field, someone has tampered
    // // with the BCT.
    // if(Context->ProvisioningKeyNum != Bct->SecProvisioningKeyNum_Secure)
        // return NvBootError_SecProvisioningBctKeyMismatch;

    // return NvBootError_Success;
// }

/**
 * ValidateBct(): Determine if the contents of the BCT are valid.
 *
 * @param[in] Context The current context
 *
 * @retval NvBootError_Success The BCT validated successfully.
 * @retval NvBootError_ValidationFailure At least one value was out of range.
 * @retval NvBootError_BctBlockInfoMismatch The block & page sizes of the
 * BCT did not match those in the device manager.
 */
static NvBootError
ValidateBct(NvBootContext *Context)
{
    NvU32                i;
    NvU32                BlockSize;
    NvU32                PageSize;
    NvU32                PagesPerBlock;
    NvBootConfigTable   *Bct;
    NvBootLoaderInfo    *BlInfo;

    NV_ASSERT(Context != NULL);

    // Bct @ sysram virtual address for R5
    Bct = pBootConfigTable;


    /* Check for matching data structure verisons. */
#if VERSION_CHK_ENABLED
    if (Bct->BootDataVersion != NVBOOT_BOOTDATA_VERSION)
      return NvBootError_ValidationFailure;
#endif

    /* Check for legal block and page sizes. */
    if ((Bct->BlockSizeLog2 < NVBOOT_MIN_BLOCK_SIZE_LOG2) ||
        (Bct->BlockSizeLog2 > NVBOOT_MAX_BLOCK_SIZE_LOG2) ||
        (Bct->PageSizeLog2  < NVBOOT_MIN_PAGE_SIZE_LOG2 ) ||
        (Bct->PageSizeLog2  > NVBOOT_MAX_PAGE_SIZE_LOG2 ))
    {
        return NvBootError_ValidationFailure;
    }

    /*
     * Verify that the BCT's block & page sizes match those used by the
     * device manager.
     */
    if ((Bct->BlockSizeLog2 != Context->DevMgr.BlockSizeLog2) ||
        (Bct->PageSizeLog2  != Context->DevMgr.PageSizeLog2))
    {
        return NvBootError_BctBlockInfoMismatch;
    }

    BlockSize     = 1 << Bct->BlockSizeLog2;
    PageSize      = 1 << Bct->PageSizeLog2;
    PagesPerBlock = 1 << (Bct->BlockSizeLog2 - Bct->PageSizeLog2);
    
    /*
     * Check the legality of the PartitionSize.
     * It should be a multiple of the block size.
     */
    if (Bct->PartitionSize & (BlockSize - 1))
        return NvBootError_ValidationFailure;

    /* Check the range of NumParamSets */
	// FIXME this logic needs to be revisited due to 2 bct structures in place.
#if SDRAM_CHECK_ENABLED
    if (Bct->NumParamSets > NVBOOT_BCT_MAX_PARAM_SETS)
        return NvBootError_ValidationFailure;
#endif

    /* Check the range of NumSdramSets */
    if (Bct->BootLoadersUsed > NVBOOT_MAX_BOOTLOADERS)
        return NvBootError_ValidationFailure;

    uint32_t OemHeaderEnd;
    /*
     * Check the OemMb1Header data.
     */
    for (i = 0; i < Bct->BootLoadersUsed; i++)
    {
        BlInfo = &(Bct->BootLoader[i]);

        if (BlInfo->StartPage >= PagesPerBlock)
            return NvBootError_ValidationFailure;

        /*
         * Check that the header fits within the partition.
         * OemHeaderEnd stores the ending offset of the header in the device.
         */
        OemHeaderEnd = BlInfo->StartBlock * BlockSize +
                BlInfo->StartPage  * PageSize  +
                sizeof(NvBootOemBootBinaryHeader);

        if (OemHeaderEnd > Bct->PartitionSize)
            return NvBootError_ValidationFailure;
    }

    return NvBootError_Success;
}

NvBootError NvBootProcessBct(NvBootConfigTable *Bct)
{
    NvBootDebugSetDebugFeatures(&Bct->UniqueChipId,
                            Bct->SecureDebugControl_Not_ECID_Checked,
                            Bct->SecureDebugControl_ECID_Checked);

    // Program TZRAM powergating and write locks.
    uint32_t BctConfig2 = Bct->BootConfig2[0];
    BctConfig2 &= APBDEV_PMC_TZRAM_PWR_CNTRL_0_TZRAM_SD_FIELD;
    NvBootPmcSetTzramPwrControl(BctConfig2 > 0 ? true : false);
    NvBootPmcEnableWriteLockToTzramPwrControl();

    return NvBootError_Success;
}

/**
 * ReadOneBct(): Read a single BCT starting at the requested page in the
 * requested block, validate it, and, upon success, store the BCT in the
 * Context.
 *
 * @param[in] Context Pointer to the current context
 * @param[in] Block Starting block for reading a BCT
 * @param[in] Page Starting page for reading a BCT
 *
 * @retval NvBootError_Success No errors were encountered.
 * @retval TODO Error codes from NvBootReadOneObject()
 * @retval TODO Error codes from ValidateBct()
 */
static NvBootError
ReadOneBct(NvBootContext *Context, const NvU32 Block, const NvU32 Page)
{
    // Default to "fail", subsequent functions can set to pass.
    volatile NvBootError e = NvBootInitializeNvBootError();
    // Point to global variabl already initialized.
    NvBootConfigTable      *Bct = pBootConfigTable;
    NvBootDevMgr *DevMgr;
    NvBootDeviceStatus      ReadStatus;

    NV_ASSERT(Context != NULL);

    // FI counter that is incremented after every critical step in the function.
    FI_counter1 = 0;

    // Null pointer?
    if(!Bct)
        return NvBootError_InvalidParameter;

    DevMgr = &(Context->DevMgr);


    /* Setup an object description to read a single BCT w/no redundancy. */
    BctObjDesc.BlIndex      = -1; /* Marks the ObjDesc as a BCT. */
    BctObjDesc.StartBlock   = Block;
    BctObjDesc.StartPage    = Page;
    BctObjDesc.Length       = sizeof(NvBootConfigTable);

    // Inititialize SecureProvisioningMode to disabled.
    Context->FactorySecureProvisioningMode = NV_FALSE;

    /* Initiate the bct read. */
    NV_BOOT_CHECK_ERROR(DevMgr->Callbacks->Read(Block,
                                   Page,
                                   sizeof(NvBootConfigTable),
                                   (uint8_t*)Bct));

    // Poll till Status changes from ReadInProgress.
    while((ReadStatus = DevMgr->Callbacks->QueryStatus()) == \
           NvBootDeviceStatus_ReadInProgress);

    if(ReadStatus != NvBootDeviceStatus_Idle)
        return NvBootError_DeviceReadError;

    // Load the Pcp if necessary.
    e = NvBootCryptoMgrSetOemPcp(&Bct->Pcp);

    FI_counter1 += COUNTER1;

    // If we are not in a PKC boot mode, NvBootCryptoMgrSetOemPcp will
    // return NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode.
    // Continue in this case because we will be using SBK.
    if( (e != NvBootError_Success) && (e != NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode))
        return e;

    FI_counter1 += COUNTER1;

    // If PRODUCTION_MODE = 1, SECURITY_MODE = 0, check if
    // this is a Secure Provisioning Boot.
    e = NvBootFuseIsSecureProvisioningMode(Bct->SecProvisioningKeyNum_Insecure);
    if(e == NvBootError_SecProvisioningEnabled)
    {
        NV_BOOT_CHECK_ERROR(NvBootCryptoMgrFskpInit(Bct->SecProvisioningKeyNum_Insecure, (uint8_t *)&Bct->SecProvisioningKeyWrapKey));
        e = NvBootCryptoMgrAuthBctFskp(Bct);
        if(e != NvBootError_Success)
        {
            // Reset FSKP context variable to disabled.
            Context->FactorySecureProvisioningMode = NV_FALSE;
            // Reset crypto mgr context for RCM usage.
            NvBootCryptoMgrInit();

            return e;
        }

        FI_counter1 += COUNTER1;

        // Random delay before double checking
        NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

        if(e > NvBootError_Success)
            return e;

        e = NvBootCryptoMgrDecryptBctFskp(Bct);
        if(e != NvBootError_Success)
        {
            // Reset FSKP context variable to disabled.
            Context->FactorySecureProvisioningMode = NV_FALSE;
            // Reset crypto mgr context for RCM usage.
            NvBootCryptoMgrInit();

            return e;
        }

        FI_counter1 += COUNTER1;
    }
    else if(e == NvBootError_SecProvisioningInvalidKeyInput)
    {
        return e;
    }
    else
    {
        // handles else if (e == NvBootError_SecProvisioningDisabled) and all other
        // errors.

        // This is the non-FSKP "regular" path.
        // Authenticate and decrypt BCT.
        e = NvBootCryptoMgrAuthBct(Bct);
        if(e != NvBootError_Success)
        {
            /// Introduces code distance between error detection and response.
            FI_ADD_DISTANCE_STEP(2);
            return e;
        }

        FI_counter1 += COUNTER1;
        // Random delay before double checking
        NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

        if(e > NvBootError_Success)
        {
            /// Introduces code distance between error detection and response.
            FI_ADD_DISTANCE_STEP(2);
            return e;
        }

        NV_BOOT_CHECK_ERROR(NvBootCryptoMgrDecryptBct(Bct));
        FI_counter1 += COUNTER1;
    }

    // Everything that reaches this point should be e == NvBootError_Success.
    NV_ASSERT(e == NvBootError_Success);

    /* Validate the BCT */
    NV_BOOT_CHECK_ERROR(ValidateBct(Context));
    FI_counter1 += COUNTER1;

    /* Update the BCT info in the BIT */
    BootInfoTable.BctValid = NV_TRUE;
    BootInfoTable.BctBlock = Block;
    BootInfoTable.BctPage  = Page;

    // After BCT is validated, process any necessary settings.
    NV_BOOT_CHECK_ERROR(NvBootProcessBct(Bct));

    FI_counter1 += COUNTER1;

    // Check if function counter is the expected value. If not, some instruction skipping might
    // have occurred.
    if(FI_counter1 != COUNTER1 * BctValidate_COUNTER_STEPS)
    {
        /// Introduces code distance between error detection and response.
        FI_ADD_DISTANCE_STEP(2);
        return NvBootError_Fault_Injection_Detection;
    }

    // Decrement function counter.
    FI_counter1 -= COUNTER1 * BctValidate_COUNTER_STEPS;

    // Add random delay to mitigate against temporal glitching.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    // Re-check counter.
    if(FI_counter1 != 0)
    {
        /// Introduces code distance between error detection and response.
        FI_ADD_DISTANCE_STEP(2);
        return NvBootError_Fault_Injection_Detection;
    }
    
    return e;
}

static void
RecordBctReadFailure(NvBootContext *Context, NvU32 Bit, NvBool IsBctSize, NvBootError e)
{
    NvU32 Index = Bit >> 3;  /* Byte # in bit vector */
    uint8_t  Mask  = (1 << (Bit & 0x7)); /* Mask for the bit to affect */

    TODO
    // voiding this unused variable. 'for now'. Protected by TODO macro
    (void)Context;
    switch (e)
    {
        /* Validation failure cases: Clear the bit. */
        case NvBootError_BctBlockInfoMismatch:
        case NvBootError_HashMismatch:
        case NvBootError_SE_RsaPssVerify_Inconsistent:
        case NvBootError_FuseHashMismatch:
        case NvBootError_ValidationFailure:
        case NvBootError_SecProvisioningBctKeyMismatch:
        case NvBootError_SecProvisioningInvalidKeyInput:
			if(!IsBctSize)
                BootInfoTable.BctStatus[Index] &= ~Mask;
			else
				BootInfoTable.BctSizeStatus[Index] &= ~Mask;
            break;

        /* Device read error cases: Set the bit */
        case NvBootError_DeviceReadError:
        case NvBootError_HwTimeOut:
        case NvBootError_EccFailureUncorrected:
			if(!IsBctSize)
                BootInfoTable.BctStatus[Index] |= Mask;
			else
				BootInfoTable.BctSizeStatus[Index] &= ~Mask;
            break;

        default:
            /* Code should not reach here. */
            NV_ASSERT(0);
            break;
    }
}

static void
RecordLastJournalReadStatus(NvBootContext *Context, NvBool IsBctSize, NvBootError e)
{
    TODO
    // voiding this unused variable. 'for now'. Protected by TODO macro
    (void)Context;
    switch (e)
    {
        /* Successful reading of the last BCT in the journal block. */
        case NvBootError_Success:
			if(!IsBctSize)
                BootInfoTable.BctLastJournalRead = NvBootRdrStatus_Success;
			else
				BootInfoTable.BctSizeLastJournalRead = NvBootRdrStatus_Success;
            break;

        /* Validation failure cases: Clear the bit. */
        case NvBootError_BctBlockInfoMismatch:
        case NvBootError_HashMismatch:
        case NvBootError_SE_RsaPssVerify_Inconsistent:
        case NvBootError_FuseHashMismatch:
        case NvBootError_ValidationFailure:
			if(!IsBctSize)
                BootInfoTable.BctLastJournalRead =
                    NvBootRdrStatus_ValidationFailure;
			else
				BootInfoTable.BctSizeLastJournalRead = 
				    NvBootRdrStatus_ValidationFailure;
            break;

        /* Device read error cases: Set the bit */
        case NvBootError_DeviceReadError:
        case NvBootError_HwTimeOut:
        case NvBootError_EccFailureUncorrected:
			if(!IsBctSize)
                BootInfoTable.BctLastJournalRead = 
                    NvBootRdrStatus_DeviceReadError;
			else
                BootInfoTable.BctSizeLastJournalRead = 
                    NvBootRdrStatus_DeviceReadError;
            break;

        default:
            /* Code should not reach here. */
            NV_ASSERT(0);
            break;
    }
}

/**
 * NvBootReadBct(): Attempt to read a BCT from the device.
 *
 * @param Context The current context structure
 *
 * @retval NvBootError_Success Successfully read & validated a BCT.
 * @retval NvBootError_BctNotFound Unable to locate a valid BCT.
 * @retval NvBootError_BctBlockInfoMismatch The block information in the BCT
 * does not match the block size information in the device manager.
 * @retval TODO Errors from ReadOneBct()
 *
 * Search algorithm for BCTs:
 *   * First try to read from Bct size validated block. If successful, the BCT was found.
 *   * If this fails, try to find the journal block.  It should lie within
 *     the first N blocks of the device, and is recognized by finding a BCT
 *     that validates, starting at page 0.
 *   * Keep reading BCTs from the journal block until one is found that does
 *     not validate.  The previous BCT is the one to use.
 *   * If the journal block is not found, return BctNotFound.
 */
/* TODO: Should NvBootError_BctBlockInfoMismatch be part of validating
 * a BCT?
 */
NvBootError NvBootReadBct(NvBootContext *Context)
{
    NvU32         Block;
    NvU32         Page;
    NvU32         PagesPerBct;
    NvU32         PagesPerBlock;
    NvBootError   e;
    NvBootDevMgr *DevMgr;
	NvBool isBctSize = NV_FALSE;

    NV_ASSERT(Context != NULL);

    /*
    * Update the BIT to reflect the fact that BCT reading has started:
    * - Set the size and BCT pointers.
    * - Update SafeStartAddr to the memory following the BCT
    */

    BootInfoTable.BctSize = sizeof(NvBootConfigTable);
    BootInfoTable.BctPtr = pBootConfigTable;

    /// based on the buffer being used.. safeStartAdr should be updated.
    /// safeStartAddr is used by R5 only?
    BootInfoTable.SafeStartAddr = PTR_TO_ADDR(pBootConfigTable) +
          sizeof(NvBootConfigTable);

    DevMgr = &(Context->DevMgr);

    PagesPerBct = NV_ICEIL_LOG2(sizeof(NvBootConfigTable),
                                DevMgr->PageSizeLog2);

    /*
     * Attempt to read the BCT from BCT Size validated block.
     * If block was 0 and failed to validate BCT, check consecutive slot 1.
     * If this failed, read all the BCTs from the journal block and
     * keep the latest one.
     * Keep a record of useful failure information.
     */

    /* Read the BCT in block 0, slot 0. */
    e = ReadOneBct(Context, 0, 0);
    if (e == NvBootError_Success) return e;
    RecordBctReadFailure(Context, 0, isBctSize, e); /* Bit 0 in error vector in BIT */

    /* At this point, the BCT in block 0, slot 0 is not valid. Check slot 1. */
    e = ReadOneBct(Context, 0, PagesPerBct);
    if (e == NvBootError_Success) return e;
    RecordBctReadFailure(Context, 1, isBctSize, e); /* Bit 1 in error vector in BIT */

    /* Find the journal block. */
	/* Block information is obtained by validated Size of BCT */
    Block = 1;
    while (Block < NVBOOT_MAX_BCT_SEARCH_BLOCKS)
    {
    	/* Try to read a BCT at slot 0 in this block. */
    	e = ReadOneBct(Context, Block, 0);
    	if (e == NvBootError_Success) break;
    
        /* Record the failure in bit block# + 1 in error vector in BIT */
        RecordBctReadFailure(Context, Block+1, isBctSize, e);
    
    	Block++;
    }

    /* Return error if the journal block was not located. */
    if (Block == NVBOOT_MAX_BCT_SEARCH_BLOCKS)
        return NvBootError_BctNotFound;

    /*
     * Read BCTs from the journal block until an invalid one is located.
     * Keep the last good one found.
     */
    Page          = PagesPerBct; /* Skip over the BCT alread read. */
    PagesPerBlock = 1 << (DevMgr->BlockSizeLog2 - DevMgr->PageSizeLog2);

    while ((Page + PagesPerBct) < PagesPerBlock)
    {
        e = ReadOneBct(Context, Block, Page);
        if (e != NvBootError_Success) break;
        
        Page += PagesPerBct;
    }

    /* Record the status of the last BCT read in the journal block. */
    RecordLastJournalReadStatus(Context, isBctSize, e);

    /*
     * If the journal block search ended with an error, reread the last
     * good journal BCT in case the failed attempt that ended the journal
     * block search overwrote the last good BCT in memory.
     */
    if (e != NvBootError_Success)
    {
        Page -= PagesPerBct;
        e = ReadOneBct(Context, Block, Page);
    }
    return e;
}
