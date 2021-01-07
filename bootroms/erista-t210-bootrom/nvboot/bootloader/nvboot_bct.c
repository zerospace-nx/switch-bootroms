/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
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
#include "nvboot_se_int.h"
#include "nvboot_util_int.h"
#include "nvboot_version_rom.h"

#include "arse.h"


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
 * Make sure MainBCT starts at IRAM 0x40000100
 */
NV_CT_ASSERT(sizeof(NvBootInfoTable) >  0x80);
NV_CT_ASSERT(sizeof(NvBootInfoTable) < 0x100);

/*
 * NcBootDevParams size is 64 bytes
 */
NV_CT_ASSERT(sizeof(NvBootDevParams) == 64);

/* Global data */
extern NvBootInfoTable   BootInfoTable;
extern NvBootConfigTable BootConfigTable;

/*
 * Static data
 */
/* TODO: Share this with the BL object descs */
static NvBootReaderObjDesc BctObjDesc;

/*
 * Static function declarations.
 */
/* This function combines the reading of the BCT in NV Production mode
 * with the Factory Secure Provisioning feature.
 * We need to read the BCT first without validating it to know the factory
 * secure provisioning key to validate the BCT and Bootloader.
 */
static NvBootError ProcessAndReadBctInNvProdMode(NvBootContext *Context, NvBootConfigTable *Bct, NvU8 *BctDst);

static NvBootError ValidateBct(NvBootContext *Context);

static NvBootError
ReadOneBct(
    NvBootContext *Context,
    const NvU32    Block,
    const NvU32    Page);

static void
RecordBctReadFailure(NvBootContext *Context, NvU32 Bit, NvBootError e);

static void
RecordLastJournalReadStatus(NvBootContext *Context, NvBootError e);

static NvBootError
ProcessAndReadBctInNvProdMode(NvBootContext *Context, NvBootConfigTable *Bct, NvU8 *BctDst)
{
    NvBootError             e;
    NvBootAes128Key         K1;
    NvBootAes128Key         K2;
    NvBootAes256Key         SecureProvisioningKey __attribute__ ((aligned (4)));
    // Borrow NvBootSeAes128Key struct for local variable to store computed CMAC
    // hash.
    NvBootAes128Key       ComputedCmacHash;

    // Set defaults to Nv Production Mode operations except for CheckBootloaderHash.
    // Set Context->CheckBootloaderHash to NV_FALSE for the initial read of the
    // BCT. Then set Context->CheckBootloaderHash to NV_TRUE for the bootloader.
    // Secure provisioning - Disabled.
    // SBK as the key for validation and 128-bit key size.
    // AES-CMAC hash validation only, no decyrption.
    Context->CheckBootloaderHash = NV_FALSE;
    Context->DecryptBootloader = NV_FALSE;
    Context->FactorySecureProvisioningMode = NV_FALSE;
    Context->ValidationKeySize = SE_MODE_PKT_AESMODE_KEY128;
    Context->ProvisioningKeyNum = NvBootSeAesSecProvisioningKey_SecProvisiningDisabled;
    Context->DecryptionKeySlotNum = NvBootSeAesKeySlot_SBK_AES_Decrypt;
    Context->CMACHashKeySlotNum = NvBootSeAesKeySlot_SBK_AES_CMAC_Hash;

    /* Read the whole BCT without doing any verification (for now).
     * Context->CheckBootloaderHash = NV_FALSE;
     * Context->DecryptBootloader = NV_FALSE;
     */
    NV_BOOT_CHECK_ERROR(NvBootReadOneObject(Context,
                                            BctDst,
                                            &BctObjDesc,
                                            1,      /* Single obj desc */
                                            NULL)); /* No bad block table */
    // Make sure subsequent reads to BL check the hash.
    Context->CheckBootloaderHash = NV_TRUE;

    // Validate BCT insecure key input and also check the anti-cloning fuses
    // to see if we are in secure provisioning mode.
    e = NvBootFuseIsSecureProvisioningMode(Bct->SecProvisioningKeyNum_Insecure);

    // If an invalid anti-cloning fuse value or _Insecure key number is detected,
    // return the error. We should go to RCM in this case (unless any subsequent
    // BCT in flash can be found).
    if(e == NvBootError_SecProvisioningInvalidKeyInput)
    {
        return e;
    }
    else if(e == NvBootError_SecProvisioningEnabled)
    {
        // If we are in Factory Secure provisioning mode, load the appropriate key from
        // IROM and update the AES operations to be used for validating the BCT and BL.
        Context->FactorySecureProvisioningMode = NV_TRUE;
        Context->DecryptBootloader = NV_TRUE;
        Context->ValidationKeySize = SE_MODE_PKT_AESMODE_KEY256;
        Context->DecryptionKeySlotNum = NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt;
        Context->CMACHashKeySlotNum = NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash;

        if(NvBootFuseGetSecureProvisioningIndexValidity() == NvBootError_ValidAntiCloningFuse)
        {
            Context->ProvisioningKeyNum = NvBootFuseGetSecureProvisionIndex();
        }
        else
        {
            // Else we use the BCT _insecure value. No need to worry about
            // NvBootError_SecProvisioningInvalidAntiCloningKey, since we reached
            // this path after secure provisioning is confirmed enabled.
            Context->ProvisioningKeyNum = Bct->SecProvisioningKeyNum_Insecure;
        }

        // If we are in Secure Provisioning Mode, we need to load the appropriate key
        // in IROM.
        // Use this provisioning key to validate: Bct->SecProvisioningKeyNum_Insecure
        //
        // Load the key number as specified in the BCT's insecure field
        // SecProvisioningKeyNum_Insecure.
        // Copy the key from IROM into IRAM first.
        NvBootUtilMemcpy(&SecureProvisioningKey.Key[0],
                         (NvU8 *) NVBOOT_FACTORY_SECURE_PROVISIONING_KEYS_START + sizeof(NvBootAes256Key)*Context->ProvisioningKeyNum,
                         sizeof(NvBootAes256Key));

        if(Context->ProvisioningKeyNum == NvBootSeAesSecProvisioningKey_KeyWrapKey)
        {
            // Load the Key Wrap Key from IROM into unique SE key slot for
            // decrypt operations.
            NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                              SE_MODE_PKT_AESMODE_KEY256,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                              &SecureProvisioningKey.Key[0]);

            // Initialize SE key slot OriginalIv[127:0] to zero
            NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                              0);

            // Initialize SE slot UpdatedIV[127:0] to zero
            NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                              0);

            // Decrypt the encrypted user-specified AES provisioning key directly into
            // the SE key slots for decryption and CMAC hashing.
            NvBootSeAesDecryptKeyIntoKeySlot(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                                             SE_MODE_PKT_AESMODE_KEY256,
                                             NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                                             SE_MODE_PKT_AESMODE_KEY256,
                                             &(Bct->SecProvisioningKeyWrapKey.Key[0]));

            NvBootSeAesDecryptKeyIntoKeySlot(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                                             SE_MODE_PKT_AESMODE_KEY256,
                                             NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                                             SE_MODE_PKT_AESMODE_KEY256,
                                             &(Bct->SecProvisioningKeyWrapKey.Key[0]));
        }
        else
        {

            // Load the provisioning key into SE key slot for decryption
            NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                              SE_MODE_PKT_AESMODE_KEY256,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                              &SecureProvisioningKey.Key[0]);

            // Load the provisioning key into SE key slot for CMAC hashing.
            NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                              SE_MODE_PKT_AESMODE_KEY256,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                              &SecureProvisioningKey.Key[0]);
        }

        // Initialize SE key slot OriginalIv[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                          SE_MODE_PKT_AESMODE_KEY128,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                          0);

        // Initialize SE slot UpdatedIV[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                          SE_MODE_PKT_AESMODE_KEY128,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                          0);


        // Initialize SE key slot OriginalIv[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                          SE_MODE_PKT_AESMODE_KEY128,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                          0);

        // Initialize SE slot UpdatedIV[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                          SE_MODE_PKT_AESMODE_KEY128,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                          0);
    }

    // If secure provisioning mode not detected, we are in "regular" Nv Production Mode.
    // The Context setup at the beginning of the function is sufficient.

    NvBootSeAesCmacGenerateSubkey(Context->CMACHashKeySlotNum,
                                  Context->ValidationKeySize,
                                  &(K1.Key[0]),
                                  &(K2.Key[0]));

    // Hash the BCT in place.
    NvBootSeAesCmacHashBlocks((NvU32 *) &(K1.Key[0]),
                              (NvU32 *) &(K2.Key[0]),
                              (NvU32 *) (BctDst + BctObjDesc.SignatureOffset),
                              (NvU8 *) &ComputedCmacHash, // output of hash
                              Context->CMACHashKeySlotNum,
                              Context->ValidationKeySize,
                              (sizeof(NvBootConfigTable) - BctObjDesc.SignatureOffset) / NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
                              NV_TRUE, //FirstChunk
                              NV_TRUE); //LastChunk

    /* Spin wait for it to finish. */
    while(NvBootSeIsEngineBusy())
        ;

    // Compare the Hashes
    if(NvBootUtilCompareBytes((NvU8 *) &Bct->Signature.CryptoHash,
                              (NvU8 *) &ComputedCmacHash,
                              NVBOOT_SE_AES_BLOCK_LENGTH_BYTES) != NV_TRUE)
    {
        return NvBootError_HashMismatch;
    }

    if(Context->FactorySecureProvisioningMode == NV_TRUE)
    {
        // Decrypt BCT in place if Hashes match and we are
        // in Secure Provisioning Mode.
        NvBootSeAesDecrypt(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                           Context->ValidationKeySize,
                           NV_TRUE,
                           (sizeof(NvBootConfigTable) - BctObjDesc.SignatureOffset) / NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
                           (NvU8 *) &(Bct->RandomAesBlock),
                           (NvU8 *) &(Bct->RandomAesBlock));

        // Compare the Bct->SecProvisioningKeyNum_Insecure or the Anti-Cloning fuse value to the
        // secure provisioningkey number after decryption. If _Insecure number or
        // Anti-Cloning fuse value doesn't match the Secure field, someone has tampered
        // with the BCT.
        if(Context->ProvisioningKeyNum != Bct->SecProvisioningKeyNum_Secure)
            return NvBootError_SecProvisioningBctKeyMismatch;

    }

    if( (e == NvBootError_SecProvisioningEnabled) || (e == NvBootError_SecProvisioningDisabled) )
    {
        // BCT has been validated and decrypted at this point. Proceed.
        return NvBootError_Success;
    }
    else
    {
        // Return any error code other than NvBootError_SecProvisioningDisabled or
        // NvBootError_SecProvisioningEnabled.
        return e;
    }
}

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
/****
 **** TODO: Should this simply return a bool, or are there different error
 ****       codes to propagate?
 ****/
static NvBootError
ValidateBct(NvBootContext *Context)
{
    NvU32                i;
    NvU32                BlockSize;
    NvU32                PageSize;
    NvU32                PagesPerBlock;
    NvU32                BlStart;
    NvU32                BlEnd;
    NvBootConfigTable   *Bct;
    NvBootLoaderInfo    *BlInfo;
#if USE_BADBLOCKS
    NvBootBadBlockTable *Table;
#endif
    NvU8                 PadVal;

    NV_ASSERT(Context != NULL);

    Bct = &BootConfigTable;

    /* Check for matching data structure verisons. */
    if (Bct->BootDataVersion != NVBOOT_BOOTDATA_VERSION)
      return NvBootError_ValidationFailure;

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
    if (Bct->NumParamSets > NVBOOT_BCT_MAX_PARAM_SETS)
        return NvBootError_ValidationFailure;

    /* Check the range of NumSdramSets */
    if (Bct->NumSdramSets > NVBOOT_BCT_MAX_SDRAM_SETS)
        return NvBootError_ValidationFailure;

#if USE_BADBLOCKS

    /*
     * Check the BadBlockTable.
     */
    Table = &(Bct->BadBlockTable);

    /* Check for matching block sizes.  */
    if (Table->BlockSizeLog2 != Bct->BlockSizeLog2)
        return NvBootError_ValidationFailure;

    /* Check for compatible block sizes.  */
    if (Table->BlockSizeLog2 > Table->VirtualBlockSizeLog2)
        return NvBootError_ValidationFailure;

    /* Check for the portion of the table used. */
    if (Table->EntriesUsed > NVBOOT_BAD_BLOCK_TABLE_SIZE)
        return NvBootError_ValidationFailure;

    if (Table->EntriesUsed !=
        NV_ICEIL_LOG2(Bct->PartitionSize, Table->VirtualBlockSizeLog2))
        return NvBootError_ValidationFailure;
#endif

    /* Check the range of NumSdramSets */
    if (Bct->BootLoadersUsed > NVBOOT_MAX_BOOTLOADERS)
        return NvBootError_ValidationFailure;

    /*
     * Check the bootloader data.
     */
    for (i = 0; i < Bct->BootLoadersUsed; i++)
    {
        BlInfo = &(Bct->BootLoader[i]);

        if (BlInfo->StartPage >= PagesPerBlock)
            return NvBootError_ValidationFailure;

        if (BlInfo->Length == 0)
            return NvBootError_ValidationFailure;

        /*
         * Check that the bootloader fits within the partition.
         * BlEnd stores the ending offset of the BL in the device.
         */
        BlEnd = BlInfo->StartBlock * BlockSize +
                BlInfo->StartPage  * PageSize  +
                BlInfo->Length;

        if (BlEnd > Bct->PartitionSize)
            return NvBootError_ValidationFailure;

        /*
         * Check the entry point.
         * BlStart and BlEnd are address in the destination memory.
         */
        BlStart = BlInfo->LoadAddress;
        BlEnd   = BlStart + BlInfo->Length;

        if ((BlInfo->EntryPoint < BlStart) ||
            (BlInfo->EntryPoint >= BlEnd))
            return NvBootError_ValidationFailure;
    }

    /* Check the Reserved field for the correct padding pattern. */
    PadVal = 0x80;
    for (i = 0; i < NVBOOT_BCT_RESERVED_SIZE; i++)
    {
        if (Bct->Reserved[i] != PadVal) return NvBootError_ValidationFailure;
        PadVal = 0x00;
    }

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
    NvBootError             e;
    NvU8                   *BctDst = (NvU8*)(&BootConfigTable);
    NvBootConfigTable      *Bct = (NvBootConfigTable *) BctDst;
    NvU32                   Sha256Digest[ARSE_SHA256_HASH_SIZE / 8 / 4];
    NvU8                    PublicKeyHash[ARSE_SHA256_HASH_SIZE / 8];
    // Early versions of SE RSA HW did not unaligned word (32-bit) addresses;
    // Use aligned (4) attribute for now.
    NvBootSeRsaKey2048      PublicKey __attribute__ ((aligned (4)));
    NvBootFuseOperatingMode Mode;

    NV_ASSERT(Context != NULL);

    /* Default to 128-bit Key Size for AES operations */
    Context->ValidationKeySize = SE_MODE_PKT_AESMODE_KEY128;

    /* Setup an object description to read a single BCT w/no redundancy. */
    BctObjDesc.BlIndex      = -1; /* Marks the ObjDesc as a BCT. */
    BctObjDesc.StartBlock   = Block;
    BctObjDesc.StartPage    = Page;
    BctObjDesc.Length       = sizeof(NvBootConfigTable);
    /* SignatureOffset is the offset to the start of the signed section
     * of the object
     * NOTE: If the BCT unsigned section changes, SignatureOffset must be
     * updated!! */
    BctObjDesc.SignatureOffset = (NvU32) &(BootConfigTable.RandomAesBlock) -
                                 (NvU32) &BootConfigTable;

    /* Starting address of Signature */
    BctObjDesc.SignatureRef = (NvBootObjectSignature *) (&BootConfigTable.Signature);/* Start of signature */

    NvBootFuseGetOperatingMode(&Mode);

    /* Read BCT and verify it using either RSA-PSS or AES-CMAC */
    if(Mode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
    {

        /* Read the whole BCT without doing any verification.
         * Context->CheckBootloaderHash = NV_FALSE;
         * Context->DecryptBootloader = NV_FALSE;
         */
        NV_BOOT_CHECK_ERROR(NvBootReadOneObject(Context,
                                                BctDst,
                                                &BctObjDesc,
                                                1,      /* Single obj desc */
                                                NULL)); /* No bad block table */

        /*
         * Compute the SHA-256 hash the public key modulus.
         * TODO: declare input data size used in a separate .h file which maps
         * to ARSE_RSA_MAX_MODULUS_SIZE so changes to the modulus size,
         * in the future only needs a change in the .h file.
         */
        NvBootSeSHAHash(&Bct->Key.Modulus[0], ARSE_RSA_MAX_MODULUS_SIZE / 8, NULL, &Sha256Digest[0], SE_MODE_PKT_SHAMODE_SHA256);

        // Wait until the SE is done the operation.
        while(NvBootSeIsEngineBusy())
            ;

        // Get the SHA-256 hash of the public key modulus from fuses.
        NvBootFuseGetPublicKeyHash(&PublicKeyHash[0]);

        // Verify the SHA-256 hash of the public key modulus against the fuses.
        if(NvBootUtilCompareBytes((NvU8 *) &Sha256Digest, PublicKeyHash, ARSE_SHA256_HASH_SIZE / 8 ) == NV_FALSE)
               return NvBootError_FuseHashMismatch;

        /*
         * Verify the RSASSA-PSS signature of the BCT.
         * Set up the Public Key in a format expected by the SE (modulus
         * first, then exponent, in a contiguous buffer).
         */
        NvBootUtilMemcpy(&PublicKey.Modulus[0], &Bct->Key, sizeof(NvBootRsaKeyModulus));
        NvBootUtilMemset(&PublicKey.Exponent[0], 0, ARSE_RSA_MAX_EXPONENT_SIZE/8);
        PublicKey.Exponent[0] = NVBOOT_SE_RSA_PUBLIC_KEY_EXPONENT;

        /*
         * 2. Load key into the SE. Use slot 1.
         * Leave the key in the key slot for future use by the BL.
         * TODO: Set any key permissions?
         */
        NvBootSeRsaWriteKey((NvU32 *) &PublicKey, ARSE_RSA_MAX_MODULUS_SIZE, ARSE_RSA_MAX_EXPONENT_SIZE, SE_RSA_KEY_PKT_KEY_SLOT_ONE);
        // Wait until the SE is done the operation.
        while(NvBootSeIsEngineBusy())
            ;

        // 3. Call RSASSA-PSS VERIFY function.
        NV_BOOT_CHECK_ERROR(NvBootSeRsaPssSignatureVerify(SE_RSA_KEY_PKT_KEY_SLOT_ONE,
                                                            ARSE_RSA_MAX_EXPONENT_SIZE,
                                                            (NvU32 *)  (BctDst + BctObjDesc.SignatureOffset),
                                                            NULL,
                                                            BctObjDesc.Length - BctObjDesc.SignatureOffset,
                                                            &Bct->Signature.RsaPssSig.Signature[0],
                                                            SE_MODE_PKT_SHAMODE_SHA256,
                                                            ARSE_SHA256_HASH_SIZE / 8));

    }
    /* Nv Production mode needs special handling now because of the Factory
     * Secure Provisioning feature. If the Secure Provision (or Anti-Cloning)
     * index fuse isn't burned, then the key will be contained
     * in the BCT. We need to read the BCT first without validation so
     * BR can know the key number stored in IROM to use to validate the BCT. */
    else if(Mode == NvBootFuseOperatingMode_NvProduction)
    {
        NV_BOOT_CHECK_ERROR(ProcessAndReadBctInNvProdMode(Context, Bct, BctDst));
    }
    /*
     * Handle ODMS mode here.
     */
    else
    {
        /* Note: There is no bad block table available when reading the BCT. */
        NV_BOOT_CHECK_ERROR(NvBootReadOneObject(Context,
                                                BctDst,
                                                &BctObjDesc,
                                                1,      /* Single obj desc */
                                                NULL)); /* No bad block table */
    }

    /* Validate the BCT */
    NV_BOOT_CHECK_ERROR(ValidateBct(Context));

    /* Update the BCT info in the BIT */
    BootInfoTable.BctValid = NV_TRUE;
    BootInfoTable.BctBlock = Block;
    BootInfoTable.BctPage  = Page;

    return NvBootError_Success;
}

static void
RecordBctReadFailure(NvBootContext *Context, NvU32 Bit, NvBootError e)
{
    NvU32 Index = Bit >> 3;  /* Byte # in bit vector */
    NvU8  Mask  = (1 << (Bit & 0x7)); /* Mask for the bit to affect */

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
            BootInfoTable.BctStatus[Index] &= ~Mask;
            break;

        /* Device read error cases: Set the bit */
        case NvBootError_DeviceReadError:
        case NvBootError_HwTimeOut:
        case NvBootError_EccFailureUncorrected:
            BootInfoTable.BctStatus[Index] |= Mask;
            break;

        default:
            /* Code should not reach here. */
            NV_ASSERT(0);
            break;
    }
}

static void
RecordLastJournalReadStatus(NvBootContext *Context, NvBootError e)
{
    switch (e)
    {
        /* Successful reading of the last BCT in the journal block. */
        case NvBootError_Success:
            BootInfoTable.BctLastJournalRead = NvBootRdrStatus_Success;
            break;

        /* Validation failure cases: Clear the bit. */
        case NvBootError_BctBlockInfoMismatch:
        case NvBootError_HashMismatch:
        case NvBootError_SE_RsaPssVerify_Inconsistent:
        case NvBootError_FuseHashMismatch:
        case NvBootError_ValidationFailure:
            BootInfoTable.BctLastJournalRead =
                NvBootRdrStatus_ValidationFailure;
            break;

        /* Device read error cases: Set the bit */
        case NvBootError_DeviceReadError:
        case NvBootError_HwTimeOut:
        case NvBootError_EccFailureUncorrected:
            BootInfoTable.BctLastJournalRead = NvBootRdrStatus_DeviceReadError;
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
 *   * First try to read from block 0. If successful, the BCT was found.
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

    NV_ASSERT(Context != NULL);

    /*
     * Update the BIT to reflect the fact that BCT reading has started:
     * - Set the size and BCT pointers.
     * - Update SafeStartAddr to the memory following the BCT
     */
    BootInfoTable.BctSize = sizeof(NvBootConfigTable);
    BootInfoTable.BctPtr = &BootConfigTable;
    BootInfoTable.SafeStartAddr = PTR_TO_ADDR(&BootConfigTable) +
          sizeof(NvBootConfigTable);

    DevMgr = &(Context->DevMgr);

    PagesPerBct = NV_ICEIL_LOG2(sizeof(NvBootConfigTable),
                                DevMgr->PageSizeLog2);

    /*
     * Attempt to read the BCT from block 0, slot 0.
     * If this failed, check slot 1.
     * If this failed, read all the BCTs from the journal block and
     * keep the latest one.
     * Keep a record of useful failure information.
     */

    /* Read the BCT in block 0, slot 0. */
    e = ReadOneBct(Context, 0, 0);
    if (e == NvBootError_Success) return e;
    RecordBctReadFailure(Context, 0, e); /* Bit 0 in error vector in BIT */

    /* At this point, the BCT in block 0, slot 0 is not valid. Check slot 1. */
    e = ReadOneBct(Context, 0, PagesPerBct);
    if (e == NvBootError_Success) return e;
    RecordBctReadFailure(Context, 1, e); /* Bit 1 in error vector in BIT */

    /* Find the journal block. */
    Block = 1;
    while (Block < NVBOOT_MAX_BCT_SEARCH_BLOCKS)
    {
	/* Try to read a BCT at slot 0 in this block. */
	e = ReadOneBct(Context, Block, 0);
	if (e == NvBootError_Success) break;

        /* Record the failure in bit block# + 1 in error vector in BIT */
        RecordBctReadFailure(Context, Block+1, e);

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
    RecordLastJournalReadStatus(Context, e);

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
