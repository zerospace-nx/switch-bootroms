/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "arse.h"
#include "nvboot_config.h"
#include "nvboot_fuse_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_int.h"
#include "nvboot_ssk_int.h"
#include "nvboot_util_int.h"

NvBootError
NvBootSskGenerate()
{
    const NvU32 BlockLengthBytes = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
    NvU8 ssk[NVBOOT_SE_AES_BLOCK_LENGTH_BYTES];
    NvU8 uid[NVBOOT_SE_AES_BLOCK_LENGTH_BYTES];
    NvU8 dk[NVBOOT_SE_AES_BLOCK_LENGTH_BYTES];
    NvU32 i;

    // SSK is calculated as follows --
    //
    // SSK = AES(SBK; UID ^ AES(SBK; DK; Encrypt); Encrypt)

    // Get Device Key from the fuse (device key is 4 bytes).
    NvBootFuseGetDeviceKey(dk);

    // copy the 4 byte device key into the consecutive word locations
    // to frame the 16 byte AES block;
    for (i = 0; i < 4; i++)
    {
        dk[i+4] = dk[i+8] = dk[i+12] = dk[i];
    }

    // Make sure engine is idle.  Encrypt device Key with the SBK, storing
    // the result in dk.  Finally, wait for engine to become idle.
    // Passing Dummy IRAM address to ensure no outstanding memory transactions.
    while(NvBootSeIsEngineBusy((NvU8*)NV_ADDRESS_MAP_IRAM_A_BASE))
        ;

    // Initialize OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    // Encrypt dk with the SBK.
    NvBootSeAesEncrypt(
        NvBootSeAesKeySlot_SBK,
        SE_MODE_PKT_AESMODE_KEY128,
        NV_TRUE,
        1,
        dk,
        dk);

    // Get Unique ID from fuses (NvBootECID is 128-bits), so no padding needed.
    NvBootFuseGetUniqueId((NvBootECID *) &uid);

    // XOR the uid and AES(SBK, DK, Encrypt)
    for (i = 0; i < BlockLengthBytes; i++)
        ssk[i] = uid[i] ^ dk[i];

    // Make sure engine is idle. Compute the final SSK value by performing
    // a second SBK encryption operation, and store the result in ssk.
    // Wait for engine to become idle.
    while(NvBootSeIsEngineBusy((NvU8*)dk))
        ;

    // Initialize OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    NvBootSeAesEncrypt(
        NvBootSeAesKeySlot_SBK,
        SE_MODE_PKT_AESMODE_KEY128,
        NV_TRUE,
        1,
        ssk,
        ssk);

    // Store the ssk in the SE keyslot reserved for the ssk.
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SSK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, (NvU32 *) &ssk);

    // Initialize OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SSK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    // Store the ssk in the SE2 keyslot reserved for the ssk.
    NvBootSeInstanceKeySlotWriteKeyIV(NvBootSeInstance_Se2, NvBootSeAesKeySlot_SSK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, (NvU32 *) &ssk);

    // Initialize OriginalIv[127:0] to zero
    NvBootSeInstanceKeySlotWriteKeyIV(NvBootSeInstance_Se2, NvBootSeAesKeySlot_SSK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    return NvBootError_Success;
}

void NvBootSskLockSsk(NvBool IsWarmBoot0)
{
    /*
     * Set access permissions for SE keyslot reserved for SSK, 
     * registers depending on boot type.  For WB0, both read- and write-access 
     * are disabled. For other boot types (i.e., frozen/cold boot and Recovery 
     * Mode), only read access is disabled.
     */
    // Need to disable read access to key slot NvBootSeAesKeySlot_SSK
    NvBootSeDisableKeySlotReadAccess(NvBootSeAesKeySlot_SSK, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
    if (IsWarmBoot0)
    {
        // Disable write access to key slot NvbootSeAesKeySlot_SSK
        NvBootSeDisableKeySlotWriteAccess(NvBootSeAesKeySlot_SSK, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
    }
}

