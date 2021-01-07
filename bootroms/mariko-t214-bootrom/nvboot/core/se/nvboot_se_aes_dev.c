/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_aes_device_int.h"
#include "nvboot_error.h"
#include "nvboot_se_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_aes_dev_int.h"
#include "nvboot_util_int.h"

#include "arse.h"

uint32_t NvBootSeKeySizeConv(NvBootAesKeySize KeySize)
{
    switch(KeySize)
    {
        case AES_KEY_128:
            return SE_MODE_PKT_AESMODE_KEY128;
        case AES_KEY_192:
            return SE_MODE_PKT_AESMODE_KEY192;
        case AES_KEY_256:
            return SE_MODE_PKT_AESMODE_KEY256;
        default:
            return SE_MODE_PKT_AESMODE_KEY128;
    }
}

NvBootError NvBootSeAesDevInit(void)
{
	// Clock init should be taken care of in the non-secure dispatcher.
    // This function should take care of any other programming required
    // before usage.
	return NvBootError_Success;
}

uint8_t NvBootSeAesDevNumKeySlots(void)
{
	return (uint8_t) NvBootSeAesKeySlot_Num;
}

bool NvBootSeAesDevIsValidKeySlot(uint8_t KeySlot)
{
	if(KeySlot <  NvBootSeAesKeySlot_Num)
		return true;
	else
		return false;
}

NvBootError NvBootSeAesDevGetKey(uint32_t *Key,
                                 const uint8_t KeySlot,
                                 const NvBootAesKeySize KeySize)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeKeySlotReadKeyIV(KeySlot,
                             NvBootSeKeySizeConv(KeySize),
                             SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                             Key);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevSetKey(const uint32_t *Key,
                                 const uint8_t KeySlot,
                                 const NvBootAesKeySize KeySize)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeKeySlotWriteKeyIV(KeySlot,
                              NvBootSeKeySizeConv(KeySize),
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                              (uint32_t *) Key);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevEncryptCBC(uint8_t const * const Plaintext,
                                     uint8_t * const Ciphertext,
                                     const uint8_t KeySlot,
                                     const NvBootAesKeySize KeySize,
                                     size_t Length)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeAesEncrypt(KeySlot,
                       NvBootSeKeySizeConv(KeySize),
                       NV_TRUE,
                       NV_ICEIL(Length, NVBOOT_AES_BLOCK_LENGTH_BYTES),
                       (uint8_t * const)Plaintext,
                       (uint8_t * const)Ciphertext);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevDecryptCBC(uint8_t const * const Ciphertext,
                                     uint8_t * const Plaintext,
                                     const uint8_t KeySlot,
                                     const NvBootAesKeySize KeySize,
                                     size_t Length)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;


    NvBootSeAesDecrypt (KeySlot,
                        NvBootSeKeySizeConv(KeySize),
                        NV_TRUE,
                        NV_ICEIL(Length, NVBOOT_AES_BLOCK_LENGTH_BYTES),
                        (uint8_t * const) Ciphertext,
                        (uint8_t * const) Plaintext);


    return NvBootError_Success;
}

NvBootError NvBootSeAesDevSetOriginalIv(const uint32_t *Iv,
                                        const uint8_t KeySlot)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeKeySlotWriteKeyIV(KeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                      (uint32_t * const)Iv);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevClearOriginalIv(const uint8_t KeySlot)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeKeySlotWriteKeyIV(KeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                      0);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevSetUpdatedIv(const uint32_t *Iv,
                                       const uint8_t KeySlot)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeKeySlotWriteKeyIV(KeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      (uint32_t * const)Iv);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevClearUpdatedIv(const uint8_t KeySlot)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    NvBootSeKeySlotWriteKeyIV(KeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      0);

    return NvBootError_Success;
}

NvBootError NvBootSeAesDevGenCmacSubkey(uint8_t KeySlot,
                                        NvBootAesKeySize KeySize,
                                        uint32_t *pK1,
                                        uint32_t *pK2)
{
    if(NvBootSeAesDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

	NvBootSeAesCmacGenerateSubkey(KeySlot,
	                              NvBootSeKeySizeConv(KeySize),
	                              pK1,
	                              pK2);

	return NvBootError_Success;
}

NvBootError NvBootSeAesCmacHash(NvBootAesDeviceCmacContext *AesCmacContext)
{

    if(NvBootSeAesDevIsValidKeySlot(AesCmacContext->KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

//    NvBootSeAesCmacHashBlocks (NvU32 *pK1, NvU32 *pK2, NvU32 *pInputMessage, NvU8 *pHash, NvU8 KeySlot, NvU8 KeySize, NvU32 NumBlocks, NvBool FirstChunk, NvBool LastChunk)

    NvBootSeAesCmacHashBlocks(AesCmacContext->pK1,
                              AesCmacContext->pK2,
                              AesCmacContext->pInputMessage,
                              (uint8_t *)AesCmacContext->pHash,
                              AesCmacContext->KeySlot,
                              NvBootSeKeySizeConv(AesCmacContext->KeySize),
                              AesCmacContext->NumBlocks,
                              AesCmacContext->FirstChunk,
                              AesCmacContext->LastChunk);

    return NvBootError_Success;
}

