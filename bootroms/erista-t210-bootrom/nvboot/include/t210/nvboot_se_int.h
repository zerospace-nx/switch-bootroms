/*
 * Copyright (c) 2006 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_se_aes.h
 *
 * NvBootSeAes is NVIDIA's interface to the SE for Bootrom LP0 context
 * restore.
 *
 */


#ifndef INCLUDED_NVBOOT_SE_AES_INT_H
#define INCLUDED_NVBOOT_SE_AES_INT_H

#include "project.h"
#include "nvboot_se_defs.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * SE linked list buffer element.
 */
typedef struct SeLinkedListElementRec
{
    NvU32   StartByteAddress;
    NvU32   BufferByteSize;
} SeLinkedListElement;

/**
 * SE linked list strcture with only one
 * element.
 */
typedef struct SingleSeLinkedListRec
{
    NvU32 LastBufferNumber;
    SeLinkedListElement LLElement;
} SingleSeLinkedList;

/**
 * SE input/output linked list structure. 
 * LastBufferNumber specifies the last buffer number (zero based) that the 
 * SE should process. 
 */
typedef struct SeLinkedListRec
{
    NvU32                 LastBufferNumber;
    SeLinkedListElement   LLElement[NVBOOT_SE_LL_MAX_NUM_BUFFERS];
} SeLinkedList;

NvU32 NvBootGetSeReg(NvU32 Reg);
    
void NvBootSetSeReg(NvU32 Reg, NvU32 Data);

NvBool NvBootSeIsEngineBusy(void);

void NvBootSeInitializeSE(void);

void NvBootSeClearTzram(void);

/**
 * Set up the SE engine for the particular mode you wish to use.
 *
 * The caller is responsible to provide valid parameter inputs.
 * For example Mode = AES, Dst = RSA_REG is not valid but this function
 * won't reject this combination.
 *
 * @param Mode The SE mode you wish to use (AES, RSA, SHA, etc.)
 * @param Encrypt Encrypt/Decrypt based on the desired mode.
 * @param UseOrigIv TRUE if ORIGINAL IV is to be use. FALSE if UPDATED IV.
 * @param Dst Use SE_CONFIG_0_DST_* to specify destination. MEMORY if destination
 *            is memory. Other valid input are HASH_REG, KEYTABLE, SRK, and
 *            RSA_REG.
 * @param KeySlot SE key slot number
 * @param KeySize AES key size. Use SE_MODE_PKT_AESMODE_*.
 * @return void
 *
 */
void
NvBootSeSetupOpMode (
        NvBootSeOperationMode Mode,
        NvBool Encrypt,
        NvBool UseOrigIv,
        NvU8 Dst,
        NvU8 KeySlot,
        NvU8 KeySize);

void NvBootSeDisableEngine(void);

void NvBootSeDisableKeySlotReadAccess(NvU8 KeySlot, NvU8 KeySlotType);

void NvBootSeDisableKeySlotWriteAccess(NvU8 KeySlot, NvU8 KeySlotType);

void NvBootSeKeySlotReadKeyIV(NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData);

void NvBootSeKeySlotWriteKeyIV(NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData);

void NvBootSeLockSbk(void);

void  NvBootSeAesDecryptContext(NvU32 *pEncryptedContext, NvU32 *pDecryptedContext);

//TODO: make static?
NvBool  NvBootSeCheckKnownPattern(NvU8 *pPattern);

void NvBootSePmcLoadSrkFromSecureScratch(NvU32 *pSrk);

NvBootError NvBootLP0ContextRestore(void);

/**
 * 
 * SHA Hash interface
 *
 * Note: The SE RSA hardware expects integers to be represented by strings of
 * octets with the leftmost octet being the least significant octet. 
 * For example, 
 *
 *      9,202,000 = (0x) 50 69 8C. 
 */
void NvBootSeSHAHash(NvU32 *pInputMessage, NvU32 InputMessageSizeBytes, NvU32 *pInputLinkedList, NvU32 *pOutputDestination, NvU8 HashAlgorithm);


/**
 * 
 * RSA Key interface
 *
 */
//TODO: add documentation
void NvBootSeRsaReadKey(NvU32 *pKeyBuffer, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot, NvU8 ExpModSel);
void NvBootSeRsaWriteKey(NvU32 *pKeyBuffer, NvU32 RsaModulusSizeBits, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot);
void NvBootSeRsaClearKeySlot(NvU8 RsaKeySlot);

NvBootError NvBootSeRsaModularExp(NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 InputMessageLengthBytes, NvU32 *pInputMessage, NvU32 *pOutputDestination);

NvBootError
NvBootSeRsaPssSignatureVerify(NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 *pInputMessage, NvU32 *pMessageHash, NvU32 InputMessageLengthBytes, NvU32 *pSignature, NvU8 HashAlgorithm, NvS8 sLen);

/**
 * Generate AES-CMAC subkeys.
 *
 * @param KeySlot SE key slot.
 * @param KeySize AES key size; Use SE_MODE_PKT_AESMODE_KEY*
 * @param pK1 Pointer to K1.
 * @param pK2 Pointer to K2.
 * @return void
 *
 * Note: While the CMAC algorithm supports AES key sizes of 128, 192, or
 * 256-bits, the subkeys K1 and K2 are always equal to the AES block size
 * which is 128-bits.
 */
void NvBootSeAesCmacGenerateSubkey (
        NvU8 KeySlot,
        NvU8 KeySize,
        NvU32 *pK1,
        NvU32 *pK2);

/**
 * AES-CMAC hash AES-block size chunks of data.
 *
 * @param pK1 Pointer to K1 subkey.
 * @param pK2 Pointer to K2 subkey. Currently unused in this function since
 *            input must be a multiple of AES block size.
 * @param pInputMessage Pointer to input message.
 * @param pHash Pointer to buffer for AES-CMAC hash.
 * @param KeySlot SE key slot.
 * @param KeySize Specifies the AES key size to use. Use SE_MODE_PKT_AESMODE_KEY*.
 * @param NumBlocks Message size specified in AES blocks.
 * @param FirstChunk TRUE if this is the first chunk to be processed.
 * @param LastChunk TRUE if this is the last chunk to be processed.
 * @return void
 *
 * Note: This implementation requires the message size to be in multiples of AES
 *       block size (16 bytes).
 *       This function can currently handle NVBOOT_SE_LL_MAX_SIZE_BYTES at a time.
 *       This is more than enough to handle the maximum IRAM buffer size
 *       (see nvboot_buffers_int.h).
 */
void NvBootSeAesCmacHashBlocks (
        NvU32 *pK1,
        NvU32 *pK2,
        NvU32 *pInputMessage,
        NvU8 *pHash,
        NvU8 KeySlot,
        NvU8 KeySize,
        NvU32 NumBlocks,
        NvBool FirstChunk,
        NvBool LastChunk);

/**
 * Decrypt AES-block size chunks of data.
 *
 * @param KeySlot SE key slot.
 * @param KeySize AES key size, specified by SE_MODE_PKT_AESMODE_KEY*
 * @param First NV_TRUE if this is the first chunk to be processed. NV_TRUE
 *              also means original IV is used. If NV_FALSE, updated IV will be
 *              used.
 * @param NumBlocks Message size specified in AES blocks.
 * @param Src Pointer to message.
 * @param Dst Pointer to decrypted ciphertext.
 *
 * Note: This implementation requires the message size to be multiples of AES
 *       block size (16 bytes).
 *       This function can currently handle NVBOOT_SE_LL_MAX_SIZE_BYTES at a time.
 *       This is more than enough to handle the maximum IRAM buffer size
 *       (see nvboot_buffers_int.h).
 *       First specifies the use of ORIGINAL IV or UPDATED IV. The
 *       caller MUST load the correct ORIGINAL IV to be used into the SE
 *       before calling this function. If continuing an AES-decrypt operation
 *       and the updated IV is to be used, you can specify NV_FALSE for First.
 *
 *       The caller must check if the SE is idle first.
 */
void NvBootSeAesDecrypt (
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst);

/**
 * Encrypt AES-block size chunks of data.
 *
 * @param KeySlot SE key slot.
 * @param KeySize AES key size, specified by SE_MODE_PKT_AESMODE_KEY*
 * @param First NV_TRUE if this is the first chunk to be processed. NV_TRUE
 *              also means original IV is used. If NV_FALSE, updated IV will be
 *              used.
 * @param NumBlocks Message size specified in AES blocks.
 * @param Src Pointer to message.
 * @param Dst Pointer to encrypted plaintext.
 *
 * Note: This implementation requires the message size to be multiples of AES
 *       block size (16 bytes).
 *       This function can currently handle NVBOOT_SE_LL_MAX_SIZE_BYTES at a time.
 *       This is more than enough to handle the maximum IRAM buffer size
 *       (see nvboot_buffers_int.h).
 *       First specifies the use of ORIGINAL IV or UPDATED IV. The
 *       caller MUST load the correct ORIGINAL IV to be used into the SE
 *       before calling this function. If continuing an AES-encrylt operation
 *       and the updated IV is to be used, you can specify NV_FALSE for First.
 *
 *       The caller must check if the SE is idle first.
 */
void NvBootSeAesEncrypt (
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst);


/**
 * Decrypt an encrypted chunk of data containing an AES key 
 * directly into an SE AES key slot. The input chunk of data must be 1 AES block
 * (128-bit key) or 2 AES blocks (192-bit or 256-bit) in size. If loading a
 * 192-bit key, the caller must zero pad the trailing 64-bits.
 *
 * This is a blocking function. This function will block until all AES operations
 * are finished.
 *
 * @param SourceKeySlot The SE key slot loaded with the AES key used to decrypt the key data.
 * @param SourceKeySize The AES key size of SourceKeySlot.
 * @param TargetKeySlot The SE key slot we are decrypting directly to.
 * @param TargetKeySize The AES key size of the decrypted AES key loaded.
 * @param Src Pointer to the encrypted input data chunk.
 *
 */
void NvBootSeAesDecryptKeyIntoKeySlot (
        NvU8    SourceKeySlot,
        NvU8    SourceKeySize,
        NvU8    TargetKeySlot,
        NvU8    TargetKeySize,
        NvU8   *Src);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SE_AES_INT_H

