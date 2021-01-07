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
#include "nvboot_se_lp0_context.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 *  T214 has 2 identical instances of SE.
 *  The legacy instance is referred to as NvBootSeInstance_Se1.
 *  The new instance is referred to as NvBootSeInstance_Se2.
 */
typedef enum NvBootSeInstanceRec
{
    NvBootSeInstance_Se1,
    NvBootSeInstance_Se2
} NvBootSeInstance;

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

/**
 * Instance can be either 
 * NvBootSeInstance_Se1: legacy SE or 
 * NvBootSeInstance_Se2: new identical instance added for t214
 */
NvU32 NvBootGetSeInstanceReg(NvBootSeInstance Instance, NvU32 Reg);
    
void NvBootSetSeInstanceReg(NvBootSeInstance Instance, NvU32 Reg, NvU32 Data);

/**
 *  Check for operations pending, pending memory write (through SE status) and AHB 
 *  coherency if Dest Addr in DRAM range.
 */
NvBool NvBootSeInstanceIsEngineBusy(NvBootSeInstance Instance, NvU8* DestAddr);

/**
 * Is the particular SE instance enabled or disabled?
 * True if enabled, false if disabled.
 */
NvBool NvBootSeInstanceIsEngineEnabled(NvBootSeInstance Instance);

void NvBootSeInitializeSE(void);

/**
 * Check if atomic SE context save/restore is enabled for
 * a partulcar SE instance.
 */
NvBool NvBootSeIsAtomicSeContextSaveEnabled(NvBootSeInstance Instance);

/**
 * Enable Atomic SE context save feature for all instsances
 * of SE.
 * Program and locks address of the SE context blobs into secure scratch.
 *
 * Must be called BEFORE TZRAM is cleared for exit because the 
 * TZRAM clearing function uses the enablement of SE context save
 * as an input.
 *
 */
NvBootError NvBootSeEnableAtomicSeContextSave(void);

/**
 * Enable Atomic SE context save feature for a particular instance.
 * Program and locks the save address of the SE context blob
 * into secure scratch for the particular instance.
 *
 * Must be called BEFORE TZRAM is cleared for exit because the 
 * TZRAM clearing function uses the enablement of SE context save
 * as an input.
 *
 */
void NvBootSeInstanceEnableAtomicSeContextSave(NvBootSeInstance Instance);

/**
 * If SE atomic context save/restore is enabled, only clear
 * TZRAM up to the TZRAM carveouts reserved for SE context blobs.
 * Otherwise, clear all TZRAM.
 *
 * Must be called AFTER atomic SE context save feature is enabled
 * as this function uses the enablement of this feature to know
 * how much TZRAM to clear.
 */
void NvBootSeClearTzram(void);

/**
 * Set up the SE engine for the particular mode you wish to use.
 *
 * The caller is responsible to provide valid parameter inputs.
 * For example Mode = AES, Dst = RSA_REG is not valid but this function
 * won't reject this combination.
 * @param Instance SE instance targeted (Se1 or Se2).
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
NvBootSeInstanceSetupOpMode (
        NvBootSeInstance Instance,
        NvBootSeOperationMode Mode,
        NvBool Encrypt,
        NvBool UseOrigIv,
        NvU8 Dst,
        NvU8 KeySlot,
        NvU8 KeySize);

void NvBootSeDisableEngine(void);

void NvBootSeInstanceDisableKeySlotReadAccess(NvBootSeInstance SeInstance, NvU8 KeySlot, NvU8 KeySlotType);

void NvBootSeInstanceDisableKeySlotWriteAccess(NvBootSeInstance SeInstance, NvU8 KeySlot, NvU8 KeySlotType);

void NvBootSeDisableKeySlotWriteAccess(NvU8 KeySlot, NvU8 KeySlotType);

void NvBootSeInstanceKeySlotReadKeyIV(NvBootSeInstance Instance, NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData);

void NvBootSeInstanceKeySlotWriteKeyIV(NvBootSeInstance Instance, NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData);

void NvBootSeLockSbk(void);

void  NvBootSeAesDecryptContext(NvU32 *pEncryptedContext, NvU32 *pDecryptedContext);

/**
 * Swap Exponent and Modulus of RSA key struct.
 */
void NvBootSeRsaKeySlotSwapModExp(NvBootSeRsaKey2048 *RsaKey);

//TODO: make static?
NvBool  NvBootSeCheckKnownPattern(NvU8 *pPattern);

NvBootError NvBootLP0ContextRestore(void);

/**
 *  Stage 1 starts non-blocking Decryption of SE blob.
 */
void NvBootLP0ContextRestoreStage1(NvBootSeInstance SeInstance, void *pSeDecryptedContext);

/**
 *  Disables SE instance and locks key slots.
 */
void NvBootLP0ContextRestoreDisable(NvBootSeInstance SeInstance, void *pSeDecryptedContext);

/**
 *  Parses the decrypted context, set key slots and sticky bits according to SE Instance
 */
void NvBootLP0ContextRestoreStage2(NvBootSeInstance SeInstance, void *pSeDecryptedContext);

/**
 *  Restore SE_SECURITY_0 (per instance), SE_TZRAM_SECURITY, TZRAM_SECURITY, PKA1_SECURITY
 *  after all SE operations/register writes are done. Uses the Context struct
 *  to save the values to be restored.
 */
NvBootError NvBootLP0ContextRestoreStage3(void);

/**
 * Parse Decrypted context into register buffer. 
 */
void NvBootSeInstanceParseDecryptedContext(
            NvBootSeInstance    Instance,
            void *SeDecryptedContext,
            NvBootSeContextStickyBitsRegBuf *pNvBootSeContextStickyBitsRegBuf);
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
/*
 * Archiving Se Instance RSA read key.
 * void NvBootSeInstanceRsaReadKey(NvBootSeInstance SeInstance, NvU32 *pKeyBuffer, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot, NvU8 ExpModSel);
 */

void NvBootSeInstanceRsaWriteKey(NvBootSeInstance SeInstance, NvU32 *pKeyBuffer, NvU32 RsaModulusSizeBits, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot);
void NvBootSeRsaClearKeySlot(NvU8 RsaKeySlot);
void NvBootSeInstanceRsaClearKeySlot(NvBootSeInstance SeInstance, NvU8 RsaKeySlot);

/**
 * Initiate Modular exponentiation operation for a particular SE instance
 */
NvBootError NvBootSeRsaModularExp(NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 InputMessageLengthBytes, NvU32 *pInputMessage, NvU32 *pOutputDestination);

NvBootError NvBootSeInstanceRsaModularExp(NvBootSeInstance SeInstance, NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 InputMessageLengthBytes, NvU32 *pInputMessage, NvU32 *pOutputDestination);

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
 * Encrypt AES-block size chunks of data using ECB block mode of operation.
 *
 * @param SeInstance Instance number of SE engine
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
void NvBootSeInstanceAesEncryptECBStart (
        NvBootSeInstance SeInstance,
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst);

/**
 *  Non blocking and instanced version of Aes Decrypt
 */
void NvBootSeInstanceAesDecryptStart (
        NvBootSeInstance SeInstance,
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
 * @param SourceKeySlot The SE key slot loaded with the AES key used to decrypt the key data.
 * @param SourceKeySize The AES key size of SourceKeySlot.
 * @param TargetKeySlot The SE key slot we are decrypting directly to.
 * @param TargetKeySize The AES key size of the decrypted AES key loaded.
 * @param Src Pointer to the encrypted input data chunk.
 *
 */
void NvBootSeInstanceAesDecryptKeyIntoKeySlot (
        NvBootSeInstance Instance,
        NvU8    SourceKeySlot,
        NvU8    SourceKeySize,
        NvU8    TargetKeySlot,
        NvU8    TargetKeySize,
        NvU8   *Src);

/**
 *  Set up macros fixing instance to either SE1 or SE2. Provides backwards compatibility with
 *  routines calling SE1.
 */
#define NvBootSeSetupOpMode(Mode, Encrypt, UseOrigIv, Dst, KeySlot, KeySize) \
    NvBootSeInstanceSetupOpMode(NvBootSeInstance_Se1, Mode, Encrypt, UseOrigIv, Dst, KeySlot, KeySize)
#define NvBootSe2SetupOpMode(Mode, Encrypt, UseOrigIv, Dst, KeySlot, KeySize) \
    NvBootSeInstanceSetupOpMode(NvBootSeInstance_Se2, Mode, Encrypt, UseOrigIv, Dst, KeySlot, KeySize)
/***************************************************************************************************/
#define NvBootSeIsEngineBusy(DestAddr)   NvBootSeInstanceIsEngineBusy(NvBootSeInstance_Se1, DestAddr)
#define NvBootSe2IsEngineBusy(DestAddr)  NvBootSeInstanceIsEngineBusy(NvBootSeInstance_Se2, DestAddr)
/***************************************************************************************************/
#define NvBootSeKeySlotReadKeyIV(KeySlot, KeySize, KeyType, KeyData) \
        NvBootSeInstanceKeySlotReadKeyIV(NvBootSeInstance_Se1, KeySlot, KeySize, KeyType, KeyData)

#define NvBootSe2KeySlotReadKeyIV(KeySlot, KeySize, KeyType, KeyData) \
        NvBootSeInstanceKeySlotReadKeyIV(NvBootSeInstance_Se2, KeySlot, KeySize, KeyType, KeyData)
/***************************************************************************************************/
#define NvBootSeKeySlotWriteKeyIV(KeySlot, KeySize, KeyType, KeyData) \
    NvBootSeInstanceKeySlotWriteKeyIV(NvBootSeInstance_Se1, KeySlot, KeySize, KeyType, KeyData)

#define NvBootSe2KeySlotWriteKeyIV(KeySlot, KeySize, KeyType, KeyData) \
    NvBootSeInstanceKeySlotWriteKeyIV(NvBootSeInstance_Se2, KeySlot, KeySize, KeyType, KeyData)    
/***************************************************************************************************/
#define NvBootSeAesDecryptKeyIntoKeySlot(SourceKeySlot, SourceKeySize, TargetKeySlot, TargetKeySize, Src) \
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(NvBootSeInstance_Se1, SourceKeySlot, SourceKeySize, TargetKeySlot, TargetKeySize, Src)

#define NvBootSe2AesDecryptKeyIntoKeySlot(SourceKeySlot, SourceKeySize, TargetKeySlot, TargetKeySize, Src) \
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(NvBootSeInstance_Se2, SourceKeySlot, SourceKeySize, TargetKeySlot, TargetKeySize, Src)
/***************************************************************************************************/
#define NvBootSetSeReg(Reg, Data) NvBootSetSeInstanceReg(NvBootSeInstance_Se1, Reg, Data)
#define NvBootSetSe2Reg(Reg, Data) NvBootSetSeInstanceReg(NvBootSeInstance_Se2, Reg, Data)
/***************************************************************************************************/
#define NvBootGetSeReg(Reg) NvBootGetSeInstanceReg(NvBootSeInstance_Se1, Reg)
#define NvBootGetSe2Reg(Reg) NvBootGetSeInstanceReg(NvBootSeInstance_Se2, Reg)
/***************************************************************************************************/
#define NvBootSeDisableKeySlotReadAccess(KeySlot, KeySlotType) \
        NvBootSeInstanceDisableKeySlotReadAccess(NvBootSeInstance_Se1, KeySlot, KeySlotType)
/***************************************************************************************************/
#define NvBootSeRsaWriteKey(pKeyBuffer, RsaModulusSizeBits, RsaKeySizeBits, RsaKeySlot) \
        NvBootSeInstanceRsaWriteKey(NvBootSeInstance_Se1, pKeyBuffer, RsaModulusSizeBits, RsaKeySizeBits, RsaKeySlot)
/***************************************************************************************************/

/**
 * Function that cleans up SE state and locks any necessary
 * key slots. Should be called at BR secure exit.
 */
NvBootError NvBootSeHousekeepingBeforeBRExit();

/**
 * Save all SE1/SE2/PKA1 sticky bits to NvBootSePkaState.
 */
void NvBootSeGetSeStickyBits(NvBootSePkaState *pSePkaState);

/**
 * Calculate KCVs of SE key slots.
 * Save PKA key slots (leave them read unlocked).
 * Save all sticky bits.
 * Save all data to pSaveLocation.
 *
 */
void NvBootSeGetSePkaState(NvBootSePkaState *pSePkaState);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SE_AES_INT_H

