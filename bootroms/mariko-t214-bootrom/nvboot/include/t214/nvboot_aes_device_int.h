/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_AES_DEVICE_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_AES_DEVICE_INT_H_

#include <stddef.h>
#include <stdbool.h>
#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_crypto_aes_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * @section DESCRIPTION
 *
 * Function pointers to common functions of the AES device construct.
 *
 */

typedef enum
{
    NvBootAesDeviceStatus_Success,
    NvBootAesDeviceStatus_Busy,
    NvBootAesDeviceStatus_Timeout,

    NvBootAesDeviceStatus_Num,
} NvBootAesDeviceStatus;

// Crypto engine agnostic list of key slot allocations.
typedef enum
{
    // List of supported key slots.
    AES_DEVICE_KEYSLOT_0,
    AES_DEVICE_KEYSLOT_1,
    AES_DEVICE_KEYSLOT_2,
    AES_DEVICE_KEYSLOT_3,
    AES_DEVICE_KEYSLOT_4,
    AES_DEVICE_KEYSLOT_5,
    AES_DEVICE_KEYSLOT_6,
    AES_DEVICE_KEYSLOT_7,
    AES_DEVICE_KEYSLOT_8,
    AES_DEVICE_KEYSLOT_9,
    AES_DEVICE_KEYSLOT_10,
    AES_DEVICE_KEYSLOT_11,
    AES_DEVICE_KEYSLOT_12,
    AES_DEVICE_KEYSLOT_13,
    AES_DEVICE_KEYSLOT_14,
    AES_DEVICE_KEYSLOT_15,

    AES_DEVICE_KEYSLOT_NUM,

    // Put key slot allocations below.
    AES_DEVICE_KEYSLOT_NV_FEK = AES_DEVICE_KEYSLOT_0,
    AES_DEVICE_KEYSLOT_OEM_FEK = AES_DEVICE_KEYSLOT_1,

    // AES_DEVICE_KEYSLOT_FSKP_KWK_DECRYPT = key slot to load the special
    // FSKP key that decrypts the user specified key wrap key.
    AES_DEVICE_KEYSLOT_FSKP_KWK_DECRYPT = AES_DEVICE_KEYSLOT_10,
    AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE = AES_DEVICE_KEYSLOT_11,
    AES_DEVICE_KEYSLOT_FSKP_DECRYPT = AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE,
    AES_DEVICE_KEYSLOT_KEK = AES_DEVICE_KEYSLOT_12,
    AES_DEVICE_KEYSLOT_BEK = AES_DEVICE_KEYSLOT_13,
    AES_DEVICE_KEYSLOT_SBK = AES_DEVICE_KEYSLOT_14,
    AES_DEVICE_KEYSLOT_SSK = AES_DEVICE_KEYSLOT_15,
} NvBootAesDeviceKeySlot;

/**
 * Function to initialize the AES device.
 *
 * Note: This function must be called before any other function
 * in this file is called.
 *
 */
typedef NvBootError (*NvBootAesDeviceInit)(void);

/**
 * Function that returns the number of key slots
 * present in the AES device.
 *
 * All AES "devices" use a "key slot" model, that is, an AES key
 * must first be "loaded" using a Set function into a key slot before
 * the AES operation is performed.
 * This behavior allows a consistent usage model when different AES engines
 * are present in the chip. It also allows the usage of a pure software
 * AES implementation.
 *
 * @return A uint8_t with the number of key slots in the particular AES
 * device.
 *
 */
typedef uint8_t (*NvBootAesDeviceGetNumKeySlots)(void);

/**
 * Function that returns whether or not
 * a Keyslot is valid.
 *
 * @return true if the key slot is a valid number, false otherwise.
 */
typedef bool (*NvBootAesDeviceIsValidKeySlot)(uint8_t KeySlot);


/**
 * Function to read an AES key from a key slot.
 * @param[out] Key The buffer to read the AES key into.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key size to read (128, 192, or 256 bit).
 *
 * @return NvBootError.
 */
typedef NvBootError (*NvBootAesDeviceGetKey)(uint32_t *Key,
		                                     uint8_t KeySlot,
											 NvBootAesKeySize KeySize);

/**
 * Function to read an original IV from a key slot.
 * @param[out] Key The buffer to read the AES IV into.
 * @param[in] KeySlot The key slot number.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 * Note: IVs are always 128-bits.
 */
typedef NvBootError (*NvBootAesDeviceGetOriginalIv)(uint32_t *Key,
		                                      	    uint8_t KeySlot);

/**
 * Function to read an updated IV from a key slot.
 * @param[out] Key The buffer to read the AES IV into.
 * @param[in] KeySlot The key slot number.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 * Note: IVs are always 128-bits.
 */
typedef NvBootError (*NvBootAesDeviceGetUpdatedIv)(uint32_t *Key,
		                                      	   uint8_t KeySlot);

/**
 * Function to load an AES key into a key slot.
 * @param[out] Key The buffer to read the AES key into.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key size to read (128, 192, or 256 bit).
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
typedef NvBootError (*NvBootAesDeviceSetKey)(const uint32_t *Key,
                                             uint8_t KeySlot,
                                             NvBootAesKeySize KeySize);

/**
 * Function to load an updated IV to a key slot.
 * @param[out] Iv The buffer to read the AES IV into.
 * @param[in] KeySlot The key slot number.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 * Note: IVs are always 128-bits.
 */
typedef NvBootError (*NvBootAesDeviceSetOriginalIv)(const uint32_t *Iv,
                                                    uint8_t KeySlot);

/**
 * Function to load an updated IV to a key slot.
 * @param[out] Iv The buffer to read the AES IV into.
 * @param[in] KeySlot The key slot number.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 * Note: IVs are always 128-bits.
 */
typedef NvBootError (*NvBootAesDeviceSetUpdatedIv)(const uint32_t *Iv,
                                                   uint8_t KeySlot);

/**
 * Function to clear a key slot to zeroes.
 * This function will always clear all 256-bits of a key slot.
 * This function will have no effect if the write protection is set
 * on the key slot.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 */
typedef NvBootError (*NvBootAesDeviceClearKey)(uint8_t KeySlot);

/**
 * Function to clear the Original IV of a key slot to zeroes.
 * This function will always clear all 256-bits of a key slot.
 * This function will have no effect if the write protection is set
 * on the key slot.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 */
typedef NvBootError (*NvBootAesDeviceClearOriginalIv)(uint8_t KeySlot);

/**
 * Function to clear the Updated IV of a key slot to zeroes.
 * This function will always clear all 256-bits of a key slot.
 * This function will have no effect if the write protection is set
 * on the key slot.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 */
typedef NvBootError (*NvBootAesDeviceClearUpdatedIv)(uint8_t KeySlot);

/**
 * Function to set enable the read protection of a key slot.
 * Enabling the read protection means this key slot can no longer be
 * read.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
typedef NvBootError (*NvBootAesDeviceSetReadProtection)(uint8_t KeySlot);

/**
 * Function to set enable the write protection of a key slot.
 * Enabling the read protection means this key slot can no longer be
 * written.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
typedef NvBootError (*NvBootAesDeviceSetWriteProtection)(uint8_t KeySlot);


typedef struct NvBootAesDeviceKeySlotContextRec
{
    bool isReadLocked;
    bool isWriteLocked;
} NvBootAesDeviceKeySlotContext;

/**
 * Function that returns the NvBootAesDeviceKeySlotContext of a key slot.
 */
typedef NvBootError (*NvBootAesDeviceGetKeySlotStatus)(NvBootAesDeviceKeySlotContext *AesDeviceKeySlotContext);

/**
 * Function to do an AES-CBC encrypt operation.
 * @param[in] Plaintext buffer to be encrypted.
 * @param[out] Ciphertext buffer, where the encrypted values are stored.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 * Note: The plaintext buffer and the Ciphertext buffer can be the same.
 */
typedef NvBootError (*NvBootAesDeviceEncryptCBC)(const uint8_t *Plaintext,
                                                 uint8_t *Ciphertext,
                                                 uint8_t KeySlot,
                                                 NvBootAesKeySize KeySize,
                                                 size_t Length);

/**
 * Function to do an AES-CBC decrypt operation.
 * @param[in] Ciphertext The buffer to be decrypted.
 * @param[out] Plaintext The buffer where the decrypted values are stored.
 * @param[in] KeySlot The key slot number.
 * @param[in] KeySize The key size.
 * @param[in] Length Length in bytes of the input.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 *
 * Note: The plaintext buffer and the Ciphertext buffer can be the same.
 */
typedef NvBootError (*NvBootAesDeviceDecryptCBC)(const uint8_t *Ciphertext,
                                                 uint8_t *Plaintext,
                                                 uint8_t KeySlot,
                                                 NvBootAesKeySize KeySize,
                                                 size_t Length);

/**
 * Function to decrypt an encrypted key directly into a keyslot.
 * @param[in] KeySlot The key slot number.
 * @param[in] KeySize The key size.
 * @param[in] Ciphertext A pointer to the ciphertext to be decrypted.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
typedef NvBootError (*NvBootAesDeviceDecryptIntoKeySlot)(uint8_t KeySlot,
                                                         NvBootAesKeySize KeySize,
                                                         const uint32_t *Ciphertext);


/**
 * Function to generate the subkeys for the AES-CMAC operation.
 * @param[in] KeySlot The key slot number.
 * @param[in] KeySize The key size.
 * @param[in] pK1 A pointer to the buffer to store K1.
 * @param[in] pK2 A pointer to the buffer to store K2.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
typedef NvBootError (*NvBootAesDeviceGenerateCmacSubKey)(uint8_t KeySlot,
                                                         NvBootAesKeySize KeySize,
                                                         uint32_t *pK1,
                                                         uint32_t *pK2);

/**
 * Generic strut to store AES-CMAC parameters
 */
typedef struct NvBootAesDeviceCmacContextRec
{
	uint32_t *pK1;
    uint32_t *pK2;
    uint32_t *pInputMessage;
    uint32_t *pHash;
    uint8_t KeySlot;
    NvBootAesKeySize KeySize;
    uint32_t NumBlocks;
    bool FirstChunk;
    bool LastChunk;
} NvBootAesDeviceCmacContext;

/**
 * Function to generate an AES-CMAC signature.
 *
 * @param[in] AesCmacContext A pointer to the AES CMAC context.
 */
typedef NvBootError (*NvBootAesDeviceCmacHash)(NvBootAesDeviceCmacContext *AesCmacContext);

/**
 *  Shutdown the AES device and clean up state.
 */
typedef void (*NvBootAesDeviceShutdown)(void);


#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_AES_DEVICE_INT_H_ */
