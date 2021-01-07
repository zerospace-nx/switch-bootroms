/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_SE_AES_DEV_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_SE_AES_DEV_INT_H_

#include <stdbool.h>
#include "nvboot_crypto_aes_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Convert from NvBootAesKeySize to uint32_t.
 *
 * @param[in] KeySize
 * @parblock
 * Convert common NvBootAesKeySize enum to SE specific
 * SE_MODE_PKT_AESMODE_KEYxxx defines.
 * @endparblock
 *
 * @return uint32_t in SE_MODE_PKT_AESMODE_KEYxxx format.
 */
uint32_t NvBootSeKeySizeConv(NvBootAesKeySize KeySize);

/**
 * Function for any required initialization sequence for SE.
 * Note, clock init should be taken care of in the non-secure
 * dispatcher. This function should take care of other non-clock
 * programming.
 *
 * @return NvBootError.
 */
NvBootError NvBootSeAesDevInit(void);

/**
 * SE implementation of the NvBootAesDeviceNumKeySlots function
 * of the nvboot_aes_device_int.h interface.
 *
 * @return The maximal number of AES key slots.
 */
uint8_t NvBootSeAesDevNumKeySlots(void);

// Implementation of NvBootAesDeviceIsValidKeySlot
/**
 * SE implementation of the NvBootAesDeviceIsValid function
 * of the nvboot_aes_device_int.h interface
 *
 * @param[in] KeySlot The KeySlot number to validate.
 *
 * @return true if KeySlot is a valid key slot number. false otherwise.
 */
bool NvBootSeAesDevIsValidKeySlot(uint8_t KeySlot);


/**
 * Function to read an AES key from a SE key slot.
 * @param[out] Key The buffer to read the AES key into.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key size to read (128, 192, or 256 bit).
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
NvBootError NvBootSeAesDevGetKey(uint32_t *Key,
                                 uint8_t KeySlot,
                                 NvBootAesKeySize KeySize);

/**
 * Function to load an AES key into a SE key slot.
 * @param[in] Key The buffer to read the AES key from.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key size to load (128, 192, or 256 bit).
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
NvBootError NvBootSeAesDevSetKey(const uint32_t *Key,
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
NvBootError NvBootSeAesDevSetOriginalIv(const uint32_t *Iv,
                                        uint8_t KeySlot);

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
NvBootError NvBootSeAesDevClearOriginalIv(uint8_t KeySlot);

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
NvBootError NvBootSeAesDevSetUpdatedIv(const uint32_t *Iv,
                                       uint8_t KeySlot);
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
NvBootError NvBootSeAesDevClearUpdatedIv(uint8_t KeySlot);

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
NvBootError NvBootSeAesDevEncryptCBC(const uint8_t *Plaintext,
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
NvBootError NvBootSeAesDevDecryptCBC(const uint8_t * Ciphertext,
                                     uint8_t * Plaintext,
                                     uint8_t KeySlot,
                                     NvBootAesKeySize KeySize,
                                     size_t Length);

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
NvBootError NvBootSeAesDevGenCmacSubkey(uint8_t KeySlot,
                                        NvBootAesKeySize KeySize,
                                        uint32_t *pK1,
                                        uint32_t *pK2);

/**
 * Function to generate an AES-CMAC signature.
 *
 * @param[in] AesCmacContext A pointer to the AES CMAC context.
 */
NvBootError NvBootSeAesCmacHash(NvBootAesDeviceCmacContext *AesCmacContext);


#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_SE_AES_DEV_INT_H_ */
