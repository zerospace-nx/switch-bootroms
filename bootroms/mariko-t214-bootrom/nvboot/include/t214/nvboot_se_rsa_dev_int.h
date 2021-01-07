/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_SE_RSA_DEV_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_SE_RSA_DEV_INT_H_

#include <stdbool.h>
#include "nvboot_crypto_param.h"
#include "nvboot_error.h"
#include "nvboot_se_rsa.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Function for any required initialization sequence for SE.
 * Note, clock init should be taken care of in the non-secure
 * dispatcher. This function should take care of other non-clock
 * programming.
 *
 * @return NvBootError.
 */
NvBootError NvBootSeRsaDevInit(void);

/**
 * SE implementation of the NvBootRsaDeviceNumKeySlots function
 * of the nvboot_rsa_device_int.h interface.
 *
 * @return The maximal number of RSA key slots.
 */
uint8_t NvBootSeRsaDevNumKeySlots(void);

/**
 * SE implementation of the NvBootRsaDeviceIsValidKeySlot function
 * of the nvboot_rsa_device_int.h interface
 *
 * @param[in] KeySlot The KeySlot number to validate.
 *
 * @return true if KeySlot is a valid key slot number. false otherwise.
 */
bool NvBootSeRsaDevIsValidKeySlot(uint8_t KeySlot);

/**
 * SE implementation of the NvBootRsaDeviceIsValidKeySize function
 * of the nvboot_rsa_device_int.h interface
 *
 * @param[in] KeySize The RSA key size to validate.
 *
 * @return true if KeySlot is a supported key size for SE. (Note, SE may support
 *                 key sizes than the driver allows).
 */
bool NvBootSeRsaDevIsValidKeySize(NvBootCryptoRsaKeySize KeySlot);

/**
 * Function to read an RSA key from a SE RSA key slot.
 * @param[out] Key The buffer to read the RSA key into.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key size to read.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
NvBootError NvBootSeRsaDevGetKey(NvBootCryptoRsaKey *Key,
                                 const uint8_t KeySlot,
                                 const NvBootCryptoRsaKeySize KeySize);


/**
 * Function to load an RSA key to a key slot.
 * @param[in] Key The RSA key to load into the key slot
 * @param[in] KeySlot The key slot number.
 *
 * Note: Key size information is embedded in the NvBootCryptoRsaKey
 * struct.
 */
NvBootError NvBootSeRsaDevSetKey(const NvBootCryptoRsaKey *Key,
                                 const uint8_t KeySlot);

/**
 * Function to calculate the exponentiation of a base with modulus, of the form
 * c = b ^ e (mod m), where
 * c is the result
 * b is the base, the integer that is repeatedly multiplied
 * e is the exponent
 * m is the modulus
 *
 * @param[in] Base, what is to be repeatedly multiplied
 * @param[out] Result he result of the calculation.
 * @param[in] KeySlot The key slot number of the RSA key to use, which specifies
 *                    the modulus and exponent.
 * @param[in] KeySize A valid RSA key size.
 *
 */
NvBootError NvBootSeRsaDevModularExponentiation(const uint32_t *Base,
                                                uint32_t *Result,
                                                const uint8_t KeySlot,
                                                const NvBootCryptoRsaKeySize KeySize);

/**
 * Function to check if the RSA device is busy running a calculation
 *
 * @return true if the RSA device is busy, false otherwise.
 */
bool NvBootSeRsaDevIsEngineBusy(void);

/**
 *  Shutdown the Rsa device and clean up state.
 */
void (*NvBootSeRsaDevShutdown)(void);

#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_SE_RSA_DEV_INT_H_ */
