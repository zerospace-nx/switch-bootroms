/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_RSA_DEVICE_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_RSA_DEVICE_INT_H_

#include <stddef.h>
#include <stdbool.h>
#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_crypto_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif


/*
 * @section DESCRIPTION
 *
 * Function pointers to common functions of the RSA device construct.
 *
 */
typedef enum
{
    NvBootRsaDeviceStatus_Success,
    NvBootRsaDeviceStatus_Busy,
    NvBootRsaDeviceStatus_Timeout,

    NvBootRsaDeviceStatus_Num,
} NvBootRsaDeviceStatus;

/**
 * Function to initialize the Rsa device.
 *
 * Note: This function must be called before any other function
 * in this file is called.
 *
 */
typedef NvBootError (*NvBootRsaDeviceInit)(void);


/**
 * Function that returns the number of key slots
 * present in the RSA device.
 *
 * All RSA "devices" use a "key slot" model, that is, an RSA key
 * must first be "loaded" using a Set function into a key slot before
 * the RSA operation is performed.
 * This behavior allows a consistent usage model when different RSA engines
 * are present in the chip. It also allows the usage of a pure software
 * RSA implementation.
 *
 * @return A uint8_t with the number of key slots in the particular RSA
 * device.
 *
 */
typedef uint8_t (*NvBootRsaDeviceGetNumKeySlots)(void);


/**
 * Function that returns whether or not
 * a Keyslot is valid.
 *
 * @return true if the key slot is a valid number, false otherwise.
 */
typedef bool (*NvBootRsaDeviceIsValidKeySlot)(uint8_t KeySlot);

/**
 * Function that returns whether or not
 * a RSA key size is valid for the particular device.
 *
 * @return true if the key slot is a valid number, false otherwise.
 */
typedef bool (*NvBootRsaDeviceIsValidKeySize)(NvBootCryptoRsaKeySize KeySize);

/**
 * Function to read an RSA key from a key slot.
 * @param[out] Key The buffer to read the RSA key into.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key RSA size to read.
 *
 * @return NvBootError.
 */
typedef NvBootError (*NvBootRsaDeviceGetKey)(NvBootCryptoRsaKey *Key,
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
typedef NvBootError (*NvBootRsaDeviceSetKey)(const NvBootCryptoRsaKey *Key,
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
typedef NvBootError (*NvBootRsaDeviceModularExponentiation)(const uint32_t *Base,
                                                            uint32_t *Result,
                                                            const uint8_t KeySlot,
											                const NvBootCryptoRsaKeySize KeySize);
/**
 * Function to check if the RSA device is busy running a calculation
 *
 * @return true if the RSA device is busy, false otherwise.
 */
typedef bool (*NvBootRsaDeviceIsDeviceBusy)(void);

/**
 *  Shutdown the Rsa device and clean up state.
 */
typedef void (*NvBootRsaDeviceShutdown)(void);

#if defined(__cplusplus)
extern "C"
{
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_RSA_DEVICE_INT_H_ */
