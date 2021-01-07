/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_SHA_DEVICE_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_SHA_DEVICE_INT_H_

#include <stdbool.h>
#include "nvboot_crypto_sha_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Inititalize the SHA device and ready it for SHA operations.
 *
 * @param[in] ShaConfig Pointer to a ShaConfig struct.
 *
 * @return NvBootError.
 */
typedef NvBootError (*NvBootShaDeviceInit)(NvBootCryptoShaConfig *ShaConfig);

/**
 * Get the currently configured SHA family for SHA device.
 *
 * @param[in] ShaConfig Pointer to a ShaConfig struct.
 *
 * @return NvBootCryptoShaFamily.
 */
typedef NvBootCryptoShaFamily (*NvBootShaDeviceGetShaFamily)(NvBootCryptoShaConfig *ShaConfig);

/**
 * Set the SHA family if supported by the SHA device.
 *
 * @param[in] ShaConfig Pointer to a ShaConfig struct.
 * @param[in] ShaFamily Set the propsective SHA Family is supported.
 *
 * @return bool Returns true if the ShaFamily is supported. false otherwise.
 *
 */
typedef bool (*NvBootShaDeviceSetShaFamily)(NvBootCryptoShaConfig *ShaConfig, NvBootCryptoShaFamily ShaFamily);

/**
 * Get the currently configured digest size
 *
 * @param[in] ShaConfig Pointer to a ShaConfig struct.
 *
 * @return NvBootCryptoShaDigestSize.
 */
typedef NvBootCryptoShaDigestSize (*NvBootShaDeviceGetShaDigestSize)(NvBootCryptoShaConfig *ShaConfig);

/**
 * Set the SHA digest size if supported by the SHA device.
 * @param[in] ShaConfig Pointer to a ShaConfig struct.
 * @param[in] DigestSize Prospective digest size.
 *
 * @return bool Returns true if the digest size is supported. false otherwise.
 */
typedef bool (*NvBootShaDeviceSetShaDigestSize)(NvBootCryptoShaConfig *ShaConfig, NvBootCryptoShaDigestSize DigestSize);

/**
 * Check if a particular SHA family is supported by the SHA device.
 *
 * @param[in] ShaFamily SHA family i.e. SHA2, SHA3.
 * @return bool Returns true if the SHA family is supported. false otherwise.
 *
 */
typedef bool (*NvBootShaDeviceIsValidShaFamily)(NvBootCryptoShaFamily ShaFamily);

/**
 * Check if a particular SHA digest size is supported by the SHA device.
 *
 * @param[in] DigestSize Digest size in bits.
 * @return bool REturns true if the digest size is supported. false otherwise.
 */
typedef bool (*NvBootShaDeviceIsValidShaDigestSize)(NvBootCryptoShaDigestSize DigestSize);

/**
 * Function to calculate the SHA(2,3) hash of an input message.
 *
 * @param[in] InputMessage Pointer to the input message buffer.
 * @param[in] InputMessageLength Length in bytes of the input message. See fips-180-4
 *            for padding information.
 * @param[out] Hash The resulting calculated hash.
 * @param[in] DigestSize Specifies the digest size in bits for the SHA2 algorithm.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
typedef NvBootError (*NvBootShaDeviceShaHash)(const uint32_t *InputMessage,  uint32_t InputMessageLength, uint32_t *Hash, NvBootCryptoShaConfig *ShaConfig);

/**
 *  Shutdown the SHA device and clean up state.
 */
typedef void (*NvBootShaDeviceShutdown)(void);

#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_SHA_DEVICE_INT_H_ */
