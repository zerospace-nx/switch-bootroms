/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_CRYPTO_PKC_RSASSA_PSS_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_CRYPTO_PKC_RSASSA_PSS_INT_H_

#include <stdbool.h>
#include "nvboot_crypto_param.h"
#include "nvboot_sha_devmgr_int.h"
#include "nvboot_rsa_devmgr_int.h"
#include "nvboot_crypto_sha_param.h"
#include "nvboot_crypto_rsa_param.h"
#include "nvboot_util_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// PKCS #1 v2.2 / RFC 3447 spec:
// hLen is the length in octets of the hash function used.
// sLen is the length in octets of the salt.
// Generally it is recommended that the salt length be 0 or hLen.
static const uint32_t NvBootCryptoRsaSsaPssSaltLength = NV_ICEIL(SHA2_256, 8);

// These variables control the hash function used for the MGF and RSASSA-PSS
// authentication steps.
static const NvBootCryptoShaConfig RsaSsaPssHashFunction = {SHA2, SHA2_256};
static const uint32_t RsaSsaPssHashFunctionSizeBytes = NVBOOT_SHA256_LENGTH_BYTES;
static const uint32_t RsaSsaPssHashFunctionSizeWords = NVBOOT_SHA256_LENGTH_WORDS;

#define RSASSA_PSS_HASH_FUNCTION_SIZE_WORDS NVBOOT_SHA256_LENGTH_WORDS
#define RSASSA_PSS_HASH_FUNCTION_SIZE_BYTES NVBOOT_SHA256_LENGTH_BYTES

typedef struct NvBootCryptoRsaSsaPssContextRec
{
    // RSA key slot holding the RSA key.
    uint8_t RsaKeySlot;
    // We need to access the modulus to check if s is between 0 and n-1.
    NvBootCryptoRsaKey *RsaKey;
    // InputMessage to be hashed
    uint32_t *InputMessage;

    // InputMessageIsHashed == true means the hash of the InputMessage has already been pre-computed.
    bool InputMessageIsHashed;
    uint32_t *InputMessageShaHash; // Must be a valid address if InputMessageIsHashed == true.

    uint32_t InputMessageLengthBytes;
    // The Signature Representative, in octet (byte)
    // stream format. SE engine expects the integer to be in octet string format,
    // not a non-negative integer.
    NvBootCryptoRsaSsaPssSig *InputSignature;
} NvBootCryptoRsaSsaPssContext;

/**
 * This function initializes any crypto buffers and pointers used by
 * this module.
 */
NvBootError NvBootCryptoRsaSsaPssInit(void);

/**
 * This is the mask generation function as specified in the RSASSA-PSS specification.
 * @param[in] mgfSeed is a pointer to H.
 * @param[in] maskLen = emLen - hLen - 1.
 * @param[in] dbMaskBuffer, the output of the mask generation function.
 * @param[in] Pinter to a SHA device manager.
 *
 * @return NvBootError_Success if no error. Any other value, see nvboot_error.h
 */
NvBootError NvBootCryptoRsaSsaPssMGF(uint8_t *mgfSeed, uint32_t maskLen, uint8_t *dbMaskBuffer, NvBootShaDevMgr *ShaDevMgr);

/**
 *  This function runs a RSASSA-PSS-VERIFY signature verification operation per
 *  PKCS #1 v.2.2.
 *
 *  @param[in] RsaSsaPssContext Pointer to NvBootCryptoRsaSsaPssContext.
 *  @param[in] ShaDevMgr Pointer to NvBootShaDevMgr, for SHA2 operations.
 *  @param[in] RsaDevMgr Pointer to NvBootRsaDevMgr. for RSA modular exponentiation
 *             operations.
 *
 *  @return NvBootError_Success if the signature comparision is successful, any other value,
 *          see nvboot_error.h
 */
NvBootError NvBootCryptoRsaSsaPssVerify(NvBootCryptoRsaSsaPssContext *RsaSsaPssContext, NvBootShaDevMgr *ShaDevMgr, NvBootRsaDevMgr *RsaDevMgr);

#if defined(__cplusplus)
extern "C"
{
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_CRYPTO_PKC_RSASSA_PSS_INT_H_ */
