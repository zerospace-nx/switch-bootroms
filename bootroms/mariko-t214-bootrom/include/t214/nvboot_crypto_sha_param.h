/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H
#define INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H

#include "nvtypes.h"
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * SHA256 Hash of the empty string; Needed for T214 SE WAR. See http://nvbugs/1788437.
 */
static const NvU8 Sha256NullStringDigest[] =
{
    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
};

typedef enum NvBootCryptoSha2AlgorithmRec
{
    // Algorithm = Digest Size in bits
    SHA2_256 = 256,

    SHA2_INVALID,
} NvBootCryptoSha2Algorithm;

typedef enum NvBootCryptoShaDigestSizeRec
{
    SHA_256 = 256,
    SHA_384 = 384,
    SHA_512 = 512,

    SHA_INVALID_DIGEST,
} NvBootCryptoShaDigestSize;

typedef enum NvBootCryptoShaFamilyRec
{
    SHA2 = 2,
    SHA3,

    SHA_INVALID_FAMILY,
} NvBootCryptoShaFamily;

typedef struct NvBootCryptoShaConfigRec
{
    NvBootCryptoShaFamily ShaFamily;
    NvBootCryptoShaDigestSize ShaDigestSize;
} NvBootCryptoShaConfig;


/**
 * Defines the length of a SHA256 hash in bytes
 *
 */
enum {NVBOOT_SHA256_LENGTH_BYTES = SHA_256 / 8};

/**
 * Defines the length of a SHA256 hash in words
 *
 */
enum {NVBOOT_SHA256_LENGTH_WORDS = NVBOOT_SHA256_LENGTH_BYTES / 4};

enum {NVBOOT_SHA2_MAX_BYTE_SIZE = SHA_512 / 8};

typedef struct NvBootSha256HashDigestRec
{
    NvU8 Hash[NVBOOT_SHA256_LENGTH_BYTES];
} NvBootSha256HashDigest NV_ALIGN(NVBOOT_CRYPTO_BUFFER_ALIGNMENT);

#if defined(__cplusplus)
}
#endif

#endif /* INCLUDE_NVBOOT_CRYPTO_SHA_PARAM_H */
