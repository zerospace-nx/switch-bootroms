/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * \file nvboot_crypto_aes_param.h
 *
 * Defines the parameters and data structures for AES.
 *
 * AES is used for encryption, decryption, and signatures.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_AES_H
#define INCLUDED_NVBOOT_CRYPTO_AES_H

#include "nvtypes.h"
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * NEW AES ENUMS HERE!!!!!!!!!!!!!
 */
typedef enum
{
    AesKey128 = 128, //to deprecate
    AesKey192 = 192, //to deprecate
    AesKey256 = 256, //to deprecate

    AES_KEY_128 = 128,
    AES_KEY_192 = 192,
    AES_KEY_256 = 256,

    AES_KEY_INVALID,
} NvBootAesKeySize;

enum {NVBOOT_AES_BLOCK_LENGTH_BYTES = 16};

enum {NVBOOT_AES_BLOCK_LENGTH_WORDS = NVBOOT_AES_BLOCK_LENGTH_BYTES / 4};

/**
 * Defines the length of an AES block in units of log2 bytes.
 */
enum {NVBOOT_AES_BLOCK_LENGTH_LOG2 = 4};

enum {NVBOOT_AES_IV_LENGTH_BYTES = 16};

enum {NVBOOT_AES_IV_LENGTH_WORDS = NVBOOT_AES_IV_LENGTH_BYTES / 4};

enum {NVBOOT_AES_KEY_128_BYTES = 16};

enum {NVBOOT_AES_KEY_192_BYTES = 24};

enum {NVBOOT_AES_KEY_256_BYTES = 32};

enum {NVBOOT_AES_KEY_128_WORDS = NVBOOT_AES_KEY_128_BYTES / 4};

enum {NVBOOT_AES_KEY_192_WORDS = NVBOOT_AES_KEY_192_BYTES / 4};

enum {NVBOOT_AES_KEY_256_WORDS = NVBOOT_AES_KEY_256_BYTES / 4};

enum {NVBOOT_AES_KEY_MAX_BYTES = NVBOOT_AES_KEY_256_BYTES};

enum {NVBOOT_AES_KEY_MAX_WORDS = NVBOOT_AES_KEY_256_WORDS};

enum {NVBOOT_SECURE_BOOT_KEY_LENGTH_BYTES = NVBOOT_AES_KEY_128_BYTES};

enum {NVBOOT_AES128_KEYSCHED_BYTES = 176};
enum {NVBOOT_AES192_KEYSCHED_BYTES = 208};
enum {NVBOOT_AES256_KEYSCHED_BYTES = 240};

enum {NVBOOT_AES_MAX_KEYSCHED_BYTES = NVBOOT_AES256_KEYSCHED_BYTES};

#define MAX_KEYSCHED_BYTES (AES256_KEYSCHED_BYTES)

/**
 * Defines the value of const_Rb, as specified in the AES-CMAC specification.
 * See RFC 4493: http://tools.ietf.org/html/rfc4493.
 */
enum {NVBOOT_AES_CMAC_CONST_RB = 0x87};

/**
 * NEW AES ENUMS END
 */

typedef enum
{
    UserSpecified,
    SBK,
    AesInputKeyType_Num,
    AesInputKeyType_Force32 = 0x7fffffff,
} NvBootAesInputKeyType;

typedef enum
{
    ECB,
    CBC,
    CTR,
    AesBlockMode_Num,
    AesBlockMode_Force32 = 0x7fffffff,
} NvBootAesBlockCipherMode;


/**
 * Defines an 128-bit AES key.
 */
typedef struct NvBootAesKey128Rec
{
    /// Specifies the key data.
    NvU8 Key128[NVBOOT_AES_KEY_128_BYTES];
} NvBootAesKey128;

/**
 * Defines an 192-bit AES key.
 */
typedef struct NvBootAesKey192Rec
{
    /// Specifies the key data.
    NvU8 Key192[NVBOOT_AES_KEY_192_BYTES];
} NvBootAesKey192;

/**
 * Defines an 256-bit AES key.
 */
typedef struct NvBootAesKey256Rec
{
    /// Specifies the key data.
    NvU8 Key256[NVBOOT_AES_KEY_256_BYTES];
} NvBootAesKey256;
/**
 * Defines an AES Initial Vector (128-bits).
 */
typedef struct NvBootAesIvRec
{
    /// Specifies the initial vector data.
    NvU32 Iv[NVBOOT_AES_IV_LENGTH_WORDS];
} NvBootAesIv;

/**
 * Defines an AES key schedule buffer.
 */
typedef struct NvBootAesKeyScheduleRec
{
    NvU32 KeySchedule[NVBOOT_AES_MAX_KEYSCHED_BYTES/4];
} NvBootAesKeySchedule;

typedef union
{
    NvU8 Key128[NVBOOT_AES_KEY_128_BYTES];
    NvU8 Key192[NVBOOT_AES_KEY_192_BYTES];
    NvU8 Key256[NVBOOT_AES_KEY_256_BYTES];
    NvU8 KeyMax[NVBOOT_AES_KEY_MAX_BYTES]; // __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    NvU32 KeyMax32[NVBOOT_AES_KEY_MAX_WORDS]; // Force 32 bit alignment
} NvBootAesKey;

typedef struct NvBootAesCmacHashParamsRec
{
    NvU32 K1[NVBOOT_AES_BLOCK_LENGTH_WORDS]; //__attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    NvU32 K2[NVBOOT_AES_BLOCK_LENGTH_WORDS]; //__attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootAesCmacHashParams;

typedef struct NvBootAesParamsRec
{
    //NvBootCryptoMgrAesCryptoStatus Status;
    NvBootAesBlockCipherMode AesBlockMode;
    NvBootAesKey Key;
    NvBootAesKeySize KeySize;
    NvBootAesIv Iv;
    NvBootAesKeySchedule KeySchedule;
    NvBootAesCmacHashParams AesCmacParams;
    // PayloadSizeBytes must be multiple of an AES block.
    NvU32 PayloadSizeBytes;
} NvBootAesParams;

#if 0
typedef struct NvBootAesParamsRec
{
    //NvBootCryptoMgrAesCryptoStatus Status;
    NvBootAesBlockCipherMode AesBlockMode;
    NvBootAesKey Key;
    NvBootAesKeySize KeySize;
    NvBootAesIv Iv;
    NvBootAesKeySchedule KeySchedule;
    NvBootAesCmacHashParams AesCmacParams;
    // PayloadSizeBytes must be multiple of an AES block.
    NvU32 PayloadSizeBytes;
} NvBootAesParams;
#endif


// Struct for storage of an AES-CMAC hash.
typedef struct NvBootAesCmacHashRec
{
    // Hash is the calculated Message Authentication Code (MAC).
    NvU32 Hash[NVBOOT_AES_BLOCK_LENGTH_WORDS]; //__attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootAesCmacHash;


/**
 * Define a macro to check if the MSB of the input is set.
 */
#define NVBOOT_AES_CMAC_IS_MSB_SET(x)  ( (x) >> (8*sizeof(x)-1) )

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_CRYPTO_AES_H
