/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_RSA_PARAM_H
#define INCLUDED_NVBOOT_CRYPTO_RSA_PARAM_H

#include "nvboot_se_rsa.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum
{
    RsaKey2048,
    RsaKey3072,

    RSA_KEY_2048 = 2048,
    RSA_KEY_3072 = 3072,

    RSA_KEY_INVALID,
    RSA_KEY_FORCE32 = 0x7FFFFFFF,
} NvBootCryptoRsaKeySize;

//TODO: change to NVBOOT_CRYPTO_*

enum {NVBOOT_RSA_2048_MODULUS_SIZE_BYTES = 2048 / 8};

enum {NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES = 2048 / 8};

enum {NVBOOT_RSA_2048_MODULUS_SIZE_WORDS = 2048 / 8 / 4};

enum {NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS = 2048 / 8 / 4};

enum {NVBOOT_RSA_3072_MODULUS_SIZE_BYTES = 3072 / 8};

enum {NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES = 3072 / 8};

enum {NVBOOT_RSA_3072_MODULUS_SIZE_WORDS = 3072 / 8 / 4};

enum {NVBOOT_RSA_3072_EXPONENT_SIZE_WORDS = 3072 / 8 / 4};

enum {NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES = NVBOOT_RSA_2048_MODULUS_SIZE_BYTES};

enum {NVBOOT_RSA_MAX_EXPONENT_SIZE_BYTES = NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES};

enum {NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS = NVBOOT_RSA_2048_MODULUS_SIZE_WORDS};

enum {NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS = NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS};

enum {NVBOOT_RSA_MAX_MODULUS_SIZE_BITS = NVBOOT_RSA_2048_MODULUS_SIZE_WORDS * 8 * 4};

enum {NVBOOT_RSA_MAX_EXPONENT_SIZE_BITS = NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS * 8 * 4};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SIZE_BYTES = NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SIZE_WORDS = NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_HASH_ALGORITHM = SE_MODE_PKT_SHAMODE_SHA256};

enum {NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SLEN = ARSE_SHA256_HASH_SIZE / 8};

enum {NVBOOT_RSA_DEFAULT_PUBLIC_EXPONENT = 0x10001};

/**
 * Defines the length of the RSASSA-PSS salt length (sLen) to use for RSASSA-PSS
 *  signature verifications
 */
enum {NVBOOT_RSA_PSS_SALT_LENGTH_BITS = ARSE_SHA256_HASH_SIZE};
enum {NVBOOT_RSA_PSS_SALT_LENGTH_BYTES = NVBOOT_RSA_PSS_SALT_LENGTH_BITS / 8};


/*
 *  Defines the storage for an 2048-bit RSA key to be used by the SE.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKey2048NvU8Rec
{
    // The modulus size is 2048-bits.
    NvU8 Modulus[NVBOOT_RSA_2048_MODULUS_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                       // NvBootCryptoRsaKeyData is used to access this field
    // The exponent size is 2048-bits.
    NvU8 Exponent[NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                       // NvBootCryptoRsaKeyData is used to access this field
} NvBootCryptoRsaKey2048NvU8;

/*
 *  Defines the storage for an 2048-bit RSA key to be used by the BR.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKey2048NvU32Rec
{
    // The modulus size is 2048-bits.
    NvU32 Modulus[NVBOOT_RSA_2048_MODULUS_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
    // The exponent size is 2048-bits.
    NvU32 Exponent[NVBOOT_RSA_2048_EXPONENT_SIZE_WORDS];  // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
} NvBootCryptoRsaKey2048NvU32;

/*
 *  Defines the storage for an 3072-bit RSA key.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKey3072NvU8Rec
{
    // The modulus size is 3072-bits.
    NvU8 Modulus[NVBOOT_RSA_3072_MODULUS_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                       // NvBootCryptoRsaKeyData is used to access this field
    // The exponent size is 3027-bits.
    NvU8 Exponent[NVBOOT_RSA_3072_EXPONENT_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                         // NvBootCryptoRsaKeyData is used to access this field
} NvBootCryptoRsaKey3072NvU8;

/*
 *  Defines the storage for an 3072-bit RSA key.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKey3072NvU32Rec
{
    // The modulus size is 3072-bits.
    NvU32 Modulus[NVBOOT_RSA_3072_MODULUS_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
    // The exponent size is 3072-bits.
    NvU32 Exponent[NVBOOT_RSA_3072_EXPONENT_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
} NvBootCryptoRsaKey3072NvU32;

/*
 *  Defines the storage for the largest RSA key to be used by the BR. For T210/T214/T186,
 *  this is 2048-bits. For T19x, this is 3072-bits.
 *  Note: The SE can only use keys of 2048-bits. The PKA can accept 3072-bit keys.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeNvU8Rec
{
    // The max modulus size depends on the chip.
    NvU8 Modulus[NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                       // NvBootCryptoRsaKeyData is used to access this field
    // The max exponent size depends on the chip.
    NvU8 Exponent[NVBOOT_RSA_MAX_EXPONENT_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                         // NvBootCryptoRsaKeyData is used to access this field
} NvBootCryptoRsaKeyMaxSizeNvU8;

/*
 *  Defines the storage for the largest RSA key to be used by the BR. For T210/T214/T186,
 *  this is 2048-bits. For T19x, this is 3072-bits.
 *  Note: The SE can only use keys of 2048-bits. The PKA can accept 3072-bit keys.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeNvU32Rec
{
    // The max modulus size depends on the chip.
    NvU32 Modulus[NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
    // The max exponent size depends on the chip.
    NvU32 Exponent[NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                       //__attribute__((aligned(4))) not needed;
} NvBootCryptoRsaKeyMaxSizeNvU32;

/*
 *  Defines the storage for the largest RSA key that can be used by the SE
 *  RSA engine, currently 2048-bits.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeSe0NvU32Rec
{
    // The modulus size is 2048-bits.
    NvU32 Modulus[NVBOOT_SE_RSA_MAX_MODULUS_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                        //__attribute__((aligned(4))) not needed;
    // The exponent size is 2048-bits.
    NvU32 Exponent[NVBOOT_SE_RSA_MAX_EXPONENT_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                          //__attribute__((aligned(4))) not needed;
} NvBootCryptoRsaKeyMaxSizeSe0NvU32;

/*
 *  Defines the storage for the largest RSA key that can be used by the SE
 *  RSA engine, currently 2048-bits. Byte addressable struct.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeSe0NvU8Rec
{
    // The modulus size is 2048-bits.
    NvU8 Modulus[NVBOOT_SE_RSA_MAX_MODULUS_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                       // NvBootCryptoRsaKeyData is used to access this field
    // The exponent size is 2048-bits.
    NvU8 Exponent[NVBOOT_SE_RSA_MAX_EXPONENT_SIZE_BYTES]; // __attribute__((aligned(4))) not needed if 
                                                         // NvBootCryptoRsaKeyData is used to access this field
} NvBootCryptoRsaKeyMaxSizeSe0NvU8;


/*
 *  Defines the storage for the largest RSA key that can be used by the PKA1
 *  RSA engine, currently 3072-bits.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizePka1NvU32Rec
{
    // The modulus size is 3072-bits.
    NvU32 Modulus[NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
    // The exponent size is 3072-bits.
    NvU32 Exponent[NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                       // __attribute__((aligned(4))) not needed;
} NvBootCryptoRsaKeyMaxSizePka1NvU32;


/*
 *  Defines the storage for the largest RSA key that can be used
 *  by the BR, irrespective of HW engine. Currently 3072-bits.
 *  The SE expects Key inputs to be word aligned.
 */
typedef struct NvBootCryptoRsaKeyMaxSizeBRNvU32Rec
{
    // The modulus size is 3072-bits.
    NvU32 Modulus[NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                     //__attribute__((aligned(4))) not needed;
    // The exponent size is 3072-bits.
    NvU32 Exponent[NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS]; // implicitly 4 byte aligned 
                                                       //__attribute__((aligned(4))) not needed;
} NvBootCryptoRsaKeyMaxSizeBRNvU32;


typedef union NvBootCryptoRsaKeyDataRec
{
    NvBootCryptoRsaKey2048NvU8 RsaKey2048NvU8;
    NvBootCryptoRsaKey2048NvU32 RsaKey2048NvU32;
    NvBootCryptoRsaKeyMaxSizeNvU8 RsaKeyMaxSizeNvU8;
    NvBootCryptoRsaKeyMaxSizeNvU32 RsaKeyMaxSizeNvU32;
    NvBootCryptoRsaKeyMaxSizeSe0NvU8 RsaKeyMaxSizeSe0NvU8; // 2048-bits
    NvBootCryptoRsaKeyMaxSizeSe0NvU32 RsaKeyMaxSizeSe0NvU32; // NvBootCryptoRsaKeyMaxSizeSe0NvU32 
                                                             // consists of an NvU32 array for modulus
                                                             // and exponent. This should force 4 byte alignment.
} NvBootCryptoRsaKeyData;

typedef struct NvBootCryptoRsaKeyRec
{
    NvBootCryptoRsaKeySize KeySize; // type int, 32 bit aligned
    // KeySize is 4 bytes, pad this to make the total size of this struct
    // 16 byte aligned.
    NvU32 Reserved_Padding[3];
    NvBootCryptoRsaKeyData KeyData; // 4 byte aligned 
} NvBootCryptoRsaKey;

/**
 *  Defines the storage for a full 2048-bit RSA key pair.
 *  The SE expects Key inputs to be word aligned.
 *  Not used currently
 */
typedef struct NvBootCryptoFullRsa2048KeyPairRec
{
    NvU8 Modulus[NVBOOT_RSA_2048_MODULUS_SIZE_BYTES] ; //__attribute__((aligned(4)));
    NvU8 Private_exponent_d[NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES]; //__attribute__((aligned(4)));
    NvU8 Public_exponent_e[NVBOOT_RSA_2048_EXPONENT_SIZE_BYTES]; // __attribute__((aligned(4)));
} NvBootCryptoFullRsa2048KeyPair;

/**
 * RSA related parameters, not NVIDIA specific.
 */
typedef struct NvBootCryptoRsaPublicParamsRec
{
    // RSA Public Key Modulus and Exponent
    NvBootCryptoRsaKey RsaPublicKey; // 4 byte aligned struct
} NvBootCryptoRsaPublicParams;

typedef struct NvBootRsaParams
{
    NvBootCryptoRsaKeySize RsaKeySize;
    NvBootCryptoRsaPublicParams RsaPublicParams;
} NvBootRsaParams;

typedef struct NvBootCryptoRsaSsaPssSigNvU8Rec
{
    // The length of the signature is the same as the length of the key used
    // in bytes.
    NvU8 RsaSsaPssSig[NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES];
} NvBootCryptoRsaSsaPssSigNvU8;

typedef struct NvBootCryptoRsaSsaPssSigNvU32Rec
{
    // The length of the signature is the same as the length of the key used
    // in bytes.
    NvU32 RsaSsaPssSig[NVBOOT_RSA_MAX_MODULUS_SIZE_WORDS];
} NvBootCryptoRsaSsaPssSigNvU32;

typedef union NvBootCryptoRsaSsaPssSig
{
    NvBootCryptoRsaSsaPssSigNvU8 RsaSsaPssSigNvU8;
    NvBootCryptoRsaSsaPssSigNvU32 RsaSsaPssSigNvU32;
} NvBootCryptoRsaSsaPssSig;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_CRYPTO_RSA_PARAM_H
