/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_ECC_H
#define INCLUDED_NVBOOT_CRYPTO_ECC_H

#include "nvboot_util_int.h"

//enum {NVBOOT_SECP256R1_KEY_SIZE_BYTES = NV_ICEIL(256, 8)};
//enum {NVBOOT_SECP521R1_KEY_SIZE_BYTES = NV_ICEIL(521, 8)};

//enum {NVBOOT_ECC_PRIME_FIELD_DEFAULT_KEY_SIZE_BYTES = NVBOOT_SECP256R1_KEY_SIZE_BYTES};
//enum {NVBOOT_ECC_PRIME_FIELD_MAX_KEY_SIZE_BYTES = NVBOOT_SECP521R1_KEY_SIZE_BYTES};

/**
 * The Boot ROM need only support the NIST P-256 EC curve.
 * No user specified curves required.
 * See: http://nvbugs/1630454/2
 */
typedef enum
{
    EccCurve_Nist_P256,

    EccCurve_Num,
    EccCurve_Default = EccCurve_Nist_P256,

} NvBootEccEllipticCurves;

/**
 * P-192 is not defined because it is unlikely it will ever be used.
 */
typedef enum
{
    EccPrimeFieldKeyBits192 = 192,
    EccPrimeFieldKeyBits224 = 224,
    EccPrimeFieldKeyBits256 = 256,
    EccPrimeFieldKeyBits384 = 384,
    EccPrimeFieldKeyBits521 = 521,

    EccPrimeFieldKeyDefaultKeySizeBits = EccPrimeFieldKeyBits256,
} NvBootEccPrimeFieldKeySizeBits;

/**
 * IMPORTANT NOTE. An valid EC "Key" is a point on the curve. Each point is
 * specified by two integers x and y in the interval [0, p-1] for prime field
 * curves. Each integer x and y can be up to the key size in bytes.
 * For example, a 256-bit "key" Q = (x, y) is specified in 512-bits.
 *
 */
typedef enum
{
    EccPrimeFieldKeySizeBytes192 =  NV_ICEIL(EccPrimeFieldKeyBits192, 8),
    EccPrimeFieldKeySizeBytes224 =  NV_ICEIL(EccPrimeFieldKeyBits224, 8),
    EccPrimeFieldKeySizeBytes256 =  NV_ICEIL(EccPrimeFieldKeyBits256, 8),
    EccPrimeFieldKeySizeBytes384 =  NV_ICEIL(EccPrimeFieldKeyBits384, 8),
    EccPrimeFieldKeySizeBytes521 =  NV_ICEIL(EccPrimeFieldKeyBits521, 8),

    EccPrimeFieldKeyDefaultKeySizeBytes = EccPrimeFieldKeySizeBytes256,
    EccPrimeFieldKeyMaxKeySizeBytes = EccPrimeFieldKeySizeBytes384,
} NvBootEccPrimeFieldKeySizeBytes;

typedef struct NvBootEcPoint256Rec
{
    uint8_t x[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    uint8_t y[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEcPoint256 __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

typedef struct NvBootEcPoint384Rec
{
    uint8_t x[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    uint8_t y[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEcPoint384 __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

typedef struct NvBootEcPoint521Rec
{
    uint8_t x[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    uint8_t y[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEcPoint521 __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

typedef struct NvBootEcPointMaxRec
{
    uint8_t x[EccPrimeFieldKeyMaxKeySizeBytes] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    uint8_t y[EccPrimeFieldKeyMaxKeySizeBytes] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEcPointMax __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));


/**
 * \brief           A struct to store the x and y coordinate of a point on the
 *                  elliptic curve over GF(p).
 *
 */
typedef union NvBootEcPointRec
{
    NvBootEcPoint256 EcPoint256;
    NvBootEcPoint384 EcPoint384;
    NvBootEcPointMax EcPointMax;
} NvBootEcPoint;

/**
typedef struct NvBootEccPrimeFieldParamsRec
{

    uint8_t *p;
    uint8_t *n;
    uint8_t *SEED;
    uint8_t *a; // NIST curves have a=-3
    uint8_t *b;
    NvBootEcPoint *G;
    uint8_t *h;
} NvBootEccPrimeFieldParams;
*/

/**
 * \brief           Parameters for P-521 NIST elliptic Curve over Fp
 *
 * \note            Let p > 3 be an odd prime. An elliptic curve E
 *                  over Fp is defined by an equation of the form
 *                  y^2 = x^3 + ax +b.
 *
 *                  For efficiency reasons, a = -3 is always used (see
 *                  IEEE Std 1363-2000), so it is not a configurable
 *                  parameter in this struct.
 *
 * \note            See fips-4 table D-1 on the bit length of n.
 *                  Note that the bit length of n can be 256 to 383 bits
 *                  for a Prime field p = 256.
 *                  The bit length of prime field p = 521 is >=512.
 *                  Therefore, let's declare each parameter with a size
 *                  of Ceiling(521/8).
 */
typedef struct NvBootEccPrimeFieldParamsP256Rec
{
    /*!< p = Prime modulus of the base field */
    uint8_t p[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Coefficient a                   */
    uint8_t a[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Coefficient b                   */
    uint8_t b[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Order n of the curve            */
    uint8_t n[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  X-coordinate of the base point on the curve */
    uint8_t Gx[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Y-coordinate of the base point on the curve */
    uint8_t Gy[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEccPrimeFieldParamsP256 __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

/**
 * \brief           Parameters for P-384 NIST elliptic Curve over Fp
 *
 * \note            Let p > 3 be an odd prime. An elliptic curve E
 *                  over Fp is defined by an equation of the form
 *                  y^2 = x^3 + ax +b.
 *
 *                  For efficiency reasons, a = -3 is always used (see
 *                  IEEE Std 1363-2000), so it is not a configurable
 *                  parameter in this struct.
 *
 * \note            See fips-4 table D-1 on the bit length of n.
 *                  Note that the bit length of n can be 256 to 383 bits
 *                  for a Prime field p = 256.
 *                  The bit length of prime field p = 521 is >=512.
 *                  Therefore, let's declare each parameter with a size
 *                  of Ceiling(521/8).
 */
typedef struct NvBootEccPrimeFieldParamsP384Rec
{
    /*!< p = Prime modulus of the base field */
    uint8_t p[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Coefficient a                   */
    uint8_t a[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Coefficient b                   */
    uint8_t b[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Order n of the curve            */
    uint8_t n[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  X-coordinate of the base point on the curve */
    uint8_t Gx[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Y-coordinate of the base point on the curve */
    uint8_t Gy[EccPrimeFieldKeySizeBytes384] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEccPrimeFieldParamsP384 __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

/**
 * \brief           Parameters for P-521 NIST elliptic Curve over Fp
 *
 * \note            Let p > 3 be an odd prime. An elliptic curve E
 *                  over Fp is defined by an equation of the form
 *                  y^2 = x^3 + ax +b.
 *
 *                  For efficiency reasons, a = -3 is always used (see
 *                  IEEE Std 1363-2000), so it is not a configurable
 *                  parameter in this struct.
 *
 * \note            See fips-4 table D-1 on the bit length of n.
 *                  Note that the bit length of n can be 256 to 383 bits
 *                  for a Prime field p = 256.
 *                  The bit length of prime field p = 521 is >=512.
 *                  Therefore, let's declare each parameter with a size
 *                  of Ceiling(521/8).
 */
typedef struct NvBootEccPrimeFieldParamsP521Rec
{
    /*!< p = Prime modulus of the base field */
    uint8_t p[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Coefficient a                   */
    uint8_t a[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Coefficient b                   */
    uint8_t b[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Order n of the curve            */
    uint8_t n[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  X-coordinate of the base point on the curve */
    uint8_t Gx[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    /*!<  Y-coordinate of the base point on the curve */
    uint8_t Gy[EccPrimeFieldKeySizeBytes521] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEccPrimeFieldParamsP521 __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

typedef union NvBootEccPrimeFieldParamsRec
{
    NvBootEccPrimeFieldParamsP256 EccPrimeFieldParamsP256;
    NvBootEccPrimeFieldParamsP384 EccPrimeFieldParamsP384;
} NvBootEccPrimeFieldParams __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));


typedef struct NvBootEccCalcBufferP256Rec
{
    // w = (s')^-1 nod n
    uint8_t w[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    // u1 = (e' * w) mod n
    uint8_t u1[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    // u2 = (r' * w) mod n
    uint8_t u2[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    // R = (Xr, Yr) = u1 * G + u2 * Q
    NvBootEcPoint256 R __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    // v = Xr mod n
    uint8_t v[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
    // e', the modular reduced version of the hash of the input message.
    uint8_t e_prime[EccPrimeFieldKeySizeBytes256] __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));
} NvBootEcdsaCalcBufferP256;

typedef union NvBootEccCalcBufferRec
{
    NvBootEcdsaCalcBufferP256 EcdsaCalcBufferP256;
} NvBootEcdsaCalcBuffer;

typedef struct NvBootEccPublicParamsRec
{
    NvBootEcPoint EccPublicKey;
    NvBootEccPrimeFieldParams EccPrimeFieldParams;
} NvBootEccPublicParams;


typedef struct NvBootEccParamsRec
{
    NvBootEccPublicParams EccPublicParams;
} NvBootEccParams;

// ECDSA Signature S = (r,s), where r and s are integers in the interval of
// [1, n-1], specified as bytes/octets.
typedef struct NvBootEcdsaSigRec
{
    uint8_t r[EccPrimeFieldKeyMaxKeySizeBytes];
    uint8_t s[EccPrimeFieldKeyMaxKeySizeBytes];
} NvBootEcdsaSig;

/**
 * \brief   NIST P-256 curve parameters (aka secp256r1).
 *
 */
static const NvBootEccPrimeFieldParamsP256 EccPrimeFieldParamsP256_NIST_P256 =
{
    .p =
    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    },
    .a =
    {
        0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    },
    .n =
    {
        0x51, 0x25, 0x63, 0xFC, 0xC2, 0xCA, 0xB9, 0xF3,
        0x84, 0x9E, 0x17, 0xA7, 0xAD, 0xFA, 0xE6, 0xBC,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    },
    .Gx =
    {
        0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4,
        0xA0, 0x33, 0xEB, 0x2D, 0x81, 0x7D, 0x03, 0x77,
        0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8,
        0x47, 0x42, 0x2C, 0xE1, 0xF2, 0xD1, 0x17, 0x6B,
    },
    .Gy =
    {
        0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB,
        0xCE, 0x5E, 0x31, 0x6B, 0x57, 0x33, 0xCE, 0x2B,
        0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E,
        0x9B, 0x7F, 0x1A, 0xFE, 0xE2, 0x42, 0xE3, 0x4F,
    },
};
#endif
