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
 * Definition of crypto parameter structures grouped by algorithm, HW engine,
 * and NVIDIA specific parameters.
 */
#ifndef INCLUDED_NVBOOT_CRYPTO_PARAM_H
#define INCLUDED_NVBOOT_CRYPTO_PARAM_H

#include "nvboot_crypto_se_param.h"
#include "nvboot_crypto_aes_param.h"
#include "nvboot_crypto_rsa_param.h"
#include "nvboot_crypto_sha_param.h"
#include "nvboot_crypto_signatures.h"

typedef enum
{
    // Random 32-bit values instead of regular enum increment,
    // as a fault injection countermeasure.
    CryptoAlgo_None = 0x727A96B2,
    CryptoAlgo_AES = 0x72C638C0,
    CryptoAlgo_AES_CMAC = 0x7552D691,
    CryptoAlgo_RSA = 0x728543CF,
    CryptoAlgo_RSA_RSASSA_PSS = 0x7AFF7327,
    CryptoAlgo_SHA2= 0x78082AB1,
    CryptoAlgo_FSKP_CMAC = 0x72D4A935,

    CryptoAlgo_Num = 0x7,
    CryptoAlgo_Force32 = 0x7fffffff,

} NvBootCryptoAlgo;

/**
 * The list of possible HW/SW crypto calculation engines. 
 *
 * - See Figure 1 of SE IAS. for top level block diagram.
 * - SE0 is the APB access port for BR use. Other access ports like SE1-4
 * are Host1x ports.
 * - There are multiple engines in what we call the SE.
 * - AES0, AES1 are HW accelerated AES engines.
 * - PKA0 is the original RSA accelerator introducted in T114.
 * - PKA1 is the Elliptic public key accelerator BR will use for ECC operations.
 *
 */
typedef enum
{
    // Random 32-bit values instead of regular enum increment,
    // as a fault injection countermeasure.
    CryptoEngine_Se0_Aes0 = 0x7F551BD2, // NV AES engine in the SE
    CryptoEngine_Se0_Pka0 = 0x7BD843A1, // NV RSA engine from T114
    CryptoEngine_Se0_Pka1 = 0x7269DC13, // Elliptic PKA engine
    CryptoEngine_Se0_Sha2 = 0x75A13E97, // NV SE SHA2 engine
    CryptoEngine_Sw_AES_Engine = 0x7D729BCE, // SW implementation of AES inside BR

    CryptoEngine_Num = 0x5,
    CryptoEngine_Force32 = 0x7fffffff,
} NvBootCryptoEngine;

typedef enum
{
    // OEM key is the RSA or ECC public key in the BCT.
    Verify_w_OEM_Key,
    // NV key is the NVIDIA ECC public key.
    Verify_w_NV_Key,

    Verify_Force32 = 0x7fffffff,
} NvBootVerifyOp;

typedef enum
{
    // OEM key is the SBK.
    Decrypt_w_OEM_Key,
    // NV key is the MB1 decryption key.
    Decrypt_w_NV_Key,

    Decrypt_w_Force32 = 0x7fffffff,
} NvBootDecryptOp;

typedef union
{
    NvBootAesParams AesParams;
    NvBootRsaParams RsaParams;
} NvBootCryptoParams;

/**
 *
 * This struct will house all of the public, non-secret parameters
 * required for asymmmetric-key based authentication schemes.
 *
 * Notes:
 *
 * - Storage is allocated for an RSA exponent, but not used by the
 * Boot ROM. The Boot ROM will always use a RSA public exponent of 0x10001.
 * - Storage is allocated for 384-bit sized Prime field curve parameters. However,
 * user specified curves are not supported by Boot ROM for T186. Only the NIST P-256
 * curve is supported.
 * - The space reserved for the above parameters is for future usage should
 * the Boot ROM POR be changed in subsequent Tegra generations.
 *
 */
typedef struct
{
    NvBootCryptoRsaPublicParams RsaPublicParams; // This should be aligned to 4 bytes.
    /*NvBootEccPublicParams EccPublicParams __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));*/ // No ECDSA for this chip.
} NvBootPublicCryptoParameters;

typedef union
{
    NvBootCryptoSeAes0AesParams SeAes0AesParams;
    NvBootCryptoSePka0RsaParams SePka0RsaParams;
    NvBootCryptoSePka1EccParams SePka1EccParams;
} NvBootNvCryptoEngineParams;

typedef struct
{
    NvBool UseSbk;
    NvBootNvCryptoEngineParams CryptoEngineParams;
} NvBootCryptoNvParams;
#endif


