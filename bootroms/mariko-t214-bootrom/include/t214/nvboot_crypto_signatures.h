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
 * \file nvboot_crypto_signature.h
 *
 * Defines the structure to hold the various possible types of
 * cryptographic signatures supported by the Boot ROM.
 *
 */
#ifndef NVBOOT_CRYPTO_SIGNATURE_H
#define NVBOOT_CRYPTO_SIGNATURE_H

/**
 * \brief           Crypto signature structure.
 *
 * \note Force AES block alignment for all members. This matters
 *       where signatures are sometimes contained in signed sections
 *       (like in the case where BootLoader signatures are inside the
 *       BCT). AES-CMAC and RSASSA-PSS signatures are inherently
 *       16-byte aligned so the attribute isn't necessary. However,
 *       making it explicit doesn't hurt.
 */
typedef struct NvBootCryptoSignaturesRec
{
    NvBootAesCmacHash AesCmacHash;
    NvBootCryptoRsaSsaPssSig RsaSsaPssSig;
    /*NvBootEcdsaSig EcdsaSig __attribute__ ((aligned (NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));*/ // No ECDSA for this chip.
} NvBootCryptoSignatures;

#endif
