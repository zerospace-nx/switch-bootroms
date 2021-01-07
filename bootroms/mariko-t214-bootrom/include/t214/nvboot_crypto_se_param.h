/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_crypto_aes_param.h"

/**
 * SE notes.
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

/**
 * NVIDIA specific AES parameters for the AES0 engine in the SE.
 */
typedef struct NvBootCryptoSeAes0AesParamsRec
{
    NvBootAesIv OriginalIv;
    NvBootAesIv UpdatedIv;
    NvU32 KeySlotNum;
} NvBootCryptoSeAes0AesParams;

/**
 * NVIDIA specific RSA parameters for the PKA0 engine in the SE.
 */
typedef struct NvBootCryptoSePka0RsaParamsRec
{
    NvU32 KeySlotNum;
} NvBootCryptoSePka0RsaParams;

/**
 * NVIDIA specific ECC parameters for the PKA1 engine in the SE.
 */
typedef struct NvBootCryptoSePka1EccParamsRec
{
    NvU32 KeySlotNum;
} NvBootCryptoSePka1EccParams;

