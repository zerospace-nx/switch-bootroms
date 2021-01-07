/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */


#include <stdbool.h>
#include "nvboot_error.h"
#include "nvboot_se_sha_dev_int.h"
#include "nvboot_se_int.h"
#include "nvboot_crypto_sha_param.h"
#include "arse.h"

static uint8_t ConvertSeShaDigestSizeToSeFormat(NvBootCryptoShaDigestSize DigestSize)
{
    switch(DigestSize)
    {
        case SHA_256:
            return SE_MODE_PKT_SHAMODE_SHA256;
            break;
        default:
            return SE_MODE_PKT_SHAMODE_SHA256;
            break;
    }
}

NvBootError NvBootSeShaDevInit(NvBootCryptoShaConfig *ShaConfig)
{
    ShaConfig->ShaDigestSize = SHA_256;
    ShaConfig->ShaFamily = SHA2;

    return NvBootError_Success;
}

NvBootCryptoShaFamily NvBootSeShaGetShaFamily(NvBootCryptoShaConfig *ShaConfig)
{
    return ShaConfig->ShaFamily;
}

bool NvBootSeShaSetShaFamily(NvBootCryptoShaConfig *ShaConfig, NvBootCryptoShaFamily ShaFamily)
{
    switch(ShaFamily)
    {
        case SHA2:
            ShaConfig->ShaFamily = SHA2;
            return true;
        default:
            return false;
    }
}

NvBootCryptoShaDigestSize NvBootSeShaGetDigestSize(NvBootCryptoShaConfig *ShaConfig)
{
    return ShaConfig->ShaDigestSize;
}

bool NvBootSeShaSetShaDigestSize(NvBootCryptoShaConfig *ShaConfig, NvBootCryptoShaDigestSize DigestSize)
{
    switch(DigestSize)
    {
        case SHA_256:
            ShaConfig->ShaDigestSize = SHA_256;
            return true;
        default:
            return false;
            break;
    }
}

bool NvBootSeShaIsValidShaFamily(NvBootCryptoShaFamily ShaFamily)
{
    switch(ShaFamily)
    {
        case SHA2:
            return true;
        default:
            return false;
    }
}

bool NvBootSeShaIsValidShaDigestSize(NvBootCryptoShaDigestSize DigestSize)
{
    switch(DigestSize)
    {
        case SHA_256:
            return true;
        default:
            return false;
            break;
    }
}

NvBootError NvBootSeShaDevShaHash(const uint32_t *InputMessage, uint32_t InputMessageLength, uint32_t *Hash, NvBootCryptoShaConfig *ShaConfig)
{
    //if (IsValidSha2Variant(DigestSize) == false)
        //return NvBootError_UnsupportedShaVariant;
    if(NvBootSeShaIsValidShaFamily(ShaConfig->ShaFamily) == false)
        return NvBootError_Unsupported_SHA_Family;

    if(NvBootSeShaIsValidShaDigestSize(ShaConfig->ShaDigestSize) == false)
        return NvBootError_Unsupported_SHA_DigestSize;

    NvBootSeSHAHash((uint32_t *) InputMessage, InputMessageLength, NULL, Hash, ConvertSeShaDigestSizeToSeFormat(ShaConfig->ShaDigestSize));

    return NvBootError_Success;
}

/**
 *  Shutdown the SHA device and clean up state.
 */
void NvBootSeShaDeviceShutdown(void)
{
    return;
}
