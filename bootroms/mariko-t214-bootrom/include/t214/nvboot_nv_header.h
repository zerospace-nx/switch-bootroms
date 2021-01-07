/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_NV_HEADER_H
#define INCLUDED_NVBOOT_NV_HEADER_H

#include "nvboot_crypto_param.h"

#define RESERVED_SIZE 4

enum SigningType {
    SigningType_None,
    SigningType_OEM_RSA,
    SigningType_OEM_ECC,
    SigningType_NVIDIA_RSA,
    SigningType_NVIDIA_ECC, //this entry is never valid
    SigningType_Force = 0xFFFFFFFFU,
};

typedef struct NvBinarySigHeaderRec
{
    /***** Unsigned section starts (384, 0x180 bytes)*****/
    uint8_t HeaderMagic[4]; // 4 bytes
    NvU32 HeaderVersion; // 4 bytes, considering flexibility for future extension
    NvBootCryptoSignatures Signatures; // 16+256+96 bytes
    uint8_t Padding[8]; // 8 bytes

    /***** Signed section starts (16, 0x10 byte)*****/
    uint8_t BinaryMagic[4]; // 4 bytes, unique identifier for binary
    enum SigningType SignType; // 4 bytes
    NvU32 BinaryLength; // 4 bytes, in bytes
    uint8_t Reserved[RESERVED_SIZE] ; // 4 bytes, binary-specific structure/header   RESERVED_SIZE is 4
} NvBinarySignHeader; //400, 0x190 bytes

#endif // INCLUDED_NVBOOT_NV_HEADER_H
