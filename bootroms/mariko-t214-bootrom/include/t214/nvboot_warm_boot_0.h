/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * Defines the warm boot 0 information for the boot rom.
 */

#ifndef INCLUDED_NVBOOT_WARM_BOOT_0_H
#define INCLUDED_NVBOOT_WARM_BOOT_0_H

#include "nvboot_config.h"
#include "nvboot_hash.h"
#include "nvboot_se_rsa.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the recovery code header information for the boot rom.
 *
 * The recovery code immediately follows the recovery code header.
 *
 * Note that the recovery code header needs to be 16 bytes aligned to preserve
 * the alignment of relevant data for hash and decryption computations without
 * requiring extra copies to temporary memory areas.
 */
typedef struct NvBootWb0RecoveryHeaderRec
{
    /// Specifies the length of the recovery code header
    NvU32      LengthInsecure;

    /// Specifies the reserved words to maintain alignment
    NvU32      Reserved[3];

    /// The Pcp field will house public, non-secret cryptographic parameters necessary
    /// for the authentication of the SC7 firmware. These parameters are
    /// collectively known as Public Cryptographic Parameters (PCP) and they will
    /// be stored in the unsigned section of the BCT.
    /// The BR will check the validity of these parameters by calculating the SHA256
    /// hash of the Pcp and compare against the value burned in fuses.
    NvBootPublicCryptoParameters Pcp;

    /// All cryptographic signatures supported will be stored here. The BCT can be
    /// simultaneously signed by all cryptographic signature types.
    NvBootCryptoSignatures Signatures;

    /// Specifies the random block of data which is not validated but
    /// aids security.
    NvU32   RandomAesBlock[NVBOOT_AES_BLOCK_LENGTH_WORDS];

    /// Specifies the length of the recovery code header
    NvU32      LengthSecure;

    /// NOTE: This field is deprecated.
    /// Specifies the starting address of the recovery code in the
    /// destination area.
    NvU32      Destination;

    /// NOTE: This field is deprecated.
    /// Specifies the entry point of the recovery code in the destination area.
    NvU32      EntryPoint;

    /// Specifies the length of the recovery code
    NvU32      RecoveryCodeLength;
} NvBootWb0RecoveryHeader;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_WARM_BOOT_0_H */
