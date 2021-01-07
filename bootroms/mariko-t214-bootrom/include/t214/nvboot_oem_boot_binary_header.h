/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDE_T214_NVBOOT_OEM_BOOT_BINARY_HEADER_H_
#define INCLUDE_T214_NVBOOT_OEM_BOOT_BINARY_HEADER_H_

#include "nvboot_crypto_param.h"
#include "nvboot_crypto_sha_param.h"
#include "nvboot_crypto_signatures.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Stores information needed to locate and verify an OEM signed binary, such as a
 * bootloader, MB1, BL1, etc. The name is generic on purpose, for usage by
 * multiple types of OEM binaries.
 * There is one \c NvBootOemBootBinaryHeader structure for each copy of an OEM signed binary stored on
 * the device.
 *
 * This header structure is shared between chips that have MB1 and BL1.
 * Boot binary can mean different things in the context of which chip this is used for.
 * T214 - The first boot binary loaded by BR is the BL1, owned/signed by the OEM.
 * T194 - The first boot binary loaded by BR is Mb1, NV owned/signed.
 *
 */
typedef struct NvBootOemBootBinaryHeaderRec
{
    /// All cryptographic signatures supported will be stored here. The BL can be
    /// simultaneously signed by all cryptographic signature types.
    NvBootCryptoSignatures OemSignatures;

    /// Signed/hashed section starts here.
    /// Salt, i.e. Random Block of data.
    NvU32   Salt[8];

    NvBootSha256HashDigest OemBootBinaryHash;

    /// Specifies a version number for the boot binary. The assignment of numbers is
    /// arbitrary; the numbers are only used to identify redundant copies
    /// (which have the same version number) and to distinguish between
    /// different versions of the BL (which have different numbers).
    NvU32   Version;

    /// Specifies the length of the boot binary in bytes. Boot binaries must be padded
    /// to an integral number of 16 bytes with the padding pattern.
    /// @note The end of the binary cannot fall within the last 16 bytes of
    /// a page. Add another 16 bytes to work around this restriction if
    /// needed.
    /// This length is the length of boot binary only, without this header.
    NvU32   Length;

    /// Specifies the starting address of the memory region into which the
    /// BL will be loaded.
    /// NOTE: LoadAddress is only used by chips that do not have MB1, i.e.
    ///       T210, T214. In T194, The LoadAddress, the EntryPoint is specified
    ///       in the NV header.
    NvU32   LoadAddress;

    /// Specifies the entry point address in the loaded BL image.
    /// NOTE: LoadAddress is only used by chips that do not have MB1, i.e.
    ///       T210, T214. In T194, The LoadAddress, the EntryPoint is specified
    ///       in the NV header.
    NvU32   EntryPoint;

    /// Reserved field. Pad by 12 bytes to make the signed section aligned to
    /// AES block size
    NvU8 Reserved[16];
} NvBootOemBootBinaryHeader;

#if defined(__cplusplus)
extern "C"
{
#endif

#endif /* INCLUDE_T214_NVBOOT_OEM_BOOT_BINARY_HEADER_H_ */
