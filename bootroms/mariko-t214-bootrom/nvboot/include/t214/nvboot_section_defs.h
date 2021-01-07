/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_SECTION_DEFS_H
#define INCLUDED_NVBOOT_SECTION_DEFS_H

#if defined(__cplusplus)
extern "C"
{
#endif

#define FT_NONSECURE __attribute__((section(".text.nonsecure")))
#define VT_NONSECURE __attribute__((section(".rodata.nonsecure")))

/* Used by NvBootInfoTable */
#define VT_MAINBIT __attribute__((section(".MainBIT")))

/* Used by NvBootConfigTable*/
#define VT_MAINBCT __attribute__((section(".MainBCT")))


/* Used by IRamExcpHandler */
#define VT_NOZI __attribute__((section(".nozi.data")))//data.nozi

/* Used by Secure exit code */
#define VT_ZIRW __attribute__((section(".data.rw")))

/* Used to locate crypto buffers for crypto engine usage */
#define VT_CRYPTO_BUFFER __attribute__((section(".CryptoBuffer")))

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SECTION_DEFS_H

