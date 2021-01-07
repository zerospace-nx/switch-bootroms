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
 * Defines version information for the boot rom.
 */

#ifndef INCLUDED_NVBOOT_VERSION_ROM_H
#define INCLUDED_NVBOOT_VERSION_ROM_H

#include "nvboot_version_defs.h"

#define NVBOOT_BOOTROM_VERSION	NvBootBootromVersionAddress[0]
#define NVBOOT_RCM_VERSION	NvBootBootromVersionAddress[1]
#define NVBOOT_BOOTDATA_VERSION NvBootBootromVersionAddress[2]

#if defined(__cplusplus)
extern "C"
{
#endif

extern NvU32 NvBootBootromVersionAddress[3];    

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_VERSION_ROM_H */
