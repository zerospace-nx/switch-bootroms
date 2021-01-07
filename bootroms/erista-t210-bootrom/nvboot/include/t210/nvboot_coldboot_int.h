/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_coldboot_int.h - Declarations for Cold Boot procedures.
 */

#ifndef INCLUDED_NVBOOT_COLD_BOOT_INT_H
#define INCLUDED_NVBOOT_COLD_BOOT_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"

#if defined(__cplusplus)
extern "C"
{
#endif

NvBootError NvBootColdBoot(NvU32 *BootRomExitTarget);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_COLD_BOOT_INT_H */
