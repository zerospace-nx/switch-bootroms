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
 * nvboot_buffers_int.h - Definition of the buffers used for reading from boot
 * devides.
 */

#ifndef INCLUDED_NVBOOT_BUFFERS_INT_H
#define INCLUDED_NVBOOT_BUFFERS_INT_H

#include "nvtypes.h"
#include "nvboot_config_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * Definition of the actual buffer memory. The memory is declared in
 * nvboot_bootloader.c.
 */
extern NvU8 BufferMemory[NVBOOT_MAX_BUFFER_SIZE];

/* Pointers to a pair of 4k aligned buffers */
extern NvU8* Buffer[2];

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BUFFERS_INT_H */
