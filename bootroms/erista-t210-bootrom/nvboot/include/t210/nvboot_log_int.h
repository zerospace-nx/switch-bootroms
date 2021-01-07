/*
 * Copyright (c) 2012 - 2013 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_log_int.h - Declarations for boot logging  support.
 */

#ifndef INCLUDED_NVBOOT_LOG_INT_H
#define INCLUDED_NVBOOT_LOG_INT_H

#include "nvcommon.h"
#include "nvboot_msg_defs_int.h"

#if 1
#define NVBOOT_MSG(x) NvBootLog_printf x
#else
#define NVBOOT_MSG(x)
#endif

#if defined(__cplusplus)
extern "C"
{
#endif

void NvBootLogInit(void);

NvU32 NvBootLog_printf(int, ...);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_LOG_INT_H */
