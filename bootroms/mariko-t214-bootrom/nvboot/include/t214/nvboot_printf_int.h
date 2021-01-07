/*
 * Copyright (c) 2006 - 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_printf_int.h
 *
 * printf yo
 *
 */

#ifndef INCLUDED_NVBOOT_PRINTF_H
#define INCLUDED_NVBOOT_PRINTF_H

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#define MAX_CONSOLE_BUFFER_LEN 0x200

uint32_t
NvBootPrintf(const char *fmt, ...);

uint32_t
NvBoot_printf(uint8_t* buffer, size_t size, const char* format, va_list ap);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_PRINTF_H
