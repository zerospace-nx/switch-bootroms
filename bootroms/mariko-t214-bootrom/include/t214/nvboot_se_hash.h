/*
 * Copyright (c) 2006 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_se_hash.h
 *
 * Defines the parameters and data structure for SE's SHA engine
 *
 */

#ifndef INCLUDED_NVBOOT_SE_HASH_H
#define INCLUDED_NVBOOT_SE_HASH_H

#include "nvcommon.h"
#include "arse.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the length of a SHA256 hash in bytes
 *
 */
enum {NVBOOT_SE_SHA256_LENGTH_BYTES = ARSE_SHA256_HASH_SIZE / 8};

/**
 * Defines the length of a SHA256 hash in words
 *
 */
enum {NVBOOT_SE_SHA256_LENGTH_WORDS = NVBOOT_SE_SHA256_LENGTH_BYTES / 4};

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SE_HASH_H
