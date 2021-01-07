/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_PKA1_PARAM_H
#define INCLUDED_NVBOOT_CRYPTO_PKA1_PARAM_H

#define NVBOOT_PKA1_KEYSLOT_MAX_BIT_SIZE 4096
#define NVBOOT_PKA1_KEYSLOT_MAX_BYTE_SIZE (NVBOOT_PKA1_KEYSLOT_MAX_BIT_SIZE/8)
#define NVBOOT_PKA1_KEYSLOT_MAX_WORD_SIZE (NVBOOT_PKA1_KEYSLOT_MAX_BYTE_SIZE/4)

#define ARSE_PKA1_NUM_KEY_SLOTS (4)

#endif
