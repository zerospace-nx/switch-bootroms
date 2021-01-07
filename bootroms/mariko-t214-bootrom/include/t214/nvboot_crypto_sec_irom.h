/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * Definition of crypto parameter structures grouped by algorithm, HW engine,
 * and NVIDIA specific parameters.
 */
#ifndef INCLUDED_NVBOOT_CRYPTO_SEC_IROM_H
#define INCLUDED_NVBOOT_CRYPTO_SEC_IROM_H

#include "nvboot_crypto_param.h"

#define NVBOOT_SEC_IROM_SIZE 1024 * 4
#define NVBOOT_SEC_IROM_OFFSET 1024 * 92
#define NVBOOT_SEC_IROM_START (NV_ADDRESS_MAP_IROM_BASE + NVBOOT_SEC_IROM_OFFSET)
#define NVBOOT_FSPK_COUNT 64
#define NVBOOT_PSK_COUNT 12

//struture in IROM
typedef struct NvBootSecureIROMLayoutRec {
    NvBootAesKey256 FSPK[NVBOOT_FSPK_COUNT];
    NvBootAesKey128 PSK[NVBOOT_PSK_COUNT];
} NvBootSecureIromLayout;


#endif


