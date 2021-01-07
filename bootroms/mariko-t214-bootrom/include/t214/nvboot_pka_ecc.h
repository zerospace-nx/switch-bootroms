/*
 * Copyright (c) 2006 - 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_pka_ecc.h
 *
 * Defines the parameters and data structure for SE's RSA engine
 *
 */

#ifndef INCLUDED_NVBOOT_PKA_ECC_H
#define INCLUDED_NVBOOT_PKA_ECC_H

#include "nvtypes.h"
#include "nvboot_config.h"
#include "nvboot_hash.h"
#include "nvboot_se_defs.h"

#if defined(__cplusplus)
extern "C"
{
#endif


/**
 * Defines PKA ECC Key Slots
 */
typedef enum
{
    NvBootPkaEccKeySlot_1 = 0UL,
    NvBootPkaEccKeySlot_2,
    NvBootPkaEccKeySlot_3,
    NvBootPkaEccKeySlot_4,

    // Specifies max number of SE RSA Key Slots
    NvBootPkaEccKeySlot_Num,

    NvBootPkaEccKeySlot_EK = NvBootPkaEccKeySlot_4,
} NvBootPkaEccKeySlot;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_PKA_ECC_H
