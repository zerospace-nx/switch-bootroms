/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * Defines the SDRAM parameter structure.
 *
 * Note that PLLM is used by EMC.
 */

#ifndef INCLUDED_NVBOOT_SDRAM_PARAM_H
#define INCLUDED_NVBOOT_SDRAM_PARAM_H

// Don't integrate these to SW folder
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum
{
    /// Specifies the memory type to be undefined
    NvBootMemoryType_None = 0,

    #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
    /// Specifies the memory type to be DDR SDRAM
    NvBootMemoryType_Ddr,

    /// Specifies the memory type to be LPDDR SDRAM
    NvBootMemoryType_LpDdr,

    /// Specifies the memory type to be DDR2 SDRAM
    NvBootMemoryType_Ddr2,
    #endif

    /// Specifies the memory type to be LPDDR2 SDRAM
    NvBootMemoryType_LpDdr2,

    /// Specifies the memory type to be DDR3 SDRAM
    NvBootMemoryType_Ddr3,

    /// Specifies the memory type to be LPDDR4 SDRAM
    NvBootMemoryType_LpDdr4,

    NvBootMemoryType_Num,
    NvBootMemoryType_Force32 = 0x7FFFFFF
} NvBootMemoryType;


/**
 * Defines the SDRAM parameter structure
 */
#include "nvboot_sdram_param_generated.h"

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SDRAM_PARAM_H */

