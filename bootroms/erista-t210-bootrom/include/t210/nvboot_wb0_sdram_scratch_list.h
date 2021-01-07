/*
 * Copyright (c) 2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_wb0_sdram_scratch_list.h
 *
 * List macro which defines the scratch register <-> NvBootSdramParams mapping.
 */

#ifndef INCLUDED_NVBOOT_WB0_SDRAM_SCRATCH_LIST_H
#define INCLUDED_NVBOOT_WB0_SDRAM_SCRATCH_LIST_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "sdram_scratch_list_generated.h"

#include "sdram_scratch_list_secure_generated.h"

//    _(102, NvU32, CHB_CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_USE_PLLM_UD, EmcClockUsePllMUD)

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_WB0_SDRAM_SCRATCH_LIST_H
