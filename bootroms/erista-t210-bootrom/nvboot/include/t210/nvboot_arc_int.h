/*
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * nvboot_arc_int.h - Public definitions for using arc path to 
 * enable/disable access to Internal (IRAM) memory region.
 */

#ifndef INCLUDED_NVBOOT_ARC_INT_H
#define INCLUDED_NVBOOT_ARC_INT_H

#include "nvboot_error.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Enable memory region access
 * @retval None
 */
void NvBootArcEnable(void);

/**
 * Disable memory region access
 * @retval None
 */
void NvBootArcDisable(NvU32);


#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_ARC_INT_H */

