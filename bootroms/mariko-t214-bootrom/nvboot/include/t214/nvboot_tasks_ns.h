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
 * @file nvboot_tasks_ns.h
 *
 * Defines the non secure function master task and data structures 
 * related task logging.
 */

#ifndef INCLUDED_NVBOOT_TASKS_NS_H
#define INCLUDED_NVBOOT_TASKS_NS_H

#include "nvtypes.h"
#if defined(__cplusplus)
extern "C"
{
#endif

#include <nvboot_section_defs.h>

/* Export */
void *GetPtrTasksNS(void);
int GetCntTasksNS(void);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_TASKS_NS_H

