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
 * @file nvboot_tasks_s.h
 *
 * Defines the secure function master task and data structures 
 * related task logging.
 */

#ifndef INCLUDED_NVBOOT_TASKS_S_H
#define INCLUDED_NVBOOT_TASKS_S_H

#include "nvtypes.h"
#include "nvboot_dispatcher_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/* Export */
void *GetPtrTasks(NvBootTaskListId TaskListId);
int GetCntTasks(NvBootTaskListId TaskListId);
/**
 *  Check if TaskListId for a task table is valid.
 */
NvBootError IsValidTaskListId(NvBootTaskListId TaskListId);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_TASKS_S_H

