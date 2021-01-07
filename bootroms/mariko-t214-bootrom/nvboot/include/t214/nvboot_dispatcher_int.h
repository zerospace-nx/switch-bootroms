/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_DISPATCHER_INT_H
#define INCLUDED_NVBOOT_DISPATCHER_INT_H

#include <nvboot_section_defs.h>
#include <nvboot_error.h>

#if defined(__cplusplus)
extern "C"
{
#endif


/**
 * NvBootTask provides information regarding the functions in Boot ROM (BR)
 * in non secure/secure mode of operation.
 *
 * As the BR works its way through its boot sequence, BR records data in
 * the task entry/exit time, checkpoint (func ID) and task exit status in BIT.
 * This includes information determined as BR boots and a log of the 
 * boot process.
 *
 * The TaskBlock entries serves as a tool for diagnosing boot failures.  
 * The cold boot process is necessarily opaque, and these entries provides a 
 * boot flow reference of the functions invoked upto a certain limit. 
 */
typedef enum {
    // NvBootTaskListId_SecureInit, // Tasks moved to Predispatcher init.
    NvBootTaskListId_CryptoInit,
    NvBootTaskListId_ColdBoot,
    NvBootTaskListId_Rcm,
    NvBootTaskListId_SecureExit,
    NvBootTaskListId_WarmBoot,
    NvBootTaskListId_Force32=0x7FFFFFFF
} NvBootTaskListId;

typedef struct _TaskRec {
    NvBootError (*funcPtr)();
    int checkPoint;
} NvBootTask, * NvBootTaskPtr;

typedef struct _TaskListsRec {
    const NvBootTask * taskList;
    NvU32 numTasks;
} NvBootTaskLists;

typedef struct _dispatchStat {
    int curCheckPoint;
    int nTicks;
} NvBootDispatchStat, * NvBootDispatchStatPtr;
int FT_NONSECURE NvBootNonsecureDispatcher(void);
NvBootError NvBootSecureDispatcher(NvBootTaskListId);
void *memcpy(void *dest, const void *src, int n);
void *memset(void *s, int c, int n);

void NvBootMainSetupR5Cache( void);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_DISPATCHER_INT_H
