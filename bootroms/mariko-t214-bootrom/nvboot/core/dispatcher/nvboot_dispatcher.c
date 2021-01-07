/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <nvboot_bit.h>
#include <nvboot_section_defs.h>
#include <nvboot_tasks_ns.h>
#include <nvboot_tasks_s.h>
#include <nvboot_dispatcher_int.h>
#include "nvtypes.h"
#include "nvboot_devmgr_int.h"
#include "nvboot_util_int.h"
#include "nvboot_rng_int.h"
#include "nvboot_bpmp_int.h"


void FT_NONSECURE UpdateBootFlowTracker(NvU32 Init, NvU32 Exit, NvU32 Id, NvU32 Status);


extern NvBootInfoTable  BootInfoTable;

int FT_NONSECURE NvBootNonsecureDispatcher()
{
    int i = 0;
    NvBootTaskPtr TaskNSPtr = GetPtrTasksNS();
    volatile __attribute__((unused)) NvBootDispatchStat stat;
    int cnt = GetCntTasksNS();
    NvBootError (*func)(void);
    unsigned long funcStartTick;

    BootInfoTable.BootROMtracker = NvBootFlowStatus_NonSecureDispatcherEntry;

    //setup BootInfoTable log

    for (i=0; i<cnt; i++)
    {
        func = TaskNSPtr[i].funcPtr;
        stat.curCheckPoint = TaskNSPtr[i].checkPoint;
        funcStartTick = NvBootUtilGetTimeUS();
        (*func)();
        stat.nTicks = NvBootUtilGetTimeUS() - funcStartTick;
    }

    BootInfoTable.BootROMtracker = NvBootFlowStatus_NonSecureDispatcherExit;

    return 0;
}

NvBootError NvBootSecureDispatcher(NvBootTaskListId TaskListId)
{
    int i = 0;
    NvBootTaskPtr TaskPtr;
    volatile NvBootError Error = NvBootError_NotInitialized;
    volatile NvBootDispatchStat stat;
    int cnt;
    NvBootError (*func)(void);
    unsigned long funcStartTick;
    
    // Sanitize TaskListId
    Error = IsValidTaskListId(TaskListId);
    if(Error != NvBootError_Success)
        return Error;
        
    TaskPtr = GetPtrTasks(TaskListId);
    cnt = GetCntTasks(TaskListId);
    
    for (i=0; i<cnt; i++)
    {
        func = TaskPtr[i].funcPtr;
        stat.curCheckPoint = TaskPtr[i].checkPoint;
        funcStartTick = NvBootUtilGetTimeUS();
        // Introduce a random delay by looping n cycles, n in range 0-1023
        NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

        Error = (*func)();
        // Handle fault detection right away.
        if(Error == NvBootError_Fault_Injection_Detection)
        {
            do_exception();
            do_exception();
            do_exception();
        }
        
        stat.nTicks = NvBootUtilGetTimeUS() - funcStartTick;
        
        UpdateBootFlowTracker(funcStartTick, stat.nTicks, stat.curCheckPoint, (NvU32)(Error));

        if(Error != NvBootError_Success)
            return Error;

        NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

        // Double check the error returned as FI mitigation.
        if(Error != NvBootError_Success)
            return Error;
    }
    
    // Double check that the dispatcher executed all tasks in the table.
    int cnt_verify = GetCntTasks(TaskListId);
    if(i != cnt_verify)
    {
        do_exception();
        do_exception();
        do_exception();
    }

    return Error;
}

void FT_NONSECURE UpdateBootFlowTracker(NvU32 Init, NvU32 Exit, NvU32 Id, NvU32 Status)
{
    static NvU32 NvBootFlowCnt = 0;
    if(NvBootFlowCnt >= NVBOOT_FLOW_LOG_DEPTH)
        NvBootFlowCnt = 0;
    BootInfoTable.BootFlowLog[NvBootFlowCnt].NvBootFlowLogInit = Init;
    BootInfoTable.BootFlowLog[NvBootFlowCnt].NvBootFlowLogExit = Exit;
    BootInfoTable.BootFlowLog[NvBootFlowCnt].NvBootFlowFuncId  = Id;
    BootInfoTable.BootFlowLog[NvBootFlowCnt].NvBootFlowFuncStatus = Status;

    // update couter
    NvBootFlowCnt++;
}

