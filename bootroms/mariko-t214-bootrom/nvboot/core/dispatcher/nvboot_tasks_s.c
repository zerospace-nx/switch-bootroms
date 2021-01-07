/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <nvboot_section_defs.h>
#include <nvboot_tasks_s.h>
#include <nvboot_dispatcher_int.h>
#include <nvboot_bpmp_int.h>
#include <nvboot_util_int.h>
#include <nvboot_rcm_int.h>
#include "nvboot_irom_patch_int.h"
#include "nvboot_warm_boot_0_int.h"
#include "nvboot_coldboot_int.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_reset_pmu_int.h"
#include "nvboot_se_int.h"
#include "nvboot_ssk_int.h"
#include "nvboot_arc_int.h"

// This should be moved to appropriate headers.
NvBootError NvBootPllpClockSourceSetup();
NvBootError NvBootMemSysRamSetup();

/**
 * NvBootTask master task list for in secure mode of operation.
 *
 * IMPORTANT NOTE: Secure dispatcher expects functions to return
 *                 NvBootError.
*/

/// Tasks moved to PreDispatcher init routines.
// NvBootTask Tasks[] = {
    // { &NvBootCryptoMgrHwEngineInit, 0x53},
    // { &NvBootMainSecureInit, 0x54},
// };

static const NvBootTask CryptoInitTasks[] = {
    { &NvBootCryptoMgrInit, 0x71},
    { &NvBootCryptoMgrDecKeys, 0x72},
    // Note: SSK generated for all boot paths, but in the keyslot holding the SSK
    // SC7 is overwritten by SE context restore.
    { &NvBootSskGenerate, 0x73}
};

static const NvBootTask RCMTasks[] = {
    { &NvBootRCMInit, 0x201 },
    { &NvBootRCMSendUniqueId, 0x202 },
    { &NvBootRCMProcessMsgs, 0x203 },
    // NV FEK in the SE key slot is wiped out by default SE keys generation.
    // Note NV FEK is needed for coldboot FSKP and RCM.
    // OEM FEK is already wiped out after
    // NvBootCryptoMgrDecKeys or else it would also get replaced
    // by this function.
    // In SC7, the NV FEK is wiped by the process of SE context restore.
    { &NvBootCryptoMgrLoadDefaultSEKeys, 0x204},
};

// Tasks related to Coldboot flow
static const NvBootTask ColdBootTasks[] = {
    // Unconditional SE atomic save enablement only in coldboot. SC7 exit has its own
    // special sequencing during SE context restore.
    { &NvBootSeEnableAtomicSeContextSave, 0x101},
    { &NvBootColdBootInit, 		0x102 },
    { &NvBootColdBootReadBct, 	0x103 },
    { &NvBootColdBootSetupSdram, 0x104 },
    { &NvBootColdBootReInit,	0x105 },
    { &NvBootColdBootLoadBl, 	0x106 },
    // NV FEK in the SE key slot is wiped out by default SE keys generation.
    // Note NV FEK is needed for coldboot FSKP and RCM.
    // OEM FEK is already wiped out after
    // NvBootCryptoMgrDecKeys or else it would also get replaced
    // by this function.
    // In SC7, the NV FEK is wiped by the process of SE context restore.
    { &NvBootCryptoMgrLoadDefaultSEKeys, 0x107},
};

// Tasks related to BR Secure Exit flow
static const NvBootTask SecureExitBootTasks[] = {
    { &NvBootSeHousekeepingBeforeBRExit, 0x501},
    { &NvBootLP0ContextRestoreStage3, 0x502},
    { &NvBootBpmpSecureExitStart, 0x503}
};

// Tasks related to Warmboot/SC7 flow
static const NvBootTask WarmBootTasks[] = {
    { &NvBootHaltAtWarmboot, 0x601},
    { &NvBootWb0Start, 0x602},
    { &NvBootWb0TzramInit, 0x603},
    { &NvBootWarmBootUnPackSdramStartPllm, 0x604},
    { &NvBootWarmBootSdramInit, 0x605},
    { &NvBootWb0CopyHeaderAndFirmware, 0x606},
    { &NvBootWarmBootOemProcessRecoveryCode, 0x607},
    { &NvBootArcDisableUnconditional, 0x608},
    { &NvBootLP0ContextRestore, 0x609}, // Must be placed after SE/PKA is used to authenticate/decrypt SC7 FW.
    
};


// Supported array of task lists in Bootrom.
static const NvBootTaskLists TaskLists[] =  { {CryptoInitTasks, (sizeof(CryptoInitTasks)/TASK_ENTRY)},
                                              {ColdBootTasks, (sizeof(ColdBootTasks)/TASK_ENTRY)},
                                              {RCMTasks, (sizeof(RCMTasks)/TASK_ENTRY)},
                                              {SecureExitBootTasks, (sizeof(SecureExitBootTasks)/TASK_ENTRY)},
                                              {WarmBootTasks, (sizeof(WarmBootTasks)/TASK_ENTRY)},
};



NvBootError NvBootPllpClockSourceSetup()
{
	/* NOTE arun*/
	// THIS CANNOT BE invoked until PLLP is stabilized!!!

	//assert if PLLP is not locked.

	//AI clock setting is done under NvBootBpmpFabricInit().
    //from bug #1502729
    /*
        BPMP  :
        Register CLK_RST_CONTROLLER_CLK_SOURCE_BPMP_CPU_NIC_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_BPMP_APB_0
        
        
        Fabric :
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AXI_CBB_0
        
        SE    : 
        Register CLK_RST_CONTROLLER_CLK_SOURCE_SE_0
        
        
        AON Subsystem :
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_CAN0_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_CAN1_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_UART*_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_UART_FST_MIPI_CAL_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C*_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C_SLOW_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_TOUCH_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_DMIC_0
        Register CLK_RST_CONTROLLER_CLK_SOURCE_AON_SPI_0
        
        MC/EMC :
        Register CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0 
        */

    return NvBootError_Success;
}


//http://nvbugs/1503512 
// Pending Mike's final nod, we are closing towards initializing SysRAM 
// in MROM to keep things uniform across cold and warm boot
NvBootError NvBootMemSysRamSetup()
{
    // Before starting to execute code from the SysRAM, 
    // it is recommended that the processor instruction and data 
    // caches be initialized for optimal performance by 
    // setting the SCTLR.I and SCTRL.C bits.

	// shoud be handled in asm code in start.S
	// provide line reference?

	//a.		SysRAM controller setup through APB. http://nvbugs/1503512

	//b.		Enable SMMU bypass path 	http://nvbugs/200002462

	// c.	ROC's address filtering 		http://nvbugs/1503514
	// d.	MC-Distributed Arbitration	http://nvbugs/1502745
	// e.	BPMP's AST  configuration 
	
	// c.	Before starting to execute code from the SysRAM, it is recommended 
	// that the processor instruction and data caches be initialized for 
	// optimal performance by setting the SCTLR.I and SCTRL.C bits.

	// should be handled in asm code in start.S
	// line reference?

	/*
	MRC p15, 0, <Rd>, c1, c0, 0 ; Read SCTLR
	<modify> enable I and C
	MCR p15, 0, <Rd>, c1, c0, 0 ; Write SCTLR
	*/
	return NvBootError_Success;
}

/**
 * GetPtrTasksNS task list function pointer entry table.
*/

void *GetPtrTasks(NvBootTaskListId TaskListId)
{
    return (NvBootTaskPtr) TaskLists[TaskListId].taskList;
}

/**
 * GetCntTasksNS Number of tasks included in the master task table.
*/
int GetCntTasks(NvBootTaskListId TaskListId)
{
    
    return TaskLists[TaskListId].numTasks;
}

/**
 *  Check if TaskListId for a task table is valid.
 */
NvBootError IsValidTaskListId(NvBootTaskListId TaskListId)
{
    NvBootError e = NvBootError_IllegalParameter;
    
    if(TaskListId < sizeof(TaskLists)/sizeof(NvBootTaskLists))
        e = NvBootError_Success;

    return e;
}
