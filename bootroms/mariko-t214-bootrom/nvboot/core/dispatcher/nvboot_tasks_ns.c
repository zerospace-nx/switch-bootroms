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
#include <nvboot_tasks_ns.h>
#include <nvboot_dispatcher_int.h>
#include <nvboot_bpmp_int.h>
#include <nvboot_reset_pmu_int.h>
#include "nvboot_util_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_uart_int.h"
//#include <nvboot_apb2jtag_int.h>
//#include <nvboot_lowbattery_int.h>


/**
 * NvBootTask master task list for in non secure/secure mode of operation.
*/
static const NvBootTask VT_NONSECURE TasksNS[] = {
    //We do this cast because these routines do not return
    //This is OK for nonsecure dispatcher, but we should have made a diff type
    { (NvBootError(*)())&NvBootClocksEnablePmcClock, 0x01 },
    { (NvBootError(*)())&NvBootBpmpSetupOscClk, 0x2 },
    { (NvBootError(*)())&NvBootPadsExitDeepPowerDown, 0x3},
    { (NvBootError(*)())&NvBootOverrideNonPORStraps, 0x4},
    // Note: NvBootBpmpNonsecureRomEnter may branch to UART boot here
    // if the UART boot conditions are satisfied.
    { (NvBootError(*)())&NvBootBpmpNonsecureRomEnter, 0x5 },
    // This must precede External PMIC code
    { (NvBootError(*)())&NvBootBpmpEnableWdt, 0x6 },
    // This must precede Enabling PllP
    { (NvBootError(*)())&NvBootResetRailsAndResetHandler, 0x7 },
    // All code before this point must be in the non-secure
    // section (everything to get to UART boot for FA mode).
    // Move Pllp start and clock source switch of AVP to PLLP here,
    // which will speed up simulation time. This diverges from the T210
    // slightly, which had this function just before main() entry.
    { (NvBootError(*)())&NvBootBpmpEnablePllp, 0x08 },
    { (NvBootError(*)())&NvBootClocksSetAvpClockBeforeScatterLoad, 0x09 },
};

/**
 * GetPtrTasksNS task list function pointer entry table.
*/

void FT_NONSECURE *GetPtrTasksNS()
{
    return (NvBootTaskPtr) TasksNS;
}

/**
 * GetCntTasksNS Number of tasks included in the master task table.
*/
int FT_NONSECURE GetCntTasksNS()
{
    return (sizeof(TasksNS) / sizeof(NvBootTask));
}
