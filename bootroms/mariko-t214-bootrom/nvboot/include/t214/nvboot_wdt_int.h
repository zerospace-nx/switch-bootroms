/*
 * Copyright (c) 2012 - 2013 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_wdt_int.h - Declarations for wdt support.
 */

#ifndef INCLUDED_NVBOOT_WDT_INT_H
#define INCLUDED_NVBOOT_WDT_INT_H

#include "nvtypes.h"
#include "arclk_rst.h"
#include "nvboot_clocks.h"
#include "nvboot_error.h"
#include "nvboot_osc.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#define WDT_TIMEOUT_VAL_NONRCM   1000000  //  4/4 sec = 4000000/4 usec = 1us. Watchdog expires at 4th expiration

#define WDT_TIMEOUT_VAL_RCM     11250000  // 45/4  sec = 45000000/4 usec = 11250000. Watchdog expires at 4th expiration

/* Get Watchdog status
*/
NvBool
NvBootWdtGetStatus(void);
/* Configure Watchdog related registers
*/
void
NvBootWdtInit(void);
/* Start Watchdog Timer
*/
void
NvBootWdtStart(void);
/* Stop Watchdog Timer
*/
void
NvBootWdtStop(void);
/* Reload Watchdog value
*/
void
NvBootWdtReload(NvU32 WatchdogTimeout);


#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_WDT_INT_H */
