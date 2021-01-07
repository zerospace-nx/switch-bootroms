/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_main_int.h
 *
 * Main entry points for Boot ROM.
 *
 * The sequence of calls is --
 * 1. NvBootMainNonsecureRomEnter() -- never returns if the chip is in
 *    Pre-production Mode or FA Mode, otherwise returns immediately
 * 2. NvBootMainSecureRomEnterBeforeScatter() -- sets up PLLP and then
 *    returns
 * 3. NvBootMainSecureRomEnter() -- never returns
 */

#ifndef INCLUDED_NVBOOT_MAIN_INT_H
#define INCLUDED_NVBOOT_MAIN_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * This routine is located in the Non-Secure Section of iROM and handles
 * booting for --
 * * Pre-production Mode
 * * Failure Analysis
 *
 * In these two cases, we jump to the UART bootloader and never return.
 *
 * For all other operating modes, the code returns immediately and the caller
 * will need to enter the secure section of the iROM.
 *
 * This code is placed in Non-Secure Section of iROM and is executed before the 
 * C runtime environment has been set up, so there are a number of limitations
 * on the code.  Especially, no global initialized variables can be referenced.
 * Only local variables are allowed here and in all routines called from here.
 *
 * @param none
 *
 * @return none, this routine only returns when not in Pre-production Mode or
 *         FA Mode
 */
void NvBootMainNonsecureRomEnter(void);

/**
 * This routine is located in the Secure Section of iROM and is responsable
 * for starting PLLP as early as possible in the boot cycle so that all 
 * remaining boot operations can be carried out at a faster clock frequency.
 * The specific steps are as follows --
 *
 * 1. if Warm Boot 0 (WB0), retrieve stored oscillator frequency info from
 *    PMC Scratch registers; else, measure the oscillator frequency
 * 2. if no valid oscillator frequency is found, assume 13 MHz
 * 3. start PLLP and wait for it to become stable
 * 4. set the system clock (sclk) to pllp_out4 = PLLP/4 = 108 MHz (assuming
 *    a valid oscillator frequency was found)
 *
 * This routine is executed before the C runtime environment has been set up, 
 * so there are a number of limitations on the code.  Especially, no global
 * initialized variables can be referenced.  Only local variables are allowed
 * here and in all routines called from here.
 *
 * @param none
 *
 * @return none
 */
void NvBootMainSecureRomEnterBeforeScatter(void);

/**
 * This routine is located in the Secure Section of iROM and handles booting
 * for --
 * * NV Production Mode
 * * ODM Production Mode, Secure
 * * ODM Production Mode, Non-Secure
 *
 * Note that Warm Boot 0 (WB0) and Recovery Mode are supported for all
 * Production Modes; they're also handled in this routine.
 *
 * After determining that the chip is not in Preproduction Mode or Failure
 * Analysis Mode and after C runtime initialization, Boot ROM execution transfers
 * to this routine.
 *
 * This code is placed in Secure Section of iROM and is executed after the C
 * runtime environment has been set up, so there are no special restrictions
 * on the operations that can be performed here (other than that not many
 * of the standard library routines are supported).
 *
 * @param none
 *
 * @return none, this routine should never return
 */
void NvBootMainSecureRomEnter(void);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MAIN_INT_H */
