/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_BPMP_INT_H
#define INCLUDED_NVBOOT_BPMP_INT_H

#include <nvboot_section_defs.h>
#include "nvboot_osc.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Setup Context struct of BR.
 */
void NvBootMainSecureInit(void);


/**
 * The last function to be called at Boot ROM
 * exit. Will call into NvBootMainAsmSecureExit which is
 * in start.S.
 * @param BootloaderEntryAddress pointer payload entry point
 * @param StartClearAddress address for start of btcm clearing
 * @param StopClearAddress address for end of btcm clearing
 * @param SecureRegisterValueAddr The address where the value to be programmed
 *                                into SB_CSR_0 is stored.
 */
void NvBootBpmpSecureExit(NvU32 BootloaderEntryAddress, 
	NvU32 StartClearAddress,
	NvU32 StopClearAddress, 
	NvU32 SecureRegisterValueAddr);

/**
 * The last secure exit code to be run before BR hands off
 * to the next stage. Written in assembly, located in start.S
 */
void NvBootMainAsmSecureExit(int BootloaderEntryAddress,
                            int StartClearAddress,
                            int StopClearAddress,
                            int SecureRegisterValue);

/**
 * NvBootBpmpSubSystemInit.
 *
 * For T214, this function should be called to initialize MC clocks and ensure
 * proper operation of the IO brick. http://nvbugs/1877027.
 *
 * This function should only be called for non-SC7 paths.
 * SC7 paths should call NvBootEnableMemClk after unpacking the SDRAM parameters.
 *
 */
NvBootError NvBootBpmpSubSystemInit(void);

/**
 * NvBootBpmpFabricInit provides information regarding the sequence to  
 * enable pmc, NIC and fabric subsytem. 
 *
 */

void FT_NONSECURE NvBootBpmpFabricInit(void);

/**
 * Replaces NvBootMainNonsecureRomEnter from T210.
 * Performs early startup functions.
 * - Enable clock to PMC so BR can access PMC.
 * - Detect OSC. Program into CAR.
 * - Configure microsecond timer.
 * - Program any PLL auto start sequences.
 */
void FT_NONSECURE NvBootBpmpNonsecureRomEnter(void);

/**
 *  t210 bug fix http://nvbugs/1867566
 *  Tegra should not re-sample strapping option on PMC.MAIN_RST and Tegra WDT reset
 */
void FT_NONSECURE NvBootOverrideNonPORStraps(void);

/**
 * Initial clock setup needed by all paths in the Boot ROM.
 *
 */
void FT_NONSECURE NvBootBpmpSetupOscClk(void);

/**
 * Bring the pads out of Deep Power Down operation.
 *
 */
void FT_NONSECURE NvBootBpmpSetupPads(void);

/**
 * Set up and enable WDT if appropriate
 *
 */
void NvBootBpmpEnableWdt(void);

/**
 * Enable Pllp clock and check for lock bit status.
 *
 */
void NvBootBpmpEnablePllp(void);

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
 * This function replaces the NvBootMainSecureRomEnterBeforeScatter function
 * from T210.
 *
 * @param none
 *
 * @return none
 */
void NvBootBpmpSecureRomEnterBeforeScatter(void);

/**
 * Enable Apb2Jtag programmign sequence based on fuse data.
 *
 */
NvBootError NvBootBpmpEnableApb2jtag(void);

/**
 * NonSecureBootloader for looping on failure
 *
 */
void FT_NONSECURE NvBootMainNonSecureBootLoader(void);

/**
 * Secure exit start.
 *
 */
NvBootError NvBootBpmpSecureExitStart();

/**
 * Client clocks setup
 */
void NvBootClientClockSetup(uint8_t BpmpCpu, uint8_t BpmpApb, uint8_t AxiCbb, uint8_t Se, uint8_t EmcRocClk);

#if 0
/**
 * If WDT L2 Reset, we do power ungating because we plan to coldboot
 */
NvBootError NvBootBpmpUnPGL2Rst(void);
#endif

/**
 * If L2 Reset and AoWake Status set for warmboot path, BR would spin in loop.
 */

void FT_NONSECURE NvBootBpmpCheckSc7WakeStatus(void);

/**
 *  Minimal secure exit that can be achieved during early boot for errors encountered in non-secure
 *  region code.
 */
void FT_NONSECURE NvBootMinimalAssetLockDownExit(void);

/**
 * do_exception(void)
 *
 */
void do_exception(void);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_BPMP_INT_H
