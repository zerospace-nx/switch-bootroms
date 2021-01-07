// 
// Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
// 
// NVIDIA Corporation and its licensors retain all intellectual property
// and proprietary rights in and to this software and related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//

//
// boot0.ss - AP20 boot code
//

// Overrides the standard macro that inserts a UL prefix (which is confusing for asm)
#define _MK_ENUM_CONST(_constant_) _constant_

//=============================================================================
// Includes and Equates
//=============================================================================

#include "arapbpm.h"
#include "arclk_rst.h"
#include "arflow_ctlr.h"
#include "arfuse.h"
#include "arpg.h"
#include "arsecure_boot.h"
#include "nvboot_limits_int.h"
#include "nvboot_version_defs.h"
#include "project.h"
#include "nvboot_config.h"

//=============================================================================
// Non secure boot code starts here, in its own area ...
//=============================================================================

        PRESERVE8 ; Not strictly needed, but we'll start here with 8 alignment 
    	AREA	Boot0, CODE, READONLY, ALIGN=6 ; Use ALIGN 6 = relocation 
                                               ; table alignment

//=============================================================================
// Subroutines called elsewhere, and potentially useful labels
//=============================================================================
        EXPORT __boot_start
        EXPORT NvBootBootromVersionAddress

//=============================================================================
// Subroutines defined elsewhere
//=============================================================================
// XXX        IMPORT __rt_entry
        IMPORT __main
        IMPORT __use_no_heap         ; don't allow any routines that use the heap
        IMPORT __use_no_heap_region  ; don't allow any routines that use the heap
        IMPORT __use_no_semihosting_swi  ; don't allow any routines that use semihosting
        IMPORT NvBootMainNonsecureRomEnter
        IMPORT NvBootMainSecureRomEnterBeforeScatter
	    IMPORT NvBootApplyIRomPatch
        IMPORT NvBootResetRailsAndResetHandler

//=============================================================================
// EXCEPTION TABLE (32 bytes)
//=============================================================================

	ENTRY 

__boot_start

reset_exception        
        b         reset_exception_start    ; reset
        b         undefined_exception      ; undefined instruction
        b         swi_exception            ; software interrupt
        b         prefetch_abort           ; prefetch abort
        b         data_abort               ; data abort
        b         rsvd_exception           ; reserved
        b         irq_exception            ; irq handler
        b         fiq_exception            ; fiq handler

//=============================================================================
// PADDING (20 bytes) 
//=============================================================================
        ; this area could be used to encode long jump target addresses
	DCD 0,0,0,0,0

//=============================================================================
// Version (4 bytes) 
//=============================================================================
        ; we define ane export a label to this address
NvBootBootromVersionAddress
        DCD CONST_NVBOOT_BOOTROM_VERSION
	    DCD CONST_NVBOOT_RCM_VERSION
	    DCD CONST_NVBOOT_BOOTDATA_VERSION

        ALIGN 0x40

        // The relocation table used to reside here.  It has been eliminated.

//=============================================================================
// start of boot code
//=============================================================================

//=============================================================================
//	Branch the other processor here
//=============================================================================
reset_exception_start

// setup stack on AVP
// exception handlers don't need stack for the bootrom, as they all die
// so only one stack is needed, common for system and normal mode
        ldr sp, =(NVBOOT_MAIN_TOP_OF_STACK)

// Log boot init timestamp
	    ldr r0, =(NV_ADDRESS_MAP_TMRUS_BASE)
	    ldr r1, =(NV_ADDRESS_MAP_IRAM_A_BASE+0x100)
	    ldr r0, [r0]
	    str r0, [r1]

// Jump into a C function in nonsecure portion and pre C runtime
// This will check for preproduction and FA mode.  If in one of these, 
// the system performs an UART download and never returns.
// Otherwise the system returns here and jump into the secure portion
	    bl	NvBootApplyIRomPatch
        bl      NvBootMainNonsecureRomEnter
        b       NvBootMainAsmSecureEnter

// We don't expect any exception during boot ROM operation, and we have no
// real way to process them either. 
// But to allow for a determinate behavior, each exception is processed in
// the following way
// - if in production mode and not FA mode, loop forever
// otherwise
// - the processor loops for a relatively large amount of time, this
//   allows a debugger to take hold
// - if the loop terminates we generate a full chip reset
// Everything is coded using R13. R13 is banked and is normally the stack
// pointer for the different modes, but we have no stack for the exception
// code anyhow.  This allows to maintain the register values that were 
// present before the exception as clues for post mortem debugging
// R14 (banked) will indicate where the exception took place
// The CPSR mode will indicate which exception is currently processed and 
// the whole combination can be used for stack unwinding.
//=============================================================================
//	Handling for all exceptions
//=============================================================================
undefined_exception
swi_exception
prefetch_abort
data_abort
rsvd_exception
irq_exception
fiq_exception
        // read the production fuse, if set read and check for FA not set
        ldr r13, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRODUCTION_MODE_0)
        ldr r3, [r13]
        cmp r13, #1
        ldreq r3, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FA_0)
        ldreq r13, [r13]
        cmpeq r13, #0
        // if both comparisons matched, the Z flag is set and we don't loop forever
forever_exception_loop
        bne forever_exception_loop
        // if we get here, we loop for a specified amount of time then reset
        // the number of loop iterations is calculated to be a reasonable
        // number of seconds on the FPGA (8.3 MHz) and to be a simple constant
        ldr r13, =NVBOOT_EXCEPTION_LOOP_COUNT
exception_loop
        subs r13, r13, #1
        // generate a full chip reset, we can clobber other registers now
        ldreq r0, =(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0)
        ldreq r1, =(APBDEV_PMC_CNTRL_0_MAIN_RST_FIELD)
        streq r1, [r0]
        b exception_loop
       
// make sure all related literals remain close
        LTORG  

//=============================================================================
// The secure ROM exit code has some special characteristics to protect
// the information it contains.  It goes like this
// - The secure exit code is part of the non secure portion of the IROM BUT
// - The secure data it needs (e.g. sceure boot register address) is part 
//   of the secure ROM
//=============================================================================
//=============================================================================
//	Code to handle the secure boot ROM exit
//      This follows the normal calling conventions
//      r0: first parameter is the jump address for AVP
//          if 0 it means AVP is put to sleep (this essentially kills the chip)
//      r1: start address of memory to clear
//      r2: first address of memory not to clear
//      r1 and r2 are guaranteed 32 bytes aligned to allow clearing
//      using a STM of r4-r11 (8 registers) as one burst 
//      r3 bitmap to write in the secure boot register
//      Part of the literal used by this code is in the secure portion
//=============================================================================       


        PRESERVE8 ; Not really needed, but true (sp is not modified)
	    AREA	SecureExitCode, CODE, READONLY, ALIGN=5 ; cache line aligned
                                              
NvBootMainAsmSecureExit
        EXPORT NvBootMainAsmSecureExit
        // the first part efficiently clear memroy between r1 and r2
        // we could use a LDM from a guaranteed zero string
        ldr r4, =0x0
        ldr r5, =0x0
        ldr r6, =0x0
        ldr r7, =0x0
        ldr r8, =0x0
        ldr r9, =0x0
        ldr r10, =0x0
        ldr r11, =0x0
        // loop between r1 and r2, STM with autoincrement
AsmSecureExitClearLoop
        stm r1!,{r4-r11}
        cmp r1, r2
        blo AsmSecureExitClearLoop

        // Program APBDEV_PMC_DEBUG_AUTHENTICATION_0
        // Value to write into APBDEV_PMC_DEBUG_AUTHENTICATION is temporarily
        // stashed in BootInfoTable.BootRomVersion.
        ldr r1, BootInfoTableAddress 
        ldr r8, [r1] // Value of PMC_DEBUG_AUTHENTICATION_0
        // Address of APBDEV_PMC_DEBUG_AUTHENTICATION_0.
        ldr r2, SecureDebugControlAddress
        str r8, [r2]
        // Restore Boot ROM version into BootInfoTable.BootRomVersion
        ldr r2, =CONST_NVBOOT_BOOTROM_VERSION
        str r2, [r1]

        // Log boot exit timestamp
        ldr r2, TmrUsBaseAddress
        ldr r2, [r2]
        mov r1, r3
        ldr r3, [r3]
        str r2, [r1]

        // make ROM secure, uses data part of the secure part of the IROM
        ldr r8, SecureRegAddress
        str r3, [r8]
        // clear all remaining registers not further needed
        // need to be preserved: r0, r13(sp) and r15(pc)
        ldr r1, =0
        ldr r2, =0
        ldr r3, =0
        ldr r8, =0
        ldr r12, =0
        ldr r14, =0
        ;; jump to the required address, use BX to allow jump to thumb code  
        bx r0        

        // any other non explicit literal must remain in the non secure part
        // so force literal pool here
        LTORG

//=============================================================================
// Secure Boot Code starts, in its own area ...
// Large align to simplify the ECO defining the boundary (here on 256 boundary)
//=============================================================================

        PRESERVE8 ; ; Not really needed, but true (sp is not modified)
        AREA	Boot0Secure, CODE, READONLY, ALIGN=8 ; align 8 for ECO

// Data needed by the SecureRom exit that must be part of the Secure IROM
// Explicit literals
SecureRegAddress       
        DCD (NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0)
TmrUsBaseAddress
        DCD (NV_ADDRESS_MAP_TMRUS_BASE)
SecureDebugControlAddress
        DCD (NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DEBUG_AUTHENTICATION_0)
BootInfoTableAddress
        DCD (NVBOOT_BIT_IRAM_START)

NvBootMainAsmSecureEnter
        // The first step is to start PLLP, this is still pre C runtime, but
        // secure code is now acceptable (guaranteed to not be FA mode)

        // this requires to have PMC clock enabled to check the warmboot flag
        ldr r0, =(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0)
        ldr r1, [r0]
        orr r1,r1, #(1 << CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_PMC_SHIFT)
        str r1, [r0]

        // Before switch to PLLP, handle wdt and tsense
        bl      NvBootResetRailsAndResetHandler
 
        bl      NvBootMainSecureRomEnterBeforeScatter

        // We can now establish the C runtime, the AVP runs at 108 MHz
        b       __main

        LTORG

	END
    
// end of file boot0.ss
