/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "arfuse.h"
#include "arapbpm.h"
#include "project.h"
#include "nvboot_config.h"
#include "nvboot_version_defs.h"
#include <nvboot_asm.h>
#include "arsecure_boot.h"
#include "arclk_rst.h"

.section ".text.boot"
.globl _start
_start:
	b	reset
	b	arm_undefined
	b	arm_syscall
	b	arm_prefetch_abort
	b	arm_data_abort
	b	arm_reserved
	b	arm_irq
	b	arm_fiq

arm_undefined:
arm_syscall:
arm_prefetch_abort:
arm_data_abort:
arm_reserved:
arm_irq:
arm_fiq:
    /*
     * We don't expect any exception during boot ROM operation, and we have no
     * real way to process them either.
     * But to allow for a determinate behavior, each exception is processed in
     * the following way
     * - if in pre-production OR (production mode and FA mode), loop forever
     * otherwise
     * - the processor loops for a relatively large amount of time, this
     *   allows a debugger to take hold
     * - if the loop terminates we generate a full chip reset
     * Everything is coded using R13. R13 is banked and is normally the stack
     * pointer for the different modes, but we have no stack for the exception
     * code anyhow.  This allows to maintain the register values that were
     * present before the exception as clues for post mortem debugging
     * R14 (banked) will indicate where the exception took place
     * The CPSR mode will indicate which exception is currently processed and
     * the whole combination can be used for stack unwinding.
     */

/* read the production fuse, if set read and check for FA not set */
	ldr 	r13, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRODUCTION_MODE_0)
	ldr 	r13, [r13]
	cmp 	r13, #1
	ldreq 	r13, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FA_0)
	ldreq 	r13, [r13]
	cmpeq 	r13, #0

/* if both comparisons matched, the Z flag is set and we don't loop forever */
	bne 	.

/* if we get here, we loop for a specified amount of time then reset
 * the number of loop iterations is calculated to be a reasonable
 * number of seconds on the FPGA (8.3 MHz) and to be a simple constant
 */
	mvn 	r13, #1

.Lexcp_loop:
#if 0 /* Loop forever is preferred */
	subs 	r13, r13, #1
/* generate a full chip reset, we can clobber other registers now */
    ldreq   r0, =(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0)
    ldreq   r1, =(APBDEV_PMC_CNTRL_0_MAIN_RST_FIELD)
    streq   r1, [r0]
#endif 
	b 	.Lexcp_loop

/* make sure all related literals remain close */
.ltorg
.globl NvBootBootromVersionAddress
NvBootBootromVersionAddress:
    .word  CONST_NVBOOT_BOOTROM_VERSION
    .word  CONST_NVBOOT_RCM_VERSION
    .word  CONST_NVBOOT_BOOTDATA_VERSION

reset:
.Lstack_setup:
        /* set up the stack for irq, fiq, abort, undefined, system/user, and lastly supervisor mode */
        mrs     r0, cpsr
        bic     r0, r0, #0x1f

        ldr	r2, =__stack_top
        orr     r1, r0, #0x12 // irq
        msr     cpsr_c, r1
        mov	r13, r2

        orr     r1, r0, #0x11 // fiq
        msr     cpsr_c, r1
        mov	sp, r2
                    
        orr     r1, r0, #0x17 // abort
        msr     cpsr_c, r1
        mov	sp, r2
            
        orr     r1, r0, #0x1b // undefined
        msr     cpsr_c, r1
        mov	sp, r2
            
        orr     r1, r0, #0x1f // system
        msr     cpsr_c, r1
        mov	sp, r2

        orr	r1, r0, #0x13 // supervisor
        msr	cpsr_c, r1
        mov	sp, r2

        ldr r3, =0x0
        ldr r4, =0x0
        ldr r5, =0x0
        ldr r6, =0x0
        ldr r7, =0x0
        ldr r8, =0x0
        ldr r9, =0x0
        ldr r10, =0x0
        ldr r11, =0x0

        ldr r12, [pc]
        bx  r12
        .word  .Lthumb_start + 1
.code 16
.Lthumb_start:
        #=============================================================================
        # Setup Pirom Region
        bl  NvBootSetupPiromRegion
        #=============================================================================
        #=============================================================================
        #  apply irompatch logic 
        bl	NvBootApplyIRomPatch
        #=============================================================================

    /* ================================================================= */	
        bl	NvBootNonsecureDispatcher
    /* ================================================================= */
.align 4    
        bx  pc
.code 32
        b   NvBootAsmSecureEnter
.ltorg
.section ".text"
NvBootAsmSecureEnter:
        /* copy the initialized data segment out of rom if necessary */
        ldr	    r0, =__data_start_rom
        /* check for '$CPY' pattern in end of .data */
        ldr     r1, =__data_end
        ldr     r2, =__data_start
        sub     r1, r2
        add     r0, r1
        ldr     r0, [r0, #-0x04]
        ldr     r1, =0x59504324
        cmp     r0, r1
        bne     .L__do_bss
        ldr     r0, =__data_start_rom
        ldr	    r1, =__data_start
        ldr	    r2, =__data_end

        cmp	    r0, r1
        beq	    .L__do_bss

.L__copy_loop:
        cmp	    r1, r2
        ldrlt	r3, [r0], #4
        strlt	r3, [r1], #4
        blt	    .L__copy_loop

.L__do_bss:
        // clear out the bss 
        ldr	    r0, =__bss_start
        ldr	    r1, =_end
        mov	    r2, #0
.L__bss_loop:
        cmp	    r0, r1
        strlt	r2, [r0], #4
        blt	    .L__bss_loop

        bl	    main
        b	    do_exception
.ltorg

//=============================================================================
// The secure ROM exit code has some special characteristics to protect
// the information it contains.  It goes like this
// - The secure exit code is part of the non secure portion of the IROM BUT
// - The secure data it needs (e.g. sceure boot register address) is part 
//   of the secure ROM
//=============================================================================
//=============================================================================
//	Code to handle the secure boot ROM exit
//      The parameters here match NvBootMainAsmSecureExit.
//      This follows the normal calling conventions
//      r0: first parameter is the jump address for BPMP 
//          if 0 it means BPMP is put to sleep (this essentially kills the chip)
//      r1: start address of memory to clear
//      r2: first address of memory not to clear
//      r1 and r2 are guaranteed 32 bytes aligned to allow clearing
//      using a STM of r4-r11 (8 registers) as one burst 
//      r3 pointer to the address where the value to be programmed into
//      SB_CSR_0 is stored.
//      Part of the literal used by this code is in the secure portion
//=============================================================================       
.section ".text.boot"
.code 16
FUNCTION(trap_exception)
        mov     r0, #0x01
        lsl     r0, #20
        add     r0, #0x0C
        bx      r0
.code 32        
FUNCTION(NvBootMainAsmSecureExit)
        // Efficiently set memory between r1 and r2 to ARM branch to self.
        // We could also use a LDM from a guaranteed string of what we want to set.
        ldr r4, =0xEAFFFFFE
        ldr r5, =0xEAFFFFFE
        ldr r6, =0xEAFFFFFE
        ldr r7, =0xEAFFFFFE
        ldr r8, =0xEAFFFFFE
        ldr r9, =0xEAFFFFFE
        ldr r10, =0xEAFFFFFE
        ldr r11, =0xEAFFFFFE
        // loop between r1 and r2, STM with autoincrement
.AsmSecureExitClearLoop:
        stm r1!,{r4-r11}
        cmp r1, r2
        blo .AsmSecureExitClearLoop

        // Readback last written address in the clear loop above to flush the
        // last stm.
        ldr r4, [r2, #-4]

        // Log boot exit timestamp
        ldr r2, =(NV_ADDRESS_MAP_TMRUS_BASE)
        ldr r2, [r2]
        mov r1, r3 // Save ptr to BIT.NvBootTimeLogExit
        ldr r3, [r3] // Load value of SB_CSR_0 stashed in BIT.NvBootTimeLogExit
        str r2, [r1] // Store timestamp to BIT.NvBootTimeLogExit

        // Preserve r3 into r2, the value to write into SB_CSR_0.
        mov r2, r3

        // make ROM secure, uses data part of the secure part of the IROM
        ldr r8, =(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0)
        // Clear secure boot flag again, don't trust value from BIT.NvBootTimeLogExit.
        bic r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        str r3, [r8]
        ldr r3, [r8] // Force a readback of SB_CSR_0 to make sure it has
                     // taken effect before proceeding.
        and r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        // Cannot use SB_CSR_0_SECURE_BOOT_FLAG_DISABLE directly.
        // Double check that SECURE_BOOT_FLAG is 0.
        // If not, retry up to 16 times, and then trigger an exception which
        // will lead to a reset for production systems.
        cmp r3, #0
        beq  .BootRomDone

        // Retry SECURE_BOOT_FLAG clear. Reload everything from scratch.
        mov r3, r2
        ldr r8, =(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0)
        // Clear secure boot flag again, don't trust value from BIT.NvBootTimeLogExit.
        bic r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        str r3, [r8]
        ldr r3, [r8] // Force a readback of SB_CSR_0 to make sure it has
                     // taken effect before proceeding.
        and r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        // Cannot use SB_CSR_0_SECURE_BOOT_FLAG_DISABLE directly.
        // Double check that SECURE_BOOT_FLAG is 0.
        // If not, retry up to 16 times, and then trigger an exception which
        // will lead to a reset for production systems.
        cmp r3, #0
        beq .BootRomDone

        // Retry SECURE_BOOT_FLAG clear. Reload everything from scratch.
        mov r3, r2
        ldr r8, =(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0)
        // Clear secure boot flag again, don't trust value from BIT.NvBootTimeLogExit.
        bic r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        str r3, [r8]
        ldr r3, [r8] // Force a readback of SB_CSR_0 to make sure it has
                     // taken effect before proceeding.
        and r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        // Cannot use SB_CSR_0_SECURE_BOOT_FLAG_DISABLE directly.
        // Double check that SECURE_BOOT_FLAG is 0.
        // If not, retry up to 16 times, and then trigger an exception which
        // will lead to a reset for production systems.
        cmp r3, #0
        beq .BootRomDone

        // Retry SECURE_BOOT_FLAG clear. Reload everything from scratch.
        mov r3, r2
        ldr r8, =(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0)
        // Clear secure boot flag again, don't trust value from BIT.NvBootTimeLogExit.
        bic r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        str r3, [r8]
        ldr r3, [r8] // Force a readback of SB_CSR_0 to make sure it has
                     // taken effect before proceeding.
        and r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        // Cannot use SB_CSR_0_SECURE_BOOT_FLAG_DISABLE directly.
        // Double check that SECURE_BOOT_FLAG is 0.
        // If not, retry up to 16 times, and then trigger an exception which
        // will lead to a reset for production systems.
        cmp r3, #0
        beq .BootRomDone

        // Retry SECURE_BOOT_FLAG clear. Reload everything from scratch.
        mov r3, r2
        ldr r8, =(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0)
        // Clear secure boot flag again, don't trust value from BIT.NvBootTimeLogExit.
        bic r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        str r3, [r8]
        ldr r3, [r8] // Force a readback of SB_CSR_0 to make sure it has
                     // taken effect before proceeding.
        and r3, #SB_CSR_0_SECURE_BOOT_FLAG_FIELD
        // Cannot use SB_CSR_0_SECURE_BOOT_FLAG_DISABLE directly.
        // Double check that SECURE_BOOT_FLAG is 0.
        // If not, retry up to 16 times, and then trigger an exception which
        // will lead to a reset for production systems.
        cmp r3, #0
        bne do_exception

.BootRomDone:
        // clear all remaining registers not further needed
        // need to be preserved: r0, r13(sp) and r15(pc)
        ldr r1, =0
        ldr r2, =0
        ldr r3, =0
        ldr r4, =0
        ldr r5, =0
        ldr r6, =0
        ldr r7, =0
        ldr r8, =0
        ldr r9, =0
        ldr r10, =0
        ldr r11, =0
        ldr r12, =0
        ldr r14, =0
        //;; jump to the required address, use BX to allow jump to thumb code
        bx r0

.ltorg

.section ".text.boot"

/**
FUNCTION(NvBootMainSetupR5Cache)

    push { r0, r1, r14}

    /// I/D cache setting should be based on bct. until BCT is read, I/D cache is disabled by default. -- AI Arun
    //=============================================================================
    // Initialize and Enable caches (section 8.5.5 of R5 TRM) --start
    //=============================================================================
    MRC p15, 0, r1, c1, c0, 0 	// Read System Control Register configuration data
    ORR r1, r1, #0x1 <<12 	// instruction cache enable
    ORR r1, r1, #0x1 <<2 	// data cache enable
    DSB
    MCR p15, 0, r0, c15, c5, 0 // Invalidate entire data cache
    MCR p15, 0, r0, c7, c5, 0 	// Invalidate entire instruction cache
    MCR p15, 0, r1, c1, c0, 0 	// enabled cache RAMs
    ISB
    //=============================================================================
    // Initialize and Enable caches (section 8.5.5 of R5 TRM) --end
    //=============================================================================
    pop { r0, r1, r15}
.ltorg
*/
