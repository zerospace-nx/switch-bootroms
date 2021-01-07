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
// init_clib.ss - call back routine for initializing c library
//

#include "arapbpm.h"
#include "arfuse.h"
#include "nvboot_limits_int.h"
#include "project.h"

        AREA cpu_c_library, CODE, READONLY, ALIGN=5
        PRESERVE8               ; need these 8 things else linker complains.
        REQUIRE8
        EXPORT __user_initial_stackheap
        EXPORT __rt_raise
        EXPORT _sys_exit
                
__user_initial_stackheap
//===========================================================================
// The stack/heap address use the same defines as the scttaer file
// Heap is not used, set same as Stack to be safe
//===========================================================================
        ldr r0, =NVBOOT_MAIN_TOP_OF_STACK
        ldr r1, =NVBOOT_MAIN_TOP_OF_STACK
        bx lr


// For both handlers below, that should never be called, we duplicate the 
// approach used for the exception handlers: loop then reset the chip
// if we are in production mode but not in FA mode
// but we use r11 as the loop register as sp carry important information to
// preserve in this case.  R11 is lost (defined as variable register 8 
// in the aapcs, trying to avoid anything important)

__rt_raise
//===========================================================================
// from rt_misc.h
//
//   void __rt_raise(int /*sig*/, int /*type*/);
//
//   Redefine this to replace the library's entire signal handling
//   mechanism in the most efficient possible way. The default
//   implementation of this is what calls __raise (above).
//
// __rt_raise is called by the integer divide-by-zero fault, for example.  
// it must be re-implemented in order to remove semihosting support (see
// "RealView Compilation Tools: Compilers and Libraries Guide" Version 3.0, 
// ARM DUI 0205G, Section 5.3.2 "building an application for a nonsemihosted 
// environment" for details.
//===========================================================================
        // read the production fuse, if set read FA fuse and check for FA
        ldr r11, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRODUCTION_MODE_0)
        ldr r1, [r11]
        cmp r11, #1
        ldreq r1, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FA_0)
        ldreq r11, [r11]
        cmpeq r11, #0
        // if both comparisons matched, the Z flag is set and we don't loop forever
forever_raise_loop
        bne forever_raise_loop
        // if we get here, we loop for a specified amount of time then reset
        // the number of loop iterations is calculated to be a reasonable
        // number of seconds on the FPGA (8.3 MHz) and to be a simple constant
        ldr r11, =NVBOOT_EXCEPTION_LOOP_COUNT
raise_loop
        subs r11, r11, #1
        // generate a full chip reset, we can clobber other registers now
        ldreq r0, =(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0)
        ldreq r1, =(APBDEV_PMC_CNTRL_0_MAIN_RST_FIELD)
        streq r1, [r0]
        b raise_loop

_sys_exit
//===========================================================================
// from rt_sys.h
//
//  void _sys_exit(int /*returncode*/);   /* never returns */
//
//  Terminate the program, passing a return code back to the user.
//  This function may not return.
//
// from Section 5.8.1 of "RealView Compilation Tools: Compilers and Libraries
// Guide" Version 3.0, ARM DUI 0205G
//     
//  Caution: This function is called if a stack overflow occurs.  If you
//  re-implement this function, the overflow causes an immediate return to
//  _sys_exit() causing a worse stack overflow.  Do not use this function to
//  perform stack checking.
//
// Default implementation of this function appears to use semihosting;
// re-implementing in order eliminate need for semihosting.
//===========================================================================
        // read the production fuse, if set read FA fuse and check for FA
        ldr r11, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRODUCTION_MODE_0)
        ldr r1, [r11]
        cmp r11, #1
        ldreq r1, =(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FA_0)
        ldreq r11, [r11]
        cmpeq r11, #0
        // if both comparisons matched, the Z flag is set and we don't loop forever
forever_exit_loop
        bne forever_exit_loop
        // if we get here, we loop for a specified amount of time then reset
        // the number of loop iterations is calculated to be a reasonable
        // number of seconds on the FPGA (8.3 MHz) and to be a simple constant
        ldr r11, =NVBOOT_EXCEPTION_LOOP_COUNT
exit_loop
        subs r11, r11, #1
        // generate a full chip reset, we can clobber other registers now
        ldreq r0, =(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0)
        ldreq r1, =(APBDEV_PMC_CNTRL_0_MAIN_RST_FIELD)
        streq r1, [r0]
        b exit_loop




        ldr r12, =NVBOOT_EXCEPTION_LOOP_COUNT
exception_loop_sys_exit
        subs r12, r12, #1
        // generate a full chip reset, we can clobber other registers now
        ldreq r0, =(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0)
        ldreq r1, =(APBDEV_PMC_CNTRL_0_MAIN_RST_FIELD)
        streq r1, [r0]
        b exception_loop_sys_exit

        END
