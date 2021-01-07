/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_limits_int.h - Declarations of chip-specific limits
 */

#include "project.h"
#include "nvboot_hacks_int.h"

#ifndef INCLUDED_NVBOOT_LIMITS_INT_H
#define INCLUDED_NVBOOT_LIMITS_INT_H

// Simple defines for limits used by the scatter file
// Only #define should be used

// Scatter file has problems with some of the addresses defined in project.h,
// in particular those defined like ~(1234).  The offending addresses are 
// manually redefined here as straight hex constants.  Consistency checks are
// used to ensure the two defintions don't get out of sync.

//Bug 937662: Move IROM and EVPs to 0000_0000 - 00FF_FFFF, IROM moves from FFF0_0000 to 0010_0000 
#define NVBOOT_LIMITS_ADDRESS_MAP_IROM_BASE    NV_ADDRESS_MAP_IROM_BASE 

#define NVBOOT_LIMITS_ADDRESS_MAP_DATAMEM_BASE 0x40000000

#if NVBOOT_LIMITS_ADDRESS_MAP_IROM_BASE != NV_ADDRESS_MAP_IROM_BASE
 #error mismatch between project.h and local definition of IROM base address
#endif

#if NVBOOT_LIMITS_ADDRESS_MAP_DATAMEM_BASE != NV_ADDRESS_MAP_DATAMEM_BASE
 #error mismatch between project.h and local definition of DATAMEM base address
#endif

#define NVBOOT_MAIN_STACK_SIZE 0x2000
#define NVBOOT_MAIN_IRAM_SIZE  0x10000

#define NVBOOT_MAIN_NONSECURE_SIZE NV_ADDRESS_MAP_PROTECTED_ROM_START 

// exceptions and other abnormal conditions are treated by performing
// a busy loop of a few seconds then resetting the chip
// The iterations count is calculated assuming 108 MHz operation 
// It is also approximate, assuming 7 cycles per loop (not measured (yet))
// The loop count is defined to allow for immediate encoding (no literal)
#define NVBOOT_LOOPS_PER_SECOND (0x1 << 24)
#define NVBOOT_EXCEPTION_TARGET_TIME (10)
#define NVBOOT_EXCEPTION_LOOP_COUNT \
        (NVBOOT_EXCEPTION_TARGET_TIME * NVBOOT_LOOPS_PER_SECOND)

// The stack should be aligned and intially point just to the first non valid
// address.  Push and Pop are mapped into STMBD and LDMIA respectively, so
// decrement before use
#define NVBOOT_MAIN_TOP_OF_STACK \
        (NV_ADDRESS_MAP_IRAM_A_BASE + NVBOOT_MAIN_IRAM_SIZE) 

#endif /* #ifndef INCLUDED_NVBOOT_LIMITS_INT_H */
