// 
// Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
// 
// NVIDIA Corporation and its licensors retain all intellectual property
// and proprietary rights in and to this software and related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//

//
// excp_irom_patch.ss - IROM patch SWI handler
//

//=============================================================================
	AREA	IPatchNonsecure, CODE, READONLY, ALIGN=4

	EXPORT IRomPatch_prologue
	EXPORT IRomPatch_end
	EXPORT SwCYAWord
	IMPORT IRamExcpHandler

// BootRom patch SWI handler. Invoke by SWI #imme_8 thumb instruction.
// #immed_8 contains the entry offset into thumb code buffer from fuse.

IRomPatch_prologue
	stmdb sp!, {r0,r1,r2}
	mov r2, lr
	sub r2, r2, #2
	ldr r2, [r2]
	and r2, r2, #255
	mov r2, r2, LSL#1
	ldr r0, =(IRomPatch_prologue)
	ldr r1, =(IRomPatch_end)
	sub r1, r1, r0
	ldr r0, =(IRamExcpHandler)
	add r0, r0, r1
	add r2, r2, r0
	orr r2, r2, #1
	ldmia sp!, {r0,r1}
	bx  r2

// Thumb code buffer from fuse start ...	
// The SWI handler return needs to be coded in fuse. 
// R2 available in stack. The SWI handler must pop it out before returning

	LTORG
IRomPatch_end
	b	.

// For SW CYA implementation
SwCYAWord
	DCD	0

	END
