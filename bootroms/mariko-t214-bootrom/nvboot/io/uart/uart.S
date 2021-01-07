/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "project.h"
#include "nvboot_config.h"
#include <nvboot_asm.h>

.section ".text.boot"
.code 16
FUNCTION(NvBootUartJump)
        bx r0
.ltorg
.align 4
FUNCTION(NvBootUartSetupStack)
        bx  pc
.code 32        
        // Move stack to end of IRAM to accomodate larger payloads.
        // We are downloading the UART payload to start of the
        // BCT address at 0x40000100.
        // We don't preserve what was in the old stack.
        ldr r13, =(NVBOOT_UART_IRAM_BLDR_STACK)
        bx r14
.ltorg

