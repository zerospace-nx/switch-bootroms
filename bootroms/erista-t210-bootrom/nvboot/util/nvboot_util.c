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
 * nvboot_util.c - Utility functions used in bootrom
 */

#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include "project.h"

// WORKAROUND FOR MISSING CONSTANTS
#ifndef NV_RT_SIM_TERMINATION_FAIL
#define NV_RT_SIM_TERMINATION_FAIL ~(15584169)
#endif

/** NvBootUtilMemset - set a region of memory to a value.
 *
 *  @param s Pointer to the memory region
 *  @param c The value to be set in the memory
 *  @param size The length of the region
 */
void
NvBootUtilMemset( void *s, NvU32 c, size_t size )
{
    memset( s, c, size );
}

/** NvBootUtilMemcpy - copy memory.from source location to the destination location
 *
 *  @param dest pointer to the destination for copy
 *  @param src pointer to the source memory
 *  @param size The length of the copy
 */
void * 
NvBootUtilMemcpy( void *dest, const void *src, size_t size )
{
    return memcpy( dest, src, size );
}

// this code is mapped in the non secure part of the ROM
#ifdef __arm__
// NOTE: This pragma only makes sense when building for the ARM compilers.
#pragma arm section rodata = "UtilNonsecure", code = "UtilNonsecure" 
#endif

/*
 * Simulation status reporting support
 */

#if NVBOOT_TARGET_RTL

// Semihosting SWIs for Arm and Thumb modes
#ifndef __thumb
unsigned __swi(0x123456) 
int NvBootUtilSwiSemi(int, int, int, int);
#else // __thumb
unsigned __swi(0xab) 
int NvBootUtilSwiSemi(int, int, int, int);
#endif // __thumb

#else // NVBOOT_TARGET_RTL

// store status information in last two words of iRAM
#define NVBOOT_UTIL_SIM_STATUS_ADDRESS \
        (NV_ADDRESS_MAP_DATAMEM_IRAM_D_LIMIT + 1 - 2*sizeof(NvU32))

#endif // NVBOOT_TARGET_RTL

void
NvBootUtilTerminateSim(NvBootUtilSimStatus status, NvU32 arg)
{
    volatile NvU32 i = 1;
    
#if NVBOOT_TARGET_RTL
    
    NvU32 RtlStatusCode = NV_RT_SIM_TERMINATION_FAIL;
    
    if ( status == NvBootUtilSimStatus_Pass )
        RtlStatusCode = NV_RT_SIM_TERMINATION_PASS;
    
    // magic numbers come from Section A.3.2 of the "RealView Compilation Tools
    // Developer Guide" Version 3.1, DUI0203H
    
    // the following arguments end up in r0, r1, r2, and r3, respectively
    //
    // unfortunately, the SWI semihosting call only supports up to four
    // input arguments.  The first three are required by the RTL environment.
    // Since "status" is arguably more important than arg, arg is dropped.
    NvBootUtilSwiSemi(0x18, 0x20026, RtlStatusCode, status);

#else // NVBOOT_TARGET_RTL
    
    // write status info to the magic addresses where other tools expect
    // to find it
    NV_WRITE32(NVBOOT_UTIL_SIM_STATUS_ADDRESS, (NvU32) (status)); 
    NV_WRITE32(NVBOOT_UTIL_SIM_STATUS_ADDRESS+4, (NvU32) (arg));
    
#endif // NVBOOT_TARGET_RTL
    
    // spin forever
    while (i)
        ;
}

NvU32 NvBootUtilGetTimeUS( void )
{
    // Should we take care of roll over of us counter? roll over happens after 71.58 minutes.
    NvU32 usec;
    usec = *(volatile NvU32 *)(NV_ADDRESS_MAP_TMRUS_BASE);
    return usec;
}

NvU32 NvBootUtilElapsedTimeUS(NvU32 StartTime) {
    // doing a diff and ignoring the overflow gets you the correct value 
    // even at wrararound, e.g. 
    // StartTime   = 0xFFFFFFFF
    // CurrentTime = 0x00000000
    // Current - Start = 1 + overflow flag (ignored in C)
    // this would *NOT* work if the counter was not 32 bits
    return  NvBootUtilGetTimeUS() - StartTime ;
}

void NvBootUtilWaitUS( NvU32 usec )
{
    NvU32 t0;
    NvU32 t1;

    t0 = NvBootUtilGetTimeUS();
    t1 = t0;

    // Use the difference for the comparison to be wraparound safe
    while( (t1 - t0) <= usec )
    {
        t1 = NvBootUtilGetTimeUS();
    }
}

// Return to regular section
#ifdef __arm__
// NOTE: This pragma only makes sense when building for the ARM compilers.
#pragma arm section
#endif


NvU32 NvBootUtilGetLog2Number(NvU32 size)
{
    NvU32 ReturnValue  = 0;

    while (1)
    {
        if ( (size == 1) || (size == 0) )
            break;
        size = size >> 1;
        ReturnValue++;
    }

    return ReturnValue;
}

NvU32
NvBootUtilSwapBytesInNvU32(const NvU32 Value)
{
    NvU32 tmp = (Value << 16) | (Value >> 16); /* Swap halves */
    /* Swap bytes pairwise */
    tmp = ((tmp >> 8) & 0x00ff00ff) | ((tmp & 0x00ff00ff) << 8);
    return (tmp);
}

NvBool
NvBootUtilIsValidPadding(NvU8 *Padding, NvU32 Length)
{
    NvU8 RefVal = 0x80;

    while (Length > 0)
    {
        if (*Padding != RefVal) return NV_FALSE;
        Padding++;
        Length--;
        RefVal = 0x00;
    }

    return NV_TRUE;
}

NvBool NvBootUtilCompareBytes(NvU8 *Value1, NvU8 *Value2, NvU32 ValueSizeBytes) 
{
    NvU32 i;

    for(i = 0; i < ValueSizeBytes; i++)
    {
        if(*Value1++ != *Value2++) 
        {
            return NV_FALSE;
        }

    }
    return NV_TRUE;
}

