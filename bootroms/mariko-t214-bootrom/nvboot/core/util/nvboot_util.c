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
#include "project.h"
#include "nvboot_section_defs.h"
#include "nvboot_util_int.h"
#include "nvboot_dispatcher_int.h"

//
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

/*
 * Simulation status reporting support
 */
// store status information in last two words of iRAM
#define NVBOOT_UTIL_SIM_STATUS_ADDRESS \
        (NV_ADDRESS_MAP_DATAMEM_IRAM_D_LIMIT + 1 - 2*sizeof(NvU32))


NvU32 FT_NONSECURE NvBootUtilGetTimeUS( void )
{
    // Should we take care of roll over of us counter? roll over happens after 71.58 minutes.
    NvU32 usec;
    usec = *(volatile NvU32 *)(NV_ADDRESS_MAP_TMRUS_BASE);
    return usec;
}

NvU32 FT_NONSECURE NvBootUtilElapsedTimeUS(NvU32 StartTime) {
    // doing a diff and ignoring the overflow gets you the correct value 
    // even at wrararound, e.g. 
    // StartTime   = 0xFFFFFFFF
    // CurrentTime = 0x00000000
    // Current - Start = 1 + overflow flag (ignored in C)
    // this would *NOT* work if the counter was not 32 bits
    return  NvBootUtilGetTimeUS() - StartTime ;
}

void FT_NONSECURE NvBootUtilWaitUS( NvU32 usec )
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

int8_t
NvBootUtilCmpBigUnsignedInt(NvU8 *Value1, NvU8 *Value2, NvU32 ValueSizeBytes)
{
    NvU32 i;

    if(ValueSizeBytes == 0)
        return NVBOOT_BIGINT_INPUT_ERROR;

    // Can't do for(i = ValueSizeBytes - 1; i >= 0; i--)
    // since i is always >=0 for unsigned integer.
    // Found via compiler warning.
    for(i = ValueSizeBytes; i > 0; i--)
    {
        if(Value1[i-1] > Value2[i-1])
        {
            return NVBOOT_BIGINT_GT;
        }
        else if (Value1[i-1] < Value2[i-1])
        {
            return NVBOOT_BIGINT_LT;
        }
    }

    return NVBOOT_BIGINT_EQ;
}

int8_t
NvBootUtilCmpBigUnsignedIntIsZero(NvU8 *Value1, NvU32 ValueSizeBytes)
{
    NvU32 i;

    if(ValueSizeBytes == 0)
        return NVBOOT_BIGINT_INPUT_ERROR;

    for(i = 0; i < ValueSizeBytes; i++)
    {
        if(Value1[i] != 0)
        {
            return NVBOOT_BIGINT_NONZERO;
        }
    }
    return NVBOOT_BIGINT_ZERO;
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

FI_bool __attribute__((optimize("O0"))) NvBootUtilCompareConstTimeFI(const void *Buffer1, const void *Buffer2, size_t length)
{
    const uint8_t *Buf1 = (const uint8_t *) Buffer1;
    const uint8_t *Buf2 = (const uint8_t *) Buffer2;

#if 1
    NV_WRITE32(0x7000e400 + 0x2f0, (uint32_t) Buf2);
    NV_WRITE32(0x7000e400 + 0x2f0, ((uint32_t) Buf1) | (0x80 << 24));
    NV_WRITE32(0x7000e400 + 0x2f4, (uint32_t) length);
    NV_WRITE32(0x7000e400 + 0x2f4, 0x5A595043); /* CPYZ */
#endif    
    // length = 0 should not return true, in case a malicious actor can
    // manipulate the input parameter prior to the function call.
    if(length == 0)
        return FI_FALSE;

    uint8_t result = 0;
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        result |= Buf1[i] ^ Buf2[i];
    }
    // FI enhancement. Double check that loop actually looped the proper number
    // of times. Requires optimization level O0 or else compilier will optimize it out.
    if(i != length)
        return FI_FALSE;

    // If result is 0, Buffer1 and Buffer 2 are identical, return true.
    // Otherwise, return false.
    return (result == 0) ? FI_TRUE : FI_FALSE;
}

bool NvBootUtilCompareConstTime(const void *Buffer1, const void *Buffer2, size_t length)
{
    const uint8_t *Buf1 = (const uint8_t *) Buffer1;
    const uint8_t *Buf2 = (const uint8_t *) Buffer2;

    // length = 0 should not return true, in case a malicious actor can
    // manipulate the input parameter prior to the function call.
    if(length == 0)
        return false;

    uint8_t result = 0;

    for (uint32_t i = 0; i < length; i++)
    {
        result |= Buf1[i] ^ Buf2[i];
    }

    // If result is 0, Buffer1 and Buffer 2 are identical, return true.
    // Otherwise, return false.
    return (result == 0) ? true : false;
}

/**
 *  Poll a register for given value for given mask within timeout(us)
 *  Input: Reg Address, Mask, Expected Value, Timeout in us
 *  Output: NvBootError_HwTimeOut or NvBootError_Success
 */
NvBootError NvBootPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout)
{
    NvU32 RegData;
    do {
    RegData = NV_READ32(RegAddr);
    if((RegData & Mask) == ExpectedValue)
        return NvBootError_Success;
    NvBootUtilWaitUS(1);
    Timeout--;
    } while(Timeout);
    return NvBootError_HwTimeOut;
}

/**
 * Force e to be initialized to a non-success value, such that the skipping of a senstive
 * section of the code will default the action to error.
 * Tied to timer which uses a volatile declaration to avoid compiler optimization.
 * Added optimize attribute to avoid the if statement getting optimized out (since
 * Initial_e can not be zero if there is no tampering).
 */
NvBootError __attribute__((optimize("O0"))) NvBootInitializeNvBootError()
{
    NvBootError Initial_e;
    Initial_e = NvBootError_Initial_Value | NvBootUtilGetTimeUS();
    if(Initial_e != NvBootError_Success)
        return Initial_e;
    else
        return NvBootError_Initial_Value;
}


/**
 *  @brief Delay loop to introduce random delays during BR execution
 *  @param loops Number of loop execution
 *  @return NvBootError
 */

NvBootError  NvBootUtilInstrWait(const NvU32 loops)
{
    uint32_t j;
    volatile uint32_t i;
    NvBootError e = NvBootError_Fault_Injection_Detection;

    for(i = 0, j = loops; i < loops; i++, j--)
    {
        if( (i + j) != loops )
            break;
    }

    if(i == loops)
        e = NvBootError_Success;

    return e;
}

