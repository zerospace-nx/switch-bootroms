/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_util_int.h - Definition of utility functions used in the bootrom
 */

#ifndef INCLUDED_NVBOOT_UTIL_INT_H
#define INCLUDED_NVBOOT_UTIL_INT_H

#include "nvtypes.h"
#include "stddef.h"
#include <stdbool.h>
#include "nvboot_error.h"
#include <nvboot_section_defs.h>

#if defined(__cplusplus)
extern "C"
{
#endif

/** NvBootUtilMemset - set a region of memory to a value.
 *
 *  @param s Pointer to the memory region
 *  @param c The value to be set in the memory
 *  @param size The length of the region
 */
void
NvBootUtilMemset( void *s, NvU32 c, size_t size );

/** NvBootUtilMemcpy - copy memory.from source location to the destination
 *  location
 *
 *  @param dest pointer to the destination for copy
 *  @param src pointer to the source memory
 *  @param size The length of the copy
 */
void * 
NvBootUtilMemcpy( void *dest, const void *src, size_t size );

/** NvBootUtilGetTimeUS - returns the system time in microseconds.
 *
 *  The returned values are guaranteed to be monotonically increasing,
 *  but may wrap back to zero.
 *
 *  Some systems cannot gauantee a microsecond resolution timer.
 *  Even though the time returned is in microseconds, it is not gaurnateed
 *  to have micro-second resolution.
 *
 *  Please be advised that this APIs is mainly used for code profiling and 
 *  meant to be used direclty in driver code.
 *
 */
NvU32 FT_NONSECURE NvBootUtilGetTimeUS( void );

/** NvBootUtilElapsedTimeUS() - Returns the elapsed time in microseconds
 * 
 * This function is wraparound safe, the retruned value will be the real
 * delay (modulo 2^32 microseconds, i.e. more than one hour) 
 *
 *  @param NvU32 StartTime The start point of the time interval to measure
 *  @return NvU32 Elapsed time in microseconds since StartTime
 */
NvU32 FT_NONSECURE NvBootUtilElapsedTimeUS( NvU32 StartTime );

/** NvBootWaitUS - stalls the calling thread for at least the given number of
 *      microseconds.  The actual time waited might be longer, you cannot
 *      depend on this function for precise timing.
 *
 * NOTE:  It is safe to use this function at ISR time.
 *
 *  @param usec The number of microseconds to wait
 */
void FT_NONSECURE NvBootUtilWaitUS( NvU32 usec );

/** NvBootUtilWait100NS_OSC - Macro that inserts a delay of at least 100ns.
 *  The macro expands into essentially a string of NOPs - reading from
 *  a volatile memory location.
 *
 *  The macro assumes a max AVP clock of 26MHz, which is the maximum
 *  oscillator frequency.  With 1 wait state per iROM read, this is
 *  one instruction dispatch each 76.9ns.  2 instructions provide the
 *  >100ns delay.  With slower oscillators, the delay is naturally longer.
 *
 *  NOTE: This timing is very approximate.
 */
#define NVBOOT_UTIL_WAIT_100NS_OSC()  \
    do                                \
    {                                 \
        volatile char a, c;           \
        volatile char *pc = &c;       \
        a = *pc;                      \
    } while(0)

/** NvBootGetLog2Number - Converrs the size in bytes to units of log2(bytes)
 * Thus, a 1KB block size is reported as 10.
 *
 *  @param usec The number of microseconds to wait
 */
NvU32 NvBootUtilGetLog2Number(NvU32 size);

/** NvBootUtilSwapBytesInNvU32 - set a region of memory to a value.
 *
 *  @param Value The constant NvU32 value in which the bytes need to be swaped
 *  @return Swaped value
 */
NvU32
NvBootUtilSwapBytesInNvU32( const NvU32 Value );

/** NvBootUtilIsValidPadding - checks if the padding is valid or not
 *  (padding is supposed to be 0x80 followed by all 0x00's)
 *
 *  @param Padding points to the padding buffer
 *  @param Length length of the padding to be validated.
 *  @return NV_TRUE if the padding is valid else NV_FALSE
 */
NvBool
NvBootUtilIsValidPadding(NvU8 *Padding, NvU32 Length);

/** NvBootUtilCompareBytes - compares two byte buffers for differences. 
 *
 *  @param Value1 pointer to first byte buffer
 *  @param Value2 pointer to second byte buffer
 *  @param ValueSizeBytes size of byte buffer to compare
 *  @return NV_TRUE if the byte buffers are identical else NV_FALSE
 */
NvBool
NvBootUtilCompareBytes(NvU8 *Value1, NvU8 *Value2, NvU32 ValueSizeBytes);

/**
 * NvBootUtilCompareConstTime - compares two buffers for differences,
 * but always compare every byte, irrespective of mismatches. Therefore,
 * this function shall have the same runtime for every call with the same length.
 * This fuction should be used as mitigation against timing attacks for
 * signature comparisons.
 * Note, timing attack isn't possible when doing public key signature verification
 * (no secret to recover) but we should still use this function anyway.
 * For symmetric based algorithms like AES-CMAC, definitely use this function.
 *
 * @param Buffer1 pointer to first byte buffer
 * @param Buffer2 pointer to second byte buffer
 * @param length length of the buffers to compare.
 * @return false if length == 0
 *         false if the values in Buffer1 mismatch with Buffer2
 *         true if the contents of Buffer1 are identical to Buffer2
 */
bool
NvBootUtilCompareConstTime(const void *Buffer1, const void *Buffer2, size_t length);

typedef enum
{
    FI_FALSE = 0x72FBB604,
    FI_TRUE = 0x71B3E5A9,
} FI_bool;

/**
 * NvBootUtilCompareConstTime - compares two buffers for differences,
 * but always compare every byte, irrespective of mismatches. Therefore,
 * this function shall have the same runtime for every call with the same length.
 * This fuction should be used as mitigation against timing attacks for
 * signature comparisons.
 * Note, timing attack isn't possible when doing public key signature verification
 * (no secret to recover) but we should still use this function anyway.
 * For symmetric based algorithms like AES-CMAC, definitely use this function.
 *
 * @param Buffer1 pointer to first byte buffer
 * @param Buffer2 pointer to second byte buffer
 * @param length length of the buffers to compare.
 * @return if length == 0, return FI_FALSE. 
 *         return FI_TRUE if Buffer1 == Buffer2, FI_FALSE otherwise. A 32-bit return
 *         value is used as a fault injection mitigation.
 */
FI_bool
NvBootUtilCompareConstTimeFI(const void *Buffer1, const void *Buffer2, size_t length);

#define NVBOOT_BIGINT_LT -1
#define NVBOOT_BIGINT_EQ 0
#define NVBOOT_BIGINT_GT 1
#define NVBOOT_BIGINT_ZERO 0
#define NVBOOT_BIGINT_NONZERO 1
#define NVBOOT_BIGINT_INPUT_ERROR -2

/** NvBootUtilCmpBigUnSignedInt - Compare two large unsigned byte buffers.
 *
 *  @param Value1 pointer to first unsigned integer byte buffer.
 *  @param Value2 pointer to second unsigned integer byte buffer.
 *
 *  @return Returns an int8_t value. See above defines.
 */
int8_t
NvBootUtilCmpBigUnsignedInt(NvU8 *Value1, NvU8 *Value2, NvU32 ValueSizeBytes);

/** NvBootUtilCmpBigUnsignedIntIsZero - Check if a large unsigned integer is zero.
 *
 *  @param Value1 pointer to first unsigned integer byte buffer.
 *
 *  @return Returns an int8_t value. See above defines.
 */
int8_t
NvBootUtilCmpBigUnsignedIntIsZero(NvU8 *Value1, NvU32 ValueSizeBytes);

/**
 * NvBootUtilReverseNvU8 - Reverse the order of a byte stream
 *
 * @param Value pointer to the buffer to be reversed.
 * @param ValueSizeNvU8 size of byte buffer to reverse.
 *
 */
void
NvBootUtilReverseNvU8(NvU8 *Value, NvU32 ValueSizeNvU8);

/**
 * NvBootMainSecureExit - Secure exit function
 *
 * @param BootloaderEntryAddress pointer payload entry point
 * @param StartClearAddress address for start of btcm clearing
 * @param StopClearAddress address for end of btcm clearing
 * @param SecureRegisterValue secure flags lock down.
 *
 */
void 
NvBootMainSecureExit(
	NvU32 BootloaderEntryAddress, 
	NvU32 StartClearAddress,
	NvU32 StopClearAddress, 
	NvU32 SecureRegisterValue);

/**
 * Runtime termination of simulation/emulation.
 * Compiles out in release builds.
 */
    
/**
 * Termination status codes
 */
typedef enum
{
    /** an assert was encountered and failed */
    NvBootUtilSimStatus_Assert = 1,
    /** successful completion */
    NvBootUtilSimStatus_Pass = 2,
    /** unsuccessful completion */
    NvBootUtilSimStatus_Fail = 3,
    /** Recovery Mode was invoked; could not run to completion */
    NvBootUtilSimStatus_RecoveryMode = 4,
    NvBootUtilSimStatus_Num,
    NvBootUtilSimStatus_Force32 = 0x7FFFFFFF
} NvBootUtilSimStatus;
    
/** NvBootUtilTerminateSim - terminates simulation and reports
 *  exit status in an environment-specific manner
 *
 *  @param status exit status
 *  @param arg user-specified value, passed through to environment
 */
void NvBootUtilTerminateSim(NvBootUtilSimStatus status, NvU32 arg);

/**
 *  Poll a register for a given value after applying mask (for timeout value in us).
 *  Input: Reg Address, Mask, Expected Value, Timeout in us
 *  Output: NvBootError_HwTimeOut or NvBootError_Success
 */
NvBootError NvBootPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout);

/**
 *  @brief Delay loop to introduce random delays during BR execution
 *  @param loops Number of loop cycles
 *  @return NvBootError
 */

NvBootError  NvBootUtilInstrWait(const NvU32 loops);

#if NV_DEBUG || NVBOOT_TARGET_FPGA
#define NVBOOT_UTIL_TERMINATE_SIM(status, arg) \
    NvBootUtilTerminateSim(status, arg)
    
#else // NV_DEBUG || NVBOOT_TARGET_FPGA
#define NVBOOT_UTIL_TERMINATE_SIM(status, arg) \
    /* null statement */
#endif // NV_DEBUG || NVBOOT_TARGET_FPGA
    
/*
 * Assertion support (initially borrowed from nvassert.h, then stripped
 * down for BootRom use)
 */
    
#ifndef NV_ASSERT

/**
 * Runtime condition check with break into debugger if the assert fails.
 * Compiles out in release builds.
 */
#if NV_DEBUG

#define NV_ASSERT(x) \
    do { \
        if (!(x)) \
        { \
            /* break into the debugger */ \
            NVBOOT_UTIL_TERMINATE_SIM(NvBootUtilSimStatus_Assert, __LINE__); \
        } \
    } while( 0 )

#else // NV_DEBUG
#define NV_ASSERT(x) do {} while(0)
#endif // NV_DEBUG
#endif

#ifndef NV_CT_ASSERT
/** NV_CT_ASSERT: compile-time assert for constant values.

    This works by declaring a function with an array parameter.  If the
    assert condition is true, then the array size will be 1, otherwise
    the array size will be -1, which will generate a compilation error.

    No code should be generated by this macro.

    Three levels of macros are needed to properly expand the line number.

    This macro was taken in spirit from:
        //sw/main/drivers/common/inc/nvctassert.h
 */
#define NV_CT_ASSERT( x )            NV_CT_ASSERT_I( x, __LINE__ )
#define NV_CT_ASSERT_I( x,line )     NV_CT_ASSERT_II( x, line )
#define NV_CT_ASSERT_II( x, line ) \
    void compile_time_assertion_failed_in_line_##line( \
        int _compile_time_assertion_failed_in_line_##line[(x) ? 1 : -1])
#endif

/* Common functions */

/** Macro for taking min or max of a pair of numbers */
#define NV_MIN(a,b) (((a) < (b)) ? (a) : (b))
#define NV_MAX(a,b) (((a) > (b)) ? (a) : (b))

/*
 * Ceiling function macros
 * NV_ICEIL(a, b)      Returns the ceiling of a divided by b.
 * NV_ICEIL_LOG2(a, b) Returns the ceiling of a divided by 2^b.
 */
#define NV_ICEIL(a,b)      (((a) +       (b)  - 1) /  (b))
#define NV_ICEIL_LOG2(a,b) (((a) + (1 << (b)) - 1) >> (b))


#if NV_DEBUG

#define NV_BOOT_SPIN_WAIT()                   \
{                                             \
    /* Change i to break out of spin wait. */ \
    volatile int i = 0;                       \
    for (; i == 0; )                          \
        ;                                     \
}

#ifdef NV_BOOT_SPIN_WAIT_ON_ERROR

#if NV_BOOT_SPIN_WAIT_ON_ERROR
#define NV_BOOT_ERROR() NV_BOOT_SPIN_WAIT()
#else /* #if NV_BOOT_SPIN_WAIT_ON_ERROR */
#define NV_BOOT_ERROR()
#endif /* #if NV_BOOT_SPIN_WAIT_ON_ERROR */

#else /* #ifdef NV_BOOT_SPIN_WAIT_ON_ERROR */
#define NV_BOOT_ERROR()
#endif  /* #ifdef NV_BOOT_SPIN_WAIT_ON_ERROR */

#else /* #if NV_DEBUG */

#define NV_BOOT_SPIN_WAIT()
#define NV_BOOT_ERROR()

#endif /* #if NV_DEBUG */

/**
 * A helper macro to check a function's error return code and propagate any
 * errors upward.  This assumes that no cleanup is necessary in the event of
 * failure.  This macro does not locally define its own NvError variable out of
 * fear that this might burn too much stack space, particularly in debug builds
 * or with mediocre optimizing compilers.  The user of this macro is therefore
 * expected to provide their own local variable "NvError e;".
 */
#define NV_BOOT_CHECK_ERROR(expr) \
    do \
    { \
        e = (expr); \
        if (e != NvBootError_Success) \
            return e; \
    } while (0)

/**
 * A helper macro to check a function's error return code and, if an error
 * occurs, jump to a label where cleanup can take place.  Like NV_CHECK_ERROR,
 * this macro does not locally define its own NvError variable.  (Even if we
 * wanted it to, this one can't, because the code at the "fail" label probably
 * needs to do a "return e;" to propagate the error upwards.)
 */
#define NV_BOOT_CHECK_ERROR_CLEANUP(expr) \
    do \
    { \
        e = (expr); \
        if (e != NvBootError_Success) \
            goto fail; \
    } while (0)

/**
 * Helper macros to convert between 32-bit addresses and pointers.
 * Their primary function is to document when pointers are assumed to be
 * 32-bit numbers.
 *
 * NOTE: Be careful when using these macros.  If the casts are used in code
 *       that reads & writes the addresses to hardware registers, the pointers
 *       will usually need to contain physical, not virtual, addresses.
 */
#define PTR_TO_ADDR(x) ((NvU32)(x))
#define ADDR_TO_PTR(x) ((NvU8*)(x))

/**
 *  Align address to required boundary.
 */
#define ALIGN_ADDR(ADDR, BYTES) ((((ADDR)+(BYTES)-1)/(BYTES)) * (BYTES))

#define NvBootPmcUtilWaitUS(x) NvBootUtilWaitUS(x)

/**
 *  FI add code distance between detection and response
 *  FI mitigation technique
 */
 extern int32_t FI_IncrDist; // Dummy global to increase code distance between Error detection 
                             //and response.
 #define FI_ADD_DISTANCE_STEP(N)    FI_STEP_##N
 
 /**
  *  Adds 12 bytes of code distance
  */
 #define FI_STEP_1  \
 do \
 {  \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
 }while(0) \
 
 /**
  *  Adds 18 bytes of code distance
  */
 #define FI_STEP_2  \
 do \
 {  \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
 }while(0) \
 
 /**
  *  Adds 22 bytes of code distance
  */
 #define FI_STEP_3  \
 do \
 {  \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
 }while(0) \
 
 /**
  *  Adds 30 bytes of code distance
  */
 #define FI_STEP_4  \
 do \
 {  \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
    FI_IncrDist+=NV_READ32(&FI_IncrDist);   \
 }while(0) \
 
#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_UTIL_INT_H */
