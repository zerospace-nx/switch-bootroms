 /***************************************************************************\
|*                                                                           *|
|*       Copyright 1993-1998 NVIDIA, Corporation.  All rights reserved.      *|
|*                                                                           *|
|*     NOTICE TO USER:   The source code  is copyrighted under  U.S. and     *|
|*     international laws.  Users and possessors of this source code are     *|
|*     hereby granted a nonexclusive,  royalty-free copyright license to     *|
|*     use this code in individual and commercial software.                  *|
|*                                                                           *|
|*     Any use of this source code must include,  in the user documenta-     *|
|*     tion and  internal comments to the code,  notices to the end user     *|
|*     as follows:                                                           *|
|*                                                                           *|
|*       Copyright 1993-1998 NVIDIA, Corporation.  All rights reserved.      *|
|*                                                                           *|
|*     NVIDIA, CORPORATION MAKES NO REPRESENTATION ABOUT THE SUITABILITY     *|
|*     OF  THIS SOURCE  CODE  FOR ANY PURPOSE.  IT IS  PROVIDED  "AS IS"     *|
|*     WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.  NVIDIA, CORPOR-     *|
|*     ATION DISCLAIMS ALL WARRANTIES  WITH REGARD  TO THIS SOURCE CODE,     *|
|*     INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGE-     *|
|*     MENT,  AND FITNESS  FOR A PARTICULAR PURPOSE.   IN NO EVENT SHALL     *|
|*     NVIDIA, CORPORATION  BE LIABLE FOR ANY SPECIAL,  INDIRECT,  INCI-     *|
|*     DENTAL, OR CONSEQUENTIAL DAMAGES,  OR ANY DAMAGES  WHATSOEVER RE-     *|
|*     SULTING FROM LOSS OF USE,  DATA OR PROFITS,  WHETHER IN AN ACTION     *|
|*     OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  ARISING OUT OF     *|
|*     OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOURCE CODE.     *|
|*                                                                           *|
|*     U.S. Government  End  Users.   This source code  is a "commercial     *|
|*     item,"  as that  term is  defined at  48 C.F.R. 2.101 (OCT 1995),     *|
|*     consisting  of "commercial  computer  software"  and  "commercial     *|
|*     computer  software  documentation,"  as such  terms  are  used in     *|
|*     48 C.F.R. 12.212 (SEPT 1995)  and is provided to the U.S. Govern-     *|
|*     ment only as  a commercial end item.   Consistent with  48 C.F.R.     *|
|*     12.212 and  48 C.F.R. 227.7202-1 through  227.7202-4 (JUNE 1995),     *|
|*     all U.S. Government End Users  acquire the source code  with only     *|
|*     those rights set forth herein.                                        *|
|*                                                                           *|
 \***************************************************************************/

/** NOTE: nvcommon.h is exactly the same as nvtypes.h
 *  Temporarily keep both until we transition everything to nvtypes.h
 *  Keeping NVTYPES_INCLUDED define the same as in nvtypes.h to avoid
 *  redefinition errors.
 */
#ifndef NVTYPES_INCLUDED
#define NVTYPES_INCLUDED

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// XXX Deprecated sized types.  Use stdint.h types in new code.
typedef uint32_t           NvV32; // "void": enumerated or multiple fields
typedef uint8_t            NvU8;  // 0 to 255
typedef uint16_t           NvU16; // 0 to 65535
typedef uint32_t           NvU32; // 0 to 4294967295
typedef unsigned long long NvU64; // 0 to 18446744073709551615
typedef int8_t             NvS8;  // -128 to 127
typedef int16_t            NvS16; // -32768 to 32767
typedef int32_t            NvS32; // -2147483648 to 2147483647
typedef          long long NvS64; // 2^-63 to 2^63-1

// XXX Deprecated sized types.  Use stdint.h types in new code.
#ifdef __x86_64__
typedef uint64_t NVR_U64;
#endif

// XXX Deprecated boolean types.  Use C++ "bool" or C stdbool.h in new code.
enum { NV_FALSE = 0, NV_TRUE = 1 };
typedef NvU8 NvBool;
typedef int BOOL;

// Macros to extract the low and high parts of a 64-bit unsigned integer
// Also designed to work if someone happens to pass in a 32-bit integer
#define NvU64_HI32(n) ((NvU32)(((NvU64)(n)) >> 32))
#define NvU64_LO32(n) ((NvU32)((NvU64)(n)))

// Align a variable declaration to a particular # of bytes (should always be a power of two)
#if defined(__GNUC__)
#define NV_ALIGN(size) __attribute__ ((aligned (size)))
#elif defined(__arm)
#define NV_ALIGN(size) __align(size)
#elif defined(_WIN32)
#define NV_ALIGN(size) __declspec(align(size))
#else
#error Unknown compiler
#endif

// Macro for determining the size of an array
#define NV_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

// Macro for taking min or max of a pair of numbers
#ifndef NV_MIN
#define NV_MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef NV_MAX
#define NV_MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

// Macro to append ULL to 64-bit constants to avoid gcc compilation error 
#define NV_ULLC(c)         (c##ULL)
#define NV_U64C(c)        (NV_ULLC(c))

#ifdef __cplusplus
}
#endif

#endif // NVTYPES_INCLUDED
