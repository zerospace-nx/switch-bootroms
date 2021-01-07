/*
 * Copyright (c) 2007 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_reader_int.h - Definition of object reading interface.
 */

#ifndef INCLUDED_NVBOOT_READER_INT_H
#define INCLUDED_NVBOOT_READER_INT_H

#include "nvboot_badblocks.h"
#include "nvboot_bct.h"
#include "nvboot_config.h"
#include "nvboot_context_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootReaderObjDesc: Description of an object to load.
 */
typedef struct NvBootReaderObjDescRec
{
    NvS32       BlIndex; /* -1 for BCTs, 0 to 3 for BLs */
    NvU32       StartBlock;
    NvU32       StartPage;
    NvU32       Length; /* in bytes */
    NvU32       SignatureOffset; /* offset to the start of the signed
                                  * section of the object */
    NvBootObjectSignature *SignatureRef;
} NvBootReaderObjDesc;

/*
 * NvBootReadOneObject(): Top-level entry point for object reading code.
 */
NvBootError
NvBootReadOneObject(NvBootContext       *Context,
                    NvU8                *ReadDst,
                    NvBootReaderObjDesc *ObjDesc,
                    NvU32                NumCopies,
                    NvBootBadBlockTable *BadBlockTable);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_READER_INT_H */
