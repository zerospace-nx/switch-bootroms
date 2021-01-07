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
 * nvboot_bootloader_int.h - Definition of the reader code interfaces.  These
 * are used to read BCTs and BLs.
 */

#ifndef INCLUDED_NVBOOT_BOOTLOADER_INT_H
#define INCLUDED_NVBOOT_BOOTLOADER_INT_H

#include "nvboot_bct.h"
#include "nvboot_config.h"
#include "nvboot_context_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/* Proper names to be determined later... */
/* This enum is very similar to NvBootDeviceStatus... */
typedef enum
{
    NvBlReaderStatus_None = 0,
    NvBlReaderStatus_Idle,
    NvBlReaderStatus_Busy,
    NvBlReaderStatus_Done,
    NvBlReaderStatus_RecoverableError,
    NvBlReaderStatus_IrrecoverableError,
    NvBlReaderStatus_QueryError,
    NvBlReaderStatus_Force32 = 0x7fffffff
} NvBlReaderStatus;

/*
 * NvBlReaderState: Internal state of the the boot loader reader.
 * The contents of this structure are hidden from the higher-level code
 * that invokes the BlReader - they just see a handle to an
 * instance of this structure.
 */
typedef struct NvBlReaderStateRec
{
    NvBlReaderStatus CurrentState;
    NvU32 BytesRead;
    NvU32 CurrentBlock; /* Physical block */
    NvU32 CurrentPage;  /* Physical page */

    /* More TBD */
} NvBlReader;

/* 
 * NvBlReaderInit(): Initialize the boot loader reader.
 */
NvBootError
NvBlReaderInit(const NvBootLoaderInfo *BootLoader, NvBlReader *Reader);

NvBootError
NvBlReaderReadChunk(NvBlReader *Reader, NvU8 *Buffer, const NvU8 BufferSize);

NvBlReaderStatus
NvBlReaderQueryStatus(const NvBlReader *Reader, NvU32 *BytesRead);

/**
 * Check the version numbers in the NvBootLoaderInfo.Version and
 * NvBootOemBootBinaryHeader.Version fields.
 *
 * If NvBootLoaderInfo.Version is 0, then version binding is
 * disabled.
 * If NvBootLoaderInfo.Version is nonzero then NvBootLoader.Version
 * must equal NvBootOemBootBinaryHeader.Version.
 *
 */
NvBootError
NvBootBootLoaderCheckVersionBinding(NvBootLoaderInfo *BootLoaderInfo, NvBootOemBootBinaryHeader *OemBootBinaryHeader);

/*
 * NvBootLoadBootLoader(): Top-level entry point for boot loader reading
 * code.
 */
NvBootError
// TODO NvBootLoadBootLoader(NvBootContext *Context, NvU32 *BootRomBranchTarget);
NvBootLoadBootLoader(NvBootContext *Context);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BOOTLOADER_INT_H */
