/*
 * Copyright (c) 2011 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "nvboot_bit.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_foos_int.h"
#include "nvboot_device_int.h"
/** Foos is a dummy device which allows to test the entire bootrom flow on fpga or QT.
 * 
 * 1. To include foos device, Set NVBOOT_FOOS_ENABLE=1 in bootrom/make/Makefile.localdefs
 * 2. Set fuses to foos device (0x5). 
 *    Eg. for uartmon app, set RESERVED SW in fusebyp.c as follows
 *    FAB_ENTRY(RESERVED_SW, RESERVED_SW, 0x0D)
 * 3. Generate and load BCT to IRAM_C (0x40020000). 
 *    Note: Bct page and block size should match 
 *    Foos Page Size: 512 bytes and Block Size: 8192 bytes
 *    to avoid bct validation error.
 */

static NvBootFoosContext s_FoosContext;
static NvBootFoosParams s_FoosDefaultParams;
void
NvBootFoosGetParams(
    const NvU32 ParamIndex,
    NvBootFoosParams **Params)
{
    s_FoosDefaultParams.PageSizeLog2  = FOOS_PAGESIZELOG2;
    s_FoosDefaultParams.BlockSizeLog2  = FOOS_BLOCKSIZELOG2;
    *Params = &s_FoosDefaultParams;
}

NvBool
NvBootFoosValidateParams(
    const NvBootFoosParams *Params)
{
    return NV_TRUE;
}

void
NvBootFoosGetBlockSizes(
    const NvBootFoosParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    *BlockSizeLog2 = Params->BlockSizeLog2;
    *PageSizeLog2  = Params->PageSizeLog2;
}

NvBootError
NvBootFoosInit(
    const NvBootSnorParams *Params,
    NvBootFoosContext *pFoosContext)
{
    s_FoosContext.PageSizeLog2  = FOOS_PAGESIZELOG2;
    s_FoosContext.BlockSizeLog2  = FOOS_BLOCKSIZELOG2;
    return NvBootError_Success;
}

NvBootError NvBootFoosReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest)
{
    const NvU32 NumBytesToRead = (1 << s_FoosContext.PageSizeLog2);
    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < (1 << (FOOS_BLOCKSIZELOG2 - s_FoosContext.PageSizeLog2)));

    memcpy(Dest, (NvU8*)(FOOS_DATA_LOCATION + (Block <<
    s_FoosContext.BlockSizeLog2) + (Page <<s_FoosContext.PageSizeLog2)), NumBytesToRead);
    return NvBootError_Success;
}

NvBootDeviceStatus NvBootFoosQueryStatus(void)
{
    return NvBootDeviceStatus_Idle;
}
void NvBootFoosShutdown()
{
}

NvBootError NvBootFoosGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
