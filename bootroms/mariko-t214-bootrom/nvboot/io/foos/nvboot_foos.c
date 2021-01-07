/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "nvboot_bit.h"
#include "nvboot_config.h"
#include "nvboot_util_int.h"
#include "nvboot_foos_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_device_int.h"
#include "nvboot_dispatcher_int.h"
#include "nvboot_arc_int.h"
#include "project.h"

/** Foos is a dummy device which allows to test the entire bootrom flow on fpga or QT.
 * 
 * 1. To include foos device, Set NVENABLE_FOOS_SUPPORT=1 in bootrom/nvboot/make/Makefile.defs
 * 2. Set fuses to foos device (0x5). 
 *    Eg. for uartmon app, set RESERVED SW in fusebyp.c as follows
 *    FAB_ENTRY(RESERVED_SW, RESERVED_SW, 0x0D)
 * 3. Generate and load BCT to BTCM/SysRAM  /// fix the location via memory layout (0x40020000). 
 *    Note: Bct page and block size should match 
 *    Foos Page Size: 512 bytes and Block Size: 8192 bytes
 *    to avoid bct validation error.
 */

static NvBootFoosContext *foosContext;
static NvBootFoosParams s_FoosDefaultParams;

void
NvBootFoosGetParams(
    const NvU32 ParamIndex __attribute__ ((unused)),
    NvBootFoosParams **Params)
{
    s_FoosDefaultParams.PageSizeLog2  = FOOS_PAGESIZELOG2;
    s_FoosDefaultParams.BlockSizeLog2  = FOOS_BLOCKSIZELOG2;
    *Params = &s_FoosDefaultParams;
}

NvBool
NvBootFoosValidateParams(
    const NvBootFoosParams *Params __attribute__ ((unused)))
{
    return NV_TRUE;
}

void
NvBootFoosGetBlockSizes(
    const NvBootFoosParams *Params __attribute__ ((unused)),
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    *BlockSizeLog2 = foosContext->BlockSizeLog2;
    *PageSizeLog2  = foosContext->PageSizeLog2;
}

NvBootError
NvBootFoosInit(NvBootFoosContext *pFoosContext)
{
    NV_ASSERT(pFoosContext);

    foosContext = pFoosContext;

    foosContext->PageSizeLog2  = FOOS_PAGESIZELOG2;
    foosContext->BlockSizeLog2  = FOOS_BLOCKSIZELOG2;
    NvBootArcEnable();
    return NvBootError_Success;
}

NvBootError NvBootFoosReadPage(const NvU32 Block, const NvU32 Page, const NvU32 Len, uint8_t *Dest)
{
    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < (1 << (FOOS_BLOCKSIZELOG2 - foosContext->PageSizeLog2)));

    NvU32 LenCeiled = ((Len+0x0200-1)/0x200)*0x200;
    NvBootUtilMemcpy(Dest,
                     (uint8_t*)(FOOS_DATA_LOCATION +
                               (Block << foosContext->BlockSizeLog2) +
                               (Page << foosContext->PageSizeLog2)),
                     LenCeiled);// read only as per Len
    return NvBootError_Success;
}

NvBootDeviceStatus NvBootFoosQueryStatus(void)
{
    return NvBootDeviceStatus_Idle;
}
void NvBootFoosShutdown()
{
    foosContext->PageSizeLog2  = 0;
    foosContext->BlockSizeLog2  = 0;
}

NvBootError NvBootFoosGetReaderBuffersBase(
                            uint8_t** ReaderBuffersBase __attribute__ ((unused)),
                            const NvU32 Alignment __attribute__ ((unused)),
                            const NvU32 Bytes __attribute__ ((unused)))
{
    return NvBootError_Unimplemented;
}

NvBootError NvBootFoosWritePage(const NvU32 Block __attribute__ ((unused)),
                                const NvU32 Page __attribute__ ((unused)),
                                uint8_t *Dest __attribute__ ((unused)))
{
    return NvBootError_Unimplemented;
}

NvBootError NvBootFoosPinMuxInit(const NvBootFoosParams *Params __attribute__ ((unused)))
{
    return NvBootError_Success;
}

