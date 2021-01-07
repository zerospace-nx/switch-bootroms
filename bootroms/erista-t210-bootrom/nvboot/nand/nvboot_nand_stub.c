/*
 * Copyright (c) 2008 - 2012 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_nand_local.h"
#include "nvboot_util_int.h"
#include "nvboot_nand_context.h"
#include "nvboot_nand_int.h"

//This stub driver is only linked into DEBUG build. It should not be
//called. If control ever reaches any of the exported APIs, NV_ASSERT it.
static void stub_assert()
{
    NV_ASSERT(0);
}

NvBootError 
NvBootNandInit(
    const NvBootNandParams *Params, 
    NvBootNandContext *Context)
{
    stub_assert();
    return NvBootError_None;
}

void 
NvBootNandGetParams(
    const NvU32 ParamIndex, 
    NvBootNandParams **Params)
{
    stub_assert();
}

NvBool 
NvBootNandValidateParams(
    const NvBootNandParams *Params)
{
    stub_assert();
    return NV_TRUE;
}

void 
NvBootNandGetBlockSizes(
    const NvBootNandParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    stub_assert();
}

NvBootError 
NvBootNandReadPage(
    const NvU32 Block, 
    const NvU32 Page, 
    NvU8 *pBuffer)
{
    stub_assert();
    return NvBootError_None;
}

NvBootDeviceStatus NvBootNandQueryStatus(void)
{
    stub_assert();
    return NvBootDeviceStatus_None;
}

void NvBootNandShutdown(void)
{
    stub_assert();
}
