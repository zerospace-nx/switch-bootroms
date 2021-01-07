/*
 * Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_mobile_lba_nand_local.h"
#include "nvboot_util_int.h"
#include "nvboot_mobile_lba_nand_context.h"
#include "nvboot_mobile_lba_nand_int.h"

//This stub driver is only linked into DEBUG build. It should not be
//called. If control ever reaches any of the exported APIs, NV_ASSERT it.
static void stub_assert()
{
    NV_ASSERT(0);
}

NvBootError 
NvBootMobileLbaNandInit(
    const NvBootMobileLbaNandParams *Params, 
    NvBootMobileLbaNandContext *Context)
{
    stub_assert();
    return NvBootError_None;
}

void 
NvBootMobileLbaNandGetParams(
    const NvU32 ParamIndex, 
    NvBootMobileLbaNandParams **Params)
{
    stub_assert();
}

NvBool 
NvBootMobileLbaNandValidateParams(
    const NvBootMobileLbaNandParams *Params)
{
    stub_assert();
    return NV_TRUE;
}

void 
NvBootMobileLbaNandGetBlockSizes(
    const NvBootMobileLbaNandParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    stub_assert();
}

NvBootError 
NvBootMobileLbaNandReadPage(
    const NvU32 Block, 
    const NvU32 Page, 
    NvU8 *pBuffer)
{
    stub_assert();
    return NvBootError_None;
}

NvBootDeviceStatus NvBootMobileLbaNandQueryStatus(void)
{
    stub_assert();
    return NvBootDeviceStatus_None;
}

void NvBootMobileLbaNandShutdown(void)
{
    stub_assert();
}


