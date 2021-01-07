/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_snor_param.h"
#include "nvboot_snor_context.h"
#include "nvboot_snor_int.h"

/*
 * Public Function Definitions.
 */
void
NvBootSnorGetParams(
    const NvU32 ParamIndex,
    NvBootSnorParams **Params)
{
 
    NV_ASSERT(0);

}

NvBool
NvBootSnorValidateParams(
    const NvBootSnorParams *Params)
{

    NV_ASSERT(0);
    return 0;
}

void
NvBootSnorGetBlockSizes(
    const NvBootSnorParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(0);
}

NvBootError
NvBootSnorInit(
    const NvBootSnorParams *Params,
    NvBootSnorContext *pNorContext)
{
    
    NV_ASSERT(0);
    return NvBootError_Success;                                            
}



NvBootError NvBootSnorReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest)
{
    NV_ASSERT(0);
    return NvBootError_Success;
}

NvBootDeviceStatus NvBootSnorQueryStatus(void)
{
    return NvBootDeviceStatus_Idle;
}

void NvBootSnorShutdown(void)
{
    NV_ASSERT(0);
}

NvBootError NvBootSnorGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
