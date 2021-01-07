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
 * nvboot_mux_one_nand_context.h - Definitions for the mux one nand context
 * structure.
 */

#ifndef INCLUDED_NVBOOT_MUX_ONE_NAND_CONTEXT_H
#define INCLUDED_NVBOOT_MUX_ONE_NAND_CONTEXT_H

#include "nvboot_mux_one_nand_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootMuxOneNandContext - The context structure for the muxone nand driver.
 * A pointer to this structure is passed into the driver's Init() routine.
 * This pointer has the only data that can be kept in a global variable for
 * later reference.
 */
typedef struct NvBootMuxOneNandContextRec
{
    NvBool IsNonFlexTypeDevice;
    
    NvBool IsNonMuxedInterface;

    // The state of the data transfer.
    NvBool IsFlashLoading;
    
    /// Block size in Log2 scale.
    NvU8 BlockSizeLog2;
    
    /// Page size in Log2 scale.
    NvU8 PageSizeLog2;
    
    // Read start time.
    NvU32 ReadStartTime;
    
    // Read Buffer
    NvU8 *pReadBuffer;
    
    // The size of the page in the Bytes. 
    NvU32 PageSizeInBytes;
} NvBootMuxOneNandContext;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MUX_ONE_NAND_CONTEXT_H */
