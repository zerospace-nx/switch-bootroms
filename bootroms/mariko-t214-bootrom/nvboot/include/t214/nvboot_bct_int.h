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
 * nvboot_bct_int.h - Definition of the Boot Configuration Table.
 * the device interface.
 */

#ifndef INCLUDED_NVBOOT_BCT_INT_H
#define INCLUDED_NVBOOT_BCT_INT_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvboot_bct.h"

/* Forward declarations */
typedef struct NvBootContextRec NvBootContext;

NvBootError NvBootReadBct(NvBootContext *Context);

/**
 * Process BCT values/settings after BCT is validated.
 */
NvBootError NvBootProcessBct(NvBootConfigTable *Bct);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BCT_INT_H */
