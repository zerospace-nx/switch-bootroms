/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_SSK_INT_H
#define INCLUDED_NVBOOT_SSK_INT_H

#include "nvboot_config.h"
#include "nvboot_pmc_int.h"

/**
 * @file nvboot_ssk.h
 *
 * NvBootSsk interface for NvBOOT
 *
 * NvBootSsk is NVIDIA's interface for computing, loading, and storing the 
 * Secure Storage Key (SSK).
 *
 */


/**
 * Computes the SSK and stores the result in SE keyslot NvBootSeAesKeySlot_SSK.
 *
 * Note that write access to the this SE keyslot or PMC secure scratch register 
 * must be enabled for this routine to operate properly.
 *
 * Note also that this routine does not change the access permissions for the 
 * SE keyslot or PMC secure scratch register.  Access permissions will be the 
 * same upon exit as they were upon entry.
 *
 * @return None.
 */
void NvBootSskGenerate(void);

/**
 * Lock the SSK down. 
 *
 *      Set access permissions for SE keyslot reserved for SSK, 
 *      registers depending on boot type.  For WB0, both read- and write-access 
 *      Mode), only read access is disabled.
 *
 * @param IsWarmBoo0 Specifies if this is a WB0 cycle or not. 
 *
 * @return None.
 */
void NvBootSskLockSsk(
    NvBool IsWarmBoot0);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SSK_INT_H

