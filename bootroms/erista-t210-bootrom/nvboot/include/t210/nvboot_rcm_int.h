/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_RCM_INT_H
#define INCLUDED_NVBOOT_RCM_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"
#include "nvboot_rcm.h"
#include "nvboot_fuse.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * NvBootRcm() Main routine for recovery mode.
 *
 * @param IsForced Indicates if entry to RCM was forced, as opposed to a
 * natural consequence of error condition(s).
 * @param *IsFactorySecureProvosioning is a ptr to an NvBool flag indicating
 * whether or not factory secure provisioning is enabled.
 * @param *RcmSecureDebugControl A pointer to a location to store value of
 * APBDEV_PMC_DEBUG_AUTHENTICATION_0 to be set at BR exit.
 * @param RcmSecureJtagEnable Holds JTAG enable/disable decision as dictated
 * by the NvBootRcmOpcode_EnableJtag command.
 * @param EntryPoint Holds the address of the entry point for downloaded code.
 *
 * @retval NvBootError_Success Successfully downloaded an applet.
 * (No other results are currently possible.)
 */
NvBootError
NvBootRcm(NvBool IsForced, NvBool *IsFactorySecureProvisioning, NvU32 *RcmSecureDebugControl, NvU32 *EntryPoint);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_RCM_INT_H */
