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
 * nvboot_devmgr_int.h - Definition of the device manager interface.
 */

#ifndef INCLUDED_NVBOOT_DEVMGR_INT_H
#define INCLUDED_NVBOOT_DEVMGR_INT_H

#include "nvcommon.h"
#include "nvboot_bit.h"
#include "nvboot_error.h"
#include "nvboot_device_int.h"
#include "nvboot_devparams.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct NvBootDevMgrCallbacksRec
{
    NvBootDeviceGetParams      GetParams;
    NvBootDeviceValidateParams ValidateParams;
    NvBootDeviceGetBlockSizes  GetBlockSizes;
    NvBootDeviceInit           Init;
    NvBootDeviceReadPage       ReadPage;
    NvBootDeviceQueryStatus    QueryStatus;
    NvBootDeviceShutdown       Shutdown;
    NvBootDeviceGetReaderBuffersBase GetReaderBuffersBase;
} NvBootDevMgrCallbacks;

/*
 * NvBootDevMgr: State & data used by the device manager.
 */
typedef struct NvBootDevMgrRec
{
    /* Device Info */
    NvU32                   BlockSizeLog2;
    NvU32                   PageSizeLog2;
    NvBootDevMgrCallbacks  *Callbacks;    /* Callbacks to the chosen driver. */
} NvBootDevMgr;


/*
 * Functions
 */

/*
 * NvBootDevMgrInit(): Initialize the device manager and the desired device.
 * DevType selects the type of device, and is assumed to come from fuses.
 * ConfigIndex selects the parameter configuration to use, and is also
 * expected to come from fuses.
 */
NvBootError
NvBootDevMgrInit(NvBootDevMgr  *DevMgr, 
                 NvBootDevType  DevType,
                 NvU32          ConfigIndex);

/*
 * NvBootDevMgrReinitDevice(): Reinitialize the device based on parameter
 * data contained within the BCT.
 */
NvBootError
NvBootDevMgrReinitDevice(NvBootDevMgr    *DevMgr,
			 NvBootDevParams *BctParams,
			 const NvU8       ParamCount,
                         const NvU8       DeviceStraps);

/*
 * NvBootDevMgrShutdown(): Shutdown the device and the device manager.
 */
void NvBootDevMgrShutdown(NvBootDevMgr *State);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_DEVMGR_INT_H */
