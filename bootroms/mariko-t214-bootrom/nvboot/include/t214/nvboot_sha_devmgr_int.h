/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_SHA_DEVMGR_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_SHA_DEVMGR_INT_H_


#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvboot_error.h"
#include "nvboot_sha_device_int.h"
#include "nvboot_se_sha_dev_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct NvBootShaDevMgrCallbacksRec
{
    NvBootShaDeviceInit InitDevice;
    NvBootShaDeviceGetShaFamily GetShaFamily;
    NvBootShaDeviceSetShaFamily SetShaFamily;
    NvBootShaDeviceGetShaDigestSize GetDigestSize;
    NvBootShaDeviceSetShaDigestSize SetDigestSize;
    NvBootShaDeviceIsValidShaFamily IsValidShaFamily;
    NvBootShaDeviceIsValidShaDigestSize IsValidDigestSize;
    NvBootShaDeviceShaHash ShaHash;
    NvBootShaDeviceShutdown ShutdownDevice;
} NvBootShaDevMgrCallbacks;

typedef enum
{
    NvBootShaDevice_SE0,
    NvBootShaDevice_SW,

    NvBootShaDevice_Num,
} NvBootShaDeviceList;

typedef struct NvBootShaDevMgrRec
{
    NvBootShaDevMgrCallbacks *ShaDevMgrCallbacks;
    NvBootCryptoShaConfig ShaConfig;
} NvBootShaDevMgr;

NvBootError NvBootShaDevMgrInit(NvBootShaDevMgr *ShaDevMgr, NvBootShaDeviceList ShaDevice);

NvBootError NvBootShaDevMgrShutdown(NvBootShaDevMgr *ShaDevMgr);

#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_SHA_DEVMGR_INT_H_ */
