/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_AES_DEVMGR_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_AES_DEVMGR_INT_H_

#include "nvboot_error.h"
#include "nvboot_aes_device_int.h"

#include "nvboot_se_aes_dev_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct NvBootAesDevMgrCallbacksRec
{
	NvBootAesDeviceInit InitDevice;
	NvBootAesDeviceGetNumKeySlots GetNumKeySlots;
	NvBootAesDeviceIsValidKeySlot IsValidKeySlot;
	NvBootAesDeviceGetKey GetKey;
	NvBootAesDeviceGetOriginalIv GetOriginalIv;
	NvBootAesDeviceGetUpdatedIv GetUpdatedIv;
	NvBootAesDeviceSetKey SetKey;
	NvBootAesDeviceSetOriginalIv SetOriginalIv;
	NvBootAesDeviceSetUpdatedIv SetUpdatedIv;
	NvBootAesDeviceClearOriginalIv ClearOriginalIv;
	NvBootAesDeviceClearUpdatedIv ClearUpdatedIv;
    NvBootAesDeviceEncryptCBC AesEncryptCBC;
    NvBootAesDeviceDecryptCBC AesDecryptCBC;
    NvBootAesDeviceGenerateCmacSubKey AesGenerateCmacSubkey;
    NvBootAesDeviceCmacHash AesCmacHash;
    NvBootAesDeviceShutdown ShutdownDevice;
} NvBootAesDevMgrCallbacks;


/**
 * List of valid devices that implement the AES device manager API
 */
typedef enum
{
	NvBootAesDevice_SE0,
	// Reserved for future usage.
    // NvBootAesDevice_SW,

	NvBootAesDevice_Num,
} NvBootAesDeviceList;

typedef struct NvBootAesDevMgrRec
{
	NvBootAesDevMgrCallbacks *AesDevMgrCallbacks;
} NvBootAesDevMgr;


/**
 * Initialize the Aes device manager and callbacks according to the input device
 * specified.
 */
NvBootError NvBootAesDevMgrInit(NvBootAesDevMgr *AesDevMgr, NvBootAesDeviceList AesDevice);

/**
 * Do any shutdown tasks of the AES device manager if necessary.
 */
NvBootError NvBootAesDevMgrShutdown(NvBootAesDevMgr *AesDevMgr);

#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_AES_DEVMGR_INT_H_ */
