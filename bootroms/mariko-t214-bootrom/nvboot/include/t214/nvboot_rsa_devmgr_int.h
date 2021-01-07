/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_RSA_DEVMGR_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_RSA_DEVMGR_INT_H_



#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvboot_error.h"
#include "nvboot_crypto_rsa_param.h"
#include "nvboot_rsa_device_int.h"
#include "nvboot_se_rsa_dev_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct NvBootRsaDevMgrCallbacksRec
{
    NvBootRsaDeviceInit InitDevice;
    NvBootRsaDeviceGetNumKeySlots GetNumKeySlots;
    NvBootRsaDeviceIsValidKeySlot IsValidKeySlot;
    NvBootRsaDeviceIsValidKeySize IsValidKeySize;
    NvBootRsaDeviceGetKey GetKey;
    NvBootRsaDeviceSetKey SetKey;
    NvBootRsaDeviceModularExponentiation RsaModularExponentiation;
    NvBootRsaDeviceIsDeviceBusy IsDeviceBusy;
    NvBootRsaDeviceShutdown ShutdownDevice;
} NvBootRsaDevMgrCallbacks;

typedef enum
{
    NvBootRsaDevice_SE0,
    NvBootRsaDevice_PKA1,
    // NvBootRsaDevice_SW,

    NvBootRsaDevice_Num,
} NvBootRsaDeviceList;

typedef struct NvBootRsaDevMgrRec
{
    NvBootRsaDevMgrCallbacks *RsaDevMgrCallbacks;
} NvBootRsaDevMgr;

NvBootError NvBootRsaDevMgrInit(NvBootRsaDevMgr *RsaDevMgr, NvBootRsaDeviceList RsaDevice);

NvBootError NvBootRsaDevMgrShutdown(NvBootRsaDevMgr *RsaDevMgr);

#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_RSA_DEVMGR_INT_H_ */
