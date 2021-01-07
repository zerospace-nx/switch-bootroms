/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_rsa_device_int.h"
#include "nvboot_rsa_devmgr_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_se_rsa_dev_int.h"

NvBootRsaDevMgrCallbacks s_RsaDevMgrCallbacks[] =
{
        {
                .InitDevice = NvBootSeRsaDevInit,
                .GetNumKeySlots = NvBootSeRsaDevNumKeySlots,
                .IsValidKeySlot = NvBootSeRsaDevIsValidKeySlot,
                .IsValidKeySize = NvBootSeRsaDevIsValidKeySize,
                .GetKey = NvBootSeRsaDevGetKey,
                .SetKey = NvBootSeRsaDevSetKey,
                .RsaModularExponentiation = NvBootSeRsaDevModularExponentiation,
                .IsDeviceBusy = NvBootSeRsaDevIsEngineBusy,
                .ShutdownDevice = NULL,
        },
};

NvBootError NvBootRsaDevMgrInit(NvBootRsaDevMgr *RsaDevMgr, NvBootRsaDeviceList RsaDevice)
{
    RsaDevMgr->RsaDevMgrCallbacks = &(s_RsaDevMgrCallbacks[RsaDevice]);

    RsaDevMgr->RsaDevMgrCallbacks->InitDevice();

    return NvBootError_Success;
}
