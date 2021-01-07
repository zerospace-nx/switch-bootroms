/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */


#include "nvboot_sha_devmgr_int.h"
#include "nvboot_hardware_access_int.h"

NvBootShaDevMgrCallbacks s_ShaDevMgrCallbacks[] =
{
        {
                // SE0
                .InitDevice = NvBootSeShaDevInit,
                .GetShaFamily = NvBootSeShaGetShaFamily,
                .SetShaFamily = NvBootSeShaSetShaFamily,
                .GetDigestSize = NvBootSeShaGetDigestSize,
                .SetDigestSize = NvBootSeShaSetShaDigestSize,
                .IsValidShaFamily = NvBootSeShaIsValidShaFamily,
                .IsValidDigestSize = NvBootSeShaIsValidShaDigestSize,
                .ShaHash = NvBootSeShaDevShaHash,
                .ShutdownDevice = NvBootSeShaDeviceShutdown,
        },
        {
                // SW engine
                .InitDevice = NULL,
                .GetShaFamily = NULL,
                .SetShaFamily = NULL,
                .GetDigestSize = NULL,
                .SetDigestSize = NULL,
                .IsValidShaFamily = NULL,
                .IsValidDigestSize = NULL,
                .ShaHash = NULL,
                .ShutdownDevice = NULL,
        },
};

NvBootError NvBootShaDevMgrInit(NvBootShaDevMgr *ShaDevMgr, NvBootShaDeviceList ShaDevice)
{
    ShaDevMgr->ShaDevMgrCallbacks = &(s_ShaDevMgrCallbacks[ShaDevice]);

    ShaDevMgr->ShaDevMgrCallbacks->InitDevice(&ShaDevMgr->ShaConfig);

    return NvBootError_Success;
}
