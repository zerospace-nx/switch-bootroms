/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_aes_devmgr_int.h"
#include "nvboot_hardware_access_int.h"

//static NvBootAesDevMgrContext AesDevMgrContext;
//typedef uint8_t (*NvBootAesDeviceGetNumKeySlots)(void);

//typedef bool (*NvBootAesDeviceIsValidKeySlot)(uint8_t KeySlot);

NvBootAesDevMgrCallbacks s_AesDevMgrCallbacks[] =
{
		{
                // SE0 AES engine
				.InitDevice = NvBootSeAesDevInit,
				.GetNumKeySlots = NvBootSeAesDevNumKeySlots,
				.IsValidKeySlot = NvBootSeAesDevIsValidKeySlot,
				.GetKey = NvBootSeAesDevGetKey,
				.SetKey = NvBootSeAesDevSetKey,
                .AesEncryptCBC = NvBootSeAesDevEncryptCBC,
                .AesDecryptCBC = NvBootSeAesDevDecryptCBC,
                .SetOriginalIv = NvBootSeAesDevSetOriginalIv,
                .SetUpdatedIv = NvBootSeAesDevSetUpdatedIv,
                .ClearOriginalIv = NvBootSeAesDevClearOriginalIv,
                .ClearUpdatedIv = NvBootSeAesDevClearUpdatedIv,
                .AesGenerateCmacSubkey = NvBootSeAesDevGenCmacSubkey,
                .AesCmacHash = NvBootSeAesCmacHash,
		},
};

NvBootError NvBootAesDevMgrInit(NvBootAesDevMgr *AesDevMgr, NvBootAesDeviceList AesDevice)
{
	AesDevMgr->AesDevMgrCallbacks = &(s_AesDevMgrCallbacks[AesDevice]);

	AesDevMgr->AesDevMgrCallbacks->InitDevice();

	return NvBootError_Success;
}
