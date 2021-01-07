/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_aes_device_int.h"
#include "nvboot_error.h"
#include "nvboot_se_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_aes_dev_int.h"
#include "nvboot_util_int.h"

void NvBootKcvComputeKcvSE(uint32_t *Kcv, const NvBootSeInstance SeInstance, const uint8_t KeySlot, const NvBootAesKeySize KeySize)
{
    uint32_t ZeroBuffer[NVBOOT_AES_BLOCK_LENGTH_WORDS];
    uint32_t KcvBuffer[NVBOOT_AES_BLOCK_LENGTH_WORDS];
    NvBootUtilMemset(&ZeroBuffer, 0, sizeof(ZeroBuffer));
    NvBootUtilMemset(&KcvBuffer, 0, sizeof(KcvBuffer));

    NvBootSeInstanceAesEncryptECBStart(SeInstance,
                       KeySlot,
                       NvBootSeKeySizeConv(KeySize),
                       NV_TRUE,
                       1,
                       (uint8_t *)&ZeroBuffer,
                       (uint8_t *)&KcvBuffer);

    //*Start function above doesn't poll for operation complete.
    while(NvBootSeInstanceIsEngineBusy(SeInstance, (NvU8*)Kcv))
        ;

    // Zero out the 4th byte, KCVs are 3 bytes.
    *Kcv = KcvBuffer[0] & 0x00FFFFFF;
}
