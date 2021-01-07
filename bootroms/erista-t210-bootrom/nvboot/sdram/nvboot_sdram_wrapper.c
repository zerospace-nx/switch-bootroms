/*
 * Copyright (c) 2012 - 2013 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_sdram_wrapper.c - Implementation of sdram wrapper
 *
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "nvboot_sdram_int.h"
#include "project.h"

NvU32
NvBootSdramQueryTotalSize(void)
{
    NvU32 SdramTotalSizeMB;
    
#ifndef TARGET_ASIM
    // Capped at 2GB.
    SdramTotalSizeMB = NV_MIN((NV_ADDRESS_MAP_EMEM_LO_LIMIT - NV_ADDRESS_MAP_EMEM_LO_BASE) >> 20,
                NvBootSdramQueryTotalMB());
    return (SdramTotalSizeMB << 20); // Converting to bytes.
#else
    return (NV_ADDRESS_MAP_EMEM_LO_LIMIT - NV_ADDRESS_MAP_EMEM_LO_BASE);
#endif
}
