/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_ahb.c - Implementation of AHB Master coherency support
 *
 */

#include "nvcommon.h"
#include "arahb_arbc.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_util_int.h"
#include "project.h"

// 200000 microseconds timeout (= 200 ms)
#define NVBOOT_AHB_WAIT_COHERENCY_TIMEOUT_US 200000

NvBool NvBootAhbCheckCoherency(NvU8 AhbMasterID) 
{
    NvU32 AhbMemWrque = 0;

    // SW CYA for AHB coherency feature
    if(NvBootGetSwCYA() & NVBOOT_SW_CYA_AHB_COHERENCY_DISABLE)
    {
        return NV_TRUE;
    }

    AhbMemWrque = NV_READ32(NV_ADDRESS_MAP_AHB_ARBC_BASE + AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID_0);
    AhbMemWrque &= (1 << AhbMasterID);

    // if the bit corresponding to that AHB master is zero, no more writes 
    // external memory are outstanding. 
    return (AhbMemWrque == 0) ? NV_TRUE : NV_FALSE;
}

void NvBootAhbWaitCoherency(NvU8 AhbMasterID) 
{
    NvU32   AhbMemWrque = 0;
    NvU32   BeginTime;

    // SW CYA for AHB coherency feature
    if(NvBootGetSwCYA() & NVBOOT_SW_CYA_AHB_COHERENCY_DISABLE)
    {
        return;
    }

    BeginTime = NvBootUtilGetTimeUS();
    do 
    {
        AhbMemWrque = NV_READ32(NV_ADDRESS_MAP_AHB_ARBC_BASE + AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID_0);
    } while ( (AhbMemWrque & (1 << AhbMasterID)) && (NvBootUtilElapsedTimeUS(BeginTime) < NVBOOT_AHB_WAIT_COHERENCY_TIMEOUT_US) );

    return;
    
}

NvBool NvBootAhbCheckIsExtMemAddr(NvU32 *Address) 
{
    return
    ( (Address >= (NvU32 *)NV_ADDRESS_MAP_EMEM_LO_BASE) && (Address <= (NvU32 *)NV_ADDRESS_MAP_EMEM_LO_LIMIT) ) ? 
    NV_TRUE : NV_FALSE;
}
