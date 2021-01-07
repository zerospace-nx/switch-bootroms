/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "project.h"
#include "arapbpm.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_debug_int.h"
#include "nvboot_context_int.h"
#include "nvboot_util_int.h"

extern NvBootContext Context;

/**
static uint32_t GetDebugRegister(void)
{
    return NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DEBUG_AUTHENTICATION_0);
}
*/

static void SetDebugRegister(uint32_t value)
{
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DEBUG_AUTHENTICATION_0, value);
}

NvBootError
NvBootDebugSetDebugFeatures(const NvBootECID *InputEcid,
                            uint32_t SecureDebugControl_Not_ECID_Checked,
                            uint32_t SecureDebugControl_ECID_Checked)
{
    // Mask unused bits.
    SecureDebugControl_Not_ECID_Checked &= APBDEV_PMC_DEBUG_AUTHENTICATION_0_NOT_ECID_CHECKED_FIELD;
    SecureDebugControl_ECID_Checked &= APBDEV_PMC_DEBUG_AUTHENTICATION_0_ECID_CHECKED_FIELD;

    if(NvBootFuseIsPreproductionMode())
    {
        SetDebugRegister(DEBUG_ALL_ENABLED);
        return NvBootError_Success;
    }
    else if(NvBootFuseIsNvProductionMode() && (Context.FactorySecureProvisioningMode == false))
    {
        SetDebugRegister(DEBUG_ALL_ENABLED);
        return NvBootError_Success;
    }
    else
    // Else, this should be ODM production mode, i.e.
    // FUSE_PRODUCTION_MODE = 1, FUSE_SECURITY_MODE = 1, or FSKP mode.
    {
        uint32_t FinalDebugValue = DEBUG_ALL_DISABLED; // 0
        NvBootECID ChipEcid;
        NvBootFuseGetUniqueId(&ChipEcid);

        FI_bool compare_result = NvBootUtilCompareConstTimeFI((uint32_t *) InputEcid, (uint32_t *) &ChipEcid, sizeof(NvBootECID));
        if(compare_result == FI_TRUE)
        {
            FinalDebugValue |= SecureDebugControl_ECID_Checked;
        }

        FinalDebugValue |= SecureDebugControl_Not_ECID_Checked;

        SetDebugRegister(FinalDebugValue);

        if(compare_result == FI_TRUE)
            return NvBootError_Success;
        else
            return NvBootError_ECID_Mismatch;
    }
}

