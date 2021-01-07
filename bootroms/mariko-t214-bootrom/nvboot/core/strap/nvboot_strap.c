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
 * nvboot_strap.c - Implementation of strap support.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "arapbpm.h"
#include "nvboot_error.h"
#include "nvboot_hacks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_strap_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_util_int.h"
#include "project.h"

NvU32 FT_NONSECURE NvBootReadStrap(void)
{
    return NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_STRAPPING_OPT_A_0);
}

NvBool
NvBootStrapIsForceRecoveryMode(void)
{
#if ! USE_STRAPS
    return NVBOOT_DEFAULT_FORCE_RECOVERY_MODE_STRAP;
#else
    NvU32 RcmStraps, TwoButtonRcm=0;

    // See Boot GFD for details. See bug 1265136 as well.
    RcmStraps = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                        APB_MISC_PP_STRAPPING_OPT_A_0);

    // Is Volume Up Button/Force RCM pressed
    if(!NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RCM_STRAPS_FORCE_RCM, RcmStraps))
    {
        // Is 2 Button RCM Fuse set
        NvBootFuseGet2ButtonRcm(&TwoButtonRcm);
        if(TwoButtonRcm) 
        {
            // Is Home Button Pressed
            if(!NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RCM_STRAPS_2_BUTTON_RCM, RcmStraps))
                return NV_TRUE;
        }
        else
            return NV_TRUE;
    }

    return NV_FALSE;
#endif
}

// Note: This function ONLY checks for Debug RCM strap so it must
// NEVER be the sole decider for RCM exit. It should be called after
// NvBootStrapIsForceRecoveryMode
NvBool
NvBootStrapIsDebugRecoveryMode(void)
{
#if ! USE_STRAPS
    return NVBOOT_DEFAULT_FORCE_DEBUG_RCM_STRAP;
#else
    NvU32 RcmStraps;

    // See Boot GFD for details. See bug 1265136 as well.
    RcmStraps = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                        APB_MISC_PP_STRAPPING_OPT_A_0);

    // Is Volume Down Button/Debug RCM pressed
    if(!NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RCM_STRAPS_DEBUG_RCM, RcmStraps))
    {
        return NV_TRUE;
    }

    return NV_FALSE;
#endif
}

NvBool
NvBootStrapIsProdUartMode(void)
{
#if ! USE_STRAPS
    return NVBOOT_DEFAULT_NVPROD_UART_STRAP;
#else
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                        APB_MISC_PP_STRAPPING_OPT_A_0);

    if (NV_DRF_VAL(APB_MISC_PP,
                   STRAPPING_OPT_A,
                   NVPROD_UART,
                   RegData))
    {
        return NV_TRUE;
    }
    return NV_FALSE;
#endif
}

NvU8
NvBootStrapDeviceConfigurationIndex(void)
{
#if ! USE_STRAPS
    return NVBOOT_DEFAULT_PARAM_STRAP;
#else
    /** Bug 1444822, See Boot GFD for details.
     *  RAM_CODE straps have been reduced to 2 bits (used by sdram param index)
     *  Only 1 dev param set allowed. Return index 0 always. Uncomment code
     *  below if dev param index straps come back.
     */
    // NvU32 RegData;
    // NvU32 RamCode;

    // RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                        // APB_MISC_PP_STRAPPING_OPT_A_0);
    // RamCode = NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RAM_CODE, RegData);

    // return NV_DRF_VAL(STRAP, RAM_CODE, DEVICE_CONFIGURATION, RamCode);
    return 0;
#endif
}

NvU8
NvBootStrapSdramConfigurationIndex(void)
{
#if ! USE_STRAPS
    return NVBOOT_DEFAULT_SDRAM_STRAP;
#else
    NvU32 RegData;
    NvU32 RamCode;

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                        APB_MISC_PP_STRAPPING_OPT_A_0);
    RamCode = NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RAM_CODE, RegData);

    return NV_DRF_VAL(STRAP, RAM_CODE, SDRAM_CONFIGURATION, RamCode);
#endif
}

NvBootStrapDevSel
NvBootStrapDeviceSelection(void)
{
#if ! USE_STRAPS
    return NVBOOT_DEFAULT_DEV_SEL_STRAP;
#else
    NvU32 RegData;
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                       APB_MISC_PP_STRAPPING_OPT_A_0);

    return (NvBootStrapDevSel)(NV_DRF_VAL(APB_MISC_PP,
                                          STRAPPING_OPT_A,
                                          BOOT_SELECT,
                                          RegData));
#endif
}

void
NvBootStrapAliasStraps()
{
/* Strap aliasing removed.
 * This is done by fuse bypass package
 */
}
