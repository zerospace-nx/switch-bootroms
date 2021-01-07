/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_pinmux_local.h"
#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_util_int.h"
#include "project.h"

extern const NvU32* g_TegraMuxControllers[];

/*
 * TODO for bug 756348, rewrite the slam code in a way that makes it easy to ECO the values
 *  (if not the addresses)
 */

#define NV_BOOT_PADCTL_SLAM(pincfg, val) \
    NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + \
               APB_MISC_GP_##pincfg##PADCTRL_0, NVBOOT_PADCTL_SLAM_VAL_##val);

NvBootError
NvBootPadsConfigForBootDevice(NvBootFuseBootDevice BootDevice, NvU32 Config)
{
    const NvU32 *BootData;


    NV_ASSERT(BootDevice < NvBootFuseBootDevice_Max);

    // Select pin mux table. 
    BootData = g_TegraMuxControllers[BootDevice];

    // Set Pin Muxes only (DoPinMux == NV_TRUE, DoPullUpPullDown == NV_FALSE). 
    NvBootPadSetPinMuxCtl(BootData, Config, NV_TRUE, NV_FALSE); 

    // Set tristates for the device. 
    NvBootPadSetTriStates(BootData, Config, NV_FALSE);

    // Set Pull-ups/Pull-downs only (DoPinMux == NV_FALSE, 
    // DoPullUpPullDown == NV_TRUE).
    NvBootPadSetPinMuxCtl(BootData, Config, NV_FALSE, NV_TRUE); 

    return NvBootError_Success;
}

// Map this code into the non secure part of the ROM
void FT_NONSECURE
NvBootPadsExitDeepPowerDown()
{
    NvU32 regVal;

    // Clear the DPD Sample and DPD Enable fields in the PMC HW

    // Note: The order of these operations must be preserved!
    // Functions like these should not have to
    // be aware of what boot path it is. But, since this function is called
    // from the non-secure dispatcher (which cannot take in function parameters)
    // this function needs to be aware if this is an SC7 cycle or not.
    if (NvBootQueryRstStatusWarmBootFlag() == NV_FALSE)
    {
        // Disable the DPD_SAMPLE field of APBDEV_PMC_DPD_SAMPLE_0
        regVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DPD_SAMPLE_0);
        regVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                    DPD_SAMPLE,
                                    ON,
                                    APBDEV_PMC_DPD_SAMPLE_0_ON_DISABLE,
                                    regVal);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DPD_SAMPLE_0, regVal);


        // Disable the DPD_ENABLE field of APBDEV_PMC_DPD_ENABLE_0
        regVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DPD_ENABLE_0);
        regVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                    DPD_ENABLE,
                                    ON,
                                    APBDEV_PMC_DPD_ENABLE_0_ON_DISABLE,
                                    regVal);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DPD_ENABLE_0, regVal);
    }

}
