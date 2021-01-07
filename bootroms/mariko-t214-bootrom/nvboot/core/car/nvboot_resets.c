/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
 
 /*
  * nvboot_clocks.c - Implementation of Clocks support.
  *
  */

#include "nvtypes.h"
#include "nvrm_drf.h"
#include "arfuse.h"
#include "arapbpm.h"
#include "project.h"
#include "arclk_rst.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include <nvboot_section_defs.h>

void FT_NONSECURE 
NvBootResetSetEnable(const NvBootResetDeviceId DeviceId, const NvBool Enable) {
    // Offsets to the Set reset registers. Clr register offsets are always
    // +4 after the Set register.
    static const VT_NONSECURE NvU32 CarResetSetClrTbl[CarResetReg_Num] = {
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_L_SET_0,
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_H_SET_0,
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_U_SET_0,
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_X_SET_0,
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_Y_SET_0,
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_V_SET_0,
        NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_W_SET_0,
    };

    NvU32 RegAddr ;

    NVBOOT_RESET_CHECK_ID(DeviceId) ;
    NV_ASSERT( ((int) Enable == 0) ||
               ((int) Enable == 1) ) ;

    // Note for NvBootResetDeviceId_McId:
    // Bug 1501873. Writing to RSR_DEVICES has no effect for MC. ECO connected
    // only RST_DEV_CLR for MC reset de-assert.
    RegAddr = CarResetSetClrTbl[NVBOOT_RESET_REG_OFFSET(DeviceId)];

    if (!Enable)
    {
        RegAddr += 4; // CLR register is always immediately after SET register
    }
    NV_WRITE32(RegAddr, 1 << NVBOOT_RESET_BIT_OFFSET(DeviceId));

    // wait stabilization time (always)
    NvBootUtilWaitUS(NVBOOT_RESET_STABILIZATION_DELAY) ;

    return ;
}

void FT_NONSECURE
NvBootResetFullChip(void) {
    NvU32 regData ;
    regData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0) ;
    regData = NV_FLD_SET_DRF_DEF(APBDEV_PMC, CNTRL, MAIN_RST, ENABLE, regData) ;
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_CNTRL_0, regData) ;

    //This function should *NEVER* return. Spin loop here until HW reset itself.
    //Enable BootROM patcher to revert to the old style.
    while(1) {
    // wait is nto really needed, insure nothing dangerous is started
    	NvBootUtilWaitUS(NVBOOT_RESET_STABILIZATION_DELAY) ;
    }
}

