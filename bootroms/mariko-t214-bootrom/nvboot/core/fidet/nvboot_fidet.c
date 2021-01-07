/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <stdint.h>
#include "nvtypes.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "arfuse.h"

#include "nvboot_error.h"
#include "nvboot_section_defs.h"
#include "nvboot_bpmp_int.h"
#include "nvboot_hardware_access_int.h"
#include "project.h"

#define MAX_FAULTS      0xffffff
#define PERSIST_REG     APBDEV_PMC_SECURE_SCRATCH124_0

#define FUSE_RESERVED_SW_0_NOBOOT_RANGE     9:9
#define FUSE_RESERVED_FIELD_0_RMA_RANGE     1:0

static uint32_t FT_NONSECURE LoadFaultCounter(void)
{
    uint32_t val, RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
            APBDEV_PMC_RST_STATUS_0_RST_SOURCE_POR)
        val = 0;
    else
        val = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + PERSIST_REG);

    return val;
}
static void FT_NONSECURE StoreFaultCounter(uint32_t val)
{
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + PERSIST_REG, val);
}

void FT_NONSECURE NvBootCheckRMAStatus(void)
{
    uint32_t RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    if (NV_DRF_VAL(FUSE, RESERVED_SW, NOBOOT, RegData)) {
        RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_FIELD_0);
        if (NV_DRF_VAL(FUSE, RESERVED_FIELD, RMA, RegData))
            do_exception();
    }
}

void FT_NONSECURE NvBootFaultInjectDetect(void)
{
    uint32_t i, j, faults;

    faults = LoadFaultCounter();
    for (i=0x01, j=0x01; i!=MAX_FAULTS; i=(i << 1) | 0x01, j<<=1)
        if ((faults & j) == 0 &&
            (faults & (~i)) == 0) break;
    faults |= i;
    StoreFaultCounter(faults);
}

