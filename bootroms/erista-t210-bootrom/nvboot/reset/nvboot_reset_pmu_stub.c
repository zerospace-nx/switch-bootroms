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
 * nvboot_reset_pmu.c - Implementation of PMU reset support.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_reset_pmu_int.h"
#include "nvboot_util_int.h"
#include "project.h"


/* Tsense reset BASIC handler. This is only linked with DEBUG build.
 */

static NvBool NvBootResetQueryTsenseFlag()
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
	APBDEV_PMC_RST_STATUS_0_RST_SOURCE_SENSOR)
	return NV_TRUE;

    return NV_FALSE;
}

void NvBootResetRailsAndResetHandler()
{
    volatile int i = 1;

    //Check if Tsense reset flag is set, otherwise nothing to do
    //in this function.
    if (!NvBootResetQueryTsenseFlag()) 
	return;

    while(i);
}

