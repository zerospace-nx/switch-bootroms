/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_LOCK_PMC_SECURE_H
#define INCLUDED_NVBOOT_LOCK_PMC_SECURE_H

// Don't integrate these to SW folder
#include "nvboot_wb0_sdram_scratch_list.h"
#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arapb_misc.h"
#include "arapbpm.h"
#include "aremc.h"
#include "armc.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_sdram_param.h"
#include "nvboot_util_int.h"
#include "nvboot_warm_boot_0_int.h"
#include "project.h"

#define MAX_IDX_IN_PMC_SEC_DISABLE_0  8


void NvBootWriteLockPmcSecure() 
{
    NvU32 i, j;
    NvU32 forbidden_idx = 0;
    NvU32 regVal = 0;
    NvU32 cur_pmc_sec_idx; 
    NvU32 pmc_sec_disable_start_idx, pmc_sec_disable_end_idx;
    NvU32 forbiddenPmcIdx[] = SDRAM_PMC_SECURE_FORBIDDEN_IDX;
    NvU32 PmcSecDisable_Offset[] = {APBDEV_PMC_SEC_DISABLE_0, APBDEV_PMC_SEC_DISABLE2_0, APBDEV_PMC_SEC_DISABLE3_0, APBDEV_PMC_SEC_DISABLE4_0,
				 APBDEV_PMC_SEC_DISABLE5_0, APBDEV_PMC_SEC_DISABLE6_0, APBDEV_PMC_SEC_DISABLE7_0, APBDEV_PMC_SEC_DISABLE8_0}; 

    
    pmc_sec_disable_start_idx = (SDRAM_PMC_SECURE_START_IDX + MAX_IDX_IN_PMC_SEC_DISABLE_0)/16 + 1;  	
    pmc_sec_disable_end_idx   = (SDRAM_PMC_SECURE_END_IDX + MAX_IDX_IN_PMC_SEC_DISABLE_0)/16 + 1;


    for (i= pmc_sec_disable_start_idx; i<= pmc_sec_disable_end_idx; i++) {
	    regVal = 0;
        for (j = 0; j<16; j++){ 
            cur_pmc_sec_idx =  (i-1)*16-MAX_IDX_IN_PMC_SEC_DISABLE_0 + j;
            if (cur_pmc_sec_idx < SDRAM_PMC_SECURE_START_IDX || cur_pmc_sec_idx > SDRAM_PMC_SECURE_END_IDX) {
                break;	
            } else if (forbidden_idx < SDRAM_PMC_SECURE_FORBIDDEN_IDX_LEN) {
		        if (cur_pmc_sec_idx == forbiddenPmcIdx[forbidden_idx]) {
                    forbidden_idx++;
	        	} else {
		            regVal = regVal | (1 << (2*j)) ;
	            }
	         } else {
		    regVal = regVal | (1 << (2*j)) ;
	        }
        }
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + PmcSecDisable_Offset[i-1], regVal); 
    }

}

#endif
