/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

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
#include "nvboot_lock_pmc_secure.h"


/**
 * PACK_FIELD - Pull a field out of a field of an SDRAM parameter structure
 *     into a field of a scratch register. When needed, it writes out
 *     the scratch register value and clears the RegVal cache.
 *
 *     Note: For best efficiency, all fields for a scratch register should
 *           be grouped together in the list macro.
 *     Note: The compiler should optimize away redundant work in optimized
 *           builds.
 *
 *   @param n scratch register number
 *   @param c channel number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 *   @param p field name in NvBootSdramParams structure
 */

#define PACK_FIELD(n, d, r, f, p)                                             \
do {                                                                          \
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH##n##_0);  \
    RegVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC, SCRATCH##n, d##_##r##_0_##f,      \
                                NV_DRF_VAL(d, r, f, pData->p),                \
                                RegVal);                                      \
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH##n##_0, RegVal);  \
} while (0);

#define PACK_SECURE_FIELD(n, d, r, f, p)                                             \
do {                                                                                 \
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH##n##_0);  \
    RegVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC, SECURE_SCRATCH##n, d##_##r##_0_##f,      \
                                NV_DRF_VAL(d, r, f, pData->p),                       \
                                RegVal);                                             \
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH##n##_0, RegVal);  \
} while (0);

#define PACK_MC_CARVEOUT_SECURE_FIELD(n, p)                                             \
do {                                                                                 \
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH##n##_0, pData->p);  \
} while (0);


/**
 * PACK_VALUE - Pull a variable out of an SDRAM parameter structure into
 *     a field of a scratch register. Other considerations are the same
 *     as PACK_FIELD.
 *
 *   @param n scratch register number
 *   @param t data type of the SDRAM parameter (unused)
 *   @param f the name of the SDRAM parameter
 *   @param p field name in NvBootSdramParams structure
 */
#define PACK_VALUE(n, t, f, p)                                               \
do {                                                                         \
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH##n##_0); \
    RegVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC, SCRATCH##n, f,                   \
                                pData->p,                                    \
                                RegVal);                                     \
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH##n##_0, RegVal); \
} while (0);

#define PACK_SECURE_VALUE(n, t, f, p)                                               \
do {                                                                        		\
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH##n##_0); \
    RegVal = NV_FLD_SET_DRF_NUM(APBDEV_PMC, SECURE_SCRATCH##n, f,                   \
                                pData->p,                                    		\
                                RegVal);                                     		\
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH##n##_0, RegVal); \
} while (0);


void
NvBootWb0PackSdramParams(NvBootSdramParams *pData)
{
    NvU32 RegVal;

    NV_ASSERT(pData);

    // encode bit positions of permutation not explicitly saved
#define ENCODE_BIT_POS_C(_c_, _c2_, _r_, _b_) \
    pData->SwizzleRankByteEncode = \
      NV_FLD_SET_DRF_NUM(SWIZZLE, BIT6_GT_BIT7, _c2_##_RANK##_r_##_BYTE##_b_, \
      NV_DRF_VAL(EMC, SWIZZLE_RANK##_r_##_BYTE##_b_, SWZ_RANK##_r_##_BYTE##_b_##_BIT6_SEL, pData-> _c_##EmcSwizzleRank##_r_##Byte##_b_) > \
      NV_DRF_VAL(EMC, SWIZZLE_RANK##_r_##_BYTE##_b_, SWZ_RANK##_r_##_BYTE##_b_##_BIT7_SEL, pData-> _c_##EmcSwizzleRank##_r_##Byte##_b_), \
      pData->SwizzleRankByteEncode);

#define ENCODE_BIT_POS_R(_r_,_b_) \
    ENCODE_BIT_POS_C(,CH0,_r_,_b_) 

#define ENCODE_BIT_POS_B(_b_) \
    ENCODE_BIT_POS_R(0, _b_) \
    ENCODE_BIT_POS_R(1, _b_)

#define ENCODE_BIT_POS \
    ENCODE_BIT_POS_B(0) \
    ENCODE_BIT_POS_B(1) \
    ENCODE_BIT_POS_B(2) \
    ENCODE_BIT_POS_B(3)

    pData->SwizzleRankByteEncode = 0;
    ENCODE_BIT_POS

#undef ENCODE_BIT_POS
#undef ENCODE_BIT_POS_B
#undef ENCODE_BIT_POS_R
#undef ENCODE_BIT_POS_C

    SDRAM_PMC_FIELDS(PACK_FIELD);
    SDRAM_PMC_VALUES(PACK_VALUE);

    if (pData->MemoryType == NvBootMemoryType_Ddr3)
    {
        DDR3_PMC_FIELDS(PACK_FIELD);
    } else if (pData->MemoryType == NvBootMemoryType_LpDdr2)
    {
        LPDDR2_PMC_FIELDS(PACK_FIELD);
    } else if (pData->MemoryType == NvBootMemoryType_LpDdr4)
    {
        LPDDR2_PMC_FIELDS(PACK_FIELD);
    }



    #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
    else
    {
        DDR3_PMC_FIELDS(PACK_FIELD);
    }
    #endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES

    SDRAM_PMC_SECURE_FIELDS(PACK_SECURE_FIELD);
    SDRAM_PMC_MC_CARVEOUT(PACK_MC_CARVEOUT_SECURE_FIELD);
//    SDRAM_PMC_SECURE_VALUES(PACK_SECURE_VALUE);

    NvBootWriteLockPmcSecure();

}
