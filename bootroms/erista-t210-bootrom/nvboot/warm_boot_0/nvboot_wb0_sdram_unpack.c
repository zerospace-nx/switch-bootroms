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
 * nvboot_wb0_sdram_unpack.c - Implements the unpacking of SDRAM parameters
 *     from PMC Scratch registers.
 */

// Don't integrate these to SW folder
#include "nvboot_wb0_sdram_scratch_list.h"
#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arapbpm.h"
#include "arclk_rst.h"
#include "aremc.h"
#include "armc.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_sdram_param.h"
#include "nvboot_util_int.h"
#include "nvboot_warm_boot_0_int.h"
#include "project.h"

/**
 * SCRATCHNAME - Compose the scratch field name
 *
 *   @param n scratch register number
 *   @param c channel number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 */
  #define SCRATCHNAME(n, d, r, f) \
    APBDEV_PMC_SCRATCH##n##_0_##d##_##r##_0_##f##_RANGE
  #define SECURESCRATCHNAME(n, d, r, f) \
    APBDEV_PMC_SECURE_SCRATCH##n##_0_##d##_##r##_0_##f##_RANGE

#ifdef NV_DEBUG
/**
 * CHECK_FIELD_SIZE - Verify that the size of the field in PMC scratch matches
 *     the size of the field in the registers.
 *
 *   @param n scratch register number
 *   @param c channel number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 *   @param p field name in NvBootSdramParams structure (unused)
 */
#define CHECK_FIELD_SIZE(n, d, r, f, p) \
    NV_CT_ASSERT(NV_FIELD_SIZE(SCRATCHNAME(n, d, r, f)) == \
                 NV_FIELD_SIZE(d##_##r##_0_##f##_RANGE));

#define CHECK_SECURE_FIELD_SIZE(n, d, r, f, p) \
    NV_CT_ASSERT(NV_FIELD_SIZE(SECURESCRATCHNAME(n, d, r, f)) == \
                 NV_FIELD_SIZE(d##_##r##_0_##f##_RANGE));

SDRAM_PMC_FIELDS(CHECK_FIELD_SIZE);

SDRAM_PMC_SECURE_FIELDS(CHECK_SECURE_FIELD_SIZE);

#endif

#include "nvboot_wb0_sdram_unpack.h"

/**
 * UNPACK_FIELD - Pull a field out of a scratch register into a field of
 *     an SDRAM parameter structure.  When needed, it reads the next
 *     scratch register into the RegVal cache.  Note that the code is most
 *     efficient when the list of scratch mappings are sorted by PMC
 *     scratch register.  Also, the compiler should optimize away
 *     LastScratchRead in optimized builds, leaving only the necessary
 *     scratch register reads.
 *
 *   @param n scratch register number
 *   @param c channel number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 *   @param p field name in NvBootSdramParams structure
 */
  #define UNPACK_FIELD(n, d, r, f, p)                                        \
    do {                                                                     \
        if (n != LastScratchRead)                                            \
        {                                                                    \
            RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +                     \
                               APBDEV_PMC_SCRATCH##n##_0);                   \
            LastScratchRead = n;                                             \
        }                                                                    \
        pData->p = NV_FLD_SET_DRF_NUM(d, r, f,                               \
                                      NV_DRF_VAL(APBDEV_PMC, SCRATCH##n,     \
                                          d##_##r##_0_##f, RegVal),          \
                                      pData->p);                             \
    } while (0);

  #define UNPACK_SECURE_FIELD(n, d, r, f, p)                                       \
    do {                                                                     \
        if (n != LastSecureScratchRead)                                            \
        {                                                                    \
            RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +                     \
                               APBDEV_PMC_SECURE_SCRATCH##n##_0);                   \
            LastSecureScratchRead = n;                                             \
        }                                                                    \
        pData->p = NV_FLD_SET_DRF_NUM(d, r, f,                               \
                                      NV_DRF_VAL(APBDEV_PMC, SECURE_SCRATCH##n,     \
                                          d##_##r##_0_##f, RegVal),    \
                                      pData->p);                             \
    } while (0);

  #define UNPACK_MC_CARVEOUT_SECURE_FIELD(n, p)                                       \
    do {                                                                     \
        pData->p = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +                     \
                               APBDEV_PMC_SECURE_SCRATCH##n##_0);                   \
    } while (0);



/**
 * UNPACK_VALUE - Pull a field out of a scratch register into a variable of
 *     an SDRAM parameter structure.  Other considerations are the same
 *     as UNPACK_FIELD.
 *
 *   @param n scratch register number
 *   @param t data type of the SDRAM parameter
 *   @param f the name of the SDRAM parameter
 *   @param p field name in NvBootSdramParams structure
 */
#define UNPACK_VALUE(n, t, f, p)                                             \
    do {                                                                     \
        if (n != LastScratchRead)                                            \
        {                                                                    \
            RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +                     \
                               APBDEV_PMC_SCRATCH##n##_0);                   \
            LastScratchRead = n;                                             \
        }                                                                    \
        pData->p = (t) NV_DRF_VAL(APBDEV_PMC, SCRATCH##n, f, RegVal);        \
    } while (0);

#define UNPACK_SECURE_VALUE(n, t, f, p)                                             \
    do {                                                                     \
        if (n != LastSecureScratchRead)                                            \
        {                                                                    \
            RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE +                     \
                               APBDEV_PMC_SECURE_SCRATCH##n##_0);                   \
            LastSecureScratchRead = n;                                             \
        }                                                                    \
        pData->p = (t) NV_DRF_VAL(APBDEV_PMC, SECURE_SCRATCH##n, f, RegVal);        \
    } while (0);

// perm is a permuation of 0 to 7, each value placed in a nibble,
// but positions 6 and 7 of the permutation are missing (bits 24 to 31);
// bit6_gt_bit7 is set if the value of bit position 6 is greater than bit position 7;
// the function returns the complete permutation (positions 6 and 7 filled in)
NvU32 decode_permutation(NvU32 perm, NvU32 bit6_gt_bit7) {
    NvU32 m, t, i, m0, m1, a, b;

    // lsb and msb are lookup tables, each entry being 2 bits;
    // the index into the lookup table is the value of a 4-bit number
    // and the value of each entry in the table is the bit position
    // of the most significant bit (msb) or least significant bit (lsb)
    // of the 4-bit number
    int lsb = 0x12131210;
    int msb = 0xffffaa50;

    // m is a bit mask with two bits set, representing the missing two values of the permutation
    m = 0xff;
    perm &= 0x00ffffff;
    t = perm;
    for (i=0; i<6; i++) {
        m &= ~(1 << (t & 7));
        t >>= 4;
    }

    // a is the value of the smallest missing value
    // b is the value of the largest missing value
    // (the ARM instruction set will allow all of the
    //  following if/then/else blocks to be straight-line code)
    m0 = (m << 1) & 0x1e;
    m1 = (m >> 3) & 0x1e;
    if (m1) {
        b = ((msb >> m1) & 3) + 4;
    } else {
        b = ((msb >> m0) & 3);
    }
    if (m0) {
        a = ((lsb >> m0) & 3);
    } else {
        a = ((lsb >> m1) & 3) + 4;
    }

    if (bit6_gt_bit7) {
        perm |= (b << 24) | (a << 28);
    } else {
        perm |= (a << 24) | (b << 28);
    }

    return perm;
}

NvBootError
NvBootWb0UnpackSdramParams(NvBootSdramParams *pData)
{
    NvU32 RegVal = 0;
    NvU32 LastScratchRead = 0x7fffffff; // A large value to force a read.
    NvU32 LastSecureScratchRead = 0x7fffffff; // A large value to force a read.

    #include "nvboot_wb0_sdram_unpack_pllm_generated.c" 

    // Unpack all the fields into pData.
    SDRAM_PMC_FIELDS(UNPACK_FIELD);
    SDRAM_PMC_VALUES(UNPACK_VALUE);

    if (pData->MemoryType == NvBootMemoryType_Ddr3)
    {
        DDR3_PMC_FIELDS(UNPACK_FIELD);
    } else if (pData->MemoryType == NvBootMemoryType_LpDdr2 || pData->MemoryType == NvBootMemoryType_LpDdr4)
    {
        LPDDR2_PMC_FIELDS(UNPACK_FIELD);
    }

    #ifdef DONT_INTEGRATE_LEGACY_DRAM_TYPES
    else
    {
        DDR3_PMC_FIELDS(UNPACK_FIELD);
    }
    #endif // DONT_INTEGRATE_LEGACY_DRAM_TYPES
    
	// Unpack all the secure fields into pData.
    SDRAM_PMC_SECURE_FIELDS(UNPACK_SECURE_FIELD);
    //SDRAM_PMC_SECURE_VALUES(UNPACK_SECURE_VALUE);

    SDRAM_PMC_MC_CARVEOUT(UNPACK_MC_CARVEOUT_SECURE_FIELD);


    // decode bit positions of permutation not explicitly saved
#define DECODE_BIT_POS_C(_c_, _c2_, _r_, _b_) \
    pData->_c_##EmcSwizzleRank##_r_##Byte##_b_ = \
        decode_permutation(pData->_c_##EmcSwizzleRank##_r_##Byte##_b_, \
            NV_DRF_VAL(SWIZZLE, BIT6_GT_BIT7, _c2_##_RANK##_r_##_BYTE##_b_, pData->SwizzleRankByteEncode));

#define DECODE_BIT_POS_R(_r_,_b_) \
    DECODE_BIT_POS_C(,CH0,_r_,_b_)

#define DECODE_BIT_POS_B(_b_) \
    DECODE_BIT_POS_R(0, _b_) \
    DECODE_BIT_POS_R(1, _b_)

#define DECODE_BIT_POS \
    DECODE_BIT_POS_B(0) \
    DECODE_BIT_POS_B(1) \
    DECODE_BIT_POS_B(2) \
    DECODE_BIT_POS_B(3)

    DECODE_BIT_POS

#undef DECODE_BIT_POS
#undef DECODE_BIT_POS_B
#undef DECODE_BIT_POS_R
#undef DECODE_BIT_POS_C

    // MC and EMC clock override
    if (pData->EmcClkenOverrideAllWarmBoot)
    {
        pData->EmcClkenOverride = EMC_CLKEN_OVERRIDE_0_SW_DEFAULT_MASK;
    }

    if (pData->McClkenOverrideAllWarmBoot)
    {
        pData->McClkenOverride = MC_CLKEN_OVERRIDE_0_SW_DEFAULT_MASK;
    }

    return NvBootError_Success;
}
