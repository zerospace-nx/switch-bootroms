/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
 #include "nvboot_platform_int.h"
 #include <nvboot_section_defs.h>
 TODO
 /**
  *  This routine detects platform state using a predetermined fuse.
  *  fuse should be set using runscript in simulations.
  *  Ensure PLATFORM_SI is set to 1 before tapeout
  */
 
 FT_NONSECURE uint32_t NvBootIsPlatformRtl(void)
 {
#if !PLATFORM_SI
     uint32_t RegData;
     RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_GP_HIDREV_0);
     if(NV_DRF_VAL(APB_MISC_GP, HIDREV, PRE_SI_PLATFORM, RegData) == APB_MISC_GP_HIDREV_0_PRE_SI_PLATFORM_VERIFICATION_SIMULATION)
     {
         return 1;
     }
#endif
    return 0;
 }
 
 /**
  *  No condition except PLATFORM_SI flag.
  *  Make sure to set to 1 for tapeout.
  */
 FT_NONSECURE uint32_t NvBootIsPlatformSi(void)
 {
#if !PLATFORM_SI
    uint32_t RegData;
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_GP_HIDREV_0);
    if(NV_DRF_VAL(APB_MISC_GP, HIDREV, PRE_SI_PLATFORM, RegData) != APB_MISC_GP_HIDREV_0_PRE_SI_PLATFORM_SILICON)
    {
        return 0;
    }
#endif
    // Take no chances for si. Just return true;
     return 1;
 }
 
 /**
  *  Detect fpga using MISCREG_HIDREV_0[PRE_SI_PLATFORM]
  */
 FT_NONSECURE uint32_t NvBootIsPlatformFpga(void)
 {
#if !PLATFORM_SI
    uint32_t RegData;
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_GP_HIDREV_0);
    if(NV_DRF_VAL(APB_MISC_GP, HIDREV, PRE_SI_PLATFORM, RegData) == APB_MISC_GP_HIDREV_0_PRE_SI_PLATFORM_SYSTEM_FPGA)
    {
        return 1;
    }
#endif
    return 0;
 }
 
