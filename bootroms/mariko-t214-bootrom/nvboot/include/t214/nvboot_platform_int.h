/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
 #ifndef PLATFORM_INT_H
 #define PLATFORM_INT_H
 
 #include "nvboot_hardware_access_int.h"
 #include "arapb_misc.h"
 #include "project.h"
 #include "arfuse.h"
 #include "nvrm_drf.h"
  #include "nvtypes.h"
 
 #define PLATFORM_RTL 0xCAFED00D
 #ifndef PLATFORM_SI
 #define PLATFORM_SI 1
 #endif
 
 /**
  *  This routine detects platform state using a predetermined fuse.
  *  Fuse should be set using runscript in simulations.
  *  Always return false for tapeout
  */
 uint32_t NvBootIsPlatformRtl(void);
 
 /**
  *  Detects Si using APB_MISC_GP_HIDREV_0[PRE_SI_PLATFORM]
  *  Always return true for tapeout
  */
 uint32_t NvBootIsPlatformSi(void);
 
 /**
  *  Detect fpga using APB_MISC_GP_HIDREV_0[PRE_SI_PLATFORM]
  *  Always return false for tapeout
  */
 uint32_t NvBootIsPlatformFpga(void);
 
 #endif