/*
 * Copyright (c) 2006 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_sdram_int.h
 *
 * NvBootSdram interface for NvBOOT
 *
 * NvBootSdram is NVIDIA's interface for SDRAM configuration and control.
 *
 *
 * For frozen/cold boot, the process is --
 * 1. NvBootSdramInitFromParams
 * 2. NvBootSdramQueryTotalSize
 *
 *
 */

#ifndef INCLUDED_NVBOOT_SDRAM_INT_H
#define INCLUDED_NVBOOT_SDRAM_INT_H

#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_sdram_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
  * Initializes the SDRAM controller according to the given configuration
  * parameters.  Intended for frozen/cold boot use.
  *
  * Generally the steps are --
  * 1. Program and enable PLL
  * 2. Wait for the PLL to get locked
  * 3. Enable clock and disable reset to emc
  *
  * Note: Clocks must be enabled to the PMC block before calling this function.
  *
  * @param ParamData pointer to configuration parameters
  *
  */
void
NvBootSdramInit(const NvBootSdramParams *ParamData);


/**
  * Initializes the SDRAM controller according to the given configuration
  * parameters.  Intended for warm boot0 boot use.
  *
  * Generally the steps are --
  * 1. Program and enable PLL
  * 2. Wait for the PLL to get locked
  * 3. Enable clock and disable reset to emc
  * 4. Write the desired EMC and MC registers in the correct sequence
  *
  * @param ParamData pointer to configuration parameters
  *
  */
void
NvBootSdramInitWarmBoot0(const NvBootSdramParams *ParamData);



/**
  * Deprecated APIs - DO NO USE
  * Return SDRAM size in bytes. Max SDRAM size is NV_ADDRESS_MAP_EMEM_MAX_SIZE.
  */

NvU64
NvBootSdramQueryTotalSize( NvBool );

/**
  * Detects and reports total amount of SDRAM memory installed on the system.
  * Size is reported as the total amount of SDRAM memory in MBs. This API
  * can be called only after SDRAM is initialized successfully.
  *
  * @retval  total amount of SDRAM memory in MBs
  * @retval 0 If the SDRAM was not initiazlied and we are not able write/read into/from SDRAM
  *
  */

NvU32
NvBootSdramQueryTotalMB ( NvBool );

/**
    // Function: 
    //  -Brings IOBRICK's out of DPD
    //  -Enables all MSS clocks and ROC clock
    //  -deasserts MSS resets

    // Prerequisites:
    // IO rails should be stable before this function is called
    // For warmBoot (preserveDram ==1):
    //  -*pData: unpacking of bct from PMC space needs to have been executed
    //  -IOBRICK's and CMD/RESET pads held in DPD, either via DPD_REQ or RAMDUMP FSM
    // For other boot variants (preserveDram == 0)
    //   -no assumptions made on pad DPD state.  proper DPD exit sequence will be followed
    //   *pData not required

    // preserveDram == 1: requires *pData pointer to BCT settings unpacked from PMC.  Required for warmBoot
    // preserveDram == 0: *pData is not used.  DRAM contents lost 
*/    
void NvBootEnableMemClk (const NvBootSdramParams *pData, NvBool preserveDram);

#if FIXME
#define NV_ULLC(c)         (c##ULL)
#define NV_U64C(c)        (NV_ULLC(c))
#endif


#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SDRAM_INT_H

