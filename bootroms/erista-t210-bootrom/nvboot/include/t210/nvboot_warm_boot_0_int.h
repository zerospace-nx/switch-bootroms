/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_warmboot0.h - Public Prototypes for WarmBoot0 functions.
 */


#ifndef INCLUDED_NVBOOT_WARM_BOOT_0_INT_H
#define INCLUDED_NVBOOT_WARM_BOOT_0_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * Performs WarmBoot0
 * Called from the main-line BROM code if Warm Boot0 flag is set
 * Sets up EMC/MC (initializes external memory) in warm boot0.
 * Verifies integrity of a small portion of LP0 exit code in SDRAM
 * Uses AES engine for checksum computation. Returns an error if
 * checksum does match value retrieved from PMC scratch register.
 * Also starts PLLC in the beginning and waits till it is stable before
 * returning NvBootError_Success if the checksum matches.
 *
 * @param BootRomExitTarget pointer to a 32-bit variable that will 
 *        hold the address to which AVP will jump if WB0 is a success
 *
 * @retval NvBootError_Success Warm Boot0 processed successfully
 * @retval NvBootError_HashMismatch Warm Boot0 restore code checksum failed
 * @retval NvBootError_IllegalParameter AVP jump address is invalid, i.e,
 *         is not inside both (1) the SDRAM address space and (2) the
 *         recovery code instructions that were validated by checksum
 */

NvBootError
NvBootWb0WarmBoot(NvU32 *BootRomExitTarget);

/**
 * Extracts all the stored bit fields from PMC scratch registers
 * Constructs the EMC or MC register values from these and other fixed
 * values and stores them to a structure of type "NvBootSdramParams".
 *
 * @param pData Pointer to an NvBootSdramParams structure.
 *
 * @retval NvBootErrorSuccess The data was successfully unpacked.
 * @retval NvBootInvalidParameter The pointer to SDRAM parameters was invalid.
 * TODO: Document which other error codes are used.
 */
NvBootError
NvBootWb0UnpackSdramParams(NvBootSdramParams *pData);

/**
 * Packs the necessary data from an SDRAM parameter structure into the
 * bit fields in the PMC scratch registers.
 *
 * @param pData Pointer to an NvBootSdramParams structure.
 *
 * @retval none
 */
void
NvBootWb0PackSdramParams(NvBootSdramParams *pData);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_WARM_BOOT_0_INT_H */


