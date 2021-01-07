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

#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_sdram_param.h"
// arclk_rst.h is needed by nvboot_wb0_sdram_pack. and nvboot_wb0_sdram_unpack.c.
// Those two files include nvboot_warm_boot_0_int.h. Avoids having to ask
// MC team to include arclk_rst.h into those two files.
#include "arclk_rst.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Halt at wb0 if pmc flag set. Can only be
 * enabled on pre-production chips.
 */
NvBootError NvBootHaltAtWarmboot(void);

/**
 * Unpack Sdram params. Start PLLM if required.
 */
NvBootError NvBootWarmBootUnPackSdramStartPllm(void);

/**
 * Authentication of the SC7 header + firmware.
 */
NvBootError NvBootWarmBootOemProcessRecoveryCode(void);

/**
 * Warmboot sdram init.
 */
NvBootError NvBootWarmBootSdramInit(void);

/**
 * Trigger TZRAM restore from AO TZRAM
 */
NvBootError NvBootWb0TzramInit(void);

/**
 * Start SC7 resume.
 * - Clear SC7 flag.
 */
NvBootError NvBootWb0Start(void);

/**
 * Read the address where the SC7 header and firmware from the scratch
 * register.
 * Sanitize the address from the scratch register - it must be in
 * TZRAM or SDRAM range.
 * Copy the header and firmware to a fixed location in IRAM (IRAM-B).
 *
 */
NvBootError NvBootWb0CopyHeaderAndFirmware(void);

/**
 *
 */
NvBootError NvBootWarmBootOemProcessRecoveryCode(void);


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
 * The SC7 firmware exit target is stored in NvBootContext.BootLoader.
 *
 * @retval NvBootError_Success Warm Boot0 processed successfully
 * @retval NvBootError_HashMismatch Warm Boot0 restore code checksum failed
 * @retval NvBootError_IllegalParameter AVP jump address is invalid, i.e,
 *         is not inside both (1) the SDRAM address space and (2) the
 *         recovery code instructions that were validated by checksum
 */

NvBootError
NvBootWb0WarmBoot(void);

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


