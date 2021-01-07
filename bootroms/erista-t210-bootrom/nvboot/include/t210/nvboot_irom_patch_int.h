/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_IROM_PATCH_INT_H
#define INCLUDED_NVBOOT_IROM_PATCH_INT_H

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Read BootROM patch from fuse. Apply Hamming ECC correction on the patch.
 * If patch contains CAM entries, fill them up. If patch contain additional
 * instrucion words for SWI handler, setup the handler in IRAM.
 *
 * @param None
 *
 * @return None
 */
void NvBootApplyIRomPatch(void);

/**
 * Perform cleanup before BootRom exit.
 * - Restore CAMs to defefault
 * - Restore EVP for SWI
 */
void NvBootIRomPatchCleanup(void);

/**
 * Return 32-bit word for SW CYA implemetation. Only lower 16-bit
 * are valid because IPATCH CAM works on 16-bit chunk.
 */
NvU32 NvBootGetSwCYA(void);

/**
 * Define T35 BootROM SW CYA
 *
 * Example: #define NVBOOT_SW_CYA_{name}	(  1 << [15...0] >
 */

// Unused bit.
#define NVBOOT_SW_CYA_0_UNUSED (1<<0)

// AVP fast clock (216MHz) disable
#define NVBOOT_SW_CYA_AVP_FAST_CLOCK_DISABLE (1 << 1)


// Disable greater than 16K data read.
#define NVBOOT_SW_CYA_SATA_32KB_PAGE_SIZE_SUPPORT_DISABLE (1<<2)


// NVBOOT_SW_CYA_WAIT_ON_LOWBATT_ENABLE = 1, wait on Low Batt
// NVBOOT_SW_CYA_WAIT_ON_LOWBATT_ENABLE = 0, default.
#define NVBOOT_SW_CYA_WAIT_ON_LOWBATT_ENABLE (1<<3)

//USBf  for manual PLL programming
#define NVBOOT_SW_CYA_USBF_PLL_MANUAL_ENABLE (1 << 4)

// PLL divider reset (Bug 954159)
#define NVBOOT_SW_CYA_DIVRST_DISABLE    (1 << 5)

// Disable AHB coherency checking.
// NVBOOT_SW_CYA_AHB_COHERENCY_DISABLE = 0 means coherency check enabled.
// NVBOOT_SW_CYA_AHB_COHERENCY_DISABLE = 1 means coherency check disabled.
#define NVBOOT_SW_CYA_AHB_COHERENCY_DISABLE  (1 << 6)

// Disable OSC_CLK input for UTMIP. Use PLLU instead.
#define NVBOOT_SW_CYA_USE_PLLU_SRC_UTMIP (1 << 7)

// PLL auto-restart CYAs. PLLx_OVERRIDE_ENABLE
// NVBOOT_SW_CYA_PLLP_OVERRIDE_ENABLE = 0 means set PLLP_OVERRIDE_ENABLE to 0
// NVBOOT_SW_CYA_PLLP_OVERRIDE_ENABLE = 1 means set PLLP_OVERRIDE_ENABLE to 1
// NVBOOT_SW_CYA_PLLM_OVERRIDE_ENABLE = 0 means set PLLM_OVERRIDE_ENABLE to 0
// NVBOOT_SW_CYA_PLLM_OVERRIDE_ENABLE = 1 means set PLLM_OVERRIDE_ENABLE to 1
// NVBOOT_SW_CYA_PLLU_OVERRIDE_ENABLE = 0 means set PLLU_OVERRIDE_ENABLE to 0
// NVBOOT_SW_CYA_PLLU_OVERRIDE_ENABLE = 1 means set PLLU_OVERRIDE_ENABLE to 1
#define NVBOOT_SW_CYA_PLLP_OVERRIDE_ENABLE (1 << 8)
#define NVBOOT_SW_CYA_PLLM_OVERRIDE_ENABLE (1 << 9)
#define NVBOOT_SW_CYA_PLLU_OVERRIDE_ENABLE (1 << 10)

// Fallback mechanism: Workaround for bug 748271(fixed in T210 but not validated)
// This may be necessary for GEN3 drives to be detected as GEN2
#define NVBOOT_SW_CYA_SATA_WAR_BUG748271_EN (1 << 11)

// Bug 819194 : Bootrom skips loading last page of the bootloader/bct when number of bytes 
//              to be loaded is exact multiple of a page size.
// NVBOOT_SW_CYA_BUGFIX_819194_DISABLE = 0, enable bug fix
// NVBOOT_SW_CYA_BUGFIX_819194_DISABLE = 1, ignore bug fix. This also requires change in set.c file under buildimage in git repository

#define  NVBOOT_SW_CYA_BUGFIX_819194_DISABLE (1 << 12)

// Unused bit.
#define NVBOOT_SW_CYA_13_UNUSED (1<<13)

// Unused bit.
#define NVBOOT_SW_CYA_14_UNUSED (1<<14)

// NVBOOT_SW_CYA_WDT_RST_DISABLE = 1 , do not issue second reset
// NVBOOT_SW_CYA_WDT_RST_DISABLE = 0 , issue second reset
#define NVBOOT_SW_CYA_WDT_RST_DISABLE (1<<15)




#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_NVBOOT_IROM_PATCH_INT_H
