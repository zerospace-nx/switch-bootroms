/*
 * Copyright (c) 2006 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_strap_int.h
 *
 * Strap interface for NvBoot
 *
 * NvBootStrap is NVIDIA's interface for querying the boot strap settings.
 *
 */

#ifndef INCLUDED_NVBOOT_STRAP_INT_H
#define INCLUDED_NVBOOT_STRAP_INT_H

#include "nvcommon.h"

// Bug 1444822. Device configuration straps are not present in T124.
//#define STRAP_RAM_CODE_0_DEVICE_CONFIGURATION_RANGE 3:2
#define STRAP_RAM_CODE_0_SDRAM_CONFIGURATION_RANGE  1:0


#define NVBOOT_STRAP_DEVICE_CONFIGURATION_SHIFT (2)
#define NVBOOT_STRAP_DEVICE_CONFIGURATION_MASK (0x3)

#define NVBOOT_STRAP_SDRAM_CONFIGURATION_SHIFT (0)
#define NVBOOT_STRAP_SDRAM_CONFIGURATION_MASK (0x3)

// Define RCM_STRAPS fields
// Volume Up Button
#define APB_MISC_PP_STRAPPING_OPT_A_0_RCM_STRAPS_FORCE_RCM_RANGE                  10:10
// Volume Down Button
#define APB_MISC_PP_STRAPPING_OPT_A_0_RCM_STRAPS_DEBUG_RCM_RANGE                  11:11
// Home Button
#define APB_MISC_PP_STRAPPING_OPT_A_0_RCM_STRAPS_2_BUTTON_RCM_RANGE               12:12

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootStrapDevSel: Enumerated list of devices selectable via straps.
 *
 * Note: This should match //hw/ar/doc/t210/bootrom/T210_BootROM_strap.xls.
 */

/* Note: In T210, NvBootStrapDevSel_ExitRcm has been removed. We use RCM_STRAPS 
 * which are to connected to buttons volume up/down and home.
 */
typedef enum
{
    NvBootStrapDevSel_Emmc_x8_BootModeOff_26Mhz = 0x0, /* eMMC primary (x8), 512-byte page, 26Mhz */
    NvBootStrapDevSel_SpiFlash = 0x1,                  /* SPI Flash             */
    NvBootStrapDevSel_Sata = 0x2,                      /* Sata                  */
    NvBootStrapDevSel_UseFuses = 0x3,                  /* Use fuses; Only valid when PRODUCTION_MODE fuse is set */
    NvBootStrapDevSel_Usb3 = 0x4,                      /* USB3 */
    NvBootStrapDevSel_Rcm = 0x5,                       /* RCM */
    /* The following definitions must be last. */
    NvBootStrapDevSel_Num, /* Must appear after the last legal item */

    NvBootStrapDevSel_resvd1 = NvBootStrapDevSel_Rcm,

    /* Create a fail-safe strap value that we can fall back on.
     * This would be used for example, if out of bounds strap values
     * are detected. */
    NvBootStrapDevSel_Failsafe = NvBootStrapDevSel_Rcm,

    /* Uart Boot BOOT_SELECT value for use in Nv Preproduction mode */
    NvBootStrapDevSel_UartBoot_PreProduction = 0x3,

    NvBootStrapDevSel_Force32 = 0x7fffffff
} NvBootStrapDevSel;

/**
 * Reports whether the FORCE_USB_RECOVERY strap is set to force Recovery Mode.
 *
 * @return NV_TRUE if the Volume Up button is pressed, else NV_FALSE
 */
NvBool
NvBootStrapIsForceRecoveryMode(void);

/**
 * Reports whether the FORCE_DEBUG_RCM strap is set to force Recovery Mode.
 *
 * @return NV_TRUE if the Volume Down button is pressed, else NV_FALSE
 */
NvBool
NvBootStrapIsDebugRecoveryMode(void);

/**
 * Reports whether the strap is set to Production Uart Mode.
 *
 * @return NV_TRUE if the strap is pulled high, else NV_FALSE
 */
NvBool
NvBootStrapIsProdUartMode(void);

/**
 * Reports the boot device configuration index.  This index is used to select
 * one of the parameter sets defined in the BCT.
 *
 * There are two straps dedicated to this purpose, so one of four values can
 * be specified.  There are no internal passive pull-ups/downs for these
 * straps, so the value will be undefined in the absence of externals pull-
 * ups/downs.
 *
 * @returns an NvU8 contains the value 0, 1, 2, or 3, depending on the strap
 *          setting
 */
NvU8
NvBootStrapDeviceConfigurationIndex(void);

/**
 * Reports the SDRAM configuration index.  This index is used to select one of 
 * the parameter sets defined in the BCT.
 *
 * There are two straps dedicated to this purpose, so one of four values can
 * be specified.  There are no internal passive pull-ups/downs for these
 * straps, so the value will be undefined in the absence of externals pull-
 * ups/downs.
 *
 * @returns an NvU8 contains the value 0, 1, 2, or 3, depending on the strap
 *          setting
 */
NvU8
NvBootStrapSdramConfigurationIndex(void);

/**
 * Reports the Device Selection strap value.  This selects the device from
 * which to attempt booting, provided that the value is not overriden by
 * fuse data.
 *
 * There are four straps dedicated to this purpose, so one of sixteen 
 * values can be specified.  There are no internal passive pull-ups/downs
 * for these straps, so the value will be undefined in the absence of
 * external pull-ups/downs.
 *
 * @returns The NvBootStrapDevSel represented by the corresponding straps.
 */
NvBootStrapDevSel
NvBootStrapDeviceSelection(void);

/**
 * Aliases the strap values based on PMC Scratch register data.
 * Only invoked during pre-production booting when the
 * AliasStraps flag is set in PMC Scratch register 0.
 */  
void
NvBootStrapAliasStraps(void);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_STRAP_INT_H
