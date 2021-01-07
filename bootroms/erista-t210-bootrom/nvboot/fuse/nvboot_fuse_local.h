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
 * @file nvboot_fuse_local.h
 *
 * Internal definitions used by fuse
 *
 */

#ifndef INCLUDED_NVBOOT_FUSE_LOCAL_H
#define INCLUDED_NVBOOT_FUSE_LOCAL_H

#define FUSE_BOOT_DEVICE_INFO_0_BOOT_DEVICE_CONFIG_RANGE 13:0
#define FUSE_RESERVED_SW_0_BOOT_DEVICE_SELECT_RANGE       2:0
#define FUSE_RESERVED_SW_0_SKIP_DEV_SEL_STRAPS_RANGE      3:3
#define FUSE_RESERVED_SW_0_ENABLE_CHARGER_DETECT_RANGE    4:4
#define FUSE_RESERVED_SW_0_ENABLE_WATCHDOG_RANGE          5:5
#define FUSE_RESERVED_SW_0_2_BUTTON_RCM_RANGE             6:6
#define FUSE_RESERVED_SW_0_RCM_PORT_RANGE                 7:7
#define FUSE_RESERVED_PRODUCTION_0_SATA_RX_OOB_BOOT_CALIB_DISABLE_RANGE  0:0

#endif // INCLUDED_NVBOOT_FUSE_LOCAL_H
