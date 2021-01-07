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
 * @file nvboot_fuse.h
 *
 * Defines the parameters and data structures related to fuses.
 */

#ifndef INCLUDED_NVBOOT_FUSE_H
#define INCLUDED_NVBOOT_FUSE_H

#include "nvtypes.h"
#if defined(__cplusplus)
extern "C"
{
#endif

#define NVBOOT_DEVICE_KEY_BYTES (4)

typedef struct NvBootECIDRec
{
    NvU32   ECID_0;
    NvU32   ECID_1;
    NvU32   ECID_2;
    NvU32   ECID_3;
} NvBootECID;

//DRF defines for ECID fields.

//Bootrom is defining 128 bits i.e. 4 x 32 bits, out of which
// a)100 bits make ECID for non RCM modes
// b)128 bits make ECID for RCM mode

//<Lot1:2><Wafer:6><X:9><Y:9><Reserved:6>
#define                 ECID_ECID0_0_RSVD1_RANGE        5:0
#define                 ECID_ECID0_0_Y_RANGE            14:6
#define                 ECID_ECID0_0_X_RANGE            23:15
#define                 ECID_ECID0_0_WAFER_RANGE        29:24
#define                 ECID_ECID0_0_LOT1_RANGE         31:30
//<Lot0:6><Lot1:26>
#define                 ECID_ECID1_0_LOT1_RANGE         25:0
#define                 ECID_ECID1_0_LOT0_RANGE         31:26
//<Fab:6><Lot0:26>
#define                 ECID_ECID2_0_LOT0_RANGE         25:0
#define                 ECID_ECID2_0_FAB_RANGE          31:26
//<operating mode:4><rcm version:16><Reserved:8><Vendor:4>
#define                 ECID_ECID3_0_VENDOR_RANGE       3:0
// #define                 ECID_ECID3_0_RSVD2_RANGE        11:4
// #define                 ECID_ECID3_0_RCM_VERSION_RANGE  27:12 // Deprecated for T214 (see below);
// #define                 ECID_ECID3_0_OPMODE_RANGE       31:28   // Deprecated for T214 (see below); Valid for RCM mode only

/**
From ISS section 6.2.4.5.
[103:100]
T214: APB_MISC_GP_HIDREV_0_MAJORREV
T194: MISCREG_HIDREV_0_MAJORREV
[111:104]
T214: APB_MISC_GP_HIDREV_0_CHIPID
T194: MISCREG_HIDREV_0_CHIPID
[115:112]
T214: APB_MISC_GP_MINORREV
T194: MISCREG_HIDREV_0_MINORREV
[122:116]
Reserved
[123]
FUSE_RESERVED_PRODUCTION_0[3]
[126:124]
BOOT_SECURITY_INFO[2:0]
[127]
FUSE_PRODUCTION_MODE
*/
#define                 ECID_ECID3_0_MAJORREV_RANGE 7:4
#define                 ECID_ECID3_0_CHIPID_RANGE 15:8
#define                 ECID_ECID3_0_MINORREV_RANGE 19:16
#define                 ECID_ECID3_0_BOOT_SECURITY_INFO_RANGE 30:28
#define                 ECID_ECID3_0_PRODUCTION_MODE_RANGE 31:31

/*
 * NvBootFuseBootDevice -- Peripheral device where Boot Loader is stored
 *
 * This enum *MUST* match the equivalent list in nvboot_devmgr.h for
 * all valid values and None, Undefined not present in nvboot_devmgr.h
 */
typedef enum
{
    NvBootFuseBootDevice_Sdmmc,
    NvBootFuseBootDevice_SpiFlash,
    NvBootFuseBootDevice_Sata,
    NvBootFuseBootDevice_resvd_4,
    NvBootFuseBootDevice_Foos = NvBootFuseBootDevice_resvd_4,
    NvBootFuseBootDevice_Usb3,
    NvBootFuseBootDevice_ProdUart,
    NvBootFuseBootDevice_Max, /* Must appear after the last legal item */
    NvBootFuseBootDevice_Force32 = 0x7fffffff
} NvBootFuseBootDevice;

/*
 * Definitions of device fuse fields
 */

/**
 * SDMMC configuration fuses.
 */

/**
 * Fuse Bit 2,1,0: Represents the config option supported by the driver.
 */
#define SDMMC_DEVICE_CONFIG_0_DEV_CONFIG_RANGE 2:0

/**
 * Fuse Bit 1,0: Represents the config option supported by the driver.
 */
#define SPI_DEVICE_CONFIG_0_DEV_CONFIG_RANGE 1:0

/**
 * USBH configuration fuses.
 */

/**
 * Bit 0,1,2,3 -->  used for representing the root port number device is attached to the
 * usb host controller.
 */
#define USBH_DEVICE_CONFIG_0_ROOT_PORT_RANGE 3:0

/**
 * Bit 4 --> 0 = 2k page size, 1 = 16k page size
 */
#define USBH_DEVICE_CONFIG_0_PAGE_SIZE_2KOR16K_RANGE 4:4

/**
 * Bit 5,6,7--->  used to represent to program the OC pin or OC group this port.
 */
#define USBH_DEVICE_CONFIG_0_OC_PIN_RANGE 7:5

/**
 * Bit 8 --> 0 = pad 0, 1 = pad 1, used to represent tri-state of the associated vbus pad.
 */
#define USBH_DEVICE_CONFIG_0_VBUS_ENABLE_RANGE 8:8

/**
 * Ufs configuration fuses.
 */

/**
 * Fuse Bit 1,0: Represents the config option supported by the driver.
 */
#define UFS_FUSE_PARAMS_0_DEV_CONFIG_RANGE       1:0


/*
 * Secure Provisioning Fuses
 */
#define FUSE_SECURE_PROVISION_INFO_0_KEY_HIDE_RANGE 0:0
#define FUSE_SECURE_PROVISION_INFO_0_TEST_PART_RANGE 1:1

/*
 * Boot info security fuses, for information on authentication and confidentiality
 * schemes.
 */
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHIFT    _MK_SHIFT_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_FIELD    _MK_FIELD_CONST(0x3, FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_RANGE    1:0
// DEFAULT in this case is also AES-CMAC.
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_DEFAULT  _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_AES_CMAC _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA  _MK_ENUM_CONST(1)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_RESERVED1  _MK_ENUM_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_RESERVED2  _MK_ENUM_CONST(3)

#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_SHIFT        _MK_SHIFT_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_FIELD        _MK_FIELD_CONST(0x1, FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_RANGE        2:2
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_DEFAULT      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_DISABLE      _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_ENABLE       _MK_ENUM_CONST(1)

#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_SHIFT _MK_SHIFT_CONST(3)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_FIELD  _MK_FIELD_CONST(0x1, FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_RANGE 3:3
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_DEFAULT _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_DISABLE _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_ENABLE _MK_ENUM_CONST(1)

#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_SHIFT _MK_SHIFT_CONST(4)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_FIELD _MK_FIELD_CONST(0x7, FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_SHIFT)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_RANGE      6:4
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_DEFAULT    _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_TEST_KEY   _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_NVIDIA_KEY _MK_ENUM_CONST(1)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_0  _MK_ENUM_CONST(2)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_1  _MK_ENUM_CONST(3)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_2  _MK_ENUM_CONST(4)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_3  _MK_ENUM_CONST(5)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_4  _MK_ENUM_CONST(6)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_5  _MK_ENUM_CONST(7)
#define FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_SELECT_OEM_KEY_6  _MK_ENUM_CONST(8)

#define FUSE_BOOT_SECURITY_INFO_0_SE_ATOMIC_CONTEXT_SAVE_ENABLE_RANGE 7:7
#define FUSE_BOOT_SECURITY_INFO_0_SE_ATOMIC_CONTEXT_SAVE_ENABLE_DEFAULT _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_SE_ATOMIC_CONTEXT_SAVE_ENABLE_DISABLE _MK_ENUM_CONST(0)
#define FUSE_BOOT_SECURITY_INFO_0_SE_ATOMIC_CONTEXT_SAVE_ENABLE_ENABLE _MK_ENUM_CONST(1)

#define FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_SHIFT    _MK_SHIFT_CONST(2)
#define FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_FIELD    _MK_FIELD_CONST(0x1, FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_SHIFT)
#define FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_RANGE    2:2
#define FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_DEFAULT    _MK_ENUM_CONST(0)
#define FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_BANK_0    _MK_ENUM_CONST(0)
#define FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_BANK_1    _MK_ENUM_CONST(1)

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_FUSE_H
