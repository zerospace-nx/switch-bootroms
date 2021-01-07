/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_devmgr_int.h - Definition of the device manager interface.
 */

#ifndef INCLUDED_NVBOOT_DEVMGR_INT_H
#define INCLUDED_NVBOOT_DEVMGR_INT_H

#include "nvtypes.h"
#include "nvboot_bit.h"
#include "nvboot_error.h"
#include "nvboot_device_int.h"
#include "nvboot_devparams.h"
/* Include the context data of the drivers. */
#include "nvboot_sdmmc_context.h"
#include "nvboot_snor_context.h"
#include "nvboot_spi_flash_context.h"

/* Include the public header for the drivers. */
#include "nvboot_sdmmc_int.h"
#include "nvboot_snor_int.h"
#include "nvboot_spi_flash_int.h"
#include "nvboot_sata_int.h"
#include "nvboot_foos_int.h"
#if NVENABLE_XUSB_SUPPORT
#include "nvboot_usb3_int.h"
#endif
#include "nvboot_prod_uart_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * Data types
 */
typedef union
{
#if NVENABLE_XUSB_SUPPORT
    NvBootUsb3Context		   Usb3Context;
#endif
#if NVENABLE_NAND_SUPPORT
    NvBootNandContext          NandContext;
#endif
    NvBootSdmmcContext         SdmmcContext;
    NvBootSnorContext          SnorContext;
    NvBootSpiFlashContext      SpiFlashContext;
    NvBootSataContext    SataContext;
    NvBootProdUartContext      ProdUartContext;
} NvBootDevContext;

typedef struct NvBootDevMgrCallbacksRec
{
    NvBootDeviceGetParams      GetParams;
    NvBootDeviceValidateParams ValidateParams;
    NvBootDeviceGetBlockSizes  GetBlockSizes;
    NvBootDeviceInit           Init;
    NvBootDeviceRead           Read;
    NvBootDeviceQueryStatus    QueryStatus;
    NvBootDeviceShutdown       Shutdown;
    NvBootDeviceGetReaderBuffersBase GetReaderBuffersBase;
    NvBootDevicePinMuxInit     PadCtrlPinMux;	
} NvBootDevMgrCallbacks;

/*
 * Device Callbacks definition (silent init to 0)
 * Real definition exists in device driver.
 */
NvBootDevMgrCallbacks NoneDeviceCallback;
NvBootDevMgrCallbacks SpiDeviceCallback;
NvBootDevMgrCallbacks SdmmcDeviceCallback;
NvBootDevMgrCallbacks Usb3DeviceCallback;
NvBootDevMgrCallbacks SataDeviceCallback;
NvBootDevMgrCallbacks UfsDeviceCallback;
NvBootDevMgrCallbacks FoosDeviceCallback;
NvBootDevMgrCallbacks ProdUartDeviceCallback;

/*
 * NvBootDevMgr: State & data used by the device manager.
 */
typedef struct NvBootDevMgrRec
{
    /* Device Info */
    NvU32                   BlockSizeLog2;
    NvU32                   PageSizeLog2;
    const NvBootDevMgrCallbacks  *Callbacks;    /* Callbacks to the chosen driver. */
} NvBootDevMgr;


/*
 * Functions
 */

/*
 * NvBootDevMgrInit(): Initialize the device manager and the desired device.
 * DevType selects the type of device, and is assumed to come from fuses.
 * ConfigIndex selects the parameter configuration to use, and is also
 * expected to come from fuses.
 */
NvBootError
NvBootDevMgrInit(NvBootDevMgr  *DevMgr, 
                 NvBootDevType  DevType,
                 NvU32          ConfigIndex);

/*
 * NvBootDevMgrReinitDevice(): Reinitialize the device based on parameter
 * data contained within the BCT.
 */
NvBootError
NvBootDevMgrReinitDevice(NvBootDevMgr    *DevMgr,
			 NvBootDevParams *BctParams,
			 const uint8_t       ParamCount,
                         const uint8_t       DeviceStraps);

/*
 * NvBootDevMgrShutdown(): Shutdown the device and the device manager.
 */
void NvBootDevMgrShutdown(NvBootDevMgr *State);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_DEVMGR_INT_H */
