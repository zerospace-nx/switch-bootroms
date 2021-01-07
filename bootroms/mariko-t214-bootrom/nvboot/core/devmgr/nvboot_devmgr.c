/*
 * Copyright (c) 2011 - 2012 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/* New code */
/* Size: (last address, from 0)
 * Old Code: 0x4c8 (1224 bytes + extra ??)
 * New Code: 0x190 (400 bytes + extra 7 * 4 * 4 (112) bytes for table =
 *                  512 bytes) 
 */

#include "nvcommon.h"

#include "nvboot_config.h"
#include "nvboot_devmgr_int.h"
#include "nvboot_error.h"
#include "nvboot_context_int.h"
#include "nvboot_util_int.h"
#include "nvboot_devmgr_int.h"

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

/*
 * nvboot_devmgr.c - Implementation of the device manager and support code.
 */

/*
 * Function prototypes
 */
static NvBootError InitDevice(NvBootDevMgr *DevMgr, NvU32 ParamIndex);

/*
 * Static Data
 */
NvBootDevContext s_DeviceContext;

const NvBootDevMgrCallbacks s_DeviceCallbacks[] = 
{   
    //device type is as per the definition in t35/bootrom/include/t35/nvboot_bct.h

    /* NvBootDevType_None has no drivers */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    
    /* NvBootDevType_Nand has no drivers */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },

    /* NvBootDevType_Snor has no drivers */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    
    {
        /* Callbacks for the SPI Flash device */
        (NvBootDeviceGetParams)NvBootSpiFlashGetParams,
        (NvBootDeviceValidateParams)NvBootSpiFlashValidateParams,
        (NvBootDeviceGetBlockSizes)NvBootSpiFlashGetBlockSizes,
        (NvBootDeviceInit)NvBootSpiFlashInit,
        NvBootSpiFlashReadPage,
        NvBootSpiFlashQueryStatus,
        NvBootSpiFlashShutdown,
        NvBootSpiFlashGetReaderBuffersBase,
        NULL
    },
    {
        /* Callbacks for the SDMMC device */
        (NvBootDeviceGetParams)NvBootSdmmcGetParams,
        (NvBootDeviceValidateParams)NvBootSdmmcValidateParams,
        (NvBootDeviceGetBlockSizes)NvBootSdmmcGetBlockSizes,
        (NvBootDeviceInit)NvBootSdmmcInit,
        NvBootSdmmcReadPage,
        NvBootSdmmcQueryStatus,
        NvBootSdmmcShutdown,
        NvBootSdmmcGetReaderBuffersBase,
        NULL
    },

    /* NvBootDevType_Irom has no drivers */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },

    /* NvBootDevType_Uart has no drivers */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },

    /* NvBootDevType_Usb has no drivers */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },

    /* NvBootDevType_Nand_x16 has no drivers (uses Nand driver) */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    
    /* NvBootDevType_Usb3 has no drivers (uses Nand driver) */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    
    TODO // Uncomment when we have sata drivers
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    // {
        // /* Callbacks for the sata device */
        // (NvBootDeviceGetParams)NvBootSataGetParams,
        // (NvBootDeviceValidateParams)NvBootSataValidateParams,
        // (NvBootDeviceGetBlockSizes)NvBootSataGetBlockSizes,
        // (NvBootDeviceInit)NvBootSataInit,
        // NvBootSataReadPage,
        // NvBootSataQueryStatus,
        // NvBootSataShutdown,
        // NvBootSataGetReaderBuffersBase
    // },
    {
        /* Callbacks for Production Uart device */
        (NvBootDeviceGetParams)NvBootProdUartGetParams,
        (NvBootDeviceValidateParams)NvBootProdUartValidateParams,
        (NvBootDeviceGetBlockSizes)NvBootProdUartGetBlockSizes,
        (NvBootDeviceInit)NvBootProdUartInit,
        NvBootProdUartReadPage,
        NvBootProdUartQueryStatus,
        NvBootProdUartShutdown,
        NvBootProdUartGetReaderBuffersBase,
        NULL
    },

#if NVENABLE_FOOS_SUPPORT
    {
        /* Callbacks for foos device */
        (NvBootDeviceGetParams)NvBootFoosGetParams,
        (NvBootDeviceValidateParams)NvBootFoosValidateParams,
        (NvBootDeviceGetBlockSizes)NvBootFoosGetBlockSizes,
        (NvBootDeviceInit)NvBootFoosInit,
        NvBootFoosReadPage,
        NvBootFoosQueryStatus,
        NvBootFoosShutdown,
        NvBootFoosGetReaderBuffersBase
    },
#endif
};


/*
 * Function implementations
 */

/**
 * InitDevice(): Initialize the device.
 *
 * @param[in] DevMgr Pointer to the device manager structure
 * @param[in] ParamIndex Parameter index from the fuses.
 *
 * @retval NvBootError_Success The device was successfully initialized and
 * is ready for reading.
 * @retval TODO Error codes from GetParams() callback
 * @retval NvBootError_InvalidDevParams The device's ValidataParams callback
 * returned false.
 * @retval TODO Error codes from Init() callback
 * @retval TODO Error codes from GetBlockSizes() callback
 */
static NvBootError
InitDevice(NvBootDevMgr *DevMgr, NvU32 ParamIndex)
{
    NvBool                 ValidParams;
    NvBootError            e = NvBootError_Success;
    void                  *Params;

    NV_ASSERT(DevMgr != NULL);

    /* Query the boot parameters from the device. */
    DevMgr->Callbacks->GetParams(ParamIndex, &Params);

    /* Validate the parameters */
    ValidParams = DevMgr->Callbacks->ValidateParams(Params);
    if (ValidParams == NV_FALSE) return NvBootError_InvalidDevParams;

    /* Initialize the device. This must occur before querying the block sizes*/
    NV_BOOT_CHECK_ERROR(DevMgr->Callbacks->Init(Params, &s_DeviceContext));

    /* Query the block sizes. */
    DevMgr->Callbacks->GetBlockSizes(Params,
                                     &DevMgr->BlockSizeLog2,
                                     &DevMgr->PageSizeLog2);

    return NvBootError_Success;
}


/**
 * NvBootDevMgrReinitDevice(): Reinitialize the device with data from the
 * BCT, selected by fuses/straps.
 *
 * @param[in] DevMgr Pointer to the device manager
 * @param[in] BctParams An array of NvBootDevParams structures within the BCT.
 * @param[in] ParamCount The number of valid structures in the BctParams array.
 *
 * @retval NvBootError_Success The device was successfully reinitalized if
 * needed.
 * @retval NvBootError_InvalidDevParams The device's ValidataParams callback
 * returned false.
 * @retval TODO Error codes from Init() callback
 */
NvBootError
NvBootDevMgrReinitDevice(NvBootDevMgr    *DevMgr,
             NvBootDevParams *BctParams,
             const NvU8       ParamCount,
             const NvU8       DeviceStraps)
{
    NvBool                 ValidParams;
    NvBootError            e = NvBootError_Success;
    NvBootDevParams       *Params;

    NV_ASSERT(DevMgr != NULL);
    NV_ASSERT(BctParams != NULL);
    NV_ASSERT(DeviceStraps < NVBOOT_BCT_MAX_PARAM_SETS);

    /* Skip processing if there are no parameter sets in the BCT. */
    if (ParamCount == 0) return NvBootError_Success;

    /* Check that the index is in the range of valid parameter sets. */
    if (DeviceStraps >= ParamCount) return NvBootError_IllegalParameter;

    /* Select the parameters from the BCT. */
    Params = &(BctParams[DeviceStraps]);

    ValidParams = DevMgr->Callbacks->ValidateParams(Params);
    if (ValidParams == NV_FALSE) return NvBootError_InvalidDevParams;

    /* Initialize the device */
    NV_BOOT_CHECK_ERROR(DevMgr->Callbacks->Init(Params, &s_DeviceContext));

    return NvBootError_Success;
}

/**
 * NvBootDevMgrInit(): Initialize the device manager, select the boot
 * device, and initialize it.  Upon completion, the device is ready for
 * access.
 *
 * @param[in] DevMgr Pointer to the device manager
 * @param[in] DevType The type of device from the fuses.
 * @param[in] ConfigIndex The device configuration data from the fuses
 *
 * @retval NvBootErrorSuccess The device was successfully initialized.
 * @retval NvBootInvalidParameter The device type was not valid.
 * @retval TODO Error codes from InitDevice()
 */
NvBootError
NvBootDevMgrInit(NvBootDevMgr  *DevMgr,
                 NvBootDevType  DevType,
                 NvU32          ConfigIndex)
{
    NvBootError e;

    if (DevMgr == NULL) return NvBootError_InvalidParameter;

    if (DevType == NvBootDevType_None     ||
        DevType == NvBootDevType_Irom     ||
        DevType == NvBootDevType_Uart     ||
        DevType == NvBootDevType_Usb      ||
        DevType == NvBootDevType_Nand_x16)
        return NvBootError_InvalidParameter;

    /*
     * Assign the function pointers and device class.
     */
    DevMgr->Callbacks = &(s_DeviceCallbacks[DevType]);

    /* Initialize the device. */
    NV_BOOT_CHECK_ERROR_CLEANUP(InitDevice(DevMgr, ConfigIndex));

    /* Initialization is complete! */
    return NvBootError_Success;
    
 fail:
    DevMgr->Callbacks = NULL;

    return e;
}


/**
 * NvBootDevMgrShutdown(): Shutdown the device and the device manager.
 *
 * @param[in] DevMgr Pointer to the device manager.
 *
 * Upon completion, the device may no longer be accessible, and cannot
 * be accessed via the device manager.
 */
void
NvBootDevMgrShutdown(NvBootDevMgr *DevMgr)
{
    NV_ASSERT(DevMgr != NULL);

    /* Shutdown the device. */
    if (DevMgr->Callbacks->Shutdown != NULL)
    {
        DevMgr->Callbacks->Shutdown();
    }

    /* Shutdown the device manager. */
    DevMgr->Callbacks   = NULL;
}
