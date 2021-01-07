/*
 * Copyright (c) 2011 - 2012 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvcommon.h"
#include "nvboot_bct.h"
#include "nvboot_bct_int.h"
#include "nvboot_bit.h"
#include "nvboot_bootloader_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_coldboot_int.h"
#include "nvboot_config.h"
#include "nvboot_config_int.h"
#include "nvboot_context_int.h"
#include "nvboot_devmgr_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_sdram_int.h"
#include "nvboot_strap_int.h"
#include "nvboot_util_int.h"
#include "nvboot_hardware_access_int.h"
#include "arclk_rst.h"
#include "nvrm_drf.h"
#include "nvcommon.h"

extern NvBootInfoTable   BootInfoTable;
extern NvBootConfigTable BootConfigTable;
extern NvBootConfigTable *pBootConfigTable;

extern NvBootContext     Context;

/* Function Prototypes */
static NvBootError SetupBootDevice(NvBootContext *Context);
// static void        MapFusesToCryptoOps(NvBootContext *Context);
static NvBootError SetupSdram(NvBootContext *Context);

/**
 *  Attempt to disambiguate how a strap maps to a device.
 *  Strap (type NvBootStrapDevSel)
 *      |__ StrapDevSelMap[Strap] (type NvBootFuseBootDevice)
 *                              |__ MapFuseDevToBitDevType[StrapDevSelMap[Strap]] (type NvBootDevType)
 *  Strap=Rsvd < NvBootStrapDevSel_Num
 *       |__ StrapDevSelMap[Strap]=NvBootFuseBootDevice_resvd_4
 *                                  |__MapFuseDevToBitDevType[NvBootFuseBootDevice_resvd_4]= NvBootDevType_None
 *  
 *  Strap=Emmc < NvBootStrapDevSel_Num
 *       |__ StrapDevSelMap[Strap]=NvBootFuseBootDevice_Sdmmc
 *                                  |__MapFuseDevToBitDevType[NvBootFuseBootDevice_Sdmmc]= NvBootDevType_Sdmmc
 *
 */
static const NvBootFuseBootDevice StrapDevSelMap[] =
{
    NvBootFuseBootDevice_Sdmmc,         // NvBootStrapDevSel_Emmc_x8_BootModeOff
    NvBootFuseBootDevice_SpiFlash,      // NvBootStrapDevSel_SpiFlash
    NvBootFuseBootDevice_resvd_4,       // Reserved, previously NvBootStrapDevSel_Sata
    NvBootFuseBootDevice_Sdmmc,         // 0x3 = Use Fuses when PRODUCTION_MODE fuse is set. 
    NvBootFuseBootDevice_resvd_4,       // Reserved, previously NvBootStrapDevSel_Usb3
                                        // Strap-> Fuse mapping not done for this strap.
#if NVENABLE_FOOS_SUPPORT
    NvBootFuseBootDevice_Foos,          // NvBootStrapDevSel_foos
#endif
    NvBootFuseBootDevice_resvd_4,       // NvBootStrapDevSel_Rcm, Convoluted way of forcing rcm
                                        // through device none.
};

NV_CT_ASSERT((sizeof(StrapDevSelMap)/sizeof(NvBootFuseBootDevice)) ==
             NvBootStrapDevSel_Num);

#define SDMMC_CONFIG(Sdmmc_Config) \
    (((Sdmmc_Config) << 0))


#define SPI_CONFIG(Spi_Config) \
    ((Spi_Config) << 0)

#define XUSB_CONFIG(VBUS_number, OCPin, PortNumber) \
    ((VBUS_number << 8) | (OCPin << 5) | ((PortNumber) << 0))


static const NvU32 StrapDevConfigMap[] =
{
    SDMMC_CONFIG(0),           // NvBootStrapDevSel_Emmc_BMOff_x8
    SPI_CONFIG(0),              // NvBootStrapDevSel_SpiFlash : Page size 2K
    0,                          // reserved
    SDMMC_CONFIG(2),           // NvBootStrapDevSel_UseFuses; Set default config to EMMC x4, Boot Mode OFF SinglePage, SDR 25Mhz for safety.
                                // This is the same config as when BOOT_DEVICE_INFO and FUSE_RESERVED_SW[2:0] are
                                // unfused.
    XUSB_CONFIG(1, 5, 1),       // reserved
#if NVENABLE_FOOS_SUPPORT
    0, //foos
#endif
    0,                          // NvBootStrapDevSel_Rcm 0x00
};

NV_CT_ASSERT((sizeof(StrapDevConfigMap)/sizeof(NvU32)) ==
             NvBootStrapDevSel_Num);

static const NvBootDevType MapFuseDevToBitDevType[] = 
{
    // Fuse to device type as specified in t35/bootrom/include/t35/nvboot_bct.h
    NvBootDevType_Sdmmc,        // NvBootFuseBootDevice_Sdmmc
    NvBootDevType_Spi,          // NvBootFuseBootDevice_SpiFlash
    NvBootDevType_None,         // Reserved
#if NVENABLE_FOOS_SUPPORT
    NvBootDevType_Foos,         // Reserved
#else
    NvBootDevType_None,         // Reserved
#endif
    NvBootDevType_None,         // Reserved
    NvBootDevType_ProdUart,
};

NV_CT_ASSERT((sizeof(MapFuseDevToBitDevType)/sizeof(NvBootDevType)) ==
             NvBootFuseBootDevice_Max);

/**
 * SetupBootDevice(): Read boot device information from fuses, convert their
 *     values as needed, and initialize the device manager (which, in turn,
 *     will initialize the boot device).
 *
 * @param[in] Context Pointer to the boot context.
 *
 * @retval NvBootError_Success The boot device is ready for reading.
 *
 * Errors from NvBootFuseGetBootDevice:
 * @retval NvBootError_InvalidBootDeviceEncoding The device selection fuses
 * could not be mapped to a valid value.
 *
 * @retval TODO Errors from NvBootDevMgrInit
 */
static NvBootError SetupBootDevice(NvBootContext *Context)
{
    NvU32                ConfigIndex = 0;
    NvBootFuseBootDevice FuseDev = NvBootFuseBootDevice_Sdmmc;
    NvBootDevType        DevType = NvBootDevType_Sdmmc;
    NvBootError          e;
    NvBootStrapDevSel    StrapDevSel = NvBootStrapDevSel_Emmc;

    NV_ASSERT(Context != NULL);
    /* Determine the correct boot device selection & configuration. */

    /* Start by reading the device fuses and straps. */
    NvBootFuseGetBootDevice(&FuseDev);
    NvBootFuseGetBootDeviceConfiguration(&ConfigIndex);
    StrapDevSel = NvBootStrapDeviceSelection();

    /* Determine if the fuse data should be overwritten by strap data. */
    if (!NvBootFuseSkipDevSelStraps() &&
        StrapDevSel != NvBootStrapDevSel_UseFuses)
    {
        /* Handle out-of-bound values */
        /* Convoluted way of entering RCM
         * NvBootStrapDevSel_Rcm indexed into StrapDevSelMap gives NvBootFuseBootDevice_resvd_4
         * NvBootFuseBootDevice_resvd_4 indexed into MapFuseDevToBitDevType 
         * gives NvBootDevType_None.
         */
        if (StrapDevSel >= NvBootStrapDevSel_Num)
            StrapDevSel = NvBootStrapDevSel_Rcm;

        FuseDev     = StrapDevSelMap   [StrapDevSel];
        ConfigIndex = StrapDevConfigMap[StrapDevSel];
    }

    /* Production UART strap isn't affected by Ignore Straps */
    if (NvBootStrapIsProdUartMode() && NvBootFuseIsNvProductionMode())
    {
        FuseDev = NvBootFuseBootDevice_ProdUart;
    }

    /*
     * Record the boot device type.  Note that the NvBootDevType values
     * do not match the NvBootFuseBootDevice values for backward
     * compatibility with AP15, so they must be mapped.
     */

    DevType = MapFuseDevToBitDevType[FuseDev];

    BootInfoTable.SecondaryDevice = DevType;

    /* Initialize the device manager. */
    NV_BOOT_CHECK_ERROR(NvBootDevMgrInit(&(Context->DevMgr),
                                         DevType,
                                         ConfigIndex));
    return NvBootError_Success;
}

/**
 * SetupSdram(): Initialize the SDRAM.
 *
 * @param[in] Context Pointer to the boot context.
 *
 * @retval NvBootError_Success SDRAM is ready for use or there were no
 * SDRAM sets provided.
 * @retval NvBootError_IllegalParameter Strap value for SDRAM parameter
 * selection >= the number of sets in the BCT.
 * @retval TODO Errors from NvBootSdramInitFromParams
 * @retval TODO Errors from NvBootSdramQueryInitStatus
 */
static NvBootError SetupSdram(NvBootContext *Context)
{
    NvU8 SdramIndex = 0;
    NvU32 StableTime, Misc1, Misc2;
    NvBootSdramParams *Params = NULL;
    NvU32 RegData = 0;
    (void)Context;

    NV_ASSERT(Context != NULL);

    /* Do nothing if there are no SDRAM sets in the BCT. */
    if (pBootConfigTable->NumSdramSets == 0)
        return NvBootError_Success;

    SdramIndex = NvBootStrapSdramConfigurationIndex();

    if (SdramIndex >= pBootConfigTable->NumSdramSets)
    {
        return NvBootError_IllegalParameter;
    }
    
    Params = &(pBootConfigTable->SdramParams[SdramIndex]);

    // Only start PLLM if required
    RegData = NV_DRF_VAL(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_EMC,
                                 EMC_2X_CLK_SRC,
                                 Params->EmcClockSource);
    
    if((RegData == CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_UD)   ||
       (RegData == CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0))
    {
        /**
         * Set up PLLM auto-restart on WB0. Note Boot ROM is only
         * responsible for enabling the PLL auto-restart via PLLx_ENABLE. 
         * PLLx_OVERRIDE_ENABLE is the domain of the BL, OS, or LP0 entry code. 
         * Assumes OscFreq has the correct value
         * If PllU override is enabled, then do nothing. 
         */
        if (!NvBootPmcIsPllmOverrideEnabled()) 
        {
            NvBootPmcSetPllmOverride(NV_TRUE, Params);
        }

        //Pack PLLM params into Misc1 and Misc1
        Misc1 = NV_DRF_NUM(MISC1, CLK_RST_CONTROLLER_PLLM_MISC1, PLLM_SETUP, Params->PllMSetupControl);
        Misc2 = NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KVCO, Params->PllMKVCO) | \
                NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KCP, Params->PllMKCP);
        
        // Start PLLM for EMC/MC
        // TODO: Move PLL init & wait for reduced latency.
        NvBootClocksStartPll(NvBootClocksPllId_PllM, 
                             Params->PllMInputDivider,           // M
                             Params->PllMFeedbackDivider,        // N
                             Params->PllMPostDivider,            // P
                             Misc1,
                             Misc2,
                             &StableTime);

        // Poll for PLLM lock bit and timeout on polling loop
        while (!NvBootClocksIsPllStable(NvBootClocksPllId_PllM, StableTime));
    }

    /* Initialize the SDRAM. */
    NvBootSdramInit(Params);
    
    BootInfoTable.SdramInitialized = NV_TRUE;

    return NvBootError_Success;
}

/**
 * Perform Boot interface init operations needed for cold boot.
 * log Boot interface init time in tick count
 *
 * @retval NvBootError_Success, boot interface initialized for bct/bl reads.
 */
NvBootError NvBootColdBootInit()
{
    NvBootError e;
    NvU32 TickCount = 0;

    /* Record the boot type (will be overwritten upon fail over to RCM) */
    BootInfoTable.BootType = NvBootType_Cold;

    Context.BlFailBack = NvBootPmcQueryFlag(NvBootPmcFlagId_FailBack);
    if (Context.BlFailBack)
    {
        /* Clear the flag. */
        NvBootPmcSetFlag(NvBootPmcFlagId_FailBack, NV_FALSE);
        BootInfoTable.ClearedFailBack = NV_TRUE;
    }

    TickCount = NvBootUtilGetTimeUS();

    NV_BOOT_CHECK_ERROR_CLEANUP(SetupBootDevice(&Context));

    BootInfoTable.BootTimeLog.NvBootSetupTickCnt = NvBootUtilElapsedTimeUS(TickCount);
    BootInfoTable.BootROMtracker = NvBootFlowStatus_CBSetupBootDevice;

    BootInfoTable.DevInitialized = NV_TRUE;

 fail:
#if NVBOOT_SPIN_WAIT_AT_END
        NV_BOOT_SPIN_WAIT()
#endif

    return e;
}


 /*
 * Performs reading bct size and bct validation.
 * log Bct size and read times
 *
 * @retval NvBootError_Success Bct validation is successful.
 */
NvBootError NvBootColdBootReadBct()
{
    NvBootError e;
    NvU32 TickCount = 0;

    TickCount = NvBootUtilGetTimeUS();

    // BCT shall be read from the block/page ReadBCT shall invoke Full BCT.

    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootReadBct(&Context));

    // Enable Boot flow based on bct variables
    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootColdBootEnableBctFlags());

    BootInfoTable.BootTimeLog.NvBootReadBctTickCnt = NvBootUtilElapsedTimeUS(TickCount);
    BootInfoTable.BootROMtracker = NvBootFlowStatus_CBBctDone;

 fail:
#if NVBOOT_SPIN_WAIT_AT_END
        NV_BOOT_SPIN_WAIT()
#endif
        return e;


}

/**
 * Perform dram initialization.
 *
 * @retval NvBootError_Success, dram initialized.
 */
NvBootError NvBootColdBootSetupSdram()
{
    NvBootError e = NvBootError_Success;
   /*
     * Initialize DRAM before re-initializing secondary boot device.
     * This is required for secondary boot devices to use DRAM for all
     * operations.
     */
    NV_BOOT_CHECK_ERROR_CLEANUP(SetupSdram(&Context));
    BootInfoTable.BootROMtracker = NvBootFlowStatus_CBSdramInitSuccess;

 fail:
#if NVBOOT_SPIN_WAIT_AT_END
    NV_BOOT_SPIN_WAIT()
#endif
    return e;

}

/**
 * Perform Boot interface Re-init with Bct params.
 *
 * @retval NvBootError_Success, boot interface Re-initialized.
 */

NvBootError NvBootColdBootReInit()
{
    NvBootError e = NvBootError_Success;
    NvU8        DevParamIndex = 0;

   /*
     * Only perform device type error checking and reinitialization if
     * parameter sets were provided within the BCT.
     */
    if (pBootConfigTable->NumParamSets != 0)
    {
        /* Verify BCT devParams has valid data */
        if ((pBootConfigTable->DevType[DevParamIndex] == NvBootDevType_None) || 
            (pBootConfigTable->DevType[DevParamIndex] >= NvBootDevType_Max))
        {
            e = NvBootError_BctBlockInfoMismatch;
            goto fail;
        }

        /*
         * Verify that the BCT's devType[i]  match the used by the
         * device manager.
         */
        if (BootInfoTable.SecondaryDevice !=
            pBootConfigTable->DevType[DevParamIndex])
        {
            e = NvBootError_BctBlockInfoMismatch;
            goto fail;
        }

        NV_BOOT_CHECK_ERROR_CLEANUP(
            NvBootDevMgrReinitDevice(&(Context.DevMgr),
                                     pBootConfigTable->DevParams,
                                     pBootConfigTable->NumParamSets,
                                     DevParamIndex));
        BootInfoTable.BootROMtracker = NvBootFlowStatus_CBReinitSuccess;
    }

 fail:
#if NVBOOT_SPIN_WAIT_AT_END
    NV_BOOT_SPIN_WAIT()
#endif
    return e;

}

/**
 * Read and validate bootloader (MB1/BL).
 *
 * @retval NvBootError_Success, Bootloader validated.
 */
NvBootError NvBootColdBootLoadBl()
{
    NvBootError e;
    NvU32 TickCount = 0;

    TickCount = NvBootUtilGetTimeUS();
    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootLoadBootLoader(&Context));
    BootInfoTable.BootTimeLog.NvBootReadBLTickCnt = NvBootUtilElapsedTimeUS(TickCount);
    BootInfoTable.BootROMtracker = NvBootFlowStatus_CBPayloadSuccess;

 fail:
#if NVBOOT_SPIN_WAIT_AT_END
        NV_BOOT_SPIN_WAIT()
#endif
        return e;
}


/**
 * Performs action based on bct flags/variables 
 * Nothing for t214
 */
NvBootError NvBootColdBootEnableBctFlags()
{
    return NvBootError_Success;
}

#if NVBOOT_TARGET_QT
static void
NvBootMainNonsecureBootLoaderSpin(void)
{
    NV_BOOT_SPIN_WAIT();
}
#endif

/**
 * Perform all the operations needed for cold boot.
 *
 * @param[out] BootRomExitTarget A pointer to the entry point of the BL
 *
 * @retval NvBootError_Success Cold boot completed & the BL is ready to run.
 * @retval TODO Errors from SetupBootDevice()
 * @retval TODO Errors from NvBootReadBct()
 * @retval TODO Errors from NvBootDevMgrReinitDevice()
 * @retval TODO Errors from SetupSdram()
 * @retval TODO Errors from NvBootLoadBootLoader()
 */
// NvBootError NvBootColdBoot(NvU32 *BootRomExitTarget)
// {
    // NvBootError e;
    // NvU8        DevParamIndex;
    // NvU32 TickCount;

// #if NVBOOT_SPIN_WAIT_AT_START
    // NV_BOOT_SPIN_WAIT()
// #endif

    // /* Record the boot type (will be overwritten upon fail over to RCM) */
    // BootInfoTable.BootType = NvBootType_Cold;

// #if NVBOOT_TARGET_QT
    // NV_BOOT_CHECK_ERROR_CLEANUP(SetupSdram(&Context));
    
    // *BootRomExitTarget = PTR_TO_ADDR(NvBootMainNonsecureBootLoaderSpin);
    // e = NvBootError_Success; 
// #else
    // NV_BOOT_CHECK_ERROR_CLEANUP(SetupBootDevice(&Context));

    // BootInfoTable.DevInitialized = NV_TRUE;

    // MapFusesToCryptoOps(&Context);

    // TickCount = NvBootUtilGetTimeUS();
    // NV_BOOT_CHECK_ERROR_CLEANUP(NvBootReadBct(&Context));
    // BootInfoTable.BootTimeLog.NvBootReadBctTickCnt = NvBootUtilElapsedTimeUS(TickCount);

    // /* Determine the parameter index to use & check its range. */
    // DevParamIndex = NvBootStrapDeviceConfigurationIndex();

    // /*
     // * Initialize DRAM before re-initializing secondary boot device.
     // * This is required for secondary boot devices to use DRAM for all
     // * operations.
     // */
    // NV_BOOT_CHECK_ERROR_CLEANUP(SetupSdram(&Context));

    // /*
     // * Only perform device type error checking and reinitialization if
     // * parameter sets were provided within the BCT.
     // */
    // if (BootConfigTable.NumParamSets != 0)
    // {
        // /* Verify BCT devParams has valid data */
        // if ((BootConfigTable.DevType[DevParamIndex] == NvBootDevType_None) || 
            // (BootConfigTable.DevType[DevParamIndex] >= NvBootDevType_Max))
        // {
            // e = NvBootError_BctBlockInfoMismatch;
            // goto fail;
        // }

        // /*
         // * Verify that the BCT's devType[i]  match the used by the
         // * device manager.
         // */
        // if (BootInfoTable.SecondaryDevice !=
            // BootConfigTable.DevType[DevParamIndex])
        // {
            // e = NvBootError_BctBlockInfoMismatch;
            // goto fail;
        // }

        // NV_BOOT_CHECK_ERROR_CLEANUP(
            // NvBootDevMgrReinitDevice(&(Context.DevMgr),
                                     // BootConfigTable.DevParams,
                                     // BootConfigTable.NumParamSets,
                                     // DevParamIndex));
    // }



    // TickCount = NvBootUtilGetTimeUS();
    // NV_BOOT_CHECK_ERROR_CLEANUP(NvBootLoadBootLoader(&Context));
                                                     // //TODO BootRomExitTarget));
    // BootInfoTable.BootTimeLog.NvBootReadBLTickCnt = NvBootUtilElapsedTimeUS(TickCount);
// #endif
 // fail:
// #if NVBOOT_SPIN_WAIT_AT_END
    // NV_BOOT_SPIN_WAIT()
// #endif
    // return e;
// }
