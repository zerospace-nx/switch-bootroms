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

static NvBootContext     Context;

/* Function Prototypes */
static NvBootError SetupBootDevice(NvBootContext *Context);
static void        MapFusesToCryptoOps(NvBootContext *Context);
static NvBootError SetupSdram(NvBootContext *Context);

static NvBootFuseBootDevice StrapDevSelMap[] =
{
    NvBootFuseBootDevice_Sdmmc,         // NvBootStrapDevSel_Emmc_x8_BootModeOff
    NvBootFuseBootDevice_SpiFlash,      // NvBootStrapDevSel_SpiFlash
    NvBootFuseBootDevice_Sata,          // NvBootStrapDevSel_Sata
    NvBootFuseBootDevice_Sdmmc,         // 0x3 = Use Fuses when PRODUCTION_MODE fuse is set. 
    NvBootFuseBootDevice_Usb3,          // NvBootStrapDevSel_Usb3
                                        // Strap-> Fuse mapping not done for this strap.
    NvBootFuseBootDevice_resvd_4,       // NvBootStrapDevSel_Rcm, Convoluted way of forcing rcm
                                        // through device none.
};

NV_CT_ASSERT((sizeof(StrapDevSelMap)/sizeof(NvBootFuseBootDevice)) ==
             NvBootStrapDevSel_Num);

#define SDMMC_CONFIG(MultiPageSupport, ClockDivider, BusWidth, BootMode) \
    (((BusWidth) << 0) | ((BootMode) << 4) | ((ClockDivider) << 6) | \
    ((MultiPageSupport) << 10))

#define SNOR_CONFIG(NonMuxed, BusWidth) \
    (((BusWidth) << 0) | ((NonMuxed) << 1))

#define SPI_CONFIG(PageSize2kor16k) \
    ((PageSize2kor16k) << 0)

#define XUSB_CONFIG(VBUS_number, OCPin, PortNumber) \
    ((VBUS_number << 8) | (OCPin << 5) | ((PortNumber) << 0))

static NvU32 StrapDevConfigMap[] =
{
    SDMMC_CONFIG(0, 0, 1, 1),   // NvBootStrapDevSel_Emmc_BMOff_x8
    SPI_CONFIG(0),              // NvBootStrapDevSel_SpiFlash : Page size 2K
    0,                          // NvBootStrapDevSel_Sata 0x00
    SDMMC_CONFIG(0, 0, 0, 0),   // NvBootStrapDevSel_UseFuses; Set default config to EMMC x4, Boot Mode on for safety.
                                // This is the same config as when BOOT_DEVICE_INFO and FUSE_RESERVED_SW[2:0] are
                                // unfused.
    XUSB_CONFIG(1, 5, 1),       // NvBootStrapDevSel_Usb3 : Vbus Enable 1, OC Detected Vbus Pad 1, port number 1
    0,                          // NvBootStrapDevSel_Rcm 0x00
};

NV_CT_ASSERT((sizeof(StrapDevConfigMap)/sizeof(NvU32)) ==
             NvBootStrapDevSel_Num);

static NvBootDevType MapFuseDevToBitDevType[] = 
{
    // Fuse to device type as specified in t35/bootrom/include/t35/nvboot_bct.h
    NvBootDevType_Sdmmc,        // NvBootFuseBootDevice_Sdmmc
    NvBootDevType_Spi,          // NvBootFuseBootDevice_SpiFlash
    NvBootDevType_Sata,         // Reserved
#if NVENABLE_FOOS_SUPPORT
    NvBootDevType_Foos,         // Reserved
#else
    NvBootDevType_None,         // Reserved
#endif
    NvBootDevType_Usb3,         // NvBootFuseBootDevice_Usb3
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
    NvU32                ConfigIndex;
    NvBootFuseBootDevice FuseDev;
    NvBootDevType        DevType;
    NvBootError          e;
    NvBootStrapDevSel    StrapDevSel;
    NvBootFuseOperatingMode OpMode;
    
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
    NvBootFuseGetOperatingMode(&OpMode);
    if (NvBootStrapIsProdUartMode() && OpMode == NvBootFuseOperatingMode_NvProduction)
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
 * MapFusesToCryptoOps(): Determine which cryptography operations are needed
 * based on the operating mode described by the fuses.
 *
 * @param[in] Context Pointer to the boot context.
 */
static void MapFusesToCryptoOps(NvBootContext *Context)
{
    NvBootFuseOperatingMode Mode;

    NV_ASSERT(Context != NULL);

    NvBootFuseGetOperatingMode(&Mode);

    /* Only the following modes are valid for cold boot. */
    NV_ASSERT((Mode == NvBootFuseOperatingMode_Preproduction         ) ||
              (Mode == NvBootFuseOperatingMode_NvProduction          ) ||
              (Mode == NvBootFuseOperatingMode_OdmProductionSecureSBK   ) ||
              (Mode == NvBootFuseOperatingMode_OdmProductionSecurePKC ) ||
              (Mode == NvBootFuseOperatingMode_OdmProductionNonSecure));

    Context->DecryptBootloader = NV_FALSE; 
    
    if(Mode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
    {
        Context->CheckBootloaderHash = NV_FALSE;

    }
    else 
    {
        Context->CheckBootloaderHash = NV_TRUE;

        if (Mode == NvBootFuseOperatingMode_OdmProductionSecureSBK)
        {
            Context->DecryptBootloader = NV_TRUE;
        }
    }
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
    NvU8 SdramIndex;
    NvU32 StableTime, Misc1, Misc2;
    NvBootSdramParams *Params;
    NvU32 RegData = 0;

    NV_ASSERT(Context != NULL);

    /* Do nothing if there are no SDRAM sets in the BCT. */
    if (BootConfigTable.NumSdramSets == 0)
        return NvBootError_Success;

    SdramIndex = NvBootStrapSdramConfigurationIndex();

    if (SdramIndex >= BootConfigTable.NumSdramSets)
    {
        return NvBootError_IllegalParameter;
    }
    
    Params = &(BootConfigTable.SdramParams[SdramIndex]);

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
NvBootError NvBootColdBoot(NvU32 *BootRomExitTarget)
{
    NvBootError e;
    NvU8        DevParamIndex;
    NvU32 TickCount;

#if NVBOOT_SPIN_WAIT_AT_START
    NV_BOOT_SPIN_WAIT()
#endif

    /* Record the boot type (will be overwritten upon fail over to RCM) */
    BootInfoTable.BootType = NvBootType_Cold;

    Context.BlFailBack = NvBootPmcQueryFlag(NvBootPmcFlagId_FailBack);
    if (Context.BlFailBack)
    {
        /* Clear the flag. */
        NvBootPmcSetFlag(NvBootPmcFlagId_FailBack, NV_FALSE);
        BootInfoTable.ClearedFailBack = NV_TRUE;
    }

#if NVBOOT_TARGET_QT
    NV_BOOT_CHECK_ERROR_CLEANUP(SetupSdram(&Context));
    
    *BootRomExitTarget = PTR_TO_ADDR(NvBootMainNonsecureBootLoaderSpin);
    e = NvBootError_Success; 
#else
    NV_BOOT_CHECK_ERROR_CLEANUP(SetupBootDevice(&Context));

    BootInfoTable.DevInitialized = NV_TRUE;

    MapFusesToCryptoOps(&Context);

    TickCount = NvBootUtilGetTimeUS();
    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootReadBct(&Context));
    BootInfoTable.BootTimeLog.NvBootReadBctTickCnt = NvBootUtilElapsedTimeUS(TickCount);

    /*  Incorporate the BCT's FailBack enable flag. */
    Context.BlFailBack |= BootConfigTable.EnableFailBack;

    /* Determine the parameter index to use & check its range. */
    DevParamIndex = NvBootStrapDeviceConfigurationIndex();

    /*
     * Initialize DRAM before re-initializing secondary boot device.
     * This is required for secondary boot devices to use DRAM for all
     * operations.
     */
    NV_BOOT_CHECK_ERROR_CLEANUP(SetupSdram(&Context));

    /*
     * Only perform device type error checking and reinitialization if
     * parameter sets were provided within the BCT.
     */
    if (BootConfigTable.NumParamSets != 0)
    {
        /* Verify BCT devParams has valid data */
        if ((BootConfigTable.DevType[DevParamIndex] == NvBootDevType_None) || 
            (BootConfigTable.DevType[DevParamIndex] >= NvBootDevType_Max))
        {
            e = NvBootError_BctBlockInfoMismatch;
            goto fail;
        }

        /*
         * Verify that the BCT's devType[i]  match the used by the
         * device manager.
         */
        if (BootInfoTable.SecondaryDevice !=
            BootConfigTable.DevType[DevParamIndex])
        {
            e = NvBootError_BctBlockInfoMismatch;
            goto fail;
        }

        NV_BOOT_CHECK_ERROR_CLEANUP(
            NvBootDevMgrReinitDevice(&(Context.DevMgr),
                                     BootConfigTable.DevParams,
                                     BootConfigTable.NumParamSets,
                                     DevParamIndex));
    }



    TickCount = NvBootUtilGetTimeUS();
    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootLoadBootLoader(&Context,
                                                     BootRomExitTarget));
    BootInfoTable.BootTimeLog.NvBootReadBLTickCnt = NvBootUtilElapsedTimeUS(TickCount);
#endif
 fail:
#if NVBOOT_SPIN_WAIT_AT_END
    NV_BOOT_SPIN_WAIT()
#endif
    return e;
}
