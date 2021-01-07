/*
 * Copyright (c) 2008 - 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "nvboot_bit.h"
#include "nvboot_devmgr_int.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_sdmmc_context.h"
#include "nvboot_sdmmc_int.h"
#include "nvboot_sdmmc_param.h"
#include "nvboot_sdmmc_local.h"

// Can't use do while for this, as it is also called from PRINT_SDMMC_XXX.
#define DEBUG_SDMMC 0

#if DEBUG_SDMMC
#define PRINT_SDMMC_REG_ACCESS(...)  NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_SDMMC_MESSAGES(...)    NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_SDMMC_ERRORS(...)      NvOsDebugPrintf(__VA_ARGS__);
#else
#define PRINT_SDMMC_REG_ACCESS(...)
#define PRINT_SDMMC_MESSAGES(...)
#define PRINT_SDMMC_ERRORS(...)
#endif

/*
 * Sdmmc device callbacks for Device Manager.
 */
NvBootDevMgrCallbacks SdmmcDeviceCallback = 
{
    /* Callbacks for the SDMMC device */
    (NvBootDeviceGetParams)NvBootSdmmcGetParams,
    (NvBootDeviceValidateParams)NvBootSdmmcValidateParams,
    (NvBootDeviceGetBlockSizes)NvBootSdmmcGetBlockSizes,
    (NvBootDeviceInit)NvBootSdmmcInit,
    (NvBootDeviceRead)NvBootSdmmcReadPage,
    (NvBootDeviceQueryStatus)NvBootSdmmcQueryStatus,
    (NvBootDeviceShutdown)NvBootSdmmcShutdown,
    (NvBootDeviceGetReaderBuffersBase)NvBootSdmmcGetReaderBuffersBase,
    (NvBootDevicePinMuxInit)NvBootSdmmcPinMuxInit
};

NvBootSdmmcParams s_DefaultSdmmcParams;
NvBootSdmmcContext *s_SdmmcContext = NULL;
// Boot Info table.
extern NvBootInfoTable BootInfoTable;
// Pointer to Nand Bit info.
NvBootSdmmcStatus* s_SdmmcBitInfo =
                    (NvBootSdmmcStatus*)&BootInfoTable.SecondaryDevStatus[0];

/* Public Function Definitions. */
void
NvBootSdmmcGetParams(
    const uint32_t ParamIndex,
    NvBootSdmmcParams **Params)
{
    NV_ASSERT(Params != NULL);
    
    /// Fuse params derivation is bases on configs 0-4
    /// Bct configuration options will be similarly based on configs 
    /// instead of individual/device/function specific parameters. 
    
    /// Datawidth is fixed @ 8 bit wide
    /// VoltageRange is query 
    /// BootMode is *NOT* supported.
    /// Clockdivider is based on configs supported.
    /// Multipage support is based on configs.
    
    
    /*
    * Extract the configs use from Param Index, which comes from fuses.
    * Configs are defined to simplify boot params options and extract 
    * relevant device specific information.
    */
    s_DefaultSdmmcParams.SdmmcConfig = NV_DRF_VAL(SDMMC_DEVICE, CONFIG,
                DEV_CONFIG, ParamIndex);
    /*
    * Max Power class supported by target board is unknown. Bct would give us
    * the Max power class supported. So, Till that time, Let it be 0 and work
    * with power calss 0..Target board must support power class 0.
    */
    s_DefaultSdmmcParams.MaxPowerClassSupported = 0;
    
    *Params = (NvBootSdmmcParams*)&s_DefaultSdmmcParams;
    PRINT_SDMMC_MESSAGES("\r\nParamIndex=0x%x, SdmmcConfig=%d , "
        "MaxPowerClass Supported=%d", ParamIndex, s_DefaultSdmmcParams.SdmmcConfig,
        s_DefaultSdmmcParams.MaxPowerClassSupported);
}

NvBool
NvBootSdmmcValidateParams(
    const NvBootSdmmcParams *Params)
{
    NV_ASSERT(Params != NULL);
    PRINT_SDMMC_MESSAGES("\r\nValidateParams, SdmmcConfig=%d, "
        "MaxPowerClassSupported=%d", Params->SdmmcConfig, 
        Params->MaxPowerClassSupported);
    if ( (Params->SdmmcConfig < Sdmmc_Config_0) ||
         (Params->SdmmcConfig > Sdmmc_Config_4) )
        return NV_FALSE;
    if (Params->MaxPowerClassSupported > SDMMC_MAX_POWER_CLASS_SUPPORTED)
        return NV_FALSE;
    return NV_TRUE;
}

void
NvBootSdmmcGetBlockSizes(
    const NvBootSdmmcParams *Params,
    uint32_t *BlockSizeLog2,
    uint32_t *PageSizeLog2)
{
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);
    NV_ASSERT(s_SdmmcContext != NULL);

    (void)Params;
    *BlockSizeLog2 = s_SdmmcContext->BlockSizeLog2;
    *PageSizeLog2 = ( s_SdmmcContext->PageSizeLog2 );
    PRINT_SDMMC_MESSAGES("\r\nBlockSize=%d, PageSize=%d, PagesPerBlock=%d",
        (1 << s_SdmmcContext->BlockSizeLog2),(1 << s_SdmmcContext->PageSizeLog2),
        (1 << s_SdmmcContext->PagesPerBlockLog2));
}

void SdmmcConfig(NvBootSdmmcContext *pSdmmcContext)
{

    NV_ASSERT(pSdmmcContext->ConfigOption >= Sdmmc_Config_0 \
        && pSdmmcContext->ConfigOption <= Sdmmc_Config_4);

    //Derive ClockDivisor/mode/width parameters from config.

    // Card clock Divisor as 1/2 for all configs except "Sdmmc_Config_0"
    pSdmmcContext->CardClockDivisor = SDMMC_IO_CLOCK_DIVISOR_BY_HALF;

    // Controller clock divisor set @ 51Mhz PLLP for all configs except "Sdmmc_Config_3"
    pSdmmcContext->ClockDivisor = SDMMC_CNTL_CLOCK_DIVISOR_51MHZ;

    // Multipage Read support for all configs except "Sdmmc_Config_2"
    pSdmmcContext->MultiPageSupported = Sdmmc_ReadMode_MultiPage;

    // Data width @ 8 bit wide default for all configs except "Sdmmc_Config_3" and "Sdmmc_Config_4"
    pSdmmcContext->DataWidth = NvBootSdmmcDataWidth_8Bit;

    switch(pSdmmcContext->ConfigOption)
    {
        case Sdmmc_Config_0: // SDR_25.5Mhz_ReadSinglepage
            pSdmmcContext->MultiPageSupported = Sdmmc_ReadMode_SinglePage; // SinglePage Read support
            break;
        case Sdmmc_Config_1: // SDR_51Mhz_ReadMultipage
            pSdmmcContext->CardClockDivisor = SDMMC_IO_CLOCK_DIVISOR_NONE; 
            break;
        case Sdmmc_Config_2: // SDR_25.5Mhz_ReadMultipage
            break;
        case Sdmmc_Config_3: // DDR_51Mhz_ReadMultipage
            pSdmmcContext->ClockDivisor = SDMMC_CNTL_CLOCK_DIVISOR_102MHZ;//DDR 
            pSdmmcContext->DataWidth = NvBootSdmmcDataWidth_Ddr_8Bit;
            break;
        case Sdmmc_Config_4: // DDR_25.5Mhz_ReadMultipage
            pSdmmcContext->DataWidth = NvBootSdmmcDataWidth_Ddr_8Bit;
            break;
    }
}

NvBootError
NvBootSdmmcInit(
    const NvBootSdmmcParams *Params,
    NvBootSdmmcContext *Context)
{
    NvBootError e = NvBootError_Success;
    uint32_t PageSize = 0; //To be updated with code merge
    // time Init
    unsigned long funcStartTick = 0;

    NV_ASSERT(Params != NULL);
    NV_ASSERT(Context != NULL);
    NV_ASSERT(Params->SdmmcConfig >= Sdmmc_Config_0 &&
        Params->SdmmcConfig <= Sdmmc_Config_4);

    // Stash the pointer to the context structure.
    s_SdmmcContext = Context;
    s_SdmmcContext->taac = 0;
    s_SdmmcContext->nsac = 0;

    s_SdmmcContext->ConfigOption = Params->SdmmcConfig;
    s_SdmmcContext->MaxPowerClassSupported = Params->MaxPowerClassSupported;
    s_SdmmcContext->CardSupportsHighSpeedMode = NV_FALSE;
    s_SdmmcContext->ReadTimeOutInUs = SDMMC_READ_TIMEOUT_IN_US;
    s_SdmmcContext->EmmcBootPartitionSize = 0;
    s_SdmmcContext->CurrentClockRate = NvBootSdmmcCardClock_Identification;
    s_SdmmcContext->CurrentAccessRegion = SdmmcAccessRegion_Unknown;
    s_SdmmcContext->BootModeReadInProgress = NV_FALSE;
    s_SdmmcContext->SdmmcInternalBuffer = (uint8_t *)(NVBOOT_SDMMC_INT_RAM_BUF_START);

    //Derive ClockDivisor/mode/width parameters from config.
    SdmmcConfig(s_SdmmcContext);
    
    /// Controller Initialization time
    funcStartTick = NvBootUtilGetTimeUS();

    // Initialize the Hsmmc Hw controller.
    NV_BOOT_CHECK_ERROR(HwSdmmcInitController());

     /// Update Sdmmc Controller timestamp
    s_SdmmcBitInfo->SdmmcControllerInit = NvBootUtilGetTimeUS() - funcStartTick;

    // Check for card is present. NOT required
    // Bug # 826908

    // Boot Mode Enable *not* supported.

    // only eMMC is supported as boot media
    NV_BOOT_CHECK_ERROR(EmmcIdentifyCard());

    /// Update Device enumeration timestamp
    s_SdmmcBitInfo->eMMCDeviceEnumeration = NvBootUtilGetTimeUS() - s_SdmmcBitInfo->SdmmcControllerInit;

    s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_Idle;
    s_SdmmcBitInfo->DiscoveredCardType = NvBootSdmmcCardType_Emmc;

    // enable block length setting, if CYA is cleared.
    if((s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_4Bit) ||
            (s_SdmmcContext->DataWidth == NvBootSdmmcDataWidth_8Bit))
    {
        PageSize = (1 << s_SdmmcContext->PageSizeLog2);
        // Send SET_BLOCKLEN(CMD16) Command.
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SetBlockLength,
            PageSize, SdmmcResponseType_R1, NV_FALSE));
        NV_BOOT_CHECK_ERROR(EmmcVerifyResponse(SdmmcCommand_SetBlockLength,
            NV_FALSE));
    }

    /// Update SdmmcInit timestamp
    s_SdmmcBitInfo->SdmmcInit = NvBootUtilGetTimeUS() - funcStartTick;

    /// Update Bit info
    s_SdmmcBitInfo->FuseReadMode = s_SdmmcContext->MultiPageSupported;
    s_SdmmcBitInfo->FuseDataWidth = s_SdmmcContext->DataWidth;
    s_SdmmcBitInfo->FuseDdrMode = (s_SdmmcContext->DataWidth ? NvBootSdmmcDataWidth_Ddr_8Bit : NvBootSdmmcDataWidth_8Bit);
    s_SdmmcBitInfo->FuseConfig = Params->SdmmcConfig;

    return e;
}

NvBootError
NvBootSdmmcReadPage(
    const uint32_t Block,
    const uint32_t Page,
    const uint32_t Len,
    uint8_t *pBuffer)
{
    NvBootError e;
    uint32_t CommandArg;
    uint32_t ActualBlockToRead;
    uint32_t Page2Access = Page;
    uint32_t Block2Access = Block;
    uint32_t PageSize = (1 << s_SdmmcContext->PageSizeLog2);
    uint32_t PagesToRead;

    // time read time
    unsigned long funcStartTick = NvBootUtilGetTimeUS();

    NV_ASSERT(Page < (1 << s_SdmmcContext->PagesPerBlockLog2));
    NV_ASSERT(pBuffer != NULL);

    PRINT_SDMMC_MESSAGES("\r\nRead Block=%d, Page=%d", Block, Page);
    /// COMMENTED CODE IS ONLY FOR TESTING PURPOSES WHILE THE 
    /// READER CODE IS FIXED TO HANDLE DIFFERENT MEMORY DESTINATIONS
    /// FOR ALL TYPES OF MEMORY CLIENTS AND THE MODES HANDLED.

    // TBD: To check Boot mode support in both Single/Multi READ sector mode for Block/Page == 0

    // Calculate the pages to read - this will be a multiple of transfer size
    PagesToRead = Len / (1 << s_SdmmcContext->PageSizeLog2);
    if (Len - (PagesToRead * (1 << s_SdmmcContext->PageSizeLog2)) != 0)
        PagesToRead++;
    s_SdmmcBitInfo->NumPagesRead += PagesToRead;

    // Check if MultiPageRead is supported
    if (s_SdmmcContext->MultiPageSupported)
    {
    
        // If data line ready times out, try to recover from errors.
        if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
        {
            NV_BOOT_CHECK_ERROR(HwSdmmcRecoverControllerFromErrors(NV_TRUE));
        }
        // Select access region.This will intern changes block and page addresses
        // based on the region the request falls in.
        NV_BOOT_CHECK_ERROR(SdmmcSelectAccessRegion(&Block2Access));
        PRINT_SDMMC_MESSAGES("\r\nRegion=%d(1->BP1, 2->BP2, 0->UP)Block2Access=%d, "
        "Page2Access=%d", s_SdmmcContext->CurrentAccessRegion, Block2Access,
        Page2Access);
        // DMA block size is programmed to 512K for SDMA.
        // This is sufficient for 324K of Sysram and 128K of BTCM which will be used
        // by BootROM and not all of the memory will ever be used for reading data(BCT, Bootloader)
        
        // Find out the Block to read from eMMC.
        ActualBlockToRead = (Block2Access << s_SdmmcContext->PagesPerBlockLog2) +
               Page2Access;
        HwSdmmcSetNumOfBlocks(PageSize, PagesToRead);
        
        /*
        * If block to read is beyond card's capacity, then some Emmc cards are
        * responding with error back and continue to work. Some are not responding
        * for this and for subsequent valid operations also.
        */
        //if (ActualBlockToRead >= s_SdmmcContext->NumOfBlocks)
        //    return NvBootError_IllegalParameter;
        // Set number of blocks to read to 1.
        
        // Set up command arg.
        if (s_SdmmcContext->IsHighCapacityCard)
            CommandArg = ActualBlockToRead;
        else
            CommandArg = (ActualBlockToRead << s_SdmmcContext->PageSizeLog2);
        PRINT_SDMMC_MESSAGES("\r\nActualBlockToRead=%d, CommandArg=%d",
            ActualBlockToRead, CommandArg);
        // Store address of pBuffer in sdmmc context
        s_SdmmcContext->CurrentReadBufferAddress = pBuffer;
        // Setup Dma.
        HwSdmmcSetupDma(pBuffer);
        
        // settingthe block count command
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SetBlockCount,
        (PagesToRead), SdmmcResponseType_R1, NV_FALSE));
        
        // Send command to card.
        NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_ReadMulti,
            CommandArg, SdmmcResponseType_R1, NV_TRUE));
        // If data line ready times out, try to recover from errors.
        if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
        {
            NV_BOOT_CHECK_ERROR(HwSdmmcRecoverControllerFromErrors(NV_TRUE));
        }
       
    }
    else
    {

        // If data line ready times out, try to recover from errors.
        if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
        {
            NV_BOOT_CHECK_ERROR(HwSdmmcRecoverControllerFromErrors(NV_TRUE));
        }
        
        do
        {
            // Select access region.This will intern changes block and page addresses
            // based on the region the request falls in.
            NV_BOOT_CHECK_ERROR(SdmmcSelectAccessRegion(&Block2Access));
            PRINT_SDMMC_MESSAGES("\r\nRegion=%d(1->BP1, 2->BP2, 0->UP)Block2Access=%d, "
            "Page2Access=%d", s_SdmmcContext->CurrentAccessRegion, Block2Access,
            Page2Access);
            // DMA block size is programmed to 512K for SDMA.
            // This is sufficient for 324K of Sysram and 128K of BTCM which will be used
            // by BootROM and not all of the memory will ever be used for reading data(BCT, Bootloader)
            
            // Find out the Block to read from eMMC.
            ActualBlockToRead = (Block2Access << s_SdmmcContext->PagesPerBlockLog2) +
                Page2Access;
            HwSdmmcSetNumOfBlocks(PageSize, 1);
            
            /*
            * If block to read is beyond card's capacity, then some Emmc cards are
            * responding with error back and continue to work. Some are not responding
            * for this and for subsequent valid operations also.
            */
            //if (ActualBlockToRead >= s_SdmmcContext->NumOfBlocks)
            //    return NvBootError_IllegalParameter;
            // Set number of blocks to read to 1.
            
            // Set up command arg.
            if (s_SdmmcContext->IsHighCapacityCard)
                CommandArg = ActualBlockToRead;
            else
               CommandArg = (ActualBlockToRead << s_SdmmcContext->PageSizeLog2);
            PRINT_SDMMC_MESSAGES("\r\nActualBlockToRead=%d, CommandArg=%d",
            ActualBlockToRead, CommandArg);
            // Store address of pBuffer in sdmmc context
            s_SdmmcContext->CurrentReadBufferAddress = pBuffer;
            // Setup Dma.
            HwSdmmcSetupDma(pBuffer);
            
            // Send command to card.
            NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_ReadSingle,
                CommandArg, SdmmcResponseType_R1, NV_TRUE));
            
            // If response fails, return error. Nothing to clean up.
            NV_BOOT_CHECK_ERROR_CLEANUP(EmmcVerifyResponse(SdmmcCommand_ReadSingle,
                                         NV_FALSE));
            Page2Access++;
            if (Page2Access >= (uint32_t)(1 << s_SdmmcContext->PagesPerBlockLog2))
            {
                Page2Access = 0;
                Block2Access++;
            }
            pBuffer += PageSize;
            PagesToRead--;
            // If data line ready times out, try to recover from errors.
            if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
            {
                NV_BOOT_CHECK_ERROR(HwSdmmcRecoverControllerFromErrors(NV_TRUE));
            }
        }while(PagesToRead);
    }
   s_SdmmcContext->DeviceStatus = NvBootDeviceStatus_Idle;
   // s_SdmmcContext->ReadStartTime = NvBootUtilGetTimeUS();

   /// Update read timestamp and data payload
   s_SdmmcBitInfo->ReadTime = NvBootUtilGetTimeUS() - funcStartTick;
   s_SdmmcBitInfo->Payload = Len;
   
    return e;
fail:
    HwSdmmcAbortDataRead();
    return e;
}

NvBootDeviceStatus NvBootSdmmcQueryStatus(void)
{
    return HwSdmmcQueryStatus();
}

void NvBootSdmmcShutdown(void)
{
    HwSdmmcShutdown();
}

NvBootError 
NvBootSdmmcWritePage(
    const uint32_t Block,
    const uint32_t Page,
    const uint32_t Len,
    uint8_t *Dest)
{

    NvBootError e;
    uint32_t CommandArg;
    uint32_t ActualBlockToWrite;
    uint32_t Page2Access = Page;
    uint32_t Block2Access = Block;
    uint32_t PageSize = (1 << s_SdmmcContext->PageSizeLog2);
    uint32_t PagesToWrite;
    NvBootDeviceStatus WriteXferStatus;
    NV_ASSERT(Page < (1 << s_SdmmcContext->PagesPerBlockLog2));
    NV_ASSERT(pBuffer != NULL);

 
    // Calculate the pages to read - this will be a multiple of transfer size
    PagesToWrite = Len / (1 << s_SdmmcContext->PageSizeLog2);
    if (Len - (PagesToWrite * (1 << s_SdmmcContext->PageSizeLog2)) != 0)
    PagesToWrite++;
    
        do
        {
            // If data line ready times out, try to recover from errors.
            if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
            {
                NV_BOOT_CHECK_ERROR(HwSdmmcRecoverControllerFromErrors(NV_TRUE));
            }
            // Select access region.This will intern changes block and page addresses
            // based on the region the request falls in.
            NV_BOOT_CHECK_ERROR(SdmmcSelectAccessRegion(&Block2Access));
            
            // Find out the Block to write to on eMMC.
            ActualBlockToWrite = (Block2Access << s_SdmmcContext->PagesPerBlockLog2) +
                     Page2Access;
            HwSdmmcSetNumOfBlocks(PageSize, 1);
            
            // Set up command arg.
            if (s_SdmmcContext->IsHighCapacityCard)
                CommandArg = ActualBlockToWrite;
            else
                CommandArg = (ActualBlockToWrite << s_SdmmcContext->PageSizeLog2);
            // Store address of pBuffer in sdmmc context
            s_SdmmcContext->CurrentReadBufferAddress = Dest;
            // Setup Dma.
            HwSdmmcSetupDma(Dest);
            // settingthe block count command
            NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_SetBlockCount,
            (PagesToWrite), SdmmcResponseType_R1, NV_FALSE));
            NV_BOOT_CHECK_ERROR(HwSdmmcSendCommand(SdmmcCommand_WriteSingle,
            CommandArg, SdmmcResponseType_R1, NV_TRUE));
            
            // If response fails, return error. Nothing to clean up.
            NV_BOOT_CHECK_ERROR_CLEANUP(EmmcVerifyResponse(SdmmcCommand_WriteSingle,
                 NV_FALSE));

            if (HwSdmmcWaitForDataLineReady() != NvBootError_Success)
            {
                WriteXferStatus = NvBootSdmmcQueryStatus();
                if (WriteXferStatus == NvBootDeviceStatus_Idle)
                    e= NvBootError_Success;
                else
                   e = NvBootError_TxferFailed;
            }
            
            Page2Access++;
            if (Page2Access >= (uint32_t)(1 << s_SdmmcContext->PagesPerBlockLog2))
            {
                Page2Access = 0;
                Block2Access++;
            }
            Dest += PageSize;
            PagesToWrite--;
        }while(PagesToWrite);

 fail:
    return e;
}
NvBootError NvBootSdmmcGetReaderBuffersBase()
{
/// COMMENTED CODE IS ONLY FOR TESTING PURPOSES WHILE THE 
/// READER CODE IS FIXED TO HANDLE DIFFERENT MEMORY DESTINATIONS
/// FOR ALL TYPES OF MEMORY CLIENTS AND THE MODES HANDLED.
    return NvBootError_Unimplemented;
}

NvBootError NvBootSdmmcPinMuxInit()
{
    /*
     * Upper layer doesn't check for NvBootError_Unimplemented
     * and considers it an error. Not to break existing functionality,
     * return NvBootError_Success
     */
    return NvBootError_Success;//NvBootError_Unimplemented;
}
