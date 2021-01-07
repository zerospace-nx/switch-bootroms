/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_mux_one_nand_local.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
//#include "string.h"
#include "arsnor.h"
#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_mux_one_nand_context.h"
#include "nvboot_mux_one_nand_int.h"
#include "nvboot_mux_one_nand_param.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "project.h"


// Wrapper macros for reading/writing from/to SNOR register
#define SNOR_REG_READ32(reg) \
        NV_READ32(NV_ADDRESS_MAP_APB_SNOR_BASE + (SNOR_##reg##_0))
        
#define SNOR_REG_WRITE32(reg, val) \
    do { \
        NV_WRITE32((NV_ADDRESS_MAP_APB_SNOR_BASE + (SNOR_##reg##_0)), (val)); \
    } while (0)


// Wrapper macros for Reading and writing the nand flash memory register.
#define MUXONENANDFLASH_READ16(reg) \
    NV_READ16(NV_ADDRESS_MAP_NOR_BASE + (reg <<1))
    
#define MUXONENANDFLASH_WRITE16(reg, val) \
    do { \
        NV_WRITE16((NV_ADDRESS_MAP_NOR_BASE + (reg <<1)), (val)); \
    } while (0)

typedef struct 
{
    NvBool IsNonFlexTypeDevice;
    NvBool IsNonmuxedInterface;
    NvU32 PageSizeLog2;
    NvU32 BlockSizeLog2;
} MuxOneNandFuseParam;


// Boot Info table.
extern NvBootInfoTable BootInfoTable;

// Pointer to Snor Bit info.
static NvBootMuxOneNandStatus* s_pMuxOneNandBitInfo =
           &BootInfoTable.SecondaryDevStatus.MuxOneNandStatus;
static NvBootMuxOneNandParams s_DefaultMuxOneNandParams;
static NvBootMuxOneNandContext *s_pMuxOneNandContext = NULL;
static MuxOneNandFuseParam s_MuxOneNandFuseParams;

/*
 * The s_PageSizes[] table maps the fuse index values to page sizes in units
 * of log2 bytes.
 */
const static NvU32 s_PageSizes[4] =
{
    12, //  4KiB
    11, //  2KiB
    13, //  8KiB
    14  // 16KiB
};

/*
 * The s_BlockSizes[] table maps the fuse index values to block sizes in units
 * of log2 number of pages.
 */
const static NvU32 s_BlockSizes[8] =
{
     7, //  128 pages per block
     5, //   32 pages per block
     6, //   64 pages per block
     4, //   16 pages per block
     8, //  256 pages per block
     9, //  512 pages per block
    10, // 1024 pages per block
    11  // 2048 pages per block
};

/**
 * Load the data from the Nand flash memory to nand flash data memory
 */
static void 
MuxOneNandStartLoadData(
    const NvU32 BlockNumber, 
    const NvU32 PageNumber)
{
    NvU32 NandSysConfig;
    
    // Write the DFS, FBA of flash 
    // For 2GB it is 2048 blocks [10:0] on mux one nand
    // For 4Gb it is 1024 blocks [9:0] on flex mux one nand.
    // Selecting the [10:0] as we will never go beyond the number of existing 
    // block request from client and it is maximum side.
    // Support the lower flash core only.
    MUXONENANDFLASH_WRITE16(MUXONENAND_CHIPADD_BLOCK_ADD,
                            (BlockNumber & 0x7FF));
    
    // Select Dataram for DDP
    // Support the lower flash core only.
    MUXONENANDFLASH_WRITE16(MUXONENAND_CHIPADD_BUFFER_RAM, 0);
    
    // Write the page address
    MUXONENANDFLASH_WRITE16(MUXONENAND_PAGE_SECTOR_ADD, (PageNumber << 2)); 

    // Write BSA, BSC of dataRAM
    // All 4 sector in DATA RAM0
    // BSA[3] = 1 i.e. Bit 11 on register.
    MUXONENANDFLASH_WRITE16(MUXONENAND_BUFFER_NUMBER, (BSA_DATA_RAM_SELCECT)); 

    // Enable the ECC in system configuration register.
    NandSysConfig = MUXONENANDFLASH_READ16(MUXONENAND_MEM_CONFIG);
    NandSysConfig = NV_FLD_SET_DRF_DEF(MUXONENAND,
                                       SYSCONFIG,
                                       ECC,
                                       ENABLE,
                                       NandSysConfig);
    MUXONENANDFLASH_WRITE16(MUXONENAND_MEM_CONFIG, NandSysConfig); 

    // Write 0 to interrupt register
    MUXONENANDFLASH_WRITE16(MUXONENAND_INTERRUPT_STATUS, 0); 

    // Write Load command
    MUXONENANDFLASH_WRITE16(MUXONENAND_MEM_OP_COMMAND,
                            MUXONENNAD_COMMAND_DATALOAD); 
    
}

/**
 * Get the data load status from the flash storage memory to the flash dram 
 * memory.
 */
static NvBootDeviceStatus GetDataLoadStatus(
    NvBool IsNonFlexTypeDevice,
    NvU32  ReadStartTime)
{
    NvU16 IntStatus;
    NvU16 EccStatus;
    NvU32 OddSectorEcc;
    NvU32 EvenSectorEcc;
    NvU32 SectorPair;
    NvU16 SectorPairAddr;
    NvU16 ControllerStatus;

    // Check for Interrupt status DQ[15] from low to high transition.
    IntStatus = MUXONENANDFLASH_READ16(MUXONENAND_INTERRUPT_STATUS);
    if (!(IntStatus & INTERRUPT_STATUS_DQ_15))
    {
        if (NvBootUtilElapsedTimeUS(ReadStartTime) > SNOR_HW_TIMEOUT_US)
            return NvBootDeviceStatus_ReadFailure;
        else
            return  NvBootDeviceStatus_ReadInProgress;
    }

    // Check for load opeartion from controller status
    ControllerStatus = MUXONENANDFLASH_READ16(MUXONENAND_MEM_OP_STATUS);
    if (ControllerStatus & CONTROLLER_STATUS_ERROR_DQ_10)
        return NvBootDeviceStatus_ReadFailure;
 
    if (IsNonFlexTypeDevice)
    {
        // Check the Ecc Status for sectors 1 to 4 to see if correctable.
        EccStatus = MUXONENANDFLASH_READ16(MUXONENAND_ECC_STATUS);
        if (EccStatus & MUXONENAND_ECCSTATUS_ERROR_UNCORRECTABLE)
            return NvBootDeviceStatus_EccFailure;
    }
    else
    {
        // Check for Ecc Status for sector 1 to sector 8.
        // Sectors are checked in pairs.
        for (SectorPair = 0; SectorPair < 4; SectorPair++)
        {
            // Check for the ith sector pair.
            SectorPairAddr = FLEXMUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_1_2 +
                SectorPair;
        
            EccStatus      = MUXONENANDFLASH_READ16(SectorPairAddr);
            OddSectorEcc   = NV_DRF_VAL(FLEXMUXONENAND,
                                        ECCSTATUS,
                                        ODD_SECTOR, 
                                        EccStatus);
            EvenSectorEcc  = NV_DRF_VAL(FLEXMUXONENAND,
                                        ECCSTATUS,
                                        EVEN_SECTOR,
                                        EccStatus);
            if ((OddSectorEcc  > FLEXMUXONENAND_ECC_MAX_ERROR_CORRECTABLE) ||
                (EvenSectorEcc > FLEXMUXONENAND_ECC_MAX_ERROR_CORRECTABLE))
                return NvBootDeviceStatus_EccFailure;
        }
    }
    // Read is done, now host can read the data.        
    return NvBootDeviceStatus_Idle;
}

/*
 * Public Function Definitions.
 */
void 
NvBootMuxOneNandGetParams(
    const NvU32 ParamIndex,
    NvBootMuxOneNandParams **Params)
{
    NvU32 PageIndex;
    NvU32 BlockIndex;
    
    NV_ASSERT(Params != NULL);

    // Get the timing parameter and program it properly.
    s_DefaultMuxOneNandParams.ClockDivider = 1;
    s_DefaultMuxOneNandParams.ClockSource = NvBootMuxOneNandClockSource_ClockM;
    // Get the fuse parameters
    // Get the nand type
    s_MuxOneNandFuseParams.IsNonFlexTypeDevice = 
        NV_DRF_VAL(MUXONENAND_DEVICE, CONFIG, DISABLE_FLEX_TYPE, ParamIndex);
    s_MuxOneNandFuseParams.IsNonmuxedInterface =
        NV_DRF_VAL(MUXONENAND_DEVICE, CONFIG, DISABLE_MUX_INTERFACE, ParamIndex);
    PageIndex = NV_DRF_VAL(MUXONENAND_DEVICE, CONFIG, PAGE_SIZE, ParamIndex);
    BlockIndex = NV_DRF_VAL(MUXONENAND_DEVICE, CONFIG, BLOCK_SIZE, ParamIndex);
    // Get the page size in Log2
    s_MuxOneNandFuseParams.PageSizeLog2 = s_PageSizes[PageIndex];
    // Get the block size in Log2
    s_MuxOneNandFuseParams.BlockSizeLog2 = 
        s_MuxOneNandFuseParams.PageSizeLog2 + s_BlockSizes[BlockIndex];
    // Record data in the BIT.
    s_pMuxOneNandBitInfo->ClockDivider =
        s_DefaultMuxOneNandParams.ClockDivider;

    s_pMuxOneNandBitInfo->ClockSource = s_DefaultMuxOneNandParams.ClockSource;

    s_pMuxOneNandBitInfo->FuseBlockSize =
        1 << s_MuxOneNandFuseParams.BlockSizeLog2;

    s_pMuxOneNandBitInfo->FusePageSize = 
        1 << s_MuxOneNandFuseParams.PageSizeLog2;

    s_pMuxOneNandBitInfo->NumPagesRead = 0;
    s_pMuxOneNandBitInfo->LastBlockRead = 0;
    s_pMuxOneNandBitInfo->LastPageRead = 0;
    s_pMuxOneNandBitInfo->BootStatus = (NvU32)NvBootDeviceStatus_Idle;
    s_pMuxOneNandBitInfo->InitStatus = (NvU32)NvBootError_Success;
    s_pMuxOneNandBitInfo->ParamsValidated = (NvU32)NV_TRUE;

    *Params = &s_DefaultMuxOneNandParams;
}

NvBool 
NvBootMuxOneNandValidateParams(
    const NvBootMuxOneNandParams *pParams)
{
    NvBool ParamsValid = NV_TRUE;

    NV_ASSERT(pParams);

    if (pParams->ClockSource >= NvBootMuxOneNandClockSource_Num)
    {
        ParamsValid = NV_FALSE;

    }

    // Validate the Clock divider by checking for '0x0' and whether it
    // fits within 7 bits
    if ((pParams->ClockDivider == 0) || (pParams->ClockDivider > 0x7F))
    {
        ParamsValid = NV_FALSE;
    }

    s_pMuxOneNandBitInfo->ParamsValidated = (NvU32)ParamsValid;
    return ParamsValid;
}

void 
NvBootMuxOneNandGetBlockSizes(
    const NvBootMuxOneNandParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);

    *BlockSizeLog2 = s_MuxOneNandFuseParams.BlockSizeLog2;
    *PageSizeLog2  = s_MuxOneNandFuseParams.PageSizeLog2;
    
    s_pMuxOneNandBitInfo->FuseBlockSize =
        1 << s_MuxOneNandFuseParams.BlockSizeLog2;
    s_pMuxOneNandBitInfo->FusePageSize =
        1 << s_MuxOneNandFuseParams.PageSizeLog2;
}

NvBootError 
NvBootMuxOneNandInit(
    const NvBootMuxOneNandParams *Params, 
    NvBootMuxOneNandContext *pNandContext)
{
    NvU32 ConfigVal;
    NvU16 ReadStatus;
    NvBootError Error;
    NvU32 TimeOutCounter;
    
    NV_ASSERT(pNandContext);
    NV_ASSERT(Params);
    NV_ASSERT(((Params->ClockDivider != 0) && (Params->ClockDivider < 0x80)));
    NV_ASSERT(Params->ClockSource < NvBootMuxOneNandClockSource_Num);
    
    // Store the Context pointer
    s_pMuxOneNandContext = pNandContext;
    s_pMuxOneNandContext->IsFlashLoading = NV_FALSE;

    s_pMuxOneNandContext->IsNonFlexTypeDevice =
        s_MuxOneNandFuseParams.IsNonFlexTypeDevice;

    s_pMuxOneNandContext->IsNonMuxedInterface=
        s_MuxOneNandFuseParams.IsNonmuxedInterface;

    s_pMuxOneNandContext->BlockSizeLog2 = s_MuxOneNandFuseParams.BlockSizeLog2;
    s_pMuxOneNandContext->PageSizeLog2  = s_MuxOneNandFuseParams.PageSizeLog2;
    s_pMuxOneNandContext->ReadStartTime = 0;
    s_pMuxOneNandContext->pReadBuffer   = NULL;

    s_pMuxOneNandContext->PageSizeInBytes =
        1 << s_MuxOneNandFuseParams.PageSizeLog2;

    
    // Enable the Clock for SPI Controller
    NvBootClocksSetEnable(NvBootClocksClockId_SnorId, NV_TRUE);
    
    // Configure the clock source.
    NvBootClocksConfigureClock(NvBootClocksClockId_SnorId, 
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(Params->ClockDivider, 0), 
        Params->ClockSource);

    s_pMuxOneNandBitInfo->ClockDivider = Params->ClockDivider;
    s_pMuxOneNandBitInfo->ClockSource  = Params->ClockSource;
    
    // Reset of the nor controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_TRUE);
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_FALSE);


    ConfigVal = SNOR_REG_READ32(CONFIG);

    // Select mux one nand type device
    ConfigVal = NV_FLD_SET_DRF_DEF(SNOR,
                                   CONFIG,
                                   NOR_DEVICE_TYPE,
                                   MUXONENAND,
                                   ConfigVal);

    ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, SNOR_SEL,CS0, ConfigVal);

    // Select only 16 bit interface.
    // Select Mux/nonmuxed interface based on the fuse parameter.
    // Configure the mux mode.
    if (s_MuxOneNandFuseParams.IsNonmuxedInterface)
    {
        ConfigVal = NV_FLD_SET_DRF_DEF(SNOR,
                                       CONFIG,
                                       MUXMODE_GMI,
                                       AD_NONMUX,
                                       ConfigVal);
    }
    else
    {
        ConfigVal = NV_FLD_SET_DRF_DEF(SNOR,
                                       CONFIG,
                                       MUXMODE_GMI,
                                       AD_MUX,
                                       ConfigVal);
    }
    
    // Configure the 16 bit mode of snor controller.
    ConfigVal = NV_FLD_SET_DRF_DEF(SNOR,
                                   CONFIG,
                                   WORDWIDE_GMI,
                                   NOR16BIT,
                                   ConfigVal);

    SNOR_REG_WRITE32(CONFIG, ConfigVal);
    if (s_MuxOneNandFuseParams.IsNonmuxedInterface)
        Error = NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_MuxOneNand, 
                                              NvBootPinmuxConfig_OneNand);
    else
        Error = NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_MuxOneNand, 
                                              NvBootPinmuxConfig_MuxOneNand);

    s_pMuxOneNandBitInfo->InitStatus = (NvU32)Error;
    if (Error)
        return Error;

    // Send reset command to the nand
    MUXONENANDFLASH_WRITE16(MUXONENAND_MEM_OP_COMMAND, 0xF3);

    // Wait for the device ready.
    TimeOutCounter = MUXONENAND_RESET_TIMEOUT_US;
    while(TimeOutCounter)
    {
        ReadStatus = MUXONENANDFLASH_READ16(MUXONENAND_MEM_OP_STATUS);
        if (!(ReadStatus & RESET_OPERATION_BUSY))
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
    }

    return NvBootError_Success;
}

NvBootError 
NvBootMuxOneNandReadPage(
    const NvU32 BlockNumber, 
    const NvU32 PageNumber, 
    NvU8 *pReadBuffer)
{
    NV_ASSERT(pReadBuffer != NULL);
    NV_ASSERT(PageNumber < (1 << (s_MuxOneNandFuseParams.BlockSizeLog2 - 
                                  s_MuxOneNandFuseParams.PageSizeLog2)));

    s_pMuxOneNandBitInfo->NumPagesRead++;
    s_pMuxOneNandBitInfo->LastBlockRead = BlockNumber;
    s_pMuxOneNandBitInfo->LastPageRead = PageNumber;
    s_pMuxOneNandBitInfo->BootStatus =
        (NvU32)NvBootDeviceStatus_ReadInProgress;

    // First Load the data from flash memory to the flash ram
    MuxOneNandStartLoadData(BlockNumber, PageNumber);

    s_pMuxOneNandContext->ReadStartTime = NvBootUtilGetTimeUS();
    s_pMuxOneNandContext->IsFlashLoading = NV_TRUE;
    s_pMuxOneNandContext->pReadBuffer = pReadBuffer;
    s_pMuxOneNandContext->PageSizeInBytes = 
        (1 << s_pMuxOneNandContext->PageSizeLog2);

    return NvBootError_Success;
}

NvBootDeviceStatus NvBootMuxOneNandQueryStatus(void)
{
    NvBootDeviceStatus Status;
    NvU32 WordRequested;
    NvU32 ReadIndex;
    NvU16 *pReadBuff;
    if (s_pMuxOneNandContext->IsFlashLoading == NV_TRUE)
    {
        Status = GetDataLoadStatus(s_pMuxOneNandContext->IsNonFlexTypeDevice,
                                   s_pMuxOneNandContext->ReadStartTime);
        
        s_pMuxOneNandBitInfo->BootStatus = (NvU32)Status;
        if (Status == NvBootDeviceStatus_ReadInProgress)
            return Status;
            
        s_pMuxOneNandContext->IsFlashLoading = NV_FALSE;

        // If Ecc error then return the Ecc failure.
        if ((Status == NvBootDeviceStatus_EccFailure) ||
            (Status == NvBootDeviceStatus_ReadFailure))
        {                
            return Status;   
        }
        
        // Now start reading the data from the flash memory
        WordRequested = s_pMuxOneNandContext->PageSizeInBytes >> 1;
        pReadBuff = (NvU16 *)s_pMuxOneNandContext->pReadBuffer;
        for (ReadIndex = 0; ReadIndex < WordRequested; ++ReadIndex)
            *pReadBuff++ = MUXONENANDFLASH_READ16((MUXONENAND_DATA_RAM0_MAIN +
                                                   ReadIndex));
    }
    
    // Read is a success. If the decyrption address is to external memory, make
    // sure the writes to memory are coherent. 
    if (NvBootAhbCheckIsExtMemAddr((NvU32 *)s_pMuxOneNandContext->pReadBuffer)) 
    {
        NvBootAhbWaitCoherency(ARAHB_MST_ID_NAND);
    }

    s_pMuxOneNandBitInfo->BootStatus = (NvU32)NvBootDeviceStatus_Idle;
    return NvBootDeviceStatus_Idle;
}

void NvBootMuxOneNandShutdown(void)
{
    // Enable - Reset of the nor controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_TRUE);

    // Disable the Clock for SPI Controller
    NvBootClocksSetEnable(NvBootClocksClockId_SnorId, NV_FALSE);

    s_pMuxOneNandContext = NULL;
}

NvBootError NvBootMuxOneNandGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
