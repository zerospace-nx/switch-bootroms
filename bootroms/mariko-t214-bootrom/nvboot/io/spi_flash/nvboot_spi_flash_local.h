/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * nvboot_spi_flash_local.h - Defines semantics of BIT status for SPI flash reads
 * and defines required operations to set/get status.
 */

#ifndef INCLUDED_NVBOOT_SPI_FLASH_LOCAL_H
#define INCLUDED_NVBOOT_SPI_FLASH_LOCAL_H

//#include "nvboot_spi_flash_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif


/* Range Constants */
#define STATUS_REGISTER_WRITE_IN_PROGRESS_MASK 0x1
#define STATUS_REGISTER_WRITE_ENABLED_MASK 0x2



#define SPI_MAX_BIT_LENGTH 31  // 32 bit transfer per dma/pio packet in packed/unpacked mode
#define SPI_8Bit_BIT_LENGTH 7  // 8 bit transfer per dma/pio  packet in packed/unpacked mode

/* Miscellaneous constants */
#define BYTES_PER_WORD 4

#define SPI_FLASH_BLOCK_SIZE_LOG2 15  /* 32*1024 */
#define SPI_FLASH_PAGE_SIZE_LOG2 11  /*  2*1024 */
#define SPI_FLASH_ERASE_PAGE_SIZE_LOG2 18  /*  256*1024 */
#define SPI_FLASH_WRITE_PAGE_SIZE_LOG2 6  /*  256*1024 */
#define SPI_FLASH_ERASE_TIME_MILLISECONDS  1000 /* 1 second */
#define BPMP_DMA_MMIO_BURST_SIZE_WORDS     16 /* 16 WORDS */

/// The following configs are defined for spi io driver/controller
/// Transfer mode/Datawidth/clock divisor/ and read command options
/// will be retrieved from these confgiration only.
/// Defines Configurations .
///   = 0 means, PIO_X1_20.4Mhz_NormalRead
///   = 1 means, PIO_X1_19.2Mhz_NormalRead
///   = 2 means, PIO_X4_51Mhz_QuadRead
///   = 3/4/5/6/7 means reserved.

typedef enum
{
    Spi_Config_0 = 0, // PIO_X1_20.4Mhz_NormalRead, pllp_out0
    Spi_Config_1 = 1, // PIO_X1_19.2Mhz_NormalRead, clk_m
    Spi_Config_2 = 2, // PIO_X4_51Mhz_QuadRead, pllp_out0
    Spi_Config_Num,
    Spi_Config_Num_Force32 = 0x7FFFFFFF,
} Spi_Config;

/*
 * Field shifts for BootStatus
 */
typedef enum 
{
    NvBootSpiFlashStatusFldShift_Initialized = 0,

    NvBootSpiFlashStatusFldShift_ReInitialized,

    NvBootSpiFlashStatusFldShift_ParamsValidated,

    NvBootSpiFlashStatusFldShift_InvalidParams,

    NvBootSpiFlashStatusFldShift_DataTransferTimeout,

    NvBootSpiFlashStatusFldShift_ReadFailure,

    NvBootSpiFlashStatusFldShift_WriteFailure,

    NvBootSpiFlashStatusFldShift_QuadModeEnabled,
    NvBootSpiFlashStatusFldShift_WriteEnabled,
    NvBootSpiFlashStatusFldShift_QuadCommandSent,
    NvBootSpiFlashStatusFldShift_WriteDisabled,
    NvBootSpiFlashStatusFldShift_ReadRARDummyCycles,
    NvBootSpiFlashStatusFldShift_QuadModeCheck,
    NvBootSpiFlashStatusFldShift_QuadModeSetUnsetCheck,
    
   NvBootSpiFlashStatusFldShift_BpmpDmaBurstSizeError = 0x0,
   NvBootSpiFlashStatusFldShift_BpmpDmaBurstTypeError,
   NvBootSpiFlashStatusFldShift_BpmpDmaByteEnableError ,
   NvBootSpiFlashStatusFldShift_BpmpDmaAlignmentTypeError ,
   NvBootSpiFlashStatusFldShift_SpiTxFifoUnderrunError ,
   NvBootSpiFlashStatusFldShift_SpiRxFifoUnderrunError ,
   NvBootSpiFlashStatusFldShift_SpiTxFifoOverflowError ,
   NvBootSpiFlashStatusFldShift_SpiRxFifoOverflowError ,

} NvBootSpiFlashStatusFldShift;

/*
 * Field masks for BootStatus
 */
typedef enum 
{
    NvBootSpiFlashStatusFldMask_Initialized = 0x1,

    NvBootSpiFlashStatusFldMask_ReInitialized = 0x1,

    NvBootSpiFlashStatusFldMask_ParamsValidated = 0x1,

    NvBootSpiFlashStatusFldMask_InvalidParams = 0x1,

    NvBootSpiFlashStatusFldMask_DataTransferTimeout = 0x1,

    NvBootSpiFlashStatusFldMask_ReadFailure = 0x1,

    NvBootSpiFlashStatusFldMask_WriteFailure = 0x1,

    NvBootSpiFlashStatusFldMask_QuadModeEnabled = 0x1,
    NvBootSpiFlashStatusFldMask_WriteEnabled = 0x1,
    NvBootSpiFlashStatusFldMask_QuadCommandSent = 0x1,
    NvBootSpiFlashStatusFldMask_WriteDisabled = 0x1,
    NvBootSpiFlashStatusFldMask_ReadRARDummyCycles = 0x1,
    NvBootSpiFlashStatusFldMask_QuadModeCheck = 0x1,
    NvBootSpiFlashStatusFldMask_QuadModeSetUnsetCheck = 0x1,

   NvBootSpiFlashStatusFldMask_BpmpDmaBurstSizeError = 0x1,
   NvBootSpiFlashStatusFldMask_BpmpDmaBurstTypeError = 0x1,
   NvBootSpiFlashStatusFldMask_BpmpDmaByteEnableError = 0x1,
   NvBootSpiFlashStatusFldMask_BpmpDmaAlignmentTypeError = 0x1,
   NvBootSpiFlashStatusFldMask_SpiTxFifoUnderrunError = 0x1,
   NvBootSpiFlashStatusFldMask_SpiRxFifoUnderrunError = 0x1,
   NvBootSpiFlashStatusFldMask_SpiTxFifoOverflowError = 0x1,
   NvBootSpiFlashStatusFldMask_SpiRxFifoOverflowError = 0x1,
 
   
 
} NvBootSpiFlashStatusFldMask;

typedef enum
{
    NvBootSpiFlashCommand_Write = 0x02,
    NvBootSpiFlashCommand_Read = 0x03,
    NvBootSpiFlashCommand_WriteEnableDis = 0x04,
    NvBootSpiFlashCommand_ReadStatus = 0x05,
    NvBootSpiFlashCommand_WriteEnableEn = 0x06,
    NvBootSpiFlashCommand_FastRead = 0x0B,
    NvBootSpiFlashCommand_ReadConfigurationRegister = 0x35,
    NvBootSpiFlashCommand_BulkErase = 0x60,
    NvBootSpiFlashCommand_ReadAnyRegister = 0x65,
    NvBootSpiFlashCommand_WriteAnyRegister = 0x71,
    NvBootSpiFlashCommand_ReadId = 0x90,
    NvBootSpiFlashCommand_SectorErase = 0xD8,
    NvBootSpiFlashCommand_QuadRead = 0xEB,

} NvBootSpiFlashCommand;

typedef enum {
    NvBootSpiFlashQuadData_QuadDis = 0x00,
    NvBootSpiFlashQuadData_QuadEn = 0x02
} NvBootSpiFlashQuadData;

typedef enum {
    NvBootSpiFlashSpansionReg_CR1V = 0x800002,
    NvBootSpiFlashSpansionReg_CR2V = 0x800003

} NvBootSpiFlashSpansionReg;
/**
 * Specifies if the address is to be translated from Physical to Virtual 
 * or vice versa.
 * Alternatively, the usage of this enumeration could have been replaced by Mode (pio/dma)
 * if all drivers needed address translation for source/destination in memory.
 */
typedef enum {
  XlateAddr_GetBpmpAddr = 0,
  XlateAddr_GetPhysicalAddr,
  XlateAddr_Num
} XlateAddr;

typedef enum
{
    SpiFlashConstantTableIdx_ClockDivisorsTable = 0, 
    SpiFlashConstantTableIdx_TrimmersConfigTable, 
    SpiFlashConstantTableIdx_SpiFlashXferTimeoutTable,
    SpiFlashConstantTableIdx_QReadConfigTable,
    SpiFlashConstantTableIdx_Num,
}SpiFlashConstantTableIdx;

typedef struct
{
       NvU32 setup;
	
	NvU32 kcp;
	
	NvU32 kvco;
	
	NvU32 divm;
	
	NvU32 divn;
	
	NvU32 divp;
	
} NvBootSpiFlashPllParams;

typedef struct
{
    uint8_t TxTapDelay;

    uint8_t RxTapDelay;

    uint8_t Data0LineTapDelay;

    uint8_t Data1LineTapDelay;

    uint8_t Data2LineTapDelay;

    uint8_t Data3LineTapDelay;
} NvBootSpiFlashTrimmers;

typedef struct
{
    uint8_t TxFifoTriggerLevel;

    uint8_t RxFifoTriggerLevel;

    uint8_t TxFifoDepth;

    uint8_t RxFifoDepth;

} NvBootSpiFlashDataXferConfig;

typedef struct
{
   NvU32 PioReadTimeout;

   NvU32 DmaReadTimeout;
} NvBootSpiFlashTimeouts;

typedef struct  {
    uint8_t LatencyCode;
    uint8_t ModeCycles;
    uint8_t DummyCycles;
} NvBootSpiFlashQuadSdrLC;

typedef struct {
    uint8_t ChannelIdx;
    NvU32 BpmpChannelMcSeq;
    NvU32 BpmpChannelMmioSeq;
    NvU32 BpmpChannelVirtualizationEnable;
    NvU32 BpmpChannelCsr;
}NvBootSpiFlashBpmpChannelConfig;


// Macros to set/retrieve fields and bit fields in SpiFlashStatus in BootInfoTable
/*
 * f = field name of SpiFlash status in BIT
 * v = value to be assigned to field f
 */
#define SET_SPI_FLASH_BOOT_INFO_FLD(f,v) \
do { \
    s_pSpiFlashBitInfo->f = v; \
} while(0);

/*
 * f = field name of SpiFlash status in BIT
 * v = mask value for the bit field in f
 */
#define SET_SPI_FLASH_BOOT_INFO_BITFLD(f,v) \
do { \
    s_pSpiFlashBitInfo->f |=  1 << (NvBootSpiFlashStatusFldShift_##v);  \
} while(0);

#define GET_SPI_FLASH_BOOT_INFO_BITFLD(f, v) (s_pSpiFlashBitInfo->f  >> (NvBootSpiFlashStatusFldShift_##v) \
    & NvBootSpiFlashStatusFldMask_##v) 

#define GET_SPI_FLASH_BOOT_INFO_FLD(f) s_pSpiFlashBitInfo->f


// Wrapper macros for reading/writing from/to SPI
#define SPI_REG_READ32(reg) \
        NV_READ32(NV_ADDRESS_MAP_QSPI_BASE + (QSPI_##reg##_0))
        
#define SPI_REG_WRITE32(reg, val) \
    do { \
        NV_WRITE32((NV_ADDRESS_MAP_QSPI_BASE + (QSPI_##reg##_0)), (val)); \
    } while (0)

#if defined(__cplusplus)
}    NvBootSpiFlashStatusFldMask_QuadModeEnabled = 0x1,
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SPI_FLASH_LOCAL_H */

