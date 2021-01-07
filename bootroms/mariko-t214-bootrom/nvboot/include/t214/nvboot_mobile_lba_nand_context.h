/*
 * Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * nvboot_mobile_lba_nand_context.h - Definitions for the MobileLbaNand 
 * context structure.
 */

#ifndef INCLUDED_NVBOOT_MOBILE_LBA_NAND_CONTEXT_H
#define INCLUDED_NVBOOT_MOBILE_LBA_NAND_CONTEXT_H

#include "nvboot_device_int.h"
#include "nvboot_mobile_lba_nand_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#define NVBOOT_ML_NAND_SPARE_AREA_SIZE_IN_BYTES 16

/// Defines various commands as per MobileLbaNand spec.
typedef enum
{
    NvBootMlnCommands_Reset       = 0xFD,
    NvBootMlnCommands_ReadId      = 0x90,
    NvBootMlnCommands_ReadSda     = 0x00,
    NvBootMlnCommands_ReadMda     = 0x0A,
    NvBootMlnCommands_ReadStart   = 0x30,
    NvBootMlnCommands_StatusRead  = 0x70,
    NvBootMlnCommands_StatusRead2 = 0x71,
    NvBootMlnCommands_CommandNone = 0x00,
    
    NvBootMlnCommands_SetTransferProtocol         = 0x57,
    NvBootMlnCommands_SetTransferProtocol1Address = 0xA2,
    NvBootMlnCommands_SetTransferProtocol2Address = 0xA3,
    NvBootMlnCommands_SetMinimumBusyTimeAddress   = 0xA4,
    NvBootMlnCommands_SetTransferProtocol3Address = 0xA7,
    
    NvBootMlnCommands_GetMdaUnitAddress           = 0xB0,
    NvBootMlnCommands_GetSdaUnitAddress           = 0xB5,
    NvBootMlnCommands_GetTransferProtocol1Address = 0xB2,
    NvBootMlnCommands_GetTransferProtocol2Address = 0xB3,
    NvBootMlnCommands_GetMinimumBusyTimeAddress   = 0xB4,
    NvBootMlnCommands_GetTransferProtocol3Address = 0xB7,
    
    // Custom commands
    NvBootMlnCommands_ReadPio4Bytes = 0xC0,
    NvBootMlnCommands_ReadPio1Byte  = 0xC1,

    NvBootMlnCommands_Force32 = 0x7FFFFFFF
} NvBootMlnCommands;

/// Defines MobileLbaNand command structure.
typedef struct 
{
    NvBootMlnCommands Command1;
    NvBootMlnCommands Command2;
    NvU16 SectorCount;
    // It should be able to hold 5 bytes to support 7 address cycles.
    NvU64 SectorAddress;
} NvBootMlnCommand;

typedef enum
{
    NvBootMlnDataWidth_Discovey = 0,
    NvBootMlnDataWidth_8Bit,
    NvBootMlnDataWidth_16Bit,
    NvBootMlnDataWidth_Num,
    NvBootMlnDataWidth_Force32 = 0x7FFFFFFF
} NvBootMlnDataWidth;

/*
 * NvBootMobileLbaNandContext - The context structure for the MobileLbaNand driver.
 * A pointer to this structure is passed into the driver's Init() routine.
 * This pointer is the only data that can be kept in a global variable for
 * later reference.
 */
typedef struct NvBootMobileLbaNandContextRec
{
    /// Maker Code.
    NvU8 MakerCode;
    /// Device Code.
    NvU8 DeviceCode;
    /// Block size in Log2 scale.
    NvU8 BlockSizeLog2;
    /// Page size in Log2 scale.
    NvU8 PageSizeLog2;
    /// No of pages per block in Log 2 scale;
    NvU8 PagesPerBlockLog2;
    /// Number of address cycles.
    NvU8 NumOfAddressCycles;
    /// Data width.
    NvBootMlnDataWidth DataWidth;
    /// MobileLbaNand Command params.
    NvBootMlnCommand Command;
    /// Device status.
    NvBootDeviceStatus DeviceStatus;
    /// Holds the MobileLbaNand Read start time.
    NvU64 ReadStartTime;
    /// Buffer for reading spare area. Define as NvU32 to make it word aligned.
    NvU32 SpareAreaPerLba[NVBOOT_ML_NAND_SPARE_AREA_SIZE_IN_BYTES / sizeof(NvU32)];
    /// Holds the number of sectors that can be accessed from SDA.
    NvU32 SdaSectorCount;
    /// Holds the number of sectors that can be accessed from MDA.
    NvU32 MdaSectorCount;
    /// Store the current address of the external memory buffer being used. 
    NvU8 *CurrentReadBufferAddress;    
} NvBootMobileLbaNandContext;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MOBILE_LBA_NAND_CONTEXT_H */
