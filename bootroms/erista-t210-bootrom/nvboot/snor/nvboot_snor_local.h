/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_nor_local.h - Internal definitions for using NOR as a second level
 * boot device.
 */

#ifndef INCLUDED_NVBOOT_NOR_LOCAL_H
#define INCLUDED_NVBOOT_NOR_LOCAL_H

#if defined(__cplusplus)
extern "C"
{
#endif

/* Insert internal definitions here */
#define NVBOOT_NOR_BLOCK_SIZE_LOG2 15  /* 32*1024 */
#define SECTOR_SIZE_LOG2  9            /*     512 */
#define NVBOOT_NOR_PLLP_OUT0_58MHZ_DIV  7

/**
 * SNOR Mux mode
 */
typedef enum{
    Snor_Muxed = 0,
    Snor_NonMuxed,
} SnorNonMuxed;

/**
 * SNOR width
 */ 
typedef enum {
    SnorWidth16bit = 0,
    SnorWidth32bit,
} SnorWidth;
/**
 * Identifies the relative timing of RDY signal w.r.t. data start.
 */

/**
 * Identifies the device mode to be used for transfers.
 */
typedef enum {
    /// Default mode is Async. Enum can start at 0 because device
    /// should be able to boot even when boot_dev_cfg is 0(or not set)
    SnorDeviceMode_Async = 0,
    SnorDeviceMode_Page,
    SnorDeviceMode_Max,
    SnorDeviceMode_Force32  = 0x7FFFFFFF,
} SnorDeviceMode;

/**
 * Identifies the Page size in case page mode is used.
 */
typedef enum {
    /// Default page size is the burst length. Enum can start at 0 because
    /// device should be able to boot even when boot_dev_cfg is 0(or not set)
    SnorPageSize_Burst = 0,

    /// Page Size of Four DWORDs
    SnorPageSize_4Word,

    /// Page Size of Eight DWORDs
    SnorPageSize_8Word,

    /// Page Size of Sixteen DWORDs
    SnorPageSize_16Word,

    SnorPageSize_Max,
    SnorPageSize_Force32 = 0x7FFFFFFF

} SnorPageSize;


/**
 * Identifies the max one-shot transfer data size.
 */
typedef enum {
    /// Default is 512 bytes transfer. Enum can start at 0 because
    /// device should be able to boot even when boot_dev_cfg is 0(or not set)
    SnorXferDataSize_512Bytes = 0,

    SnorXferDataSize_1KBytes,

    SnorXferDataSize_2KBytes,

    SnorXferDataSize_4KBytes,

    SnorXferDataSize_8KBytes,

    SnorXferDataSize_Max,
    SnorXferDataSize_Force32 = 0x7FFFFFFF
} SnorXferDataSize;

/**
 * Shift constants used to extract boot device config fields
 */
 
typedef enum {
    SnorFuseParamOffset_Width = 0,   //  Bits [0:0] 0x0 = x16 , 0x1 = x32

    SnorFuseParamOffset_NonMuxed = 1,   //  Bits [1:1] 0x0= Muxed, 0x1= Non Muxed

    SnorFuseParamOffset_UseNorTimingParams = 2,     // Bits [2:2] 1=Use hard coded timings from BootROM

    SnorFuseParamOffset_ChipSelect = 3,     // Bits [3:3]
    
    SnorFuseParamOffset_Reserved = 4,     // Bits [5:4]
   
    SnorFuseParamOffset_DeviceMode = 6, // Bits [6:6]  0x0=Async, 0x1=Async Page  
    
    SnorFuseParamOffset_DataXferMode = 7, // Bits [7:7]  0x0=PIO, 0x1=DMA
    
    SnorFuseParamOffset_PageSize = 8, // Bits [9:8] Page mode Page Size.

    SnorFuseParamOffset_XferDataSizeLog2 = 10, // Bits [12:10]  I am not sure if we need this.
    
    SnorFuseParamOffset_HighSpeed = 13, // Bits [13:13]  Default clock speed = Osc Speed. High Speed = PLLP_OUT0/7.
    
    SnorFuseParamOffset_Max,
    SnorFuseParamOffset_Force32 = 0x7FFFFFFF
}SnorFuseParamOffset;

/**
 * Mask constants used to extract boot device config fields
 */
typedef enum {
    /// Mask to extract 1 Bit wide field
    SnorFuseParamMask_BitWidth1 = 0x1,

    /// Mask to extract 2 Bits wide field
    SnorFuseParamMask_BitWidth2 = 0x3,

    /// Mask to extract 3 Bits wide field
    SnorFuseParamMask_BitWidth3 = 0x7,

    SnorFuseParamMask_Max,
    SnorFuseParamMask_Force32 = 0x7FFFFFFF
}SnorFuseParamMask;

/**
 * Constants to indicate the direction of data transfer
 */
typedef enum {
    SnorDmaDirection_NORtoAHB = 0,
    SnorDmaDirection_AHBtoNOR,
    
    SnorDmaDirection_Force32 = 0x7FFFFFFF
  
} SnorDmaDirection;

/**
 * Shift constants used to extract snor boot info fields
 */
typedef enum {
    /// Bits [0:0]
    SnorBitFldOffset_SnorConfig_BlockSizeLog2 = 0,
    /// Bits [15:8]
    SnorBitFldOffset_SnorConfig_XferSizeLog2 = 8,
    /// Bits [17:16]
    SnorBitFldOffset_SnorConfig_SnorPageSize = 16,
    /// Bits [19:18]
    SnorBitFldOffset_SnorConfig_SnorBurstLength = 18,
    /// Bits [20:20]
    SnorBitFldOffset_SnorConfig_DeviceMode = 20,
    /// Bits [21:21]
    SnorBitFldOffset_SnorConfig_DataXferMode = 21,

    /// Bits [0:0]
    SnorBitFldOffset_SnorDriverStatus_ParamsValidated = 0,
    /// Bits [8:1]
    SnorBitFldOffset_SnorDriverStatus_InitStatus = 1,
    /// Bits [16:9]
    SnorBitFldOffset_SnorDriverStatus_ReadPageStatus = 9,
    /// Bits [17:17]
    SnorBitFldOffset_SnorDriverStatus_WaitForControllerBsy = 17,
    /// Bits [18:18]
    SnorBitFldOffset_SnorDriverStatus_WaitForDmaDone = 18,
    /// Bits [19:19]
    SnorBitFldOffset_SnorDriverStatus_ControllerBsy = 19,
    /// Bits [20:20]
    SnorBitFldOffset_SnorDriverStatus_DmaDone = 20,

    SnorBitFldOffset_Force32 = 0x7FFFFFFF
}SnorBitFldOffset;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_NOR_LOCAL_H */
