/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * nvboot_spi_flash_context.h - Definitions for the SPI_FLASH context
 * structure.
 */

#ifndef INCLUDED_NVBOOT_SPI_FLASH_CONTEXT_H
#define INCLUDED_NVBOOT_SPI_FLASH_CONTEXT_H

#include "nvboot_spi_flash_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif


/**
 * NvBootSpiFlashContext - The context structure for the SPI_FLASH driver.
 * A pointer to this structure is passed into the driver's Init() routine.
 * This pointer is the only data that can be kept in a global variable for
 * later reference.
 */
typedef struct NvBootSpiFlashContextRec
{
    /**
     * Chip select polarity - will be selected through BOOT_DEVICE_INFO fuse only
     * Can't be set again through BCT because it doesn't make any sense to set it htrough BCT
     * Once set in context using the fuse, the driver will use the context and not the params to 
     * to drive chip select.
     */
     NvBootSpiFlashCSActivePolarity ChipSelectPolarity;

    /// Read Command  Type to be used for Read
    NvBool ReadCommandTypeFast;
    /// Page Read Start time in Micro Seconds, Used for finding out timeout.
    NvU32 ReadStartTimeinUs;
    /// !!! Store the current address of the external memory buffer being used. 
    uint8_t *CurrentReadBufferAddress;
    /// Block size in Log2 scale.
    uint8_t BlockSizeLog2;
    /// Page size in Log2 scale.
    uint8_t PageSizeLog2;

    /**
     * Specifies the clock source to use.
     */
    NvBootSpiClockSource ClockSource;
    /**
     * Specifes the clock divider to use.
     * The value is a 7-bit value based on an input clock of 432Mhz.
     * Divider = (432+ DesiredFrequency-1)/DesiredFrequency;
     * Typical values:
     *     NORMAL_READ at 20MHz: 22
     *     FAST_READ   at 33MHz: 14
     *     FAST_READ   at 40MHz: 11
     *     FAST_READ   at 50MHz:  9
     */
    uint8_t ClockDivider;

    /// Data Width
    NvBootSpiDataWidth BusWidth;
    /// Xfer Mode
    NvBootSpiXferMode XferMode;
    /// Erase SectorSize on Log2 scale
    NvU32 EraseSectorSizeLog2;
    /// Time for Sector Erase to complete
    NvU32 EraseTime;
    /// Write page size on Log2 scale
    NvU32 WritePageSizeLog2;

    uint8_t TxFifoDepth;

    uint8_t RxFifoDepth;	

    uint8_t TxFifoTriggerLevel;

    uint8_t RxFifoTriggerLevel;

    NvU64 DataXferTimeout;

    uint8_t ReadRegisterDummyCycles;

    uint8_t QuadReadDummyCycles;	
 } NvBootSpiFlashContext;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SPI_FLASH_CONTEXT_H */

