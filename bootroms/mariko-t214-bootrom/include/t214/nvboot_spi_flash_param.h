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
 * Defines the parameters and data structure for SPI FLASH devices.
 */

#ifndef INCLUDED_NVBOOT_SPI_FLASH_PARAM_H
#define INCLUDED_NVBOOT_SPI_FLASH_PARAM_H

#if defined(__cplusplus)
extern "C"
{
#endif
//Clock sources supported by spi driver in bootrom are pllp and clkm only
typedef enum 
{
    /// Specifies SPI clock source to be PLLP.
    NvBootSpiClockSource_PllPOut0 = 0,

    /// Specifies SPI clock source to be PLLC.
    //NvBootSpiClockSource_PllCOut0 = 2,

    /// Specifies SPI clock source to be PLLC4_Out2.
    NvBootSpiClockSource_PllC4_Muxed = 4,

    /// Specifies SPI clock source to be ClockM.
    NvBootSpiClockSource_ClockM = 6,

    NvBootSpiClockSource_Num,
    NvBootSpiClockSource_Force32 = 0x7FFFFFF
} NvBootSpiClockSource;

typedef enum
{
    NvBootSpiDataWidth_x1 = 0,

    NvBootSpiDataWidth_x2,

    NvBootSpiDataWidth_x4,

    NvBootSpiDataWidth_Num,
    NvBootSpiDataWidth_Force32 = 0x7FFFFFF

} NvBootSpiDataWidth;

typedef enum
{
    NvBootSpiXferMode_Pio = 0,

    NvBootSpiXferMode_Dma,

    NvBootSpiXferMode_Num,
    NvBootSpiXferMode_Force32 = 0x7FFFFFF

} NvBootSpiXferMode;

typedef enum {
	NvBootSpiFlashCSActivePolarity_ActiveLow = 0,
	NvBootSpiFlashCSActivePolarity_ActiveHigh = 1,
	NvBootSpiFlashCSActivePolarity_Num,
	NvBootSpiFlashCSActivePolarity_Force32 = 0x7fffffff
} NvBootSpiFlashCSActivePolarity;

/**
 * Defines the parameters for SPI FLASH devices.
 */
typedef struct NvBootSpiFlashParamsRec
{
    /** 
     * Specifies the Config supported by driver.
     * Spi Config (0-2).
     */
    int8_t SpiConfig;

} NvBootSpiFlashParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SPI_FLASH_PARAM_H */

