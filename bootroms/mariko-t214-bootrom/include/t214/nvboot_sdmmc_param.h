/*
 * Copyright (c) 2008 - 2011 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * Defines the parameters and data structure for eMMC and eSD devices, which
 * attach to the same SDMMC controller.
 */

#ifndef INCLUDED_NVBOOT_SDMMC_PARAM_H
#define INCLUDED_NVBOOT_SDMMC_PARAM_H

#if defined(__cplusplus)
extern "C"
{
#endif

/// Defines various data widths supported.
typedef enum
{
    /**
     * Specifies a 1 bit interface to eMMC.
     * Note that 1-bit data width is only for the driver's internal use.
     * Fuses doesn't provide option to select 1-bit data width.
     * The driver selects 1-bit internally based on need.
     * It is used for reading Extended CSD and when the power class
     * requirements of a card for 4-bit or 8-bit transfers are not
     * supported by the target board.
     */
    NvBootSdmmcDataWidth_1Bit = 0,

    /// Specifies a 4 bit interface to eMMC.
    NvBootSdmmcDataWidth_4Bit = 1,

    /// Specifies a 8 bit interface to eMMC.
    NvBootSdmmcDataWidth_8Bit = 2,

    /// Specifies a 4 bit Ddr interface to eMMC.
    NvBootSdmmcDataWidth_Ddr_4Bit = 5,

    /// Specifies a 8 bit Ddr interface to eMMC.
    NvBootSdmmcDataWidth_Ddr_8Bit = 6,

    NvBootSdmmcDataWidth_Num,
    NvBootSdmmcDataWidth_Force32 = 0x7FFFFFFF
} NvBootSdmmcDataWidth;

/// Defines the parameters that can be changed after BCT is read.
typedef struct NvBootSdmmcParamsRec
{
    /// Specifies the Config supported by driver.
    /// Sdmmc Config (0-4)
    int8_t SdmmcConfig;

    ///  only Reinit option.
    /** 
     * Max Power class supported by the target board.
     * The driver determines the best data width and clock frequency
     * supported within the power class range (0 to Max) if the selected
     * data width cannot be used at the chosen clock frequency.
     */
    NvU8 MaxPowerClassSupported;
} NvBootSdmmcParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SDMMC_PARAM_H */

