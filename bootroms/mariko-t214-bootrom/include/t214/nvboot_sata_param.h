/*
 * Copyright (c) 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * Defines the parameters and data structure for SATA devices.
 */

#ifndef INCLUDED_NVBOOT_SATA_PARAM_H
#define INCLUDED_NVBOOT_SATA_PARAM_H

#include "nvcommon.h"

#if defined(__cplusplus)
extern "C"
{
#endif


typedef enum
{
    /// Specifies Legacy mode
    NvBootSataMode_Legacy = 1,

    /// Specifies AHCI mode compliant with AHCI 1.3 spec
    NvBootSataMode_Ahci,

    NvBootSataMode_Num,

    NvBootSataMode_Force32 = 0x7FFFFFF
} NvBootSataMode;

typedef enum
{
    /// Specifies Pio mode transfer from device to memory by SATA controller
    NvBootSataTransferMode_Pio = 1,

    /// Specifies DMA transfer from device to memory by SATA controller
    NvBootSataTransferMode_Dma,

    NvBootSataTransferMode_Num,

    NvBootSataTransferMode_Force32 = 0x7FFFFFF
} NvBootSataTransferMode;

typedef enum {
    /// Specifies SATA clock source to be PLLP.
    NvBootSataClockSource_PllPOut0 = 0,

    /// Specifies SATA clock source to be PLLC4_Out0.
    NvBootSataClockSource_PllC4Out0 = 3,

    /// Specifies SATA clock source to be PLLC4_Out0.
    NvBootSataClockSource_PllC4Out1 = 5,
    
    /// Specifies SATA clock source to be ClkM.
    NvBootSataClockSource_ClkM = 6,

    /// Supported number of clock sources for SATA
    NvBootSataClockSource_Num,

    /// Specifies SATA clock source to be PLLC.
    /// Supported as clock source for SATA but not suppoted in bootrom because
    /// PLLC is not enabled in bootrom.
    NvBootSataClockSource_PllCOut0,

    /// Pllc4_Out2 may not be needed considering that
    /// Pllc4_Out0 and Pllc4_Out1 serve as alternatives to 
    /// Pll meant for peripherals.
    NvBootSataClockSource_Pllc4Out2,

    NvBootSataClockSource_Force32 = 0x7FFFFFF

} NvBootSataClockSource;


typedef enum {
    /// Specifies SATA clock source to be PLLP.
    NvBootSataOobClockSource_PllPOut0 = 0,

    /// Specifies SATA clock source to be PLLC4_Out0.
    NvBootSataOobClockSource_PllC4Out0,

    /// Specifies SATA clock source to be PLLC4_Out0.
    NvBootSataOobClockSource_PllC4Out1 = 3,
    
    /// Specifies SATA clock source to be ClkM.
    NvBootSataOobClockSource_ClkM = 6,

    /// Supported number of clock sources for SATA
    NvBootSataOobClockSource_Num,

    /// Specifies SATA clock source to be PLLC.
    /// Supported as clock source for SATA but not suppoted in bootrom because
    /// PLLC is not enabled in bootrom.
    NvBootSataOobClockSource_PllCOut0,

    /// Pllc4_Out2 may not be needed considering that
    /// Pllc4_Out0 and Pllc4_Out1 serve as alternatives to 
    /// Pll meant for peripherals.
    NvBootSataOobClockSource_Pllc4Out2,

    NvBootSataOobClockSource_Force32 = 0x7FFFFFF

} NvBootSataOobClockSource;



typedef struct {
    /// Holds value of PLLE_SSCMAX(#bits = 9) for SSA
    NvU16 PlleSSCoeffSscmax;
    
    /// Holds value of PLLE_SSCINC(#bits = 8)
    NvU8 PlleSSCoeffSscinc;

    /// Holds value of PLLE_SSCINCINTRVL(#bits = 6) for SSA
    NvU8 PlleSSCoeffSscincintrvl;

    /// Holds value of PLLE_INTEGOFFSET(#bits = 2) for SSA
    NvU8 PlleSSCoeffIntegoffset;

    /// Holds value of PLLE_SSCCENTER(#bits = 1) for SSA
    NvBool PlleSSCoeffSsccenter;

    /// Holds value of PLLE_SSCINVERT(#bits = 1) for SSA
    NvBool PlleSSCoeffSscinvert;

} NvBootPlleSsaCoeff;

typedef struct {
    /// Holds value of PLLE_SSC_MAX(#bits = 16) for SSD
    NvU16 PlleSSCoeffSdmsscmax;
    /// Holds value of PLLE_SSC_MIN(#bits = 16) for SSD
    NvU16 PlleSSCoeffSdmsscmin;
    /// Holds value of PLLE_SSC_STEP#bits = 16) for SSD
    NvU16 PlleSSCoeffSdmsscstep;
    /// Holds value of PLLE_SSC_DIN(#bits = 16) for SSD
    NvU16 PlleSSCoeffSdmdin;
} NvBootPlleSsdCoeff;

typedef union {
  NvBootPlleSsaCoeff PlleSsaCoeff;
  NvBootPlleSsdCoeff PlleSsdCoeff;
} NvBootSataPlleSSCoeff;
/**
 * Defines the parameters SATA devices.
 */
typedef struct NvBootSataParamsRec
{
    /// Specifies the clock source for SATA controller.
    NvBootSataClockSource SataClockSource;

    /// Specifes the clock divider to be used.
    NvU8 SataClockDivider;

    /// Specifies the clock source for SATA controller.
    NvBootSataClockSource SataOobClockSource;

    /// Specifes the clock divider to be used.
    NvU8 SataOobClockDivider;

    /// Specifies the SATA mode to be used.
    NvBootSataMode SataMode;

    /// Specifies mode of transfer from device to memory via controller.
    /// Note: transfer mode doesn't imply SW intervention.
    NvBootSataTransferMode TransferMode;

    /// If SDRAM is initialized, then AHCI DMA mode can be used.
    /// If SDRAM is not initialized, then always use legacy pio mode.
    NvBool isSdramInitialized;

   /// Start address(in SDRAM) for command and FIS structures.
    NvU32 SataBuffersBase;

   /// Start address(in SDRAM) for data buffers.
    NvU32 DataBufferBase;

    /// Plle ref clk divisors
    NvU8 PllRefeDivM;

    NvU8 PllRefeDivN;

    NvU8 PllRefeDivP;

    NvU32 PllRefeMisc;

    /// Plle ref clk divisors
    NvU8 PlleDivM;

    NvU8 PlleDivN;

    NvU8 PlleDivPlCml;

    NvU32 PlleMisc;

    NvBool PlleRefSrc;

    /// Plle SS coefficients whcih will take effect only when
    /// Plle SS is enabled through the boot dev config fuse
    NvBootSataPlleSSCoeff PlleSSCoeff;

    NvBool IsPlleSSA;

    NvBool EnablePlleSS;

} NvBootSataParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SATA_PARAM_H */
