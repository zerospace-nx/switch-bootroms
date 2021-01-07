/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_sata_local.h - Internal definitions for using SATA as a second level
 * boot device.
 */

#ifndef INCLUDED_NVBOOT_SATA_LOCAL_H
#define INCLUDED_NVBOOT_SATA_LOCAL_H

#include "nvcommon.h"
#include "nvboot_clocks_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/* Internal definitions */
#define NVBOOT_SATA_BLOCK_SIZE_LOG2 15  /* 32*1024 */

#define  CMD_READ_SECTOR                       0x20
#define  CMD_WRITE_SECTOR                      0x30

#define  CMD_READ_DMA                          0xc8
#define  CMD_WRITE_DMA                         0xca

#define NVBOOT_SATA_PAD_PLL_CAL_DONE_WAIT 200
#define NVBOOT_SATA_PAD_PLL_RCAL_DONE_WAIT 5
#define NVBOOT_SATA_PAD_PLL_LOCK_WAIT    20

// To account for small violations in timeouts and inadvertent error flags being set.
#define NVBOOT_SATA_PAD_PLL_WAIT_MARGIN 20

#define NVBOOT_SATA_DATA_TRANSFER_TIMEOUT    1000000
#define NVBOOT_SATA_COMMAND_COMPLETE_TIMEOUT    1000000
#define NVBOOT_SATA_BUSY_TIMEOUT    3000000

#define NUM_SATA_XUSB_PAD_PLL_REGS    11
#define NUM_SATA_XUSB_MISC_PAD_REGS    9

/*
 * x : Field name
 * c : = d (digital SS), = a (analog SS)
 */ 
#define SET_PLLE_SS_COEFF(x, c) \
do { \
    SSCoefficientTable##.##x = Params->PlleSSCoeff.PlleSs##c##Coeff.PlleSSCoeff##x; \
} while(0);


/*
 * f = field name of Sata status in BIT
 * v = value to be assigned to field f
 */
#define SET_SATA_BOOT_INFO_FLD(f,v) \
do { \
    s_pSataBitInfo->##f = v; \
} while(0);

/*
 * f = field name of Sata status in BIT
 * v = mask value for the bit field in f
 */
#define SET_SATA_BOOT_INFO_BITFLD(f,v) \
do { \
    s_pSataBitInfo->##f |= v; \
} while(0);

#define GET_SATA_BOOT_INFO_FLD(f) s_pSataBitInfo->##f

/*
 * idx : BOOT_DEVICE_INFO value
 * f : Fuse param (field in BOOT_DEVICE_INFO)
 * v : Fuse param value
 */
#define GET_FUSE_PARAM(idx, f, v) \
do { \
    s_SataFuseParamData.##v = (idx & f##_FUSE_MASK) >> f##_FUSE_SHIFT; \
} while(0);

/*
 * Sets the value v to the field f of register r
 */
#define SATA0_DFPCI_SET_FLD(r, f, v, rval) \
do { \
    rval = NV_FLD_SET_DRF_NUM(SATA0, r, f, v, rval); \
} while(0);

/*
 * Reads value of register r
 */
#define SATA0_DFPCI_READ32(r, v) \
do { \
    v = NV_READ32(NV_SATA_APB_DFPCI_CFG + T_SATA0_##r); \
} while(0);


/*
 * Reads value of register r
 */
#define SATA0_DFPCI_WRITE32(r, v) \
do { \
    NV_WRITE32(NV_SATA_APB_DFPCI_CFG + T_SATA0_##r, v); \
} while(0);

/*
 * Reads value of register r
 */
#define SATA0_REF_DFPCI_READ32(r, v) \
do { \
    v = NV_READ32(NV_SATA_APB_DFPCI_CFG + SATA0_##r); \
} while(0);


/*
 * Reads value of register r
 */
#define SATA0_REF_DFPCI_WRITE32(r, v) \
do { \
    NV_WRITE32(NV_SATA_APB_DFPCI_CFG + SATA0_##r, v); \
} while(0);


/*
 * Writes field f of register r of module m in IPFS register space.
 */
#define IPFS_REG_WRITE_FLD(m, r, f, v, rval) \
do { \
    rval = NV_READ32(NV_ADDRESS_MAP_APB_SATA_BASE + m##_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(m, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_APB_SATA_BASE + m##_##r##_0, rval); \
} while(0);

#define PMC_REG_WRITE_FLD(m, r, f, v, rval)        \
do { \
    rval = NV_READ32(NV_ADDRESS_MAP_APB_PMC_BASE + m##_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(m, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_APB_PMC_BASE + m##_##r##_0, rval); \
} while(0);

#define CLKRST_REG_WRITE_FLD(m, r, f, v, rval)        \
do { \
    rval = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + m##_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(m, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + m##_##r##_0, rval); \
} while(0);

#define XUSB_PADCTL_WRITE_FLD(m, r, f, v, rval)       \
do { \
    rval = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + m##_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(m, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + m##_##r##_0, rval); \
} while(0);

/*
 * id : Pll id as defined in clocks code
 *  t : Max stable time for Pll to lock
 *  s : status in BIT
 *  e : error (default set to Pll Not Locked)
 */
#define WAIT_FOR_PLLLOCK(id, t, s) \
do { \
    while(!NvBootClocksIsPllStable(id, t)); \
    if (!((NvBootUtilGetTimeUS() - t ) & (1U << 31)) \
      && (NvBootClocksIsPllStable(id, t)) ) \
    { \
        e = NvBootError_Success; \
    } \
    else \
    { \
       e = NvBootError_PllNotLocked; \
       SET_SATA_BOOT_INFO_BITFLD(InitStatus, s);  \
    } \
} while(0);

typedef enum
{
    ConstantTableIdx_SataCalibPadVal = 0,

    ConstantTableIdx_OscFreqIndex,

    ConstantTableIdx_PllDiv,

    ConstantTableIdx_UphyPllCfg,

    ConstantTableIdx_UphyMiscPadCfg,

    ContantTableIdx_SsaCoeffCfg,

    ContantTableIdx_SsdCoeffCfg,

    ConstantTableIdx_SataExtElectricalCntrls

} ConstantTableIdx;
/*
 * FUSE_BOOT_DEVICE_INFO[0:0] = PLLE clock source
 * FUSE_BOOT_DEVICE_INFO[1:1] = PLLE spread spectrum enable
 * FUSE_BOOT_DEVICE_INFO[3:2] = Number of retries for COMRESET
 * FUSE_BOOT_DEVICE_INFO[4:4] = Force GEN1
 * FUSE_BOOT_DEVICE_INFO[5:5] = Enable 32 bit reads for legacy pio mode
 * FUSE_BOOT_DEVICE_INFO[6:8] = Wait timeout for Cominit receival
 * FUSE_BOOT_DEVICE_INFO[9:10] = sector size multiplier (log2 value)
 */

/* Fuse params for SATA Boot device Configuration. */
typedef struct
{
    NvU32 UsePllpSrcForPlle;

    NvU32 NumComresetRetries;

    NvU32 ForceGen1;

    NvU32 Mode;

    NvU32 ComInitWaitIndex;

    NvU32 PageSizeMultLog2;

} SataFuseParam;


/* Mask values for boot device configuration fuse fields*/
typedef enum
{
    USE_PLLP_SOURCE_FUSE_MASK = 0x1,

    ENB_PLLE_SS_FUSE_MASK = 0x2,

    COMRESET_RETRIES_FUSE_MASK = 0xC,

    FORCE_GEN1_FUSE_MASK = 0x10,

    MODE_FUSE_MASK = 0x20,

    COMINIT_WAIT_IND_FUSE_MASK = 0x1C0,

    PAGE_SIZE_MULT_LOG2_FUSE_MASK = 0xE00

} SataFuseParamMask;

/* Offsets for boot device configuration fuse fields*/
typedef enum
{
    USE_PLLP_SOURCE_FUSE_SHIFT = 0x0,

    ENB_PLLE_SS_FUSE_SHIFT = 0x1,

    COMRESET_RETRIES_FUSE_SHIFT = 0x2,

    FORCE_GEN1_FUSE_SHIFT = 0x4,

    MODE_FUSE_SHIFT = 0x5,

    COMINIT_WAIT_IND_FUSE_SHIFT = 0x6,

    PAGE_SIZE_MULT_LOG2_FUSE_SHIFT = 0x9

} SataFuseParamShift;

/* Byte enable mask values */
typedef enum
{
    ENABLE_ALL_BYTES = 0x0,

    ENABLE_BYTE0 = 0xe,

    ENABLE_BYTE1 = 0xd,

    ENABLE_BYTE2 = 0xb,

    ENABLE_BYTE3 = 0x7,

    ENABLE_BYTE1_BYTE0 = 0xc

} LegacyPioBe;

/* Sata Controller Status */
/*
 * [0] : Sata was running a command list and was unable to accept further
  * commands then 1, else 0
 * [1] : Drq was high and sata driver timed out  then 1, else 0
 * [2] : Bsy was high and sata driver timed out  then 1, else 0
 * [3] : Port was busy then 1, else 0
 */
typedef enum
{

    SataStatus_CommandListRunning = 0x1,

    SataStatus_DrqHigh = 0x2,

    SataStatus_BsyHigh = 0x4,

    SataStatus_PortBsy = 0x8

} SataStatus;

/* Sata Init Status */
/*
 * [5:0] : initialization status (makes use of the fact that NvBootError has not
 * more than 48 enumerated values)
 * [6] : Sata Reinitialized then 1, else 0
 * [7] : Plle reinitialized then 1, else 0
 * [8] : PlleInitFailed then 1, else 0
 * [9] : ComResetFailed (cominit was not received within the expected time
 * limit)  then 1, else 0
 * [10] : SSD detection failed then 1, else 0
 * [12] : XUSB/SATA Pad pll locked then 1, else 0
 * [12] : Params Valid then 1, else 0
 */
 typedef enum
{
    SataInitStatus_SataReinitialized = 0x40,

    SataInitStatus_PlleReinitialized = 0x80,

    SataInitStatus_PlleInitFailed = 0x100,

    SataInitStatus_PlleSSFailed = 0x200,

    SataInitStatus_PllRefeInitFailed = 0x400,

    SataInitStatus_ComResetFailed = 0x800,

    SataInitStatus_SsdNotdetected = 0x1000,

    SataInitStatus_PadPllNotLocked = 0x2000,

    SataInitStatus_ParamsValid = 0x4000,

    SataInitStatus_UphyPllCalFailed = 0x8000,

    SataInitStatus_UphyPllCalNotCleared = 0x10000,

    SataInitStatus_UphyPllRCalFailed = 0x20000,

    SataInitStatus_UphyPllRCalNotCleared = 0x40000


} SataInitStatus;

/* Sata Ahci Status */
/*
 * [0] : Ahci Dma transfer could not be completed within a stipulated time then 1, else 0
 * [1] : SDB FIS was not received then 1, else 0
 * [2] : An error occurred during ahci dma transfer then 1, else 0
 * [3] : equivalent of AhciDataXmissionError in T30
 *       Specifies if there was an ahci data transmission error
 * [4] : equivalent of AhciCommandError in T30
 *       Specifies if there was a command error in case the last transaction
 *       was of ahci dma type.
 * [5] : equivalent of AhciTfdError in T30
 *       Specifies the Task File Data Error that occurred in case the last transaction
 *       was of ahci dma type
 */
 typedef enum
{
    SataAhciStatus_AhciDmaNotComplete = 0x1,

    SataAhciStatus_DmaCmdNotComplete = 0x2,

    SataAhciStatus_AhciError = 0x4,

    SataAhciStatus_AhciDataXmissionError = 0x8,

    SataAhciStatus_AhciCommandError = 0x10,

    SataAhciStatus_AhciTfdError = 0x20

} SataAhciStatus;

/* Sata Pad Cntrl Values */
typedef struct
{
    NvU8 Gen1TxAmp;

    NvU8 Gen1TxPeak;

    NvU8 Gen1RxCtle;

    NvU8 Gen2TxAmp;

    NvU8 Gen2TxPeak;

    NvU8 Gen2RxCtle;

} SataPadCntrlReg;

/* Sata Common Electrical Controls */
typedef struct
{
    NvU32 Gen1RxEqCtrlL;

    NvU32 Gen2RxEqCtrlL;

    NvU32 Gen1RxEqCtrlH;

    NvU32 Gen2RxEqCtrlH;

} SataExtElectricalControls;

/* Structure to hold M/N/P divisors 
 * for PLLREFE, PLLE and PLLC4 for different 
 * oscillator frequencies and for PLLP_OUT0
 * as input clock.
 */
typedef struct
{
    NvU8 PllrefeDivM;

    NvU8 PllrefeDivN;

    NvU8 PllrefeDivP;

    NvU8 PlleDivM;

    NvU8 PlleDivN;

    NvU8 PlleDivPlCml;

    NvU8 Pllc4DivM;

    NvU8 Pllc4DivN;

    NvU8 Pllc4DivP;

} SataSrcPllDivisors;


#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SATA_LOCAL_H */
