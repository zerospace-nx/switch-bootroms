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
 * nvboot_sata_local.h - unit local functions for BootROM SATA driver
 */

#ifndef INCLUDED_NVBOOT_SATA_LOCAL_H
#define INCLUDED_NVBOOT_SATA_LOCAL_H

#include "nvtypes.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Sector size is 512 Bytes per spec
#define SECTOR_SIZE 512
#define SECTOR_SIZE_LOG2 9

#define CEIL_SECTOR(len) (((len)+(SECTOR_SIZE)-1)/(SECTOR_SIZE))

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
#define NVBOOT_SATA_D2HFIS_TIMEOUT    10000000

/* id : Pll id as defined in clocks code
 *  t : Max stable time for Pll to lock
 *  s : status in BIT
 */
#define WAIT_FOR_PLLLOCK(id, t, s) \
do { \
    while(!NvBootClocksIsPllStable(id, t)); \
    if (!((NvBootUtilGetTimeUS() - t) & (1U << 31)) && (NvBootClocksIsPllStable(id, t))){ \
        e = NvBootError_Success; \
    } else { \
        e = NvBootError_PllNotLocked; \
        SET_SATA_BOOT_INFO_BITFLD(InitStatus, s); \
    } \
} while(0);

#define IPFS_RMW_FLD_NUM(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + SATA_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(SATA, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + SATA_##r##_0, rval); \
} while(0)

/* Simplify Common Accesses
 * Can access Generic Host Control or Port 0, everything BR needs 
 */
#define SATA_PRI_READ32(r) \
        NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR0_FPCI_BASE + SATA_PRI_##r##_0)

#define SATA_PRI_WRITE32(r, val) \
        NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR0_FPCI_BASE + SATA_PRI_##r##_0, val)

#define SATA_PRI_RMW_FLD(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR0_FPCI_BASE + SATA_PRI_##r##_0); \
    rval = NV_FLD_SET_DRF_DEF(SATA_PRI, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR0_FPCI_BASE + SATA_PRI_##r##_0, rval); \
} while(0)

#define SATA_PRI_RMW_FLD_NUM(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR0_FPCI_BASE + SATA_PRI_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(SATA_PRI, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR0_FPCI_BASE + SATA_PRI_##r##_0, rval); \
} while(0)

/* Simplify Common 
 * Can access Generic Host Control or Port 0, everything BR needs 
 */
#define SATA0_READ32(r) \
        NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_PCI_CONFIG_FPCI_BASE + SATA0_##r##_0)

#define SATA0_WRITE32(r, val) \
        NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_PCI_CONFIG_FPCI_BASE + SATA0_##r##_0, val)

#define SATA0_RMW_FLD(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_PCI_CONFIG_FPCI_BASE +  SATA0_##r##_0); \
    rval = NV_FLD_SET_DRF_DEF(SATA0, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_PCI_CONFIG_FPCI_BASE + SATA0_##r##_0, rval); \
} while(0)

#define SATA0_RMW_FLD_NUM(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_PCI_CONFIG_FPCI_BASE +  SATA0_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(SATA0, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_PCI_CONFIG_FPCI_BASE + SATA0_##r##_0, rval); \
} while(0)

/* Simplify Common AHCI (BAR5) accesses
 * Can access Generic Host Control or Port 0, everything BR needs 
 */
#define AHCI_READ32(r) \
        NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR5_FPCI_BASE + AHCI_##r##_0)

#define AHCI_WRITE32(r, val) \
        NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR5_FPCI_BASE + AHCI_##r##_0, val)

#define AHCI_RMW_FLD(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR5_FPCI_BASE +  AHCI_##r##_0); \
    rval = NV_FLD_SET_DRF_DEF(AHCI, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR5_FPCI_BASE + AHCI_##r##_0, rval); \
} while(0)

#define AHCI_RMW_FLD_NUM(r, f, v) \
do { \
    NvU32 rval = NV_READ32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR5_FPCI_BASE +  AHCI_##r##_0); \
    rval = NV_FLD_SET_DRF_NUM(AHCI, r, f, v, rval); \
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + NV_SATA_BAR5_FPCI_BASE + AHCI_##r##_0, rval); \
} while(0)

/*
 * FUSE_BOOT_DEVICE_INFO[0:0] = PLLE clock source
 * FUSE_BOOT_DEVICE_INFO[3:2] = Number of retries for COMRESET
 * FUSE_BOOT_DEVICE_INFO[4:4] = Allow Gen2
 * FUSE_BOOT_DEVICE_INFO[5:5] = 0 = PIO, 1 = AHCI DMA
 * FUSE_BOOT_DEVICE_INFO[6:8] = Wait timeout for Cominit receival
 * FUSE_BOOT_DEVICE_INFO[9:10] = sector size multiplier (log2 value)
 */
// Use NV_DRF macros for device configuration fuses
#define SATA_FUSE_PARAM_0_PLLE_USE_PLLREFE_RANGE  0:0
#define SATA_FUSE_PARAM_0_COMRESET_RETRIES_RANGE 3:2
#define SATA_FUSE_PARAM_0_ALLOW_GEN2_RANGE   4:4
#define SATA_FUSE_PARAM_0_MODE_RANGE 5:5
#define SATA_FUSE_PARAM_0_COMINIT_WAIT_INDEX_RANGE 8:6
#define SATA_FUSE_PARAM_0_PAGE_SIZE_MULT_LOG2_RANGE 11:9

/* Fuse params for SATA Boot device Configuration. */
typedef struct
{
    NvU32 PlleUseOscSrc;
    NvU32 ComInitWaitIndex;
    NvU32 ComresetRetries;
    NvU32 AllowGen2;
    NvU32 PageSizeMultLog2;
} SataFuseParam;

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

//TODO fix BIT bug
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

/* Structure to hold M/N/P divisors 
 * for PLLREFE, PLLE and PLLC4 for different 
 * oscillator frequencies and for PLLP_OUT0
 * as input clock.
 */
typedef struct
{
    uint8_t PllrefeDivM;
    uint8_t PllrefeDivN;
    uint8_t PllrefeDivP;
    uint8_t PlleDivM;
    uint8_t PlleDivN;
    uint8_t PlleDivPlCml;

} SataSrcPllDivisors;

/**
 * Defines the status from SATA
 * Overlay this structure with space reserved for this purpose in BIT
 * SecondaryDevStatus[NVBOOT_SIZE_DEV_STATUS]
 */
typedef struct NvBootSataStatusRec
{

    /// Specifies the sata and sata oob clock sources
    /// SataClockSource[15:0] : Sata clock source
    /// SataClockSource[31:16]: Sata Oob clock source
    NvU32 SataClockSource;

    /// Specifies the sata clock divider
    NvU32 SataClockDivider;

    /// Specifies the sata oob clock divider
    NvU32 SataOobClockDivider;

    /// Specifies the last used mode
    /// SataMode[15:0] : of type NvBootSataMode, could be AHCI or Legacy
    /// SataMode[31:16]: of type NvBootSataTransferMode, could be PIO or DMA
    NvU32 SataMode;

    /// Specifies the init status
    /// InitStatus[5:0] : initialization status (makes use of the fact that
    /// NvBootError has notmore than 48 enumerated values)
    /// InitStatus[6:6] : Sata Reinitialized then 1, else 0
    /// InitStatus[7:7] : Plle reinitialized then 1, else 0
    /// InitStatus[8:8] : PlleInitFailed then 1, else 0
    /// InitStatus[9:9] : ComResetFailed (cominit was not received within the
    /// expected timelimit)  then 1, else 0
    /// InitStatus[10:10] : SSD detection failed then 1, else 0
   ///  InitStatus[11:11] : Params validated
    NvU32 InitStatus;

    /// Specifies the number of pages read from the beginning.
    NvU32 NumPagesRead;

    /// Specifies the last page read
    NvU32 LastBlockRead;

    /// Specifies the last page read
    NvU32 LastPageRead;

    /// Specifies the Port Error that occurred.
    /// PortStatus[0:0] : SataStatus_CommandListRunning
    /// PortStatus[1:1] : SataStatus_DrqHigh
    /// PortStatus[2:2] : SataStatus_BsyHigh
    /// PortStatus[3:3] : SataStatus_PortBsy
    NvU32 PortStatus;

    /// Specifies the sata buffers base
    NvU32 AhciSataBuffersBase;

    /// Specifies the data buffers base for ahci dma
    NvU32 AhciDataBuffersBase;

    /// Specifies if Ahci Dma Status
    /// AhciDmaStatus[0:0] : Ahci Dma transfer could not be completed within a
    /// stipulated time then 1, else 0
    /// AhciDmaStatus[1:1] : SDB FIS was not received then 1, else 0
    /// AhciDmaStatus[2:2] : An error occurred during ahci dma transfer then 1,
    /// else 0. If this bit is set, look at other fields of Bit sata status to
    /// ascertain the type of Ahci Dma Error that occurred.
    /// AhciDmaStatus[3:3] : equivalent of AhciDataXmissionError in T30
    /// Specifies if there was an ahci data transmission error
    /// AhciDmaStatus[4:4] : equivalent of AhciCommandError in T30
    /// Specifies if there was a command error in case the last transaction
    /// was of ahci dma type.
    /// AhciDmaStatus[5:5] : equivalent of AhciTfdError in T30
    /// Specifies the Task File Data Error that occurred in case the last transaction
    /// was of ahci dma type
    NvU32 AhciDmaStatus;

} NvBootSataStatus;

/*
 * f = field name of Sata status in BIT
 * v = value to be assigned to field f
 */
#define SET_SATA_BOOT_INFO_FLD(f,v) \
do { \
    ((NvBootSataStatus*)&BootInfoTable.SecondaryDevStatus[0])->f = v; \
} while(0)

/*
 * f = field name of Sata status in BIT
 * v = mask value for the bit field in f
 */
#define SET_SATA_BOOT_INFO_BITFLD(f,v) \
do { \
    ((NvBootSataStatus*)&BootInfoTable.SecondaryDevStatus[0])->f |= v; \
} while(0)

#define GET_SATA_BOOT_INFO_FLD(f) \
    ((NvBootSataStatus*)&BootInfoTable.SecondaryDevStatus[0])->f

void ConfigureClockSource(const NvBootClocksClockId Id, const NvBootSataParams *Params);

void ProgramPadCntrlRegisters(void);

void ProgramSquelch();

void EnableSpaces();

void EnableBARs();

void UpdateClasscode();

void AhciEnable();

NvU32 PortImplemented();

NvU32 GetNumCmdSlots();

NvBootError CheckPortCommandErr();

void ProgramCommandAndFISReceive();

void ClearPortErrors();

NvBootError IssueComReset(uint8_t ComInitWaitIndex, uint8_t ComresetRetires);

NvBootError AllocateHBAMemBuffers(const NvBootSataParams *Params);

NvBootError NvBootSataLegacyPioRead(const NvU32 Block, const NvU32 Page, const NvU32 Length, uint8_t *Dest);

NvBootError NvBootSataLegacyPioWrite(const NvU32 Block, const NvU32 Page, uint8_t *Src);

NvBootError NvBootSataAhciDmaRead(const NvU32 Block, const NvU32 Page, const NvU32 Length, uint8_t *Dest);

NvBootError EnablePLLE(const NvBootSataParams *Params);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SATA_LOCAL_H */
