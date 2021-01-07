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
 * nvboot_sdmmc_context.h - Definitions for the Sdmmc context structure.
 */

#ifndef INCLUDED_NVBOOT_SDMMC_CONTEXT_H
#define INCLUDED_NVBOOT_SDMMC_CONTEXT_H

#include "nvboot_sdmmc_param.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/// Sdmmc Response buffer size.
#define NVBOOT_SDMMC_RESPONSE_BUFFER_SIZE_IN_BYTES 16

/// Defines Emmc/Esd card states.
typedef enum
{
    SdmmcState_Idle = 0,
    SdmmcState_Ready,
    SdmmcState_Ident,
    SdmmcState_Stby,
    SdmmcState_Tran,
    SdmmcState_Data,
    SdmmcState_Rcv,
    SdmmcState_Prg,
    SdmmcState_Force32 = 0x7FFFFFFF
} SdmmcState;

/// Defines Emmc card partitions.
typedef enum
{
    SdmmcAccessRegion_UserArea = 0,
    SdmmcAccessRegion_BootPartition1,
    SdmmcAccessRegion_BootPartition2,
    SdmmcAccessRegion_Num,
    SdmmcAccessRegion_Unknown,
    SdmmcAccessRegion_Force32 = 0x7FFFFFFF
} SdmmcAccessRegion;

/// Defines various card types supported.
typedef enum
{
    NvBootSdmmcCardType_Emmc = 0,
    NvBootSdmmcCardType_Esd,
    NvBootSdmmcCardType_Num,
    NvBootSdmmcCardType_Force32 = 0x7FFFFFFF
} NvBootSdmmcCardType;

/// Defines various clock rates to use for accessing the Emmc/Esd card at.
typedef enum
{
    NvBootSdmmcCardClock_Identification = 0,
    NvBootSdmmcCardClock_DataTransfer,
    NvBootSdmmcCardClock_20MHz,
    NvBootSdmmcCardClock_Num,
    NvBootSdmmcCardClock_Force32 = 0x7FFFFFFF
} NvBootSdmmcCardClock;

/**
 * The context structure for the Sdmmc driver.
 * A pointer to this structure is passed into the driver's Init() routine.
 * This pointer is the only data that can be kept in a global variable for
 * later reference.
 */
typedef struct NvBootSdmmcContextRec
{
    /// Device config 
    uint8_t ConfigOption;
    /// Block size.
    uint8_t BlockSizeLog2;
    /// Page size.
    uint8_t PageSizeLog2;
    /// No of pages per block;
    uint8_t PagesPerBlockLog2;
    /**
     * Clock divisor for the Sdmmc Controller Clock Source. 
     * Sdmmc Controller gets PLL_P clock, which is 432MHz, as a clock source.
     * If it is set to 18, then clock Frequency at which Emmc controller runs 
     * is 432/18 = 24MHz.
     */
    uint8_t ClockDivisor;
    /// Card's Relative Card Address.
    uint32_t CardRca;
    /// Data bus width.
    NvBootSdmmcDataWidth DataWidth;
    /// Response buffer. Define it as NvU32 to make it word aligned.
    uint32_t SdmmcResponse[NVBOOT_SDMMC_RESPONSE_BUFFER_SIZE_IN_BYTES / sizeof(uint32_t)];
    /// Device status.
    NvBootDeviceStatus DeviceStatus;
    /// Holds the Movi Nand Read start time.
    uint32_t ReadStartTime;
    /**
     * This sub divides the clock that goes to card from controller.
     * If the controller clock is 24 Mhz and if this set to 2, then
     * the clock that goes to card is 12MHz.
     */
    uint8_t CardClockDivisor;
    /// Indicates whether to access the card in High speed mode.
    NvBool HighSpeedMode;
    /// Indicates whether the card is high capacity card or not.
    NvBool IsHighCapacityCard;
    /// Spec version.
    uint8_t SpecVersion;
    /// Holds Emmc Boot Partition size.
    uint32_t EmmcBootPartitionSize;
    /// Holds the current access region.
    SdmmcAccessRegion CurrentAccessRegion;
    /// Holds the current clock rate.
    NvBootSdmmcCardClock CurrentClockRate;
    /// Buffer for selecting High Speed, reading extended CSD and SCR.
    uint8_t *SdmmcInternalBuffer;
    /// Read access time1.
    uint8_t taac;
    /// Read access time2.
    uint8_t nsac;
    /// The clock frequency when not in high speed mode.
    uint8_t TranSpeed;
    /// Transfer speed in MHz.
    uint8_t TranSpeedInMHz;
    /// Indicates whether host supports high speed mode.
    NvBool HostSupportsHighSpeedMode;
    /// Indicates whether card supports high speed mode.
    NvBool CardSupportsHighSpeedMode;
    /// Indicates the page size to use for card capacity calculation.
    uint8_t PageSizeLog2ForCapacity;
    /// Power class for 26MHz at 3.6V.
    uint8_t PowerClass26MHz360V;
    /// Power class for 52MHz at 3.6V.
    uint8_t PowerClass52MHz360V;
    /// Power class for 26MHz at 1.95V.
    uint8_t PowerClass26MHz195V;
    /// Power class for 52MHz at 1.95V.
    uint8_t PowerClass52MHz195V;
    /// Boot Config from ExtCSD.
    uint8_t BootConfig;
    /// Indicates whether high voltage range is used for Card identification.
    NvBool IsHighVoltageRange;
    /// Max Power class supported by target board.
    uint8_t MaxPowerClassSupported;
    /// Number of blocks present in card.
    uint32_t NumOfBlocks;
    /// Holds read time out at current card clock frequency.
    uint32_t ReadTimeOutInUs;
    /// Flag to indicate whether reading is boot mode.
    NvBool BootModeReadInProgress;
    /// Flag to indicate the card speed and operating voltage level
    uint8_t CardSupportSpeed;
    /// Flag identicates card's bus width.
    uint8_t CardBusWidth;
    /// Spec 3 version or higher
    uint8_t Spec3Version;
    /// Power class for 52 Mhz DDR @ 3.6V.
    uint8_t PowerClass52MHzDdr360V;//PWR_CL_DDR_52_360
    /// Power class for 52 Mhz DDR @ 1.95V.
    uint8_t PowerClass52MHzDdr195V;//PWR_CL_DDR_52_195
    /// Indicates whether Ddr mode is used for data transfer
    uint8_t IsDdrMode;
    /// Store the current address of the external memory buffer being used. 
    uint8_t *CurrentReadBufferAddress;
    /// MultiPage support
    uint8_t MultiPageSupported;
} NvBootSdmmcContext;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SDMMC_CONTEXT_H */

