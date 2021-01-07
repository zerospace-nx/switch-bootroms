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
 * nvboot_sdmmc_local.h - Internal definitions for using Sdmmc as a
 * second level boot device.
 */

#ifndef INCLUDED_NVBOOT_SDMMC_LOCAL_H
#define INCLUDED_NVBOOT_SDMMC_LOCAL_H
#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_sdmmc_param.h"
#include "nvboot_device_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/// Defines for vendor clock control register
#define TAP_LOW_BIT 0
#define TAP_HIGH_BIT (TAP_LOW_BIT + \
        NV_FIELD_SIZE(SDMMC_VENDOR_CLOCK_CNTRL_0_TAP_VAL_RANGE)-1)
#define TRIM_LOW_BIT (TAP_HIGH_BIT+1)
#define TRIM_HIGH_BIT (TRIM_LOW_BIT + \
        NV_FIELD_SIZE(SDMMC_VENDOR_CLOCK_CNTRL_0_TRIM_VAL_RANGE)-1)
#define SDMMCBOOT_VENDOR_CLOCK_0_TAP_VAL_RANGE TAP_HIGH_BIT:TAP_LOW_BIT
#define SDMMCBOOT_VENDOR_CLOCK_0_TRIM_VAL_RANGE TRIM_HIGH_BIT:TRIM_LOW_BIT
typedef struct
{
    uint16_t sdrVendorClkCtrl;
    uint16_t ddrVendorClkCtrl;
} NvBootSdmmcVendorClkCtrl;


/// These defines are for Emmc operations time out.
#define SDMMC_COMMAND_TIMEOUT_IN_US 100000
#define SDMMC_READ_TIMEOUT_IN_US 200000
#define SDMMC_TIME_OUT_IN_US 100000

#if NVBOOT_TARGET_FPGA
#define SDMMC_OP_COND_TIMEOUT_IN_US 100000000
#define SDMMC_PLL_FREQ_IN_MHZ 13
#else
#define SDMMC_OP_COND_TIMEOUT_IN_US 1000000
#define SDMMC_PLL_FREQ_IN_MHZ 408 // from 216
#endif

/// The following configs are defined for sdmmc io driver/controller
/// Datawidth/clock divisor/mode (SDR/DDR) and multi/single page options
/// will be retrieved from these confgiration only.
/// Defines Configurations .
///   = 0 means, SDR_25.5Mhz_ReadSinglepage
///   = 1 means, SDR_51Mhz_ReadMultipage
///   = 2 means, SDR_25.5Mhz_ReadMultipage
///   = 3 means, DDR_51Mhz_ReadMultipage
///   = 4 means, DDR_25.5Mhz_ReadMultipage
///   = 5/6/7 means reserved.

typedef enum
{
    Sdmmc_Config_0 = 0, // SDR_25.5Mhz_ReadSinglepage
    Sdmmc_Config_1 = 1, // SDR_51Mhz_ReadMultipage
    Sdmmc_Config_2 = 2, // SDR_25.5Mhz_ReadMultipage
    Sdmmc_Config_3 = 3, // DDR_51Mhz_ReadMultipage
    Sdmmc_Config_4 = 4, // DDR_25.5Mhz_ReadMultipage
    Sdmmc_Config_Num,
} Sdmmc_Config;

typedef enum
{
    Sdmmc_ReadMode_SinglePage = 0, // Read Single page mode
    Sdmmc_ReadMode_MultiPage = 1, // Read Multiple page mode
    Sdmmc_ReadMode_Num,
} Sdmmc_ReadMode;


#define SDMMC_IO_CLOCK_DIVISOR_BY_HALF 	2
#define SDMMC_IO_CLOCK_DIVISOR_NONE		1


#define SDMMC_CNTL_CLOCK_DIVISOR_51MHZ 	8
#define SDMMC_CNTL_CLOCK_DIVISOR_102MHZ	4


/// The following divider limits are based on the requirement that
/// SD Host controler clock should be in between 10MHz to 63MHz.
/// Max valid clock divider for PLLP clock source(408MHz).
#define SDMMC_MAX_CLOCK_DIVIDER_SUPPORTED 41
/// Min valid clock divider for PLLP clock source(408MHz).
#define SDMMC_MIN_CLOCK_DIVIDER_SUPPORTED 1
/// Max Power class supported.by card.
#define SDMMC_MAX_POWER_CLASS_SUPPORTED 16
/// This needs to be more than the BCT struct info.
#define NVBOOT_SDMMC_BLOCK_SIZE_LOG2 14
/// Maximum multiple block size supported
#define SDMMC_MAX_BLOCK_SIZE_LOG2 5

/// These defines are as per Emmc Spec.
#define EMMC_SWITCH_BUS_WIDTH_ARG 0x03b70000
#define EMMC_SWITCH_BUS_WIDTH_OFFSET 8
#define EMMC_SWITCH_1BIT_BUS_WIDTH_ARG 0x03b70000
#define EMMC_SWITCH_4BIT_BUS_WIDTH_ARG 0x03b70100
#define EMMC_SWITCH_8BIT_BUS_WIDTH_ARG 0x03b70200

#define EMMC_SWITCH_4BIT_DDR_BUS_WIDTH_ARG 0x03b70500
#define EMMC_SWITCH_8BIT_DDR_BUS_WIDTH_ARG 0x03b70600

#define EMMC_SWITCH_CARD_TYPE_ARG 0x03C20000
#define EMMC_SWITCH_INDEX_OFFSET 8

#define EMMC_SWITCH_FUNCTION_TYPE_ARG 0x80000000
#define EMMC_SWITCH_CURRENT_OFFSET 12
#define EMMC_SWITCH_DRIVESTRENGHT_OFFSET 8
#define EMMC_SWITCH_ACCESSMODE_OFFSET 0


#define EMMC_SWITCH_HIGH_SPEED_ENABLE_ARG  0x03b90100
#define EMMC_SWITCH_HIGH_SPEED_DISABLE_ARG 0x03b90000

#define EMMC_SWITCH_SELECT_PARTITION_ARG   0x03b30000
#define EMMC_SWITCH_SELECT_PARTITION_MASK  0x7
#define EMMC_SWITCH_SELECT_PARTITION_OFFSET 0x8

#define EMMC_SWITCH_SELECT_POWER_CLASS_ARG 0x03bb0000
#define EMMC_SWITCH_SELECT_POWER_CLASS_OFFSET 8

/// Emmc Extended CSD fields.
#define EMMC_ECSD_SECTOR_COUNT_0_OFFSET  212
#define EMMC_ECSD_SECTOR_COUNT_1_OFFSET  213
#define EMMC_ECSD_SECTOR_COUNT_2_OFFSET  214
#define EMMC_ECSD_SECTOR_COUNT_3_OFFSET  215
#define EMMC_ECSD_POWER_CL_26_360_OFFSET 203
#define EMMC_ECSD_POWER_CL_52_360_OFFSET 202
#define EMMC_ECSD_POWER_CL_26_195_OFFSET 201
#define EMMC_ECSD_POWER_CL_52_195_OFFSET 200
#define EMMC_ECSD_CARD_TYPE_OFFSET       196
#define EMMC_ECSD_POWER_CLASS_OFFSET     187
#define EMMC_ECSD_HS_TIMING_OFFSET       185
#define EMMC_ECSD_BUS_WIDTH              183
#define EMMC_ECSD_BOOT_CONFIG_OFFSET     179
#define EMMC_ECSD_BOOT_PARTITION_SIZE_OFFSET 226
#define EMMC_ECSD_POWER_CLASS_MASK 0xF
#define EMMC_ECSD_POWER_CLASS_4_BIT_OFFSET 0
#define EMMC_ECSD_POWER_CLASS_8_BIT_OFFSET 4


#define EMMC_ECSD_BOOT_BUS_WIDTH    177

// Emmc Extended CSD fields v4.4 addition and updates
#define EMMC_ECSD_POWER_CL_DDR_52_360_OFFSET 239
#define EMMC_ECSD_POWER_CL_DDR_52_195_OFFSET 238
#define EMMC_ECSD_MIN_PERF_DDR_W_8_52_OFFSET 235
#define EMMC_ECSD_MIN_PERF_DDR_R_8_52_OFFSET 234

#define EMMC_ECSD_BOOT_INFO_OFFSET       228
#define EMMC_ECSD_CARD_CSD_STRUCT_OFFSET   194
#define EMMC_ECSD_CARD_EXT_CSD_REV_OFFSET   192

// EXT CSD_STRUCTURE REV versions
#define EMMC_ECSD_CARD_EXT_CSD_REV_15   5
#define EMMC_ECSD_CARD_EXT_CSD_REV_14   4
#define EMMC_ECSD_CARD_EXT_CSD_REV_13   3
#define EMMC_ECSD_CARD_EXT_CSD_REV_12   2
#define EMMC_ECSD_CARD_EXT_CSD_REV_11   1
#define EMMC_ECSD_CARD_EXT_CSD_REV_10   0

// High-Speed Dual Data Rate MultimediaCard @ 52MHz - 1.2V I/O
#define EMMC_ECSD_CT_HS_DDR_52_120 8 // card type high speed DDR 52 mhz @ 1.2v
#define EMMC_ECSD_CT_HS_DDR_52_180_300 4 // card type high speed DDR 52 @ either 1.8 or 3.0v
#define EMMC_ECSD_CT_HS_DDR_52_120_MASK             0x8
#define EMMC_ECSD_CT_HS_DDR_52_180_300_MASK     0x4
#define EMMC_ECSD_CT_HS_DDR_RANGE                       3:2
#define EMMC_ECSD_CT_HS_DDR_OFFSET                      2
#define EMMC_ECSD_CT_HS_DDR_MASK                        0xC
#define EMMC_ECSD_CT_HS_52 2
#define EMMC_ECSD_CT_HS_26 1

// BOOT_PARTITION_ENABLE
#define EMMC_ECSD_BC_BPE_RANGE              5:3
#define EMMC_ECSD_BC_BPE_OFFSET             3
#define EMMC_ECSD_BC_BPE_MASK               0x7
#define EMMC_ECSD_BC_BPE_NOTENABLED   0
#define EMMC_ECSD_BC_BPE_BAP1               1
#define EMMC_ECSD_BC_BPE_BAP2               2
#define EMMC_ECSD_BC_BPE_UAE                7

#define EMMC_CSD_UHS50_TRAN_SPEED 0x0B
#define EMMC_CSD_UHS104_TRAN_SPEED 0x2B

/// Emmc CSD fields
#define EMMC_CSD_0_CSD_STRUC_RANGE 23:22 // 119:1118 of response register.
#define EMMC_CSD_0_SPEC_VERS_RANGE 21:18 // 117:114 of response register.
#define EMMC_CSD_0_TAAC_RANGE 15:8 // 111:104 of response register.
#define EMMC_CSD_TAAC_TIME_UNIT_MASK 0x07
#define EMMC_CSD_TAAC_TIME_VALUE_OFFSET 3
#define EMMC_CSD_TAAC_TIME_VALUE_MASK 0x0F
#define EMMC_CSD_0_NSAC_RANGE 7:0 // 103:96 of response register.
#define EMMC_CSD_0_TRAN_SPEED_RANGE 31:24 // 95:88 of response register.
#define EMMC_CSD_0_CCC_RANGE 23:12 // 87:76 of response register.
#define EMMC_CSD_0_READ_BL_LEN_RANGE 11:8 // 75:72 of response register.
#define EMMC_CSD_V4_3_TRAN_SPEED 0x32
#define EMMC_CSD_0_C_SIZE_0_RANGE 31:22 // 63:54 of response register.
#define EMMC_CSD_0_C_SIZE_1_RANGE 1:0 // 65:64 of response register.
#define EMMC_CSD_C_SIZE_1_LEFT_SHIFT_OFFSET 10
#define EMMC_CSD_0_C_SIZE_MULTI_RANGE 9:7 // 41:39 of response register.
#define EMMC_CSD_MAX_C_SIZE 0xFFF
#define EMMC_CSD_MAX_C_SIZE_MULTI 0x7

/// Esd CSD Version 2 fields
#define EMMC_CSD_0_C_SIZE_V2_0_RANGE 29:8 // 61:40 of response register.
/// Card status fields.
#define SDMMC_CS_0_ADDRESS_OUT_OF_RANGE_RANGE 31:31
#define SDMMC_CS_0_ADDRESS_MISALIGN_RANGE 30:30
#define SDMMC_CS_0_BLOCK_LEN_ERROR_RANGE 29:29
#define SDMMC_CS_0_COM_CRC_ERROR_RANGE 23:23
#define SDMMC_CS_0_ILLEGAL_CMD_RANGE 22:22
/// Card internal ECC was applied but failed to correct the data.
#define SDMMC_CS_0_CARD_ECC_FAILED_RANGE 21:21
/// A card error occurred, which is not related to the host command.
#define SDMMC_CS_0_CC_ERROR_RANGE 20:20
#define SDMMC_CS_0_CURRENT_STATE_RANGE 12:9
#define SDMMC_CS_0_SWITCH_ERROR_RANGE 7:7

/// Defines as per ESD spec.
#define ESD_HOST_HIGH_VOLTAGE_RANGE 0x100
#define ESD_HOST_DUAL_VOLTAGE_RANGE 0x300
#define ESD_HOST_LOW_VOLTAGE_RANGE 0x200
#define ESD_HOST_CHECK_PATTERN 0xA5
#define ESD_CARD_OCR_VALUE 0x00FF8000
#define ESD_CMD8_RESPONSE_VHS_MASK 0xF00
#define ESD_CMD8_RESPONSE_CHECK_PATTERN_MASK 0xFF
#define ESD_ACMD41_HIGH_CAPACITY_BIT_OFFSET 30
#define ESD_DATA_WIDTH_1BIT 0
#define ESD_DATA_WIDTH_4BIT 2
#define ESD_HIGHSPEED_SET 0x80FFFF01

#define ESD_SCR_SD_SPEC_BYTE_OFFSET 0
#define ESD_SCR_0_SD_SPEC_RANGE 3:0

#define ESD_BOOT_PARTITION_ID (1 << 24)
#define ESD_SCR_DATA_LENGTH 8

/// Defines common to ESD and EMMC.
#define SDMMC_RCA_OFFSET 16
#define SDMMC_MAX_PAGE_SIZE_LOG_2 9
#define SDMMC_OCR_READY_MASK 0x80000000
#define SDMMC_CARD_CAPACITY_MASK 0x40000000

#define SDMMC_OCR_RESPONSE_WORD 0
#define SDMMC_MAX_CLOCK_FREQUENCY_IN_MHZ 52

/// Defines Command Responses of Emmc/Esd.
typedef enum
{
    SdmmcResponseType_NoResponse = 0,
    SdmmcResponseType_R1 = 1,
    SdmmcResponseType_R2 =2,
    SdmmcResponseType_R3 =3,
    SdmmcResponseType_R4 = 4,
    SdmmcResponseType_R5 = 5,
    SdmmcResponseType_R6 = 6,
    SdmmcResponseType_R7 = 7,
    SdmmcResponseType_R1B = 8,
    SdmmcResponseType_Num,
    SdmmcResponseType_Force32 = 0x7FFFFFFF
} SdmmcResponseType;

/**
 * Defines Emmc/Esd Commands as per Emmc/Esd spec's.
 * Emmc specific Commands starts with prefix Emmc and .Esd specific Commands 
 * starts with prefix Esd. Common commands has no prefix.
 */
typedef enum
{
    SdmmcCommand_GoIdleState = 0,
    SdmmcCommand_EmmcSendOperatingConditions = 1,
    SdmmcCommand_AllSendCid = 2,
    SdmmcCommand_EmmcSetRelativeAddress = 3,
    SdmmcCommand_EsdSendRelativeAddress = 3,
    SdmmcCommand_Switch = 6,
    SdmmcCommand_SelectDeselectCard = 7,
    SdmmcCommand_EsdSendInterfaceCondition = 8,
    SdmmcCommand_EmmcSendExtendedCsd = 8,
    SdmmcCommand_SendCsd = 9,
    SdmmcCommand_StopTransmission =12,//0xc
    SdmmcCommand_SendStatus = 13,//0xd
    SdmmcCommand_SetBlockLength = 16,//0x10
    SdmmcCommand_ReadSingle = 17,//0x11
    SdmmcCommand_ReadMulti = 18,//0x12
    SdmmcCommand_SendTuningPattern = 19,//0x13
    SdmmcCommand_SetBlockCount = 23,//0x17
    SdmmcCommand_WriteSingle = 24,//0x18
    SdmmcCommand_WriteMulti = 25,//0x19
    SdmmcCommand_EsdAppSendOperatingCondition = 41,//0x29
    SdmmcCommand_EsdSelectPartition = 43,//0x2b
    SdmmcCommand_EsdAppSendScr = 51,//0x33
    SdmmcCommand_EsdAppCommand = 55,//0x37
    SdmmcCommand_Force32 = 0x7FFFFFFF
} SdmmcCommand;

/// Defines operating voltages for voltage range validation during card
/// identification. The values of the enumerant map to device configuration
/// fuse values.
typedef enum
{
    NvBootSdmmcVoltageRange_QueryVoltage = 0,
    NvBootSdmmcVoltageRange_HighVoltage,
    NvBootSdmmcVoltageRange_DualVoltage,
    NvBootSdmmcVoltageRange_LowVoltage,
    NvBootSdmmcVoltageRange_Num,
    NvBootSdmmcVoltageRange_Force32 = 0x7FFFFFFF,
} NvBootSdmmcVoltageRange;

/// Defines Emmc Ocr register values to use for operating voltage
/// range validation during card identification.  Note that the
/// code will OR in the bit which indicates support for sector-based
/// addressing.
typedef enum
{
    /// Query the voltage supported.
    EmmcOcrVoltageRange_QueryVoltage = 0x00000000,
    /// High voltage only.
    EmmcOcrVoltageRange_HighVoltage = 0x00ff8000,
    ///  Both voltages.
    EmmcOcrVoltageRange_DualVoltage = 0x00ff8080,
    ///  Low voltage only.
    EmmcOcrVoltageRange_LowVoltage  = 0x00000080,
} EmmcOcrVoltageRange;

// This struct holds the Info from fuses.
typedef struct
{
    // Sdmmc4 config
    uint8_t SdmmcConfig;
    // Ddr mode selection
    NvU8 DdrMode;
    // sdmmc4 Multi Page support
    NvU8 SdmmcMultiPageSupport;
} NvBootSdmmcFuseInfo;

/**
*
*
*/

NvBootError HwSdmmcRecoverControllerFromErrors(NvBool IsDataCmd);

void HwSdmmcAbortDataRead(void);

NvBootError HwSdmmcInitController(void);

void HwSdmmcSetNumOfBlocks(uint32_t BlockLength, uint32_t NumOfBlocks);

void HwSdmmcSetupDma(uint8_t *pBuffer);

NvBootError HwSdmmcWaitForDataLineReady(void);

void HwSdmmcCalculateCardClockDivisor(void);

NvBootError SdmmcSelectAccessRegion(uint32_t* Block);

NvBootError EmmcReadDataInBootMode(uint8_t* pBuffer, uint32_t NumOfBlocks);

NvBootError EmmcIdentifyCard(void);

NvBootError
HwSdmmcSendCommand(
    SdmmcCommand CommandIndex,
    uint32_t CommandArg,
    SdmmcResponseType ResponseType,
    NvBool IsDataCmd);

NvBootError
EmmcVerifyResponse(
    SdmmcCommand command,
    NvBool AfterCmdExecution);

NvBootDeviceStatus HwSdmmcQueryStatus(void);

void HwSdmmcShutdown(void);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_SDMMC_LOCAL_H */
