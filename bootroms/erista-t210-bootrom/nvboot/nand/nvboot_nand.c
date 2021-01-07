/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_nand_local.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arclk_rst.h"
#include "arnandflash.h"
#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_codecov_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_nand_context.h"
#include "nvboot_nand_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_buffers_int.h"

#define DEBUG_NAND 0

#define JEDEC_ID_ENABLE 0

#if DEBUG_NAND
#include "nvos.h"
#define PRINT_NAND_REG_ACCESS(...)  NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_NAND_MESSAGES(...)    NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_NAND_ERRORS(...)      NvOsDebugPrintf(__VA_ARGS__);
#else
#define PRINT_NAND_REG_ACCESS(...)
#define PRINT_NAND_MESSAGES(...)
#define PRINT_NAND_ERRORS(...)
#endif

// Compile-time assertions
NV_CT_ASSERT( (NvBootNandEccSelection_Discovery == 0) &&
              (NvBootNandEccSelection_Bch4 == 1) &&
              (NvBootNandEccSelection_Bch8 == 2) && 
              (NvBootNandEccSelection_Bch16 == 3) &&
              (NvBootNandEccSelection_Bch24 == 4) &&
              (NvBootNandEccSelection_Off == 5) );

#define NV_NAND_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_ADDRESS_MAP_APB_NANDCTRL_BASE + NAND_##reg##_0)); \
        PRINT_NAND_REG_ACCESS("\r\nRead %s Offset 0x%x = 0x%8.8x", #reg, \
            NAND_##reg##_0, value); \
    } while (0)

#define NV_NAND_WRITE(reg, value) \
    do \
    { \
        NV_WRITE32((NV_ADDRESS_MAP_APB_NANDCTRL_BASE + NAND_##reg##_0), value); \
        PRINT_NAND_REG_ACCESS("\r\nWrite %s Offset 0x%x = 0x%8.8x", #reg, \
            NAND_##reg##_0, value); \
    } while (0)

/*
 * This contians default params that will be used pre BCT.Some of the 
 * params of this struct, which are specific to Nand flash present on board,
 * will be changed based on the fuse settings passed to the  API 
 * NvBootNandGetParams(). See NvBootNandGetParams() for more info on what 
 * gets changed based on fuse values.
 */
static NvBootNandParams s_DefaultNandParams;
// Pointer to store the context memory address.
static NvBootNandContext *s_NandContext = NULL;
// Boot Info table.
extern NvBootInfoTable BootInfoTable;
// Pointer to Nand Bit info.
static NvBootNandStatus* s_NandBitInfo = 
                                &BootInfoTable.SecondaryDevStatus.NandStatus;

// This struct holds the Info from fuses.
typedef struct
{
    // Selected Data width.
    NvBootNandDataWidth DataWidth;
    // Onfi support status.
    NvBool DisableOnfiSupport;
    // ECC Selection.
    NvBootNandEccSelection EccSelection;
    // PageBlock size correction offset.
    NvU8 PageBlockSizeOffset;
    // Toggle DDR mode select
    NvBootNandToggleDDR ToggleDDR;
    // Main ECC Offset
    NvU8 MainEccOffset;
    // Bch sector size
    NvU8 BchSectorSize;
    // Error Free Nand Version select
    NvBootEFNandCtrlVer EFNandCtrlVer;
} NvBootNandFuseInfo;

static NvBootNandFuseInfo s_FuseInfo = 
{NvBootNandDataWidth_Discovey, 0, NvBootNandEccSelection_Discovery, 0,
    NvBootNandToggleDDR_Discovey, 0, 0, NvBootEFNand_Disable};

// To enable testing of Ecc discovery functionality, This is defined here 
// instead of internal to the function.
static NvBool s_PerformEccDiscovery = NV_TRUE;

/* Private functions forward declaration.*/
/* Private Function Definitions */

static void HwNandSetPinmux(NvBootNandDataWidth DataWidth)
{
    NvU32 PinmuxSelection;
    NV_ASSERT(DataWidth < NvBootNandDataWidth_Num);

    if (DataWidth == NvBootNandDataWidth_16Bit)
    {
        PRINT_NAND_MESSAGES("\r\nSetting Pinmux for 16-bit");
        PinmuxSelection = NvBootPinmuxConfig_Nand_Std_x16;
    }
    else
    {
        // Data width discovery case and 8-bit case will fall here.
        PRINT_NAND_MESSAGES("\r\nSetting Pinmux for 8-bit");
        PinmuxSelection = NvBootPinmuxConfig_Nand_Std_x8;
    }
    (void)NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_NandFlash, 
        PinmuxSelection);
}

static void
HwNandInitializeHwController(
    const NvBootNandParams *Params)
{
    NvU32 ConfigReg2;
    NvU32 MainEccOffsetReg;
    NvU32 FbioCfgReg;

    // Set pinmuxing.
    HwNandSetPinmux(s_NandContext->DataWidth);
    // Reset the controller.
    NvBootResetSetEnable(NvBootResetDeviceId_NandId, NV_TRUE);
    // Configure the clock source for nand clock
    NvBootClocksConfigureClock(NvBootClocksClockId_NandId, 
     NVBOOT_CLOCKS_7_1_DIVIDER_BY(Params->ClockDivider, 0), 
     CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_PLLP_OUT0);
    // Enable the nand clock.
    NvBootClocksSetEnable(NvBootClocksClockId_NandId, NV_TRUE);
    // Enable the nand speed colock only when device is not EF Nand and ECC is enabled
    if ((s_FuseInfo.EccSelection != NvBootNandEccSelection_Off) &&
        (!s_NandContext->EFNandCtrlVer))
    {
        // Configure the clock source for nand speed clock
        NvBootClocksConfigureClock(NvBootClocksClockId_NandSpeedId, 
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(Params->ClockDivider, 0), 
        CLK_RST_CONTROLLER_CLK_SOURCE_NAND_SPEED_0_NAND_SPEED_CLK_SRC_PLLP_OUT0);
        // Enable  nand speed clock.
        NvBootClocksSetEnable(NvBootClocksClockId_NandSpeedId, NV_TRUE);
    }
    // Remove the controller from Reset.
    NvBootResetSetEnable(NvBootResetDeviceId_NandId, NV_FALSE);
   // For GMI pin mux option, CFG_RETIME_STAGES=0x0 setting is required
    NV_NAND_READ(FBIO_CFG, FbioCfgReg);
    FbioCfgReg = NV_FLD_SET_DRF_NUM(NAND,
                            FBIO_CFG, 
                            CFG_RETIME_STAGES,
                            0,
                            FbioCfgReg);
    NV_NAND_WRITE(FBIO_CFG, FbioCfgReg);
    NV_NAND_READ(CONFIG2, ConfigReg2);
    // Enable T30 mode
    ConfigReg2 = NV_DRF_DEF(NAND, CONFIG2, PROG_OFFSET_EN, ENABLE);
    // Select Bch sector size
    ConfigReg2 = NV_FLD_SET_DRF_NUM(NAND,
                            CONFIG2, 
                            CODEWORD_SEL,
                            s_NandContext->BchSectorSize,
                            ConfigReg2);
    // Select new Timing Registers and disable sync ddr mode
    ConfigReg2 |= NV_DRF_DEF(NAND, CONFIG2, TIMING_SEL, ENABLE) |
        NV_DRF_DEF(NAND, CONFIG2, SYNC_DDR, DISABLE);
    NV_NAND_WRITE(CONFIG2, ConfigReg2);
    // Set Main ECC Offset
    NV_NAND_READ(MAIN_ECC_OFFSET, MainEccOffsetReg);
    MainEccOffsetReg = NV_DRF_NUM(NAND, MAIN_ECC_OFFSET, START_ADDR,
         ((s_NandContext->MainEccOffset)*4) + 4);
    NV_NAND_WRITE(MAIN_ECC_OFFSET, MainEccOffsetReg);
    // Set the Nand Async timing registers.
    NV_NAND_WRITE(ASYNC_TIMING_0, Params->NandAsyncTiming0);
    NV_NAND_WRITE(ASYNC_TIMING_1, Params->NandAsyncTiming1);
    NV_NAND_WRITE(ASYNC_TIMING_2, Params->NandAsyncTiming2);
    NV_NAND_WRITE(ASYNC_TIMING_3, Params->NandAsyncTiming3);
}

static NvBootError HwNandWaitCommandDone(void)
{
    NvU32 CommandGo;
    NvU32 CommandRegValue;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    
    // Wait for Command to be sent out with time out.
    while (TimeOutCounter)
    {
        NV_NAND_READ(COMMAND, CommandRegValue);
        CommandGo = NV_DRF_VAL(NAND, COMMAND, GO, CommandRegValue);
        // CommandGo = 1 indicates that command send cycle is not yet complete.
        // CommandGo = 0 indicates that command is sent.
        if (CommandGo == 0)
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
    }
    return NvBootError_Success;
}

static NvBootError HwNandWaitDmaDone(void)
{
    NvU32 DmaGo;
    NvU32 IsIdle;
    NvU32 StatusRegValue;
    NvU32 DmaMasterControl;
    NvU32 TimeOutCounter = NVBOOT_NAND_READ_TIMEOUT_IN_US;
    
    // Wait for Command to be sent out with time out.
    while (TimeOutCounter)
    {
        // Check whether the read is completed.
        NV_NAND_READ(DMA_MST_CTRL, DmaMasterControl);
        DmaGo = NV_DRF_VAL(NAND, DMA_MST_CTRL, DMA_GO, DmaMasterControl);
        // Check the status register.
        NV_NAND_READ(STATUS, StatusRegValue);
        IsIdle = NV_DRF_VAL(NAND, STATUS, ISEMPTY, StatusRegValue);
        // DmaGo = 1 indicates that Dma is still busy.
        // DmaGo = 0 indicates that Dma is done. i.e. reading is done.
        if ( (DmaGo == 0) && IsIdle )
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
    }
    return NvBootError_Success;
}

static NvBootError HwNandWaitChipReady(void)
{
    NvU32 StatusReg;
    NvU32 ReadyBusy;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    
    // Wait for chip to be ready with time out.
    while (TimeOutCounter)
    {
        NV_NAND_READ(STATUS, StatusReg);
        ReadyBusy = NV_DRF_VAL(NAND, STATUS, RBSY0, StatusReg);
        // ReadyBusy = 1 indicates that chip is ready.
        // ReadyBusy = 0 indicates that chip is busy.
        if (ReadyBusy)
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
    }
    return NvBootError_Success;
}

static void 
HwNandSetupAddressRegisters(
    NvBootNandCommand* pCommand)
{
    NvU32 AddressReg1;
    NvU32 AddressReg2;
    // This should be able to hold up to 5 bytes of page address.
    NvU64 PageAddress;
    
    PageAddress = (pCommand->BlockNumber << 
                   s_NandContext->PagesPerBlockLog2) + pCommand->PageNumber;
    // Setup Address Registers
    AddressReg1 = pCommand->ColumnNumber | 
                  ((PageAddress & NVBOOT_NAND_ADDR_REG_1_PAGE_MASK) <<
                    NVBOOT_NAND_ADDR_REG_1_PAGE_OFFSET);
    AddressReg2 = (PageAddress >> NVBOOT_NAND_ADDR_REG_2_PAGE_OFFSET);
    NV_NAND_WRITE(ADDR_REG1, AddressReg1);
    NV_NAND_WRITE(ADDR_REG2, AddressReg2);
}

static NvBootError HwNandSendCommand(NvBootNandCommand* pCommand)
{
    NvBootError e;
    NvU32 CommandReg = 0;
    NvU32 IsrRegValue;
    
    // Read and write Isr Register to clear previously set events.
    NV_NAND_READ(ISR, IsrRegValue);
    NV_NAND_WRITE(ISR, IsrRegValue);
    // setup command cycle byte count for enhanced commands
    if (pCommand->Command1 == NvBootNandCommands_Read_DeviceParameterStart_Status) 
    {
        CommandReg  |= NV_DRF_DEF(NAND, COMMAND, CLE_BYTE_SIZE, CLE_BYTES3);
    }
    // Setup command registers with commands.
    NV_NAND_WRITE(CMD_REG1, pCommand->Command1);
    NV_NAND_WRITE(CMD_REG2, pCommand->Command2);
    if ((pCommand->Command1 != NvBootNandCommands_Read_DeviceParameterStart_Status) 
        && (pCommand->Command1 != NvBootNandCommands_ReadMode))
    {
        // Setup the address registers.
        HwNandSetupAddressRegisters(pCommand);
    }

    // Setting up the command register.
    // Enable Command cycle.
    // Enable Address cycle.
    CommandReg |= NV_DRF_DEF(NAND, COMMAND, CLE, ENABLE) |
                 NV_DRF_DEF(NAND, COMMAND, ALE, ENABLE);
   
    switch (pCommand->Command1)
    {
        case NvBootNandCommands_Reset:
            // Enable Command cycle.
            CommandReg = NV_DRF_DEF(NAND, COMMAND, CLE, ENABLE);
            break;
        case NvBootNandCommands_ReadId:
            // Enable Rx data.
            // Enable PIO mode.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, RX, ENABLE);
            // Set the bytes to receive as 4 incase of ONFI signature otherwise receive as 8.
            if (pCommand->ColumnNumber == 0x20)
                CommandReg |= NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES4);
            else
                CommandReg |= NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES8);
            break;
        case NvBootNandCommands_Read:
            // Enable Rx data.
            // Enable Secondary command.
            // Set ALE bytes.
            // Enable main data receive.
            // Set the bytes to receive as page size of nand.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, SEC_CMD, ENABLE) |
                          NV_DRF_NUM(NAND, COMMAND, ALE_BYTE_SIZE,
                            (s_NandContext->NumOfAddressCycles - 1)) |
                          NV_DRF_DEF(NAND, COMMAND, A_VALID, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, 
                            BYTES_PAGE_SIZE_SEL);
            break;
        case NvBootNandCommands_ReadParamPage:
            break;
        case NvBootNandCommands_ReadParamPageStart:
            CommandReg = NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, A_VALID, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, 
                            BYTES_PAGE_SIZE_SEL);
            break;
        case NvBootNandCommands_SetFeature:
            // Send the parameters 
            NV_NAND_WRITE(RESP, pCommand->Param32);
            // Enable Tx data.
            // Enable PIO mode.
            // Set the bytes to send as 4.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, TX, ENABLE) | 
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES4);
            break;
        case NvBootNandCommands_Status:
            // Enable Rx data.
            // Enable PIO mode.
            // Set the bytes to receive as 1.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) | 
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES1);
            break;
        case NvBootNandCommands_DeviceStatus:
            // Enable Rx data.
            // Enable PIO mode.
            // Set the bytes to receive as 1.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) | 
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES1);
            break;
        case NvBootNandCommands_ReadMode:
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, A_VALID, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, 
                            BYTES_PAGE_SIZE_SEL);
            break;
        case NvBootNandCommands_Read_DeviceParameter:
            break;
        case NvBootNandCommands_Read_DeviceParameterStart_Status:
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES1);
        case NvBootNandCommands_OperationStatus:
            // Enable Rx data.
            // Enable PIO mode.
            // Set the bytes to receive as 1.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, RX, ENABLE) | 
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES1);
            break;
        case NvBootNandCommands_GetNext_OperationStatus:
            break;

        default:
            NV_ASSERT(NV_FALSE);
    }
    // Enable chip select.
    // Send the command out.
    CommandReg |= NV_DRF_DEF(NAND, COMMAND, CE0, ENABLE) |
                  NV_DRF_DEF(NAND, COMMAND, GO, ENABLE);
    NV_NAND_WRITE(COMMAND, CommandReg);
    // Wait till command is done and chip is ready.
    NV_BOOT_CHECK_ERROR(HwNandWaitCommandDone());
    NV_BOOT_CHECK_ERROR(HwNandWaitChipReady());
    return e;
}

static NvBootError HwNandResetNandFlash(void)
{
    NvBootError e;
    NvBootNandCommand* pCommand = &s_NandContext->Command;
    
    pCommand->Command1 = NvBootNandCommands_Reset;
    e = HwNandSendCommand(pCommand);
    return e;
}

static NvBootError HwNandReadId(void)
{
    NvU32 ReadId[2];
    NvBootError e;
    NvU32 TempData;
    NvBootNandCommand* pCommand = &s_NandContext->Command;
#if JEDEC_ID_ENABLE
    NvS8* JedecId = "JEDEC";

    // Send Read Id command for JEDEC ID
    pCommand->Command1 = NvBootNandCommands_ReadId;
    pCommand->Command2 = NvBootNandCommands_CommandNone;
    pCommand->ColumnNumber = NvBootNandCommands_JedecId_Address;
    pCommand->PageNumber = 0;
    pCommand->BlockNumber = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    NV_NAND_READ(RESP, ReadId[0]);
    NV_NAND_READ(RESP2, ReadId[1]);
    // Check for JEDEC signature
    if (!NvBootUtilMemcpy(JedecId, ReadId, 5))
    {
        // check for Toggle mode support
        s_NandContext->IsToggleModeSupported = 
        NV_DRF_VAL(NAND_DEVICE, READID2_JEDEC, IS_TOGGLE, ReadId[1]);
    }
#endif

    pCommand->Command1 = NvBootNandCommands_ReadId;
    pCommand->BlockNumber = 0;
    pCommand->PageNumber = 0;
    pCommand->ColumnNumber = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    NV_NAND_READ(RESP, ReadId[0]);
    NV_NAND_READ(RESP2, ReadId[1]);
    if (s_FuseInfo.ToggleDDR ==  NvBootNandToggleDDR_Discovey)
    {
        NvBool IsTDDRSupported = NV_FALSE;

        s_NandBitInfo->IdRead2 = ReadId[1];
        // check for Toggle mode support
        IsTDDRSupported = NV_DRF_VAL(NAND_DEVICE, READID2, IS_TOGGLE, ReadId[1]);
        if (IsTDDRSupported)
            s_NandContext->ToggleDDR = NvBootNandToggleDDR_Enable;
        else
            s_NandContext->ToggleDDR = NvBootNandToggleDDR_Disable;
    }
    s_NandBitInfo->IdRead = ReadId[0];
    // Extract the Nand params from id data.
    s_NandContext->MakerCode = NV_DRF_VAL(NAND_DEVICE, READID,
        MAKER_CODE, ReadId[0]);
    s_NandContext->DeviceCode = NV_DRF_VAL(NAND_DEVICE, READID,
        DEVICE_CODE, ReadId[0]);
    // Extract the page size 
    TempData = NV_DRF_VAL(NAND_DEVICE, READID, PAGE_SIZE, ReadId[0]);
    /*
     * Two bits are used to represent the Page size. The bits value starting 
     * from zero to three, corresponds to page size log2 of 10, 11, 12 or 13.
     * This is equivalent to adding 10 to the bit value.
     */
    if (s_NandContext->PageSizeLog2 == 0)
        s_NandContext->PageSizeLog2 = 10 + TempData + s_FuseInfo.PageBlockSizeOffset;
    // AP20 BootRom intend to support pages of size 2KB,4KB and 8KB only.
    if ( (s_NandContext->PageSizeLog2 < NVBOOT_NAND_2KB_PAGE_SIZE_IN_LOG2) ||
         (s_NandContext->PageSizeLog2 > NVBOOT_NAND_8KB_PAGE_SIZE_IN_LOG2) )
        return NvBootError_DeviceUnsupported;
    // Extract the block size 
    TempData = NV_DRF_VAL(NAND_DEVICE, READID, BLOCK_SIZE, ReadId[0]);
    /*
     * Two bits are used to represent the Block size. The bits value starting 
     * from zero to three, corresponds to block size log2 of 16, 17, 18 or 19.
     * This is equivalent to adding 16 to the bit value.
     */
    if (s_NandContext->BlockSizeLog2 == 0)
        s_NandContext->BlockSizeLog2 = 16 + TempData + s_FuseInfo.PageBlockSizeOffset;

    s_NandContext->DataWidth = s_FuseInfo.DataWidth;
    if (s_FuseInfo.DataWidth == NvBootNandDataWidth_Discovey)
    {
        // Extract bus width. Valid bus width enum starts from value '1'. 
        // So, add '1'.
        TempData = NV_DRF_VAL(NAND_DEVICE, READID, BUS_WIDTH, ReadId[0]);
        s_NandContext->DataWidth = (NvBootNandDataWidth)(TempData + 1);
    }
    s_NandContext->PagesPerBlockLog2 = s_NandContext->BlockSizeLog2 - 
                                       s_NandContext->PageSizeLog2;
    return e;
}

static void HwNandSetupEccOption(NvBootNandEccSelection EccSelection)
{
    NvU32 ConfigReg;
    NvU32 BchConfig = 0;
    const NvU32 BchTvalueArray[] = 
    {
        0,
        NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL4),
        NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL8),
        NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL16),
        NV_DRF_DEF(NAND, BCH_CONFIG, BCH_TVALUE, BCH_TVAL24)
    };
    NV_ASSERT(EccSelection != NvBootNandEccSelection_Discovery);

    PRINT_NAND_MESSAGES("\r\nEccOption=%d(0->Disc, 1->Bch4, 2->Bch8, 3->Bch16,"
        "4->Bch24, 5->Off)", EccSelection);
    NV_NAND_READ(CONFIG, ConfigReg);
    /*
     * Config Register is cleared before calling this function, which means 
     * that the Ecc selection is off.
     */
    if (EccSelection == NvBootNandEccSelection_Bch4 ||
        EccSelection == NvBootNandEccSelection_Bch8 ||
        EccSelection == NvBootNandEccSelection_Bch16 ||
        EccSelection == NvBootNandEccSelection_Bch24)
    {
        // Select Ecc as Bch4 or Bch8 or Bch16 or Bch24.
        BchConfig = BchTvalueArray[EccSelection] |
                    NV_DRF_DEF(NAND, BCH_CONFIG, BCH_ECC, ENABLE);
    }
    else
    {
        // Ecc is turned off.
        // Re-read Config Register to discard above changes.
        NV_NAND_READ(CONFIG, ConfigReg);
    }
    NV_NAND_WRITE(CONFIG, ConfigReg);
    NV_NAND_WRITE(BCH_CONFIG, BchConfig);
}

static void HwNandSetupDma(NvU8* pBuffer, NvU32 NumOfBytes)
{
    NvU32 ConfigReg;
    NvU32 DmaMasterControl;
    NvU8 PageSize2KBLog2 = 11;
    const NvU32 PageSizeValArray[] = 
    {
        NV_DRF_DEF(NAND, CONFIG, PAGE_SIZE_SEL, PAGE_SIZE_512),
        NV_DRF_DEF(NAND, CONFIG, PAGE_SIZE_SEL, PAGE_SIZE_2048),
        NV_DRF_DEF(NAND, CONFIG, PAGE_SIZE_SEL, PAGE_SIZE_4096),
        NV_DRF_DEF(NAND, CONFIG, PAGE_SIZE_SEL, PAGE_SIZE_8192),
        NV_DRF_DEF(NAND, CONFIG, PAGE_SIZE_SEL, PAGE_SIZE_16384),
    };
    NvU32 Index = (NumOfBytes >> PageSize2KBLog2);
    NvS8 PageIndex= 0;
    while(Index)
    {
        Index >>= 1;
        PageIndex++;
    }
    // Check for expected values.
    NV_ASSERT((NumOfBytes == 512) || (NumOfBytes == 2048) ||
        (NumOfBytes == 4096) || (NumOfBytes == 8192) || (NumOfBytes == 16384));
    NV_ASSERT(PageIndex < NV_ARRAY_SIZE(PageSizeValArray));
    PRINT_NAND_MESSAGES("\r\nSetting up Dma");
    // Set data block pointer.
    NV_NAND_WRITE(DATA_BLOCK_PTR, PTR_TO_ADDR(pBuffer));
    // Setting Dma data transfer size.
    NV_NAND_WRITE(DMA_CFG_A, (NumOfBytes - 1));

    // Select the page size.
    // Select bus width as 8 or 16 bit.
    ConfigReg = PageSizeValArray[PageIndex];
    if (s_NandContext->DataWidth == NvBootNandDataWidth_16Bit)
        ConfigReg |= NV_DRF_DEF(NAND, CONFIG, BUS_WIDTH, BUS_WIDTH_16);
    NV_NAND_WRITE(CONFIG, ConfigReg);
    // Set Ecc selection. This method changes CONFIG and BCH_CONFIG Regs.
    HwNandSetupEccOption(s_NandContext->EccSelection);

    // Set the Dma burst size to 8 words.
    // Enable Dma for main area.
    // Finally, Enable the Dma.
    DmaMasterControl = NV_DRF_DEF(NAND, DMA_MST_CTRL, BURST_SIZE, BURST_8WORDS)|
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_EN_A, ENABLE) |
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_GO, ENABLE);
    NV_NAND_WRITE(DMA_MST_CTRL, DmaMasterControl);
}

static NvS8 NvBootGetMsbSet(NvU32 Value)
{
    NvS8 ReturnValue = -1;
    while(Value)
    {
        Value >>= 1;
        ReturnValue++;
    }
    return ReturnValue;
}

static NvU16 NvBootCalculateCrc16(NvU8* pBuffer)
{
    // Bit by bit algorithm without augmented zero bytes
    const unsigned long Crcinit = NVBOOT_NAND_ONFI_CRC16_INIT;   // Initial CRC value in the shift register
    const NvU32 Order = NVBOOT_NAND_ONFI_CRC16_ORDER;               // Order of the CRC-16 
    const unsigned long Polynom = NVBOOT_NAND_ONFI_CRC16_POLYNOMIAL;   // Polynomial 
    unsigned long Crc = Crcinit;
    unsigned long j, c, bit;
    NvU32 DataByteCount;
    unsigned long Crcmask, Crchighbit;
    NvU32 paramdatasize = 254;

    Crcmask = ((((unsigned long)1<<(Order-1))-1)<<1)|1;
    Crchighbit = (unsigned long)1<<(Order-1);
    // Input byte stream, one byte at a time, bits processed from MSB to LSB
    for (DataByteCount = 0; DataByteCount < paramdatasize;
            DataByteCount++)
    {
        c = (unsigned long) pBuffer[DataByteCount];
        for (j=0x80; j; j>>=1)
        {
            bit = Crc & Crchighbit;
            Crc<<= 1;
            if (c & j) bit^= Crchighbit; 
            if (bit) Crc^= Polynom;
        }
        Crc&= Crcmask;
    }
    return (NvU16) Crc;
}
    static NvBootError HwNandReadPageStatus(void)
    {
        NvBootNandCommand* pCommand = &s_NandContext->Command;
        NvBootError e;
    
        // Set up the command structure.
        pCommand->Command1 = NvBootNandCommands_Status;
        pCommand->Command2 = NvBootNandCommands_CommandNone;
        pCommand->ColumnNumber = 0;
        pCommand->PageNumber = 0;
        pCommand->BlockNumber = 0;
    
        // Send the Read Status Command.
        NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
        return e;
    }
// Enhanced command set. Supported for EF Nand 2.0/3.0 & Enhanced clear Nand devices
static NvBootError HwNandReadMode(void)
{
    NvBootNandCommand* pCommand = &s_NandContext->Command;
    NvBootError e;
    
    // issue controller status command
    pCommand->Command1 = NvBootNandCommands_ReadMode;
    pCommand->Command2 = NvBootNandCommands_CommandNone;
    pCommand->ColumnNumber = 0;
    pCommand->PageNumber = 0;
    pCommand->BlockNumber = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    return e;
}


static NvBootError HwNandDeviceStatus(void)
{
    NvBootNandCommand* pCommand = &s_NandContext->Command;
    NvBootError e;
    NvU32 StatusRegValue;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;

    // issue devicer status command
    pCommand->Command1 = NvBootNandCommands_DeviceStatus;
    pCommand->Command2 = NvBootNandCommands_CommandNone;
    pCommand->ColumnNumber = 0;
    pCommand->PageNumber = 0;
    pCommand->BlockNumber = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    // Wait for device status to become valid
    while (TimeOutCounter)
    {
         NV_BOOT_CHECK_ERROR(HwNandDeviceStatus());
        // Read device status
        NV_NAND_READ(RESP, StatusRegValue);
        // wait for device Ready .
        if (StatusRegValue == 0x40)
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
    }
    return e;
}

static NvBootError HwNandGetNextOperationStatus(void)
{

NvBootNandCommand* pCommand = &s_NandContext->Command;
NvBootError e;

// issue controller status command
pCommand->Command1 = NvBootNandCommands_GetNext_OperationStatus;
pCommand->Command2 = NvBootNandCommands_CommandNone;
pCommand->ColumnNumber = 0;
pCommand->PageNumber = 0;
pCommand->BlockNumber = 0;
NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
return e;
}

static NvBootError HwNandGetOperationStatus(void)
{
    NvBootNandCommand* pCommand = &s_NandContext->Command;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    NvU32 StatusRegValue;
    NvBootError e;

    // issue controller status command
    pCommand->Command1 = NvBootNandCommands_OperationStatus;
    pCommand->Command2 = NvBootNandCommands_CommandNone;
    pCommand->ColumnNumber = 0;
    pCommand->PageNumber = 0;
    pCommand->BlockNumber = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    // Wait for operation status to become valid
    while (TimeOutCounter)
    {
        // Send operation status command
         NV_BOOT_CHECK_ERROR(HwNandGetOperationStatus());
        // Read controller status
        NV_NAND_READ(RESP, StatusRegValue);
        // wait for Ready/Busy .
        if (StatusRegValue == 0x40)
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
    }
    return e;
}

    // ReadDevice Parameter contains device info similar to ONFI ReadParameter page.
    // Supported for EF Nand 2.0/3.0 devices
static NvBootError NvBootEFNandReadDeviceParameter()
{
    NvBootNandCommand* pCommand = &s_NandContext->Command;
    NvBootError e = NvBootError_None;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    NvU8* pBuffer = NULL;
    NvU32 ConfigReg2;
    NvU32 StatusRegValue;
    NvU32 BufferIndex = 0;

    pCommand->Command1 = NvBootNandCommands_Read_DeviceParameter;
    pCommand->Command2 = NvBootNandCommands_CommandNone;
    pCommand->ColumnNumber = 0;
    pCommand->PageNumber = 0;
    pCommand->BlockNumber = 0;
    s_NandContext->DataWidth = NvBootNandDataWidth_8Bit;
    s_NandContext->EccSelection = NvBootNandEccSelection_Off;
    // For Reading param page 512 Bch sector size need to be used.
    if ( s_NandContext->BchSectorSize == NvBootNandBchSectorSize_1024)
    {
        NV_NAND_READ(CONFIG2, ConfigReg2);
        ConfigReg2 = NV_FLD_SET_DRF_DEF(NAND,
                                CONFIG2, 
                                CODEWORD_SEL,
                                BCH_512,
                                ConfigReg2);
        NV_NAND_WRITE(CONFIG2, ConfigReg2);
    }
    pBuffer = Buffer[BufferIndex];
    // Setup the Dma
    HwNandSetupDma(pBuffer, NVBOOT_EFNAND_PARAM_BUF_SIZE);
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    do
    {
        pCommand->Command1 = NvBootNandCommands_Read_DeviceParameterStart_Status;
        NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
        // Read controller status
        NV_NAND_READ(RESP, StatusRegValue);
        // wait for Ready/Busy .
        if (StatusRegValue == 0x40)
            break;
        NvBootUtilWaitUS(1);
        TimeOutCounter--;
        if (!TimeOutCounter)
            return NvBootError_HwTimeOut;
     } while(TimeOutCounter);
    pCommand->Command1 = NvBootNandCommands_ReadMode;
    pCommand->Command2 = NvBootNandCommands_GetNext_OperationStatus;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    NV_BOOT_CHECK_ERROR(HwNandWaitDmaDone());
    // Read Operation status
    NV_NAND_READ(RESP, StatusRegValue);
    // check I/O bit 0 for pass/fail
    if (StatusRegValue & 0x01)
        return NvBootError_DeviceError;

    // Again configure Bch sector size according to the fuse setting
    if ( s_NandContext->BchSectorSize == NvBootNandBchSectorSize_1024)
    {
        NV_NAND_READ(CONFIG2, ConfigReg2);
        ConfigReg2 = NV_FLD_SET_DRF_NUM(NAND,
                                CONFIG2, 
                                CODEWORD_SEL,
                                s_NandContext->BchSectorSize,
                                ConfigReg2);
        NV_NAND_WRITE(CONFIG2, ConfigReg2);
    }
    s_NandContext->PageSizeLog2 = NvBootUtilGetLog2Number( 
            pBuffer[NVBOOT_EFNAND_PAGE_SIZE_BYTE0_OFFSE] | 
            (pBuffer[NVBOOT_EFNAND_PAGE_SIZE_BYTE1_OFFSE] << 8) | 
            (pBuffer[NVBOOT_EFNAND_PAGE_SIZE_BYTE2_OFFSE] << 16) | 
            (pBuffer[NVBOOT_EFNAND_PAGE_SIZE_BYTE3_OFFSE] << 24) );
    if ( (s_NandContext->PageSizeLog2 < NVBOOT_NAND_2KB_PAGE_SIZE_IN_LOG2) || 
         (s_NandContext->PageSizeLog2 > NVBOOT_NAND_16KB_PAGE_SIZE_IN_LOG2) )
    {
        // This will let normal identification overwrite this value.
        s_NandContext->PageSizeLog2 = 0;
        return NvBootError_DeviceUnsupported;
    }
    s_NandContext->PagesPerBlockLog2 = NvBootUtilGetLog2Number( 
        pBuffer[NVBOOT_EFNAND_PAGES_PER_BLOCK_BYTE0_OFFSET] | 
        (pBuffer[NVBOOT_EFNAND_PAGES_PER_BLOCK_BYTE1_OFFSET] << 8) | 
        (pBuffer[NVBOOT_EFNAND_PAGES_PER_BLOCK_BYTE2_OFFSET] << 16) | 
        (pBuffer[NVBOOT_EFNAND_PAGES_PER_BLOCK_BYTE3_OFFSET] << 24) );
    s_NandContext->BlockSizeLog2 = s_NandContext->PagesPerBlockLog2 + 
                                   s_NandContext->PageSizeLog2;
    return e;
}

static NvBootError HwNandInitializeOnfiFlash(const NvBootNandParams *Params)
{
    NvBootError e;
    NvU8* pBuffer = NULL;
    NvU32 TempData;
    NvU32 ConfigReg2;
    NvU16 Crc16 = 0;
    NvU16 DeviceCrc16 = 0;
    NvU16 RedundantParamPage = 0;
    NvU16 ParamPageOffset = 0;
    NvU32 AllParamPagesFail;
    NvU32 FirstGoogParamPage;
    NvU32 BufferIndex = 0;

    NvBootNandCommand* pCommand = &s_NandContext->Command;

    // Reset Nand Flash.
    NV_BOOT_CHECK_ERROR(HwNandResetNandFlash());
    // Configure the device DQ signals as non-mirrored mode by sending the legacy status cmd
    //required for Enhanced clear nand
    NV_BOOT_CHECK_ERROR(HwNandReadPageStatus());
    // Send Read Id command.
    pCommand->Command1 = NvBootNandCommands_ReadId;
    pCommand->Command2 = NvBootNandCommands_CommandNone;
    pCommand->ColumnNumber = NvBootNandCommands_ReadId_Address;
    pCommand->PageNumber = 0;
    pCommand->BlockNumber = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    NV_NAND_READ(RESP, TempData);
    s_NandBitInfo->IdRead = TempData;
    // Check the signature.
    if ( TempData == NVBOOT_NAND_ONFI_SIGNATURE )
    {
        // If ONFI parameter page buffer is already filled don't issue the param page 
        // command again
        if (s_NandContext->ReadParamOnfiSignature == NVBOOT_NAND_ONFI_SIGNATURE)
        {
            pBuffer = Buffer[BufferIndex];
        }
        else
        {
            pCommand->Command1 = NvBootNandCommands_ReadParamPage;
            pCommand->Command2 = NvBootNandCommands_CommandNone;
            pCommand->ColumnNumber = 0;
            pCommand->PageNumber = 0;
            pCommand->BlockNumber = 0;
            // For Reading param page 8bit bus width need to be used.
            s_NandContext->DataWidth = NvBootNandDataWidth_8Bit;
            s_NandContext->EccSelection = NvBootNandEccSelection_Off;
            // For Reading param page 512 Bch sector size need to be used.
            if ( s_NandContext->BchSectorSize == NvBootNandBchSectorSize_1024)
            {
                NV_NAND_READ(CONFIG2, ConfigReg2);
                ConfigReg2 = NV_FLD_SET_DRF_DEF(NAND,
                                        CONFIG2, 
                                        CODEWORD_SEL,
                                        BCH_512,
                                        ConfigReg2);
                NV_NAND_WRITE(CONFIG2, ConfigReg2);
            }
            // Enable HW CRC by checking the SW CYA fuse
            if (!(NvBootGetSwCYA() & NVBOOT_SW_CYA_ONFI_HW_CRC_CHECK_DISABLE))
            {
                NV_NAND_READ(CONFIG2, ConfigReg2);
                ConfigReg2 = NV_FLD_SET_DRF_DEF(NAND,
                                        CONFIG2,
                                        ONFI_PARAM_CRC_CHECK_ENABLE,
                                        ENABLE,
                                        ConfigReg2);
                NV_NAND_WRITE(CONFIG2, ConfigReg2);
            }
            pBuffer =  Buffer[BufferIndex];
            // Setup the Dma
            HwNandSetupDma(pBuffer, NVBOOT_NAND_ONFI_PARAM_BUF_SIZE);
            NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
            pCommand->Command1 = NvBootNandCommands_ReadParamPageStart;
            NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
            NV_BOOT_CHECK_ERROR(HwNandWaitDmaDone());
            // Run SW CRC if HW CRC check is disabled via irom patch.
            if ((NvBootGetSwCYA() & NVBOOT_SW_CYA_ONFI_HW_CRC_CHECK_DISABLE))
            {
                 // calculate CRC for parameter page data starting from 0 to 253 bytes
                do
                {
                     ParamPageOffset = RedundantParamPage*NVBOOT_NAND_ONFI_PARAM_PAGE_SIZE;
                     pBuffer += ParamPageOffset;
                     Crc16 = NvBootCalculateCrc16(pBuffer);
                     // read LSB to pBuffer[255] and MSB to pBuffer[254] 
                     DeviceCrc16 = (pBuffer[ParamPageOffset + NVBOOT_NAND_ONFI_CRC_BYTE1_OFFSET] << 8) |
                         (pBuffer[ParamPageOffset + NVBOOT_NAND_ONFI_CRC_BYTE0_OFFSET]);
                     RedundantParamPage++;
                 } while((DeviceCrc16 != Crc16) && (RedundantParamPage < 
                     NVBOOT_NAND_ONFI_MAX_REDUNDANT_PARAM_PAGES));
                 // Return error if CRC validation fails for all the parameter pages available on the device
                 if (DeviceCrc16 != Crc16)
                     return NvBootError_DeviceUnsupported;
            }
            else
            {
                NV_NAND_READ(CONFIG2, ConfigReg2);
                AllParamPagesFail = NV_DRF_VAL(NAND, CONFIG2, ALL_PARAM_PAGES_FAIL,
                                                ConfigReg2);
                if (AllParamPagesFail == 1)
                {
                    return NvBootError_DeviceUnsupported;
                }
                else
                {
                   FirstGoogParamPage =  NV_DRF_VAL(NAND, CONFIG2, 
                                                        FIRST_GOOD_ONFI_PARAM_PAGE, ConfigReg2);
                   ParamPageOffset = FirstGoogParamPage*NVBOOT_NAND_ONFI_PARAM_PAGE_SIZE;
                   pBuffer += ParamPageOffset;
                }
            }
        }
        // Again configure Bch sector size according to the fuse setting
        if ( s_NandContext->BchSectorSize == NvBootNandBchSectorSize_1024)
        {
            NV_NAND_READ(CONFIG2, ConfigReg2);
            ConfigReg2 = NV_FLD_SET_DRF_NUM(NAND,
                                    CONFIG2, 
                                    CODEWORD_SEL,
                                    s_NandContext->BchSectorSize,
                                    ConfigReg2);
            NV_NAND_WRITE(CONFIG2, ConfigReg2);
        }
        // Check the signature again. It should match.
        TempData = ( pBuffer[NVBOOT_NAND_ONFI_SIGNATURE_BYTE0_OFFSET] | 
                            (pBuffer[NVBOOT_NAND_ONFI_SIGNATURE_BYTE1_OFFSET] << 8) | 
                            (pBuffer[NVBOOT_NAND_ONFI_SIGNATURE_BYTE2_OFFSET] << 16) | 
                            (pBuffer[NVBOOT_NAND_ONFI_SIGNATURE_BYTE3_OFFSET] << 24) );
        if ( TempData != NVBOOT_NAND_ONFI_SIGNATURE )
            return NvBootError_DeviceUnsupported;
        s_NandContext->ReadParamOnfiSignature = TempData;
        // Extract onfi revision number
        s_NandContext->OnfiRevNum = 
        (NvBootNandOnfiRevNum) NvBootGetMsbSet(
        pBuffer[NVBOOT_NAND_ONFI_REVISION_NUMBER_BYTE_OFFSET]);
        // Check EZ Nand supported or not
        s_NandContext->IsOnfiEZNand = 
            (pBuffer[NVBOOT_NAND_ONFI_EZNAND_SUPPORT_BYTE_OFFSET] & 
                NVBOOT_NAND_ONFI_EZNAND_SUPPORT_MASK) ? NV_TRUE : NV_FALSE;
        // Check ONFI 2.3 Rev & EZNand support before Enable/Disable automatic retries feature
        if ((s_NandContext->OnfiRevNum == NvBootNandOnfiRevNum_2_3) ||
            (s_NandContext->IsOnfiEZNand))
        {
            // Check whether auto retries supported or not
            s_NandContext->IsSupportAutoRetries = 
                (pBuffer[NVBOOT_NAND_ONFI_EZNAND_SUPPORT_AUTO_RETRIES_BYTE_OFFSET] &
                    NVBOOT_NAND_ONFI_EZNAND_SUPPORT_AUTO_RETRIES_MASK) ?
                    NV_TRUE : NV_FALSE;
            // If AutoRetries is set to one, then enable automatic retries using Set Feature.
            // If AutoRetries is set to zero, then the EZ Nand controller determines whether to
            // perform a retry without host intervention
            if (s_NandContext->IsSupportAutoRetries)
            {
                // Enable/Disable automatic retires feature using Set Feature command
                pCommand->Command1 = NvBootNandCommands_SetFeature;
                pCommand->Command2 = NvBootNandCommands_CommandNone;
                pCommand->ColumnNumber = 
                NvBootNandCommands_SetFeature_EZNand_Control_Address;
                pCommand->PageNumber = 0;
                pCommand->BlockNumber = 0;
                pCommand->Param32 = NvBootOnfiEZNandAutoRetries_Enable;
                NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
            }
        }
        // Check NAND part supports source synchronous interface or not
        s_NandContext->IsSyncDDRModeSupported = 
        (pBuffer[NVBOOT_NAND_ONFI_DDR_SUPPORT_BYTE_OFFSET] & 
        NVBOOT_NAND_ONFI_DDR_SUPPORT_MASK) ? NV_TRUE : NV_FALSE;
        // Identify enhanced clear nand part by checking the EZNand & sync ddr support
        if ((s_NandContext->IsSyncDDRModeSupported) && (s_NandContext->IsOnfiEZNand))
            s_NandContext->IsEnhancedClearNand = NV_TRUE;
        // SyncDDR interface can be enabled or disabled through BCT 
        if (s_NandContext->DisableSyncDDR)
        {
            s_NandContext->IsSyncDDRModeSupported = NV_FALSE;
        }
        else
        {
            NvU32 ConfigReg2;
            NvBool IsSetFeatureCmdSupported;

            // Check SetFeature command supported on device or not
            IsSetFeatureCmdSupported = 
            (pBuffer[NVBOOT_NAND_ONFI_SETFEATURE_CMD_SUPPORT_BYTE_OFFSET] & 
            NVBOOT_NAND_ONFI_SETFEATURE_CMD_SUPPORT_MASK) ? NV_TRUE : NV_FALSE;
            if ((!IsSetFeatureCmdSupported) ||(!s_NandContext->IsSyncDDRModeSupported))
                return NvBootError_DeviceUnsupported;
            // Extract max synchronous timing mode supported on device
            s_NandContext->MaxSyncTimingMode =
            NvBootGetMsbSet(
             pBuffer[NVBOOT_NAND_ONFI_SYNC_TIMING_MODE_BYTE_OFFSET]);
            // switch to Sync DDR mode using setfeature command
            pCommand->Command1 = NvBootNandCommands_SetFeature;
            pCommand->Command2 = NvBootNandCommands_CommandNone;
            pCommand->ColumnNumber = 
            NvBootNandCommands_SetFeature_TimingMode_Address;
            pCommand->PageNumber = 0;
            pCommand->BlockNumber = 0;
            pCommand->Param32 =  (s_NandContext->MaxSyncTimingMode | 
            (NvBootNandDataInterface_Sync << 4));
            NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
            // Set Sync DDR mode timing registers
            NV_NAND_WRITE(ASYNC_TIMING_0, Params->NandAsyncTiming0);
            NV_NAND_WRITE(ASYNC_TIMING_1, Params->NandAsyncTiming1);
            NV_NAND_WRITE(ASYNC_TIMING_2, Params->NandAsyncTiming2);
            NV_NAND_WRITE(SDDR_TIMING_0, Params->NandSDDRTiming0);
            NV_NAND_WRITE(SDDR_TIMING_1, Params->NandSDDRTiming1);
            // Enable Sync DDR Interface mode
            NV_NAND_READ(CONFIG2, ConfigReg2);
            //   Sync mode, DQS is to be driven high per the spec 
            ConfigReg2 = NV_FLD_SET_DRF_DEF(
                        NAND, CONFIG2, DRIVE_DQS_DURING_ASYNC_MODE, ENABLE, ConfigReg2);
            ConfigReg2 = NV_FLD_SET_DRF_DEF(
            NAND, CONFIG2, SYNC_DDR, ENABLE, ConfigReg2);
            NV_NAND_WRITE(CONFIG2, ConfigReg2);
        }
        // Extract max asynchronous timing mode supported on device
        s_NandContext->MaxAsyncTimingMode =
        NvBootGetMsbSet(
            pBuffer[NVBOOT_NAND_ONFI_ASYNC_TIMING_MODE_BYTE_OFFSET]);
        // Extract the required params.
        s_NandContext->PageSizeLog2 = NvBootUtilGetLog2Number( 
            pBuffer[NVBOOT_NAND_ONFI_PAGE_SIZE_BYTE0_OFFSET] | 
            (pBuffer[NVBOOT_NAND_ONFI_PAGE_SIZE_BYTE1_OFFSET] << 8) | 
            (pBuffer[NVBOOT_NAND_ONFI_PAGE_SIZE_BYTE2_OFFSET] << 16) | 
            (pBuffer[NVBOOT_NAND_ONFI_PAGE_SIZE_BYTE3_OFFSET] << 24) );
        // AP20 BootRom intend to support pages of size 2KB,4KB and 8KB only.
        if ( (s_NandContext->PageSizeLog2 < NVBOOT_NAND_2KB_PAGE_SIZE_IN_LOG2) || 
             (s_NandContext->PageSizeLog2 > NVBOOT_NAND_8KB_PAGE_SIZE_IN_LOG2) )
        {
            // This will let normal identification overwrite this value.
            s_NandContext->PageSizeLog2 = 0;
            return NvBootError_DeviceUnsupported;
        }
        s_NandContext->PagesPerBlockLog2 = NvBootUtilGetLog2Number( 
            pBuffer[NVBOOT_NAND_ONFI_PAGES_PER_BLOCK_BYTE0_OFFSET] | 
            (pBuffer[NVBOOT_NAND_ONFI_PAGES_PER_BLOCK_BYTE1_OFFSET] << 8) | 
            (pBuffer[NVBOOT_NAND_ONFI_PAGES_PER_BLOCK_BYTE2_OFFSET] << 16) | 
            (pBuffer[NVBOOT_NAND_ONFI_PAGES_PER_BLOCK_BYTE3_OFFSET] << 24) );
        s_NandContext->BlockSizeLog2 = s_NandContext->PagesPerBlockLog2 + 
                                       s_NandContext->PageSizeLog2;
        s_NandContext->NumOfAddressCycles = 
            (pBuffer[NVBOOT_NAND_ONFI_ADDRESS_CYCLES_BYTE_OFFSET] & 0xF) + 
            ((pBuffer[NVBOOT_NAND_ONFI_ADDRESS_CYCLES_BYTE_OFFSET] >> 4) & 0xF);
        s_NandContext->DataWidth = s_FuseInfo.DataWidth;
        if (s_FuseInfo.DataWidth == NvBootNandDataWidth_Discovey)
        {
            s_NandContext->DataWidth = 
            (pBuffer[NVBOOT_NAND_ONFI_BUS_WIDTH_BYTE_OFFSET] & 
            NVBOOT_NAND_ONFI_BUS_WIDTH_MASK) ? NvBootNandDataWidth_16Bit : 
            NvBootNandDataWidth_8Bit;
        }
        s_NandContext->IsOnfi = NV_TRUE;
        return e;
    }
    return NvBootError_DeviceUnsupported;
}

static NvBootError 
NvBootNandPrivReadPage(
    const NvU32 Block, 
    const NvU32 Page, 
    NvU8 *pBuffer)
{
    NvBootError e = NvBootError_None;
    NvBootNandCommand* pCommand = &s_NandContext->Command;
    NV_ASSERT(Page < (1 << s_NandContext->PagesPerBlockLog2));
    NV_ASSERT(pBuffer != NULL);
    
    PROFILE();
    // Set up the command structure.
    pCommand->Command1 = NvBootNandCommands_Read;
    pCommand->Command2 = NvBootNandCommands_ReadStart;
    pCommand->ColumnNumber = 0;
    pCommand->PageNumber = Page;
    pCommand->BlockNumber = Block;
    PRINT_NAND_MESSAGES("\r\n     Reading Block=%d, Page=%d", Block, Page);
    // Setup the Dma
    HwNandSetupDma(pBuffer, (1 << s_NandContext->PageSizeLog2));
    // Send the Read Command.
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadInProgress;
    s_NandContext->ReadStartTime = NvBootUtilGetTimeUS();
    return e;
}

static NvBootError 
HwNandDiscoverEccSelection(
    const NvU32 Block, 
    const NvU32 Page, 
    NvU8 *pBuffer)
{
    NvBootError e;
    NvBootNandEccSelection EccSelection;
    NvBootDeviceStatus Status = NvBootDeviceStatus_ReadInProgress;
    
    PRINT_NAND_MESSAGES("\r\nTrying to Discover Ecc")
    for (EccSelection = NvBootNandEccSelection_Bch4; 
         EccSelection < NvBootNandEccSelection_Off; EccSelection++)
    {
        s_NandContext->EccSelection = EccSelection;
        PRINT_NAND_MESSAGES("\r\nTry to discover with Ecc=%d", EccSelection);
        NV_BOOT_CHECK_ERROR(NvBootNandPrivReadPage(Block, Page, pBuffer));
        do
        {
            // NvBootNandQueryStatus returns timeout error, if the status 
            // doesn't change from NvBootDeviceStatus_ReadInProgress with in the
            // expected time. So, while loop with no time out is okay here.
            Status = NvBootNandQueryStatus();
        } while (Status == NvBootDeviceStatus_ReadInProgress);
        if (Status == NvBootDeviceStatus_EccFailure)
        {
            s_NandBitInfo->NumUncorrectableErrorPages--;
            continue;
        }
        if ( (Status == NvBootDeviceStatus_Idle) || 
             (Status == NvBootDeviceStatus_CorrectedEccFailure) )
            // Found the Ecc.
            break;
        if (Status == NvBootDeviceStatus_ReadFailure)
            // Read timed out.
            return NvBootError_DeviceReadError;
    }
    if (EccSelection == NvBootNandEccSelection_Off)
    {
        s_NandBitInfo->NumUncorrectableErrorPages++;
        return NvBootError_EccDiscoveryFailed;
    }
    return NvBootError_Success;
}

/* Public Function Definitions */
NvBootError 
NvBootNandInit(
    const NvBootNandParams *Params, 
    NvBootNandContext *Context)
{
    NvBootError e = NvBootError_DeviceUnsupported;

    // Validate params first.
    NV_ASSERT(Context != NULL);
    NV_ASSERT(Params != NULL);
    NV_ASSERT( (Params->ClockDivider != 0) && 
        (Params->ClockDivider <= NVBOOT_NAND_MAX_CLOCK_DIVIDER_SUPPORTED) );
    PROFILE();
    PRINT_NAND_MESSAGES("\r\n     NandInit");
    // Store the pointer to the context structure for later reference.
    s_NandContext = Context;
    s_NandContext->NumOfAddressCycles = NVBOOT_NAND_MAX_ADDRESS_CYCLES_SUPPORTED;
    s_NandContext->DataWidth = s_FuseInfo.DataWidth;
    s_NandContext->MainEccOffset = s_FuseInfo.MainEccOffset;
    s_NandContext->BchSectorSize = s_FuseInfo.BchSectorSize;
    s_NandContext->EFNandCtrlVer = s_FuseInfo.EFNandCtrlVer;
    s_NandContext->IsOnfiEZNand = NV_FALSE;
    s_NandContext->IsSupportAutoRetries = NvBootOnfiEZNandAutoRetries_Disable;
    s_NandContext->BlockSizeLog2 = Params->BlockSizeLog2;
    s_NandContext->PageSizeLog2 = Params->PageSizeLog2;
    s_NandContext->IsOnfi = NV_FALSE;
    s_NandContext->DisableSyncDDR = Params->DisableSyncDDR;
    s_NandContext->ToggleDDR= s_FuseInfo.ToggleDDR;
    s_NandContext->IsSyncDDRModeSupported = NV_FALSE;
    s_NandContext->OnfiRevNum = NvBootNandOnfiRevNum_None;
    s_NandContext->IsEnhancedClearNand = NV_FALSE;

    // Initialize the Nand Hw controller.
    HwNandInitializeHwController(Params);
    if (s_FuseInfo.DisableOnfiSupport == NV_FALSE)
        e = HwNandInitializeOnfiFlash(Params);
    // If Onfi initialization fails, try to do the usual initialization.
    if (e != NvBootError_Success)
    {
        NV_BOOT_CHECK_ERROR(HwNandResetNandFlash());
        // EF Nand 2.0/3.0 ReadID structure is not generic so don't call ReadID function for them
        if (!(s_NandContext->EFNandCtrlVer == NvBootEFNand_2 ||
         s_NandContext->EFNandCtrlVer == NvBootEFNand_3))
        {
            NV_BOOT_CHECK_ERROR(HwNandReadId());
        }
    }
    if (s_NandContext->ToggleDDR == NvBootNandToggleDDR_Enable)
    {
        NvU32 ConfigReg2;

        // Set Toggle mode timing registers
        NV_NAND_WRITE(ASYNC_TIMING_0, Params->NandAsyncTiming0);
        NV_NAND_WRITE(ASYNC_TIMING_1, Params->NandAsyncTiming1);
        NV_NAND_WRITE(ASYNC_TIMING_2, Params->NandAsyncTiming2);
        NV_NAND_WRITE(ASYNC_TIMING_3, Params->NandAsyncTiming3);
        NV_NAND_WRITE(TDDR_TIMING_0, Params->NandTDDRTiming0);
        NV_NAND_WRITE(TDDR_TIMING_1, Params->NandTDDRTiming1);
        // Enable Toggle DDR Interface mode
        NV_NAND_READ(CONFIG2, ConfigReg2);
        ConfigReg2 = NV_FLD_SET_DRF_DEF(
           NAND, CONFIG2, TOGGLE_DDR, ENABLE, ConfigReg2);
        NV_NAND_WRITE(CONFIG2, ConfigReg2);
    }
    // Safe trimmer settings required for DDR interfaces (Toggle/ONFI)
    if (s_NandContext->ToggleDDR == NvBootNandToggleDDR_Enable ||
        s_NandContext->IsSyncDDRModeSupported)
    {
        NvU32 RegData;
        NvU32 FbioCfgReg;
        NvU32 FbioQuseDlyReg;
        NvU32 FbioDqsibDlyReg;

        // QUSE LATE settings for DDR interface
        NV_NAND_READ(FBIO_CFG, FbioCfgReg);
        FbioCfgReg = NV_FLD_SET_DRF_NUM(NAND,
            FBIO_CFG, 
            CFG_QUSE_LATE,
            Params->NandFbioCfgQuseLate,
            FbioCfgReg);
        NV_NAND_WRITE(FBIO_CFG, FbioCfgReg);
        // QUSE DLY settings for Toggle DDR interface
        NV_NAND_READ(FBIO_QUSE_DLY, FbioQuseDlyReg);
        FbioQuseDlyReg = NV_FLD_SET_DRF_NUM(NAND,
            FBIO_QUSE_DLY, 
            CFG_QUSE_DLY_BYTE_0,
            Params->NandFbioQuseDlyByte,
            FbioQuseDlyReg);
        NV_NAND_WRITE(FBIO_QUSE_DLY, FbioQuseDlyReg);
        // Program DQS fine tune delay for ONFI DDR timing mode0
        NV_NAND_READ(FBIO_DQSIB_DLY, FbioDqsibDlyReg);
        FbioDqsibDlyReg = NV_FLD_SET_DRF_NUM(NAND,
            FBIO_DQSIB_DLY,
            CFG_DQSIB_DLY_BYTE_0,
            Params->NandFbioDqsibDlyByte,
            FbioDqsibDlyReg);
        NV_NAND_WRITE(FBIO_DQSIB_DLY, FbioDqsibDlyReg);

        // NDFLASH_DIV2_SEL=1 setting is required for DDR modes.
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
            CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                             CLK_SOURCE_NDFLASH,
                             NDFLASH_DIV2_SEL,
                             1,
                             RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + 
            CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0, RegData);
    }
    // EF Nand 2.0/3.0 ReadID structure is not generic so get the device info from ReadDevice
    // Parameter and this needs to be called after toggle ddr settings
    if (s_NandContext->EFNandCtrlVer == NvBootEFNand_2 ||
     s_NandContext->EFNandCtrlVer == NvBootEFNand_3)
    {
        NV_BOOT_CHECK_ERROR(NvBootEFNandReadDeviceParameter());
    }
    // Set pinmuxing again to reflect the data width change, if any.
    HwNandSetPinmux(s_NandContext->DataWidth);
    // update contex structure for EccSelection
    s_NandContext->EccSelection = s_FuseInfo.EccSelection;
    // Set EccSelection as off and don't perform EccDiscovery for EFNand/EZNand devices
    if (s_NandContext->EFNandCtrlVer || s_NandContext->IsOnfiEZNand)
    {
        s_NandContext->EccSelection = NvBootNandEccSelection_Off;
        s_PerformEccDiscovery = NV_FALSE;
    }
    s_NandContext->DeviceStatus = NvBootDeviceStatus_Idle;
    // Fill Nand Bit info.
    s_NandBitInfo->DiscoveredDataWidth = s_NandContext->DataWidth;
    s_NandBitInfo->IsPartOnfi = s_NandContext->IsOnfi;
    return NvBootError_Success;
}

void 
NvBootNandGetParams(
    const NvU32 ParamIndex, 
    NvBootNandParams **Params)
{
    NV_ASSERT(Params != NULL);
    
    PROFILE();
    /*
     * Extract Ecc selection from Param Index, which comes from fuses.
     * Three Fuse bits are used for Ecc Selection. The bits value starting 
     * from zero to four, corresponds to Discovery, Bch4, Bch8, Bch16, Bch24
     * and Ecc Off.
     */
    s_FuseInfo.EccSelection = (NvBootNandEccSelection)NV_DRF_VAL(NAND_DEVICE, 
                                   CONFIG, ECC_SELECT, ParamIndex);
    /*
     * Extract Data width selection from Param Index, which comes from fuses.
     * Two Fuse bits are used for Data width Selection. The bits value starting 
     * from zero to three, corresponds to Discovery, 8-bit, 16-bit and rsvd.
     */
    s_FuseInfo.DataWidth = (NvBootNandDataWidth)NV_DRF_VAL(NAND_DEVICE, CONFIG, 
                                DATA_WIDTH, ParamIndex);
    /*
     * Extract Onfi Support option from Param Index, which comes from fuses.
     * One Fuse bit is used for Disabling Onfi. The bit values 0 and 1 indicate
     * Onfi Support Enable and Disable respectively.
     */
    s_FuseInfo.DisableOnfiSupport = NV_DRF_VAL(NAND_DEVICE, CONFIG, 
                                        DISABLE_ONFI, ParamIndex);
    /*
     * Extract Page/Block size offset to use from Param Index, which comes from fuses.
     * One Fuse bit is used for Page/Block size offset. The bits value starting 
     * from zero to one, corresponds to offsets zero to one.
     */
    s_FuseInfo.PageBlockSizeOffset = NV_DRF_VAL(NAND_DEVICE, CONFIG, 
                                    PAGE_BLOCK_SIZE_OFFSET, ParamIndex);
    /*
     * Extract Programmable offset for Main data ecc bytes in spare.for different Nand parts, 
     * which comes from fuses.Two fuse bits are used to select Main ECC offset. The bit value 
     * starting from zero to three, corresponds to offset 4, 8, 12 and 16 bytes respectively.
     */
    s_FuseInfo.MainEccOffset = NV_DRF_VAL(NAND_DEVICE, CONFIG, 
                                       MAIN_ECC_OFFSET, ParamIndex);
    /*
     * Extract BCH sector size, which comes from fuses.
     * One Fuse bit is used to select the sector size for ECC computation.The bit values 0 and
     *  1 indicate 512 and 1024 bytes.
     */
    s_FuseInfo.BchSectorSize = NV_DRF_VAL(NAND_DEVICE, CONFIG,
                                       BCH_SECTOR_SIZE, ParamIndex);
    /*
     * Extract EF NAND Controller version from Param Index, which comes from fuses.
     * Two Fuse bits used to select the controller version.The bit values starting from zero to four
     * corresponds to DisableEF, EF NAND 1.0, EF NAND 2.0 and EF NAND 3.0 respectively.
     */
    s_FuseInfo.EFNandCtrlVer = (NvBootEFNandCtrlVer)NV_DRF_VAL(NAND_DEVICE, CONFIG,
                                       EF_NAND_CTRL_VER, ParamIndex);
    /*
     * Extract ToggleDDR mode, which comes from fuses.
     * Twoe Fuse bits used to select the toggle ddr mode. The bit values starting 
     * from zero to four, corresponds to discovery, disable, enable & rsvd.
     */
    s_FuseInfo.ToggleDDR = (NvBootNandToggleDDR)NV_DRF_VAL(NAND_DEVICE, CONFIG,
                                       TOGGLE_DDR, ParamIndex);
    /*
     * Set the Nand clock source frequency as 24MHz before BCT is read.
     * The Clock Source to Nand is PLLP and which is operating at 216MHz.
     * To get 24MHz to Nand controller, divide it by 9. 216/9 = 24MHz.
     */
    s_DefaultNandParams.ClockDivider = 9;
    /*
     * Disable Synchronous DDR mode before BCT is read. 
     * This mode can be enabled after reading the BCT if Nand parts supports it.
     */
    s_DefaultNandParams.DisableSyncDDR = NV_TRUE;

    s_DefaultNandParams.NandAsyncTiming0 =
        NV_DRF_NUM(NAND, ASYNC_TIMING_0, TAR,   10) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_0, TCR,   0) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_0, TRR,   0) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_0, TWHR,   10) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_0, TWB,   10); 
    
    s_DefaultNandParams.NandAsyncTiming1 =
        NV_DRF_NUM(NAND, ASYNC_TIMING_1, TRHW,  5) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_1, TWHR2, 10) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_1, TCS,   1) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_1, TADL,  10);

    s_DefaultNandParams.NandAsyncTiming2 =
        NV_DRF_NUM(NAND, ASYNC_TIMING_2, TCWAW, 10) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_2, TCH,   0) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_2, TCLH,  0) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_2, TCLS,  0) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_2, TALH,  0) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_2, TALS,  0);

    s_DefaultNandParams.NandAsyncTiming3 =
        NV_DRF_NUM(NAND, ASYNC_TIMING_3, TREA, 1) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_3, TWH,   1) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_3, TWP,   1) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_3, TREH,  1) |
        NV_DRF_NUM(NAND, ASYNC_TIMING_3, TRP,    1);

    s_DefaultNandParams.NandSDDRTiming0 =
        NV_DRF_NUM(NAND, SDDR_TIMING_0, TCKWR,  2) |
        NV_DRF_NUM(NAND, SDDR_TIMING_0, TCAD,   2) |
        NV_DRF_NUM(NAND, SDDR_TIMING_0, TWRCK,  0) |
        NV_DRF_NUM(NAND, SDDR_TIMING_0, TWPST,  0) |
        NV_DRF_NUM(NAND, SDDR_TIMING_0, TWPRE,  0);

    s_DefaultNandParams.NandSDDRTiming1 =
        NV_DRF_NUM(NAND, SDDR_TIMING_1, TCCS,   10);

    s_DefaultNandParams.NandTDDRTiming0 =
        NV_DRF_NUM(NAND, TDDR_TIMING_0, TRPSTH, 1) |
        NV_DRF_NUM(NAND, TDDR_TIMING_0, TRPST,  1) |
        NV_DRF_NUM(NAND, TDDR_TIMING_0, TRPRE,  1);

    s_DefaultNandParams.NandTDDRTiming1 =
        NV_DRF_NUM(NAND, TDDR_TIMING_1, TCDQSH, 0) |
        NV_DRF_NUM(NAND, TDDR_TIMING_1, TCDQSS, 4) |
        NV_DRF_NUM(NAND, TDDR_TIMING_1, TWPSTH, 1) |
        NV_DRF_NUM(NAND, TDDR_TIMING_1, TWPST,  1) |
        NV_DRF_NUM(NAND, TDDR_TIMING_1, TWPRE,  0);

    // FBIO DQSIB DLY register default setting for ONFI DDR interface
    s_DefaultNandParams.NandFbioDqsibDlyByte = NVBOOT_NAND_FBIO_DQSIB_DLY_MAX;

    // FBIO QUSE DLY register default setting for Toggle DDR interface
    s_DefaultNandParams.NandFbioQuseDlyByte = 0;

    // FBIO config register default setting for DDR interfaces
    s_DefaultNandParams.NandFbioCfgQuseLate = 0;

    // Set the default block size to zero. This lets read id to overwrite 
    // block size value.
    s_DefaultNandParams.BlockSizeLog2 = 0;
    // Set the default page size to zero. This lets read id to overwrite
    // page size value.
    s_DefaultNandParams.PageSizeLog2 = 0;
    
    *Params = (NvBootNandParams*)&s_DefaultNandParams;
    /*
     * NvBootNandGetParams() will never be called more than once by bootrom. 
     * Setting this flag enables Ecc discovery process, if fuse option is 
     * discovery. This function will be called by test code multiple times for 
     * Ecc discovery testing.
     */
    s_PerformEccDiscovery = NV_TRUE;
    // Fill Bit info.
    s_NandBitInfo->FuseDataWidth = s_FuseInfo.DataWidth;
    s_NandBitInfo->FuseDisableOnfiSupport = s_FuseInfo.DisableOnfiSupport;
    s_NandBitInfo->FuseEccSelection = s_FuseInfo.EccSelection;
    s_NandBitInfo->FusePageBlockSizeOffset = s_FuseInfo.PageBlockSizeOffset;
    PRINT_NAND_MESSAGES("\r\nParamIndex=0x%x, EccSelecti"
        "on=%d(0->Disc, 1->Bch4, 2->Bch8, 3->Bch16, 4->Bch24, 5->Ecc Off),"
        "DataWidth=%d(0->Disc, 1->8bit, 2->16bit), DisableOnfiSupport=%d,"
        "PageBlockSizeOffset=%d,"
        "ClockDivider=%d",
        ParamIndex, s_FuseInfo.EccSelection, 
        s_FuseInfo.DataWidth, s_FuseInfo.DisableOnfiSupport,
        s_FuseInfo.PageBlockSizeOffset, s_FuseInfo.PageBlockSizeOffset, 
        s_DefaultNandParams.ClockDivider);
}

NvBool 
NvBootNandValidateParams(
    const NvBootNandParams *Params)
{
    PROFILE();
    PRINT_NAND_MESSAGES("\r\nValidateParams , EccSelecti"
        "on=%d(0->Disc, 1->Bch4, 2->Bch8, 3->Bch16, 4->Bch24, 5->Ecc Off),"
        "DataWidth=%d(0->Disc, 1->8bit, 2->16bit), DisableOnfiSupport=%d,"
        "PageBlockSizeOffset=%d,"
        "ClockDivider=%d",
        s_FuseInfo.EccSelection,
        s_FuseInfo.DataWidth, s_FuseInfo.DisableOnfiSupport,
        s_FuseInfo.PageBlockSizeOffset, s_FuseInfo.PageBlockSizeOffset,
        s_DefaultNandParams.ClockDivider);
    
    // Validate Params.
    if (s_FuseInfo.EccSelection >= NvBootNandEccSelection_Num)
        return NV_FALSE;
    if (s_FuseInfo.DataWidth >= NvBootNandDataWidth_Num)
        return NV_FALSE;
    if ( (Params->ClockDivider < NVBOOT_NAND_MIN_CLOCK_DIVIDER_SUPPORTED) || 
         (Params->ClockDivider > NVBOOT_NAND_MAX_CLOCK_DIVIDER_SUPPORTED) )
        return NV_FALSE;
    return NV_TRUE;
}

void 
NvBootNandGetBlockSizes(
    const NvBootNandParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);
    NV_ASSERT(s_NandContext != NULL);
    
    *BlockSizeLog2 = s_NandContext->BlockSizeLog2;
    *PageSizeLog2 = s_NandContext->PageSizeLog2;
    PRINT_NAND_MESSAGES("\r\nBlockSize=%d, PageSize=%d, PagesPerBlock=%d", 
        (1 << s_NandContext->BlockSizeLog2),(1 << s_NandContext->PageSizeLog2),
        (1 << s_NandContext->PagesPerBlockLog2));
}

NvBootError 
NvBootNandReadPage(
    const NvU32 Block, 
    const NvU32 Page, 
    NvU8 *pBuffer)
{
    NvBootError e;
    static NvBootNandEccSelection DiscoveredEcc = NvBootNandEccSelection_Bch4;
    NvBootDeviceStatus Status = NvBootDeviceStatus_ReadInProgress;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    NvU32 StatusRegValue;


    s_NandBitInfo->NumPagesRead++;
    if ( (s_FuseInfo.EccSelection == NvBootNandEccSelection_Discovery) &&
         (s_PerformEccDiscovery == NV_TRUE) )
    {
        // The fuse option is discovery and not yet discovered valid Ecc.
        e = HwNandDiscoverEccSelection(Block, Page, pBuffer);
        if (e == NvBootError_Success)
        {
            s_PerformEccDiscovery = NV_FALSE;
            DiscoveredEcc = s_NandContext->EccSelection;
            s_NandBitInfo->DiscoveredEccSelection = s_NandContext->EccSelection;
            PRINT_NAND_MESSAGES("\r\nDiscoveredEcc=%d(0->Disc, 1->Bch4,"
                "2->Bch8, 3->Bch16, 4->Bch24)", DiscoveredEcc);
        }
        else if (e == NvBootError_EccDiscoveryFailed)
        {
            // Discovery failed. i.e Read Ecc faliure error occured for all 
            // supported Ecc's. The pBuffer should be having the data already,
            // which is last read during discovery. So, return from here.
            PRINT_NAND_MESSAGES("\r\nEcc Discovery Failed");
            // We can return NvBootError_Success here NvBootNandQueryStatus() 
            // would return Ecc Failure.
            e = NvBootError_Success;
        }
        return e;
    }
    else if ( (s_FuseInfo.EccSelection == NvBootNandEccSelection_Discovery) &&
         (s_PerformEccDiscovery == NV_FALSE) )
    {
        // The fuse option is discovery and already discovered valid Ecc.
        s_NandContext->EccSelection = DiscoveredEcc;
    }
    else
    {
        // The fuse option is not discovery. So, use what ever is specified by 
        // fuses.
        s_NandContext->EccSelection = s_FuseInfo.EccSelection;
        s_NandBitInfo->DiscoveredEccSelection = s_NandContext->EccSelection;
    }
    if (s_NandContext->IsEnhancedClearNand)
    {
        NV_BOOT_CHECK_ERROR(HwNandDeviceStatus());
        // Read device status
        NV_NAND_READ(RESP, StatusRegValue);
        // wait untill device is ready to accept read operation .
        while (TimeOutCounter)
        {
            if (StatusRegValue & 0x2)
                break;
             NvBootUtilWaitUS(1);
            TimeOutCounter--;
            if (!TimeOutCounter)
                return NvBootError_HwTimeOut;
      }
    }
    NV_BOOT_CHECK_ERROR(NvBootNandPrivReadPage(Block, Page, pBuffer));
    do
    {
        // NvBootNandQueryStatus returns timeout error, if the status 
        // doesn't change from NvBootDeviceStatus_ReadInProgress with in the
        // expected time. So, while loop with no time out is okay here.
        Status = NvBootNandQueryStatus();
    } while (Status == NvBootDeviceStatus_ReadInProgress);
    if (Status == NvBootDeviceStatus_EccFailure)
    {
        return NvBootError_EccFailureUncorrected;
    }
    if (Status == NvBootDeviceStatus_ReadFailure)
        return NvBootError_DeviceReadError;
    if (s_NandContext->IsEnhancedClearNand)
    {
        NV_BOOT_CHECK_ERROR(HwNandReadMode());
        NV_BOOT_CHECK_ERROR(HwNandDeviceStatus());
        // Read device status
        NV_NAND_READ(RESP, StatusRegValue);
        // Check IO bit 1 for ready to accept read operation.
        if (StatusRegValue & 0x2)
            return NvBootError_DeviceReadError;
    }
    // Read is a success. If the decyrption address is to external memory, make
    // sure the writes to memory are coherent. 
    if (NvBootAhbCheckIsExtMemAddr((NvU32 *)pBuffer)) 
    {
        NvBootAhbWaitCoherency(ARAHB_MST_ID_NAND);
    }
    
    return e;
}

NvBootDeviceStatus NvBootNandQueryStatus(void)
{
    NvU32 DmaGo;
    NvU32 ErrorCount;
    NvU32 IsrRegValue;
    NvU32 DecodeStatusBuf;
    NvU32 DmaMasterControl;
    NvU32 IsEccDecodeFailed;
    NvU32 UncorrectableSectors;
    NvU32 StatusRegValue;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    NvBootError e;

    const NvU32 BchErrorTriggerValueArray[] = 
    {
        NAND_BCH_CONFIG_0_ERR_ATTN_LVL_DATA_CORR_ERRS_0,
        NAND_BCH_CONFIG_0_ERR_ATTN_LVL_DATA_CORR_ERRS_4,
        NAND_BCH_CONFIG_0_ERR_ATTN_LVL_DATA_CORR_ERRS_8,
        NAND_BCH_CONFIG_0_ERR_ATTN_LVL_DATA_CORR_ERRS_16,
        NAND_BCH_CONFIG_0_ERR_ATTN_LVL_DATA_CORR_ERRS_24,
        NAND_BCH_CONFIG_0_ERR_ATTN_LVL_DATA_CORR_ERRS_0
    };
    
    NV_ASSERT(s_NandContext->EccSelection != NvBootNandEccSelection_Discovery);
    
    if (s_NandContext->DeviceStatus == NvBootDeviceStatus_ReadInProgress)
    {
        // Check whether the read is completed.
        NV_NAND_READ(DMA_MST_CTRL, DmaMasterControl);
        DmaGo = NV_DRF_VAL(NAND, DMA_MST_CTRL, DMA_GO, DmaMasterControl);
        // DmaGo = 1 indicates that Dma is still busy.
        // DmaGo = 0 indicates that Dma is done. i.e. reading is done.

        if (DmaGo == 0)
        {
            s_NandContext->DeviceStatus = NvBootDeviceStatus_Idle;
            if (s_NandContext->IsEnhancedClearNand)
            {
                  e = HwNandDeviceStatus();
                  if ( e != NvBootError_Success)
                      s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
                  // Read device status
                  NV_NAND_READ(RESP, StatusRegValue);
                  // wait untill device operation status is available (check for IO bit 2).
                  while (TimeOutCounter)
                  {
                      if (StatusRegValue & 0x4)
                          break;
                       NvBootUtilWaitUS(1);
                      TimeOutCounter--;
                      if (!TimeOutCounter)
                      {
                          return NvBootDeviceStatus_ReadFailure;
                          PRINT_NAND_ERRORS("\r\n*****QueryStatus Timed out");
                      }
                }
                // Send GetNextOpetration Satus command
                e= HwNandGetNextOperationStatus();
                if ( e != NvBootError_Success)
                    s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
                // Send Opetration Satus command
                e = HwNandGetOperationStatus();
                if ( e != NvBootError_Success)
                    s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
                // Read Operation status
                NV_NAND_READ(RESP, StatusRegValue);
                // Check I/O bit 0 for un-correctable ecc errors
                if (StatusRegValue & 0x1)
                {
                    s_NandContext->DeviceStatus = NvBootDeviceStatus_EccFailure;
                    s_NandBitInfo->NumUncorrectableErrorPages++;
                }
                // Check I/O bit 1/2/3 for Refresh block/Retire block/Erased page errors 
                //These errors may occur without ECC failure also
                else if((StatusRegValue & 0x2) || (StatusRegValue & 0x4) ||
                    (StatusRegValue & 0x8))
                    s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
            }
            // Find any uncorrectable errors for Error Free Nand 1.0
            if (s_NandContext->EFNandCtrlVer  == NvBootEFNand_1)
            {
                // Send Read Stautus command 
                e = HwNandReadPageStatus();
                if ( e != NvBootError_Success)
                    s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
                NV_NAND_READ(RESP, StatusRegValue);
                // check I/O bit 0 for uncorrectable errors
                if (StatusRegValue & 0x01)
                {
                    PRINT_NAND_ERRORS("\r\n*****Ecc Failure");
                    PRINT_NAND_MESSAGES("\r\nStatus changed to EccFailure ");
                    s_NandContext->DeviceStatus = NvBootDeviceStatus_EccFailure;
                    s_NandBitInfo->NumUncorrectableErrorPages++;
                }
            }
            else
            {
                // Check whether ECC decode failed.
                NV_NAND_READ(ISR, IsrRegValue);
                IsEccDecodeFailed = NV_DRF_VAL(NAND, ISR, CORRFAIL_ERR, IsrRegValue);
                if (IsEccDecodeFailed)
                {
                    NV_ASSERT(s_NandContext->EccSelection != NvBootNandEccSelection_Off);
                    NV_NAND_READ(BCH_DEC_STATUS_BUF, DecodeStatusBuf);
                    PRINT_NAND_MESSAGES("\r\nDecodeStatusBuf=0x%8.8x", DecodeStatusBuf);
                    UncorrectableSectors = NV_DRF_VAL(NAND, BCH_DEC_STATUS_BUF,
                                            FAIL_SEC_FLAG, DecodeStatusBuf);
                    ErrorCount = NV_DRF_VAL(NAND, BCH_DEC_STATUS_BUF, MAX_CORR_CNT, 
                                    DecodeStatusBuf);
                    if (UncorrectableSectors)
                    {
                        PRINT_NAND_ERRORS("\r\n*****Ecc Failure");
                        PRINT_NAND_MESSAGES("\r\nStatus changed to EccFailure");
                        s_NandContext->DeviceStatus = NvBootDeviceStatus_EccFailure;
                        s_NandBitInfo->NumUncorrectableErrorPages++;
                    }
                    else if (ErrorCount == 
                             BchErrorTriggerValueArray[s_NandContext->EccSelection])
                    {
                        PRINT_NAND_MESSAGES("\r\nStatus changed to CorrectedRead"
                            "Failure");
                        s_NandContext->DeviceStatus = 
                                            NvBootDeviceStatus_CorrectedEccFailure;
                    }
                    if ( (UncorrectableSectors == 0) && ErrorCount)
                    {
                        s_NandBitInfo->NumCorrectableErrorPages++;
                        if (ErrorCount > s_NandBitInfo->MaxCorrectableErrorsEncountered)
                            s_NandBitInfo->MaxCorrectableErrorsEncountered = ErrorCount;
                    }
                }
                // Clear the status.
                NV_NAND_WRITE(ISR, IsrRegValue);
            }
        }
        // Check for TimeOut.
        else if (NvBootUtilElapsedTimeUS(s_NandContext->ReadStartTime) > 
                 NVBOOT_NAND_READ_TIMEOUT_IN_US)
        {
            s_NandContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
            PRINT_NAND_ERRORS("\r\n*****QueryStatus Timed out");
        }
    }
    return s_NandContext->DeviceStatus;
}

void NvBootNandShutdown(void)
{
    // Keep the controller in Reset and disable the clock.
    NvBootResetSetEnable(NvBootResetDeviceId_NandId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_NandId, NV_FALSE);
}

NvBootError NvBootNandGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
