/*
 * Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_mobile_lba_nand_local.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arclk_rst.h"
#include "arnandflash.h"
#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_codecov_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_mobile_lba_nand_context.h"
#include "nvboot_mobile_lba_nand_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "project.h"

// TODO: Remove this conditional compilation after a T35 equivalent for this
// functionality has been implemented.
#if INCLUDE_HEADERS_FPGA_NET1
#define NAND_FIFO_CTRL_0  (0x3c)
#define NAND_FIFO_CTRL_0_LL_BUF_CLR_RANGE 3:3
#define NAND_FIFO_CTRL_0_LL_BUF_CLR_CLEAR_ALL_FIFO 1
#define NAND_CONFIG_0_LPDDR1_MODE_RANGE 20:20
#define NAND_DMA_MST_CTRL_0_DMA_PERF_EN_RANGE 29:29
#define NAND_DMA_MST_CTRL_0_DMA_PERF_EN_ENABLE 1
#endif

#define DEBUG_NAND 0

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
 * params of this struct, which are specific to MobileLbaNand flash present 
 * on board, will be changed based on the fuse settings passed to the 
 * API NvBootMobileLbaNandGetParams(). See NvBootMobileLbaNandGetParams() 
 * for more info on what gets changed based on fuse value.
 */
static NvBootMobileLbaNandParams s_DefaultMlnParams;
// Pointer to store the context memory address.
static NvBootMobileLbaNandContext *s_MlnContext = NULL;
// Boot Info table.
extern NvBootInfoTable BootInfoTable;
// Pointer to MLba Nand Bit info.
static NvBootMobileLbaNandStatus* s_MlnBitInfo = 
                                    &BootInfoTable.SecondaryDevStatus.MLbaStatus;

// This struct holds the Info from fuses.
typedef struct
{
    // Selected Data width.
    NvBootMlnDataWidth DataWidth;
    // Number of address cycles.
    NvU8 NumOfAddressCycles;
    // Region to read from.
    NvBool ReadBctFromSda;
    // Pinmux selection.
    NvU8 PinmuxSelection;
    // Pin Order selection.
    NvU8 PinOrder;
} NvBootMlnFuseInfo;

static NvBootMlnFuseInfo s_FuseInfo = {NvBootMlnDataWidth_Discovey, 7, NV_TRUE, 0};

/* Private functions forward declaration.*/
/* Private Function Definitions */

static void HwNandSetPinmux(NvBootMlnDataWidth DataWidth)
{
    NvU32 PinmuxSelection;
    NV_ASSERT(s_FuseInfo.PinmuxSelection <= 1);
    NV_ASSERT(DataWidth < NvBootMlnDataWidth_Num);
    
    if (s_FuseInfo.PinmuxSelection)
    {
        PRINT_NAND_MESSAGES("\r\nSetting Pinmux for Alt");
        PinmuxSelection = NvBootPinmuxConfig_Nand_Alt;
    }
    else if (DataWidth == NvBootMlnDataWidth_16Bit)
    {
        PRINT_NAND_MESSAGES("\r\nSetting Pinmux for 16-bit");
        PinmuxSelection = NvBootPinmuxConfig_Nand_Std_x16;
    }
    else
    {
        PRINT_NAND_MESSAGES("\r\nSetting Pinmux for 8-bit");
        PinmuxSelection = NvBootPinmuxConfig_Nand_Std_x8;
    }
    (void)NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_NandFlash, 
        PinmuxSelection);
}

static void
HwNandInitializeHwController(
    const NvBootMobileLbaNandParams *Params)
{
    NvU32 FifoControl;
    
    // Set pinmuxing.
    HwNandSetPinmux(s_MlnContext->DataWidth);
    // Reset the controller.
    NvBootResetSetEnable(NvBootResetDeviceId_NandId, NV_TRUE);
    // Configure the clock source.
    NvBootClocksConfigureClock(NvBootClocksClockId_NandId, 
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(Params->ClockDivider, 0), 
        CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0_NDFLASH_CLK_SRC_PLLP_OUT0);
    // Enable the clock.
    NvBootClocksSetEnable(NvBootClocksClockId_NandId, NV_TRUE);
    // Remove the controller from Reset.
    NvBootResetSetEnable(NvBootResetDeviceId_NandId, NV_FALSE);
    // Clear all FIFO's.
    FifoControl = NV_DRF_DEF(NAND, FIFO_CTRL, LL_BUF_CLR, CLEAR_ALL_FIFO);
    NV_NAND_WRITE(FIFO_CTRL, FifoControl);

    // Set the Nand timming registers.
    NV_NAND_WRITE(TIMING, Params->NandTiming);
    NV_NAND_WRITE(TIMING2, Params->NandTiming2);
}

static NvBootError HwNandWaitCommandDone(void)
{
    NvU32 CommandGo;
    NvU32 CommandReg;
    NvU32 TimeOutCounter = NVBOOT_NAND_COMMAND_TIMEOUT_IN_US;
    
    // Wait for Command to be sent out with time out.
    while (TimeOutCounter)
    {
        NV_NAND_READ(COMMAND, CommandReg);
        CommandGo = NV_DRF_VAL(NAND, COMMAND, GO, CommandReg);
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

static void HwNandSetupAddressRegisters(NvBootMlnCommand* pCommand)
{
    NvU32 AddressReg1 = 0;
    NvU32 AddressReg2 = 0;
    
    // Setup Address Registers
    AddressReg1 = pCommand->SectorCount | 
                  ((pCommand->SectorAddress & NVBOOT_NAND_ADDR_REG_1_PAGE_MASK) << 
                    NVBOOT_NAND_ADDR_REG_1_PAGE_OFFSET);
    AddressReg2 = (pCommand->SectorAddress >> NVBOOT_NAND_ADDR_REG_2_PAGE_OFFSET);
    NV_NAND_WRITE(ADDR_REG1, AddressReg1);
    NV_NAND_WRITE(ADDR_REG2, AddressReg2);
}

static NvBootError HwNandSendCommand(NvBootMlnCommand* pCommand)
{
    NvBootError e;
    NvU32 ConfigReg;
    NvU32 IsrRegValue;
    NvU32 CommandReg = 0;
    
    PRINT_NAND_MESSAGES("\r\n     Sending Cmd1=0x%x,Cmd2=0x%x, Sc=0x%x, Sa=0x%x", 
        pCommand->Command1, pCommand->Command2, pCommand->SectorCount, 
        pCommand->SectorAddress);
    
    // Read and write Isr Register to clear previously set events.
    NV_NAND_READ(ISR, IsrRegValue);
    NV_NAND_WRITE(ISR, IsrRegValue);
    // Setup command registers with commands.
    NV_NAND_WRITE(CMD_REG1, pCommand->Command1);
    NV_NAND_WRITE(CMD_REG2, pCommand->Command2);
    // Setup the address registers.
    HwNandSetupAddressRegisters(pCommand);
    // Setting up the command register.
    // Enable Command cycle.
    // Enable Address cycle.
    // Enable Rx data.
    CommandReg = NV_DRF_DEF(NAND, COMMAND, CLE, ENABLE) |
                 NV_DRF_DEF(NAND, COMMAND, ALE, ENABLE) |
                 NV_DRF_DEF(NAND, COMMAND, RX, ENABLE);
    switch (pCommand->Command1)
    {
        case NvBootMlnCommands_Reset:
            CommandReg = NV_DRF_DEF(NAND, COMMAND, CLE, ENABLE);
            break;
        case NvBootMlnCommands_ReadId:
            // Enable PIO mode.
            // Set the bytes to receive as 4.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES4);
            break;
        case NvBootMlnCommands_ReadPio4Bytes:
            // Enable PIO mode.
            // Set the bytes to receive as 4.
            CommandReg = NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES4)| 
                         NV_DRF_DEF(NAND, COMMAND, RX, ENABLE);
            break;
        case NvBootMlnCommands_ReadPio1Byte:
            // Enable PIO mode.
            // Set the bytes to receive as 1.
            CommandReg = NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES1)| 
                         NV_DRF_DEF(NAND, COMMAND, RX, ENABLE);
            break;
        case NvBootMlnCommands_ReadMda:
        case NvBootMlnCommands_ReadSda:
            // Enable Secondary command.
            // Set ALE bytes.
            // Enable main data receive.
            // Set the bytes to receive as page size of nand.
            CommandReg |= NV_DRF_DEF(NAND, COMMAND, SEC_CMD, ENABLE) |
                          NV_DRF_NUM(NAND, COMMAND, ALE_BYTE_SIZE, 
                            (s_MlnContext->NumOfAddressCycles - 1)) |
                          NV_DRF_DEF(NAND, COMMAND, A_VALID, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, B_VALID, ENABLE) |
                          NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES_PAGE_SIZE_SEL);
            break;
        case NvBootMlnCommands_StatusRead2:
            // Enable PIO mode.
            // Set the bytes to receive as 1.
            CommandReg = NV_DRF_DEF(NAND, COMMAND, CLE, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, PIO, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, TRANS_SIZE, BYTES1)| 
                         NV_DRF_DEF(NAND, COMMAND, RX, ENABLE);
            break;
        default:
            break;
    }
    switch (pCommand->Command2)
    {
        case NvBootMlnCommands_SetTransferProtocol:
            CommandReg = NV_DRF_DEF(NAND, COMMAND, SEC_CMD, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, CLE, ENABLE) |
                         NV_DRF_DEF(NAND, COMMAND, ALE, ENABLE) |
                         NV_DRF_NUM(NAND, COMMAND, ALE_BYTE_SIZE, 
                            (s_MlnContext->NumOfAddressCycles - 1));
            break;
        default:
            break;
    }
    
    // Set pin ordering. This should be done before enabling go in command reg.
    NV_ASSERT( (s_FuseInfo.PinOrder == 0) || (s_FuseInfo.PinOrder == 1) );
    NV_NAND_READ(CONFIG, ConfigReg);

    ConfigReg |= NV_DRF_NUM(NAND, CONFIG, LPDDR1_MODE, s_FuseInfo.PinOrder);

    NV_NAND_WRITE(CONFIG, ConfigReg);
    // Enable chip select.
    // Send the command out.
    CommandReg |= NV_DRF_DEF(NAND, COMMAND, CE0, ENABLE) |
                  NV_DRF_DEF(NAND, COMMAND, GO, ENABLE);
    NV_NAND_WRITE(COMMAND, CommandReg);
    // Wait till command done and chip ready.
    NV_BOOT_CHECK_ERROR(HwNandWaitCommandDone());
    NV_BOOT_CHECK_ERROR(HwNandWaitChipReady());
    return e;
}

static NvBootError MlnReset(void)
{
    NvBootError e;
    NvBootMlnCommand* pCommand = &s_MlnContext->Command;
    
    pCommand->Command1 = NvBootMlnCommands_Reset;
    pCommand->Command2 = NvBootMlnCommands_CommandNone;
    e = HwNandSendCommand(pCommand);
    return e;
}

static NvBootError MlnReadId(void)
{
    NvU32 Id;
    NvBootError e;
    NvU32 TempData;
    NvBootMlnCommand* pCommand = &s_MlnContext->Command;
    
    // The data width must be x8 during read id as per sepc.
    pCommand->Command1 = NvBootMlnCommands_ReadId;
    pCommand->Command2 = NvBootMlnCommands_CommandNone;
    pCommand->SectorAddress = 0;
    pCommand->SectorCount = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    NV_NAND_READ(RESP, Id);
    s_MlnBitInfo->IdRead = Id;
    // Extract the MobileLbaNand params from id data.
    s_MlnContext->MakerCode = NV_DRF_VAL(ML_NAND_DEVICE, READID, MAKER_CODE, Id);
    s_MlnContext->DeviceCode = NV_DRF_VAL(ML_NAND_DEVICE, READID, DEVICE_CODE, Id);
    s_MlnContext->PageSizeLog2 = NVBOOT_ML_NAND_PAGE_SIZE_LOG2;
    s_MlnContext->BlockSizeLog2 = NVBOOT_ML_NAND_BLOCK_SIZE_LOG2;
    s_MlnContext->PagesPerBlockLog2 = NVBOOT_ML_NAND_PAGES_PER_BLOCK_LOG2;
    s_MlnContext->DataWidth = s_FuseInfo.DataWidth;
    if (s_FuseInfo.DataWidth == NvBootMlnDataWidth_Discovey)
    {
        // Extract bus width. Valid bus width enum starts from value '1'. So, 
        // add '1'.
        TempData = NV_DRF_VAL(ML_NAND_DEVICE, READID, BUS_WIDTH, Id);
        s_MlnContext->DataWidth = (NvBootMlnDataWidth)(TempData + 1);
    }
    PRINT_NAND_MESSAGES("\r\nResponse=0x%x", Id);
    PRINT_NAND_MESSAGES("\r\nDataWidth=%d(0->Disc, 1->8bit, 2->16bit)", 
        s_MlnContext->DataWidth);
    return e;
}

static NvBootError MlnReadStatus(void)
{
    NvBootError e;
    NvU32 DataRead;
    NvBootMlnCommand* pCommand = &s_MlnContext->Command;
    
    // Check the status.
    pCommand->SectorAddress = 0;
    pCommand->Command1 = NvBootMlnCommands_StatusRead2;
    pCommand->Command2 = NvBootMlnCommands_CommandNone;
    pCommand->SectorCount = 0;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    NV_NAND_READ(RESP, DataRead);
    DataRead &= 0xFF;
    PRINT_NAND_MESSAGES("\r\nStatusRead2=0x%x", DataRead);
    return e;
}

static NvBootError 
MlnSendTransferProtocolCommand(
    NvU16 Address, 
    NvBool CheckStatus)
{
    NvBootError e;
    NvBootMlnCommand* pCommand = &s_MlnContext->Command;
    
    pCommand->SectorAddress = 0;
    pCommand->Command1 = NvBootMlnCommands_CommandNone;
    pCommand->Command2 = NvBootMlnCommands_SetTransferProtocol;
    pCommand->SectorCount = Address;
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    if (CheckStatus == NV_TRUE)
        NV_BOOT_CHECK_ERROR(MlnReadStatus());
    return e;
}

static NvU32 MlnReadTransferProtocolData(NvU8 NumBytes)
{
    NvU32 RxData;
    NvBootError e;
    NvBootMlnCommand* pCommand = &s_MlnContext->Command;
    
    NV_ASSERT( (NumBytes == 1) || (NumBytes == 4) );
    // Read data.
    pCommand->SectorAddress = 0;
    pCommand->Command1 = ( (NumBytes == 1) ? NvBootMlnCommands_ReadPio1Byte :
                           NvBootMlnCommands_ReadPio4Bytes );
    pCommand->Command2 = NvBootMlnCommands_CommandNone;
    pCommand->SectorCount = 0;
    NV_BOOT_CHECK_ERROR_CLEANUP(HwNandSendCommand(pCommand));
    NV_NAND_READ(RESP, RxData);
    RxData &= ( (NumBytes == 1) ? 0xFF : -1 );
    return RxData;
fail:
    return 0;
}

static NvU32 MlnGetSdaSectorCount(void)
{
    NvBootError e;
    NvU16 Address;
    NvU32 SectorCount;
    
    // Get Sda Unit size.
    Address = NvBootMlnCommands_GetSdaUnitAddress;
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_FALSE));
    // Read data.
    SectorCount = MlnReadTransferProtocolData(4);
    PRINT_NAND_MESSAGES("\r\nSdaSectorCount=0x%x", SectorCount);
    return SectorCount;
}

static NvU32 MlnGetMdaSectorCount(void)
{
    NvBootError e;
    NvU16 Address;
    NvU32 SectorCount;
    
    // Get Mda Unit size.
    Address = NvBootMlnCommands_GetMdaUnitAddress;
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_FALSE));
    // Read data.
    SectorCount = MlnReadTransferProtocolData(4);
    PRINT_NAND_MESSAGES("\r\nMdaSectorCount=0x%x", SectorCount);
    return SectorCount;
}

static NvBootError MlnSetMinimumBusyTime(void)
{
    NvBootError e;
    NvU8 BusyTime;
    NvU16 Address;
    
    // Set minimum busy time.
    Address = NvBootMlnCommands_SetMinimumBusyTimeAddress | 
                  (NVBOOT_ML_NAND_MINIMUM_BUSY_TIME << 8);
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_TRUE));
    // Validate it back.
    Address = NvBootMlnCommands_GetMinimumBusyTimeAddress;
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_FALSE));
    // Read data.
    BusyTime = MlnReadTransferProtocolData(1);
    if (BusyTime != NVBOOT_ML_NAND_MINIMUM_BUSY_TIME)
    {
        PRINT_NAND_ERRORS("\r\nBusyTime=0x%x", BusyTime);
        PRINT_NAND_ERRORS("\r\nBusyTime not matching");
        NV_ASSERT(NV_FALSE);
    }
    return e;
}

static NvBootError MlnSetTransferProtocols(void)
{
    NvBootError e;
    NvU16 Address;
    NvU32 DataRead;
    
    // Set Transfer Protocol 2.
    Address = NvBootMlnCommands_SetTransferProtocol2Address | 
                  (NVBOOT_ML_NAND_TRANSFER_PRTOCOL2_CONFIG << 8);
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_TRUE));
    // Set Transfer Protocol 1.
    Address = NvBootMlnCommands_SetTransferProtocol1Address | 
                  (NVBOOT_ML_NAND_TRANSFER_PRTOCOL1_CONFIG << 8);
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_TRUE));
    // Set Transfer Protocol 3.
    Address = NvBootMlnCommands_SetTransferProtocol3Address | 
                  (NVBOOT_ML_NAND_TRANSFER_PRTOCOL3_CONFIG << 8);
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_TRUE));
    
    // Validate the tranfer protocols back.
    // Get Transfer Protocol 1.
    Address = NvBootMlnCommands_GetTransferProtocol1Address;
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_FALSE));
    // Read data.
    DataRead = MlnReadTransferProtocolData(1);
    if (DataRead != NVBOOT_ML_NAND_TRANSFER_PRTOCOL1_CONFIG)
    {
        PRINT_NAND_ERRORS("\r\nDataRead1=0x%x", DataRead);
        PRINT_NAND_ERRORS("\r\nDataRead1 not matching");
        NV_ASSERT(NV_FALSE);
    }
    
    // Get Transfer Protocol 2.
    Address = NvBootMlnCommands_GetTransferProtocol2Address;
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_FALSE));
    // Read data.
    DataRead = MlnReadTransferProtocolData(1);
    if (DataRead != NVBOOT_ML_NAND_TRANSFER_PRTOCOL2_CONFIG)
    {
        PRINT_NAND_ERRORS("\r\nDataRead2=0x%x", DataRead);
        PRINT_NAND_ERRORS("\r\nDataRead2 not matching");
        NV_ASSERT(NV_FALSE);
    }
    
    // Get Transfer Protocol 3.
    Address = NvBootMlnCommands_GetTransferProtocol3Address;
    NV_BOOT_CHECK_ERROR(MlnSendTransferProtocolCommand(Address, NV_FALSE));
    // Read data.
    DataRead = MlnReadTransferProtocolData(1);
    if (DataRead != NVBOOT_ML_NAND_TRANSFER_PRTOCOL3_CONFIG)
    {
        PRINT_NAND_ERRORS("\r\nDataRead3=0x%x", DataRead);
        PRINT_NAND_ERRORS("\r\nDataRead3 not matching");
        NV_ASSERT(NV_FALSE);
    }
    return e;
}

static void HwNandSetupDma(NvU8* pBuffer, NvU32 NumBytes)
{
    NvU32 ConfigReg;
    NvU32 DmaMasterControl;
    
    PRINT_NAND_MESSAGES("\r\nSetting up Dma");
    // Set data block and tag pointers.
    // Set Dma data transfer sizes.
    NV_NAND_WRITE(DATA_BLOCK_PTR, (NvU32)pBuffer);
    NV_NAND_WRITE(DMA_CFG_A, NumBytes - 1);
    NV_NAND_WRITE(TAG_PTR, (NvU32)(&s_MlnContext->SpareAreaPerLba[0]));
    NV_NAND_WRITE(DMA_CFG_B, NVBOOT_ML_NAND_SPARE_AREA_SIZE_IN_BYTES - 1);
    
    // Select the page size.
    // Select bus width as 8 or 16 bit.
    ConfigReg = NV_DRF_DEF(NAND, CONFIG, PAGE_SIZE_SEL, PAGE_SIZE_512) |
                NV_DRF_NUM(NAND, CONFIG, TAG_BYTE_SIZE, 
                    (NVBOOT_ML_NAND_SPARE_AREA_SIZE_IN_BYTES - 1));
    if (s_MlnContext->DataWidth == NvBootMlnDataWidth_16Bit)
        ConfigReg |= NV_DRF_DEF(NAND, CONFIG, BUS_WIDTH, BUS_WIDTH_16);
    NV_NAND_WRITE(CONFIG, ConfigReg);
    
    // Set the Dma burst size to 8 words.
    // Enable Dma performance.
    // Enable Dma for main area.
    // Finally, Enable the Dma.
    DmaMasterControl = NV_DRF_DEF(NAND, DMA_MST_CTRL, BURST_SIZE, BURST_8WORDS)|
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_PERF_EN, ENABLE) |
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_EN_A, ENABLE) |
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_EN_B, ENABLE) |
                       NV_DRF_DEF(NAND, DMA_MST_CTRL, DMA_GO, ENABLE);

    NV_NAND_WRITE(DMA_MST_CTRL, DmaMasterControl);
}

/* Public Function Definitions */

NvBootError 
NvBootMobileLbaNandInit(
    const NvBootMobileLbaNandParams *Params, 
    NvBootMobileLbaNandContext *Context)
{
    NvU32 SdaSectorCount;
    NvBootError e = NvBootError_DeviceUnsupported;
    
    // Validate params first.
    NV_ASSERT(Context != NULL);
    NV_ASSERT(Params != NULL);
    
    PROFILE();
    // Store the pointer to the context structure for later reference.
    s_MlnContext = Context;
    s_MlnContext->NumOfAddressCycles = s_FuseInfo.NumOfAddressCycles;
    s_MlnContext->DataWidth = s_FuseInfo.DataWidth;
    // Initialize the MobileLbaNand Hw controller.
    HwNandInitializeHwController(Params);
    // Wait for 30ms as per spec.
    NvBootUtilWaitUS(NVBOOT_ML_NAND_BOOT_TIME_IN_US);
    NV_BOOT_CHECK_ERROR(MlnReset());
    // Wait for 30ms as per spec.
    NvBootUtilWaitUS(NVBOOT_ML_NAND_BOOT_TIME_IN_US);
    NV_BOOT_CHECK_ERROR(MlnReadId());
    // Set pinmuxing again to reflect the data width change, if any.
    HwNandSetPinmux(s_MlnContext->DataWidth);
    // When Secondary command is enabled, Nand Controller expects RBSY to go low
    // and comeback to high. Mobile Lba nand may not do it, if requested 
    // operation can be done without indicating busy. This causes the problem.
    // So, Set Minimum Busy time to 500ns. This ensures that the chip keeps RBSY
    // line low(busy) for a minimum of 500ns, even if the chip is ready for 
    // requested operation with in 500ns.
    NV_BOOT_CHECK_ERROR(MlnSetMinimumBusyTime());
    // Set transfer protocols for accessing Mobile Lba Nand.
    NV_BOOT_CHECK_ERROR(MlnSetTransferProtocols());
    
    // Get Sda Size.
    SdaSectorCount = MlnGetSdaSectorCount();
    if (s_FuseInfo.ReadBctFromSda && (Params->SdaSectorCount == 0xFFFFFFFF))
        // Read from Sda is set & Params->SdaSectorCount is set by GetParams().
        s_MlnContext->SdaSectorCount = SdaSectorCount;
    else if (Params->SdaSectorCount == 0xFFFFFFFF)
        // Read from Mda is set & Params->SdaSectorCount is set by GetParams().
        s_MlnContext->SdaSectorCount = 0;
    else
        // Use Sda sector count specified by cfg file irrespective of fuse.
        s_MlnContext->SdaSectorCount = Params->SdaSectorCount;
    
    // Check whether it conforms to Sda region.if not, make it.
    if (s_MlnContext->SdaSectorCount > SdaSectorCount)
        s_MlnContext->SdaSectorCount = SdaSectorCount;
    // Get Mda Size.
    s_MlnContext->MdaSectorCount = MlnGetMdaSectorCount();
    s_MlnContext->DeviceStatus = NvBootDeviceStatus_Idle;
    s_MlnBitInfo->DiscoveredDataWidth = s_MlnContext->DataWidth;
    return NvBootError_Success;
}

void 
NvBootMobileLbaNandGetParams(
    const NvU32 ParamIndex, 
    NvBootMobileLbaNandParams **Params)
{
    NvU32 Index;
    NV_ASSERT(Params != NULL);
    
    PROFILE();
    // Extract Num of address cycles from Param Index, which comes from fuses.
    Index = NV_DRF_VAL(ML_NAND_DEVICE, CONFIG, ADDRESS_CYCLE, ParamIndex);
    /*
     * Two Fuse bits are used for Address cycle count. The bits value starting 
     * from zero to three, corresponds to address cycle counts of 7, 4, 5 or 6.
     */
    s_FuseInfo.NumOfAddressCycles = Index ? (Index + 3) : 7;
    /*
     * Extract Data width selection from Param Index, which comes from fuses.
     * Two Fuse bits are used for Data width Selection. The bits value starting 
     * from zero to three, corresponds to Discovery, 8-bit, 16-bit and rsvd.
     */
    s_FuseInfo.DataWidth = (NvBootMlnDataWidth)NV_DRF_VAL(ML_NAND_DEVICE, CONFIG, 
                                DATA_WIDTH, ParamIndex);
    // Extract region to read from Param Index, which comes from fuses.
    Index = NV_DRF_VAL(ML_NAND_DEVICE, CONFIG, REGION_SELECT, ParamIndex);
    /*
     * One Fuse bit is used for Region Selection. The bits value starting 
     * from zero to one, corresponds to Sda and Mda.
     */
    s_FuseInfo.ReadBctFromSda = (Index ? NV_FALSE : NV_TRUE);
    /*
     * Extract Pinmux selection from Param Index, which comes from fuses.
     * One Fuse bit is used for Pinmux Selection. The bits value starting 
     * from zero to one, corresponds to Primary and Secondary.
     */
    s_FuseInfo.PinmuxSelection = NV_DRF_VAL(ML_NAND_DEVICE, CONFIG, 
                                    PINMUX_SELECTION, ParamIndex);
    /*
     * Extract Pin order selction to use from Param Index, which comes from fuses.
     * One Fuse bit is used to select pin order. The bit value starting 
     * from zero to one, corresponds to primary and secondary pin order.
     */
    s_FuseInfo.PinOrder = NV_DRF_VAL(ML_NAND_DEVICE, CONFIG, 
                            PIN_ORDER_SELECTION, ParamIndex);
    /*
     * Set the MobileLbaNand clock source frquency as 24MHz before BCT is read.
     * The Clock Source to MobileLbaNand is PLLP and which is operating at 432MHz.
     * To get 24MHz to MobileLbaNand controller, divide it by 18. 432/18 = 24MHz.
     */
    s_DefaultMlnParams.ClockDivider = 18;
    /*
     * These are the timings that would work for All MobileLbaNand Flashes, 
     * whose read cycle is <= 50ns. the resultant values should be 0x14020001 
     * and 0x00000002.
     */
    s_DefaultMlnParams.NandTiming = NV_DRF_NUM(NAND, TIMING, TRP_RESP_CNT, 1) |
                                       NV_DRF_NUM(NAND, TIMING, TWB_CNT, 4) |
                                       NV_DRF_NUM(NAND, TIMING, TCR_TAR_TRR_CNT,0) |
                                       NV_DRF_NUM(NAND, TIMING, TWHR_CNT, 2) |
                                       NV_DRF_NUM(NAND, TIMING, TCS_CNT, 0) |
                                       NV_DRF_NUM(NAND, TIMING, TWH_CNT, 0) |
                                       NV_DRF_NUM(NAND, TIMING, TWP_CNT, 0) |
                                       NV_DRF_NUM(NAND, TIMING, TRH_CNT, 0) |
                                       NV_DRF_NUM(NAND, TIMING, TRP_CNT, 1);
    s_DefaultMlnParams.NandTiming2 = NV_DRF_NUM(NAND, TIMING2, TADL_CNT, 2);
    s_DefaultMlnParams.SdaSectorCount = 0xFFFFFFFF;
    *Params = (NvBootMobileLbaNandParams*)&s_DefaultMlnParams;
    // Fill MLba Bit info.
    s_MlnBitInfo->FuseDataWidth = s_FuseInfo.DataWidth;
    s_MlnBitInfo->FuseNumAddressCycles = s_FuseInfo.NumOfAddressCycles;
    s_MlnBitInfo->FuseReadBctFromSda = s_FuseInfo.ReadBctFromSda;
    s_MlnBitInfo->FusePinmuxSelection = s_FuseInfo.PinmuxSelection;
    s_MlnBitInfo->FusePinOrder = s_FuseInfo.PinOrder;
    PRINT_NAND_MESSAGES("\r\nParamIndex=0x%x, NumOfAddressCycles=%d, DataWidth="
        "%d(0->Disc, 1->8bit, 2->16bit), ReadBctFromSda=%d, PinmuxSelection=%d,"
        "PinOrder=%d, ClockDivider=%d, NandTiming=0x%x, NandTiming2=0x%x", 
        ParamIndex, s_FuseInfo.NumOfAddressCycles, s_FuseInfo.DataWidth, 
        s_FuseInfo.ReadBctFromSda, s_FuseInfo.PinmuxSelection, s_FuseInfo.PinOrder
        s_DefaultMlnParams.ClockDivider, s_DefaultMlnParams.NandTiming, 
        s_DefaultMlnParams.NandTiming2);
}

NvBool 
NvBootMobileLbaNandValidateParams(
    const NvBootMobileLbaNandParams *Params)
{
    PRINT_NAND_MESSAGES("\r\nValidateParams, NumOfAddressCycles=%d, DataWidth="
        "%d(0->Disc, 1->8bit, 2->16bit), ReadBctFromSda=%d, PinmuxSelection=%d,"
        "PinOrder=%d,  ClockDivider=%d, NandTiming=0x%x, NandTiming2=0x%x", 
        s_FuseInfo.NumOfAddressCycles, s_FuseInfo.DataWidth, 
        s_FuseInfo.ReadBctFromSda, s_FuseInfo.PinmuxSelection, s_FuseInfo.PinOrder
        s_DefaultMlnParams.ClockDivider, s_DefaultMlnParams.NandTiming, 
        s_DefaultMlnParams.NandTiming2);
    
    PROFILE();
    // Validate Params.
    if (s_FuseInfo.DataWidth >= NvBootMlnDataWidth_Num)
        return NV_FALSE;
    if ( (Params->ClockDivider == 0) || 
         (Params->ClockDivider > NVBOOT_NAND_MAX_CLOCK_DIVIDER_SUPPORTED) )
        return NV_FALSE;
    return NV_TRUE;
}

void 
NvBootMobileLbaNandGetBlockSizes(
    const NvBootMobileLbaNandParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);
    NV_ASSERT(s_MlnContext != NULL);
    
    *BlockSizeLog2 = s_MlnContext->BlockSizeLog2;
    *PageSizeLog2 = s_MlnContext->PageSizeLog2;
    PRINT_NAND_MESSAGES("\r\nBlockSize=%d, PageSize=%d, PagesPerBlock=%d", 
        (1 << s_MlnContext->BlockSizeLog2),(1 << s_MlnContext->PageSizeLog2),
        (1 << s_MlnContext->PagesPerBlockLog2));
}

NvBootError 
NvBootMobileLbaNandReadPage(
    const NvU32 Block, 
    const NvU32 Page, 
    NvU8 *pBuffer)
{
    NvBootError e;
    NvU32 SectorAddress;
    NvBootMlnCommand* pCommand = &s_MlnContext->Command;
    
    NV_ASSERT(Page < (1 << s_MlnContext->PagesPerBlockLog2));
    NV_ASSERT(pBuffer != NULL);
    
    PRINT_NAND_MESSAGES("\r\n     Reading Block=%d, Page=%d", Block, Page);
    PROFILE();
    s_MlnBitInfo->NumPagesRead++;
    // Set up the command structure.
    SectorAddress = (Block << s_MlnContext->PagesPerBlockLog2) + Page;
    if (SectorAddress >= s_MlnContext->SdaSectorCount)
    {
        // Subtract the sda sector count to get the offset in MDA.
        SectorAddress -= s_MlnContext->SdaSectorCount;
        // Sector that is being read falls in MDA region. MDA region starts from
        // 0x200:0000 sector number. So, Add the start address to block.
        SectorAddress += NVBOOT_ML_NAND_MDA_START_ADDRESS;
        pCommand->Command1 = NvBootMlnCommands_ReadMda;
        PRINT_NAND_MESSAGES(", Mda SectorAddress=%d", SectorAddress);
    }
    else
    {
        pCommand->Command1 = NvBootMlnCommands_ReadSda;
        PRINT_NAND_MESSAGES(", Sda SectorAddress=%d", SectorAddress);
    }
    pCommand->Command2 = NvBootMlnCommands_ReadStart;
    pCommand->SectorCount = 1;
    pCommand->SectorAddress = SectorAddress;
    // Setup the Dma
    HwNandSetupDma(pBuffer, (1 << s_MlnContext->PageSizeLog2));
    // Store address of pBuffer in mobilenand context. 
    s_MlnContext->CurrentReadBufferAddress = pBuffer;
    // Send the Read Command.
    NV_BOOT_CHECK_ERROR(HwNandSendCommand(pCommand));
    s_MlnContext->DeviceStatus = NvBootDeviceStatus_ReadInProgress;
    s_MlnContext->ReadStartTime = NvBootUtilGetTimeUS();
    return e;
}

NvBootDeviceStatus NvBootMobileLbaNandQueryStatus(void)
{
    NvU32 DmaGo;
    NvU32 IsIdle;
    NvU32 StatusRegValue;
    NvU32 DmaMasterControl;
    NvU32 IsEccDecodeFailed;
    
    if (s_MlnContext->DeviceStatus == NvBootDeviceStatus_ReadInProgress)
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
        {
            s_MlnContext->DeviceStatus = NvBootDeviceStatus_Idle;
            IsEccDecodeFailed = (s_MlnContext->SpareAreaPerLba[
                                 NVBOOT_ML_NAND_SPARE_AREA_ECC_ERR_OFFSET] & 
                                 NVBOOT_ML_NAND_SPARE_AREA_ECC_ERR_MASK);
            if (IsEccDecodeFailed)
            {
                s_MlnContext->DeviceStatus = NvBootDeviceStatus_EccFailure;
                PRINT_NAND_MESSAGES("\r\nNvBootDeviceStatus_EccFailure");
                s_MlnBitInfo->NumUncorrectableErrorPages++;
            }
        }
        // Check for TimeOut.
        else if (NvBootUtilElapsedTimeUS(s_MlnContext->ReadStartTime) > 
                NVBOOT_NAND_READ_TIMEOUT_IN_US)
        {
            s_MlnContext->DeviceStatus = NvBootDeviceStatus_ReadFailure;
        }
    }

    // Read is a success. If the decyrption address is to external memory, make
    // sure the writes to memory are coherent. 
    if (NvBootAhbCheckIsExtMemAddr((NvU32 *)s_MlnContext->CurrentReadBufferAddress)) 
    {
        NvBootAhbWaitCoherency(ARAHB_MST_ID_NAND);
    }

    return s_MlnContext->DeviceStatus;
}

void NvBootMobileLbaNandShutdown(void)
{
    // Keep the controller in Reset and disable the clock.
    NvBootResetSetEnable(NvBootResetDeviceId_NandId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_NandId, NV_FALSE);
}

NvBootError NvBootMobileLbaNandGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
