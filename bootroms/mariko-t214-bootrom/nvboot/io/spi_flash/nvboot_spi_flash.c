/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "arspi.h"
#include "arqspi.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_device_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_spi_flash_context.h"
#include "nvboot_spi_flash_int.h"
#include "nvboot_spi_flash_param.h"
#include "nvboot_spi_flash_local.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_util_int.h"


#include "nvboot_car_int.h"
#include "nvboot_devmgr_int.h"
#include "project.h"

/*
 * Spi flash device callbacks for Device Manager.
 */
NvBootDevMgrCallbacks SpiDeviceCallback = 
    {
        /* Callbacks for the Spi flash */
        (NvBootDeviceGetParams)NvBootSpiFlashGetParams,
        (NvBootDeviceValidateParams)NvBootSpiFlashValidateParams,
        (NvBootDeviceGetBlockSizes)NvBootSpiFlashGetBlockSizes,
        (NvBootDeviceInit)NvBootSpiFlashInit,
        (NvBootDeviceRead)NvBootSpiFlashReadPage,
        (NvBootDeviceQueryStatus)NvBootSpiFlashQueryStatus,
        (NvBootDeviceShutdown)NvBootSpiFlashShutdown,
        (NvBootDeviceGetReaderBuffersBase)NvBootSpiFlashGetReaderBuffersBase,
        (NvBootDevicePinMuxInit)NvBootSpiFlashPinMuxInit
    };


/* Note that BootROM SPI code remains largely unchanged for several chips.
   The way the SPI controller is used is undocumented, but working. The controller
   is set up for DMA, but no apbdma channel is set up, and data is read with
   pio from the RX_FIFO. */

static NvBootSpiFlashContext * s_pSpiFlashContext = NULL;

// Boot Info table.
extern NvBootInfoTable BootInfoTable;

// Pointer to spi flash Bit info.
static NvBootSpiFlashStatus* s_pSpiFlashBitInfo = \
                        (NvBootSpiFlashStatus*)&BootInfoTable.SecondaryDevStatus[0];

static  uint8_t CommandData[8];
static  uint8_t DataRead[16];

static const NvU32 s_ClockDiv[] =
{
    20,  /*Clock Source pllp_out0, divisor to get 20.4MHz */
    2,  /*Clock Source clk_m, divisor to get 19.2MHz */
    8  /*Clock Source pllp_out0, divisor to get 51MHz */
};

static NvBootSpiFlashTrimmers s_Trimmers[] =
{
    	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

static const NvBootSpiFlashTimeouts s_SpiFlashXferTimeouts[] =
{
    // Read time out of 200 milli seconds.
    { 200000, 200000 }
};

static const NvBootSpiFlashQuadSdrLC s_QuadOpTable[] =
{
    // <=50MHz for Spansion QSPI
    // DO NOT use dummy cycles interpreted from the lookup table
    // for QUAD SDR performance.
    // HW team has recommended the default value or the value read
    // from CR1V register to be used for the number of dummy cycles.
    {0x3, 2, 8}
};

/**
  * One set of defaults Parameters
  */
static NvBootSpiFlashParams s_DefaultSpiFlashParams;

// fwd declartion 
void NvBootSpiClockTable(void **SpiClockTable, ClockTableType *Id);


// Primary clock table for SPI controller .
ClockInst s_SpiClkTable_Init[] = 
{
    // Assert Reset to Device, SS, Host (in case of Host boot), PadCtl
    Instc(Clk_W1b,  RST_DEV_Y_SET, SET_QSPI_RST),

    // Configure the clock source with divider 20, which gives 20.4MHz.
    // (Pllp_out0@408M/20)
    Instc(Clk_Src,  CLK_SOURCE_QSPI, QSPI_CLK_SRC, PLLP_OUT0,
                                              QSPI_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(20, 0)),
    // Enable the clock.
    Instc(Clk_W1b,  CLK_ENB_Y_SET, SET_CLK_ENB_QSPI),

    // Remove the controller from Reset.
    Instc(Clk_W1b,  RST_DEV_Y_CLR, CLR_QSPI_RST),

    {0} // Null terminated.

};

// clock Divisor for Spi controller 51 Mhz IO clock.
const ClockInst s_SpiClkDiv_51[] = 
{
    // Configure the clock source with divider 8, which gives 51MHz.
    // (Pllp_out0@408M/8)
    Instc(Clk_Src,  CLK_SOURCE_QSPI, QSPI_CLK_SRC, PLLP_OUT0,
                                              QSPI_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(8, 0)),
    {0} // Null terminated.
};

// clock Divisor for Spi controller 20.4 Mhz IO clock.
const ClockInst s_SpiClkDiv_20_4[] = 
{
    // Configure the clock source with divider 20, which gives 20.4MHz.
    // (Pllp_out0@408M/20)
    Instc(Clk_Src,  CLK_SOURCE_QSPI, QSPI_CLK_SRC, PLLP_OUT0,
                                              QSPI_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(20, 0)),
    {0} // Null terminated.
};

// clock Divisor for Spi controller 19.2 Mhz IO clock.
const ClockInst s_SpiClkDiv_19_2[] = 
{
    // Configure the clock source with divider 2, which gives 19.2MHz.
    // (PllM@38.4/2)
    Instc(Clk_Src,  CLK_SOURCE_QSPI, QSPI_CLK_SRC, CLK_M,
                                              QSPI_CLK_DIVISOR, NVBOOT_CLOCKS_7_1_DIVIDER_BY(2, 0)),
    {0} // Null terminated.
};

void NvBootSpiClockTable(void **SpiClockTable, ClockTableType *Id)
{
    // Passing in list of tables.
    *SpiClockTable= s_SpiClkTable_Init;
    *Id = TYPE_SINGLE_TABLE;
}

NvBootError SpiTalkToDevice(
    uint8_t *pWriteBuffer,
    NvU32 BytesToWrite,
    uint8_t *pReadBuffer,
    NvU32 BytesToRead,
    NvBool IsDataCmd);

NvU32 * GetSpiFlashConstantsTable(SpiFlashConstantTableIdx idx)
{
   NvU32 *pTable = NULL;
   switch(idx)
   {
    case SpiFlashConstantTableIdx_ClockDivisorsTable:
        pTable = (NvU32 *)s_ClockDiv;
        break;
    case SpiFlashConstantTableIdx_TrimmersConfigTable: 
        pTable = (NvU32 *)s_Trimmers;
        break;
    case SpiFlashConstantTableIdx_SpiFlashXferTimeoutTable:
        pTable = (NvU32 *)s_SpiFlashXferTimeouts;
        break;
    case SpiFlashConstantTableIdx_QReadConfigTable:
        pTable = (NvU32 *)s_QuadOpTable;
        break;
    case SpiFlashConstantTableIdx_Num:
        break;
    }
    return pTable;
}

static NvBootError    EnableClockSource(const NvBootSpiFlashContext *Context)
{
    /*
     * During debug, clk mux selection was found to be missing in the parameters.
     * Therefore, it has been combined with the KVCO parameter into one uint8_t value as follows:
     * PllcClkSelKVCO[5:4] = CLK_SEL
     * PllcClkSelKVCO[0:0] = KVCO
     */
    
    switch(Context->ClockSource)
    {
    case NvBootSpiClockSource_PllC4_Muxed:
        break;
    case NvBootSpiClockSource_PllPOut0:
        /* PLLP is a FO pll. Do nothing. */
    case NvBootSpiClockSource_ClockM:
        /* Do nothing */
        break;
    default:
        break;
    }

    return NvBootError_Success;
}

static void PopulateCommand(uint8_t *Command, uint8_t CommandNum, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4)
{
    Command[0] = CommandNum; // 1 opcode
    Command[1] = Byte1;                     // 3 address cycle
    Command[2] = Byte2;
    Command[3] = Byte3;
    Command[4] = Byte4;
}

static void QuadModeEnDis(NvBool QuadEnable)
{
    NvBootError e = NvBootError_Success;
    uint8_t QuadEnableData = (QuadEnable == NV_TRUE)? NvBootSpiFlashQuadData_QuadEn: NvBootSpiFlashQuadData_QuadDis;

    //Enable Write
    PopulateCommand(CommandData, NvBootSpiFlashCommand_WriteEnableEn, 0, 0, 0, 0);
    NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, 0, 0, NV_FALSE));

    SET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus,  WriteEnabled);
    PopulateCommand(CommandData, NvBootSpiFlashCommand_WriteAnyRegister, 
        (uint8_t)((NvBootSpiFlashSpansionReg_CR1V >> 16) & 0xFF),
        (uint8_t)((NvBootSpiFlashSpansionReg_CR1V >> 8) & 0xFF),
        (uint8_t)((NvBootSpiFlashSpansionReg_CR1V) & 0xFF), QuadEnableData);
    NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 5, 0, 0, NV_FALSE));
    SET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus,  QuadCommandSent);

   //Check Busy not done yet
    PopulateCommand(CommandData, NvBootSpiFlashCommand_ReadStatus, 1, 0, 0, 0);
    NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, DataRead, 1, NV_FALSE));

    while ((DataRead[0] & STATUS_REGISTER_WRITE_IN_PROGRESS_MASK) & 0x1)
    {
        goto fail;
    }

    //Disable Write
    PopulateCommand(CommandData, NvBootSpiFlashCommand_WriteEnableDis, 0, 0, 0, 0);
    NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, 0, 0, NV_FALSE));
    SET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus,  QuadModeSetUnsetCheck);

fail:
       if (QuadEnable)
         SET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus, QuadModeEnabled);
}
void SetReadDummyCycles(NvU32 DummyCycles)
{
    NvU32 RegData;
    RegData = SPI_REG_READ32(MISC);
    RegData = NV_FLD_SET_DRF_NUM(QSPI, MISC, NUM_OF_DUMMY_CLK_CYCLES, DummyCycles, RegData);
    SPI_REG_WRITE32(MISC, RegData);
}

static void ProgramTrimmerValues()
{

    NvBootSpiFlashTrimmers * Trimmers = (NvBootSpiFlashTrimmers *)
            (GetSpiFlashConstantsTable(SpiFlashConstantTableIdx_TrimmersConfigTable));

    NvU32 RegVal = SPI_REG_READ32(COMMAND2);
    RegVal = NV_FLD_SET_DRF_NUM(SPI, COMMAND2,  Tx_Clk_TAP_DELAY, Trimmers->TxTapDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(SPI, COMMAND2,  Rx_Clk_TAP_DELAY, Trimmers->RxTapDelay, RegVal);
    SPI_REG_WRITE32(COMMAND2, RegVal);

    RegVal = SPI_REG_READ32(TIMING3);
    RegVal = NV_FLD_SET_DRF_NUM(QSPI, TIMING3,  DATA0_LINE_TAP_DELAY, Trimmers->Data0LineTapDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(QSPI, TIMING3,  DATA1_LINE_TAP_DELAY, Trimmers->Data1LineTapDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(QSPI, TIMING3,  DATA2_LINE_TAP_DELAY, Trimmers->Data2LineTapDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(QSPI, TIMING3,  DATA3_LINE_TAP_DELAY, Trimmers->Data3LineTapDelay, RegVal);
    SPI_REG_WRITE32(TIMING3, RegVal);

}

/* Response from hw team
   When Both Tx & Rx are enabled (Full Duplex Mode), set DMA_BLOCK_SIZE to 
   sum of tx bytes + rx bytes - 1 if bit_length is set to 7.
   You need to ignore first 4 Bytes of Rx which SPI received when the command is sent to the device.
   Similarly Device will neglect the next 8 Bytes which SPI sends, when the Device is sending data in 
   response to the command which was sent earlier.
*/
NvBool IsSpiTransferComplete( void)
{
    // Read the Status register and findout whether the SPI is Bsy
    // Should be called only if rx and/or tx was enabled else rdy bit is 0 by default
    NvU32 StatusReg = SPI_REG_READ32( TRANSFER_STATUS);
    if (StatusReg & NV_DRF_DEF(SPI, TRANSFER_STATUS, RDY, READY))
    {
        //Write 1 to clear RDY field
        SPI_REG_WRITE32( TRANSFER_STATUS,StatusReg);
        return NV_TRUE;
    }
    else
    return NV_FALSE;
}

void SetSpiChipSelectLevel(NvBool ChipSelectOp)
{
    //Is effective only when SW CS is being used
    NvU32 CmdRg = SPI_REG_READ32(COMMAND);
    NvBool ProgramVal ;
    ProgramVal = (s_pSpiFlashContext->ChipSelectPolarity) ? (!ChipSelectOp): (ChipSelectOp);
    //ProgramVal = (ChipSelectOp == s_pSpiFlashContext->ChipSelectPolarity) ? ChipSelectOp : (s_pSpiFlashContext->ChipSelectPolarity - ChipSelectOp) ;
    
    CmdRg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CS_SW_VAL, ProgramVal, CmdRg);
    SPI_REG_WRITE32(COMMAND, CmdRg);
}
void SpiHwFlushFifos(NvU32 FlushField)
{
    NvU32 StatusReg;
    // Write in to Status register to clear the FIFOs
    SPI_REG_WRITE32(FIFO_STATUS, FlushField);
    // Wait until those bits become 0.
    do
    {
        StatusReg = SPI_REG_READ32(FIFO_STATUS);
    }while (StatusReg & FlushField);
}

void SpiHwDisableTransfer(void)
{
    NvU32 Reg_Val;
    //Disable DMA mode
    Reg_Val = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, DMA_EN, DISABLE, 
                        SPI_REG_READ32(DMA_CTL));
    SPI_REG_WRITE32( DMA_CTL, Reg_Val);
    //Flush Tx fifo 
    SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_FLUSH, FLUSH));
    //Flush Rx fifo
    SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_FLUSH, FLUSH));
}
NvBootError SpiHwCheckTimeout(NvU32 TransferStartTimeinUs)
{
    NvBootError Error = NvBootError_Success;
    if (NvBootUtilElapsedTimeUS(TransferStartTimeinUs) > s_pSpiFlashContext->DataXferTimeout)
    {
        // Check last time before saying timeout.
        if (!(IsSpiTransferComplete()))
        {
            Error = NvBootError_HwTimeOut;
            SpiHwDisableTransfer();
        }
    }
    return Error;
}
NvBootError SpiChkHwRdyOrTimeout(NvU32 TransferStartTimeinUs)
{
    NvBootError Error = NvBootError_Success;
    while(1)
    {
        if (IsSpiTransferComplete())
        {
            break;
        }
        else
        {
            Error = SpiHwCheckTimeout(TransferStartTimeinUs);
            if (Error != NvBootError_Success)
            {
                //hw timeout detected
                break;
            }
        }
    }//end of while(1)
    return Error;
}

NvBootError HwSpiWriteInTransmitFifo(uint8_t *pTxBuff, NvU32 WordsOrBytesToWrite,NvU32 DmaPktLen,NvU32 WriteStartTimeinUs)
{
    NvU32 RegData;
    NvBootError Error = NvBootError_Success;

    while (WordsOrBytesToWrite)
    {
        // Read the Status register and find if the Tx fifo is FULL.Push data only when tx fifo is not full
        RegData = SPI_REG_READ32(FIFO_STATUS);
        if (RegData & NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_FULL, FULL))
        {
            Error = SpiHwCheckTimeout(WriteStartTimeinUs);
            if (Error != NvBootError_Success)
            {
                //hw timeout detected
                break;
            }
            else
            {
                continue;
            }           
        }
        //Tx fifo is empty. Now write the data into the fifo,increment the buffer pointer and decrement the count

        //SPI protocol expects most significant bit of a byte first i.e. (first)bit7, bit6....bit0 (last)
        //During SPI controller initialization LSBi_FE is set to LAST and LSBy_FE is also set to LAST so that 
        //Data transmitted : (last)[bit24-bit31],[bit16-bit23],[bit8-bit15], [bit0-bit7] (first) [rightmost bit is transmitted first]

        //32 bits are read from a pointer pointing to UInt8.
        //E.g.pTxBuff is pointing to memory address 0x1000 and bytes stored are
        //0x1000 = 0x12
        //0x1001 = 0x34
        //0x1002 = 0x56
        //0x1003 = 0x78
        //Reading 32 bit from location 0x1000 in little indian format would read 0x78563412 and this is
        //the data that is being stored in tx fifo. By proper setting of  LSBi_FE and LSBy_FE bits in 
        //command register, bits can be transferred in desired manner
        //In the example given above 0x12 byte is transmitted first and also most significant bit gets out first
        if(DmaPktLen == BYTES_PER_WORD)
        {
            RegData = (*((NvU32 *)pTxBuff)); 
        }
        else
        {
            RegData = (NvU32)(*pTxBuff);
        }
        SPI_REG_WRITE32( TX_FIFO, RegData);
        //increment buffer pointer  
        pTxBuff += DmaPktLen;
        //decrement requested number of words
            (WordsOrBytesToWrite)--;
    }
    return(Error);
}
NvBootError PollRdyClearRdy()
{
    NvU32 Reg_Val;
    NvU32 Timeout = 1000;
    NvBootError e = NvBootError_Success;

    do {
        NvBootUtilWaitUS(1);
        Reg_Val = SPI_REG_READ32(TRANSFER_STATUS);
        Timeout--;
    }  while((Timeout) && (!(NV_DRF_VAL(QSPI, TRANSFER_STATUS,RDY, Reg_Val))));
     if ((Timeout == 0) && (!(NV_DRF_VAL(QSPI, TRANSFER_STATUS, RDY, Reg_Val))) )
        e = NvBootError_Busy;
    // Clear interrupt- clean up code later
    SPI_REG_WRITE32(TRANSFER_STATUS, 0x40000000);

    return e;

}

void SetBusWidth(NvBootSpiDataWidth BusWidth)
{
    NvU32 Reg_Val;

    Reg_Val = SPI_REG_READ32(COMMAND);
    // Set bus width based on the selection
    Reg_Val = NV_FLD_SET_DRF_NUM(QSPI, COMMAND, INTERFACE_WIDTH, BusWidth, Reg_Val);
    SPI_REG_WRITE32(COMMAND, Reg_Val);
}

void TransmitEnDis(NvBool Enable)
{
    NvU32 Reg_Val;

    //Enable/Disable Tx
    if (Enable)
    Reg_Val = NV_FLD_SET_DRF_DEF(QSPI, COMMAND, Tx_EN, ENABLE, 
                        SPI_REG_READ32(COMMAND));
    else
    Reg_Val = NV_FLD_SET_DRF_DEF(QSPI, COMMAND, Tx_EN, DISABLE, 
                        SPI_REG_READ32(COMMAND));

    SPI_REG_WRITE32( COMMAND, Reg_Val);

}
void EnableDmaCtrl(NvBool Enable)
{
    NvU32 Reg_Val;
    if (Enable)
    Reg_Val = NV_FLD_SET_DRF_DEF(QSPI, DMA_CTL, DMA_EN, ENABLE, 
                            SPI_REG_READ32(DMA_CTL));
    else
    Reg_Val = NV_FLD_SET_DRF_DEF(QSPI, DMA_CTL, DMA_EN, DISABLE, 
                            SPI_REG_READ32(DMA_CTL));
     SPI_REG_WRITE32( DMA_CTL, Reg_Val);

}
void SetModeDummyCycles()
{
    NvU32 Reg_Val;
    if((s_pSpiFlashContext->BusWidth == NvBootSpiDataWidth_x4) && (GET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus, QuadModeEnabled)))
    {
        Reg_Val = SPI_REG_READ32(MISC);
	Reg_Val = NV_FLD_SET_DRF_NUM(QSPI, MISC, NUM_OF_DUMMY_CLK_CYCLES, s_pSpiFlashContext->QuadReadDummyCycles , Reg_Val);
        SPI_REG_WRITE32(MISC, Reg_Val);
    }
}


/*
 * @param pCommand pointer to command and address buffer.
 * @param NumBytes total number of bytes for Command and Address
 *
 */
NvBootError SendCommand(uint8_t *pCommand, uint8_t NumBytes)
{
    // This function should do Phase 1 and Phase 2 for any read/write/non-data command
    // Isolating this function allows us to decouple Tx-Rx method and replace it with any
    // other method in future chips. For example, we can choose to replace it with combined 
    // sequence mode.
    // This will also allow decoupling command/address phases from actual data transfer and
    // specifics of buffers used for data transfer.
    // This function will always do PIO mode transfer since the number of bytes is very less and 
    // certainly lesser than 64B
    // PRE-REQUISITES/ASSUMPTIONS for this function call:
    // Command is always 1 byte and always sent in x1.
    // Bus width for address/data is set in the context.
    // For x1, only 1-1-1 mode is supported. For x4, only 1-4-4 is supported.
    // This is consistent with combined sequence mode of controller also.
    NvU32 DmaPktLen,Reg_Val;  
    NvU32 WriteStartTimeinUs;
    NvBootError e = NvBootError_Success;


    NV_ASSERT(NumBytes <= s_pSpiFlashContext->TxFifoDepth);
    NV_ASSERT(NumBytes);
    // =============PHASE 1====================
    // Compute number of words in command transfer and address transfer - removed 
    // all bytes will be transferred with 8 bit mode only
    Reg_Val = SPI_REG_READ32(COMMAND); 
    Reg_Val = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_8Bit_BIT_LENGTH, Reg_Val);  
     //Set length of a packet in bits.
    SPI_REG_WRITE32(COMMAND, Reg_Val);  
    DmaPktLen = 1;

    // Set Block size/transfer size
    //Set dma block size to 0. This will allow transmitting 1 byte of command.
    SPI_REG_WRITE32( DMA_BLK_SIZE, 0);


    // Set bus width to x1 for command
    SetBusWidth(NvBootSpiDataWidth_x1);

    //Flush Tx fifo before enabling Tx or before disabling TX
    SpiHwFlushFifos(NV_DRF_DEF(QSPI, FIFO_STATUS, TX_FIFO_FLUSH, FLUSH));

    // Enable transfer in controller
    TransmitEnDis(NV_TRUE);

    //Get start time
    WriteStartTimeinUs = NvBootUtilGetTimeUS();
    if (GET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus, QuadModeEnabled))
    {
    // Send command
    NV_BOOT_CHECK_ERROR_CLEANUP(HwSpiWriteInTransmitFifo(pCommand,1,DmaPktLen,WriteStartTimeinUs));

    NvBootUtilWaitUS(1);
    // Enable dma transfer (data will not be pushed out of TX_FIFO if this is not done)
    EnableDmaCtrl(NV_TRUE);
    // Wait for Controller Ready
    // Check error
    NV_BOOT_CHECK_ERROR_CLEANUP(PollRdyClearRdy());

    NumBytes --;
    pCommand++;
    } else
    {
        // It's a x1 command
        // Send all bytes for command and address at the same time. No need to change bus width.
    }  


    // If successful, go to Phase 2 else disable transfer, clear errors.
    // =============PHASE 2====================
    if (NumBytes)
    {
        //Set Block size/transfer size
            SPI_REG_WRITE32( DMA_BLK_SIZE, NumBytes-1);
        // Set bus width
        SetBusWidth((GET_SPI_FLASH_BOOT_INFO_BITFLD(InitStatus, QuadModeEnabled))? s_pSpiFlashContext->BusWidth: NvBootSpiDataWidth_x1);
        // Enable transfer in controller
        TransmitEnDis(NV_TRUE);
           
        // Set mode+dummy cycles from Context. Note: Mode and Dummy cycles should appear in Context before calling this function
        SetModeDummyCycles();

        // Send address
        NV_BOOT_CHECK_ERROR_CLEANUP(HwSpiWriteInTransmitFifo(pCommand,NumBytes,DmaPktLen,WriteStartTimeinUs));

        // Enable dma transfer (data will not be pushed out of TX_FIFO if this is not done)
        EnableDmaCtrl(NV_TRUE);    
        // Check error 
        NV_BOOT_CHECK_ERROR_CLEANUP(PollRdyClearRdy());    
        // If successful, go to Phase 3 else disable transfer, clear errors.
    }
fail:
    // =============END OF TRANSFER=====
    //Disable transfer, clear errors.

    EnableDmaCtrl(NV_FALSE);
    //Disable Tx
    TransmitEnDis(NV_FALSE);
    Reg_Val = SPI_REG_READ32( FIFO_STATUS);
    //clear the status register
    SPI_REG_WRITE32( FIFO_STATUS, Reg_Val);

    return e;
}

NvBootError SpiHwProcWrite(uint8_t *pWriteBuffer, NvU32 BytesToWrite)
{
    NvBootError e = NvBootError_Success;
    NvU32 WriteStartTimeinUs;
    NvU32 DmaBlkSz,DmaPktLen,Reg_Val,WordsToWrite;

    NV_ASSERT(BytesToWrite <= 16);
    NV_ASSERT(BytesToWrite);

    Reg_Val = SPI_REG_READ32(COMMAND); 
    if(BytesToWrite % BYTES_PER_WORD != 0)
    {
        //Number of bytes to be read is not multiple of 4 bytes 
        //Transfer just 8 bits out of 32 bits of data in the fifo. Rest bits are ignored in unpacked mode
        Reg_Val = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_8Bit_BIT_LENGTH, Reg_Val);
        DmaBlkSz = BytesToWrite;
        //Number of meaningful bytes in one packet = 1 byte
        DmaPktLen = 1;
    }
    else
    {
        Reg_Val = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_MAX_BIT_LENGTH, Reg_Val);
        DmaBlkSz = BytesToWrite / (BYTES_PER_WORD);
        //Number of meaningful bytes in one packet = 4 bytes
        DmaPktLen = BYTES_PER_WORD;
    }
    WordsToWrite = DmaBlkSz;
    //Set length of a packet in bits.
    SPI_REG_WRITE32(COMMAND, Reg_Val);  
    //Set dma block size. 
    SPI_REG_WRITE32( DMA_BLK_SIZE,(DmaBlkSz - 1));
    //Flush Tx fifo before enabling Tx 
    //SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_FLUSH, FLUSH));
    //Enable Tx
    Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);
    //Get start time
    WriteStartTimeinUs = NvBootUtilGetTimeUS();
    if (WordsToWrite)
    {
        //something to be transmitted
        if(WordsToWrite > s_pSpiFlashContext->TxFifoDepth)
        {
            NV_BOOT_CHECK_ERROR_CLEANUP(HwSpiWriteInTransmitFifo(pWriteBuffer, s_pSpiFlashContext->TxFifoDepth,DmaPktLen,WriteStartTimeinUs));
            WordsToWrite = WordsToWrite - s_pSpiFlashContext->TxFifoDepth; 
        }
        else
        {
           NV_BOOT_CHECK_ERROR_CLEANUP(HwSpiWriteInTransmitFifo(pWriteBuffer,WordsToWrite,DmaPktLen,WriteStartTimeinUs));
           WordsToWrite = 0;
        }

        if (e == NvBootError_Success)
        {
            //Before enabling DMA mode, be sure to wait 5 spi_clk cycles
            //chose 5 MHz as min freq. (It has worked in the past without delay)
            NvBootUtilWaitUS(1);

            //Data was written successfully
            //Enable DMA mode
            Reg_Val = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, DMA_EN, ENABLE, 
                            SPI_REG_READ32(DMA_CTL));
            SPI_REG_WRITE32( DMA_CTL, Reg_Val);

            if (WordsToWrite)
            {
                //More data to be written in FIFO
                //Write into fifo
                //Since dma is already enabled, just keep filling fifo
                NV_BOOT_CHECK_ERROR_CLEANUP(HwSpiWriteInTransmitFifo(pWriteBuffer,WordsToWrite,DmaPktLen,WriteStartTimeinUs));
            }
            //Make sure spi hw is ready at the end and there is no timeout
            e = SpiChkHwRdyOrTimeout(WriteStartTimeinUs);
        }
    }//end if (WordsToWrite)
    //Disable Tx
    Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Tx_EN, DISABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);
    Reg_Val = SPI_REG_READ32( FIFO_STATUS);
    //clear the status register
    SPI_REG_WRITE32( FIFO_STATUS, Reg_Val);
fail:
    //return Error local variable
    return e;
}

NvBootError SpiReadFromReceiveFifo(uint8_t *pRxBuff, NvU32 WordsOrBytesToRead,NvU32 DmaPktLen,NvU32 ReadStartTimeinUs)
{
    NvU32 Reg_Val;
    NvBootError Error = NvBootError_Success;

    NV_ASSERT(pRxBuff);
    NV_ASSERT(WordsOrBytesToRead);
    while (WordsOrBytesToRead)
    {
        // Read the Status register and find whether the RX fifo Empty
        Reg_Val = SPI_REG_READ32(FIFO_STATUS);
        if (Reg_Val & NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_EMPTY, EMPTY))
        {
            Error = SpiHwCheckTimeout(ReadStartTimeinUs);
            if (Error != NvBootError_Success)
            {
                //hw timeout detected
                break;
            }
            else
            {
                continue;
            }
        }
        // Rx fifo is found non empty. Read from rx fifo, increment the buffer pointer and decrement the count

        //SPI protocol expects most significant bit of a byte first i.e. (first)bit7, bit6....bit0 (last)
        //During SPI controller initialization LSBi_FE is set to LAST and LSBy_FE is also set to LAST so that 
        //Data received : (last) [bit24-bit31], [bit16-bit23], [bit8-bit15], [bit0-bit7] (first) [rightmost bit is received first]
        Reg_Val = SPI_REG_READ32(RX_FIFO);
        if(DmaPktLen == BYTES_PER_WORD)
        {
            //All 4 bytes are valid data
            *((NvU32 *)pRxBuff) = Reg_Val;
        }
        else
        {
            //only 1 byte is valid data
            (*pRxBuff) = (uint8_t)(Reg_Val);
        }

        //increment buffer pointer 
        pRxBuff += DmaPktLen;
        //decrement requested number of words
        WordsOrBytesToRead--;
    }
    return(Error); 
}
NvBootError SpiHwProcRead(uint8_t *pReadBuffer, NvU32 BytesToRead, NvBool IsDataCmd)
{
    NvU32 ReadStartTimeinUs;
    NvU32 DmaBlkSz = 0,DmaPktLen = 0;
    NvU32 Reg_Val;
    NvBootError Error = NvBootError_Success;
    
    NV_ASSERT(pReadBuffer);
    NV_ASSERT(BytesToRead);

    Reg_Val = SPI_REG_READ32(COMMAND);
    // After spi controller is ready for transfers, while reading data
    // we need to distinguish non-data commands so we can use PIO mode
    // specifically when XferMode is set to Dma. For example, Read Any Register
    // will always return 1 byte (uwith packed mode with DMA minimum burst length
    // of 16 words, the transfer will be busted, error out and can leave controller
    // and flash in undesirable state
    if ((s_pSpiFlashContext->XferMode == NvBootSpiXferMode_Pio) || (!IsDataCmd))
    {
        if(BytesToRead % BYTES_PER_WORD != 0)
        {
            //Number of bytes to be read is not multiple of 4 bytes 
            Reg_Val = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_8Bit_BIT_LENGTH, Reg_Val);
            DmaBlkSz = BytesToRead;
            //Number of meaningful bytes in one packet = 1 byte
            DmaPktLen = 1;
        }
        else
        {
            Reg_Val = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_MAX_BIT_LENGTH, Reg_Val);
            DmaBlkSz = BytesToRead / (BYTES_PER_WORD);
            //Number of meaningful bytes in one packet = 4 bytes
            DmaPktLen = BYTES_PER_WORD;
        }
    }
    else
    {
        return NvBootError_InvalidParameter;
    }

    SPI_REG_WRITE32(COMMAND, Reg_Val);
    if(DmaBlkSz)
        //Set dma block size. 
        SPI_REG_WRITE32(DMA_BLK_SIZE,(DmaBlkSz - 1));
    //Flush Rx fifo before enabling receive 
    SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_FLUSH, FLUSH));
    // Set Read DummyCycles to 0 during data phase
    SetReadDummyCycles(0);
    //Enable Rx
    Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);

    //Get start time
    ReadStartTimeinUs = NvBootUtilGetTimeUS();
    //Try reading data from fifo
    //Dma is already enabled so keep reading if rx fifo is non empty and hw is not timed out
    //Assumption is that pReadBuffer is pointing to a buffer which is large enough to hold requested
    //number of bytes
    if (( s_pSpiFlashContext->XferMode == NvBootSpiXferMode_Pio) || (!IsDataCmd))
    {

        //Before enabling DMA mode, be sure to wait 5 spi_clk cycles
        //Chose 5 MHz as minimum clk. (has worked in the past without this)
        NvBootUtilWaitUS(1);

        //Enable DMA mode
        Reg_Val = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, DMA_EN, ENABLE, 
                            SPI_REG_READ32(DMA_CTL));
            SPI_REG_WRITE32( DMA_CTL, Reg_Val);

        SpiReadFromReceiveFifo(pReadBuffer, DmaBlkSz,DmaPktLen,ReadStartTimeinUs);

        //Make sure spi hw is ready at the end and there is no timeout
        Error = SpiChkHwRdyOrTimeout(ReadStartTimeinUs);
        if (Error != NvBootError_Success)
            goto fail;
    }

    //Disable Rx
    Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Rx_EN, DISABLE,
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);
    //DMA_EN bits get cleared by hw if all the data was tranferred successfully else s/w will disbale it upon detection of h/w timeout
    //clear the status register
    Reg_Val = SPI_REG_READ32( FIFO_STATUS);
    SPI_REG_WRITE32( FIFO_STATUS, Reg_Val);
fail:
    //return Error local variable
    return Error;
}


NvBootError SpiTalkToDevice(
    uint8_t *pWriteBuffer,
    NvU32 BytesToWrite,
    uint8_t *pReadBuffer,
    NvU32 BytesToRead,
    NvBool IsDataCmd)
{
    NvBootError Error = NvBootError_Success;

    //set cs sw val to low 
    SetSpiChipSelectLevel(NV_FALSE);
    //Send/Rx
    if(BytesToWrite)
    {
        Error = SendCommand(pWriteBuffer, BytesToWrite);
    }
    if((Error == NvBootError_Success) && BytesToRead)
    {
        Error = SpiHwProcRead(pReadBuffer, BytesToRead, IsDataCmd);
    }
    //set cs sw val to high
    SetSpiChipSelectLevel(NV_TRUE);
    return Error;
}
NvBootError SpiWriteToDevice(
    uint8_t *pCommand,
    NvU32 CommandLength,
    uint8_t *pWriteBuffer,
    NvU32 BytesToWrite)
{
    NvBootError Error = NvBootError_Success;

    //set cs sw val to low 
    SetSpiChipSelectLevel(NV_FALSE);
    //Send/Rx
    if(BytesToWrite)
    {
        Error = SendCommand(pCommand, CommandLength);

    }
    if((Error == NvBootError_Success) && BytesToWrite)
    {
        Error = SpiHwProcWrite(pWriteBuffer, BytesToWrite);
    }
    //set cs sw val to high
    SetSpiChipSelectLevel(NV_TRUE);
    return Error;
}


static NvBootError 
SpiFlashRead(NvU32 Address, uint8_t *pDest, NvU32 BytesToRead, NvBool IsFastRead)
{
    NvBootError Error;
    uint8_t CommandData[8];
    NvU32 BytesRequested;

    NV_ASSERT(pDest);
    //prepare buffer as per spi devices' data sheet for sending out read request to the device
    if (s_pSpiFlashContext->BusWidth == NvBootSpiDataWidth_x4)
    {
        CommandData[0] = NvBootSpiFlashCommand_QuadRead;
        BytesRequested = 4;
    } else {
        if(!IsFastRead)
        {
            CommandData[0] = NvBootSpiFlashCommand_Read;
            BytesRequested = 4;
        }
        else
        {
        CommandData[0] = NvBootSpiFlashCommand_FastRead;
        CommandData[4] = 0;
        CommandData[5] = 0;
        CommandData[6] = 0;
        CommandData[7] = 0;
        BytesRequested = 5;
        }
    }
    //Address is basically to tell which block and which page to be read
    CommandData[1] = (Address >> 16)& 0xFF;
    CommandData[2] = (Address >> 8)& 0xFF;
    CommandData[3] = (Address)& 0xFF;
    SetReadDummyCycles(0);
    Error = SpiTalkToDevice(CommandData, BytesRequested, pDest, BytesToRead, NV_TRUE);

    return Error;
}
NvBootError
NvBootSpiFlashReadPage(
    const NvU32 Block,
    const NvU32 Page,
    const NvU32 Len,
    uint8_t *Dest)
{
    NvBootError ErrorStatus = NvBootError_Success;
    NvU32 Address;
    NvU32 PageSize;
    NvU32 PagesToRead;
    NvU32 PagesPerBlock;
    NvU32 Block2Read = Block, Page2Read = Page;
    NV_ASSERT(s_pSpiFlashContext);
    NV_ASSERT(Dest);
    // Make sure the Dest is 4-byte aligned
    NV_ASSERT(!((NvU32)Dest & 0x3));
    NV_ASSERT(Page < 
        (1<< ((s_pSpiFlashContext->BlockSizeLog2) - (s_pSpiFlashContext->PageSizeLog2))));

    // read time
    unsigned long funcStartTick = NvBootUtilGetTimeUS();

    // Len parameter  translated to Pages to read in case of Block read
    // and bytes to read in case of continurous mode. Fix for Block boundary.

    // Breaking the request size into PageSize/BlockSize long requests 
    // may be required if there are spi flash BlockSize constraints
    // and will be required for QSPI BFM in TOP RTL sims 
    PageSize = 1 << (s_pSpiFlashContext->PageSizeLog2);    
    PagesToRead = Len/PageSize;
    PagesPerBlock = 1 << ((s_pSpiFlashContext->BlockSizeLog2) - (s_pSpiFlashContext->PageSizeLog2));
    
    if (Len - (PagesToRead * PageSize) != 0)
        PagesToRead++;
    PageSize = 1 << (s_pSpiFlashContext->PageSizeLog2);
    s_pSpiFlashBitInfo->NumPagesRead += PagesToRead;
    s_pSpiFlashBitInfo->LastBlockRead = Block;
    s_pSpiFlashBitInfo->LastPageRead = Page; 

    while(PagesToRead)
    {
        if (Page2Read == PagesPerBlock)
        {
            Block2Read++; 
            Page2Read = 0;
        }
        Address = (Block2Read << (s_pSpiFlashContext->BlockSizeLog2)) + 
                (Page2Read << (s_pSpiFlashContext->PageSizeLog2));

         // Store address of Dest in spi flash context
        s_pSpiFlashContext->CurrentReadBufferAddress = Dest;
        // Store the Read Start time 
        s_pSpiFlashContext->ReadStartTimeinUs = NvBootUtilGetTimeUS();
        //read one page
        ErrorStatus = SpiFlashRead(Address, Dest, PageSize, 
                    s_pSpiFlashContext->ReadCommandTypeFast);
        //store the read status in boot info table
        s_pSpiFlashBitInfo->ReadStatus = (NvU32)ErrorStatus;
        if (ErrorStatus != NvBootError_Success)
        {
             // Reset Read Start time 
            s_pSpiFlashContext->ReadStartTimeinUs = 0;
        }
        Page2Read++;
        Dest += PageSize;
        PagesToRead--;
    }

    /// Update read timestamp and data payload
    s_pSpiFlashBitInfo->ReadTime = NvBootUtilGetTimeUS() - funcStartTick;
    s_pSpiFlashBitInfo->Payload = Len;

    return ErrorStatus;
}
void 
NvBootSpiFlashGetParams(
    const NvU32 ParamIndex,
    NvBootSpiFlashParams **Params)
{
    NV_ASSERT(Params);

    /// Fuse params derivation is bases on configs 0-2
    /// Bct configuration options will be similarly based on configs 
    /// instead of individual/device/function specific parameters. 
    
    /// Datawidth is based on configs supported
    /// ClockSource is based on configs supported
    /// ReadCommandTypeFast is based on configs supported
    /// Clockdivider is based on configs supported
    /// XferMode is based on configs supported
    /// ChipSelectPolarity is fixed @ ActiveLow
    /// PageSizeLog2 is fixed @ 11, EraseSectorSizeLog2 is fixed @ 18, WritePageSizeLog2 is fixed @ 6
    /// EraseTime is fixed @ 1 sec

    /*
     * Extract the configs use from Param Index, which comes from fuses.
     * Configs are defined to simplify boot params options and extract 
     * relevant device specific information.
     */
    s_DefaultSpiFlashParams.SpiConfig = NV_DRF_VAL(SPI_DEVICE, CONFIG, DEV_CONFIG, ParamIndex);

    /* Overwrite the ParamDefaults pointer with real data. */
    *Params = (NvBootSpiFlashParams*)&(s_DefaultSpiFlashParams);
}
NvBool NvBootSpiFlashValidateParams(const NvBootSpiFlashParams *Params)
{
    NvBool ParamsValid = NV_TRUE;

    NV_ASSERT(Params);

   if ( (Params->SpiConfig < Spi_Config_0) ||
        (Params->SpiConfig > Spi_Config_2) )
        return NV_FALSE;

    s_pSpiFlashBitInfo->ParamsValidated = (NvU32)ParamsValid;
    return ParamsValid;
}
void
NvBootSpiFlashGetBlockSizes(
    const NvBootSpiFlashParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_CT_ASSERT(Params);
    NV_ASSERT(Params);
    NV_ASSERT(BlockSizeLog2);
    NV_ASSERT(PageSizeLog2);

    *BlockSizeLog2 = s_pSpiFlashContext->BlockSizeLog2;
    *PageSizeLog2  = s_pSpiFlashContext->PageSizeLog2;
}
NvBootError
NvBootSpiFlashInit(
    const NvBootSpiFlashParams *Params,
    NvBootSpiFlashContext *Context)
{
    NvBootError e = NvBootError_Success;
    NvU32 RegData;
    void *SpiClockTable;

    NV_ASSERT(Context);
    NV_ASSERT(Params);

    NV_ASSERT(Params->SpiConfig >= Spi_Config_0 &&
        Params->SpiConfig <= Spi_Config_2);
    // time Init
    unsigned long funcStartTick = 0;

    NvBootSpiFlashTimeouts * SpiFlashXferTimeouts = (NvBootSpiFlashTimeouts *)(GetSpiFlashConstantsTable(SpiFlashConstantTableIdx_SpiFlashXferTimeoutTable));
    NvU32 * ClockDiv = (NvU32 *)(GetSpiFlashConstantsTable(SpiFlashConstantTableIdx_ClockDivisorsTable));
    NvBootSpiFlashQuadSdrLC * QReadTable = (NvBootSpiFlashQuadSdrLC *)(GetSpiFlashConstantsTable(SpiFlashConstantTableIdx_QReadConfigTable));

    // Stash the Context pointer
    s_pSpiFlashContext = Context;

    s_pSpiFlashContext->XferMode = NvBootSpiXferMode_Pio;

    if (Params->SpiConfig == Spi_Config_0 || Params->SpiConfig == Spi_Config_1)
        s_pSpiFlashContext->BusWidth = NvBootSpiDataWidth_x1;
    else if (Params->SpiConfig == Spi_Config_2)
        s_pSpiFlashContext->BusWidth = NvBootSpiDataWidth_x4;

    if(SpiFlashXferTimeouts)
        s_pSpiFlashContext->DataXferTimeout = (s_pSpiFlashContext->XferMode== NvBootSpiXferMode_Dma) ? SpiFlashXferTimeouts->DmaReadTimeout : SpiFlashXferTimeouts->PioReadTimeout;

    if (Params->SpiConfig == Spi_Config_0 || Params->SpiConfig == Spi_Config_2)
    {
        s_pSpiFlashContext->ClockSource = (NvBootSpiClockSource)CLK_RST_CONTROLLER_CLK_SOURCE_QSPI_0_QSPI_CLK_SRC_PLLP_OUT0;
    }else if (Params->SpiConfig == Spi_Config_1)
    {
        s_pSpiFlashContext->ClockSource = (NvBootSpiClockSource)CLK_RST_CONTROLLER_CLK_SOURCE_QSPI_0_QSPI_CLK_SRC_CLK_M;
    }

    // On FPGA, osc freq=19.2MHz, All IOs @ 20MHz, APB and R5 @ 6.5MHz.
    // Maximum 10MHz for SPI controller works for reliable writes/reads.
    // On silicon, P0 frequency is 38.4MHz and 12MHz is also supported.
    // At any point of time, we need 20MHz or less.
    if(ClockDiv)
        s_pSpiFlashContext->ClockDivider = ClockDiv[Params->SpiConfig];
    if (Params->SpiConfig == Spi_Config_0 || Params->SpiConfig == Spi_Config_1)
    s_pSpiFlashContext->ReadCommandTypeFast = NV_FALSE;
    else if (Params->SpiConfig == Spi_Config_2)
    s_pSpiFlashContext->ReadCommandTypeFast = NV_TRUE;

    // Fixed Spi params
    s_pSpiFlashContext->ChipSelectPolarity = NvBootSpiFlashCSActivePolarity_ActiveLow;
    s_pSpiFlashContext->EraseSectorSizeLog2 = SPI_FLASH_ERASE_PAGE_SIZE_LOG2; /* 256 K */
    s_pSpiFlashContext->EraseTime = SPI_FLASH_ERASE_TIME_MILLISECONDS;
    s_pSpiFlashContext->PageSizeLog2 = SPI_FLASH_PAGE_SIZE_LOG2;
    s_pSpiFlashContext->BlockSizeLog2 = SPI_FLASH_BLOCK_SIZE_LOG2;

    // Read BCT function requires at least two pages per block. Also due to block erase 
    //command's limitation, block size is fixed to 32K
    s_pSpiFlashContext->WritePageSizeLog2 = SPI_FLASH_WRITE_PAGE_SIZE_LOG2;

    // Initialize the ReadStartTimeinUs to zero
    Context->ReadStartTimeinUs = 0;
    if(QReadTable)
        s_pSpiFlashContext->QuadReadDummyCycles = (QReadTable->DummyCycles + QReadTable->ModeCycles);

    s_pSpiFlashBitInfo->InitStatus = 0;
    /// Controller Initialization time
    funcStartTick = NvBootUtilGetTimeUS();

    // Configure the clock source and divider, default is s_SpiClkDiv_20_4
    SpiClockTable = s_SpiClkTable_Init;
    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootClocksEngine(SpiClockTable, TYPE_SINGLE_TABLE));
    // Extract clock table from Spiconfig
    if (Params->SpiConfig == Spi_Config_0)
        SpiClockTable = s_SpiClkDiv_20_4;
    else if (Params->SpiConfig == Spi_Config_1)
        SpiClockTable = s_SpiClkDiv_19_2;
    else if (Params->SpiConfig == Spi_Config_2)
        SpiClockTable = s_SpiClkDiv_51;

    EnableClockSource(Context);
    NV_BOOT_CHECK_ERROR_CLEANUP(NvBootClocksEngine(SpiClockTable, TYPE_SINGLE_TABLE));

    // QSPI IAS section "Pgrogramming Guidelines".
    // "Override SLCG by writing 0x1 to SE_CLK_VR_ON in
    // CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0 register." 
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRD, QSPI_CLK_OVR_ON, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0, RegData);

    // update boot info table
    s_pSpiFlashBitInfo->IsFastRead = (NvU32)s_pSpiFlashContext->ReadCommandTypeFast;
    s_pSpiFlashBitInfo->ClockDivider = (NvU32)s_pSpiFlashContext->ClockDivider;
    s_pSpiFlashBitInfo->ClockSource = (NvU32)s_pSpiFlashContext->ClockSource;
    s_pSpiFlashBitInfo->ClkEnable = CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0_CLK_ENB_QSPI_ENABLE;
    s_pSpiFlashBitInfo->ClkRstStatus = CLK_RST_CONTROLLER_RST_DEV_Y_CLR_0_CLR_QSPI_RST_DISABLE;
    s_pSpiFlashBitInfo->Mode = s_pSpiFlashContext->XferMode;

    //program pin muxes
    e = NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_SpiFlash,
        NvBootPinmuxConfig_Spi);

    s_pSpiFlashBitInfo->InitStatus = (NvU32)e;

    if (e == NvBootError_Success)
    {
        //Configure initial settings
        RegData = SPI_REG_READ32(COMMAND);
    
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, M_S, MASTER, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, MODE, Mode0, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SEL, CS0, RegData);
        RegData = NV_FLD_SET_DRF_NUM(SPI, COMMAND, CS_POL_INACTIVE0, !(Context->ChipSelectPolarity), RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SW_HW, SOFTWARE, RegData);
        //Look at SPI SST/Winbond devices' data sheet. CS pin of flash device is active low
        //To rx/tx, transition CS from high to low, send/rx, transition CS from low to high
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SW_VAL, HIGH, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, IDLE_SDA, DRIVE_LOW, RegData);
        //SPI protocol requires msb of a byte to be sent first
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Bit, LAST, RegData);
        //32bit data in little endian format = 0x12345678. 0x78 gets transmitted first
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Byte, LAST, RegData);
        //Number of bits to be transmitted per packet in unpacked mode = 32
        RegData = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_MAX_BIT_LENGTH, RegData);
        SPI_REG_WRITE32(COMMAND, RegData);
        //Flush Tx fifo 
        SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_FLUSH, FLUSH));
        //Flush Rx fifo
        SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_FLUSH, FLUSH));
        // Program Trimmers
        ProgramTrimmerValues();

        if(s_pSpiFlashContext->BusWidth == NvBootSpiDataWidth_x4)
        {
            QuadModeEnDis((s_pSpiFlashContext->BusWidth == NvBootSpiDataWidth_x4)? NV_TRUE : NV_FALSE);
            RegData = SPI_REG_READ32(COMMAND);
            // Set bus width based on the selection
            RegData = NV_FLD_SET_DRF_NUM(QSPI, COMMAND, INTERFACE_WIDTH, (Context->BusWidth), RegData);
            SPI_REG_WRITE32(COMMAND, RegData);
        }

        s_pSpiFlashBitInfo->DataWidth = s_pSpiFlashContext->BusWidth;
    /// Update Spi flash nit timestamp
    s_pSpiFlashBitInfo->QspiInit = NvBootUtilGetTimeUS() - funcStartTick;

    }

fail:
    return e;
}

NvBootDeviceStatus NvBootSpiFlashQueryStatus(void)
{ // Check the status of pending operations.

    NvBootDeviceStatus Status = NvBootDeviceStatus_Idle;

    NV_ASSERT(s_pSpiFlashContext);

    if (s_pSpiFlashBitInfo->ReadStatus == NvBootError_HwTimeOut)
    {
        //timeout has already been detected during transfer and appropriate action for disabling the hw has already been taken
        Status = NvBootDeviceStatus_ReadFailure;
    }

    return Status;
}
void NvBootSpiFlashShutdown(void)
{
    // Set the context to NULL saying the driver has to be Initialized again before using.
    s_pSpiFlashContext = NULL;
    // Enable Reset for SPI controller
    NvBootResetSetEnable(NvBootResetDeviceId_SpiId, NV_TRUE);
    // Enable Reset for APB DMA 
#ifdef NvBootResetDeviceId_ApbDmaId    
    NvBootResetSetEnable(NvBootResetDeviceId_ApbDmaId, NV_TRUE);
#endif
    // Disable the clock for SPI controller
    NvBootClocksSetEnable(NvBootClocksClockId_SpiId, NV_FALSE);
    // Disable the clock for APB DMA
#ifdef NvBootClocksClockId_ApbDmaId
    NvBootClocksSetEnable(NvBootClocksClockId_ApbDmaId, NV_FALSE);
#endif
}

#if 0
static NvBootError ChipErase(uint8_t ManId, uint8_t DevId)
{
    return NvBootError_Unimplemented;
}
//    static NvU32 PhysicalAddr ;
#endif

NvBootError
NvBootSpiFlashWritePage(
    const NvU32 Block, 
    const NvU32 Page, 
    const NvU32 Len, 
    uint8_t *Dest)
{
    NvU32 PageSize = 0;
    NvBootError e;
    NvU32 Address;
    NvU32 PagesToWrite;
    //static NvU32 ewratio = 0;
    NvU32 AddrOffEraseSectStart = 0;

    NV_ASSERT(s_pSpiFlashContext);
    NV_ASSERT(Dest);
    // Make sure the Dest is 4-byte aligned
    NV_ASSERT(!((NvU32)Dest & 0x3));
    NV_ASSERT(Page <
        (1<< ( (s_pSpiFlashContext->BlockSizeLog2) - (s_pSpiFlashContext->PageSizeLog2) )));


    Address = (Block  << (s_pSpiFlashContext->BlockSizeLog2)) +
                (Page << (s_pSpiFlashContext->PageSizeLog2));

    
   
    //FIXME: If Page number is  an integral multiple of SectorSize/BlockSize
    AddrOffEraseSectStart = (Address/(1 << s_pSpiFlashContext->EraseSectorSizeLog2) );
    AddrOffEraseSectStart = Address - (AddrOffEraseSectStart * (1 << s_pSpiFlashContext->EraseSectorSizeLog2));	  
    if ( AddrOffEraseSectStart == 0)
    {
        //Enable Write
        PopulateCommand(CommandData, NvBootSpiFlashCommand_WriteEnableEn, 0, 0, 0, 0);
        NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, 0, 0, NV_FALSE));
        //Sector Erase - program A23:A0 TBD
        PopulateCommand(CommandData, NvBootSpiFlashCommand_SectorErase, (uint8_t)((Address >> 16) & 0xFF),
            (uint8_t)((Address >> 8) & 0xFF),
            (uint8_t)((Address) & 0xFF), 0);
        NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 4, 0, 0, NV_FALSE));
        NvBootUtilWaitUS(s_pSpiFlashContext->EraseTime * 1000); //Use Erase Sector Time in milliseconds

    }

    //Check Busy not done yet
    PopulateCommand(CommandData, NvBootSpiFlashCommand_ReadStatus, 1, 0, 0, 0);
    NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, DataRead, 1, NV_FALSE));

    while ((DataRead[0] & STATUS_REGISTER_WRITE_IN_PROGRESS_MASK) & 0x1)
    {
        goto fail;
    }
 
    PageSize = 1 << (s_pSpiFlashContext->WritePageSizeLog2);
    PagesToWrite = Len/PageSize;
    if (Len - (PagesToWrite * PageSize) != 0)
        PagesToWrite++;

    while(PagesToWrite > 0)
    {
        //Enable Write
        PopulateCommand(CommandData, NvBootSpiFlashCommand_WriteEnableEn, 0, 0, 0, 0);
        NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, 0, 0, NV_FALSE));

        // Write data
        PopulateCommand(CommandData, NvBootSpiFlashCommand_Write, (uint8_t)((Address >> 16) & 0xFF),
            (uint8_t)((Address >> 8) & 0xFF),
            (uint8_t)((Address) & 0xFF), 0);

        // NV_BOOT_CHECK_ERROR_CLEANUP(SpiWriteToDevice(CommandData, 4, Dest, (1<<(s_pSpiFlashContext->WritePageSizeLog2))));
        SpiWriteToDevice(CommandData, 4, Dest, (1<<(s_pSpiFlashContext->WritePageSizeLog2)));
        Dest += (1<<(s_pSpiFlashContext->WritePageSizeLog2));
        Address += (1<<(s_pSpiFlashContext->WritePageSizeLog2));
        PagesToWrite--;
        //Check Busy not done yet
        PopulateCommand(CommandData, NvBootSpiFlashCommand_ReadStatus, 1, 0, 0, 0);
        NV_BOOT_CHECK_ERROR_CLEANUP(SpiTalkToDevice(CommandData, 1, DataRead, 1, NV_FALSE));

        if ((DataRead[0] & STATUS_REGISTER_WRITE_IN_PROGRESS_MASK) & 0x1)
        {
            // Change this time to timeout for PIO writes.
            NvBootUtilWaitUS(s_pSpiFlashContext->DataXferTimeout);
        }
    }

    return NvBootError_Success;
 fail:
    return NvBootError_WriteFailed;
}

NvBootError NvBootSpiFlashGetReaderBuffersBase(uint8_t** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    NV_CT_ASSERT(ReaderBuffersBase);
    NV_CT_ASSERT(Alignment);
    NV_CT_ASSERT(Bytes);
    return NvBootError_Unimplemented;
}

NvBootError NvBootSpiFlashPinMuxInit(const void * Params)
{
    NV_CT_ASSERT(Params);
    
    /*
     * Upper layer doesn't check for NvBootError_Unimplemented
     * and considers it an error. Not to break existing functionality,
     * return NvBootError_Success
     */
    return NvBootError_Success;
}

