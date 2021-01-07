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
#include "arahb_arbc.h"
#include "arapbdma.h"
#include "arapbdmachan.h"
#include "arspi.h"
#include "arqspi.h"

#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_device_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_spi_flash_context.h"
#include "nvboot_spi_flash_int.h"
#include "nvboot_spi_flash_param.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_util_int.h"
#include "project.h"


// Wrapper macros for reading/writing from/to SPI

#define SPI_REG_READ32(reg) \
        NV_READ32(NV_ADDRESS_MAP_APB_QSPI_BASE + (QSPI_##reg##_0))
        
#define SPI_REG_WRITE32(reg, val) \
    do { \
        NV_WRITE32((NV_ADDRESS_MAP_APB_QSPI_BASE + (QSPI_##reg##_0)), (val)); \
    } while (0)


enum { SPI_FLASH_NORMAL_READ_COMMAND = 0x03};
enum { SPI_FLASH_FAST_READ_COMMAND = 0x0B};

#define SPI_MAX_BIT_LENGTH 31  // 32 bit transfer per dma/pio packet in packed/unpacked mode
#define SPI_8Bit_BIT_LENGTH 7  // 8 bit transfer per dma/pio  packet in packed/unpacked mode
#define SPI_FIFO_DEPTH 32      // 32 * 32 bits = 32 entries of 4 bytes each

#define BYTES_PER_WORD 4
// Read time out of 200 milli seconds.
#define SPI_HW_TIMEOUT 200000

/* Note that BootROM SPI code remains largely unchanged for several chips.
   The way the SPI controller is used is undocumented, but working. The controller
   is set up for DMA, but no apbdma channel is set up, and data is read with
   pio from the RX_FIFO. */

static NvBootSpiFlashContext * s_pSpiFlashContext = NULL;

// Boot Info table.
extern NvBootInfoTable BootInfoTable;

// Pointer to spi flash Bit info.
static NvBootSpiFlashStatus* s_pSpiFlashBitInfo = \
                        &BootInfoTable.SecondaryDevStatus.SpiStatus;

/**
  * One set of defaults Parameters
  */
static NvBootSpiFlashParams s_DefaultSpiFlashParams;

#define SPI_FLASH_BLOCK_SIZE_LOG2 15  /* 32*1024 */
#define SPI_FLASH_PAGE_SIZE_LOG2  11  /*  2*1024 */

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

void SetSpiChipSelectLevel(NvBool IsLevelHigh)
{
    //Is effective only when SW CS is being used
    NvU32 CmdRg = SPI_REG_READ32(COMMAND);
    
    CmdRg = NV_FLD_SET_DRF_NUM(SPI, COMMAND,  CS_SW_VAL, IsLevelHigh, CmdRg);
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
	if (NvBootUtilElapsedTimeUS(TransferStartTimeinUs) > SPI_HW_TIMEOUT)
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

NvBootError HwSpiWriteInTransmitFifo(NvU8 *pTxBuff, NvU32 WordsOrBytesToWrite,NvU32 DmaPktLen,NvU32 WriteStartTimeinUs)
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
NvBootError SpiHwProcWrite(NvU8 *pWriteBuffer, NvU32 BytesToWrite)
{
    NvBootError Error = NvBootError_Success;
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
	SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_FLUSH, FLUSH));
	//Enable Tx
	Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);
    //Get start time
	WriteStartTimeinUs = NvBootUtilGetTimeUS();
    if (WordsToWrite)
    {
        //something to be transmitted
        if(WordsToWrite > SPI_FIFO_DEPTH)
        {
            Error = HwSpiWriteInTransmitFifo(pWriteBuffer,SPI_FIFO_DEPTH,DmaPktLen,WriteStartTimeinUs);
            WordsToWrite = WordsToWrite - SPI_FIFO_DEPTH; 
        }
        else
        {
           Error = HwSpiWriteInTransmitFifo(pWriteBuffer,WordsToWrite,DmaPktLen,WriteStartTimeinUs);
           WordsToWrite = 0;
        }

        if (Error == NvBootError_Success)
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
                Error = HwSpiWriteInTransmitFifo(pWriteBuffer,WordsToWrite,DmaPktLen,WriteStartTimeinUs);
            }
            //Make sure spi hw is ready at the end and there is no timeout
            Error = SpiChkHwRdyOrTimeout(WriteStartTimeinUs);
        }
    }//end if (WordsToWrite)
    //Disable Tx
    Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Tx_EN, DISABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);
    //clear the status register
    SPI_REG_WRITE32( FIFO_STATUS, SPI_REG_READ32( FIFO_STATUS));
	//return Error local variable
	return Error;
}

NvBootError SpiReadFromReceiveFifo(NvU8 *pRxBuff, NvU32 WordsOrBytesToRead,NvU32 DmaPktLen,NvU32 ReadStartTimeinUs)
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
            (*pRxBuff) = (NvU8)(Reg_Val);
        }

		//increment buffer pointer 
		pRxBuff += DmaPktLen;
		//decrement requested number of words
        WordsOrBytesToRead--;
    }
    return(Error); 
}
NvBootError SpiHwProcRead(NvU8 *pReadBuffer, NvU32 BytesToRead)
{
    NvU32 ReadStartTimeinUs;
    NvU32 DmaBlkSz,DmaPktLen;
	NvU32 Reg_Val;
    NvBootError Error = NvBootError_Success;
    
    NV_ASSERT(pReadBuffer);
    NV_ASSERT(BytesToRead);

    Reg_Val = SPI_REG_READ32(COMMAND); 
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
    SPI_REG_WRITE32(COMMAND, Reg_Val);  
	//Set dma block size. 
	SPI_REG_WRITE32( DMA_BLK_SIZE,(DmaBlkSz - 1));
    //Flush Rx fifo before enabling receive 
	SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_FLUSH, FLUSH));
	//Enable Rx
	Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);

        //Before enabling DMA mode, be sure to wait 5 spi_clk cycles
        //Chose 5 MHz as minimum clk. (has worked in the past without this)
        NvBootUtilWaitUS(1);

    //Get start time
	ReadStartTimeinUs = NvBootUtilGetTimeUS();
	//Enable DMA mode
	Reg_Val = NV_FLD_SET_DRF_DEF(SPI, DMA_CTL, DMA_EN, ENABLE, 
                        SPI_REG_READ32(DMA_CTL));
    SPI_REG_WRITE32( DMA_CTL, Reg_Val);
    //Try reading data from fifo
    //Dma is already enabled so keep reading if rx fifo is non empty and hw is not timed out
    //Assumption is that pReadBuffer is pointing to a buffer which is large enough to hold requested
    //number of bytes
	Error = SpiReadFromReceiveFifo(pReadBuffer, DmaBlkSz,DmaPktLen,ReadStartTimeinUs);
    //Make sure spi hw is ready at the end and there is no timeout
    Error = SpiChkHwRdyOrTimeout(ReadStartTimeinUs);
	//Disable Rx
	Reg_Val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, Rx_EN, DISABLE, 
                        SPI_REG_READ32(COMMAND));
    SPI_REG_WRITE32( COMMAND, Reg_Val);
	//DMA_EN bits get cleared by hw if all the data was tranferred successfully else s/w will disbale it upon detection of h/w timeout
	//clear the status register
    SPI_REG_WRITE32( FIFO_STATUS, SPI_REG_READ32( FIFO_STATUS));
	//return Error local variable
	return Error;
}
NvBootError SpiTalkToDevice(NvU8 *pWriteBuffer, NvU32 BytesToWrite, NvU8 *pReadBuffer, NvU32 BytesToRead)
{
    NvBootError Error = NvBootError_Success;

    //set cs sw val to low 
    SetSpiChipSelectLevel(NV_FALSE);
    //Send/Rx
    if(BytesToWrite)
    {
        Error =  SpiHwProcWrite(pWriteBuffer, BytesToWrite);
    }
    if((Error == NvBootError_Success) && BytesToRead)
    {
        Error = SpiHwProcRead(pReadBuffer, BytesToRead);
    }
    //set cs sw val to high
    SetSpiChipSelectLevel(NV_TRUE);
    return Error;
}


static NvBootError 
SpiFlashRead(NvU32 Address, NvU8 *pDest, NvU32 BytesToRead, NvBool IsFastRead)
{
    NvBootError Error;
    NvU8 CommandData[8];
    NvU32 BytesRequested;

    NV_ASSERT(pDest);
	//prepare buffer as per spi devices' data sheet for sending out read request to the device

	if(!IsFastRead)
	{
		CommandData[0] = SPI_FLASH_NORMAL_READ_COMMAND;
		BytesRequested = 4;
        //Command details
        //Op Code Cycle       = 1
        //AddressCycle(s)     = 3
        //Dummy Cycle(s)      = 0
        //Maximum Frequency   = SST: 20MHz ; WinBond : 20MHz
	}
	else
    {
        CommandData[0] = SPI_FLASH_FAST_READ_COMMAND;
        CommandData[4] = 0;
        CommandData[5] = 0;
        CommandData[6] = 0;
        CommandData[7] = 0;
        BytesRequested = 5;
        //Command details
        //Op Code Cycle       = 1
        //AddressCycle(s)     = 3
        //Dummy Cycle(s)      = 1
        //Maximum Frequency   = SST: 40MHz ; WinBond : 80MHz
	}
	//Address is basically to tell which block and which page to be read
    CommandData[1] = (Address >> 16)& 0xFF;
    CommandData[2] = (Address >> 8)& 0xFF;
    CommandData[3] = (Address)& 0xFF;

    Error = SpiTalkToDevice(CommandData, BytesRequested, pDest, BytesToRead);

    return Error;
}
NvBootError
NvBootSpiFlashReadPage(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest)
{
    NvBootError ErrorStatus = NvBootError_Success;
    NvU32 Address;
    NvU32 PageSize;
  
    NV_ASSERT(s_pSpiFlashContext);
    NV_ASSERT(Dest);
    // Make sure the Dest is 4-byte aligned
    NV_ASSERT(!((NvU32)Dest & 0x3));
    NV_ASSERT(Page < 
        (1<< ((s_pSpiFlashContext->BlockSizeLog2) - (s_pSpiFlashContext->PageSizeLog2))));

	s_pSpiFlashBitInfo->NumPagesRead++;
    s_pSpiFlashBitInfo->LastBlockRead = Block;
    s_pSpiFlashBitInfo->LastPageRead = Page;  
	
	Address = (Block  << (s_pSpiFlashContext->BlockSizeLog2)) + 
                (Page << (s_pSpiFlashContext->PageSizeLog2));
    PageSize = 1 << (s_pSpiFlashContext->PageSizeLog2);
	// Store address of Dest in sdmmc context
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
    return ErrorStatus;
}
void 
NvBootSpiFlashGetParams(
    const NvU32 ParamIndex,
    NvBootSpiFlashParams **Params)
{
    NV_ASSERT(Params);

    // Param Index comes from Fuses and we dont have any fuses for SPI Flash.
    // Maximum crystal freq is 26MHz and as clockM as clock source we need to 
    // operate on less than 20MHz.

	// integer part of 7.1 clock divider, [7:1]= integer part; [0]= fractional part i.e. div by .5
    s_DefaultSpiFlashParams.ClockDivider = 2; 
	//clock source
    s_DefaultSpiFlashParams.ClockSource = CLK_RST_CONTROLLER_CLK_SOURCE_QSPI_0_QSPI_CLK_SRC_CLK_M;
	//normal/fast read command opcode to be issued to the device
    s_DefaultSpiFlashParams.ReadCommandTypeFast = NV_FALSE;
    //Get page size option
    s_DefaultSpiFlashParams.PageSize2kor16k = NV_DRF_VAL(SPI_DEVICE, CONFIG,
                                    PAGE_SIZE_2KOR16K, ParamIndex);

    //store the default params into boot info table
	s_pSpiFlashBitInfo->ClockDivider = 
        (NvU32)s_DefaultSpiFlashParams.ClockDivider;
    s_pSpiFlashBitInfo->ClockSource = s_DefaultSpiFlashParams.ClockSource;
    s_pSpiFlashBitInfo->IsFastRead =
        (NvU32)s_DefaultSpiFlashParams.ReadCommandTypeFast;

    s_pSpiFlashBitInfo->NumPagesRead = 0;
    s_pSpiFlashBitInfo->LastBlockRead = 0;
    s_pSpiFlashBitInfo->LastPageRead = 0;
    s_pSpiFlashBitInfo->BootStatus = (NvU32)NvBootDeviceStatus_Idle; 
    s_pSpiFlashBitInfo->InitStatus = (NvU32)NvBootError_Success;
    s_pSpiFlashBitInfo->ReadStatus = (NvU32)NvBootError_Success;
    s_pSpiFlashBitInfo->ParamsValidated = (NvU32)NV_TRUE;

    /* Overwrite the ParamDefaults pointer with real data. */
    *Params = (NvBootSpiFlashParams*)&(s_DefaultSpiFlashParams);
}
NvBool NvBootSpiFlashValidateParams(const NvBootSpiFlashParams *Params)
{
    NvBool ParamsValid = NV_TRUE;

    NV_ASSERT(Params);

    // Validate the Clock divider by checking for '0x0' and whether it
    // fits within 7 bits. Checking for integer part of divider
    if ((Params->ClockDivider == 0) || (Params->ClockDivider > 0x7F))
    {
        ParamsValid = NV_FALSE;
    }
    //Clock sources supported by spi driver in bootrom are pllp and clkm only
    if ((Params->ClockSource != NvBootSpiClockSource_PllPOut0) && \
        (Params->ClockSource != NvBootSpiClockSource_ClockM))
    {
        ParamsValid = NV_FALSE;
    }
    s_pSpiFlashBitInfo->ParamsValidated = (NvU32)ParamsValid;
    return ParamsValid;
}
void
NvBootSpiFlashGetBlockSizes(
    const NvBootSpiFlashParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
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
    NvBootError Error = NvBootError_Success;
	NvU32 RegData;
    
    NV_ASSERT(Context);
    NV_ASSERT(Params);
    NV_ASSERT(((Params->ClockDivider != 0) && (Params->ClockDivider < 0x80)));

	// Stash the Context pointer
    s_pSpiFlashContext = Context;
    // Initialize the ReadStartTimeinUs to zero
    Context->ReadStartTimeinUs = 0;   
    // Initialize the fast type read 
    Context->ReadCommandTypeFast = Params->ReadCommandTypeFast;

    //Since bootrom data buffer is of 16K, page size of 16K is expected 
    //to improve performance
    Context->PageSizeLog2  = (Params->PageSize2kor16k)? NVBOOT_MAX_PAGE_SIZE_LOG2: SPI_FLASH_PAGE_SIZE_LOG2;
     // Read BCT function requires at least two pages per block. Also due to block erase command's limitation, block size is fixed to 32K
    Context->BlockSizeLog2 = SPI_FLASH_BLOCK_SIZE_LOG2;    

	//update boot info table
    s_pSpiFlashBitInfo->ClockDivider = (NvU32)Params->ClockDivider;
    s_pSpiFlashBitInfo->ClockSource = (NvU32)Params->ClockSource;
    s_pSpiFlashBitInfo->IsFastRead = (NvU32)Params->ReadCommandTypeFast;

    // Reset the SPI controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SpiId, NV_TRUE);   
    // Enable Reset for APB DMA ( There is no other APB peripheral uses APB DMA for data transfer)
    NvBootResetSetEnable(NvBootResetDeviceId_ApbDmaId, NV_TRUE);
    // Configure the clock source and divider
    NvBootClocksConfigureClock(
        NvBootClocksClockId_SpiId,
        NVBOOT_CLOCKS_7_1_DIVIDER_BY(Params->ClockDivider,0),
        Params->ClockSource);
    // Enable the Clock for SPI Controller
    NvBootClocksSetEnable(NvBootClocksClockId_SpiId, NV_TRUE);
    // Enable the Clock APB DMA
    NvBootClocksSetEnable(NvBootClocksClockId_ApbDmaId, NV_TRUE);
    // Remove the reset to the SPI Controller
    NvBootResetSetEnable(NvBootResetDeviceId_SpiId, NV_FALSE);
    // Remove the reset to the APB DMA
    NvBootResetSetEnable(NvBootResetDeviceId_ApbDmaId, NV_FALSE);
	//program pin muxes
    Error = NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_SpiFlash, 
                                            NvBootPinmuxConfig_Spi);

    s_pSpiFlashBitInfo->InitStatus = (NvU32)Error;

    if (Error == NvBootError_Success)
    {
        //Configure initial settings
        RegData = SPI_REG_READ32(COMMAND);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, M_S, MASTER, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, MODE, Mode0, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SEL, CS0, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE0, DEFAULT, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SW_HW, SOFTWARE, RegData);
        //Look at SPI SST/Winbond devices' data sheet. CS pin of flash device is active low
        //To rx/tx, transition CS from high to low, send/rx, transition CS from low to high
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SW_VAL, HIGH, RegData);
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, IDLE_SDA, DRIVE_LOW, RegData);
        //SPI protocol requires msb of a byte to be sent first
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Bit, LAST, RegData);
        //32bit data in little endian format = 0x12345678. 0x78 gets transmitted first.
        RegData = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Byte, LAST, RegData);
        //Number of bits to be transmitted per packet in unpacked mode = 32
        RegData = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,SPI_MAX_BIT_LENGTH, RegData);
        SPI_REG_WRITE32(COMMAND, RegData);
        //configure timing register
        RegData = SPI_REG_READ32(TIMING_REG2);
        RegData = NV_FLD_SET_DRF_DEF(SPI, TIMING_REG2,CS_ACTIVE_BETWEEN_PACKETS_0,DEFAULT, RegData);
        SPI_REG_WRITE32(TIMING_REG2, RegData);
        //Flush Tx fifo 
	    SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_FLUSH, FLUSH));
        //Flush Rx fifo
	    SpiHwFlushFifos(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_FLUSH, FLUSH));
    }
    return Error;
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

    s_pSpiFlashBitInfo->BootStatus = (NvU32)Status;
    return Status;
}
void NvBootSpiFlashShutdown(void)
{
	// Set the context to NULL saying the driver has to be Initialized again before using.
    s_pSpiFlashContext = NULL;
    // Enable Reset for SPI controller
    NvBootResetSetEnable(NvBootResetDeviceId_SpiId, NV_TRUE);
    // Enable Reset for APB DMA 
    NvBootResetSetEnable(NvBootResetDeviceId_ApbDmaId, NV_TRUE);
    // Disable the clock for SPI controller
    NvBootClocksSetEnable(NvBootClocksClockId_SpiId, NV_FALSE);
    // Disable the clock for APB DMA
    NvBootClocksSetEnable(NvBootClocksClockId_ApbDmaId, NV_FALSE);
}


NvBootError NvBootSpiFlashGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
