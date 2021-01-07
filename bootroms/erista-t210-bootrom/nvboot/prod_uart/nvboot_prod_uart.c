/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "nvboot_bit.h"
#include "nvboot_util_int.h"
#include "nvboot_version_rom.h"
#include "project.h"
#include "nvboot_uart_int.h"
#include "nvboot_prod_uart_int.h"
#include "nvboot_device_int.h"

/** ProdUart is a way for ATE to send test vectors even in production mdoe
 * 1. Generate a real BCT+BL image with no DevParams.
 * 2. Set the Prod Uart strap while in Production Mode
 * 3. Observe that you recieve the PromptMsg.
 * 4. Load BCT+BL to IRAM_C (0x40020000). 
 *    Note: Bct page and block size should match BL page and Block size.
 *    Use Page Size: 512 bytes and Block Size: 16384 bytes.
 */

#define ASCII_HEX_DIGIT(x) (((x) < 10) ? ((x) + '0') : ((x) - 10 + 'A')) 

// Hardcode index into PeomptMsg where BootROM version will be updated.
#define MSG_VERSION_INDEX	0x14
const static NvU8 PromptMsgConst[] = 
{
    '\n',
    '\r',
    'N',
    'V',
    ' ',
    'P',
    'r',
    'o',
    'd',
    ' ',
    'B',
    'o',
    'o',
    't',
    ' ',
    'T',
    '2',
    '1',
    '0',
    ' ',
    'W',
    'X',
    'Y',
    'Z',
    '.',
    'H',
    'I',
    'J',
    'K',
    '\n',
    '\r'
}; 

static NvBootProdUartContext *s_ProdUartContext = NULL;
static NvBootProdUartParams s_ProdUartDefaultParams;

void NvBootProdUartGetParams(
    const NvU32 ParamIndex,
    NvBootProdUartParams **Params)
{
    s_ProdUartDefaultParams.PageSizeLog2  = PROD_UART_PAGESIZELOG2;
    s_ProdUartDefaultParams.BlockSizeLog2  = PROD_UART_BLOCKSIZELOG2;
    *Params = &s_ProdUartDefaultParams;
}

NvBool NvBootProdUartValidateParams(
    const NvBootProdUartParams *Params)
{
    return NV_TRUE;
}

void NvBootProdUartGetBlockSizes(
    const NvBootProdUartParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    *BlockSizeLog2 = Params->BlockSizeLog2;
    *PageSizeLog2  = Params->PageSizeLog2;
}

NvBootError NvBootProdUartInit(
    const NvBootProdUartParams *Params,
    NvBootProdUartContext *pProdUartContext)
{
    NvU32 nRead, nWritten;
    NvU8 *RxBuf;
    NvU32 RxBufOffset = 0;
    NvU32 paddedBCTLength, blockLength, pageLength, payloadLength;
    NvU32 BytesToBlStart, BlocksToBlStart;
    NvU32 i;

    NvU8 PromptMsg[sizeof(PromptMsgConst)];

    // Stash the context
    s_ProdUartContext = pProdUartContext;
    s_ProdUartContext->PageSizeLog2  = Params->PageSizeLog2;
    s_ProdUartContext->BlockSizeLog2  = Params->BlockSizeLog2;

    (void) NvBootUtilMemcpy(PromptMsg, PromptMsgConst, sizeof(PromptMsgConst));
    // As a sanity check we test the '.'
    i = MSG_VERSION_INDEX;
    NV_ASSERT(PromptMsg[MSG_VERSION_INDEX + 4] == '.');
    //Load BootROM version into PromptMsg
    i = MSG_VERSION_INDEX;
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 28) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 24) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 20) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 16) & 0x0F);
    i++; // '.' stays
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 12) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >>  8) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >>  4) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >>  0) & 0x0F);

    NvBootUartInit(NvBootClocksGetOscFreq(), NV_TRUE);

    // send the prompt
    (void) NvBootUartPollingWrite(PromptMsg, sizeof(PromptMsg),  &nWritten);
 
    RxBuf = (NvU8 *) PROD_UART_DATA_LOCATION;
    // Size of a BCT is block length in bytes
    blockLength = 1 << s_ProdUartContext->BlockSizeLog2;
    pageLength = 1 << s_ProdUartContext->PageSizeLog2;
    paddedBCTLength = blockLength;
    // receive one BCT, with padding
    (void) NvBootUartPollingRead(RxBuf, paddedBCTLength, &nRead);

    RxBufOffset += nRead;

    // read rest of data before payload starts
    // using just the block size here will be ok for images buildimage generates there
    // may be valid images that don't align like this, but ATE will use buildimage
    BlocksToBlStart = ((NvBootConfigTable *)RxBuf)->BootLoader[0].StartBlock - 1;
    BytesToBlStart = BlocksToBlStart * blockLength;

    // We only use IRAM_C, so limit the payloadLength if its too long.
    // This doesn't neccesarily fix the invalid image, just stops it.
    if(RxBufOffset + BytesToBlStart > NV_ADDRESS_MAP_DATAMEM_IRAM_C_SIZE) {
        return NvBootError_DeviceResponseError;
    }

    (void) NvBootUartPollingRead(&RxBuf[RxBufOffset], BytesToBlStart, &nRead);

    RxBufOffset += nRead;

    // use page size with length to figure out how much to read
    payloadLength = ((NvBootConfigTable *)RxBuf)->BootLoader[0].Length;
    payloadLength = ((payloadLength / pageLength) + 1) * pageLength;

    // We only use IRAM_C, so limit the payloadLength if its too long.
    // This doesn't neccesarily fix the invalid image, just stops it.
    if(RxBufOffset + payloadLength > NV_ADDRESS_MAP_DATAMEM_IRAM_C_SIZE) {
        payloadLength = NV_ADDRESS_MAP_DATAMEM_IRAM_C_SIZE - RxBufOffset;
    }

    // actually get the data
    (void) NvBootUartPollingRead(&RxBuf[RxBufOffset], payloadLength, &nRead);

    return NvBootError_Success;
}

NvBootError NvBootProdUartReadPage(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest)
{
    const NvU32 NumBytesToRead = (1 << s_ProdUartContext->PageSizeLog2);

    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < (1 << (s_ProdUartContext->BlockSizeLog2 - s_ProdUartContext->PageSizeLog2)));
    

    (void) NvBootUtilMemcpy(Dest, (NvU8*)(PROD_UART_DATA_LOCATION
                                  + (Block << s_ProdUartContext->BlockSizeLog2)
                                  + (Page << s_ProdUartContext->PageSizeLog2)),
                            NumBytesToRead);

    return NvBootError_Success;
}

NvBootDeviceStatus NvBootProdUartQueryStatus(void)
{
    return NvBootDeviceStatus_Idle;
}

void NvBootProdUartShutdown(void)
{
    // Unimplemented
}

NvBootError NvBootProdUartGetReaderBuffersBase(
    NvU8** ReaderBuffersBase,
    const NvU32 Alignment,
    const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}
