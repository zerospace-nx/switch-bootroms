/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "stdbool.h"
#include "nvrm_drf.h"
#include "nvboot_bit.h"
#include "nvboot_util_int.h"
#include "nvboot_version_rom.h"
#include "project.h"
#include "nvboot_uart_int.h"
#include "nvboot_strap_int.h"
#include "nvboot_prod_uart_int.h"
#include "nvboot_device_int.h"
#include <string.h>

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
static const uint8_t PromptMsgConst[31] = 
    "\n\rNV Prod Boot T214 WXYZ.HIJK\n\r";

static NvBootProdUartContext *s_ProdUartContext = NULL;
static NvBootProdUartParams s_ProdUartDefaultParams;


void NvBootProdUartGetParams(
    const NvU32 ParamIndex  __attribute__ ((unused)),
    NvBootProdUartParams **Params)
{
    s_ProdUartDefaultParams.PageSizeLog2  = PROD_UART_PAGESIZELOG2;
    s_ProdUartDefaultParams.BlockSizeLog2  = PROD_UART_BLOCKSIZELOG2;
    *Params = &s_ProdUartDefaultParams;
}

NvBool NvBootProdUartValidateParams(
    const NvBootProdUartParams *Params  __attribute__ ((unused)))
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
    size_t nRead, nWritten;
    NvU32 i;

    uint8_t PromptMsg[sizeof(PromptMsgConst)];

    // Stash the context
    s_ProdUartContext = pProdUartContext;
    s_ProdUartContext->PageSizeLog2  = Params->PageSizeLog2;
    s_ProdUartContext->BlockSizeLog2  = Params->BlockSizeLog2;

    (void) memcpy(PromptMsg, PromptMsgConst, sizeof(PromptMsgConst));
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

    NvBootUartInit(NvBootClocksGetOscFreq(), true);

    // send the prompt
    (void) NvBootUartPollingWrite(PromptMsg, sizeof(PromptMsg),  &nWritten);

    NvU8 * RxBuf = (NvU8 *) NV_ADDRESS_MAP_IRAM_C_BASE;

    // receive 64K
    const size_t ramSize = NV_ADDRESS_MAP_DATAMEM_IRAM_C_SIZE;
    (void) NvBootUartPollingRead(RxBuf, ramSize, &nRead);

    return NvBootError_Success;
}

NvBootError NvBootProdUartReadPage(
    const NvU32 Block,
    const NvU32 Page,
    const NvU32 Len,
    NvU8 *Dest)
{

    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < (1 << (s_ProdUartContext->BlockSizeLog2 - s_ProdUartContext->PageSizeLog2)));
    
    (void) memcpy(Dest, (uint8_t*)(NV_ADDRESS_MAP_IRAM_C_BASE
                                  + (Block << s_ProdUartContext->BlockSizeLog2)
                                  + (Page << s_ProdUartContext->PageSizeLog2)),
                            Len);

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

NvBootError NvBootProdUartWritePage(
    const NvU32 Block  __attribute__ ((unused)),
    const NvU32 Page  __attribute__ ((unused)),
    const NvU32 Len  __attribute__ ((unused)),
    uint8_t *Dest  __attribute__ ((unused)))
{
    // Unimplemented
    return NvBootError_Unimplemented;
}

NvBootError NvBootProdUartGetReaderBuffersBase(
    uint8_t** ReaderBuffersBase  __attribute__ ((unused)),
    const NvU32 Alignment  __attribute__ ((unused)),
    const NvU32 Bytes  __attribute__ ((unused)))
{
    return NvBootError_Unimplemented;
}

NvBootDeviceStatus NvBootProdUartPinMuxInit(const void *Params  __attribute__ ((unused)))
{
    /*
     * Upper layer doesn't check for NvBootError_Unimplemented
     * and considers it an error. Not to break existing functionality,
     * return NvBootError_Success
     */
    return NvBootError_Success;
}

