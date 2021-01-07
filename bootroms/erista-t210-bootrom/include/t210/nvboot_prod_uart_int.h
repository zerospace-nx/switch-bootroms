/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef PROD_UART_INT_H
#define PROD_UART_INT_H
#include "nvboot_device_int.h"

#define PROD_UART_DATA_LOCATION NV_ADDRESS_MAP_IRAM_C_BASE
// Block size 16384
#define PROD_UART_BLOCKSIZELOG2 14
// Page size 512
#define PROD_UART_PAGESIZELOG2 9

typedef struct NvBootProdUartContextRec
{
    NvU32 PageSizeLog2;
    NvU32 BlockSizeLog2;
}
NvBootProdUartContext;

typedef struct NvBootProdUartParamsRec
{
    NvU32 PageSizeLog2;
    NvU32 BlockSizeLog2;
} NvBootProdUartParams;

/**
 * Gets parameters to configure interface. There is only one set of parameters
 * for
 *
 * @param ParamIndex not used, there is only one set of parameters
 * @param Params double pointer that will return the parameters
 */
void NvBootProdUartGetParams(
    const NvU32 ParamIndex,
    NvBootProdUartParams **Params);

/**
 * For ProdUart, ValidateParams always returns NV_TRUE
 *
 * @param Params pointer to params, unused for ProdUart
 *
 * @retval NV_TRUE the only value this fuction will return
 */
NvBool NvBootProdUartValidateParams(
    const NvBootProdUartParams *Params);

/**
 * Queries the block and page sizes for this device from Params in units of
 * log2(bytes).
 *
 * @param Params Pointer to Param info.
 * @param BlockSizeLog2 returns block size in log2 scale.
 * @param PageSizeLog2 returns page size in log2 scale.
 */
void NvBootProdUartGetBlockSizes(
    const NvBootProdUartParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2);

/**
 * Uses the data pointed to by Params to intialize the device for reading.
 * Other device Inits can be called multiple times, ProdUartInit should NOT
 * be called multiple times, do not include DevParam sets in a ProdUart BCT.
 *
 * Uses IRAM_C to store the loaded image.
 *
 * @retval NvBootError_Success Always returns success or hangs.
 */
NvBootError NvBootProdUartInit(
    const NvBootProdUartParams *Params,
    NvBootProdUartContext *pFoosContext);

/**
 * Initiate the reading of a page of data into Dest. buffer.
 * 
 * @param Block Block number to read from.
 * @param Page Page number in the block to read from. 
 *          valid range is 0 <= Page < PagesPerBlock.
 * @param Dest Buffer to rad the data into.
 *
 * @retval NvBootError_Success Read is done from iRam, so it will always succeed
 */
NvBootError NvBootProdUartReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest);

/**
 * Check the satus of a read operation. ProdUart uses blocking copies,
 * so this really isn't relevant.
 *
 * @retval NvBootDeviceStatus_Idle Always returns this
 */
NvBootDeviceStatus NvBootProdUartQueryStatus(void);

/**
 * Shutdown device and cleanup. Unimplemented in ProdUart
 */
void NvBootProdUartShutdown(void);

/**
 * Allocates buffers for reader code. Unimplemented for ProdUart.
 *
 * @retval NvBootError_Unimplemented Always returns this.
 */
NvBootError NvBootProdUartGetReaderBuffersBase(
    NvU8** ReaderBuffersBase,
    const NvU32 Alignment,
    const NvU32 Bytes);

#endif //PROD_UART_INT_H
