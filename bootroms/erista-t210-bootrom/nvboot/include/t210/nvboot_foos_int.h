#ifndef FOOS_INT_H
#define FOOS_INT_H
#include "nvboot_device_int.h"

#define FOOS_DATA_LOCATION NV_ADDRESS_MAP_IRAM_C_BASE
// Block size 8192
#define FOOS_BLOCKSIZELOG2 14
// Page size 512
#define FOOS_PAGESIZELOG2 9
typedef struct NvBootFoosContextRec
{
    NvU32 PageSizeLog2;
    NvU32 BlockSizeLog2;
}
NvBootFoosContext;
typedef struct NvBootFoosParamsRec
{
    NvU32 PageSizeLog2;
    NvU32 BlockSizeLog2;
} NvBootFoosParams;

void
NvBootFoosGetParams(
    const NvU32 ParamIndex,
    NvBootFoosParams **Params);
NvBool
NvBootFoosValidateParams(
    const NvBootFoosParams *Params);
void
NvBootFoosGetBlockSizes(
    const NvBootFoosParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2);
NvBootError
NvBootFoosInit(
    const NvBootSnorParams *Params,
    NvBootFoosContext *pFoosContext);
NvBootError NvBootFoosReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest);
NvBootDeviceStatus NvBootFoosQueryStatus(void);
void NvBootFoosShutdown(void);
NvBootError NvBootFoosGetReaderBuffersBase(
    NvU8** ReaderBuffersBase,
    const NvU32 Alignment,
    const NvU32 Bytes);
#endif //FOOS_INT_H
