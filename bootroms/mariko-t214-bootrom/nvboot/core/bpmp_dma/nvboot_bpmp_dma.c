/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#include "nvtypes.h"
#include "nvrm_drf.h"
#include "nvboot_hardware_access_int.h"
#include "argpcdma_bpmp.h"
#include "address_map_new.h"
#include "nvboot_reset_int.h"
#include "nvboot_error.h"
#include "nvboot_util_int.h"
#include "nvboot_config.h"

#define SHIFT(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFT(PERIPH##_##REG##_0_##FIELD##_##RANGE)
#define SHIFTMASK(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFTMASK(PERIPH##_##REG##_0_##FIELD##_##RANGE)

#define PHYSICAL_SYSRAM_ADDR(Addr) \
((((Addr)&0xF0000000) == 0x40000000)?(((Addr) & ~(0xF0000000)) | 0x30000000):(Addr))

/**
 * Deassert resets for BPMP DMA engine
 */
void NvBootBpmpDmaInit()
{
    // Assert reset.
    NvBootResetSetEnable(NvBootResetDeviceId_BpmpDmaId, NV_TRUE);
    NvBootUtilWaitUS(1);
    // De-assert
    NvBootResetSetEnable(NvBootResetDeviceId_BpmpDmaId, NV_FALSE);
}

static NvBootError NvBootPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout)
{
    NvU32 RegData;
    do {
    RegData = NV_READ32(RegAddr);
    if((RegData & Mask) == ExpectedValue)
        return NvBootError_Success;
    NvBootUtilWaitUS(1);
    Timeout--;
    } while(Timeout);
    return NvBootError_HwTimeOut;
}

/**
 * BPMP DMA transfer from DRAM to SYSRAM.
 */
NvBootError NvBootBpmpMem2Mem(NvU32 Src, NvU32 Dest, NvU32 Bytes, NvU32 BurstLengthWords)
{
    NvBootError e;
    NvU32 RegData, ExpectedValue, ShiftMask;

    // If 0 bytes, just return
    if(!Bytes)
        return NvBootError_Success;

    // Set Src Address.
    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_SRC_PTR_0, Src);

    Dest = PHYSICAL_SYSRAM_ADDR(Dest);

    // Set Destination Address
    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_DST_PTR_0, Dest);
    // Clear HI-ADR
    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_HI_ADR_PTR_0, 0);

    // Set number of words. Round bytes to nearest word and divide/4

    Bytes = ALIGN_ADDR(Bytes, 4);
    Bytes = Bytes/4;

    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_BCOUNT_0, Bytes - 1);


    // Set MC Burst. only 2 options if user passes in 2 or less, use 2
    // else 16
    RegData = NV_READ32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_MC_SEQ_0);
    if(BurstLengthWords <= 2)
    {
        // BURST 2
        RegData = NV_FLD_SET_DRF_DEF(GPCDMA_BPMP, CHANNEL_CH0_MC_SEQ, MC_BURST, DMA_BURST_2WORDS, RegData);
    }
    else
    {
        // BURST 16
        RegData = NV_FLD_SET_DRF_DEF(GPCDMA_BPMP, CHANNEL_CH0_MC_SEQ, MC_BURST, DMA_BURST_16WORDS, RegData);
    }
    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_MC_SEQ_0, RegData);


    // Set transfer attributes.
    // Once only, Mem2Mem
    RegData = NV_READ32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_CSR_0);
    RegData = NV_FLD_SET_DRF_NUM(GPCDMA_BPMP, CHANNEL_CH0_CSR, ONCE, 1, RegData);
    RegData = NV_FLD_SET_DRF_DEF(GPCDMA_BPMP, CHANNEL_CH0_CSR, DMA_MODE, MEM2MEM, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_CSR_0, RegData);

    // Enable!
    RegData = NV_READ32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_CSR_0);
    RegData = NV_FLD_SET_DRF_NUM(GPCDMA_BPMP, CHANNEL_CH0_CSR, ENB, 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_CSR_0, RegData);
    
    // Check if BSY Channel Status goes from 0-1-0 (0-Wait, 1-Active)
    // Alternatively just wait 1 us and then start polling for 0
    ShiftMask = SHIFTMASK(GPCDMA_BPMP, CHANNEL_CH0_STA, BSY);
    //ExpectedValue = 1 << SHIFT(GPCDMA_BPMP, CHANNEL_CH0_STA, BSY);
    
    //e = NvBootPollField(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_STA_0,
    //                    ShiftMask,
    //                    ExpectedValue,
    //                    100); // 100 ms timeout.
    NvBootUtilWaitUS(10);

    ExpectedValue = 0;
    
    e = NvBootPollField(NV_ADDRESS_MAP_BPMP_DMA_BASE + GPCDMA_BPMP_CHANNEL_CH0_STA_0,
                        ShiftMask,
                        ExpectedValue,
                        100); // 100 ms timeout.
    if(e != NvBootError_Success)
        return e;

    return NvBootError_Success;

}
