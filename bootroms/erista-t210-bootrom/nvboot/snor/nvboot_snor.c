/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_snor_local.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arsnor.h"
#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_snor_context.h"
#include "nvboot_snor_int.h"
#include "nvboot_util_int.h"
#include "project.h"

// Wrapper macros for reading/writing from/to SNOR register
#define SNOR_REG_READ32(reg) \
        NV_READ32(NV_ADDRESS_MAP_APB_SNOR_BASE + (SNOR_##reg##_0))

#define SNOR_REG_WRITE32(reg, val) \
    do { \
        NV_WRITE32((NV_ADDRESS_MAP_APB_SNOR_BASE + (SNOR_##reg##_0)), (val)); \
    } while (0)

/**
 * Extract field from boot device configuration fuse value.
 * 
 * @param field  Member of boot dev configuration fuse when device type is snor.
 * @param fieldtype Type of the member.
 * @param width  Width of the member (in number of bits).
 *
 */
#define GET_FUSE_PARAM_FIELD(field, fieldtype, width)   (fieldtype)((ParamIndex >> SnorFuseParamOffset_##field) & SnorFuseParamMask_##width);

/**
 * Check validity of fuse param extracted from boot device configuration
 * @param f bootdevice config parameter
 * @param t type of config parameter
 *
 */
#define CHECK_FUSE_PARAM_VALID(f,t) \
    do { \
        if (f >= t##_Max) \
            goto fail; \
    } while(0)

/**
 * Set boot information fields for snor
 * 
 * @param r  Member of snor status in BIT.
 * @param f  Field for member of snor status in BIT.
 * @param v  State of the field f.
 *
 */
#define SNOR_BIT_FIELD_INFO(r, f, v)    v << (SnorBitFldOffset_##r##_##f);

/**
 * Calculate the number of 32 bit words for the number of bytes given as input
 * Computation gives nw = ceil[nb/4]
 * @param nb Number of bytes for which the word count is to be computed
 * @param nw Number of words corresponding to numbytes
 *
 */
#define DMA_WORDCOUNT(nb, nw) \
    do { \
        nw = nb/4 + ( ((nb - ((nb/4) *4)) == 0) ? 0 : 1); \
    } while(0)

/**
 * Wait for a particular bit to change state
 * 
 * @param r  register
 * @param f field
 * @param s state to which the field f should move to
 * @param t max time(in usec) to wait for the bit to change state 
 *
 */
#define SNOR_WAIT(r, f, s, t) \
    do { \
        Timeout = t; \
        do { \
            NvBootUtilWaitUS(1); \
            Timeout--; \
            RegVal = SNOR_REG_READ32(r); \
            RegVal = NV_DRF_VAL(SNOR, r, f, RegVal); \
        } while ( (Timeout) && (RegVal != s) ); \
        if (RegVal != s) \
        { \
            Error = NvBootError_HwTimeOut; \
            goto fail; \
        } \
    } while(0)

/**
 * POR values of timing registers. This will allow bootrom to override the POR
 * values for timing(using irom patch) even before reading the first piece of
 * data(when snor params from BCT are not available).
 */
static const NvBootSnorTimingParam s_SnorTimingCfgVal =
{
    /* SNOR_TIMING0_0 */
    0x30201114 ,

    /* SNOR_TIMING1_0 */
    0x10010103,

    /* SNOR_TIMING2_0 */
    0x00000004
};

/// Configuration of snor controller for timing parameters
typedef struct 
{
    NvBool Width;
    
    NvBool NonMuxed;
    
    /// Bit to use either the POR or default values for Timing registers
    /// or new values. This bit will allow changing default values through
    /// irom patch even for PIO mode.
    /// 0 to use default timing, 1 to use timing params
    NvBool UseNorTimingParams;
 
    /// Chip Select for snor. This field can have values 0 to 7.
    NvU8 ChipSelect;

    /// Mode of snor device for data transfer
    SnorDeviceMode DeviceMode;
    
    /// DMA or PIO
    SnorDataXferMode DataXferMode;

    /// Page Size for snor in case Page mode is used.
    SnorPageSize PageSize;

    /// Read Transfer Data size. This is different from the page size for
    /// snor. This is to allow one-shot larger than 512 bytes data transfers so
    /// that data and/or binaries can be read faster.
    SnorXferDataSize XferDataSizeLog2;
    
    NvU8 HighSpeed;

} SnorFuseParam;

// Boot Info table.
extern NvBootInfoTable BootInfoTable;

// Pointer to Snor Bit info.
static NvBootSnorStatus* s_pSnorBitInfo = &BootInfoTable.SecondaryDevStatus.SnorStatus;

static SnorFuseParam s_SnorFuseParamData;
static NvBootSnorParams s_DefaultSnorParams;

// Context of snor driver
static NvBootSnorContext s_SnorContext;

/*
 * Public Function Definitions.
 */
void
NvBootSnorGetParams(
    const NvU32 ParamIndex,
    NvBootSnorParams **Params)
{
    NvU32 ConfigVal = 0;
    NV_ASSERT(Params != NULL);

    // Get the timing parameter and program it properly.
    s_DefaultSnorParams.ClockDivider = 1;
    s_DefaultSnorParams.ClockSource = NvBootSnorClockSource_ClockM;
    s_DefaultSnorParams.SnorTimingCfg0 = s_SnorTimingCfgVal.SnorTimingCfg0;
    s_DefaultSnorParams.SnorTimingCfg1 = s_SnorTimingCfgVal.SnorTimingCfg1;
    s_DefaultSnorParams.SnorTimingCfg2 = s_SnorTimingCfgVal.SnorTimingCfg2;

    // To Do: find out default Controller Busy and DMA Transfer timeout in us. 
    // If DMA mode is default, then time taken to transfer BCT size.
    s_DefaultSnorParams.CntrllerBsyTimeout = 4000;
    s_DefaultSnorParams.DmaTransferTimeout = 4000;

    s_SnorFuseParamData.Width = GET_FUSE_PARAM_FIELD(Width, NvBool, BitWidth1);
    s_SnorFuseParamData.NonMuxed = GET_FUSE_PARAM_FIELD(NonMuxed, NvBool, BitWidth1);
    s_SnorFuseParamData.UseNorTimingParams = GET_FUSE_PARAM_FIELD(UseNorTimingParams, NvBool, BitWidth1);
    s_SnorFuseParamData.ChipSelect = GET_FUSE_PARAM_FIELD(ChipSelect, NvU8, BitWidth1);
    s_SnorFuseParamData.DeviceMode = GET_FUSE_PARAM_FIELD(DeviceMode, SnorDeviceMode, BitWidth1);
    s_SnorFuseParamData.DataXferMode = GET_FUSE_PARAM_FIELD(DataXferMode, SnorDataXferMode, BitWidth1);
    s_SnorFuseParamData.PageSize = GET_FUSE_PARAM_FIELD(PageSize, SnorPageSize, BitWidth2);
    s_SnorFuseParamData.XferDataSizeLog2 = GET_FUSE_PARAM_FIELD(XferDataSizeLog2, SnorXferDataSize, BitWidth3);
    s_SnorFuseParamData.HighSpeed = GET_FUSE_PARAM_FIELD(HighSpeed, NvU8, BitWidth1);

    if(s_SnorFuseParamData.DataXferMode == SnorDataXferMode_Dma)
        s_DefaultSnorParams.DataXferMode = SnorDataXferMode_Dma;
    else
        s_DefaultSnorParams.DataXferMode = SnorDataXferMode_Pio;
    
    if(s_SnorFuseParamData.HighSpeed)
    {
        // This would give a frequency of 58 Mhz (PLLP_OUT0 408Mhz/7)
        s_DefaultSnorParams.ClockDivider = NVBOOT_NOR_PLLP_OUT0_58MHZ_DIV;
        s_DefaultSnorParams.ClockSource = NvBootSnorClockSource_PllPOut0;
    }
    
    s_SnorContext.PageSizeLog2 = SECTOR_SIZE_LOG2 + s_SnorFuseParamData.XferDataSizeLog2;

    s_pSnorBitInfo->ClockDivider =  s_DefaultSnorParams.ClockDivider;
    s_pSnorBitInfo->ClockSource =  s_DefaultSnorParams.ClockSource;
 
    s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, BlockSizeLog2, NVBOOT_NOR_BLOCK_SIZE_LOG2);
    s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, XferSizeLog2, s_SnorContext.PageSizeLog2);

    s_pSnorBitInfo->LastBlockRead =  0;
    s_pSnorBitInfo->LastPageRead =  0;

    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ReadPageStatus, NvBootError_NotInitialized);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, InitStatus, NvBootError_NotInitialized);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ParamsValidated, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, WaitForControllerBsy, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, WaitForDmaDone, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ControllerBsy, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, DmaDone, NV_FALSE);

    ConfigVal = SNOR_REG_READ32(TIMING0);
    s_pSnorBitInfo->TimingCfg0 =  ConfigVal;
    ConfigVal = SNOR_REG_READ32(TIMING1);
    s_pSnorBitInfo->TimingCfg1 =  ConfigVal;
    ConfigVal = SNOR_REG_READ32(TIMING2);
    s_pSnorBitInfo->TimingCfg2 =  ConfigVal;

    *Params = &s_DefaultSnorParams;
}

NvBool
NvBootSnorValidateParams(
    const NvBootSnorParams *Params)
{
    NvBool ParamsValid = NV_FALSE;

    NV_ASSERT(Params);

    if ((Params->ClockSource > NvBootSnorClockSource_Num))
    {
        goto fail;
    }
        
    // Validate the Clock divider by checking for '0x0' and whether it
    // fits within 8 bits
    if ((!Params->ClockDivider) || ((Params->ClockDivider + 1) & ~(0x000000FF) ))
    {
        goto fail;
    }
    if (Params->DataXferMode >= SnorDataXferMode_Max)
    {
        goto fail;
    } 
    if (Params->DataXferMode == SnorDataXferMode_Dma)
    {
        if ((!Params->CntrllerBsyTimeout) || (!Params->DmaTransferTimeout))
            goto fail;
    }

    // Check if boot device configuration parameters are valid
    CHECK_FUSE_PARAM_VALID(s_SnorFuseParamData.DeviceMode, SnorDeviceMode);
    CHECK_FUSE_PARAM_VALID(s_SnorFuseParamData.PageSize,  SnorPageSize);
    CHECK_FUSE_PARAM_VALID(s_SnorFuseParamData.XferDataSizeLog2,  SnorXferDataSize);

    ParamsValid = NV_TRUE;

fail:
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ParamsValidated, ParamsValid);
    return ParamsValid;
}

void
NvBootSnorGetBlockSizes(
    const NvBootSnorParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);
   
    *BlockSizeLog2 = NVBOOT_NOR_BLOCK_SIZE_LOG2;
    *PageSizeLog2 = s_SnorContext.PageSizeLog2;

    s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, BlockSizeLog2, NVBOOT_NOR_BLOCK_SIZE_LOG2);
    s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, XferSizeLog2, s_SnorContext.PageSizeLog2);

}

NvBootError
NvBootSnorInit(
    const NvBootSnorParams *Params,
    NvBootSnorContext *pNorContext)
{
    NvU32 ConfigVal;
    NvBootError Error;
    NvBootPinmuxConfig SnorPinMuxConfig = NvBootPinmuxConfig_Snor_Non_Mux_x16;
    
    NV_ASSERT(pNorContext);
    NV_ASSERT(Params);
    NV_ASSERT(((Params->ClockDivider != 0) && (Params->ClockDivider < 0xFF)));
    NV_ASSERT(Params->ClockSource <= NvBootSnorClockSource_Num);

    s_SnorContext.PageSizeLog2 = SECTOR_SIZE_LOG2 + s_SnorFuseParamData.XferDataSizeLog2;
    s_SnorContext.DataXferMode = Params->DataXferMode;
    s_SnorContext.CntrllerBsyTimeout = Params->CntrllerBsyTimeout;
    s_SnorContext.DmaTransferTimeout = Params->DmaTransferTimeout;

    // Reset the controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_TRUE);   
    // Configure the clock source.
    NvBootClocksConfigureClock(NvBootClocksClockId_SnorId, 
        NVBOOT_CLOCKS_7_1_DIVIDER_BY((Params->ClockDivider), 0), 
        Params->ClockSource);

    s_pSnorBitInfo->ClockDivider =  Params->ClockDivider;
    s_pSnorBitInfo->ClockSource =  Params->ClockSource;

    
    // Enable the Clock for snor Controller
    NvBootClocksSetEnable(NvBootClocksClockId_SnorId, NV_TRUE);

    // Reset of the nor controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_TRUE);
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_FALSE);
    
    ConfigVal = SNOR_REG_READ32(CONFIG);

    // Chip Select - on fpga, it's CS2 for Numonyx and CS0 for Samsung.
    ConfigVal = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, SNOR_SEL,
                s_SnorFuseParamData.ChipSelect, ConfigVal);

    // Default would be Asynchronous mode.
    ConfigVal = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, DEVICE_MODE,
            s_SnorFuseParamData.DeviceMode, ConfigVal);
    
    if(s_SnorFuseParamData.DeviceMode == SnorDeviceMode_Page)
    {
        ConfigVal = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, PAGE_SIZE,
                s_SnorFuseParamData.PageSize, ConfigVal);
        s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, SnorPageSize, s_SnorFuseParamData.PageSize);
    }
    
    s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, DeviceMode, s_SnorFuseParamData.DeviceMode);

    // Device mode
    if (Params->DataXferMode == SnorDataXferMode_Dma)
    {
        ConfigVal = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, MST_ENB,
                NV_TRUE, ConfigVal);    //Master Enable
    }

    // Device type is SNOR
    ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, NOR_DEVICE_TYPE, SNOR, ConfigVal);
    
    // Configure the mux mode and width for data transfer.
    if (s_SnorFuseParamData.NonMuxed == Snor_NonMuxed) // NON MUXED
    {
        if (s_SnorFuseParamData.Width == SnorWidth16bit) // NON MUXED 16
        {
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, WORDWIDE_GMI, NOR16BIT, ConfigVal);
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MUXMODE_GMI, AD_NONMUX, ConfigVal);
            SnorPinMuxConfig = NvBootPinmuxConfig_Snor_Non_Mux_x16;
        }
        else // NON MUXED 32
        {
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, WORDWIDE_GMI, NOR32BIT, ConfigVal);
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MUXMODE_GMI, AD_NONMUX, ConfigVal);
            SnorPinMuxConfig = NvBootPinmuxConfig_Snor_Non_Mux_x32;
        }
    }
    else // MUXED
    {
        if (s_SnorFuseParamData.Width == SnorWidth16bit) // MUXED 16
        {
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, WORDWIDE_GMI, NOR16BIT, ConfigVal);
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MUXMODE_GMI, AD_MUX, ConfigVal);
            SnorPinMuxConfig = NvBootPinmuxConfig_Snor_Mux_x16;
        }
        else // MUXED 32
        {
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, WORDWIDE_GMI, NOR32BIT, ConfigVal);
            ConfigVal = NV_FLD_SET_DRF_DEF(SNOR, CONFIG, MUXMODE_GMI, AD_MUX, ConfigVal);
            SnorPinMuxConfig = NvBootPinmuxConfig_Snor_Mux_x32;
        }
    
    }

    SNOR_REG_WRITE32(CONFIG, ConfigVal);

    // Set the timing registers
    if (s_SnorFuseParamData.UseNorTimingParams)
    {
        SNOR_REG_WRITE32(TIMING0,Params->SnorTimingCfg0);
        SNOR_REG_WRITE32(TIMING1,Params->SnorTimingCfg1);
        SNOR_REG_WRITE32(TIMING2,Params->SnorTimingCfg2);
    }
    ConfigVal = SNOR_REG_READ32(TIMING0);
    s_pSnorBitInfo->TimingCfg0 =  ConfigVal;
    ConfigVal = SNOR_REG_READ32(TIMING1);
    s_pSnorBitInfo->TimingCfg1 =  ConfigVal;
    ConfigVal = SNOR_REG_READ32(TIMING2);
    s_pSnorBitInfo->TimingCfg2 =  ConfigVal;

    // Calling the pinmux configuration
    Error = NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_SnorFlash, 
                                            SnorPinMuxConfig);

    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, InitStatus, Error);
    return Error;                                            
}
/* This functions assumes Dest is 16 bit/32 bit aligned for x16/x32 cases.
   Currently buffer coming in from reader satisfies this alignment.
 */
static 
NvBootError SnorPioReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest,
    const NvU32 NumBytesToRead)
{
    NvU16* pSrc;
    NvU16* pDest;
    NvU32* pSrc32;
    NvU32* pDest32;
    NvU32 Index;

    if(s_SnorFuseParamData.Width == SnorWidth16bit)
    {
        pSrc = (NvU16*)(NV_ADDRESS_MAP_NOR_BASE +
                        (Block << NVBOOT_NOR_BLOCK_SIZE_LOG2) +
                        (Page << s_SnorContext.PageSizeLog2));
        pDest = (NvU16*)Dest;

        for (Index = 0; Index < (NumBytesToRead/2) ; ++Index)
            *pDest++ = *pSrc++;
    }
    else
    {
        pSrc32 = (NvU32*)(NV_ADDRESS_MAP_NOR_BASE +
                        (Block << NVBOOT_NOR_BLOCK_SIZE_LOG2) +
                        (Page << s_SnorContext.PageSizeLog2));
        pDest32 = (NvU32*)Dest;

        for (Index = 0; Index < (NumBytesToRead/4) ; ++Index)
            *pDest32++ = *pSrc32++;
    }
    // If the destination address is in external memory, make sure the writes
    // to memory are coherent. 
    if (NvBootAhbCheckIsExtMemAddr((NvU32 *)Dest)) 
    {
        NvBootAhbWaitCoherency(ARAHB_MST_ID_SNOR);
    }
    return NvBootError_Success;
}
/* SnorDmaReadPage
 * Note: Bug 1209079. Nor addr in Page mode should be aligned to page size.
 * Nor driver reads in logical page sizes of 512, 1024, 2048, 4196, 8192.
 * This automatically ensures alignment with device page sizes 16, 32, 64.
 */
static 
NvBootError SnorDmaReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest,
    const NvU32 NumBytesToRead)
{
    NvU32 RegVal = 0;
    NvU32 WordCount = 0;
    NvU32 Timeout = 0;
    NvU32 NorConfig = 0;
    NvBootError Error = NvBootError_DeviceReadError;
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, WaitForControllerBsy, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, WaitForDmaDone, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ControllerBsy, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, DmaDone, NV_FALSE);

    
    /// Set Source and Destination address pointers
    RegVal = (NV_ADDRESS_MAP_NOR_BASE +
                (Block << NVBOOT_NOR_BLOCK_SIZE_LOG2) +
                (Page << s_SnorContext.PageSizeLog2));

    SNOR_REG_WRITE32(NOR_ADDR_PTR, RegVal);

    SNOR_REG_WRITE32(AHB_ADDR_PTR, (NvU32)Dest);
    /// Configure DMA
    RegVal = SNOR_REG_READ32(DMA_CFG);

    /// Set Read/Write operation
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, DIR,
                SnorDmaDirection_NORtoAHB, RegVal);
    /// Configure 4 WORD burst size - shouldn't be hardcoded.
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, BURST_SIZE, 5, RegVal);
    /// Allow once only DMA transfer
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, CONTINUOUS, 0, RegVal);

    /// Calculate word count - Will exactly correspond to num of bytes in this case
    DMA_WORDCOUNT(NumBytesToRead, WordCount);
    /// Set Word Count
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, WORD_COUNT, (WordCount ? (WordCount -1): 0), RegVal);

    SNOR_REG_WRITE32(DMA_CFG, RegVal);
    
    /// A NOR operation commences
    RegVal = SNOR_REG_READ32(CONFIG);
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, GO_NOR, NV_TRUE, RegVal);
    SNOR_REG_WRITE32(CONFIG, RegVal);

    /// Enable DMA
    RegVal = SNOR_REG_READ32(DMA_CFG);
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, DMA_GO, NV_TRUE, RegVal);
    SNOR_REG_WRITE32(DMA_CFG, RegVal);

    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, WaitForControllerBsy, NV_TRUE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ControllerBsy, NV_TRUE);
 
    /// Wait for Controller Busy
    SNOR_WAIT(DMA_CFG, BSY, 0, s_SnorContext.CntrllerBsyTimeout);

    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ControllerBsy, NV_FALSE);
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, WaitForDmaDone, NV_TRUE);

    /// Bug 965703  Wait for Dma Go to be de-asserted instead of DMA Done for polling
    SNOR_WAIT(DMA_CFG, DMA_GO, 0, s_SnorContext.DmaTransferTimeout);

    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, DmaDone, NV_TRUE);
    /// Check Error Bits and set the values in BIT.
    
    // If the destination address is in external memory, make sure the writes
    // to memory are coherent. 
    if (NvBootAhbCheckIsExtMemAddr((NvU32 *)Dest)) 
    {
        NvBootAhbWaitCoherency(ARAHB_MST_ID_SNOR);
    }
    Error = NvBootError_Success;

fail:
    
    /// Clear NOR CONFIG GO bit
    NorConfig = SNOR_REG_READ32(CONFIG);
    NorConfig = NV_FLD_SET_DRF_NUM(SNOR, CONFIG, GO_NOR, NV_FALSE, NorConfig);
    SNOR_REG_WRITE32(CONFIG, NorConfig);
    /// Write Clear DMA DONE bit
    RegVal = SNOR_REG_READ32(DMA_CFG);
    RegVal = NV_FLD_SET_DRF_NUM(SNOR, DMA_CFG, IS_DMA_DONE, NV_TRUE, RegVal);
    SNOR_REG_WRITE32(DMA_CFG, RegVal);
    
    /// Bug 1240715. Controller Reset after every dma read.
    /// Wait 2us for data to be committed to memory after DMA done and before reset.
    /// Also restore Nor config and timing registers.
    NvBootUtilWaitUS(2);
    // Reset of the nor controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_TRUE);
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_FALSE);

    // Restore config and timing registers
    SNOR_REG_WRITE32(CONFIG, NorConfig);
    if (s_SnorFuseParamData.UseNorTimingParams)
    {
        SNOR_REG_WRITE32(TIMING0,s_pSnorBitInfo->TimingCfg0);
        SNOR_REG_WRITE32(TIMING1,s_pSnorBitInfo->TimingCfg1);
        SNOR_REG_WRITE32(TIMING2,s_pSnorBitInfo->TimingCfg2);
    }
    
    return Error;
}

NvBootError NvBootSnorReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest)
{

    NvBootError Error = NvBootError_DeviceReadError;
    const NvU32 NumBytesToRead = (1 << s_SnorContext.PageSizeLog2);
    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < (1 << (NVBOOT_NOR_BLOCK_SIZE_LOG2 - s_SnorContext.PageSizeLog2)));

    s_pSnorBitInfo->LastBlockRead =  Block;
    s_pSnorBitInfo->LastPageRead =  Page;
    s_pSnorBitInfo->SnorConfig |= SNOR_BIT_FIELD_INFO(SnorConfig, DataXferMode, s_SnorContext.DataXferMode);

    switch (s_SnorContext.DataXferMode)
    {
        case SnorDataXferMode_Pio:
            Error = SnorPioReadPage(Block, Page, Dest, NumBytesToRead);
            break;
        case SnorDataXferMode_Dma:
            Error = SnorDmaReadPage(Block, Page, Dest, NumBytesToRead);
            break;
        default:
            break;
    };
    s_pSnorBitInfo->SnorDriverStatus |= SNOR_BIT_FIELD_INFO(SnorDriverStatus, ReadPageStatus, Error);
    return Error;
}

NvBootDeviceStatus NvBootSnorQueryStatus(void)
{
    return NvBootDeviceStatus_Idle;
}

void NvBootSnorShutdown(void)
{
    // Enable - Reset of the nor controller.
    NvBootResetSetEnable(NvBootResetDeviceId_SnorId, NV_TRUE);

    // Disable the Clock for SPI Controller
    NvBootClocksSetEnable(NvBootClocksClockId_SnorId, NV_FALSE);

}

NvBootError NvBootSnorGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    return NvBootError_Unimplemented;
}

