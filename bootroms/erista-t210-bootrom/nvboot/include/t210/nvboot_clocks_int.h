/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_clocks_int.h - Declarations for clocks support.
 */

#ifndef INCLUDED_NVBOOT_CLOCKS_INT_H
#define INCLUDED_NVBOOT_CLOCKS_INT_H

#include "nvcommon.h"
#include "arclk_rst.h"
#include "nvboot_clocks.h"
#include "nvboot_error.h"
#include "nvboot_osc.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Set of clocks supported in the API
// The corresponding enum embeds some information on the register structure
// The lowest byte of the enum is unique per clock and built like this
// Bit 8:6 is the clock ID encoding from NvBootClocksRegId below. Possible values from 0 to 5.
// Bit 5 == 1: the clock is associated with a bit in register CLK_ENB. 
//             bits 4:0 is the bit offset in the CLK_ENB register, that is the _SHIFT part
// Bit 5 == 0: the clock has no associated bit in register CLK_ENB.
//             it also happens that none of these clocks has a source register in standard format
// The two MSB encode the offset of the source register if of standard format, 0 if not
#define NVBOOT_CLOCKS_STANDARD_ENB  (0x20)
#define NVBOOT_CLOCKS_ENB_BIT_OFFSET_MASK (0x1F)
#define NVBOOT_CLOCKS_REG_SHIFT (0x06)
#define NVBOOT_CLOCKS_REG_MASK (0x1C0)
#define NVBOOT_CLOCKS_BIT_OFFSET(ClockId) (((NvU32) ClockId) & NVBOOT_CLOCKS_ENB_BIT_OFFSET_MASK)
#define NVBOOT_CLOCKS_REG_OFFSET(ClockId) ((((NvU32) ClockId) & NVBOOT_CLOCKS_REG_MASK) >> NVBOOT_CLOCKS_REG_SHIFT)         

// Define enums to index into ClocksRegSetClrTbl[]    
typedef enum
{
    ClocksReg_L = 0,
    ClocksReg_H,
    ClocksReg_U,
    ClocksReg_X,
    ClocksReg_V,
    ClocksReg_W,
    ClocksReg_Y,
    ClocksReg_Num,
} NvBootClocksRegId;

#define NVBOOT_CLOCKS_L_REG (ClocksReg_L << NVBOOT_CLOCKS_REG_SHIFT)
#define NVBOOT_CLOCKS_H_REG (ClocksReg_H << NVBOOT_CLOCKS_REG_SHIFT)
#define NVBOOT_CLOCKS_U_REG (ClocksReg_U << NVBOOT_CLOCKS_REG_SHIFT)
#define NVBOOT_CLOCKS_X_REG (ClocksReg_X << NVBOOT_CLOCKS_REG_SHIFT)
#define NVBOOT_CLOCKS_V_REG (ClocksReg_V << NVBOOT_CLOCKS_REG_SHIFT)
#define NVBOOT_CLOCKS_W_REG (ClocksReg_W << NVBOOT_CLOCKS_REG_SHIFT)
#define NVBOOT_CLOCKS_Y_REG (ClocksReg_Y << NVBOOT_CLOCKS_REG_SHIFT)

#define NVBOOT_CLOCKS_SOURCE_SHIFT (16)
#define NVBOOT_CLOCKS_HAS_STANDARD_ENB(ClockId) (((NvU32) ClockId) & NVBOOT_CLOCKS_STANDARD_ENB)
#define NVBOOT_CLOCKS_SOURCE_OFFSET(ClockId)  ( (((NvU32) ClockId) & ~((1<<NVBOOT_CLOCKS_SOURCE_SHIFT) -1))\
                                                 >> NVBOOT_CLOCKS_SOURCE_SHIFT)
typedef enum
{  
    NvBootClocksClockId_SclkId   = 0x0,
    NvBootClocksClockId_HclkId,
    NvBootClocksClockId_PclkId,
    NvBootClocksClockId_CclkId   = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_CPU_SHIFT    
                                 + NVBOOT_CLOCKS_STANDARD_ENB,

    NvBootClocksClockId_UsbId    = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_USBD_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB,

    NvBootClocksClockId_I2c1Id   = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_I2C1_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB 
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_UartaId  = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_UARTA_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

#if NVENABLE_NAND_SUPPORT
    NvBootClocksClockId_NandId   = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_NDFLASH_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_NDFLASH_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_NandSpeedId   = CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0_CLK_ENB_NAND_SPEED_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_U_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_NAND_SPEED_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),
#endif
    NvBootClocksClockId_SdmmcId  = CLK_RST_CONTROLLER_CLK_OUT_ENB_L_0_CLK_ENB_SDMMC4_SHIFT  
                                 + NVBOOT_CLOCKS_STANDARD_ENB
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_ApbDmaId = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_APBDMA_SHIFT 
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG,

    NvBootClocksClockId_AhbDmaId = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_AHBDMA_SHIFT 
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG, 

    NvBootClocksClockId_EmcId    = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_EMC_SHIFT    
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG,

    NvBootClocksClockId_McId    = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_MEM_SHIFT    
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG,

    NvBootClocksClockId_SpiId    = CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0_CLK_ENB_QSPI_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_Y_REG,

    NvBootClocksClockId_PmcId    = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_PMC_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG,

    NvBootClocksClockId_SeId     = CLK_RST_CONTROLLER_CLK_OUT_ENB_V_0_CLK_ENB_SE_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_V_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_SE_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_I2c2Id   = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_I2C2_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_I2C2_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_I2c3Id   = CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0_CLK_ENB_I2C3_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_U_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_I2C3_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_I2c4Id   = CLK_RST_CONTROLLER_CLK_OUT_ENB_V_0_CLK_ENB_I2C4_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_V_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_I2C4_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_I2c5Id = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_I2C5_SHIFT   
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_I2C5_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_Sbc1Id	 = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_SPI1_SHIFT
                                + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG
                                + (CLK_RST_CONTROLLER_CLK_SOURCE_SPI1_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_Sbc2Id	 = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_SPI2_SHIFT
                            + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG
                            + (CLK_RST_CONTROLLER_CLK_SOURCE_SPI2_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_Sbc3Id	 = CLK_RST_CONTROLLER_CLK_OUT_ENB_H_0_CLK_ENB_SPI3_SHIFT
                            + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_H_REG
                            + (CLK_RST_CONTROLLER_CLK_SOURCE_SPI3_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

#ifdef NV_MC_HAS_DUAL_CHANNEL_ENABLED
    NvBootClocksClockId_Emc1Id    = CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0_CLK_ENB_EMC1_SHIFT    
                             + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_W_REG,

    NvBootClocksClockId_Mc1Id    = CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0_CLK_ENB_MC1_SHIFT    
                             + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_W_REG,
#endif

    NvBootClocksClockId_XUsbId   = CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0_CLK_ENB_XUSB_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_W_REG,

    NvBootClocksClockId_XUsbHostId   = CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0_CLK_ENB_XUSB_HOST_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_U_REG,

    /// CLK_SOURCE_SATA_0 register follows the standard format w.r.t. bit fields for clock source
    /// and clock divider but has reserved bits utilized for other purposes. Therefore, the two MSB
    /// are populated with the SATA clock source register offset.
    NvBootClocksClockId_SataId = CLK_RST_CONTROLLER_CLK_OUT_ENB_V_0_CLK_ENB_SATA_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_V_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_SATA_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    /// SATA AUX Tx/RX clocks can be enabled using a different bit in CAR register for SATA
    NvBootClocksClockId_SataAuxId = CLK_RST_CONTROLLER_CLK_OUT_ENB_V_0_CLK_ENB_SATA_SHIFT
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_SATA_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_SataOobId = CLK_RST_CONTROLLER_CLK_OUT_ENB_V_0_CLK_ENB_SATA_OOB_SHIFT
                                 + NVBOOT_CLOCKS_STANDARD_ENB + NVBOOT_CLOCKS_V_REG
                                 + (CLK_RST_CONTROLLER_CLK_SOURCE_SATA_OOB_0 << NVBOOT_CLOCKS_SOURCE_SHIFT),

    NvBootClocksClockId_Force32 = 0x7fffffff,

    NvBootClocksClockId_Sbc5Id = NvBootClocksClockId_Force32,

    NvBootClocksClockId_Sbc6Id = NvBootClocksClockId_Force32,

    NvBootClocksClockId_SnorId = NvBootClocksClockId_Force32,

    /// BSEA and VDE have been removed and are not expected to be back in 
    /// future chips. Replaced with SE and NVDEC.
    NvBootClocksClockId_BseaId = NvBootClocksClockId_Force32,

    NvBootClocksClockId_BsevId = NvBootClocksClockId_Force32,

    NvBootClocksClockId_VdeId  = NvBootClocksClockId_Force32,

    /// KBC and USB3 are not present in T210. USB3 here refers to the third
    /// instance of legacy USB controller and not to XUSB.
    NvBootClocksClockId_KbcId  = NvBootClocksClockId_Force32,

    NvBootClocksClockId_Usb3Id = NvBootClocksClockId_Force32

} NvBootClocksClockId;

#define NVBOOT_CLOCKS_CHECK_CLOCKID(ClockId) \
    NV_ASSERT( (ClockId == NvBootClocksClockId_SclkId) || \
               (ClockId == NvBootClocksClockId_HclkId) || \
               (ClockId == NvBootClocksClockId_PclkId) || \
               (ClockId == NvBootClocksClockId_CclkId) || \
               (ClockId == NvBootClocksClockId_UsbId)  || \
               (ClockId == NvBootClocksClockId_Usb3Id) || \
               (ClockId == NvBootClocksClockId_I2c1Id) || \
               (ClockId == NvBootClocksClockId_UartaId) || \
               (ClockId == NvBootClocksClockId_SdmmcId)||  \
               (ClockId == NvBootClocksClockId_BseaId)  || \
               (ClockId == NvBootClocksClockId_BsevId)  || \
               (ClockId == NvBootClocksClockId_KbcId)   || \
               (ClockId == NvBootClocksClockId_VdeId)   || \
               (ClockId == NvBootClocksClockId_ApbDmaId) || \
               (ClockId == NvBootClocksClockId_AhbDmaId) || \
               (ClockId == NvBootClocksClockId_SnorId)    || \
               (ClockId == NvBootClocksClockId_EmcId)    || \
               (ClockId == NvBootClocksClockId_McId)    || \
               (ClockId == NvBootClocksClockId_PmcId)    || \
               (ClockId == NvBootClocksClockId_SpiId) || \
               (ClockId == NvBootClocksClockId_SeId) || \
               (ClockId == NvBootClocksClockId_I2c2Id) || \
               (ClockId == NvBootClocksClockId_I2c3Id) || \
               (ClockId == NvBootClocksClockId_I2c4Id) || \
               (ClockId == NvBootClocksClockId_I2c5Id) || \
               (ClockId == NvBootClocksClockId_Sbc1Id) || \
               (ClockId == NvBootClocksClockId_Sbc2Id) || \
               (ClockId == NvBootClocksClockId_Sbc3Id) || \
               (ClockId == NvBootClocksClockId_Sbc5Id) || \
               (ClockId == NvBootClocksClockId_Sbc6Id) || \
               (ClockId == NvBootClocksClockId_XUsbId)    || \
               (ClockId == NvBootClocksClockId_XUsbHostId)    || \
               (ClockId == NvBootClocksClockId_SataId) || \
               (ClockId == NvBootClocksClockId_SataOobId) || \
               (ClockId == NvBootClocksClockId_SataAuxId) || \
               0);
#if NVENABLE_NAND_SUPPORT
               (ClockId == NvBootClocksClockId_NandId) || \
               (ClockId == NvBootClocksClockId_NandSpeedId) || 
#endif
#ifdef NV_MC_HAS_DUAL_CHANNEL_ENABLED
               (ClockId == NvBootClocksClockId_Emc1Id)    || \
               (ClockId == NvBootClocksClockId_Mc1Id)    || 
#endif

// similar when changing a clock, much smaller than a PLL
#define NVBOOT_CLOCKS_CLOCK_STABILIZATION_TIME (0x2)

// Set of PLL supported in the API
// The enum encodes their base and misc offset
typedef enum
{  
    NvBootClocksPllId_PllC4 = (CLK_RST_CONTROLLER_PLLC4_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLC4_MISC_0,
    NvBootClocksPllId_PllM = (CLK_RST_CONTROLLER_PLLM_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLM_MISC2_0,
    NvBootClocksPllId_PllU = (CLK_RST_CONTROLLER_PLLU_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLU_MISC_0,
    NvBootClocksPllId_PllP = (CLK_RST_CONTROLLER_PLLP_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLP_MISC_0, 
    NvBootClocksPllId_PllX = (CLK_RST_CONTROLLER_PLLX_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLX_MISC_0,
    NvBootClocksPllId_PllE = (CLK_RST_CONTROLLER_PLLE_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLE_MISC_0,
    NvBootClocksPllId_PllREFE = (CLK_RST_CONTROLLER_PLLREFE_BASE_0 << 16) | CLK_RST_CONTROLLER_PLLREFE_MISC_0,
    NvBootClocksPllId_Force32 = 0x7fffffff
} NvBootClocksPllId;

// define structure for misc param
typedef struct
{
    NvU32 setup; // just 24 bits are valid data
    NvU8  kcp;
    NvU8  kvco;
    NvU8  divm;
    NvU8  divn;
    NvU8  divp;
}NvBootClocksMiscParams;

typedef struct
{
    NvU16 Sdmdin;
    NvU16 Sdmsscstep;
    NvU16 Sdmsscmax;
    NvU16 Sdmsscmin;
    NvU16 Sscmax;
    NvU8 Sscinc;
    NvU8 Sscincintrvl;
    NvU8 Integoffset;
    NvBool Ssccenter;
    NvBool Sscinvert;
    NvBool IsSSA;
} NvBootClocksSSParams;

#define NVBOOT_CLOCKS_CHECK_PLLID(PllId) \
    NV_ASSERT( (PllId == NvBootClocksPllId_PllC4) || \
               (PllId == NvBootClocksPllId_PllM) || \
               (PllId == NvBootClocksPllId_PllU) || \
               (PllId == NvBootClocksPllId_PllP) || \
               (PllId == NvBootClocksPllId_PllX) || \
               (PllId == NvBootClocksPllId_PllE) || \
               (PllId == NvBootClocksPllId_PllREFE)  ) ;

#define NVBOOT_CLOCKS_PLL_BASE(PLLID) ((((NvU32) PLLID) >> 16 ) & 0xFFFF)
#define NVBOOT_CLOCKS_PLL_MISC(PLLID) ((((NvU32) PLLID) >>  0 ) & 0xFFFF) 

/* 
 * NvBootClocksOscFreq NvBootClocksGetOscFreq(): get the current osc frequency
 */

NvBootClocksOscFreq 
NvBootClocksGetOscFreq(void);

/* 
 * void NvBootClocksSetOscFreq(NvBootClocksOscFreq): set the current
 * osc frequency
 */

void
NvBootClocksSetOscFreq(NvBootClocksOscFreq OscFreq);

/* 
 * NvBootCLocksBypassPll(): Put the PLL in bypass, insuring an active clock 
 *
 */
void
NvBootClocksBypassPll(NvBootClocksPllId PllId) ;

/* PLL Misc1 & Misc 2 mapping
 */
// PLLU
// LFCON and CPCON removed in T210

// PLLM
#define MISC2_PLLM_KVCO_LOW_BIT 0
#define MISC2_PLLM_KVCO_HIGH_BIT (MISC2_PLLM_KVCO_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KVCO_RANGE)-1)
#define MISC2_PLLM_KCP_LOW_BIT (MISC2_PLLM_KVCO_HIGH_BIT+1)
#define MISC2_PLLM_KCP_HIGH_BIT (MISC2_PLLM_KCP_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KCP_RANGE)-1)
#define MISC1_PLLM_SETUP_LOW_BIT 0
#define MISC1_PLLM_SETUP_HIGH_BIT (MISC1_PLLM_SETUP_LOW_BIT + \
                            NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_SETUP_RANGE)-1)


#define MISC2_CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KVCO_RANGE            MISC2_PLLM_KVCO_HIGH_BIT: MISC2_PLLM_KVCO_LOW_BIT
#define MISC2_CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KCP_RANGE             MISC2_PLLM_KCP_HIGH_BIT: MISC2_PLLM_KCP_LOW_BIT
#define MISC1_CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_SETUP_RANGE           MISC1_PLLM_SETUP_HIGH_BIT: MISC1_PLLM_SETUP_LOW_BIT
// PLLX
#define MISC2_PLLX_KVCO_LOW_BIT 0
#define MISC2_PLLX_KVCO_HIGH_BIT (MISC2_PLLX_KVCO_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KVCO_RANGE)-1)
#define MISC2_PLLX_KCP_LOW_BIT (MISC2_PLLX_KVCO_HIGH_BIT+1)
#define MISC2_PLLX_KCP_HIGH_BIT (MISC2_PLLX_KCP_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KCP_RANGE)-1)
#define MISC1_PLLX_SETUP_LOW_BIT 0
#define MISC1_PLLX_SETUP_HIGH_BIT (MISC1_PLLX_SETUP_LOW_BIT + \
                            NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_MISC_1_0_PLLX_SETUP_RANGE)-1)

#define MISC2_CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KVCO_RANGE           MISC2_PLLX_KVCO_HIGH_BIT: MISC2_PLLX_KVCO_LOW_BIT
#define MISC2_CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KCP_RANGE            MISC2_PLLX_KCP_HIGH_BIT: MISC2_PLLX_KCP_LOW_BIT
#define MISC1_CLK_RST_CONTROLLER_PLLX_MISC_1_0_PLLX_SETUP_RANGE          MISC1_PLLX_SETUP_HIGH_BIT: MISC1_PLLX_SETUP_LOW_BIT

/* 
 * NvBootClocksStartPll(): Start the identified PLL, track its stabilization time 
 */
void
NvBootClocksStartPll(NvBootClocksPllId PllId,
                     NvU32 M,
                     NvU32 N,
                     NvU32 P,
                     NvU32 Misc1,
                     NvU32 Misc2,
                     NvU32 *StableTime) ;            // Must be a valid address

/* 
 * NvBootClocksIsPllStable(): Check if the identified PLL is stable 
 */
NvBool
NvBootClocksIsPllStable(NvBootClocksPllId PllId, NvU32 StableTime);

/* 
 * NvBootClocksStopPll(): Stop the identified PLL, no check it is in use or not
 */
void
NvBootClocksStopPll(NvBootClocksPllId PllId);

/* 
 * NvBootClocksConfigureClock(): Configure the identified clock
 */
void
NvBootClocksConfigureClock(NvBootClocksClockId ClockId, 
                           NvU32 Divider,
                           NvU32 Source);

/*
 * NvBootClocksEnablePllSpreadSpectrum() : Enable Spread Spectrum in Pll
 */
void
NvBootClocksEnablePllSpreadSpectrum(NvBootClocksPllId PllId,
                           NvBootClocksSSParams *SSCoefficientTable);

/* For 7.1 the ratio can be an integer multiple of 0.5 larger than 1 */
/* the divider value to program equal the desired ratio * 2 - 2 or alternately 2 * (desired ratio - 1) */
/* to avoid floating point, this is done as 2 * integer part of ratio + 1 if odd multiple of 0.5 - 2 */
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY(INT_RATIO, PLUS_HALF) ( 2 * INT_RATIO + PLUS_HALF - 2)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_1    NVBOOT_CLOCKS_7_1_DIVIDER_BY( 1, 0)  
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_2    NVBOOT_CLOCKS_7_1_DIVIDER_BY( 2, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_3    NVBOOT_CLOCKS_7_1_DIVIDER_BY( 3, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_4    NVBOOT_CLOCKS_7_1_DIVIDER_BY( 4, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_5    NVBOOT_CLOCKS_7_1_DIVIDER_BY( 5, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_9    NVBOOT_CLOCKS_7_1_DIVIDER_BY( 9, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_13_5 NVBOOT_CLOCKS_7_1_DIVIDER_BY(13, 1)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_17   NVBOOT_CLOCKS_7_1_DIVIDER_BY(17, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_20   NVBOOT_CLOCKS_7_1_DIVIDER_BY(20, 0)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_67_5 NVBOOT_CLOCKS_7_1_DIVIDER_BY(67, 1)
#define NVBOOT_CLOCKS_7_1_DIVIDER_BY_270  NVBOOT_CLOCKS_7_1_DIVIDER_BY(270, 0)

/* for devices, reference is PLLP, so we can express that in absolute frequency */
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_432_0_MHZ NVBOOT_CLOCKS_7_1_DIVIDER_BY_1
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_216_0_MHZ NVBOOT_CLOCKS_7_1_DIVIDER_BY_2
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_144_0_MHZ NVBOOT_CLOCKS_7_1_DIVIDER_BY_3
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_108_0_MHZ NVBOOT_CLOCKS_7_1_DIVIDER_BY_4
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_86_4_MHZ  NVBOOT_CLOCKS_7_1_DIVIDER_BY_5
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_48_0_MHZ  NVBOOT_CLOCKS_7_1_DIVIDER_BY_9
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_25_4_MHZ  NVBOOT_CLOCKS_7_1_DIVIDER_BY_17
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_6_4_MHZ   NVBOOT_CLOCKS_7_1_DIVIDER_BY_67_5
#define NVBOOT_CLOCKS_PLLP_OUT0_DIV_TO_100_KHZ   NVBOOT_CLOCKS_7_1_DIVIDER_BY_270

/* some clocks have straight divider in n-1 format */
#define NVBOOT_CLOCKS_N_DIVIDER_BY(N) (N-1)
#define NVBOOT_CLOCKS_N_DIVIDER_BY_1 NVBOOT_CLOCKS_N_DIVIDER_BY(1)  
#define NVBOOT_CLOCKS_N_DIVIDER_BY_2 NVBOOT_CLOCKS_N_DIVIDER_BY(2)
#define NVBOOT_CLOCKS_N_DIVIDER_BY_3 NVBOOT_CLOCKS_N_DIVIDER_BY(3)
#define NVBOOT_CLOCKS_N_DIVIDER_BY_4 NVBOOT_CLOCKS_N_DIVIDER_BY(4)

/* 
 * NvBootClocksSetEnable(): Change the enable status
 */
void
NvBootClocksSetEnable(NvBootClocksClockId ClockId, NvBool Enable);

/* 
 * NvBootClocksSetAvpClockBeforeScatterLoad(): Special function that runs
 * before scatter loading. Assumes PLLP is stable.
 * Cannot use anything that rely on scatter loading, i.e. no dynamic global
 * Local variables (stack) and static global (ROM) should be OK
 */
void
NvBootClocksSetAvpClockBeforeScatterLoad(void);

/* 
 * NvBootClocksStartPllpBeforeScatterLoad(): Special function that runs
 * before scatter loading
 * Cannot use anything that rely on scatter loading, i.e. no dynamic global
 * Local variables (stack) and static global (ROM) should be OK
 */
void
NvBootClocksStartPllpBeforeScatterLoad(NvBootClocksOscFreq OscFreq);

/**
 * NvBootClocksMeasureOscFreq()
 *
 * This function use the HW engine in CAR to measure the Osc Frequency
 * against the 32 kHz clock.
 * This function may be called while in FA mode and so must be in 
 * the non secure area of memory
 *
 * @return: The osc frequency
 *
 */
NvBootClocksOscFreq
NvBootClocksMeasureOscFreq(void);

/* 
 * NvBootConfigureUsecTimer(): Special function that runs
 * before scatter loading
 * Cannot use anything that rely on scatter loading, i.e. no dynamic global
 * Local variables (stack) and static global (ROM) should be OK
 */
void
NvBootClocksConfigureUsecTimer(NvBootClocksOscFreq OscFreq);

/* 
 * NvBootClocksGetOscFreq(): 
 * This function obtains the actual osc frequency in Mhz
 */
NvU32 
NvBootClocksGetOscFreqMhz(void);

/*
 *  NvBootClocksGetPllMiscParams(NvBootClocks_ClocksId)
 *  
 *  Return pointer to table for PllMiscParams
 */
NvBootClocksMiscParams *
NvBootClocksGetPllMiscParams(NvBootClocksPllId PllId);

/*
 *  NvBootClocksIsPllC4Stable(NvU32 StableTime)
 *
 *  Version of IsPllStable for PllC4 only. Located in Nonsecure section
 */
NvBool
NvBootClocksIsPllC4Stable(NvU32 StableTime);


/*
 *  NvBootClocksIsPllC4Stable()
 *
 *  Version of StartPll for PllC4 only. Located in Nonsecure section
 *  Should be identical to the StartPll located in Secure Section for
 *  the PllC4 case.
 */
void
NvBootClocksStartPllC4(NvU32 M,
                       NvU32 N,
                       NvU32 P,
                       NvU32 Misc,
                       NvU32 *StableTime);

/*
 * Enable/Disable _RSTN to PLL OUTx dividers
 */
void
NvBootClocksPllDivRstCtrl(NvBootClocksPllId PllId, NvU32 EnDis);
#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CLOCKS_INT_H */
