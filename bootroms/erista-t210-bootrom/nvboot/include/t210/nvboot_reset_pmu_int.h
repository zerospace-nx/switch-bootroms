
/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_reset_pmu_int.h - Declarations for reset support.
 * 
 */

#ifndef INCLUDED_NVBOOT_RESET_PMU_INT_H
#define INCLUDED_NVBOOT_RESET_PMU_INT_H


#if defined(__cplusplus)
extern "C"
{
#endif

#define SCRATCH_PMU_A_0_LOWORD_RANGE		15:0
#define SCRATCH_PMU_A_0_HIWORD_RANGE		31:16    

#define SCRATCH_PMU_B_0_I2CSLV1_RANGE		 6:0
#define SCRATCH_PMU_B_0_GPIO_CNTLR_RANGE	 3:0
#define SCRATCH_PMU_B_0_PINMUX_OFFSET_RANGE	12:4
#define SCRATCH_PMU_B_0_PINMUX_ENABLE_RANGE	13:13
#define SCRATCH_PMU_B_0_USE_GPIO_RANGE		14:14
#define SCRATCH_PMU_B_0_16BITOP_RANGE		15:15	
#define SCRATCH_PMU_B_0_CHKSUM_RANGE		23:16
#define SCRATCH_PMU_B_0_GPIO_PORT_RANGE		26:24
#define SCRATCH_PMU_B_0_CNTLR_ID_RANGE		29:27
#define SCRATCH_PMU_B_0_CNTLR_TYPE_RANGE	30:30
#define SCRATCH_PMU_B_0_RST_ENABLE_RANGE	31:31

#define PMU_SCRATCH_A_WREG  APBDEV_PMC_SCRATCH52_0
#define PMU_SCRATCH_B_WREG  APBDEV_PMC_SCRATCH53_0    
#define PMU_SCRATCH_A_REG   APBDEV_PMC_SCRATCH54_0
#define PMU_SCRATCH_B_REG   APBDEV_PMC_SCRATCH55_0

// For NvBootResetProgramPowerRails()
#define SCRATCH_RAIL_HEADER_0_TX_RETRIES_RANGE     2:0
#define SCRATCH_RAIL_HEADER_0_INTERTX_DELAY_RANGE  7:3
#define SCRATCH_RAIL_HEADER_0_I2CSLV_BLKS_RANGE   10:8
#define SCRATCH_RAIL_HEADER_0_BUS_CLR_DELAY_RANGE 15:11

#define SCRATCH_RAIL_BLOCK_HEADER_0_I2CSLV_ADDRESS_RANGE  6:0
#define SCRATCH_RAIL_BLOCK_HEADER_0_NUM_WRITE_CMDS_RANGE 13:8
#define SCRATCH_RAIL_BLOCK_HEADER_0_16BITOP_RANGE        15:15
#define SCRATCH_RAIL_BLOCK_HEADER_0_CHKSUM_RANGE         22:16
#define SCRATCH_RAIL_BLOCK_HEADER_0_RST_EN_RANGE         31:31

#define SCRATCH_RAIL_WDT_INFO_0_WDT_DURING_BR_RANGE 0:0

#define SCRATCH_RAIL_HEADER APBDEV_PMC_SCRATCH250_0
#define SCRATCH_RAIL_WDT_INFO APBDEV_PMC_SCRATCH190_0

typedef struct NvBootI2CContextRec {
    NvU32 CntlrId;
    NvU16 Control;
    NvU16 Slv1Addr;
    NvU32 Data1;
} NvBootI2CContext;

typedef struct NvBootGpioContextRec {
    NvU16 GpioId;
    NvU16 GpioPort;
    NvU32 RegCnfOE;
    NvU32 RegOut;
} NvBootGpioContext;

typedef struct NvBootPmuCntlrTblRec {
    NvBootResetDeviceId DeviceId;
    NvBootClocksClockId ClockId;
} NvBootPmuCntlrTbl;

void NvBootResetRailsAndResetHandler(void);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_RESET_PMU_INT_H */
