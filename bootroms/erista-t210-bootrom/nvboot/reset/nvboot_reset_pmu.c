/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_reset_pmu.c - Implementation of PMU reset support.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "ari2c.h"
#include "argpio.h"
#include "arapb_misc.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_reset_pmu_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_util_int.h"
#include "project.h"

// Declarations for this file. Only the main handler is in the .h file
static NvBootError NvBootResetPmuI2cWrite(NvBootI2CContext *pI2CContext);
static NvBootError NvBootResetI2CSlaveBlock(NvU32 BlockNumber,
                                            NvU32 InterCommandDelay,
                                            NvU32 Retries,
                                            NvBootPmuCntlrTbl CntlrTbl);
static void NvBootResetTsenseHandler(NvU32 RstFlags, NvU32 WdtDuringBR);
static void NvBootResetProgramPowerRails(NvBootPmuCntlrTbl CntlrTbl);
static void NvBootResetI2cBusClear(NvU32 I2CBaseAddress, NvU32 DelayBeforeBusClear); 

#define I2C_BASE_ADDR NV_ADDRESS_MAP_I2C5_BASE

#define PINMUX_REG_SHIFT	0x10
#define I2C_PINMUX_ENTRY(p,m) \
    ((NvU32)(PINMUX_AUX_##p##_0) << PINMUX_REG_SHIFT) | \
	NV_DRF_DEF(PINMUX_AUX, p, PM, m) | \
	NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, PUPD, NONE) | \
	NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, TRISTATE, PASSTHROUGH) | \
	NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, E_INPUT, ENABLE) | \
	NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, E_OD, ENABLE) | \
        NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, E_SCHMT, ENABLE)

#define RSTFLAG_SENSOR      0x01
#define RSTFLAG_WDT         0x02
#define RSTFLAG_SWMAIN      0x04

// Size of rails programming header and slave block headers in bytes
#define HEADER_SIZE         4

// Maximum offset for a valid pinmux_aux register, used for GPIO reset
#define PINMUX_OFFSET_MAX (PINMUX_AUX0_LAST_REG - PINMUX_AUX0_FIRST_REG) / 4

/// Timeout of .2ms. Seshi V said this time would be OK for the cross domain load
#define CONFIG_LOAD_TIMEOUT_US 200

// One millisecond in microseconds 
#define TRANSACTION_COMPLETE_TIMEOUT_US 1000

#define reset_spinloop()  \
    do { \
    } while (1)

// Make sure the A and B registers are contiguous
NV_CT_ASSERT(PMU_SCRATCH_B_REG == (PMU_SCRATCH_A_REG + 0x04));
NV_CT_ASSERT(PMU_SCRATCH_B_WREG == (PMU_SCRATCH_A_WREG + 0x04));


static NvBootError NvBootResetPmuI2cWrite(NvBootI2CContext *pI2CContext)
{
    NvU32 RegData, TimeoutStartTime;
    NvBool IsBusy = NV_TRUE;
    //Put I2C controller's register initialization in a patchable var
    volatile NvU32 RegInitData = I2C_I2C_CNFG_0_SW_DEFAULT_VAL;

    // Load slave address
    RegData = NV_DRF_NUM(I2C, I2C_CMD_ADDR0, ADDR0, pI2CContext->Slv1Addr);
    RegData <<= 1;
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CMD_ADDR0_0, RegData);
    // Load Data
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CMD_DATA1_0, pI2CContext->Data1);
    // Load Control
    RegData = RegInitData;
    RegData |= pI2CContext->Control;
    RegData = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, MULTI_MASTER_MODE, ENABLE, RegData);
    RegData = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, SEND, NOP, RegData);
    RegData = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, PACKET_MODE_EN, NOP, RegData);
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CNFG_0, RegData);

    // Write MSTR_CONFIG_LOAD
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CONFIG_LOAD_0,
               NV_DRF_DEF(I2C, I2C_CONFIG_LOAD, MSTR_CONFIG_LOAD, ENABLE));

    /// Wait for i2c configuration to be loaded from pclk domain to i2c_clk domain
    /// and the controller to auto-clear the corresponding bit.
    TimeoutStartTime = NvBootUtilGetTimeUS();
    while(NvBootUtilElapsedTimeUS(TimeoutStartTime) < CONFIG_LOAD_TIMEOUT_US) {
        RegData = NV_READ32(I2C_BASE_ADDR + I2C_I2C_CONFIG_LOAD_0);
        if (NV_DRF_VAL(I2C, I2C_CONFIG_LOAD, MSTR_CONFIG_LOAD, RegData) == 0) {
            break;
        }
    }

    // Hit "GO"
    RegData = NV_READ32(I2C_BASE_ADDR + I2C_I2C_CNFG_0);
    RegData = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, SEND, GO, RegData);
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CNFG_0, RegData);

    // Wait for transaction to complete, times out in 1ms
    TimeoutStartTime = NvBootUtilGetTimeUS();
    while(NvBootUtilElapsedTimeUS(TimeoutStartTime) < TRANSACTION_COMPLETE_TIMEOUT_US && IsBusy) {
        RegData = NV_READ32(I2C_BASE_ADDR + I2C_I2C_STATUS_0);
        if (NV_DRF_VAL(I2C, I2C_STATUS, BUSY, RegData) != I2C_I2C_STATUS_0_BUSY_BUSY) {
            IsBusy = NV_FALSE;
        }
    }
    if(IsBusy) {
        // Timed out above
        return NvBootError_Busy;
    }

    // Check Transaction success
    RegData = NV_READ32(I2C_BASE_ADDR + I2C_I2C_STATUS_0);
    RegData = NV_DRF_VAL(I2C, I2C_STATUS, CMD1_STAT, RegData);
    return (RegData == I2C_I2C_STATUS_0_CMD1_STAT_SL1_XFER_SUCCESSFUL) ?
                           NvBootError_Success : NvBootError_TxferFailed;
}

static NvBool NvBootResetValidatePmuScratch(NvU32 reg)
{
    NvU8 cbChksum = 0;
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + reg);
    cbChksum += RegData & 0xFF;
    RegData >>= 0x08;
    cbChksum += RegData & 0xFF;
    RegData >>= 0x08;
    cbChksum += RegData & 0xFF;
    RegData >>= 0x08;
    cbChksum += RegData & 0xFF;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + (reg + 0x04));
    cbChksum += RegData & 0xFF;
    RegData >>= 0x08;
    cbChksum += RegData & 0xFF;
    RegData >>= 0x08;
    cbChksum += RegData & 0xFF;
    RegData >>= 0x08;
    cbChksum += RegData & 0xFF;

    return ((cbChksum == 0)? NV_TRUE : NV_FALSE);
}

static NvBool NvBootResetQuerySWMainFlag()
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
            APBDEV_PMC_RST_STATUS_0_RST_SOURCE_SW_MAIN)
        return NV_TRUE;

    return NV_FALSE;
}

static NvBool NvBootResetQueryTsenseFlag()
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
            APBDEV_PMC_RST_STATUS_0_RST_SOURCE_SENSOR)
        return NV_TRUE;

    return NV_FALSE;
}

static NvBool NvBootResetQueryAOTAGFlag()
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
            APBDEV_PMC_RST_STATUS_0_RST_SOURCE_AOTAG)
        return NV_TRUE;

    return NV_FALSE;
}

static NvBool NvBootResetQueryWDTFlag()
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
            APBDEV_PMC_RST_STATUS_0_RST_SOURCE_WATCHDOG)
    {
            return NV_TRUE;
    }
    return NV_FALSE;
}

static void NvBootResetTsenseHandler(NvU32 RstFlags, NvU32 WdtDuringBR)
{
    NvU32 RegData, AReg = 0;
    NvU16 GpioPinmuxOffset = 0;
    NvU8 UseGpio;
    NvU8 PmuContext[sizeof(NvBootI2CContext)];
    NvBool GpioPinmuxEnable = NV_FALSE;

    NvBootI2CContext *pI2cContext = (NvBootI2CContext *) &PmuContext;
    NvBootGpioContext *pGpioContext = (NvBootGpioContext *) &PmuContext;
    NV_CT_ASSERT(sizeof(NvBootI2CContext) == sizeof(NvBootGpioContext));

    //ECO from NvBug #739786 - Re-enable SCRATCH regs access
    //after they are locked down by Tsense reset.
    //Available on NET12 onwards.
#ifdef APBDEV_PMC_SENSOR_CTRL_0_BLOCK_SCRATCH_WRITE_FIELD
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SENSOR_CTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(APBDEV_PMC, SENSOR_CTRL, BLOCK_SCRATCH_WRITE, OFF,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SENSOR_CTRL_0, RegData);
#endif

    if (RstFlags & RSTFLAG_WDT)
    {
        if (NvBootGetSwCYA() & NVBOOT_SW_CYA_WDT_RST_DISABLE) {
            //Do not issue second reset because of WDT reset
            return;
        }
        if (WdtDuringBR == 0) {
            return;
        }
    }

    if (RstFlags & RSTFLAG_SENSOR) {
        //We have Tsense reset event. Validate SCRATCHs used for PMU reset.
        //If the SCRATCHs were not properly set up, spinloop().
        if (!NvBootResetValidatePmuScratch(PMU_SCRATCH_A_REG)) {
            reset_spinloop();
        }
        RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_B_REG);
        //If RST_ENABLE == 0, spinloop().
        if (NV_DRF_VAL(SCRATCH, PMU_B, RST_ENABLE, RegData) == 0){
            reset_spinloop();
        }
        AReg = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_A_REG);
    }

    if (RstFlags & RSTFLAG_WDT) {
        //We have WDT reset event. If SCRATCHs validation failed just return.
        if (!(NvBootResetValidatePmuScratch(PMU_SCRATCH_A_WREG))) {
            return;
        }
        RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_B_WREG);
        if (NV_DRF_VAL(SCRATCH, PMU_B, RST_ENABLE, RegData) == 0){
            return;
        }
        AReg = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_A_WREG);
    }

    if (RstFlags & RSTFLAG_SWMAIN) {
        //No PMU reset for SW_MAIN
        return;
    }

    UseGpio = NV_DRF_VAL(SCRATCH, PMU_B, USE_GPIO, RegData);

    if(!UseGpio)
    {
        // I2C5 is the only entry, so Id is 0
        pI2cContext->CntlrId = 0;
        pI2cContext->Slv1Addr = NV_DRF_VAL(SCRATCH, PMU_B, I2CSLV1, RegData);
        pI2cContext->Data1 = AReg;
        pI2cContext->Control = (NV_DRF_VAL(SCRATCH, PMU_B, 16BITOP, RegData))?
                               NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, 0x03) :
                               NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, 0x01);

        //Finally, deliver PMU reset....
        (void) NvBootResetPmuI2cWrite(pI2cContext);
        //Shouldn't return, but if it does
    } else {
        // Only valid when USE GPIO is set
        GpioPinmuxOffset = NV_DRF_VAL(SCRATCH, PMU_B, PINMUX_OFFSET, RegData);
        GpioPinmuxEnable = NV_DRF_VAL(SCRATCH, PMU_B, PINMUX_ENABLE, RegData);

        pGpioContext->GpioPort = NV_DRF_VAL(SCRATCH, PMU_B, GPIO_PORT, RegData);
        pGpioContext->GpioId = NV_DRF_VAL(SCRATCH, PMU_B, GPIO_CNTLR, RegData);
        pGpioContext->RegCnfOE = NV_DRF_VAL(SCRATCH, PMU_A, LOWORD, AReg);
        pGpioContext->RegOut = NV_DRF_VAL(SCRATCH, PMU_A, HIWORD, AReg);

        // If PinmuxEnable is set and PinmuxOffset is valid, set the TRISTATE bit to
        // NORMAL in a PINMUX_AUX register. Without this the GPIO write won't work.
        if(GpioPinmuxEnable && GpioPinmuxOffset <= PINMUX_OFFSET_MAX) {
            RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + PINMUX_AUX0_FIRST_REG + 
                                4 * GpioPinmuxOffset);
            // All pinmux_aux registers look like SDMMC1_CLK, so use it for field names
            RegData = NV_FLD_SET_DRF_DEF(PINMUX_AUX, SDMMC1_CLK, TRISTATE, PASSTHROUGH, RegData);
            NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + PINMUX_AUX0_FIRST_REG +
                       4 * GpioPinmuxOffset,
                       RegData);
        }

        //Allocate and drive GPIO to deliver PMU reset....
        NV_WRITE32(NV_ADDRESS_MAP_GPIO1_BASE + (pGpioContext->GpioId * 0x100) +
                   (pGpioContext->GpioPort * 0x04) + GPIO_MSK_CNF, pGpioContext->RegCnfOE);
        NV_WRITE32(NV_ADDRESS_MAP_GPIO1_BASE + (pGpioContext->GpioId * 0x100) +
                   (pGpioContext->GpioPort * 0x04) + GPIO_MSK_OE, pGpioContext->RegCnfOE);
        NV_WRITE32(NV_ADDRESS_MAP_GPIO1_BASE + (pGpioContext->GpioId * 0x100) +
                   (pGpioContext->GpioPort * 0x04) + GPIO_MSK_OUT, pGpioContext->RegOut);
    }

    //!!!! Should never reach here !!!!
    reset_spinloop();
}

void NvBootResetRailsAndResetHandler()
{
    NvU32 RegData, WdtDuringBr;
    NvU32 RstFlags = 0;
    NvU8 NumPins;

    // Put formerly global data here. Pass it if we need it elsewhere
    const NvU32 I2c5Pinmux[] = {
        I2C_PINMUX_ENTRY(PWR_I2C_SDA, I2CPMU),
        I2C_PINMUX_ENTRY(PWR_I2C_SCL, I2CPMU),
    };

    // As of t210 We only support one I2C config, but keep the table for ease
    const NvBootPmuCntlrTbl I2cPmuCntlrData = {NvBootResetDeviceId_I2c5Id,
                                               NvBootClocksClockId_I2c5Id};

    RstFlags |= NvBootResetQueryTsenseFlag() ? RSTFLAG_SENSOR : 0;
    // Not a typo, Tsense flag and AOTAG flag should be handled the same
    RstFlags |= NvBootResetQueryAOTAGFlag() ? RSTFLAG_SENSOR : 0;
    RstFlags |= NvBootResetQueryWDTFlag() ? RSTFLAG_WDT : 0;
    RstFlags |= NvBootResetQuerySWMainFlag() ? RSTFLAG_SWMAIN : 0;

    //Check if Tsense reset flag is set or WDT flag is set, otherwise nothing to do
    //in this function.
    if (RstFlags == 0) {
        // When RST_STATUS is POR we clear a few scratch registers
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_HEADER, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_A_WREG, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_B_WREG, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_A_REG, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + PMU_SCRATCH_B_REG, 0);

        // We set WDT_DURING_BR bit (SCRATCH190 bit 0) for LP0 path
        RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
        if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) ==
                       APBDEV_PMC_RST_STATUS_0_RST_SOURCE_LP0)
        {
            RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_WDT_INFO);
            RegData = NV_FLD_SET_DRF_NUM(SCRATCH_RAIL, WDT_INFO, WDT_DURING_BR, 1, RegData);
            NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_WDT_INFO, RegData);
        }
        return;
    }

    // If BootROM returns to having multiple options for I2C controller, pinmux
    // etc. then this will have to be relocated and a more sophisticated init
    // used, as different blocks could use different controllers, and either
    // reinit'ing or keeping track between Reset and Rails which have
    // been init'd may be neccesary. For now doing init once here will be fine.
    //This is I2C PMU. Set up pinmux for I2C
    for (NumPins = 0; NumPins < 2; NumPins++) {
        NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE +
                   (I2c5Pinmux[NumPins] >> PINMUX_REG_SHIFT),
                   I2c5Pinmux[NumPins] & 0x1FF);
    }

    //Assert I2C controller reset
    NvBootResetSetEnable(I2cPmuCntlrData.DeviceId, NV_TRUE);

    NvBootClocksConfigureClock(I2cPmuCntlrData.ClockId,
                               NVBOOT_CLOCKS_7_1_DIVIDER_BY_1,
                               CLK_RST_CONTROLLER_CLK_SOURCE_I2C5_0_I2C5_CLK_SRC_CLK_M);
    //Enable clock to I2C controller
    NvBootClocksSetEnable(I2cPmuCntlrData.ClockId, NV_TRUE);
    //De-aasert I2C controller reset
    NvBootResetSetEnable(I2cPmuCntlrData.DeviceId, NV_FALSE);
    //Select CLK_M for I2C controller, either 12 MHz or 38.4MHz, reset value in TLOW, THIGH:
    //I2C_clock = CLK_SOURCE / (8*(InternalDivisor+1))
    //Target I2C clock is between 100 and 400kHz. Internal Divisor = 104, Predivisor = 1.
    RegData = NV_READ32(I2C_BASE_ADDR + I2C_I2C_CLK_DIVISOR_REGISTER_0);
    RegData = NV_DRF_VAL(I2C, I2C_CLK_DIVISOR_REGISTER, I2C_CLK_DIVISOR_STD_FAST_MODE, 12);
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CLK_DIVISOR_REGISTER_0, RegData);

    // Wait as the PMIC will not accept i2c commands immediately after warm reset
    // This is for MAX77660 and TPS65913, related bug is #1215721
    NvBootUtilWaitUS(33 * 1000);

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_WDT_INFO);
    WdtDuringBr = NV_DRF_VAL(SCRATCH_RAIL, WDT_INFO, WDT_DURING_BR, RegData);
    // WDT, TSense, and SW_MAIN should have WDT_DURING_DR (SCRATCH190) set
    // not doing a read modify write here to reflect behavior of BR after patch
    RegData = NV_FLD_SET_DRF_NUM(SCRATCH_RAIL, WDT_INFO, WDT_DURING_BR, 1, 0);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_WDT_INFO, RegData);

    // First Try Reset, then Program Power Rails if we make it here 
    NvBootResetTsenseHandler(RstFlags, WdtDuringBr);

    // If we did not hang or power off above, let's adjust rails for coldboot
    NvBootResetProgramPowerRails(I2cPmuCntlrData);
}

static void NvBootResetProgramPowerRails(NvBootPmuCntlrTbl CntlrTbl)
{
    NvU32 DelayBeforeBusClear, DelayBetweenCommands;
    NvU32 I2CSlaveConfigBlocks;
    NvU32 RetriesPerCommand;
    NvU32 RegData;
    NvBootError BlockSuccess;
    NvU32 i;

    // Extract Header Data
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_HEADER);
    DelayBeforeBusClear = (1 << NV_DRF_VAL(SCRATCH, RAIL_HEADER, BUS_CLR_DELAY, RegData));
    DelayBetweenCommands = (1 << NV_DRF_VAL(SCRATCH, RAIL_HEADER, INTERTX_DELAY, RegData));
    I2CSlaveConfigBlocks = NV_DRF_VAL(SCRATCH, RAIL_HEADER, I2CSLV_BLKS, RegData);
    RetriesPerCommand = NV_DRF_VAL(SCRATCH, RAIL_HEADER, TX_RETRIES, RegData);

    // In the future, if multiple controllers are back, we will have to lookahead
    // to the next slave block and Bus Clear the correct controller
    NvBootResetI2cBusClear(I2C_BASE_ADDR, DelayBeforeBusClear);

    for(i = 0; i < I2CSlaveConfigBlocks; i++) {
        BlockSuccess = NvBootResetI2CSlaveBlock(i, DelayBetweenCommands, RetriesPerCommand, CntlrTbl);

        // We send another Bus Clear if we fail, but still continue to the next block
        if(BlockSuccess != NvBootError_Success) {
            NvBootResetI2cBusClear(I2C_BASE_ADDR, DelayBeforeBusClear);
        }
    }
}

static NvBootError NvBootResetI2CSlaveBlock(NvU32 BlockNumber, NvU32 InterCommandDelay, NvU32 Retries, NvBootPmuCntlrTbl CntlrTbl)
{
    NvU32 RegData = 0;
    NvU32 i;
    NvU32 NumberOfCommands = 0;
    NvU32 BytesToCheck;
    NvU32 SlaveBlockOffset;
    NvU32 CmdOffset = 0;
    NvBootError TxError;
    NvU8 Checksum = 0;

    NvBool IsSuccessful;
    NvBool Is32BitOp = NV_FALSE;

    NvU8 PmuContext[sizeof(NvBootI2CContext)];
    NvBootI2CContext *pI2cContext = (NvBootI2CContext *) &PmuContext;

    // Find beginning of requested block
    SlaveBlockOffset = HEADER_SIZE;
    for(i = 0; i <= BlockNumber; i++) {
        RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_HEADER + SlaveBlockOffset);
        NumberOfCommands = NV_DRF_VAL(SCRATCH, RAIL_BLOCK_HEADER, NUM_WRITE_CMDS, RegData);
        Is32BitOp = NV_DRF_VAL(SCRATCH, RAIL_BLOCK_HEADER, 16BITOP, RegData) ? NV_TRUE : NV_FALSE;

        if(i < BlockNumber) {
            SlaveBlockOffset += ((Is32BitOp ? 4 : 2)*NumberOfCommands / 4 + 1) * 4;
        }
    }
        
    //Call it a success if RST_EN isn't set and we have nothing to do
    if(NV_DRF_VAL(SCRATCH, RAIL_BLOCK_HEADER, RST_EN, RegData) == 0) {
        return NvBootError_Success;
    }

    //Sum of all bytes in payload AND 0xFF should equal 0
    BytesToCheck = HEADER_SIZE + (Is32BitOp ? 4 : 2)*NumberOfCommands;
    for(i = 0; i < BytesToCheck; i++) {
        Checksum += NV_READ8(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_HEADER + SlaveBlockOffset + i);
    }
    if(Checksum != 0) {
        return NvBootError_ValidationFailure;
    }

    //Set up an I2C context
    pI2cContext->CntlrId = 0;
    pI2cContext->Slv1Addr = NV_DRF_VAL(SCRATCH, RAIL_BLOCK_HEADER, I2CSLV_ADDRESS, RegData);
    pI2cContext->Control = Is32BitOp ? NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, 0x03) 
                                     : NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, 0x01);

    //Reuse the basic context for each command sent, just change the data field
    while(CmdOffset < NumberOfCommands) {
        if(Is32BitOp) {
            pI2cContext->Data1 = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_HEADER +
                                           SlaveBlockOffset + HEADER_SIZE + 4*CmdOffset);
        } else {
            RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_HEADER +
                                SlaveBlockOffset + HEADER_SIZE + 4*(CmdOffset/2));
            pI2cContext->Data1 = (CmdOffset % 2) ? ((RegData >> 16) & 0xFFFF) : (RegData & 0xFFFF);
        }

        // Loop over retries
        IsSuccessful = NV_FALSE;
        i = 0;
        while(i < Retries && !IsSuccessful) {
            TxError = NvBootResetPmuI2cWrite(pI2cContext);
            NvBootUtilWaitUS(InterCommandDelay);

            if(TxError == NvBootError_Success) {
                IsSuccessful = NV_TRUE;
            } else if( TxError == NvBootError_Busy) {
                // No good way to handle stuck at busy, but will try resetting controller
                NvBootResetSetEnable(CntlrTbl.DeviceId, NV_TRUE);
                NvBootResetSetEnable(CntlrTbl.DeviceId, NV_FALSE);
            }
            i++;
        }
        if(!IsSuccessful) {
            return NvBootError_TxferFailed;
        }
        CmdOffset++;
    }
    return NvBootError_Success;
}

static void NvBootResetI2cBusClear(NvU32 I2cBaseAddress, NvU32 USDelayBeforeBusClear)
{
    NvU32 RegData, TimeoutStartTime;

    NvBootUtilWaitUS(USDelayBeforeBusClear);

    RegData = I2C_I2C_BUS_CLEAR_CONFIG_0_RESET_VAL;
    RegData = NV_FLD_SET_DRF_DEF(I2C, I2C_BUS_CLEAR_CONFIG, BC_TERMINATE, IMMEDIATE, RegData);
    RegData = NV_FLD_SET_DRF_NUM(I2C, I2C_BUS_CLEAR_CONFIG, BC_SCLK_THRESHOLD, 9, RegData);
    RegData = NV_FLD_SET_DRF_DEF(I2C, I2C_BUS_CLEAR_CONFIG, BC_STOP_COND, NO_STOP, RegData);
    NV_WRITE32(I2cBaseAddress + I2C_I2C_BUS_CLEAR_CONFIG_0, RegData);

    // Write MSTR_CONFIG_LOAD
    NV_WRITE32(I2cBaseAddress + I2C_I2C_CONFIG_LOAD_0, 
               NV_DRF_DEF(I2C, I2C_CONFIG_LOAD, MSTR_CONFIG_LOAD, ENABLE));

    /// Wait for i2c configuration to be loaded from pclk domain to i2c_clk domain
    /// and the controller to auto-clear the corresponding bit.
    TimeoutStartTime = NvBootUtilGetTimeUS();
    while(NvBootUtilElapsedTimeUS(TimeoutStartTime) < CONFIG_LOAD_TIMEOUT_US) {
        RegData = NV_READ32(I2cBaseAddress + I2C_I2C_CONFIG_LOAD_0);
        if (NV_DRF_VAL(I2C, I2C_CONFIG_LOAD, MSTR_CONFIG_LOAD, RegData) == 0) {
            break;
        }
    }

    RegData = NV_READ32(I2cBaseAddress + I2C_I2C_BUS_CLEAR_CONFIG_0);
    RegData = NV_FLD_SET_DRF_NUM(I2C, I2C_BUS_CLEAR_CONFIG, BC_ENABLE, 1, RegData);
    NV_WRITE32(I2cBaseAddress + I2C_I2C_BUS_CLEAR_CONFIG_0, RegData);

    // Wait for Bus Clear to complete, timeout in 1ms
    TimeoutStartTime = NvBootUtilGetTimeUS();
    while(NvBootUtilElapsedTimeUS(TimeoutStartTime) < TRANSACTION_COMPLETE_TIMEOUT_US) {
        RegData = NV_READ32(I2cBaseAddress + I2C_INTERRUPT_STATUS_REGISTER_0);
        if (NV_DRF_VAL(I2C, INTERRUPT_STATUS_REGISTER, BUS_CLEAR_DONE, RegData) == 1) {
            break;
        }
    }

    // Write '1' to clear BUS_CLEAR_DONE
    RegData = NV_DRF_DEF(I2C, INTERRUPT_STATUS_REGISTER, BUS_CLEAR_DONE, SET);
    NV_WRITE32(I2cBaseAddress + I2C_INTERRUPT_STATUS_REGISTER_0, RegData);
}
