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
#include "nvboot_i2c.h"

// Declarations for this file. Only the main handler is in the .h file
static NvBootError NvBootResetI2CSlaveBlock(NvU32 BlockNumber,
                                            NvU32 InterCommandDelay,
                                            NvU32 Retries,
                                            NvBootI2cCntlrTbl CntlrTbl);
static void NvBootResetTsenseHandler(NvU32 RstFlags, NvU32 WdtDuringBR);
static void NvBootResetProgramPowerRails(NvBootI2cCntlrTbl CntlrTbl);

#define I2C_BASE_ADDR NV_ADDRESS_MAP_I2C5_BASE

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

NvU32 CalculateBlockLength(NvU32 numCmds, NvBool is16Or32)
{
    size_t CmdSize = is16Or32 ? 4 : 2;
    // Blocks are DWORD aligned, this is a Ceiling function for DWORD size
    return (((CmdSize*numCmds) + 4 - 1)/4)*4 + HEADER_SIZE;
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
        //Set up an I2C context
        // I2C5 is the only entry, so Id is 0
        NvBool SendSize = NV_DRF_VAL(SCRATCH, PMU_B, 16BITOP, RegData);
        pI2cContext->Is32BitOp = SendSize ? NV_TRUE : NV_FALSE;
        pI2cContext->CntlrId = 0;
        pI2cContext->Slv1Addr = NV_DRF_VAL(SCRATCH, PMU_B, I2CSLV1, RegData);
        pI2cContext->Data1 = AReg;

        //Finally, deliver PMU reset....
        (void) NvBootI2cWrite(pI2cContext);
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

    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_WDT_INFO);
    WdtDuringBr = NV_DRF_VAL(SCRATCH_RAIL, WDT_INFO, WDT_DURING_BR, RegData);
    // WDT, TSense, and SW_MAIN should have WDT_DURING_DR (SCRATCH190) set
    RegData = NV_FLD_SET_DRF_NUM(SCRATCH_RAIL, WDT_INFO, WDT_DURING_BR, 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + SCRATCH_RAIL_WDT_INFO, RegData);

    // As of t210 We only support one I2C config, but keep the table for ease
    const NvBootI2cCntlrTbl I2cPmuCntlrData = {NvBootResetDeviceId_I2c5Id,
                                               NvBootClocksClockId_I2c5Id};
    NvBootI2cInit(I2cPmuCntlrData);

    // Wait as the PMIC will not accept i2c commands immediately after warm reset
    // This is for MAX77660 and TPS65913, related bug is #1215721
    NvBootUtilWaitUS(33 * 1000);

    // First Try Reset, then Program Power Rails if we make it here 
    NvBootResetTsenseHandler(RstFlags, WdtDuringBr);

    // If we did not hang or power off above, let's adjust rails for coldboot
    NvBootResetProgramPowerRails(I2cPmuCntlrData);
}

static void NvBootResetProgramPowerRails(NvBootI2cCntlrTbl CntlrTbl)
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
    NvBootI2cBusClear(I2C_BASE_ADDR, DelayBeforeBusClear);

    for(i = 0; i < I2CSlaveConfigBlocks; i++) {
        BlockSuccess = NvBootResetI2CSlaveBlock(i, DelayBetweenCommands, RetriesPerCommand, CntlrTbl);

        // We send another Bus Clear if we fail, but still continue to the next block
        if(BlockSuccess != NvBootError_Success) {
            NvBootI2cBusClear(I2C_BASE_ADDR, DelayBeforeBusClear);
        }
    }
}

static NvBootError NvBootResetI2CSlaveBlock(NvU32 BlockNumber, NvU32 InterCommandDelay, NvU32 Retries, NvBootI2cCntlrTbl CntlrTbl)
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
            SlaveBlockOffset += CalculateBlockLength(NumberOfCommands, Is32BitOp);
        }
    }
        
    //Call it a success if RST_EN isn't set and we have nothing to do
    if(NV_DRF_VAL(SCRATCH, RAIL_BLOCK_HEADER, RST_EN, RegData) == 0) {
        return NvBootError_Success;
    }

    //Sum of all bytes in payload AND 0xFF should equal 0
    // Don't use calculate block length here, to match t210 behvaior
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
    pI2cContext->Is32BitOp = Is32BitOp;

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
            TxError = NvBootI2cWrite(pI2cContext);
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
