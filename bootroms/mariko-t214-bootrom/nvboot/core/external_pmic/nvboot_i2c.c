/*
 * Copyright (c) 2007 - 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvcommon.h"
#include "project.h"

#include "nvrm_drf.h"
#include "ari2c.h"
#include "arclk_rst.h"
#include "arapbpm.h"
#include "arapb_misc.h"

#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"

#include "nvboot_i2c.h"

#define PINMUX_REG_SHIFT        0x10
#define I2C_PINMUX_ENTRY(p,m) \
    ((NvU32)(PINMUX_AUX_##p##_0) << PINMUX_REG_SHIFT) | \
        NV_DRF_DEF(PINMUX_AUX, p, PM, m) | \
        NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, PUPD, NONE) | \
        NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, TRISTATE, PASSTHROUGH) | \
        NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, E_INPUT, ENABLE) | \
        NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, E_OD, ENABLE) | \
        NV_DRF_DEF(PINMUX_AUX, PWR_I2C_SDA, E_SCHMT, ENABLE)

#define I2C_BASE_ADDR NV_ADDRESS_MAP_I2C5_BASE

void NvBootI2cInit(const NvBootI2cCntlrTbl i2cCntlr)
{
    // If BootROM returns to having multiple options for I2C controller, pinmux
    // etc. then this will have to be relocated and a more sophisticated init
    const NvU32 I2c5Pinmux[] = {
        I2C_PINMUX_ENTRY(PWR_I2C_SDA, I2CPMU),
        I2C_PINMUX_ENTRY(PWR_I2C_SCL, I2CPMU),
    };

    for (NvU32 NumPins = 0; NumPins < 2; NumPins++) {
        NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE +
                   (I2c5Pinmux[NumPins] >> PINMUX_REG_SHIFT),
                   I2c5Pinmux[NumPins] & 0x1FF);
    }

    //Assert I2C controller reset
    NvBootResetSetEnable(i2cCntlr.DeviceId, NV_TRUE);

    NvBootClocksConfigureClock(i2cCntlr.ClockId,
                               NVBOOT_CLOCKS_7_1_DIVIDER_BY_1,
                               CLK_RST_CONTROLLER_CLK_SOURCE_I2C5_0_I2C5_CLK_SRC_CLK_M);
    //Enable clock to I2C controller
    NvBootClocksSetEnable(i2cCntlr.ClockId, NV_TRUE);
    //De-aasert I2C controller reset
    NvBootResetSetEnable(i2cCntlr.DeviceId, NV_FALSE);
    //Select CLK_M for I2C controller, either 12 MHz or 38.4MHz, reset value in TLOW, THIGH:
    //I2C_clock = CLK_SOURCE / (8*(InternalDivisor+1))
    //Target I2C clock is between 100 and 400kHz. Internal Divisor = 104, Predivisor = 1.
    //TODO: is this bug?
    NvU32 I2cClkDivisor = NV_READ32(I2C_BASE_ADDR + I2C_I2C_CLK_DIVISOR_REGISTER_0);
    I2cClkDivisor = NV_DRF_VAL(I2C, I2C_CLK_DIVISOR_REGISTER, I2C_CLK_DIVISOR_STD_FAST_MODE, 12);
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CLK_DIVISOR_REGISTER_0, I2cClkDivisor);
}

NvBootError FT_NONSECURE NvBootI2cWrite(NvBootI2CContext *pI2CContext)
{
    NvU32 RegData, TimeoutStartTime;
    NvBool IsBusy = NV_TRUE;
    //Put I2C controller's register initialization in a patchable var
    volatile NvU32 DefaultCnfg = I2C_I2C_CNFG_0_SW_DEFAULT_VAL;

    // Load slave address
    RegData = NV_DRF_NUM(I2C, I2C_CMD_ADDR0, ADDR0, pI2CContext->Slv1Addr);
    RegData <<= 1;
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CMD_ADDR0_0, RegData);
    // Load Data
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CMD_DATA1_0, pI2CContext->Data1);
    // Load Control
    NvU32 I2cCnfg = DefaultCnfg;
    I2cCnfg |= (pI2CContext->Is32BitOp ? NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, 0x03)
                                       : NV_DRF_NUM(I2C, I2C_CNFG, LENGTH, 0x01));
    I2cCnfg = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, MULTI_MASTER_MODE, ENABLE, I2cCnfg);
    I2cCnfg = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, SEND, NOP, I2cCnfg);
    I2cCnfg = NV_FLD_SET_DRF_DEF(I2C, I2C_CNFG, PACKET_MODE_EN, NOP, I2cCnfg);
    NV_WRITE32(I2C_BASE_ADDR + I2C_I2C_CNFG_0, I2cCnfg);

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

void FT_NONSECURE NvBootI2cBusClear(NvU32 I2cBaseAddress, NvU32 USDelayBeforeBusClear)
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

