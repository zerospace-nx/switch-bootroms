/*
 * Copyright (c) 2007 - 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDE_NVBOOT_I2C_H
#define INCLUDE_NVBOOT_I2C_H

#include "nvcommon.h"

#include "nvboot_error.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "project.h"

#define I2C_BASE_ADDR NV_ADDRESS_MAP_I2C5_BASE

// Timeout of .2ms Seshi V said this time is OK for t210 Cross Domain Load
#define CONFIG_LOAD_TIMEOUT_US 200

#define TRANSACTION_COMPLETE_TIMEOUT_US 1000

typedef struct {
    NvBootResetDeviceId DeviceId;
    NvBootClocksClockId ClockId;
} NvBootI2cCntlrTbl;

typedef struct {
    NvU32 CntlrId;
    NvBool Is32BitOp;
    NvU16 Slv1Addr;
    NvU32 Data1;
} NvBootI2CContext;

void NvBootI2cInit(NvBootI2cCntlrTbl i2cCntlrTbl);

NvBootError NvBootI2cWrite(NvBootI2CContext *pI2CContext);

void NvBootI2cBusClear(NvU32 I2cBaseAddress, NvU32 USDelayBeforeBusClear);

#endif
