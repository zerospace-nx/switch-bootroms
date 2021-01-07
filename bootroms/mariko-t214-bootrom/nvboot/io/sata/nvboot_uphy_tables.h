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

#define PLL_PARAMS_LENGTH 4
#define LANE_PARAMS_LENGTH 13

struct NvUphyCfgReg {
    NvBool en; //Will not write if enable is false
    uint8_t addr;
    uint16_t wdata;
};

const struct NvUphyCfgReg NvUphyClm_SataG1G2G3_init[PLL_PARAMS_LENGTH] = {
    {.en = NV_TRUE, .addr = 0x0,  .wdata = 0x001E},
    {.en = NV_TRUE, .addr = 0x2,  .wdata = 0x0000},
    {.en = NV_TRUE, .addr = 0x3,  .wdata = 0x7001}, 
    {.en = NV_FALSE, .addr = 0,  .wdata = 0}
};

const struct NvUphyCfgReg NvUphyDlm_SataG1G2_init[LANE_PARAMS_LENGTH] = {
    {.en = NV_TRUE, .addr = 0x1,  .wdata = 0x0003},
    {.en = NV_TRUE, .addr = 0x3,  .wdata = 0x0010},
    {.en = NV_TRUE, .addr = 0x4,  .wdata = 0x0030},
    {.en = NV_TRUE, .addr = 0x5,  .wdata = 0x0031},
    {.en = NV_TRUE, .addr = 0x7,  .wdata = 0x0800},
    {.en = NV_TRUE, .addr = 0x8,  .wdata = 0x0811},
    {.en = NV_TRUE, .addr = 0x28, .wdata = 0x0022},
    {.en = NV_TRUE, .addr = 0x29, .wdata = 0x0022},
    {.en = NV_TRUE, .addr = 0x2E, .wdata = 0x0050},
    {.en = NV_TRUE, .addr = 0x2F, .wdata = 0x0050},
    {.en = NV_TRUE, .addr = 0x49, .wdata = 0x0F37},
    {.en = NV_TRUE, .addr = 0x4A, .wdata = 0x0F67}, 
    {.en = NV_FALSE, .addr = 0, .wdata = 0}
};

