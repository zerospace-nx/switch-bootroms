/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef _NVBOOT_LOWBATTERY_H
#define _NVBOOT_LOWBATTERY_H

// Register GPIO_Y_DEBOUNCE_THRESHOLD_02_0
#define GPIO_Y_DEBOUNCE_THRESHOLD_02_0                  _MK_ADDR_CONST(0x11644)

// Register GPIO_Y_ENABLE_CONFIG_02_0
#define GPIO_Y_ENABLE_CONFIG_02_0                       _MK_ADDR_CONST(0x11640)

#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_SHIFT                     _MK_SHIFT_CONST(0)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_FIELD                     _MK_FIELD_CONST(0x1, GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_SHIFT)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_RANGE                     0:0
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_WOFFSET                   0x0
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_DEFAULT                   _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_DEFAULT_MASK                      _MK_MASK_CONST(0x1)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_SW_DEFAULT                        _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_SW_DEFAULT_MASK                   _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_DISABLE                   _MK_ENUM_CONST(0)
#define GPIO_Y_ENABLE_CONFIG_02_0_GPIO_ENABLE_ENABLE                    _MK_ENUM_CONST(1)

#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_SHIFT                       _MK_SHIFT_CONST(5)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_FIELD                       _MK_FIELD_CONST(0x1, GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_SHIFT)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_RANGE                       5:5
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_WOFFSET                     0x0
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_DEFAULT                     _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_DEFAULT_MASK                        _MK_MASK_CONST(0x1)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_SW_DEFAULT                  _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_SW_DEFAULT_MASK                     _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_DISABLE                     _MK_ENUM_CONST(0)
#define GPIO_Y_ENABLE_CONFIG_02_0_DEBOUNCE_FUNCTION_ENABLE                      _MK_ENUM_CONST(1)

#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_SHIFT                  _MK_SHIFT_CONST(1)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_FIELD                  _MK_FIELD_CONST(0x1, GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_SHIFT)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_RANGE                  1:1
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_WOFFSET                        0x0
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_DEFAULT                        _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_DEFAULT_MASK                   _MK_MASK_CONST(0x1)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_SW_DEFAULT                     _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_SW_DEFAULT_MASK                        _MK_MASK_CONST(0x0)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_IN                     _MK_ENUM_CONST(0)
#define GPIO_Y_ENABLE_CONFIG_02_0_IN_OUT_OUT                    _MK_ENUM_CONST(1)

// Register GPIO_Y_DEBOUNCE_THRESHOLD_02_0
#define GPIO_Y_DEBOUNCE_THRESHOLD_02_0                  _MK_ADDR_CONST(0x11644)

// Register GPIO_Y_INPUT_02_0
#define GPIO_Y_INPUT_02_0                       _MK_ADDR_CONST(0x11648)

#endif //_NVBOOT_LOWBATTERY_H