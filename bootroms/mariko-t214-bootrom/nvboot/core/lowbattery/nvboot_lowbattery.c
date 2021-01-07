/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#include "nvboot_lowbattery.h"
#include <arpadctl_UART.h>
#include <address_map_new.h>
#include "nvboot_util_int.h"
#include "nvrm_drf.h"
#include "nvtypes.h"
#include <nvboot_lowbattery_int.h>
#include <nvboot_reset_int.h>
#include <nvboot_clocks_int.h>
#include <nvboot_hardware_access_int.h>
#include <nvboot_fuse_int.h>
#include <nvboot_bit.h>

extern NvBootInfoTable  BootInfoTable;

void FT_NONSECURE NvBootLowBatteryCheck()
{
    NvU32 RegData, BatteryCharged = 0;

    if(NvBootFuseIsDeadBatteryCheckEnabled())
    {

        // Log Bit
        BootInfoTable.BootROMtracker = NvBootFlowStatus_LowBat_NotCharged;

        // Enable clocks. Default CLK_M
        NvBootClocksSetEnable(NvBootClocksClockId_GpioCtl1Id, NV_TRUE);
        // de-assert reset.
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_RST_DEV_GPIO_CTL1_0);
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                     RST_DEV_GPIO_CTL1,
                                     SWR_GPIO_CTL1_RST,
                                     DISABLE,
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                   CLK_RST_CONTROLLER_RST_DEV_GPIO_CTL1_0,RegData);

        // http://nvbugs/1590997/14
        // a. Set GPIO_Y_DEBOUNCE_THRESHOLD_02_0 with 0x01 corresponding to the debouncing threshold of 1 msec to 2 msec to ensure transients are not sampled.
        NV_WRITE32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_DEBOUNCE_THRESHOLD_02_0, 0x1);

        // Bug 200106546: Enable debounce feature
        RegData = NV_READ32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_ENABLE_CONFIG_02_0);
        RegData = NV_FLD_SET_DRF_NUM(GPIO_Y, ENABLE_CONFIG_02, DEBOUNCE_FUNCTION, 1, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_ENABLE_CONFIG_02_0, RegData);

        // b. Configure the GPIO input path related configuration
         // i)Set GPIO_Y_ENABLE_CONFIG_02_0.INOUT to IN
        RegData = NV_READ32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_ENABLE_CONFIG_02_0);
        RegData = NV_FLD_SET_DRF_DEF(GPIO_Y, ENABLE_CONFIG_02, IN_OUT, IN, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_ENABLE_CONFIG_02_0, RegData);

        // c. Set the pad to ensure  source is GPIO i.e. .
       // PADCTL_UART_GPIO_MDM3_0 .GPIO_SFIO_SEL field to GPIO.
        RegData = NV_READ32(NV_ADDRESS_MAP_PADCTL_A13_BASE + PADCTL_UART_GPIO_MDM3_0);
        RegData = NV_FLD_SET_DRF_DEF(PADCTL_UART, GPIO_MDM3, GPIO_SF_SEL, GPIO, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A13_BASE + PADCTL_UART_GPIO_MDM3_0, RegData);

        // d. Set PADOUPUTSTATE i.e. PADCTL_UART_GPIO_MDM3_0.TRISTATE is configured to TRISSTATE/FLOATED state to enable functional logic driving/sampling the I/Os
        RegData = NV_READ32(NV_ADDRESS_MAP_PADCTL_A13_BASE + PADCTL_UART_GPIO_MDM3_0);
        RegData = NV_FLD_SET_DRF_DEF(PADCTL_UART, GPIO_MDM3, TRISTATE, TRISTATE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A13_BASE + PADCTL_UART_GPIO_MDM3_0, RegData);

        // e. Set E_INPUT pins of the pads i.e. PADCTL_UART_GPIO_MDM3_0.E_INPUT as ENABLE to have it enabled for input.
        RegData = NV_READ32(NV_ADDRESS_MAP_PADCTL_A13_BASE + PADCTL_UART_GPIO_MDM3_0);
        RegData = NV_FLD_SET_DRF_DEF(PADCTL_UART, GPIO_MDM3, E_INPUT, ENABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A13_BASE + PADCTL_UART_GPIO_MDM3_0, RegData);

        // f. Set GPIO_Y_ENABLE_CONFIG_02.GPIO_ENABLE to ENABLE so that GPIO configuration is fully done
        RegData = NV_READ32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_ENABLE_CONFIG_02_0);
        RegData = NV_FLD_SET_DRF_DEF(GPIO_Y, ENABLE_CONFIG_02, GPIO_ENABLE, ENABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_ENABLE_CONFIG_02_0, RegData);

        // g. Boot ROM can start Polling/Reading GPIO_Y_INPUT_02_0 to get the GPIO value
        while(!BatteryCharged)
        {
            BatteryCharged  = NV_READ32(NV_ADDRESS_MAP_GPIO_CTL_BASE + GPIO_Y_INPUT_02_0);
            NvBootUtilWaitUS(1);
        }
        // Log Bit
        BootInfoTable.BootROMtracker = NvBootFlowStatus_LowBat_Charged;
    }
}
