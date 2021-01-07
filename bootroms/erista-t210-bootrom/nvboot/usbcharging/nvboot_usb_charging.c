/*
 * Copyright (c) 2007 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"

#include "arusb.h"


#include "nvboot_fuse_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_buffers_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include "nvboot_usbf_int.h"
#include "nvboot_log_int.h"
#include "nvboot_usb_charging_local.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_i2c_int.h"
#include "nvboot_usbcharging_int.h"

#include "nvboot_error.h"
#include "nvboot_bit.h"
#include "project.h"



// ---------------------Global Variables Declaration--------------------------
static const UsbChargingGpioInfo s_UsbChargingGpioVal[] =
{
        {
            /* LOW_BATT# (LCD_GPIO2) (= IO3_PV.03) GPIO Controller 6, port B, bit 3   */
            0x06,
            0x01,
            0x03
        },
        {
            /* USB_HIGHCURRENT# (USB_VBUS_EN) (IO3_PCC.05) GPIO controller 8 base, port A, bit 5*/
            0x08,
            0x00,
            0x05
        }
};


static const UsbChargingTimeoutInfo s_UsbChargingTimeoutVal[] =
{
        {
            5000,
            1000000
        }
};

static NvBootUsbChargingContext gs_UsbChargingContext;
// Boot Info table.
extern NvBootInfoTable BootInfoTable;
// Pointer to Snor Bit info.
static NvU32* s_pUsbChargingBitInfo = &BootInfoTable.UsbChargingStatus;

// ---------------------Private Functions Declaration--------------------------


/*
 * Queries the status of battery
 */
NvBool
NvBootUsbChargingGetLowBatteryStatus(void);

/*
 * Set up of pins/gpios for Pmic 
 */ 
void
NvBootUsbChargingConfigPmicPins(void);

/*
 * Sets HICURRENT to Pmic
 */
void
NvBootUsbChargingEnablePmicHiCurrent(NvBool Enable);


// ---------------------Public Functions Definitions---------------------------


void
NvBootUsbChargingInit()
{
    /*
     * This function utilizes usbf code in order to initialize usb controller and also
     * in recovery mode, to enumerate the device @ 500mA in case usb charging is enabled.
     */
    NvU32 RegData = 0;

    gs_UsbChargingContext.IsBatteryLow = NV_FALSE;
    *s_pUsbChargingBitInfo = 0;    /// This is redundant because BIT is zero-initialized.

    NVBOOT_MSG((Q_USBCHARGING_START));

    gs_UsbChargingContext.ChargerDetectionEnabled = NvBootFuseIsUsbChargerDetectionEnabled();

    USBCHARGING_BIT_INFO(UsbChargingBITStat_ChargerDetectionEnabled, gs_UsbChargingContext.ChargerDetectionEnabled);
    if (gs_UsbChargingContext.ChargerDetectionEnabled)
    {
        NVBOOT_MSG((Q_USBCHARGING_ENABLED));

        /*
         * Due to USB ECN which allows the usb2.0 device to draw 500mA 
         * when it's in connected and unconfigured state, BootRom need not
         * attempt charger type detection and enumerate the device. 
         * BootRom can enjoy Margheritas now!!
         * DO ONLY
         * If (usb charging enabled)
         *     Configure LOW_BATT and HICURRENT pins
         *     if (LOW_BATT asserted)
         *         enable VDP_SRC
         *         assert HICURRENT
         *         optionally wait for LOW_BATT to de-assert or wait for 5ms before proceeding.
         *     
         */
        /// Configure GPIOs for LOWBATT and USB_HICURRENT for PMIC
        NvBootUsbChargingConfigPmicPins();

        gs_UsbChargingContext.IsBatteryLow = NvBootUsbChargingGetLowBatteryStatus();
        USBCHARGING_BIT_INFO(UsbChargingBITStat_IsBatteryLowDetected, gs_UsbChargingContext.IsBatteryLow);
        if (gs_UsbChargingContext.IsBatteryLow)
        {
            /// Irrespective of cable connection, proceed to change current limit to 500mA.

            /// Set USB_HICURRENT to 500mA
           NvBootUsbChargingEnablePmicHiCurrent(NV_TRUE);
           USBCHARGING_BIT_INFO(UsbChargingBITStat_PmicHighCurrentAsserted, 1);

           /// Enable V(DP_SRC)
           RegData = NV_READ32(NV_ADDRESS_MAP_USB_BASE + USB1_UTMIP_BAT_CHRG_CFG0_0);
           RegData = NV_DRF_NUM(USB1_UTMIP, BAT_CHRG_CFG0, UTMIP_OP_SRC_EN, 1);
           NV_WRITE32(NV_ADDRESS_MAP_USB_BASE + USB1_UTMIP_BAT_CHRG_CFG0_0, RegData);
           /// PMIC will charge battery indefinitely at 500mA or 100mA else PMIC will 
           /// charge battery for T(SLVD_CON_WKB) = 45 min
           /// and then reduce the charging current to I(SUSP)-- See figure 27 (PMIC Usb
           /// Charging Control Algorithm).

           if (NvBootGetSwCYA() & NVBOOT_SW_CYA_WAIT_ON_LOWBATT_ENABLE)
           {
                NVBOOT_MSG((Q_USBCHARGING_CHARGE_BEFORE_COLDBOOT));
                /// Wait till Low battery status disappears
                /// DANGEROUS LOOP
                do {
                    NvBootUtilWaitUS(s_UsbChargingTimeoutVal[0].WaitGranularityForLowBattWait);
                    gs_UsbChargingContext.IsBatteryLow = NvBootUsbChargingGetLowBatteryStatus();
                } while(gs_UsbChargingContext.IsBatteryLow); 
                /// At this point, we know that we have a good battery
                /// PMIC expected to drop the current to 100mA. BootRom expected to boot
                /// the device or transfer control to recovery mode applet such that 
                /// usb enumeration can be done within T(CON_ISUSP) else device will shutdown(2.5mA)
            }
            else
            {
               /// Wait for 5ms
               NvBootUtilWaitUS(s_UsbChargingTimeoutVal[0].WaitAfterHiCurrentAssert);
            }
        } // End if (gs_UsbChargingContext.IsBatteryLow)
     } //End if (gs_UsbChargingContext.ChargerDetectionEnabled)
     NVBOOT_MSG((Q_USBCHARGING_END));
}

/*
 * Set up of pins/gpios for Pmic 
 *
 * @param None
 *
 * @retval None
 */ 
void
NvBootUsbChargingConfigPmicPins()
{
    //Need to change this code to use configuration created in pinmux tables
    // Set LOW_BATT# = LCD_GPIO1 (IO3_PV.03) GPIO Controller 6, port B, bit 3
    // Set the tristate and Pullup/PullDown to Normal
    // For this pin, E_INPUT is default enabled.
    SET_PIN_VAL(LCD_GPIO1, PARK, NORMAL);
    SET_PIN_VAL(LCD_GPIO1, PUPD, NONE);
    SET_PIN_VAL(LCD_GPIO1, TRISTATE, PASSTHROUGH);
    
    /// GPIO controller 3 base, port B, bit 3
    GPIO_SET_VAL(CNF, s_UsbChargingGpioVal[NvBootUsbChargingGpio_LowBatt].Controller,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_LowBatt].Port,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_LowBatt].Pin, 1);


    // Set USB_HICURRENT# = USB_VBUS_EN1 (IO3_PCC.05) GPIO controller 8 base, port A, bit 5
    // Set the tristate and Pullup/PullDown to Normal
    SET_PIN_VAL(USB_VBUS_EN1, PARK, NORMAL);
    SET_PIN_VAL(USB_VBUS_EN1 , PUPD, NONE);
    SET_PIN_VAL(USB_VBUS_EN1, E_INPUT, DISABLE);
    SET_PIN_VAL(USB_VBUS_EN1 , TRISTATE, PASSTHROUGH);

    /// GPIO controller 8 base, port A, bit 5
    GPIO_SET_VAL(CNF, s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Controller,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Port,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Pin, 1);
    /// Enable Output
    GPIO_SET_VAL(OE, s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Controller,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Port,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Pin, 1);
    NVBOOT_MSG((Q_USBCHARGING_CONFIG_PMIC_PINS));
}


/*
 * Queries the status of battery
 *
 * @param None
 *
 * @retval NV_TRUE if low battery detected else NV_FALSE
 */
NvBool
NvBootUsbChargingGetLowBatteryStatus(void)
{
    NvBool LowBattGpioStatus = NV_FALSE;
    NvU32 IsSignalHigh;
    /// NvU32 RegData;

    /// TODO: LOW_BATT is active low.
    /// T210 customer pinmux sheet: LCD_GPIO1 used for LOW_BATT.
    GPIO_READ_VAL(s_UsbChargingGpioVal[NvBootUsbChargingGpio_LowBatt].Controller,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_LowBatt].Port,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_LowBatt].Pin, IsSignalHigh);
    // LOW_BATT# is Active-low
    if (IsSignalHigh == 0)
    {
    	LowBattGpioStatus = NV_TRUE;
        NVBOOT_MSG((Q_USBCHARGING_LOW_BATT));
    }

    return LowBattGpioStatus;
}

/*
 * Sets HICURRENT to allow device to draw upto 500mA
 *
 * @param None
 *
 * @retval None
 */
void
NvBootUsbChargingEnablePmicHiCurrent(NvBool Enable)
{
    /// USB_HICURRENT is set to high for 500mA current limit
    /// and set to low for 100mA(default) current limit
    /// NvU32 RegData;

    /// USB_HICURRENT is Active-high
    /// T210 customer pinmux sheet: USB_VBUS_EN1 used for USB_HIGHCURRENT.

    GPIO_SET_VAL(OUT, s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Controller,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Port,
                s_UsbChargingGpioVal[NvBootUsbChargingGpio_HiCurrent].Pin, Enable);
    NVBOOT_MSG((Q_USBCHARGING_HICURRENT_SET));
}

