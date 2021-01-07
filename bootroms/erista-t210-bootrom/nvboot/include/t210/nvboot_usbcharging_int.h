/*
 * Copyright (c) 2006 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_usbcharging_int.h
 *
 * NvBootUsbCharging interface for NvBOOT
 *
 * NvBootUsbCharging is NVIDIA's interface for detection of usb charger and
 * enabling charging.
 *
 */

#ifndef INCLUDED_NVBOOT_USB_CHARGING_INT_H
#define INCLUDED_NVBOOT_USB_CHARGING_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"


#if defined(__cplusplus)
extern "C"
{
#endif

/// Specifies BIT UsbCharging Status field shift values
typedef enum {
    /// [0:0] = ChargerDetectionEnabled (0 if disabled, 1 if enabled)
    UsbChargingBITStat_ChargerDetectionEnabled = 0,

    /// [1:1] = IsBatteryLowDetected (1 if detected, else 0)
    UsbChargingBITStat_IsBatteryLowDetected,

    /// [2:2] = IsUsbCableConnected (1 if usb cable is connected, else 0)
    UsbChargingBITStat_IsUsbCableConnected,

    /// [3:3] = RESERVED. Was IsChargingPort (1 if charging port, else 0 if SDP)
    UsbChargingBITStat_IsChargingPort,

    /// [4:4] = RESERVED. Was IsDividerChargerDetected (1 if divider charger detected, else 0)
    UsbChargingBITStat_IsDividerChargerDetected,

    /// [5:5] = RESERVED. Was IsACADetected (1 if ACA A/B/C detected, else 0)
    UsbChargingBITStat_IsACADetected,

    /// [6:6] = RESERVED. Was IsNonCompliantChargerDetected (1 if charger is non-compliant, else 0)
    UsbChargingBITStat_IsNonCompliantChargerDetected,

    /// [7:7] = RESERVED. Was IsDcpCdpDetected (1 if DCP/CDP, else 0)
    UsbChargingBITStat_IsDcpCdpDetected,

    /// [8:8] = PmicHighCurrentStatusAsserted (1 if driven high, 0 if low)
    UsbChargingBITStat_PmicHighCurrentAsserted,

    /// [9:9] = RESERVED. Was DeviceEnumerationComplete (1 if device enumeration performed)
    UsbChargingBITStat_DeviceEnumerationComplete,

    /// [10:10] = RESERVED. Was DeviceEnumerationFailed (1 if device enum failed, else 0)
    UsbChargingBITStat_DeviceEnumerationFailed,

    /// [11:11] = RESERVED. Was ReadPMICUsbChargerControlForHiCurrentFailed (set to 1 if read/write failed, else 0)
    UsbChargingBITStat_ReadPMICUsbChargerControlForHiCurrentFailed,

    /// [12:12] = RESERVED. Was UsbChargingBITStat_ReadPMICUsbChargerControlForUsbSuspendFailed (set to 1 if 
    ///        read/write failed, else 0)
    UsbChargingBITStat_ReadPMICUsbChargerControlForUsbSuspendFailed,

    /// [13:13] = RESERVED. Was UsbChargingBITStat_Timer0Disabled (set to 1 if Timer0 disabled, else 0)
    UsbChargingBITStat_Timer0Disabled,

    UsbChargingBITStat_Force32 = 0x7fffffff
} UsbChargingBITStat;

#define CHECK_DEVICE_ENUMERATION_STATUS(s, f)    s & (1 << f)

/**
 * Detects a USB charger and enables charging
 *
 * @param None
 *
 * @return None
 */
void
NvBootUsbChargingInit(void);


#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_USB_CHARGING_INT_H
