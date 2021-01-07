/*
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_usb_charging_local.h - Internal definitions for using Usb charging code
 */

#ifndef INCLUDED_NVBOOT_USB_CHARGING_LOCAL_H
#define INCLUDED_NVBOOT_USB_CHARGING_LOCAL_H

#include "nvcommon.h"
#include "argpio.h"
#include "arapb_misc.h"

#if defined(__cplusplus)
extern "C"
{
#endif
//---------------------Macro Declarations---------------------------------
#define SET_PIN_VAL(pin, f, v)                          \
    do {                                            \
       NvU32 RegData;                                 \
       RegData = NV_READ32(NV_ADDRESS_MAP_MISC_BASE + PINMUX_AUX_##pin##_0); \
       RegData = NV_FLD_SET_DRF_DEF(PINMUX_AUX, pin, f, v, RegData);         \
       NV_WRITE32(NV_ADDRESS_MAP_MISC_BASE + PINMUX_AUX_##pin##_0, RegData); \
    } while (0);

#define GET_PIN_VAL(pin, f, v)                  \
    do {                                        \
         v = NV_READ32(NV_ADDRESS_MAP_MISC_BASE + PINMUX_AUX_##pin##_0);    \
    } while (0);
  

#define GPIO_SET_VAL(reg, cntrlNo, port, bit, v)                      \
    do {        \
        NvU32 RegData;                \
        NvU32 cntrllerBase = NV_ADDRESS_MAP_GPIO1_BASE + ((cntrlNo - 1) * NV_ADDRESS_MAP_PPSB_GPIO1_SIZE);     \
        NvU32 portRegOffset = port * sizeof(NvU32); \
        RegData = NV_READ32(cntrllerBase + portRegOffset + GPIO_MSK_##reg##_0);    \
        RegData |= 1 <<  (GPIO_MSK_##reg##_0_MSK0_SHIFT + bit);    \
        RegData |= (v << bit);    \
        NV_WRITE32(cntrllerBase + portRegOffset + GPIO_MSK_##reg##_0, RegData);      \
    } while (0);

#define GPIO_READ_VAL(cntrlNo, port, bit, v)             \
    do {                                                   \
        NvU32 RegData;    \
        NvU32 cntrllerBase = NV_ADDRESS_MAP_GPIO1_BASE + ((cntrlNo - 1) * NV_ADDRESS_MAP_PPSB_GPIO1_SIZE);     \
        NvU32 portRegOffset = port * sizeof(NvU32); \
        RegData = NV_READ32(cntrllerBase + portRegOffset + GPIO_IN_0);    \
        v = (RegData >> (GPIO_IN_0_BIT_0_SHIFT + bit)) & 1;       \
    } while (0);
	
#define USBCHARGING_BIT_INFO(f,v) \
    do { \
      if (v) \
      { \
          *s_pUsbChargingBitInfo |= (1 << f); \
      } \
      else \
      { \
          *s_pUsbChargingBitInfo &= ~(1 << f); \
      } \
    } while(0);

#define GET_USBCHARGING_BIT_INFO(f,v) \
    do { \
        v = ((*s_pUsbChargingBitInfo & (1 << f)) >> f); \
    } while(0);

//---------------------Enumerations--------------------------------------

typedef enum{
    NvBootUsbChargingGpio_LowBatt = 0,

    NvBootUsbChargingGpio_HiCurrent,

    NvBootUsbChargingGpio_Force32 = 0x7fffffff
} NvBootUsbChargingGpio;

//---------------------Structures--------------------------------------
/* Information on Gpios used for usb charger detection and
 * for usb charging initiation through PMIC
 */
typedef struct {
    NvU8 Controller;

    NvU8 Port;

    NvU8 Pin;

} UsbChargingGpioInfo;

/*
 * Information on Wait/Timeout variables 
 * All wait/timeout values to be specified in unit of microseconds.
 */
typedef struct {

    /* Time to wait for after USB_HICURRENT# is asserted */
    NvU32 WaitAfterHiCurrentAssert;

    /* Time to wait before reading low battery status */
    NvU32 WaitGranularityForLowBattWait;

} UsbChargingTimeoutInfo;

/*
 * Context for UsbCharging. 
 */
typedef struct {
    NvBool IsBatteryLow;

    NvBool ChargerDetectionEnabled;

} NvBootUsbChargingContext;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_USB_CHARGING_LOCAL_H */
