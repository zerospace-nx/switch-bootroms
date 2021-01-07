/*
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_i2c_int.h
 *
 * NvBootI2c interface for NvBOOT
 *
 * NvBootI2c is NVIDIA's interface for communicating with a slave device over
 * i2c interface.
 *
 *
 */

#ifndef INCLUDED_NVBOOT_I2C_INT_H
#define INCLUDED_NVBOOT_I2C_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

//////////////////////////////////////////////////////////////////////////////
// Defines PMIC register fields used.
#define PMIC_BOOT_SEL_0_BOOT_SEL10_SHIFT    _MK_SHIFT_CONST(0)
#define PMIC_BOOT_SEL_0_BOOT_SEL10_RANGE    1:0
#define PMIC_BOOT_SEL_0_BOOT_SEL10_DEFAULT_MASK    _MK_MASK_CONST(0x3)
#define PMIC_BOOT_SEL_0_BOOT_SEL10_COLDBOOT_EMMC_X8_BOOTMODE_OFF      _MK_ENUM_CONST(3)

#define PMIC_BOOT_SEL_0_FORCE_RCM_SHIFT     _MK_SHIFT_CONST(7)
#define PMIC_BOOT_SEL_0_FORCE_RCM_RANGE     7:7
#define PMIC_BOOT_SEL_0_FORCE_RCM_DEFAULT_MASK    _MK_MASK_CONST(0x1)
#define PMIC_BOOT_SEL_0_FORCE_RCM_ENABLE      _MK_ENUM_CONST(1)

#define PMIC_USB_CHARGER_CONTROL_0_DISABLE_TIMER0_SHIFT     _MK_SHIFT_CONST(7)
#define PMIC_USB_CHARGER_CONTROL_0_DISABLE_TIMER0_RANGE     7:7
#define PMIC_USB_CHARGER_CONTROL_0_DISABLE_TIMER0_DEFAULT_MASK    _MK_MASK_CONST(0x1)
#define PMIC_USB_CHARGER_CONTROL_0_DISABLE_TIMER0_ENABLE      _MK_ENUM_CONST(1)

#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_SHIFT     _MK_SHIFT_CONST(4)
#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_RANGE     6:4
#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_DEFAULT_MASK    _MK_MASK_CONST(0x7)
#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_RID_FLOAT      _MK_ENUM_CONST(0)
#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_RID_A      _MK_ENUM_CONST(1)
#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_RID_B      _MK_ENUM_CONST(2)
#define PMIC_USB_CHARGER_CONTROL_0_USB_ID_STATUS_RID_C      _MK_ENUM_CONST(3)

#define PMIC_USB_CHARGER_CONTROL_0_USB_HICURRENT_SHIFT     _MK_SHIFT_CONST(3)
#define PMIC_USB_CHARGER_CONTROL_0_USB_HICURRENT_RANGE     3:3
#define PMIC_USB_CHARGER_CONTROL_0_USB_HICURRENT_DEFAULT_MASK    _MK_MASK_CONST(0x1)
#define PMIC_USB_CHARGER_CONTROL_0_USB_HICURRENT_SET_100MA_CURRENT_LIMIT      _MK_ENUM_CONST(0)
#define PMIC_USB_CHARGER_CONTROL_0_USB_HICURRENT_SET_500MA_CURRENT      _MK_ENUM_CONST(1)

#define PMIC_USB_CHARGER_CONTROL_0_USB_SUSPEND_SHIFT     _MK_SHIFT_CONST(2)
#define PMIC_USB_CHARGER_CONTROL_0_USB_SUSPEND_RANGE     2:2
#define PMIC_USB_CHARGER_CONTROL_0_USB_SUSPEND_DEFAULT_MASK    _MK_MASK_CONST(0x1)
#define PMIC_USB_CHARGER_CONTROL_0_USB_SUSPEND_SET_HICURRENT_LIMIT      _MK_ENUM_CONST(0)
#define PMIC_USB_CHARGER_CONTROL_0_USB_SUSPEND_SET_SUSP_CURRENT      _MK_ENUM_CONST(1)

/// Note: In T148 PMIC, low batt signal is active high unlike T114 PMIC
/// wherein low batt signal was defined as active low.
#define PMIC_USB_CHARGER_CONTROL_0_LOW_BATT_SHIFT     _MK_SHIFT_CONST(0)
#define PMIC_USB_CHARGER_CONTROL_0_LOW_BATT_RANGE     0:0
#define PMIC_USB_CHARGER_CONTROL_0_LOW_BATT_DEFAULT_MASK    _MK_MASK_CONST(0x1)
#define PMIC_USB_CHARGER_CONTROL_0_LOW_BATT_BATT_VOLTAGE_HIGH      _MK_ENUM_CONST(0)
#define PMIC_USB_CHARGER_CONTROL_0_LOW_BATT_BATT_VOLTAGE_LOW      _MK_ENUM_CONST(1)

//////////////////////////////////////////////////////////////////////////////

/**
 * 
 * Pmu Register Id defined here so that Actual Register
 * details are masked from the the calling code.
 * Based on the PMIC vendor spec, registers can change from
 * one chip to another. They are also specific to the PMIC.
 */
typedef enum NvBootPmuRegIdRec{
    NvBootPmuRegId_UsbChargerControl0 = 0,
    NvBootPmuRegId_BootSelect,
    NvBootPmuRegId_RTCDomainStatus1,
    NvBootPmuRegId_RTCDomainStatus2,
    NvBootPmuRegId_Num,
    NvBootPmuRegId_Force32 = 0x7fffffff
} NvBootPmuRegId;

/**
 * 
 * Pmu Register Field Id defined here so that Actual Register
 * Fields are masked from the the calling code.
 * Based on the PMIC vendor spec, registers can change from
 * one chip to another. They are also specific to the PMIC.
 * This is also a provision for the case when register/field
 * offsets within PMIC may accidentally not be the same as the
 * ones defined the PMIC vendor spec.
 */
typedef enum NvBootPmuRegFldIdRec{
    NvBootPmuRegFldId_DisTimer0 = 0,
    NvBootPmuRegFldId_UsbIdStatus,
    NvBootPmuRegFldId_UsbHiCurrent,
    NvBootPmuRegFldId_UsbSuspend,
    NvBootPmuRegFldId_VotgSessVld,
    NvBootPmuRegFldId_LowBatt,

    NvBootPmuRegFldId_ForceRcm,
    NvBootPmuRegFldId_Bctl,
    NvBootPmuRegFldId_BootSel,

    NvBootPmuRegFldId_NoPower,
    NvBootPmuRegFldId_SuOnKey,
    NvBootPmuRegFldId_SuVbusAttach,
    NvBootPmuRegFldId_SuChg,
    NvBootPmuRegFldId_SuRtc,

    NvBootPmuRegFldId_SuLowBatt,
    NvBootPmuRegFldId_VrFault,
    NvBootPmuRegFldId_SwRst,
    NvBootPmuRegFldId_SwShutdown,
    NvBootPmuRegFldId_LpShutdown,
    NvBootPmuRegFldId_HwShutdown,
    NvBootPmuRegFldId_Ovt,
    NvBootPmuRegFldId_SuWdt,

    NvBootPmuRegFldId_Num,
    NvBootPmuRegFldId_Force32 = 0x7fffffff
} NvBootPmuRegFldId;

typedef enum NvBootI2cControllerRec{
    NvBootI2cController_I2c1,
    NvBootI2cController_I2c2,
    NvBootI2cController_I2c3,
    NvBootI2cController_I2c4,
    NvBootI2cController_I2c5,
    NvBootI2cController_I2c6,
    NvBootI2cController_Num,
    NvBootI2cController_Force32 = 0x7fffffff
} NvBootI2cController;

/**
 * Initializes the I2c Controller
 *
 * @param None
 *
 * @return None
 */
void NvBootI2cInitialize(void);


/**
 * Writes a register of an I2C slave
 *
 * @param Id  Id for the register to be written
 * @param data pointer to data which is to be written to regAddr.
 *
 * @return None
 */
NvBootError
NvBootI2cWriteReg(const NvBootPmuRegId Id, const NvU32 *data);
/**
 * Reads a register of an I2C slave
 *
 * @param Id  Id for the register to be read
 * @param data pointer to mem address where data is to be read.
 *
 * @return None
 */
NvBootError
NvBootI2cReadReg(const NvBootPmuRegId Id, NvU32 *data);

/*
 * NvBootI2cReadStrap() reads BOOT_SEL
 * register of PMIC and populates the Boot device selection
 * and force recovery strap values in STRAPPING_OPT_A.
 *
 * @retval none
 */
void
NvBootI2cReadStrap(void);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_AES_INT_H
