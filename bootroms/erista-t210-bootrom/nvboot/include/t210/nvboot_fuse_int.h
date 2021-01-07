/*
 * Copyright (c) 2006 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_fuse_int.h
 *
 * Fuse interface for NvBoot
 *
 * NvBootFuse is NVIDIA's interface for fuse query and programming.
 *
 * Note that fuses have a value of zero in the unburned state; burned 
 * fuses have a value of one.
 *
 */

#ifndef INCLUDED_NVBOOT_FUSE_INT_H
#define INCLUDED_NVBOOT_FUSE_INT_H

#include "nvcommon.h"
#include "nvboot_error.h"
#include "nvboot_fuse.h"
#include "nvboot_sku_int.h"
#include "nvboot_rcm_port_int.h"

#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_RANGE 5:0
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P0_RANGE 5:0
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P1_RANGE 16:11
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P2_RANGE 22:17
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P3_RANGE 28:23
// This is just the old name for CURR_LEVEL
#define FUSE_USB_CALIB_0_SETUP_RANGE 5:0
#define FUSE_USB_CALIB_0_TERM_RANGE_ADJ_RANGE 10:7
#define FUSE_USB_CALIB_EXT_0_RPD_CTRL_RANGE 4:0

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootFuseOperatingMode -- The chip's current operating mode
 */
typedef enum
{
    NvBootFuseOperatingMode_None = 0,
    NvBootFuseOperatingMode_Preproduction,
    NvBootFuseOperatingMode_FailureAnalysis,
    NvBootFuseOperatingMode_NvProduction,
    NvBootFuseOperatingMode_OdmProductionNonSecure,
    NvBootFuseOperatingMode_OdmProductionSecureSBK,
    NvBootFuseOperatingMode_OdmProductionSecurePKC,
    NvBootFuseOperatingMode_Max, /* Must appear after the last legal item */
    NvBootFuseOperatingMode_Force32 = 0x7fffffff
} NvBootFuseOperatingMode;

/**
 * Reports chip's operating mode
 *
 * The Decision Tree is as follows --
 * 1. if Failure Analysis (FA) Fuse burned, then Failure Analysis Mode
 * 2. if ODM Production Fuse burned and ...
 *    a. if SBK Fuses are NOT all zeroes, then ODM Production Mode Secure
 *    b. if SBK Fuses are all zeroes, then ODM Production Mode Non-Secure
 * 3. if NV Production Fuse burned, then NV Production Mode
 * 4. else, Preproduction Mode
 *
 *                                  Fuse Value*
 *                             ----------------------
 * Operating Mode              FA    NV    ODM    SBK
 * --------------              --    --    ---    ---
 * Failure Analysis            1     x     x      x
 * ODM Production Secure       0     x     1      <>0
 * ODM Production Non-Secure   0     x     1      ==0
 * NV Production               0     1     0      x
 * Preproduction               0     0     0      x
 *
 * * where 1 = burned, 0 = unburned, x = don't care
 *
 * @param pMode pointer to buffer where operating mode is to be stored
 *
 * @return void, table is complete decode and so cannot fail
 */
void
NvBootFuseGetOperatingMode(NvBootFuseOperatingMode *pMode);
    
/**
 * Reports whether chip is in Failure Analysis (FA) Mode
 *
 * Failure Analysis Mode means all of the following are true --
 * 1. Failure Analysis Fuse is burned
 *
 * @return NvTrue if chip is in Failure Analysis Mode; else NvFalse
 */
NvBool
NvBootFuseIsFailureAnalysisMode(void);
    
/**
 * Reports whether chip is in ODM Production Mode
 *
 * Production Mode means all of the following are true --
 * 1. FA Fuse is not burned
 * 2. ODM Production Fuse is burned
 *
 * @return NvTrue if chip is in ODM Production Mode; else NvFalse
 */
NvBool
NvBootFuseIsOdmProductionMode(void);

/**
 * Reports whether chip is in ODM Production Mode Secure
 *
 * Production Mode means all of the following are true --
 * 1. chip is in ODM Production Mode
 * 2. Secure Boot Key is not all zeroes
 *
 * @return NvTrue if chip is in ODM Production Mode Secure; else NvFalse
 */
NvBool
NvBootFuseIsOdmProductionModeSecure(void);

/**
 * Reports whether chip is in ODM Production Mode Non-Secure
 *
 * Production Mode means all of the following are true --
 * 1. chip is in ODM Production Mode
 * 2. Secure Boot Key is all zeroes
 *
 * @return NvTrue if chip is in ODM Production Mode Non-Secure; else NvFalse
 */
NvBool
NvBootFuseIsOdmProductionModeNonsecure(void);

/**
 * Reports whether chip is in NV Production Mode
 *
 * Production Mode means all of the following are true --
 * 1. FA Fuse is not burned
 * 2. NV Production Fuse is burned 
 * 3. ODM Production Fuse is not burned
 *
 * @return NvTrue if chip is in NV Production Mode; else NvFalse
 */
NvBool
NvBootFuseIsNvProductionMode(void);

/**
 * Reports whether chip is in Preproduction Mode
 *
 * Pre-production Mode means all of the following are true --
 * 1. FA Fuse is not burned
 * 2. ODM Production Fuse is not burned
 * 3. NV Production Fuse is not burned 
 *
 * @return NvTrue if chip is in Preproduction Mode; else NvFalse
 */
NvBool
NvBootFuseIsPreproductionMode(void);
    
/**
 * Reports whether chip is in PKC secure boot mode or 
 * symmetric key based secure boot mode. 
 * If burned, pkc_disable disables the use of PKC secure boot, and 
 * the BR reverts back to symmetric key based secure boot infrastructure. 
 * Note: The pkc_disable fuse has no effect if the security_mode fuse
 * (i.e. ODM production mode fuse) has not been burned. In this case
 * the pkc_disable fuse is always 0. 
 *
 * @return NvTrue if pkc_disable bit is 0; else NvFalse
 */
NvBool
NvBootFuseIsPkcBootMode(void);

/**
 * Reads Secure Boot Key (SBK) from fuses
 *
 * User must guarantee that the buffer can hold 16 bytes
 *
 * @param pKey pointer to buffer where SBK value will be placed
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetSecureBootKey(NvU8 *pKey);
    
/**
 * Reads Device Key (DK) from fuses
 *
 * User must guarantee that the buffer can hold 4 bytes
 *
 * @param pKey pointer to buffer where DK value will be placed
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetDeviceKey(NvU8 *pKey);

/**
 * Hide SBK and DK
 *
 */
void
NvBootFuseHideKeys(void);

/**
 * Reads the SHA-256 Hash of the public key modulus 
 * from fuses. 
 *
 * User must guarantee that the buffer can hold 32 bytes
 *
 * @param pKey pointer to buffer where public key modulus will be placed
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetPublicKeyHash(NvU8 *pKey);
    
/**
 * Reads Chip's Unique 64-bit Id (UID) from fuses
 *
 * @param pId pointer to NvU64 where chip's Unique Id number is to be stored
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetUniqueId(NvBootECID *pId);

/**
 * Get the SkipDevSelStraps value from fuses
 *
 * @return The SKipDevSelStraps() value.
 */
NvBool
NvBootFuseSkipDevSelStraps(void);

/**
 * Get the EnableChargerDetect value from fuses
 *
 * @return The EnableChargerDetect value.
 */
NvBool
NvBootFuseIsUsbChargerDetectionEnabled(void);

/**
 * Get Boot Device Id from fuses
 *
 * @param pDev pointer to buffer where ID of boot device is to be stored
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetBootDevice(NvBootFuseBootDevice *pDev);

/**
 * Get Boot Device Configuration Index from fuses
 *
 * @param pConfigIndex pointer to NvU32 where boot device configuration 
 *        index is to be stored
 *
 * @return void ((assume trusted caller, no parameter validation) 
 */
void
NvBootFuseGetBootDeviceConfiguration(NvU32 *pConfigIndex);

/**
 * Get 2 Button RCM field from fuses
 *
 * @param pSwReserved pointer to NvU32 where SwReserved field to be stored 
 *        note this is only 8 bits at this time, but could grow
 */
void
NvBootFuseGet2ButtonRcm(NvU32 *p2ButtonRcm); 

/**
 * Get Marketing-defined SKU ID information from fuses
 *
 * SKU ID is a subset of the full SKU field
 *
 * @param pSkuId pointer to SkuId variable
 *
 * @return void ((assume trusted caller, no parameter validation) 
 */
void
NvBootFuseGetSku(NvBootSku_SkuId *pSkuId); 

/**
 * Get SKU field from fuses
 *
 * @param pSku pointer to NvU32 variable
 *
 * @return void ((assume trusted caller, no parameter validation) 
 */
void
NvBootFuseGetSkuRaw(NvU32 *pSku); 

/**
 * Get SATA calibration field from fuses
 *
 * @return void ((assume trusted caller, no parameter validation) 
 */
void
NvBootFuseGetSataCalib(NvU32 *pSataCalib);

/**
 * Get value of SATA OOB RX Calibration Disable fuse
 * @return NV_TRUE if SATA OOB RX Calibration in hardware is disabled
 * @return NV_FALSE if SATA OOB RX Calibration in hardware is enabled
 */
NvBool
NvBootFuseIsSataOobRxCalibDisabled(void);

/**
 * Alias the fuses from the PMC scratch registers.
 */
void
NvBootFuseAliasFuses(void);
/**
 * Get Watchdog enable info from fuse
 */
NvBool
NvBootFuseIsWatchdogEnabled(void);

/**
 * Retrieve port used for RCM.
 */
void
NvBootFuseGetRcmPort(NvBootRCMPortID_T *pRCMPortId);

/**
 * Get the value of the FUSE_SECURE_PROVISON_INDEX_0.
 *
 * @return NvU8 value of fuse.
 *
 */
NvU32
NvBootFuseGetSecureProvisionIndex(void);

/**
 * Get the value of the FUSE_SECURE_PROVISON_INFO_0.
 *
 * @return NvU8 value of fuse.
 *
 */
NvU32
NvBootFuseGetSecureProvisionInfo(void);

/**
 * Reports whether chip has Key Hide fuse burned.
 *
 * @return NvTrue if chip has Key Hide fuse burned; else NvFalse
 */
NvBool
NvBootFuseIsSecureProvisionKeyHideFuseBurned(void);

/**
 * Reports whether chip has Test Part fuse burned.
 *
 * @return NvTrue if chip has Test Part fuse burned; else NvFalse
 */
NvBool
NvBootFuseIsSecureProvisionTestPartFuseBurned(void);

/*
 * Check if the "Anti-cloning" / secure provisioning key index fuses are
 * burned and validate the value read back.
 *
 * Note: This function will validate the value returned from fuses. Only
 *       fuse values between NvBootSeAesSecProvisioningKey_AntiCloningKeyStart
 *       and NvBootSeAesSecProvisioningKey_AntiCloningKeyEnd are
 *       allowed.
 *
 * @return NvBootError_SecProvisioningAntiCloningKeyDisabled
 *         If a secure provisioning key index of 0 is burned.
 * @return NvBootError_ValidAntiCloningFuse
 *         If a valid secure provisioning index fuse is
 *         burned.
 * @return NvBootError_SecProvisioningInvalidAntiCloningKey
 *         If an invalid secure provisioning index fuse is
 *         burned.
 */
NvBootError
NvBootFuseGetSecureProvisioningIndexValidity(void);

/**
 * Check if the conditions have been met for Boot ROM to be in
 * secure provisioning mode:
 * 1. Nv Production mode only
 * 2. a. Anti Cloning key (i.e. secure_provision_index fuse) burned
 * 2. b. Or, Bct value of provisioning key is non-zero
 *
 * The function will ignore key numbers creater than the maximum
 * number allowed (i.e. > NvBootSeAesSecProvisioningKey_Num).
 *
 * See section 10.4 of Secure Boot ISS for truth table of BR handling entitled
 * "Factory Secure Provosioning Use Case Handling"
 *
 * @param SecProvisioningKeyNum The input is the secure provisioning key number
 *                              from the BCT.
 * @return NvBootError_SecProvisioningEnabled if in secure provisioning mode.
 *         NvBootError_SecProvisioningDisabled if not in secure provisioning mode.
 *         NvBootError_SecProvisioningInvalidKeyInput if the inputted key is out of bounds
 *         or invalid.
 */
NvBootError
NvBootFuseIsSecureProvisioningMode(NvU32 SecProvisioningKeyNum);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_FUSE_INT_H
