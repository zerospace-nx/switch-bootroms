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

#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_fuse.h"
#include "nvboot_sku_int.h"
#include "nvboot_rcm_port_int.h"
#include "nvboot_section_defs.h"

#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_RANGE 5:0
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P0_RANGE 5:0
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P1_RANGE 16:11
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P2_RANGE 22:17
#define FUSE_USB_CALIB_0_HS_CURR_LEVEL_P3_RANGE 28:23
#define FUSE_USB_CALIB_0_HS_SQUELCH_LEVEL_RANGE 31:29
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

    // AES-CMAC authentication, No AES encryption
    NvBootFuseOperatingMode_OdmProductionSecureAesCMACAuth_NoEncryption,
    // RSASSA-PSS authentication, No AES encryption
    NvBootFuseOperatingMode_OdmProductionSecureRSAAuth_NoEncryption,
    // ECDSA authentication, No AES encryption
    NvBootFuseOperatingMode_OdmProductionSecureEccAuth_NoEncryption,
    // AES-CMAC authentication, AES encryption
    NvBootFuseOperatingMode_OdmProductionSecureAesCMACAuth_AesEncryption,
    // RSASSA-PSS authentication, AES encryption
    NvBootFuseOperatingMode_OdmProductionSecureRSAAuth_AesEncryption,
    // ECDSA authentication, AES encryption
    NvBootFuseOperatingMode_OdmProductionSecureEccAuth_AesEncryption,

    NvBootFuseOperatingMode_OdmProductionSecureSBK, // Deprecated, for removal
    NvBootFuseOperatingMode_OdmProductionSecurePKC, // Deprecated, for removal
    NvBootFuseOperatingMode_Max, /* Must appear after the last legal item */
    NvBootFuseOperatingMode_Force32 = 0x7fffffff
} NvBootFuseOperatingMode;

/**
 * Reports whether chip is in Failure Analysis (FA) Mode
 *
 * Failure Analysis Mode means all of the following are true --
 * 1. Failure Analysis Fuse is burned
 *
 * @return NvTrue if chip is in Failure Analysis Mode; else NvFalse
 */
NvBool FT_NONSECURE 
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
NvBool FT_NONSECURE
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
NvBool FT_NONSECURE
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
 * Reports the fuse state of NV_PRODUCTION
 *
 * DOES NOT CHECK FOR TEST FUSE WHAT SO EVER
 *
 * @return NvTrue if chip is has NV_PRODUCTION fuse burnt; else NvFalse
 */
NvBool
NvBootFuseNvProductionFuseCheck(void);

/**
 * Reports whether chip has OEM Fuse Encryption Enabled
 *
 * @return NvTrue if chip OEM fuse encryption is enabled, else NvFlase
 */
NvBool
NvBootFuseIsOemFuseEncryptionEnabled(void);

/*
 * Get the fuse decryption key selection
 *
 * @return NvU32 number representation of the key selection
 */
NvU32
NvBootFuseGetFuseDecrypitonKeySelection(void);

/**
 * Reports whether or not SE atomic context save is enabled.
 *
 * @return NV_TRUE if SE aomtic context save is enabled.
 */
NvBool
NvBootFuseIsSeContextAtomicSaveEnabled(void);

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
NvBool FT_NONSECURE
NvBootFuseIsPreproductionMode(void);

/**
 * Reports whether chip is in PKC (asymmetric key) secure boot mode or
 * symmetric key based secure boot mode.
 * If the ECC or RSA authentication bits are burned in BOOT_SECURITY_INFO
 * this function returns NV_TRUE.
 *
 * @return NV_TRUE if ECC or RSA authentication bits are set; else NvFalse
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
NvBootFuseGetSecureBootKey(uint8_t *pKey);
    
/**
 * Reads Device Key (DK) from fuses
 *
 * User must guarantee that the buffer can hold 16 bytes
 *
 * @param pKey pointer to buffer where DK value will be placed
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetDeviceKey(uint8_t *pKey);

/**
 * Hide SBK and DK
 *
 */
void
NvBootFuseHideKeys(void);

/**
 * Reads the SHA-256 Hash of the Public Crypto Parameters.
 *
 * User must guarantee that the buffer can hold 32 bytes
 *
 * @param pKey pointer to buffer where public key modulus will be placed
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void
NvBootFuseGetPcpHash(NvU8 *pKey);

/**
 * Reads Chip's Unique 64-bit Id (UID) from fuses
 *
 * @param pId pointer to NvU64 where chip's Unique Id number is to be stored
 *
 * @return void (assume trusted caller, no parameter validation)
 */
void FT_NONSECURE
NvBootFuseGetUniqueId(NvBootECID *pId);

/**
  Encode additional info into ECID to send during initial RCM
  handshake.
    [103:100]
    T214: APB_MISC_GP_HIDREV_0_MAJORREV
    T194: MISCREG_HIDREV_0_MAJORREV
    [111:104]
    T214: APB_MISC_GP_HIDREV_0_CHIPID
    T194: MISCREG_HIDREV_0_CHIPID
    [115:112]
    T214: APB_MISC_GP_MINORREV
    T194: MISCREG_HIDREV_0_MINORREV
    [122:116]
    Reserved
    [123]
    FUSE_RESERVED_PRODUCTION_0[3]
    [126:124]
    BOOT_SECURITY_INFO[2:0]
    [127]
    FUSE_PRODUCTION_MODE
*/
void NvBootFuseAddAdditionalEcidInfo(NvBootECID *pEcid);

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
void FT_NONSECURE
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
 * Get skip Delay sequence info from fuse
 */
void
NvBootFuseSkipDelaySeq(NvU32 *pSkipSequence);

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
NvBool FT_NONSECURE
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

/**
 * Get the value of Authentication scheme from the FUSE_BOOT_SECURITY_INFO_0.
 *
 * @return NvU32 value of fuse.
 *
 */
void FT_NONSECURE
NvBootFuseGetBootSecurityAuthenticationInfo(NvU32 *pSecureIndex);


/**
 * Reports whether Boot security info encryption fuse burned.
 *
 * @return NvTrue if chip has encryption fuse burned; else NvFalse
 */
NvBool FT_NONSECURE
NvBootFuseBootSecurityIsEncryptionEnabled();

/**
 * Reports whether the NV Production Mode fuse is set (burned)
 *
 * Note that this fuse by itself does not determine whether the chip is in
 * NV Production Mode.
 *
 * If the SECURE_PROVISION_INDEX_0[1] is set (i.e. TEST_PART), we always return
 * NV_TRUE.
 * If the TEST_PART fuse is burned, the chip can never
 * be an ODM production part. Always force Nv_Production mode.
 * We can also use this TEST_PART fuse to simulate Nv Production
 * mode in an unfused chip (for debugging BR with JTAG), as well
 * as debug the Secure Provisioning feature.
 * TEST_PART cannot be burned after SECURITY_MODE has been burned.
 *
 * @param none
 *
 * @return NV_TRUE if TEST_PART fuse is set.
 *         NV_FALSE if Nv production Mode fuse is unset (un-burned).
 *         NV_TRUE if Nv production Mode fuse is set (burned) and TEST_PART fuse unset.
 */
NvBool
NvBootFuseIsNvProductionModeFuseSet(void);

/**
 *  Select Bank for FEK fuses. There are 2 banks.
 *  @return 0: Bank0, 1:Bank1
 */
NvU32
NvBootFuseGetOemFekBankSelect();

/**
 *  Determine if OEM fuses are encrypted.
 *  @return 0:False 1:True
 */
NvBool
NvBootFuseIsOemFuseEncryptionEnabled(void);

/**
 *  Get OEM FEK Selection
 */
NvU32 NvBootFuseGetFuseDecryptionKeySelection();

/**
 *  Check FUSE_JTAG_SECUREID_VALID_0. This is a dependency for SE RNG.
 */
NvBool NvBootFuseIsJtagSecureIdFuseSet(void);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_FUSE_INT_H
