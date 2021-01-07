/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_error.h - error codes.
 */

#ifndef INCLUDED_NVBOOT_ERROR_H
#define INCLUDED_NVBOOT_ERROR_H

#if defined(__cplusplus)
extern "C"
{
#endif

#define NVBOOT_ERRORS \
    X(Success/*No Synonyms for Success should be used.*/) \
    X(InvalidParameter) \
    X(IllegalParameter) \
    X(HwTimeOut) \
    X(NotInitialized) \
    X(DeviceNotResponding) \
    X(DataCorrupted) \
    X(DataUnderflow) \
    X(DeviceError) \
    X(DeviceReadError) \
    X(DeviceUnsupported) \
    X(DeviceResponseError) \
    X(Unimplemented) \
    X(ValidationFailure) \
    X(EccDiscoveryFailed) \
    X(EccFailureCorrected) \
    X(EccFailureUncorrected) \
    X(Busy) \
    X(Idle) \
    X(MemoryNotAllocated) \
    X(MemoryNotAligned) \
    X(BctNotFound) \
    X(BootLoaderLoadFailure) \
    X(BctBlockInfoMismatch) \
    X(IdentificationFailed) \
    X(HashMismatch) \
    X(TxferFailed) \
    X(WriteFailed) \
    X(EpNotConfigured) \
    X(WarmBoot0_Failure) \
    X(AccessDenied) \
    X(InvalidOscFrequency) \
    X(PllNotLocked) \
    X(InvalidDevParams) \
    X(InvalidBootDeviceEncoding) \
    X(CableNotConnected) \
    X(InvalidBlDst) \
    X(SE_ModExp_OOR/*Message or Signature representative out of range*/) \
    X(SE_RsaPssVerify_Inconsistent) \
    X(RsaPssVerify_Inconsistent) \
    X(RsaPssVerify_Inconsistent_1/* if emLen < hLen + sLen + 2 */) \
    X(RsaPssVerify_Inconsistent_bc) \
    X(RsaPssVerify_Inconsistent_2) \
    X(RsaPssVerify_Inconsistent_MGF) \
    X(RsaPssVerify_Inconsistent_3) \
    X(RsaPssVerify_Inconsistent_4) \
    X(RsaPssVerify_ShaHash_Error) \
    X(RsaPssVerify_Inconsistent_5) \
    X(FuseHashMismatch/* Mismatch of hash of public key modulus read from secondary storage and fuses*/) \
    X(XusbDeviceNotAttached) \
    X(XusbPortResetFailed) \
    X(XusbInvalidBmRequest) \
    X(XusbParseConfigDescFail) \
    X(XusbMscInvalidCmd) \
    X(XusbCswStatusCmdFail) \
    X(XusbMscResetRecovery) \
    X(XusbEpStalled) \
    X(XusbEpError) \
    X(XusbEpRetry) \
    X(XusbEpNotReady) \
    X(XusbControlSeqNumError) \
    X(XusbControlDirError) \
    X(XusbOutofSync) \
    X(XusbPortError) \
    X(XusbDisconnected) \
    X(XusbReset) \
    X(CryptoMgr_Busy) \
    X(CryptoMgr_VerifyFailure) \
    X(CryptoMgr_InvalidAuthScheme) \
    X(CryptoMgr_InvalidVerifyOp) \
    X(CryptoMgr_InvalidEllipticCurve) \
    X(CryptoMgr_Ecdsa_R_S_Out_Of_Range) \
    X(CryptoMgr_Ecdsa_Invalid_R_is_O/*R = u1G + u2Q = O = point at infinity*/) \
    X(CryptoMgr_Ecdsa_Invalid_Sig) \
    X(CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode/*Pcp not loaded because not PKC or ECC mode*/) \
    X(CryptoMgr_Pcp_Hash_Mismatch/*SHA2 hash of Pcp mismatches the hash in the public key fuses*/) \
    X(CryptoMgr_OemBootBinaryHeader_NotAuthenticated) \
    X(CryptoMgr_BlBinaryPackage_Auth_Error) \
    X(CryptoMgr_Encryption_NotEnabled) \
    X(CryptoMgr_AesCmacHash_Compare_Failure) \
    X(SE_Context_Restore_Failure) \
    X(SecProvisioningBctKeyMismatch/*Indicates a mismatch between SecProvisioningKeyNum_Secure and _Insecure*/) \
    X(SecProvisioningRcmKeyMismatch/*Indicates a mismatch between SecProvisioningKeyNum_Secure and _Insecure*/) \
    X(SecProvisioningDisabled) \
    X(SecProvisioningEnabled) \
    X(SecProvisioningInvalidKeyInput/*Indicates an invalid/out of bounds value in SecProvisioningKeyNum_Insecure*/) \
    X(SecProvisioningInvalidAntiCloningKey) \
    X(SecProvisioningAntiCloningKeyDisabled) \
    X(SecProvisioningInvalidType/*Error returned if type is not coldboot or RCM*/) \
    X(InvalidSeKeySlotNum) \
    X(InvalidSeKeySize) \
    X(Unsupported_SHA_Family/*Could be unsupported by the driver, not the device*/) \
    X(Unsupported_SHA_DigestSize/*Could be unsupported by the driver, not the device*/) \
    X(RSA_SSA_PSS_Signature_Out_Of_Range/* Signature is not in the range 0 to n-1 inclusive*/) \
    X(Unsupported_RSA_Key_Size) \
    X(Invalid_SC7_FW_Save_Address/*SC7 FW can only be stored in A0 TZRAM or DRAM*/) \
    X(Invalid_Bl_Load_Address/*Invalid BL load address*/) \
    X(SC7_FW_Size_Too_Large/*SC7 FW can only be stored in A0 TZRAM or DRAM*/) \
    X(SC7_FW_Size_Too_Small/*SC7 FW size cannot be 0 */) \
    X(SC7_Insecure_To_Secure_Size_Mismatch/*LengthInsecure != LengthSecure */) \
    X(ECID_Mismatch) \
    X(Sc7TzramRestoreTimeout) \
    X(BCTVersionAndOemHeaderVersionMismatch) \
    X(Invalid_Bl_Size_And_Or_Destination/*Invalid load addres and or length*/) \
    X(RcmDebugRcm) \
    X(FakeErrorDoNotUse/*Add new errors before this error.*/) \
    X(Initial_Value=0x7faf5fe5/*Special Error. Use this for items that should default to error, such as authentication*/) \
    \
    X(Force32/*This is no longer needed because of the mode attribute used in the declaration of NvBootError. Kept in to so the code which uses Force32 can still build.*/) \
    X(NumErrors/*Should be last error in case we need to know the number of errors defined.*/)

#define X(a) NvBootError_##a,
typedef enum
{
    NVBOOT_ERRORS
} NvBootError __attribute__((mode(SI))); // mode(SI) Forces 4-byte size (signed int) for NvBootError.
                                         // If not forced, gcc will reduce the enum size to the smallest
                                         // unit that can fit all the enum values (tested via experimentation).
#undef X

/**
 * Notes on the mode attribute:
 *
 * https://gcc.gnu.org/onlinedocs/gcc-3.2/gcc/Variable-Attributes.html
 * mode (mode)
 * This attribute specifies the data type for the declaration--whichever type corresponds to the mode mode.
 * This in effect lets you request an integer or floating point type according to its width.
 *
 * https://gcc.gnu.org/onlinedocs/gccint/Machine-Modes.html
 * and http://www.delorie.com/gnu/docs/gcc/gcc_80.html.
 * SI means "SImode" or "Single Integer". This usually represents four times the size of the smallest addressable unit, byte.
 * gcc uses "BITS_PER_UNIT" to define the byte size.
 */

#if 0
/*
 * NvBootError: Enumerated error codes
 */
typedef enum
{
    NvBootError_None = 0,
    NvBootError_Success = 0,
    NvBootError_SE_Signature_Valid = 0, //TODO: remove this in favor of NvBootError_Success?
    NvBootError_ValidAntiCloningFuse = 0,
    NvBootError_InvalidParameter,
    NvBootError_IllegalParameter,
    NvBootError_HwTimeOut,
    NvBootError_NotInitialized,
    NvBootError_DeviceNotResponding,
    NvBootError_DataCorrupted,
    NvBootError_DataUnderflow,
    NvBootError_DeviceError,
    NvBootError_DeviceReadError,
    NvBootError_DeviceUnsupported,
    NvBootError_DeviceResponseError,
    NvBootError_Unimplemented,
    NvBootError_ValidationFailure,
    NvBootError_EccDiscoveryFailed,
    NvBootError_EccFailureCorrected,
    NvBootError_EccFailureUncorrected,
    NvBootError_Busy,
    NvBootError_Idle,
    NvBootError_MemoryNotAllocated,
    NvBootError_MemoryNotAligned,
    NvBootError_BctNotFound,
    NvBootError_BootLoaderLoadFailure,
    NvBootError_BctBlockInfoMismatch,
    NvBootError_IdentificationFailed,
    NvBootError_HashMismatch,
    NvBootError_TxferFailed,
    NvBootError_WriteFailed,
    NvBootError_EpNotConfigured,
    NvBootError_WarmBoot0_Failure,
    NvBootError_AccessDenied,
    NvBootError_InvalidOscFrequency,
    NvBootError_PllNotLocked,
    NvBootError_InvalidDevParams,
    NvBootError_InvalidBootDeviceEncoding,
    NvBootError_CableNotConnected,
    NvBootError_InvalidBlDst,
    NvBootError_SE_ModExp_OOR,   // Message or Signature representative
                                 // out of range
    NvBootError_SE_RsaPssVerify_Inconsistent, //TODO: add more granulatiry on the errors
    NvBootError_FuseHashMismatch, // Mismatch of hash of public key modulus read from
                                  // secondary storage and fuses

    //Xusb related new error codes
    NvBootError_XusbDeviceNotAttached,
    NvBootError_XusbPortResetFailed,
    NvBootError_XusbInvalidBmRequest,
    NvBootError_XusbParseConfigDescFail,
    NvBootError_XusbMscInvalidCmd,
    NvBootError_XusbCswStatusCmdFail,
    NvBootError_XusbMscResetRecovery,
    NvBootError_XusbEpStalled,
    NvBootError_XusbEpError,
    NvBootError_XusbEpRetry,
    NvBootError_XusbEpNotReady,
    //end Xusb
    // Xusb device error codes
    NvBootError_XusbControlSeqNumError,
    NvBootError_XusbControlDirError,
    NvBootError_XusbOutofSync,
    NvBootError_XusbPortError,

    NvBootError_SE_Context_Restore_Failure,
    NvBootError_SecProvisioningBctKeyMismatch, // Indicates a mismatch between SecProvisioningKeyNum_Secure
                                               // SecProvisioningKeyNum_Insecure
    NvBootError_SecProvisioningRcmKeyMismatch, // Indicates a mismatch between SecProvisioningKeyNum_Secure
                                               // SecProvisioningKeyNum_Insecure
    NvBootError_SecProvisioningDisabled,
    NvBootError_SecProvisioningEnabled,
    NvBootError_SecProvisioningInvalidKeyInput, // Indicates an invalid/out of bounds value
                                                // in SecProvisioningKeyNum_Insecure
    NvBootError_SecProvisioningInvalidAntiCloningKey,
    NvBootError_SecProvisioningAntiCloningKeyDisabled,
    NvBootError_Force32 = 0x7fffffff
} NvBootError;
#endif

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_ERROR_H */

