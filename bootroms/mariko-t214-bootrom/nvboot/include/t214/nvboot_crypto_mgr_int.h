  /*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * NvBoot Crypto Manager interface.
 *
 * This is a simple, HW engine and algorithm independent interface
 * for use by boot code. It's primary purpose is to separate the
 * underlying algorithms, HW engines and implementation from the boot code, thus
 * simplifying maintenance of said code from generation to generation.
 *
 * Currently the crypto manager will only accept one operation at a time.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_MGR_INT_H
#define INCLUDED_NVBOOT_CRYPTO_MGR_INT_H

#include "nvboot_crypto_param.h"
#include "nvboot_bct.h"
#include "nvboot_oem_boot_binary_header.h"
#include "nvboot_nv_header.h"
#include "nvboot_warm_boot_0.h"
#include "nvboot_rcm.h"
#include "nvboot_crypto_pkc_rsassa_pss_int.h"
#include "nvboot_sha_devmgr_int.h"
#include "nvboot_aes_devmgr_int.h"
#include "nvboot_rsa_devmgr_int.h"
#include "nvboot_crypto_fskp.h"

typedef enum
{
    CryptoMgrStatus_Idle,
    CryptoMgrStatus_Success = CryptoMgrStatus_Idle,
    CryptoMgrStatus_Busy,
    CryptoMgrStatus_Unsupported,
    CryptoMgrStatus_FunctionCallRejected,

    CryptoMgrStatus_Num,
    CryptoMgrStatus_Force32 = 0x7fffffff,
} NvBootCryptoMgrStatus;

static const uint8_t CRYPTOMGR_RSA_PUBLIC_KEY_SLOT = 0;

typedef struct NvBootCryptoMgrBuffersRec
{
    uint32_t FusePcpHashBuf[NVBOOT_SHA256_LENGTH_WORDS];
    NvBootPublicCryptoParameters Pcp;
    NvBootCryptoRsaSsaPssContext RsaPssContext;
    NvBootSha256HashDigest OemBootBinaryHash;
    NvBootSha256HashDigest Calculated_BlHash;
    NvBootAesDeviceCmacContext AesCmacContext;
    uint32_t AesCmacHashResult[NVBOOT_AES_BLOCK_LENGTH_WORDS];
    uint32_t CmacK1[NVBOOT_AES_BLOCK_LENGTH_WORDS];
    uint32_t CmacK2[NVBOOT_AES_BLOCK_LENGTH_WORDS];
} NvBootCryptoMgrBuffers;

// Random 32-bit integers instead of bool, as a fault injection
// countermeasure.
typedef enum
{
    OEM_HEADER_NOT_AUTHENTICATED = 0x70ED729D,
    OEM_HEADER_AUTHENTICATED = 0x755AA14F,

    OEM_HEADER_FORCE32 = 0x7FFFFFFF,
} OemHeaderEnum;

typedef struct CryptoMgrContextRec
{
    NvU32 TaskId;
    NvBootCryptoMgrStatus Status;
    //Add engine selection mechanism for each possible algorithm.

    NvBootCryptoAlgo AuthenticationScheme;
    NvBootCryptoAlgo EncryptionScheme;

    NvBootAesKeySize AesKeySize;

    NvBootCryptoEngine EngineForAuthentication;
    NvBootCryptoEngine EngineForEncryption;

    // Algorithm specific parameters.
    NvBootCryptoParams CryptoParams;
    NvBootCryptoNvParams NvCryptoParams;

    /// Not used in t214
    // NvBootEccPrimeFieldParams EccPrimeFieldParams;

    // // NVIDIA ECC public key
    // NvBootEcPoint NvEccPublicKey;

    // NVIDIA RSA public key
    NvBootCryptoRsaKey NvRsaPublicKey;

    // Parameter to indicate if the inputted public key from
    // BCT/LP0 header/RCM header has been validated against the
    // SHA256 hash of the public key fuses.
    NvBool  IsOemPcpValidated;


    uint32_t IsOemBootBinaryHeaderAuthenticated;
} NvBootCryptoMgrContext __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

/**
 * \brief       Publicly accessible buffer storage for usage by RCM, coldboot, LP0.
 *
 * Storing the computed signatures here eases debug.
 *
 * NOTE!! Must be declaraed in SYSRAM for usage with SE.
 */
typedef struct NvBootCryptoMgrPublicBufRec
{
    // Public buffer for Nv-signed computed signatures.
    NvBootCryptoSignatures CalculatedSignaturesNvSigned;

    // Public buffer for OEM-signed computed signatures.
    NvBootCryptoSignatures CalculatedSignaturesOemSigned;

    // Public buffer for the output digest of a SHA256 Hash.
    NvBootSha256HashDigest Sha256HashDigest;
} NvBootCryptoMgrPublicBuf __attribute__((aligned(NVBOOT_CRYPTO_BUFFER_ALIGNMENT)));

/**
 * \brief       Setup software context of crypto manager.
 *              Loads NIST P-256 curve parameters.
 *              Sense security state of the chip.
 *
 * \note        Always returns NvBootError_Success because this
 *              function is included in the secure dispatcher table.
 */

NvBootError NvBootCryptoMgrInit(void);

/**
 * \brief       Sense the chip state and set the CryptoMgrContext accordingly.
 *
 * Detect the production level of the chip (Pre, Nv, ODM).
 *
 */
void NvBootCryptoMgrSenseChipState(void);

/**
 * \brief       Do any required HW initialization sequence for
 *              all crypto engines.
 *
 * \note        Always returns void 
 *
 */
void NvBootCryptoMgrHwEngineInit(void);

void NvBootCryptoMgrSetStatus(NvBootCryptoMgrStatus Status);

NvBootCryptoMgrStatus CryptoMgrGetStatus(void);

void NvBootCryptoMgrSetStatus(NvBootCryptoMgrStatus Status);

NvBootCryptoAlgo NvBootCryptoMgrGetAuthScheme(void);
void NvBootCryptoMgrSetAuthScheme(NvBootCryptoAlgo AuthScheme);

/**
 * \brief           Load an AES key into the crypto mgr context for
 *                  use when in SW AES mode.
 *
 * \note            Valid for use only in the cases where SW AES can
 *                  be used.
 */
void NvBootCryptoMgrSetOemAesKeyForSwAes(uint8_t *Key, NvBootAesKeySize KeySize);

/**
 * \brief           Load a user specified SBK into the appropriate
 *                  HW key slot. Only applicable when the PRODUCTION_MODE
 *                  fuse is NOT set.
 *
 * \note            This function will also zero out the IV slots.
 * \note            Key must be in a word aligned buffer.
 */
void NvBootCryptoMgrOverrideSBK(NvU32 *Key);

// Override the RSA public key that the crypto manager uses.
void NvBootCryptoMgrSetOemRsaPublicKey(uint8_t *Key, NvBootCryptoRsaKeySize KeySize);

/**
 * \brief           Copy the Pcp into the crypto mgr context.
 *
 * \param Pcp       Buffer holding the Pcp. MUST be in SYSRAM.
 *
 * \return          NvBootError_Scucess if Pcp is valid against the
 *                  SHA256 hash of the PUBLIC_KEY* fuses.
 *
 * Note, the Pcp must still be validated against the SHA256 hash
 * of the Pcp burned in fuses.
 * CryptoMgrContext.IsOemPublicKeyValidated will be NV_TRUE if the
 * public key has been validated.
 */
NvBootError NvBootCryptoMgrSetOemPcp(NvBootPublicCryptoParameters *Pcp);

/**
 * \brief           Load the Pcp currently in the crypto mgr context into
 *                  the appropriate HW engines.
 */
void NvBootCryptoMgrLoadPcpIntoEngines();

/**
 * \brief           Validate the Pcp loaded into the crypto mgr context
 *
 * Calculate the SHA256 hash of the Pcp and compare it against the
 * value burned into fuses (FUSE_PUBLIC_KEY0-7).
 * If true, set NvBootCryptoMgrContext.IsOemPublicKeyValidated = NV_TRUE;
 */
void NvBootCryptoMgrValidatePcp(void);

NvBool NvBootCryptoMgrIsOemPcpValidated(void);

void NvBootCryptoMgrOverrideEngineForAuth(NvBootCryptoEngine Engine);

/**
void NvBootCryptoMgrOverrideEncryptionScheme(NvBootCryptoMgrContext *CryptoMgrContext, NvBootCryptoAlgo EncryptionScheme);

void NvBootCryptoMgrOverrideEngineForEncrypt(NvBootCryptoMgrContext *CryptoMgrContext, NvBootCryptoEngine Engine);
*/



/**
 * \brief           Encrypt a contiguous paypload with a user specifed encryption
 *                  algorithm
 *
 * \param Algoirthm Encryption algorithm to use.
 * \param AlgoParms Parameters specific to the encryption algorithm.
 * \param NvParams  Parameters specific to NVIDIA.
 * \param Input     Buffer holding the input data.
 * \param Output    Buffer holding the output data
 * \param Length    Length in bytes of the overall payload.
 *
 */
void NvBootCryptoMgrEncrypt(NvBootCryptoAlgo Algorithm,
                      NvBootCryptoParams *AlgoParams,
                      NvBootCryptoNvParams *NvParams,
                      uint8_t *Input,
                      uint8_t *Output,
                      NvU32 Length);

/**
 * \brief           Hash a contiguous paypload with a user specifed
 *                  algorithm
 *
 * \param Algoirthm Hashing algorithm to use.
 * \param AlgoParms Parameters specific to the hash algorithm.
 * \param NvParams  Parameters specific to NVIDIA.
 * \param Input     Buffer holding the input data.
 * \param Output    Buffer holding the output data
 * \param Length    Length in bytes of the overall payload.
 *
 */
NvBootError NvBootCryptoMgrHash(NvBootCryptoAlgo Algorithm,
                         NvBootCryptoParams *AlgoParams,
                         NvBootCryptoNvParams *NvParams,
                         uint8_t *Input,
                         uint8_t *Output,
                         NvU32 Length);

NvBootError NvBootCryptoMgrDecKeys();

NvBootError NvBootCryptoMgrReadyEncKey();

/**
 * \brief           Load OEM AES key.
 *
 * \note            We currently only have the OEM owned SBK.
 *
 * \note            This function does NOT lock the SBK key slot.
 *                  The BR exit functions must do this.
 *
 * \note            Always returns NvBootError_Success because this
 *                  function is included in the secure dispatcher table.
 */
NvBootError NvBootCryptoMgrLoadOemAesKeys();

/**
 * \brief           Load NVIDIA keys from the IROM secure keys space
 *                  or from the regular key space into the appropriate
 *                  Engines and/or CryptoMgr context.
 *
 * \note            BR needs the MB1 decryption key and the ECC public
 *                  key.
 *
 * \note            If the PRODUCTION_MODE fuse is set, we read from
 *                  the 4KB of IROM space at the top of the IROM
 *                  address space.
 *                  If the PRODUCTION_MODE fuse is not set AND
 *                  the DUMMY_KEY_FUSE isn't set, we read
 *                  from the 4KB of IROM space.
 *                  If the PRODUCTION_MODE fuse is not set AND
 *                  the DUMMY_KEY_FUSE is set, we get the keys
 *                  from nvboot_crypto_nv_dev_keys.h
 *
 * \note            Always returns NvBootError_Success because this
 *                  function is included in the secure dispatcher table.
 */
NvBootError NvBootCryptoMgrLoadNvKeys();

/**
 * \brief           Load FSPK from secure IROM.
 *
 * \param           KeySelection The FSPK number.
 *
 * \param           KeySlot The SE keyslot to load into.
 *
 */
void NvBootCryptoMgrLoadFSPK(uint8_t KeySelection, uint8_t KeySlot);

/**
 * \brief           If the BCT BctKEKKeySelect set to 1, update KEK keyslot
 *                  with the full 256-bit AES encryption key stored in fuses.
 *
 */
void NvBootCryptoMgrLoadOemKEK1As256b();

/**
 * \brief           Check the validity of an Elliptic Curve Public key
 *                  by calculating if it lies on the specified curve.
 *
 * \param           Qx x-coordinate of the point.
 * \param           Qy y-coordinate of the point.
 */
// NvBootError NvBootCryptoMgrVerifyEcPoint(NvU32 *Qx, NvU32 *Qy, NvBootEccEllipticCurves EcCurve);



/**
 * New BR specific authentication/decryption functions
 *
 */

/**
 * Authenticate the BCT. Crypto manager will automatically detect the authentication
 * scheme.
 *
 * @param Bct Pointer to BCT structure. Bct must be in on-chip memory accessible by
 *            the crypto engines.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *
 */
NvBootError NvBootCryptoMgrAuthBct(const NvBootConfigTable *Bct);

/**
 * Decrypt the BCT using the OEM specified SBK or BEK.
 *
 * @param Bct Pointer to BCT structure. Bct must be in on-chip memory accessible
 *        by the crypto engines.
 *
 * @return NvBootError NvBootError_Success if decryption successful or disabled.
 *         Error if Bct pointer is in invalid memory location, or if the AES callback
 *         function returns an error.
 */
NvBootError NvBootCryptoMgrDecryptBct(NvBootConfigTable *Bct);

/**
 * Authenticate the OEM signed boot binary header.
 *
 * @param OemHeader Pointer to NvBootOemBootBinaryHeader structure. OemHeader must be in
 * on-chip memory accessible by the crypto engines.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *         Error will be returned if the authentication length is larger than
 *         the maximum buffer size.
 *
 */
NvBootError NvBootCryptoMgrAuthOemBootBinaryHeader(const NvBootOemBootBinaryHeader *OemHeader);


/**
 * Authenticate the encrypted NV Binary package which consists of the NV header
 * plus the encrypted MB1 binary using the OEM specified authentication key.
 * Cryptomgr automatically detects which key and authentication scheme to use.
 * Size information is in the NvBootOemBootBinaryHeader.
 *
 * @param OemHeader Pointer to NvBootOemBootBinaryHeader structure. Must be in on-chip memory accessible
 *        by the crypto engines.
 * @param OemNvBinaryPackage Pointer to the whole NV Binary package.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *         Error will be returned if the authentication length is larger than
 *         the maximum buffer size.

 */
NvBootError NvBootCryptoMgrAuthOemNvBinaryPackage(const NvBootOemBootBinaryHeader *OemHeader, uint32_t *OemNvBinaryPackage);


/**
 * Decrypted the encrypted NV Binary package which consists of the NV header plus
 * the encrypted MB1 binary with the OEM encryption key (SBK).
 *
 * @param OemHeader Pointer to NvBootOemBootBinaryHeader structure. Must be in on-chip memory accessible
 *        by the crypto engines.
 * @param OemNvBinaryPackage Pointer to the whole NV Binary package.
 *
 * @return NvBootError NvBootError_Success if decryption is successful. Error will be
 *         returned if the Length is larger than the maximum buffer size.
 *
 */
NvBootError NvBootCryptoMgrDecryptOemNvBinaryPackage(const NvBootOemBootBinaryHeader *OemHeader, uint32_t *OemNvBinaryPackage);


/**
 * Perform NV authentication of the NV binary and NV header using an NVIDIA
 * owned authentication key. Cryptomgr automatically detects which key and
 * authentication scheme to use.
 * This function assumes the NvBinary (MB1 in BR's case) will be
 * present immediately following the NvHeader.
 *
 * Size information of the NV binary is taken from NvHeader->BinaryLength.
 * BR will perform authentication to the smaller of BinaryLength or the
 * maximum size of the buffer in SYSRAM holding the NV binary and NV header.
 *
 * @param NvHeader Pointer to the NvBinarySignHeader.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *         Error will be returned if the authentication length is larger than
 *         the maximum buffer size.
 *
 */
NvBootError NvBootCryptoMgrAuthNvBinary(const NvBinarySignHeader *NvHeader);

/**
 * Decrypt the encrypted NV binary using an NVIDIA owned decryption key.
 * Cryptomgr automatically detects which key to use.
 *
 * @param NvHeader Pointer to NvHeader structure. Use the destination field in the
 *                 header to know where to decrypt to. Is is also assumed the source
 *                 for the decryption is immediately following the NvHeader.
 *
 * @return NvBootError NvBootError_Success if decryption is successful. Error will be
 *         returned if the Length is larger than the maximum buffer size.
 *         NvBootError_Encryption_Not_Enabled if encryption is not
 *          enabled (Note however, currently NV encryption is mandatory).
 */
NvBootError NvBootCryptoMgrDecryptNvBinary(const NvBinarySignHeader *NvHeader);


/**
 * Perform authentication of the BL1 header and binary using an OEM owned
 * authentication key.
 * Cryptomgr automatically detects which key and authentication scheme to use.
 *
 * @param OemHeader Pointer to the NvBootOemBootBinaryHeader struct.
 * @param BlBinary Pointer to the BL1 binary.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *         Error will be returned if the authentication length is larger than
 *         the maximum buffer size.
 */
NvBootError NvBootCryptoMgrAuthBlPackage(const NvBootOemBootBinaryHeader *OemHeader, uint32_t *BlBinary);

/**
 * Perform decryption of the BL binary using an OEM owned decryption key.
 * Cryptomgr automatically detects which key and decryption algorithm to use.
 * This function will sanitize the load address and length.
 *
 * @param OemHeader Pointer to the NvBootOemBootBinaryHeader struct.
 * @param BlBinary Pointer to the BL1 binary.
 *
 * @return NvBootError NvBootError_Success if decryption is successful or if
 *         encryption is disabled.
 */
NvBootError NvBootCryptoMgrDecryptBlPackage(const NvBootOemBootBinaryHeader *OemHeader, uint32_t *BlBinary);

/**
 * Perform authentication of the SC7 firmware using the OEM authentication key.
 * Cryptomgr automatically detects which key and authentication scheme
 * to use.
 *
 * @param Sc7Header Pointer to NvBootWb0RecoveryHeader. The SC7 binary should be
 *                  located immediately after the header.
 *
 */
NvBootError NvBootCryptoMgrOemAuthSc7Fw(const NvBootWb0RecoveryHeader *Sc7Header);

/**
 * Perform decryption of the SC7 header and firmware using an OEM owned
 * authentication key.
 * This function will decrypt to the (validated) Destination listed in the header.
 * Cryptomgr automatically detects which key and authentication scheme to use.
 *
 * @return NvBootError NvBootError_Success if decryption is successful.
 *         Error will be returned if the decryption length is larger than
 *         the maximum buffer size.
 *         NvBootError_Encryption_Not_Enabled if encryption is not
 *         enabled.
 */
NvBootError NvBootCryptoMgrOemDecryptSc7Fw(const NvBootWb0RecoveryHeader *Sc7Header);

/**
 * Perform NV authentication of the NV signed SC7 firmware and NV header
 * using an NVIDIA owned authentication key. Cryptomgr automatically
 * detects which key and authentication scheme to use.
 * This function assumes the NvBinary (MB1 in BR's case) will be
 * present immediately following the NvHeader.
 *
 * @param NvHeader Pointer to NvHeader structure. Use the destination field in the
 *                 header to know where to decrypt to. Is is also assumed the source
 *                 for the decryption is immediately following the NvHeader.
 */
NvBootError NvBootCryptoMgrNvAuthSc7Fw(const NvBinarySignHeader *NvHeader);


/**
 * Decrypt the binary encrypted by an NVIDIA owned decryption key.
 * Cryptomgr automatically detects which key to use.
 * @param NvHeader Pointer to NvHeader structure. Use the destination field in the
 *                 header to know where to decrypt to. Is is also assumed the source
 *                 for the decryption is immediately following the NvHeader.
 *
 * @return NvBootError NvBootError_Success if decryption is successful. Error will be
 *         returned if the Length is larger than the maximum buffer size.
 *         NvBootError_Encryption_Not_Enabled if encryption is not
 *          enabled (Note however, currently NV encryption is mandatory).
 *
*/
NvBootError NvBootCryptoMgrNvDecryptSc7Fw(const NvBinarySignHeader *NvHeader);



NvBootError NvBootCryptoMgrOemAuthRcmPayload(const NvBootRcmMsg *RcmMsg);

NvBootError NvBootCryptoMgrOemDecryptRcmPayload(const NvBootRcmMsg *RcmMsg);

NvBootError NvBootCryptoMgrNvAuthRcmPayload(const NvBinarySignHeader *NvHeader);

NvBootError NvBootCryptoMgrNvDecryptRcmPayload(const NvBinarySignHeader *NvHeader);

NvBootError NvBootCryptoMgrLoadDefaultSEKeys();

typedef enum
{
    FSKP_TYPE_COLDBOOT,
    FSKP_TYPE_RCM,

    FSKP_TYPE_FORCE32 = 0x7fffffff,

} NvBootCryptoMgrFskpType;

/**
 * Initialize and load FSKP key for use.
 * Handles key wrap key case as well.
 *
 * @param[in] KeyNum, the FSKP key number specified in the BCT or RCM
 * header.
 * @param[in] KeyWrapKey Pointer to the key wrap key structure in the BCT
 * or RCM header.
 */
NvBootError NvBootCryptoMgrFskpInit(FskpKeyNum KeyNum, uint8_t *KeyWrapKey);

/**
 * Authenticate the BCT using the FSKP AES key. FSKP must be loaded into an
 * SE key slot before calling this function.
 *
 * @param Bct Pointer to BCT structure. Bct must be in on-chip memory accessible by
 *            the crypto engines.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *
 */
NvBootError NvBootCryptoMgrAuthBctFskp(const NvBootConfigTable *Bct);

/**
 * Decrypt the BCT using the FSKP key loaded into an SE key slot..
 *
 * @param Bct Pointer to BCT structure. Bct must be in on-chip memory accessible
 *        by the crypto engines.
 *
 * @return NvBootError NvBootError_Success if decryption successful or disabled.
 *         Error if Bct pointer is in invalid memory location, or if the AES callback
 *         function returns an error.
 */
NvBootError NvBootCryptoMgrDecryptBctFskp(NvBootConfigTable *Bct);

/**
 * Authenticate the RCM header and payload using the FSKP AES key. FSKP must be loaded into an
 * SE key slot before calling this function.
 *
 * @param Bct Pointer to RCM structure. Bct must be in on-chip memory accessible by
 *            the crypto engines.
 *
 * @return NvBootError NvBootError_Success if authentication is successful.
 *
 */
NvBootError NvBootCryptoMgrAuthRcmPayloadFskp(const NvBootRcmMsg *RcmMsg);

/**
 * Decrypt the RCM header and payload the FSKP key loaded into an SE key slot..
 *
 * @param Bct Pointer to BCT structure. Bct must be in on-chip memory accessible
 *        by the crypto engines.
 *
 * @return NvBootError NvBootError_Success if decryption successful or disabled.
 *         Error if Bct pointer is in invalid memory location, or if the AES callback
 *         function returns an error.
 */
NvBootError NvBootCryptoMgrDecryptRcmPayloadFskp(const NvBootRcmMsg *RcmMsg);

#endif //INCLUDED_NVBOOT_CRYPTO_MGR_INT_H
