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
#include "arapbpm.h"
#include "nvboot_bit.h"
#include "nvboot_buffers_int.h"
#include "nvboot_config.h"
#include "nvboot_config_int.h"
#include "nvboot_wdt_int.h"
#include "nvboot_error.h"
#include "nvboot_fuse.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_hash_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_rcm_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_int.h"
#include "nvboot_se_hash.h"
#include "nvboot_se_rsa.h"
#include "nvboot_usbf_int.h"
#include "nvboot_util_int.h"
#include "nvboot_version_rom.h"
#include "nvboot_rcm_port_int.h"

/* Local state structure */
typedef struct NvBootRcmStateRec
{
    NvBootFuseOperatingMode RcmOpMode;
    NvBootObjectSignature   ComputedSignature;
    NvBool                  IsPublicKeyValidated;
    NvU32                   SecureDebugControl;
    NvU32                   BufferIndex; /* Index of USB receive buffer */
    NvBool                  FactorySecureProvisioningMode;
    NvU8                    ValidationKeySize;
    NvU8                    ProvisioningKeyNum;
    NvU8                    DecryptionKeySlotNum;
    NvU8                    CMACHashKeySlotNum;
    NvBool                  FirstMessageProcessed;
} NvBootRcmState;

typedef struct NvBootRcmSeInputLinkedListRec
{
    NvU32                   LastBufferNumber;
    SeLinkedListElement     LLElement[2];
} NvBootRcmSeInputLinkedList;

#define NV_BOOT_RCM_CHECK_ERROR(expr) \
    do \
    { \
        e = (expr); \
        if (e == NvBootError_EpNotConfigured) \
        { \
            pRcmPort->Context.Connected = NV_FALSE; \
            continue; \
        } \
        else if(e != NvBootError_Success) \
            return e; \
    } while (0)


/* External data */
extern NvBootInfoTable BootInfoTable;

/*
 * Compile-time assertion that the hashable & decryptable portion of
 * NvBootRcmMsg is a multiple of 16 bytes.
 */
NV_CT_ASSERT((sizeof(NvBootRcmMsg) - offsetof(NvBootRcmMsg, RandomAesBlock) & 0xf) == 0);

/*
 * Function Prototypes
 */

#if USE_FULL_RCM

static NvBootRcmState s_State;
static NvBootRcmMsg   s_Msg;
static NvBootRcmSeInputLinkedList   s_SeInputLL;

static void        CompressRcmVersion(NvU32 OriginalVersion, NvU16 *CompressedVersion);
static NvBootError SendUniqueId(void);
static NvBootError SendResponse(NvU32 Response);
static void        HandleError(NvU32 Response);
static NvBool      IsEntryPointWithinPayload(void);
static NvBootError ReceiveMessage(void);
static void        ComputeHash(void);
static void        Decrypt(void);
static NvBootError Validate(void);
static NvBootError Execute(NvBool *AppletDownloaded);

/**
 * Compress/truncate 32-bit RCM version to 16-bits to fit into the extra
 * 24-bits of unused space in the last dword of the ECID.
 *
 * This function takes the lower byte of each of the major revision and the
 * minor revision and creates a 16-bit value.
 *
 * For example, if the RCM version is 0x00140001, this function truncates the version
 * to 0x1401.
 *
 * @param The 32-bit RCM version that needs compression
 * @param The pointer to the 16-bit compressed version number.
 *
 * @return void
 *
 */
static void CompressRcmVersion(NvU32 OriginalVersion, NvU16 *CompressedVersion)
{
    NvU8 MajorRevision;
    NvU8 MinorRevision;

    MajorRevision = OriginalVersion >> 16;
    MinorRevision = (NvU8) OriginalVersion;

    *CompressedVersion = (MajorRevision << 8) | MinorRevision;
}

/* Synchronous transmission */
static NvBootError SendUniqueId(void)
{
    NvU8               *SendBuffer;
    NvU16               CompressedRcmVersion;
    NvBootError BootError = NvBootError_Success;
    NvU32 BytesTransmitted;
    NvBootRCMPort_T   *pRcmPort = NvBootRcmGetPortHandle();

    SendBuffer = Buffer[s_State.BufferIndex];

    /* Read the ECID into the send buffer. */
    NvBootFuseGetUniqueId((NvBootECID *)SendBuffer);

    //Embed operating mode info into ECID
    ((NvBootECID *)SendBuffer)->ECID_3 |= NV_DRF_NUM(ECID,ECID3,OPMODE,s_State.RcmOpMode);

    // Embed RCM version into unused portion of ECID.
    CompressRcmVersion(CONST_NVBOOT_RCM_VERSION, &CompressedRcmVersion);
    ((NvBootECID *)SendBuffer)->ECID_3 |= NV_DRF_NUM(ECID,ECID3,RCM_VERSION,CompressedRcmVersion);

    pRcmPort->TransferStart(SendBuffer, sizeof(NvBootECID));

    /*
     * Query the endpoint status until the transfer is completed, there
     * is an error detected over the USB, or cable disconnect.
     */

    // Note: Timeout is not used by Synopsys USB driver. Setting a high value for future ports.
    // The buffer passed is used by Synopsys for any pending control transfers.
    // Going forward, RCM port should use their internal resources for this purpose.
    BootError = pRcmPort->TransferPoll(&BytesTransmitted, 0xFFFFFFFF, Buffer[s_State.BufferIndex ^ 1]);
    return BootError;
}

/* Synchronous transmission */
static NvBootError
SendResponse(NvU32 Response)
{
    NvBootError  e;
    NvU8        *SendBuffer;
    NvU32        BytesTransmitted;
    NvBootRCMPort_T   *pRcmPort = NvBootRcmGetPortHandle();
    /*
     * Use the second buffer of the buffer pair, in case a receive is
     * in progress on the other buffer.
     */

    SendBuffer = Buffer[s_State.BufferIndex ^ 1];

    // Buffer must be 4K bytes and aligned to 4K bytes
    *((NvU32*)SendBuffer) = Response;

    e = pRcmPort->Transfer(SendBuffer, sizeof(NvU32), &BytesTransmitted);
    // if there is any error, mostly could be Cable disconnect or
    // timeout of 1 second, host is not ready to take data in 1 second.
    if (e)
    {
        // if cable is present then Stall out endpoint saying host
        // not send any more data to us.
        if (e != NvBootError_EpNotConfigured)
            pRcmPort->HandleError();
        // then stop here.
        return e;
    }

    return NvBootError_Success;
}


static void HandleError(NvU32 Response)
{
    NvBootRCMPort_T   *pRcmPort = NvBootRcmGetPortHandle();
    // The caller will report an error, so it doesn't matter if
    // the sending of the response fails.
    (void)SendResponse(Response);

    // Stall the endpoint
    pRcmPort->HandleError();
}

static NvBool IsValidInsecureLength(NvU32 Length)
{
    const NvU32 UnsignedLengthModAESBlk = ((NvU32) &(s_Msg.RandomAesBlock) - (NvU32) &(s_Msg)) % NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    if (Length >= NVBOOT_RCM_MAX_MSG_LENGTH) return NV_FALSE;
    if (Length <  NVBOOT_RCM_MIN_MSG_LENGTH) return NV_FALSE;

    /* For AES reasons, the hashed (& encrypted in OdmSecure) portions of
     * the message must be a multiple of 16 bytes.  The unsigned
     * length AES block remainder should be everything before RandomAesBlock % 16.
     */
    if ((Length % NVBOOT_SE_AES_BLOCK_LENGTH_BYTES) != UnsignedLengthModAESBlk) return NV_FALSE;

    return NV_TRUE;
}

static NvBool
IsEntryPointWithinPayload(void)
{
    NvU32 EntryPoint = s_Msg.Args.DownloadData.EntryPoint;

    if (EntryPoint < NVBOOT_RCM_DLOAD_BASE_ADDRESS) return NV_FALSE;

    if (EntryPoint >= (NVBOOT_RCM_DLOAD_BASE_ADDRESS + s_Msg.PayloadLength))
        return NV_FALSE;

    return NV_TRUE;
}

/**
 * Begin by storing the received data into the header structure.
 * Once the header is complete, the remaining data fills the 96KB starting at
 * 0x40008000.
 *
 * Notes:
 *   BytesLeftInMessage refers to data remaining to be copied.
 *   BytesRead counts the amount of data that has been read from USB.
 *   ReadFromPort is true on all cycles of the while() loop where there is
 *     reading from USB.
 */
static NvBootError ReceiveMessage(void)
{
    NvU32        BytesLeftInMessage = NVBOOT_RCM_MIN_MSG_LENGTH;
    NvU32        TotalBytesReceived = 0;
    NvBool       IsDefaultLength    = NV_TRUE;
    NvBool       ReadFromPort;
    NvU32        BytesRead          = 0;
    NvU32        BytesToCopy;
    NvU8        *Dst                = (NvU8*)&s_Msg;
    NvU8        *Src;
    NvBootError  e;
    NvBootRCMPort_T *pRcmPort = NvBootRcmGetPortHandle();

    while (BytesLeftInMessage > 0)
    {
        /* Swap buffers */
        s_State.BufferIndex ^= 1;

        /* Initiate read from USB if more data is expected */
        ReadFromPort = (BytesLeftInMessage > BytesRead);
        if (ReadFromPort)
            pRcmPort->ReceiveStart(Buffer[s_State.BufferIndex],
                                   NVBOOT_BUFFER_LENGTH);

        /* Process the buffer that completed. */
        Src = Buffer[s_State.BufferIndex ^ 1];

        /* Copy data into the destination(s) */
        if ((BytesRead > 0) &&
            (TotalBytesReceived < sizeof(NvBootRcmMsg)))
        {
            /* Copy data into the header */
            BytesToCopy = NV_MIN(sizeof(NvBootRcmMsg) - TotalBytesReceived,
                                 BytesRead);

            memcpy(Dst, Src, BytesToCopy);

            /* Update pointers and counts. */
            Dst += BytesToCopy;
            Src += BytesToCopy;

            TotalBytesReceived += BytesToCopy;
            BytesLeftInMessage -= BytesToCopy;
            BytesRead          -= BytesToCopy;

            if (TotalBytesReceived == sizeof(NvBootRcmMsg))
            {
                /* Switch to storing data in the 224KB of iRAM */
                Dst = (NvU8*)(NVBOOT_RCM_DLOAD_BASE_ADDRESS);
            }

            /* Switch to the insecure length when it has been read. */
            if (IsDefaultLength && (TotalBytesReceived >= sizeof(NvU32)))
            {
                if (!IsValidInsecureLength(s_Msg.LengthInsecure))
                {
                    HandleError(NvBootRcmResponse_InvalidInsecureLength);
                    return NvBootError_ValidationFailure;
                }

                BytesLeftInMessage = s_Msg.LengthInsecure -
                    TotalBytesReceived;
                IsDefaultLength = NV_FALSE;
            }
        }

        /* Copy any remaining data to Dst */
        BytesToCopy = NV_MIN(BytesLeftInMessage, BytesRead);
        memcpy(Dst, Src, BytesToCopy);

        /* Update pointers and counts. */
        Dst += BytesToCopy;
        Src += BytesToCopy;

        TotalBytesReceived += BytesToCopy;
        BytesLeftInMessage -= BytesToCopy;
        BytesRead          -= BytesToCopy;

        /* Any remaining bytes are an overflow error. */
        if (BytesRead > 0)
        {
            HandleError(NvBootRcmResponse_XferOverflow);
            return NvBootError_ValidationFailure;
        }

        /* Spin wait for USB read to complete */
        if (ReadFromPort)
        {
            /* Note: Timeout value is not used by Synopsys driver.
             * Setting a high value for future RCM ports.
             * The buffer passed is used by Synopsys driver for pending control transfers.
             * Going forward, RCM ports should use internal resources for this purpose.
             */
            e = pRcmPort->ReceivePoll(&BytesRead, 0xFFFFFFFF,Buffer[s_State.BufferIndex]);
            if ( e == NvBootError_EpNotConfigured)
            {
                // Endpoint will be un-configured only it reset occurs,
                // If reset occurs, need do re-enumearation
                return e;
            }
            else if (e != NvBootError_Success)
            {
                HandleError(NvBootRcmResponse_UsbError);
                return e;
            }
        }
    }

    /* Message was successfully received. */
    return  NvBootError_Success;
}

static void
ComputeHash(void)
{
    NvBool      First = NV_TRUE;
    NvBool      Last  = NV_FALSE;
    NvU32       HeaderBlocks;
    NvU32       TotalBlocks;
    NvU8       *Src = (NvU8*)&(s_Msg.RandomAesBlock);
    NvU32       Offset; /* to the start of the hashed area of the message */
    static NvU32 K1[NVBOOT_SE_AES_BLOCK_LENGTH];
    static NvU32 K2[NVBOOT_SE_AES_BLOCK_LENGTH];
    static NvBool firstTimeVisit = NV_TRUE;

    Offset       = (NvU8*)&(s_Msg.RandomAesBlock) - (NvU8*)&s_Msg;
    HeaderBlocks = (sizeof(NvBootRcmMsg) - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;
    TotalBlocks  = (s_Msg.LengthInsecure - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;

    if (firstTimeVisit)
    {
        NvBootSeAesCmacGenerateSubkey(s_State.CMACHashKeySlotNum,
                                      s_State.ValidationKeySize,
                                      &K1[0],
                                      &K2[0]);
        firstTimeVisit = NV_FALSE;
    }

    NvBootSeAesCmacHashBlocks(&K1[0],
                              &K2[0],
                              (NvU32 *) Src,
                              (NvU8 *) &(s_State.ComputedSignature.CryptoHash),
                              s_State.CMACHashKeySlotNum,
                              s_State.ValidationKeySize,
                              HeaderBlocks,
                              First,
                              Last);

    /* Wait for completion */
    while(NvBootSeIsEngineBusy())
        ;

    First = NV_FALSE;
    Last = NV_TRUE;
    Src = (NvU8*)(NVBOOT_RCM_DLOAD_BASE_ADDRESS);

    NvBootSeAesCmacHashBlocks(&K1[0],
                              &K2[0],
                              (NvU32 *) Src,
                              (NvU8 *)  &(s_State.ComputedSignature.CryptoHash),
                              s_State.CMACHashKeySlotNum,
                              s_State.ValidationKeySize,
                              TotalBlocks - HeaderBlocks,
                              First,
                              Last);

    /* Wait for completion */
    while(NvBootSeIsEngineBusy())
        ;
}

static void
Decrypt(void)
{
    NvU32       BlocksLeft;
    NvU32       HeaderBlocks;
    NvU8       *Src = (NvU8*)&(s_Msg.RandomAesBlock);
    NvU32       Offset; /* to the start of the hashed area of the message */

    Offset       = (NvU8*)&(s_Msg.RandomAesBlock) - (NvU8*)&s_Msg;
    BlocksLeft   = (s_Msg.LengthInsecure - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;
    HeaderBlocks = (sizeof(NvBootRcmMsg) - Offset) >>
        NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;

    // Clear original IV.
    NvBootSeKeySlotWriteKeyIV(s_State.DecryptionKeySlotNum,
                              s_State.ValidationKeySize,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                              0);

    /* Decrypt the header */
    NvBootSeAesDecrypt(s_State.DecryptionKeySlotNum,
                       s_State.ValidationKeySize,
                       NV_TRUE,
                       HeaderBlocks,
                       Src,
                       Src);

    /* Wait for completion */
    while(NvBootSeIsEngineBusy())
        ;

    /* Decrypt the data */
    BlocksLeft -= HeaderBlocks;
    if (BlocksLeft > 0)
    {
        Src = (NvU8*)(NVBOOT_RCM_DLOAD_BASE_ADDRESS);
        NvBootSeAesDecrypt(s_State.DecryptionKeySlotNum,
                           s_State.ValidationKeySize,
                           NV_FALSE,
                           BlocksLeft,
                           Src,
                           Src);

        /* Wait for completion */
        while(NvBootSeIsEngineBusy())
            ;
    }
}

static NvBootError
Validate(void)
{
    NvU32  i;
    NvU32  PaddingSize;
    NvU8  *PaddingStart;

    /* Common checks */

    if(s_State.FactorySecureProvisioningMode == NV_TRUE)
    {
        // Compare the detected provisioning key number (either the anti-cloning fuse
        // or the _Insecure field) to the _Secure field after decryption
        // If the _Insecure field or anti-cloning fuse doesn't match the Secure field,
        // someone has tried to tamper with the Rcm message.
        if(s_State.ProvisioningKeyNum != s_Msg.SecProvisioningKeyNum_Secure)
        {
            HandleError(NvBootRcmResponse_SecProvisioningRcmKeyMismatch);
            return NvBootError_SecProvisioningRcmKeyMismatch;
        }
    }

    if(s_State.RcmOpMode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
    {
        // If the public key has not been validated, then the message just
        // received cannot be validated. Immediately return validation failure.
        if(s_State.IsPublicKeyValidated == NV_FALSE)
        {
            HandleError(NvBootRcmResponse_PublicKeyNotValidated);
            return NvBootError_ValidationFailure;
        }
    }
    else
    {
        /* Check the hash. */
        for (i = 0; i < NVBOOT_AES_KEY_LENGTH; i++)
        {
            if (s_State.ComputedSignature.CryptoHash.hash[i] != s_Msg.Signature.CryptoHash.hash[i])
            {
                HandleError(NvBootRcmResponse_HashCheckFailed);
                return NvBootError_ValidationFailure;
            }
        }
    }

    if (s_Msg.LengthInsecure != s_Msg.LengthSecure)
    {
        HandleError(NvBootRcmResponse_LengthMismatch);
        return NvBootError_ValidationFailure;
    }

    if (s_Msg.PayloadLength > (s_Msg.LengthSecure - sizeof(NvBootRcmMsg)))
    {
        HandleError(NvBootRcmResponse_PayloadTooLarge);
        return NvBootError_ValidationFailure;
    }

    /*
     * Do not check s_Msg.RcmVersion.  This was removed from AP20 to allow
     * software to query version information without prior knowledge of which
     * version of the chip is connected.
     *
     * This is not a security hole, because the messages are still checked
     * proper hashing.
     */

    /* Check Msg padding */
    if (!NvBootUtilIsValidPadding(s_Msg.Padding, NVBOOT_RCM_MSG_PADDING_LENGTH))
    {
        HandleError(NvBootRcmResponse_BadMsgPadding);
        return NvBootError_ValidationFailure;
    }

    /* Check Data padding */
    PaddingStart = ADDR_TO_PTR(NVBOOT_RCM_DLOAD_BASE_ADDRESS +
                               s_Msg.PayloadLength);
    PaddingSize = s_Msg.LengthSecure - s_Msg.PayloadLength -
      sizeof(NvBootRcmMsg);

    if (!NvBootUtilIsValidPadding(PaddingStart, PaddingSize))
    {
        HandleError(NvBootRcmResponse_BadDataPadding);
        return NvBootError_ValidationFailure;
    }

    /* Command-specific checks */
    switch (s_Msg.Opcode)
    {
        case NvBootRcmOpcode_Sync:
        case NvBootRcmOpcode_QueryBootRomVersion:
        case NvBootRcmOpcode_QueryRcmVersion:
        case NvBootRcmOpcode_QueryBootDataVersion:
        case NvBootRcmOpcode_SetDebugFeatures:
            if (s_Msg.PayloadLength > 0)
            {
                HandleError(NvBootRcmResponse_PayloadTooLarge);
                return NvBootError_ValidationFailure;
            }
            break;

        case NvBootRcmOpcode_ProgramFuses:
        case NvBootRcmOpcode_VerifyFuses:
        case NvBootRcmOpcode_ProgramFuseArray:
        case NvBootRcmOpcode_VerifyFuseArray:
        case NvBootRcmOpcode_EnableJtag:
            // ProgramFuses ,  VerifyFuses, ProgramFuseArray & VerifyFuseArray
            // are not supported by  T30.
            // EnableJtag is deprecated as of T210.
            // Send a response but don't stall.
            return  SendResponse(NvBootRcmResponse_UnsupportedOpcode);

        case NvBootRcmOpcode_DownloadExecute:
            if (!IsEntryPointWithinPayload())
            {
                HandleError(NvBootRcmResponse_InvalidEntryPoint);
                return NvBootError_ValidationFailure;
            }
            break;

        default:
            /* Invalid opcode */
            HandleError(NvBootRcmResponse_InvalidOpcode);
            return NvBootError_ValidationFailure;
    }

    return NvBootError_Success;
}

/**
 * @retval NvBootError_Success The command executed correctly.
 * @retval Otherwise           There was an error.
 * @retval *AppletDownloaded = NV_TRUE An applet was successfully downloaded
 *                                     and should be run.
 * @retval *AppletDownloaded = NV_FALSE Otherwise.
 */
static NvBootError
Execute(NvBool *AppletDownloaded)
{
    NvBootError  e;
    NvBootECID  pUniqueId;

    *AppletDownloaded = NV_FALSE;

    switch (s_Msg.Opcode)
    {
        case NvBootRcmOpcode_Sync:
            NV_BOOT_CHECK_ERROR(SendResponse(NvBootRcmResponse_Success));
            break;

        // Fuse programing using RCM is not supported T30.
        // To avoid extra changes to the program logic, these commands
        // need to remain here as NOPs.
        case NvBootRcmOpcode_ProgramFuses:
        case NvBootRcmOpcode_VerifyFuses:
        case NvBootRcmOpcode_ProgramFuseArray:
        case NvBootRcmOpcode_VerifyFuseArray:
        case NvBootRcmOpcode_EnableJtag:
            // Do nothing.
            break;

        case NvBootRcmOpcode_DownloadExecute:
            NV_BOOT_CHECK_ERROR(SendResponse(NvBootRcmResponse_Success));
            *AppletDownloaded = NV_TRUE;
            break;

        case NvBootRcmOpcode_QueryBootRomVersion:
            NV_BOOT_CHECK_ERROR(SendResponse(NVBOOT_BOOTROM_VERSION));
            break;

        case NvBootRcmOpcode_QueryRcmVersion:
            NV_BOOT_CHECK_ERROR(SendResponse(NVBOOT_RCM_VERSION));
            break;

        case NvBootRcmOpcode_QueryBootDataVersion:
            NV_BOOT_CHECK_ERROR(SendResponse(NVBOOT_BOOTDATA_VERSION));
            break;

        case NvBootRcmOpcode_SetDebugFeatures:
            NvBootFuseGetUniqueId(&pUniqueId);

            if( (pUniqueId.ECID_0 == s_Msg.UniqueChipId.ECID_0) &&
                (pUniqueId.ECID_1 == s_Msg.UniqueChipId.ECID_1) &&
                (pUniqueId.ECID_2 == s_Msg.UniqueChipId.ECID_2) &&
                (pUniqueId.ECID_3 == s_Msg.UniqueChipId.ECID_3) )
            {
                s_State.SecureDebugControl = s_Msg.SecureDebugControl;
                NV_BOOT_CHECK_ERROR(SendResponse(NvBootRcmResponse_Success));
                break;
            }

            NV_BOOT_CHECK_ERROR(SendResponse(NvBootRcmResponse_ECIDMismatch));
            break;
        default:
            /* Illegal opcodes should be handled by Validate. */
            HandleError(NvBootRcmResponse_InvalidOpcode);
            return NvBootError_ValidationFailure;
    }

    return NvBootError_Success;
}

#endif // #if USE_FULL_RCM

static NvBootError ProcessMsgForSecureProvisioning(NvBootAes256Key *SecureProvisioningKey)
{
    NvBootError e;

    // First, check if we are in factory secure provisioning mode.
    e = NvBootFuseIsSecureProvisioningMode(s_Msg.SecProvisioningKeyNum_Insecure);

    if (e != NvBootError_SecProvisioningEnabled)
        return e;

    // Consider this as factory secure provisioning mode for now.
    // We still need to verify SecProvisioningKeyNum_Insecure == SecProvisioningKeyNum_Secure
    // later in Validate().
    s_State.FactorySecureProvisioningMode = NV_TRUE;
    s_State.ValidationKeySize = SE_MODE_PKT_AESMODE_KEY256;
    s_State.DecryptionKeySlotNum = NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt;
    s_State.CMACHashKeySlotNum = NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash;

    if(NvBootFuseGetSecureProvisioningIndexValidity() == NvBootError_ValidAntiCloningFuse)
    {
        s_State.ProvisioningKeyNum = NvBootFuseGetSecureProvisionIndex();
    }
    else
    {
        // Else we use the RCM header's _insecure value. No need to worry about
        // NvBootError_SecProvisioningInvalidAntiCloningKey, since we reached
        // this path after secure provisioning is confirmed enabled.
        s_State.ProvisioningKeyNum = s_Msg.SecProvisioningKeyNum_Insecure;
    }

    // If we are in Secure Provisioning Mode, we need to load the appropriate key
    // from IROM.
    // Use this provisioning key to validate: s_Msg.SecProvisioningKeyNum_Insecure
    //
    // Load the key number as specified in the RCM message header's insecure field
    // SecProvisioningKeyNum_Insecure.
    // Copy the key from IROM into IRAM first.
    NvBootUtilMemcpy(SecureProvisioningKey,
             (NvU8 *) NVBOOT_FACTORY_SECURE_PROVISIONING_KEYS_START + sizeof(NvBootAes256Key)*s_State.ProvisioningKeyNum,
             sizeof(NvBootAes256Key));

    if(s_State.ProvisioningKeyNum == NvBootSeAesSecProvisioningKey_KeyWrapKey)
    {
        // Load the Key Wrap Key from IROM into unique SE key slot for
        // decrypt operations.
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                          SE_MODE_PKT_AESMODE_KEY256,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                          &SecureProvisioningKey->Key[0]);

        // Initialize SE key slot OriginalIv[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                              0);

        // Initialize SE slot UpdatedIV[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                          SE_MODE_PKT_AESMODE_KEY128,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                          0);

        // Decrypt the encrypted user-specified AES provisioning key directly into
        // the SE key slots for decryption and CMAC hashing.
        NvBootSeAesDecryptKeyIntoKeySlot(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                                         SE_MODE_PKT_AESMODE_KEY256,
                                         NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                                         SE_MODE_PKT_AESMODE_KEY256,
                                         &(s_Msg.SecProvisioningKeyWrapKey.Key[0]));

        NvBootSeAesDecryptKeyIntoKeySlot(NvBootSeAesKeySlot_Secure_Provisioning_KeyWrapKey_Decrypt,
                                         SE_MODE_PKT_AESMODE_KEY256,
                                         NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                                         SE_MODE_PKT_AESMODE_KEY256,
                                         &(s_Msg.SecProvisioningKeyWrapKey.Key[0]));
    }
    else
    {

        // Load the provisioning key into SE key slot for decryption
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                          SE_MODE_PKT_AESMODE_KEY256,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                          &SecureProvisioningKey->Key[0]);

        // Load the provisioning key into SE key slot for CMAC hashing.
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                          SE_MODE_PKT_AESMODE_KEY256,
                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                          &SecureProvisioningKey->Key[0]);

    }
    // Initialize SE key slot OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                          0);

    // Initialize SE slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      0);

    // Initialize SE key slot OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                      0);

    // Initialize SE slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      0);

    return NvBootError_SecProvisioningEnabled;
}

/**
 * Note: AesEngines will already be set up w/the correct keys.
 * Note: IsForced is currently unused.
 */
NvBootError
NvBootRcm(NvBool IsForced, NvBool *IsFactorySecureProvisioning, NvU32 *RcmSecureDebugControl, NvU32 *EntryPoint)
{
#if USE_FULL_RCM
    NvBool      LaunchApplet = NV_FALSE;
    NvBootError e;
    NvU32       Sha256Digest[NVBOOT_SE_SHA256_LENGTH_WORDS];
    NvU8        PublicKeyHash[NVBOOT_SE_SHA256_LENGTH_BYTES];
    NvBootRCMPort_T   *pRcmPort = NvBootRcmGetPortHandle();
    // Offset to the start of the signed/hashed area of the message.
    const NvU32 Offset = (NvU8*)&(s_Msg.RandomAesBlock) - (NvU8*)&s_Msg;
    // Early versions of SE RSA HW did not unaligned word (32-bit) addresses;
    // Use aligned (4) attribute for now.
    NvBootSeRsaKey2048      PublicKey __attribute__ ((aligned (4)));
    NvBootAes256Key         SecureProvisioningKey __attribute__ ((aligned (4)));
#endif // #if USE_FULL_RCM

    /* Record the boot type & device (may overwrite cold boot values) */
    BootInfoTable.BootType = NvBootType_Recovery;

    s_State.FirstMessageProcessed = NV_FALSE;
    s_State.IsPublicKeyValidated = NV_FALSE;
    s_State.SecureDebugControl = APBDEV_PMC_DEBUG_AUTHENTICATION_0_SW_DEFAULT_VAL;
    s_State.FactorySecureProvisioningMode = NV_FALSE;
    s_State.ValidationKeySize = SE_MODE_PKT_AESMODE_KEY128;
    s_State.ProvisioningKeyNum = NvBootSeAesSecProvisioningKey_SecProvisiningDisabled;
    s_State.DecryptionKeySlotNum = NvBootSeAesKeySlot_SBK_AES_Decrypt;
    s_State.CMACHashKeySlotNum = NvBootSeAesKeySlot_SBK_AES_CMAC_Hash;

    // Create SE input linked list for SHA hashing purposes.
    // Two SE input buffers, zero based.
    s_SeInputLL.LastBufferNumber = 1;
    // First buffer is the signed section of the NvBootRcmMsg header.
    s_SeInputLL.LLElement[0].StartByteAddress = (NvU32) &(s_Msg.RandomAesBlock);
    s_SeInputLL.LLElement[0].BufferByteSize = sizeof(NvBootRcmMsg) - Offset;
    // Second buffer is the payload, if any.
    s_SeInputLL.LLElement[1].StartByteAddress = (NvU32) NVBOOT_RCM_DLOAD_BASE_ADDRESS;
    // Skip the setup of the second buffer size until the RCM message is
    // received.

#if USE_FULL_RCM
    NvBootFuseGetOperatingMode(&(s_State.RcmOpMode));
    if(pRcmPort->Context.Initialized != NV_TRUE)
        return NvBootError_NotInitialized;

    /* Main command processing loop. */
    while (1)
    {
        if (pRcmPort->Context.Connected == NV_FALSE)
        {
            s_State.BufferIndex = 0;

            //reload different timeout value for rcm
            NvBootWdtReload(WDT_TIMEOUT_VAL_RCM);
            if (pRcmPort->Connect(Buffer[s_State.BufferIndex]) != NvBootError_Success)
                HandleError(NvBootRcmResponse_UsbSetupFailed);

            /* Send the UniqueId in the clear. */
            e = SendUniqueId();
            if (e == NvBootError_EpNotConfigured)
            {
                // Device got reset from the Host, need to do reenumeration
                pRcmPort->Context.Connected = NV_FALSE;
                continue;
            }
            // Set Enumeration done flag
            pRcmPort->Context.Connected = NV_TRUE;
        }

        NV_BOOT_RCM_CHECK_ERROR(ReceiveMessage());

        // Finalize setup of second SE linked list buffer after the message
        // has been received and s_Msg.LengthInsecure is available.
        s_SeInputLL.LLElement[1].BufferByteSize = s_Msg.LengthInsecure - sizeof(NvBootRcmMsg);

        if(s_State.RcmOpMode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
        {
            if(s_State.IsPublicKeyValidated == NV_FALSE)
            {
                // Compute the SHA-256 hash the public key modulus.
                NvBootSeSHAHash(&s_Msg.Key.Modulus[0],
                                NVBOOT_SE_RSA_MODULUS_LENGTH_BYTES,
                                NULL,
                                &Sha256Digest[0],
                                SE_MODE_PKT_SHAMODE_SHA256);

                // Get the SHA-256 hash of the public key modulus from fuses.
                NvBootFuseGetPublicKeyHash(&PublicKeyHash[0]);

                // Verify the SHA-256 hash of the public key modulus against the fuses.
                if(NvBootUtilCompareBytes((NvU8 *) &Sha256Digest, PublicKeyHash, NVBOOT_SE_SHA256_LENGTH_BYTES) == NV_TRUE)
                {
                    s_State.IsPublicKeyValidated = NV_TRUE;

                     // Set up the public key in a format expected by the SE (modulus
                     // first, then exponent, in a contiguous buffer).
                    NvBootUtilMemcpy(&PublicKey.Modulus[0], &s_Msg.Key.Modulus[0], sizeof(NvBootRsaKeyModulus));
                    NvBootUtilMemset(&PublicKey.Exponent[0], 0, NVBOOT_SE_RSA_PUBLIC_KEY_EXPONENT_LENGTH_BYTES);
                    PublicKey.Exponent[0] = NVBOOT_SE_RSA_PUBLIC_KEY_EXPONENT;

                    // Load RSA key into the SE. Use slot 1.
                    NvBootSeRsaWriteKey((NvU32 *) &PublicKey,
                                         NVBOOT_SE_RSA_MODULUS_LENGTH_BITS,
                                         NVBOOT_SE_RSA_EXPONENT_LENGTH_BITS,
                                         SE_RSA_KEY_PKT_KEY_SLOT_ONE);
                }
            }

            // Only validate the message and payload if the public key has
            // been validated itself. Once the public key is validated, the
            // code will ignore any subsequent public key included the RCM
            // header and proceed directly to an RSASSA-PSS-VERIFY operation
            // of the RCM header and message with the RSA key in the SE.
            if(s_State.IsPublicKeyValidated == NV_TRUE)
            {
                // Hash the RCM message and payload.
                NvBootSeSHAHash((NvU32 *)&s_Msg.RandomAesBlock,
                                s_Msg.LengthInsecure - Offset,
                                (NvU32 *) &s_SeInputLL,
                                &Sha256Digest[0],
                                SE_MODE_PKT_SHAMODE_SHA256);

                // Verify the RCM message.
                e = NvBootSeRsaPssSignatureVerify(SE_RSA_KEY_PKT_KEY_SLOT_ONE,
                                                            NVBOOT_SE_RSA_MODULUS_LENGTH_BITS,
                                                            (NvU32 *)&s_Msg.RandomAesBlock,
                                                            &Sha256Digest[0],
                                                            s_Msg.LengthInsecure - Offset,
                                                            &s_Msg.Signature.RsaPssSig.Signature[0],
                                                            SE_MODE_PKT_SHAMODE_SHA256,
                                                            NVBOOT_SE_RSA_PSS_SALT_LENGTH_BYTES);
                if(e != NvBootError_Success)
                {
                    HandleError(NvBootRcmResponse_SignatureCheckFailed);
                    return NvBootError_ValidationFailure;
                }
            }
        }
        else
        {
            // The first message must be a secure provisioning message for
            // secure provisioning to take effect.
            // ProcessMsgForSecureProvisioning will check if this is indeed
            // a secure provisioning message.
            if(s_State.FirstMessageProcessed == NV_FALSE)
            {
                e = ProcessMsgForSecureProvisioning(&SecureProvisioningKey);
                // If an invalid key was sent as part of the RCM message, return
                // an error code and exit RCM.
                if(e == NvBootError_SecProvisioningInvalidKeyInput)
                {
                    HandleError(NvBootRcmResponse_SecProvisioningRcmInvalidKeyInput);
                    return e;
                }

            }

            ComputeHash();

            /* Only decrypt in ODM Secure mode or secure provisioning mode */
            if (s_State.RcmOpMode == NvBootFuseOperatingMode_OdmProductionSecureSBK ||
                s_State.FactorySecureProvisioningMode == NV_TRUE)
                Decrypt();
        }

        NV_BOOT_RCM_CHECK_ERROR(Validate());

        NV_BOOT_RCM_CHECK_ERROR(Execute(&LaunchApplet));
        if (LaunchApplet)
        {
            *RcmSecureDebugControl = s_State.SecureDebugControl;
            *IsFactorySecureProvisioning = s_State.FactorySecureProvisioningMode;
            *EntryPoint = s_Msg.Args.DownloadData.EntryPoint;
            return NvBootError_Success;
        }
        // Indicate that the first message has been successfully processed.
        s_State.FirstMessageProcessed = NV_TRUE;
    }

#else // #if USE_FULL_RCM

    /* Notify simulation environment that we entered Recovery Mode on
     * a platform that doesn't support it; then terminate simulation.
     *
     * Note:  Many tests end up triggering Recovery Mode, so it's very
     * important to know that we ended up here.
     *
     * NVBOOT_UTIL_TERMINATE_SIM never returns if NV_DEBUG is defined;
     * otherwise it becomes a no-op
     */

    NVBOOT_UTIL_TERMINATE_SIM(NvBootUtilSimStatus_RecoveryMode,
                              (NvU32)IsForced);
    return NvBootError_Unimplemented;

#endif // #if USE_FULL_RCM
}
