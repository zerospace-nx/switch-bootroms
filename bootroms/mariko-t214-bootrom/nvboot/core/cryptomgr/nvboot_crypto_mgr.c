/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvtypes.h"
#include "nvrm_drf.h"
#include "arfuse.h"
#include "arse.h"
#include "arapb_misc.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_config.h"
#include "project.h"
#include "nvboot_error.h"
#include "nvboot_se_aes.h"
#include "nvboot_crypto_fskp.h"
#include "nvboot_crypto_param.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_crypto_sec_irom.h"
#include "nvboot_crypto_fskp.h"
//#include "nvboot_sw_aes_int.h"
#include "nvboot_context_int.h"
#include "nvboot_se_int.h"
#include "nvboot_pka_int.h"
#include "nvboot_pka_ecc.h"
#include "nvboot_se_rsa.h"
#include "nvboot_util_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_address_int.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_bpmp_int.h"
#include "nvboot_rng_int.h"

// Not static for debugability.
NvBootCryptoMgrContext s_CryptoMgrContext;
NvBootAesDevMgr AesDevMgr;
NvBootRsaDevMgr RsaDevMgr;
NvBootShaDevMgr ShaDevMgr;

VT_CRYPTO_BUFFER NvBootCryptoMgrBuffers s_CryptoMgr_Buffers;
VT_CRYPTO_BUFFER NvBootAesKey256 FskpKeyBuf;

extern NvBootContext Context;

extern int32_t FI_counter1;

static void LoadKeyIVIntoSlot(uint8_t KeySlot, uint8_t KeySize, uint8_t *KeyData, uint8_t *IV, uint8_t *UIV)
{

    // Load 128-bit AES key to SE0.
    NvBootSeKeySlotWriteKeyIV(KeySlot,
            KeySize,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)KeyData);

    NvBootSeKeySlotWriteKeyIV(KeySlot,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
            (NvU32*)IV);

    // Initialize key slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(KeySlot,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
            (NvU32*)UIV);
}

static void LoadKeyIVIntoSlotSE2(uint8_t KeySlot, uint8_t KeySize, uint8_t *KeyData, uint8_t *IV, uint8_t *UIV)
{

    // Load 128-bit AES key to SE0.
    NvBootSe2KeySlotWriteKeyIV(KeySlot,
            KeySize,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)KeyData);

    NvBootSe2KeySlotWriteKeyIV(KeySlot,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
            (NvU32*)IV);

    // Initialize key slot UpdatedIV[127:0] to zero
    NvBootSe2KeySlotWriteKeyIV(KeySlot,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
            (NvU32*)UIV);
}

/**
 * Initialize crypto manager.
 * Tasks:
 *  - Detect chip security state.
 *
 */
NvBootError NvBootCryptoMgrInit()
{
    s_CryptoMgrContext.Status = CryptoMgrStatus_Idle;
    s_CryptoMgrContext.IsOemPcpValidated = false;
    s_CryptoMgrContext.IsOemBootBinaryHeaderAuthenticated = OEM_HEADER_NOT_AUTHENTICATED;

    // Initialize Authentication scheme to AES-CMAC.
    s_CryptoMgrContext.AuthenticationScheme = CryptoAlgo_AES_CMAC;
    // Initialize Encryption scheme to none.
    s_CryptoMgrContext.EncryptionScheme = CryptoAlgo_None;

    s_CryptoMgrContext.AesKeySize = AES_KEY_128;

    // Default to HW engines for operations.
    s_CryptoMgrContext.EngineForAuthentication = CryptoEngine_Se0_Aes0;
    s_CryptoMgrContext.EngineForEncryption = CryptoEngine_Se0_Aes0;

    // Detect chip security state first here.
    NvBootCryptoMgrSenseChipState();

    // Initialize crypto buffers.
    NvBootUtilMemset(&s_CryptoMgr_Buffers, 0, sizeof(NvBootCryptoMgrBuffers));

    // Initialize device managers to use SE engine.
    NvBootAesDevMgrInit(&AesDevMgr, NvBootAesDevice_SE0);
    NvBootRsaDevMgrInit(&RsaDevMgr, NvBootRsaDevice_SE0);
    NvBootShaDevMgrInit(&ShaDevMgr, NvBootShaDevice_SE0);

    // Initialize any buffers in the RsaSsaPss module.
    NvBootCryptoRsaSsaPssInit();

    // Setup SHA device with expected Hash function.
    ShaDevMgr.ShaDevMgrCallbacks->SetShaFamily(&ShaDevMgr.ShaConfig, SHA2);
    ShaDevMgr.ShaDevMgrCallbacks->SetDigestSize(&ShaDevMgr.ShaConfig, SHA_256);

    return NvBootError_Success;
}

void NvBootCryptoMgrSenseChipState()
{
    NvU32 RegData;

    // Detect authentication and confidentiality schemes.
    NvBootFuseGetBootSecurityAuthenticationInfo(&RegData);
    switch(RegData)
    {
        case FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_AES_CMAC:
            s_CryptoMgrContext.AuthenticationScheme = CryptoAlgo_AES_CMAC;
            s_CryptoMgrContext.EngineForAuthentication = CryptoEngine_Se0_Aes0;
            break;
        case FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA:
            s_CryptoMgrContext.AuthenticationScheme = CryptoAlgo_RSA_RSASSA_PSS;
            s_CryptoMgrContext.EngineForAuthentication = CryptoEngine_Se0_Pka0;
            break;
        default:
            s_CryptoMgrContext.AuthenticationScheme = CryptoAlgo_RSA_RSASSA_PSS;
            s_CryptoMgrContext.EngineForAuthentication = CryptoEngine_Se0_Pka0;
            break;

    }

    if(NvBootFuseBootSecurityIsEncryptionEnabled())
    {
        s_CryptoMgrContext.EncryptionScheme = CryptoAlgo_AES;
    }
}

void NvBootCryptoMgrHwEngineInit()
{
    NvBootError e = NvBootError_NotInitialized;
    NvBootSeInitializeSE();
    NvBootPkaHwInit();
    
    // Any error from the RNG is considered an attempt to glitch. Lock down and sit tight.
    e = NvBootRngInit();
    if(e != NvBootError_Success)
        do_exception();
}

NvBootCryptoMgrStatus NvBootCryptoMgrGetStatus()
{
    return s_CryptoMgrContext.Status;
}

void NvBootCryptoMgrSetStatus(NvBootCryptoMgrStatus Status)
{
    NV_ASSERT(Status < CryptoMgrStatus_Num);
    s_CryptoMgrContext.Status = Status;
}

NvBootCryptoAlgo NvBootCryptoMgrGetAuthScheme()
{
    return s_CryptoMgrContext.AuthenticationScheme;
}

NvBootError NvBootCryptoMgrSetOemPcp(NvBootPublicCryptoParameters *Pcp)
{
    // Don't load any PCP if we aren't in ECC or RSA authentication
    // mode.
    if(NvBootFuseIsPkcBootMode() == false)
    {
        return NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode;
    }
    // Skip loaded if already done.
    if(s_CryptoMgrContext.IsOemPcpValidated == true)
    {
        return NvBootError_Success;
    }

    ShaDevMgr.ShaDevMgrCallbacks->ShaHash((uint32_t *) Pcp,
                                          sizeof(NvBootPublicCryptoParameters),
                                          (uint32_t *) &s_CryptoMgr_Buffers.FusePcpHashBuf,
                                          &ShaDevMgr.ShaConfig);

    uint32_t FusePcpHash[NVBOOT_SHA256_LENGTH_WORDS];
    // Get SHA256 hash of Pcp in fuses.
    NvBootFuseGetPcpHash((uint8_t *) &FusePcpHash);

    FI_bool compare_result = FI_FALSE;
    compare_result = NvBootUtilCompareConstTimeFI(&s_CryptoMgr_Buffers.FusePcpHashBuf,
                                                &FusePcpHash,
                                                sizeof(s_CryptoMgr_Buffers.FusePcpHashBuf));

    if(compare_result == FI_FALSE)
    {
        return NvBootError_FuseHashMismatch;
    }

    // If validated successfully, copy Pcp into local buffer accessible by crypto engines.
    NvBootUtilMemcpy(&s_CryptoMgr_Buffers.Pcp, Pcp, sizeof(NvBootPublicCryptoParameters));

    // Validate the RSA key size is supported by this chip.
    bool is_valid_key_size = false;
    is_valid_key_size = RsaDevMgr.RsaDevMgrCallbacks->IsValidKeySize(s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey.KeySize);
    if(is_valid_key_size == false)
    {
        return NvBootError_Unsupported_RSA_Key_Size;
    }

    // Override public exponent with hardcoded value of 0x10001.
    NvBootUtilMemset(s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey.KeyData.RsaKeyMaxSizeSe0NvU32.Exponent,
                     0,
                     NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES);

    s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey.KeyData.RsaKeyMaxSizeNvU32.Exponent[0] = NVBOOT_SE_RSA_PUBLIC_KEY_EXPONENT;

    // Load public keys in the Pcp into appropriate engines.
    NvBootError e;
    e = RsaDevMgr.RsaDevMgrCallbacks->SetKey(&s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey,
                                         CRYPTOMGR_RSA_PUBLIC_KEY_SLOT);
    if(e != NvBootError_Success)
        return e;

    s_CryptoMgrContext.IsOemPcpValidated = true;

    return NvBootError_Success;
}

void NvBootCryptoMgrLoadPcpIntoEngines()
{

}

NvBool NvBootCryptoMgrIsOemPcpValidated()
{
    return s_CryptoMgrContext.IsOemPcpValidated;
}

void NvBootCryptoMgrOverrideEngineForAuth(NvBootCryptoEngine Engine)
{
    // Ignore any crypto scheme overrides if the PRODUCTION_MODE fuse is set.
    if(NvBootFuseIsNvProductionMode() == NV_TRUE)
    {
        return;
    }

    s_CryptoMgrContext.EngineForAuthentication = Engine;
}

static void NvBootCryptoMgrCopyArrayFromRegToBuffer(uint8_t* Buf, NvU32 RegAddr, NvU32 Size)
{
    NvU32 * Buf32 = (NvU32*)Buf;
    while(Size) {
        *Buf32 = NV_READ32(RegAddr);

        Buf32 ++;
        RegAddr += sizeof(NvU32);
        if(Size <= sizeof(NvU32))
           break;
        Size -= sizeof(NvU32);
    }
}

#define TEST_SEC_KEYS 0

#if TEST_SEC_KEYS
#define NVBOOT_SEC_IROM_TEST_OFFSET 1024 * 116
#define NVBOOT_SEC_IROM_TEST_START (NV_ADDRESS_MAP_BPMP_BOOTROM_BASE + NVBOOT_SEC_IROM_TEST_OFFSET)
#define NVBOOT_SEC_IROM_TEST_SIG "BR_SKEYS_TST"

static void NvBootCryptoMgrKeyTestGoToDie(NvU32 e)
{
    NvU32 * write_to_memory = (NvU32*)NVBOOT_SEC_IROM_TEST_START;
    *write_to_memory = e;
    __asm__("b .");
}

static NvBootError NvBootCryptoMgrKeyTestRsaVerif(uint8_t* mod, uint8_t* sig, uint8_t* data_s, NvU32 data_len)
{
    uint8_t exp[NVBOOT_RSA_2048_MODULUS_SIZE_BYTES];

    NvBootUtilMemset(exp, 0, NVBOOT_RSA_2048_MODULUS_SIZE_BYTES);

    NvBootSeRsaWriteKey((NvU32*)mod,
                        NVBOOT_RSA_MAX_MODULUS_SIZE_BITS,
                        NvBootSeRsaKeySlot_OEM_Key_Verify_Key_Slot,
                        SE_RSA_KEY_PKT_EXPMOD_SEL_MODULUS);

    // Set exponent to fixed value of 0x10001.
    *(NvU32*)exp = NVBOOT_SE_RSA_PUBLIC_KEY_EXPONENT;

    NvBootSeRsaWriteKey((NvU32*)exp,
                        NVBOOT_RSA_MAX_EXPONENT_SIZE_BITS,
                        NvBootSeRsaKeySlot_OEM_Key_Verify_Key_Slot,
                        SE_RSA_KEY_PKT_EXPMOD_SEL_EXPONENT);

    uint8_t * sig_s = (uint8_t*)NVBOOT_CRYPTO_MGR_BUF_KEY_DECRYPTION_REGION_R5;

    NvBootUtilMemcpy(sig_s, sig, NVBOOT_RSA_2048_MODULUS_SIZE_BYTES);
    //reverse it, temp fix
//    NvU32 i = 0;
//    uint8_t t;
//    for(;i<0x100;i++) {
//        sig_s[i] = sig[0xFF-i];
//    }

    NvBootError e = NvBootSeRsaPssSignatureVerify(NvBootSeRsaKeySlot_OEM_Key_Verify_Key_Slot,
                                              NVBOOT_RSA_MAX_MODULUS_SIZE_BITS,
                                              (NvU32 *) data_s,
                                              NULL,
                                              data_len,
                                              (NvU32 *) sig_s,
                                              NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_HASH_ALGORITHM,
                                              NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SLEN,
                                              (NvU32*)mod);

    return e;
}

void NvBootCryptoMgrKeyTest()
{
    uint8_t * test_blob = (uint8_t*)NVBOOT_SEC_IROM_TEST_START;

    //test signature
    if(NvBootUtilCompareBytes(test_blob, NVBOOT_SEC_IROM_TEST_SIG, sizeof(NVBOOT_SEC_IROM_TEST_SIG)-1) == NV_FALSE)
    {
        NvBootCryptoMgrKeyTestGoToDie(0x11110001);
    }
    test_blob += sizeof(NVBOOT_SEC_IROM_TEST_SIG)-1;

    //get known text size
    NvU32 known_text_size = *(NvU32*)test_blob;
    test_blob += sizeof(NvU32);

    //get known text
    uint8_t * known_text = (uint8_t *)NVBOOT_CRYPTO_MGR_BUF_KEY_DECRYPTION_REGION_R5 + 0x1000;
    uint8_t * cipher_text = (uint8_t *)NVBOOT_CRYPTO_MGR_BUF_KEY_DECRYPTION_REGION_R5;
    NvBootUtilMemcpy(known_text, test_blob, known_text_size);
    test_blob += known_text_size;

    //check all fspk
    //all fspk temp goes to slot 1
    //nv fek still in se
    //uint8_t FSPK[0x20];
    uint8_t i=0;
    for(;i<64;i++)
    {
        NvBootCryptoMgrLoadFSKP(i, NvBootSeAesKeySlot_1);

//        NvBootSeKeySlotReadKeyIV(NvBootSeAesKeySlot_1,
//                SE_MODE_PKT_AESMODE_KEY256,
//                SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
//                (NvU32*)FSPK);

        NvBootSeAesEncrypt(
                NvBootSeAesKeySlot_1,
                SE_MODE_PKT_AESMODE_KEY256,
                NV_TRUE,
                known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
                SimpleAddressTranslateBpmpToClient(known_text),
                SimpleAddressTranslateBpmpToClient(cipher_text));
        if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
            NvBootCryptoMgrKeyTestGoToDie(0x22220000 + i);
        test_blob += known_text_size;
    }

    //check the rest
    //mts
    NvBootSeAesEncrypt(
            NvBootSeAesKeySlot_MTS_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(known_text),
            SimpleAddressTranslateBpmpToClient(cipher_text));
    if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
        NvBootCryptoMgrKeyTestGoToDie(0x33330001);
    test_blob += known_text_size;
    //bpmp
    NvBootSeAesEncrypt(
            NvBootSeAesKeySlot_BPMP_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(known_text),
            SimpleAddressTranslateBpmpToClient(cipher_text));
    if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
        NvBootCryptoMgrKeyTestGoToDie(0x33330002);
    test_blob += known_text_size;
    //spe
    NvBootSeAesEncrypt(
            NvBootSeAesKeySlot_SPE_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(known_text),
            SimpleAddressTranslateBpmpToClient(cipher_text));
    if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
        NvBootCryptoMgrKeyTestGoToDie(0x33330003);
    test_blob += known_text_size;
    //ape
    NvBootSeAesEncrypt(
            NvBootSeAesKeySlot_APE_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(known_text),
            SimpleAddressTranslateBpmpToClient(cipher_text));
    if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
        NvBootCryptoMgrKeyTestGoToDie(0x33330004);
    test_blob += known_text_size;
    //sce
    NvBootSeAesEncrypt(
            NvBootSeAesKeySlot_SCE_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(known_text),
            SimpleAddressTranslateBpmpToClient(cipher_text));
    if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
        NvBootCryptoMgrKeyTestGoToDie(0x33330005);
    test_blob += known_text_size;
    //mb1
    NvBootSeAesEncrypt(
            NvBootSeAesKeySlot_MB1_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            known_text_size/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(known_text),
            SimpleAddressTranslateBpmpToClient(cipher_text));
    if(NvBootUtilCompareBytes(test_blob, cipher_text, known_text_size) == NV_FALSE)
        NvBootCryptoMgrKeyTestGoToDie(0x33330006);
    test_blob += known_text_size;

    NvBootSecureIromLayout *KeysLayout = (NvBootSecureIromLayout *)(NVBOOT_SEC_IROM_START);
    //rsa time
    if(NvBootCryptoMgrKeyTestRsaVerif(KeysLayout->BPMP_AK, test_blob, known_text, known_text_size) != NvBootError_Success)
        NvBootCryptoMgrKeyTestGoToDie(0x44440001);
    test_blob += 0x100;

    if(NvBootCryptoMgrKeyTestRsaVerif(KeysLayout->SPE_AK, test_blob, known_text, known_text_size) != NvBootError_Success)
        NvBootCryptoMgrKeyTestGoToDie(0x44440002);
    test_blob += 0x100;

    if(NvBootCryptoMgrKeyTestRsaVerif(KeysLayout->APE_AK, test_blob, known_text, known_text_size) != NvBootError_Success)
        NvBootCryptoMgrKeyTestGoToDie(0x44440003);
    test_blob += 0x100;

    if(NvBootCryptoMgrKeyTestRsaVerif(KeysLayout->SCE_AK, test_blob, known_text, known_text_size) != NvBootError_Success)
        NvBootCryptoMgrKeyTestGoToDie(0x44440004);
    test_blob += 0x100;

    if(NvBootCryptoMgrKeyTestRsaVerif(KeysLayout->MTS_AK, test_blob, known_text, known_text_size) != NvBootError_Success)
        NvBootCryptoMgrKeyTestGoToDie(0x44440005);
    test_blob += 0x100;

    if(NvBootCryptoMgrKeyTestRsaVerif(KeysLayout->MB1_AK, test_blob, known_text, known_text_size) != NvBootError_Success)
        NvBootCryptoMgrKeyTestGoToDie(0x44440006);
    test_blob += 0x100;

    //die a successful death
    NvBootCryptoMgrKeyTestGoToDie(0x99999999);
}
#endif

#define TEST_NV_SIGN 0
#if TEST_NV_SIGN

#define NVBOOT_SEC_IROM_TEST_OFFSET 0x0D490000
#define NVBOOT_SEC_IROM_TEST_2_OFFSET 0x0D494000
#define NVBOOT_SEC_IROM_TEST_SYSRAM 0x40020000

static void NvBootCryptoMgrSignTestGoToDie(NvU32 e)
{
    NvU32 * write_to_memory = (NvU32*)NVBOOT_SEC_IROM_TEST_OFFSET;
    *write_to_memory = e;
    __asm__("b .");
}

void NvBootTestNvSign(void)
{
    NvBinarySignHeader * header = (NvBinarySignHeader*)(uint8_t*)NVBOOT_SEC_IROM_TEST_OFFSET;

    NvBootUtilMemcpy((uint8_t*)NVBOOT_SEC_IROM_TEST_SYSRAM, header, header->BinaryLength + sizeof(NvBinarySignHeader));
    header = (NvBinarySignHeader*)(uint8_t*)NVBOOT_SEC_IROM_TEST_SYSRAM;
    uint8_t * bin_offset = (uint8_t*)NVBOOT_SEC_IROM_TEST_SYSRAM + sizeof(NvBinarySignHeader);

    NvBootError e = NvBootSeRsaPssSignatureVerify(NvBootSeRsaKeySlot_NV_Key_Verify_Key_Slot,
                                              NVBOOT_RSA_MAX_MODULUS_SIZE_BITS,
                                              (NvU32 *) header->BinaryMagic,
                                              NULL,
                                              header->BinaryLength + 0x10,
                                              header->Signatures.RsaSsaPssSig.RsaSsaPssSigNvU32.RsaSsaPssSig,
                                              NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_HASH_ALGORITHM,
                                              NVBOOT_RSA_PSS_SIGNATURE_DEFAULT_SLEN,
                                              (NvU32*)pCryptoMgrContext->NvRsaPublicKey.RsaKeyMaxSizeNvU32.Modulus);
    if(e != NvBootError_Success)
        NvBootCryptoMgrSignTestGoToDie(0x01010101);

    NvBootSeAesDecrypt(
            NvBootSeAesKeySlot_MB1_Decryption_Key,
            SE_MODE_PKT_AESMODE_KEY128,
            NV_TRUE,
            header->BinaryLength/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
            SimpleAddressTranslateBpmpToClient(bin_offset),
            SimpleAddressTranslateBpmpToClient(bin_offset + header->BinaryLength));

    if( NvBootUtilCompareBytes(bin_offset+header->BinaryLength, (uint8_t*)NVBOOT_SEC_IROM_TEST_2_OFFSET, header->BinaryLength) == NV_FALSE)
        NvBootCryptoMgrSignTestGoToDie(0x02020202);

    NvBootCryptoMgrSignTestGoToDie(0x99999999);
}
#endif

static void DecrementReadyEncKeyCounter(void)
{
    FI_counter1 -= READY_ENC_KEY_STEPS*COUNTER1;
    if(NvBootFuseIsOemFuseEncryptionEnabled())
        FI_counter1 -= READY_ENC_KEY_FUSE_ENCRYPTION_STEPS*COUNTER1;
}
static void DecrementLoadOemAesKeysCounter(void)
{
    FI_counter1 -= LOAD_OEM_AES_KEY_STEPS*COUNTER1;
}
NvBootError NvBootCryptoMgrDecKeys()
{
    NvBootError e;
    FI_counter1 = 0;
    e = NvBootCryptoMgrReadyEncKey();
    DecrementReadyEncKeyCounter();
    if(FI_counter1 != 0)
        do_exception();
    if(e != NvBootError_Success)
        return e;

    FI_counter1 = 0;
    e = NvBootCryptoMgrLoadOemAesKeys();
    DecrementLoadOemAesKeysCounter();
    if(FI_counter1 != 0)
        do_exception();
    if(e != NvBootError_Success)
        return e;
#if 0
    e = NvBootCryptoMgrLoadNvKeys();
    if(e != NvBootError_Success)
        return e;
#endif

#if TEST_SEC_KEYS
    NvBootCryptoMgrKeyTest();
#endif

#if TEST_NV_SIGN
    NvBootTestNvSign();
#endif

    return NvBootError_Success;
}

NvBootError NvBootCryptoMgrReadyEncKey()
{
    /**
     *      Need NV FEK for loading FSKP and Default SE/SE2.
     *      Secure provisioning is enabled only in production systems 
     */
    
    uint8_t NvAesFuseDecryptionKey[NVBOOT_SE_AES_KEY128_LENGTH_BYTES] __attribute__((aligned(4)));
    uint32_t NvKeyRegAddr;
    
    //NV KEY. Load NvKey0 for Bank0, NvKey1 for Bank1
    //load FEK and clear out the IRAM right away
    if(NvBootFuseGetOemFekBankSelect()==FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_BANK_0)
    {
        NvKeyRegAddr = NV_ADDRESS_MAP_APB_MISC_BASE + FEK_FUSEROMENCRYPTIONNVKEY0_0_0;
    }
    else
    {
        NvKeyRegAddr = NV_ADDRESS_MAP_APB_MISC_BASE + FEK_FUSEROMENCRYPTIONNVKEY1_0_0;
    }
    FI_counter1 += COUNTER1;

    // Read lock SE key slot before loading.
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_NV_FEK, NvBootSeInstance_Se1);
    FI_counter1 += COUNTER1;
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_NV_FEK, NvBootSeInstance_Se2);
    FI_counter1 += COUNTER1;

    // Load NV FEK into SE and SE2, slot 0
    NvBootCryptoMgrCopyArrayFromRegToBuffer(NvAesFuseDecryptionKey, NvKeyRegAddr, NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
    // Load into SE, Slot 0
    LoadKeyIVIntoSlot(AES_DEVICE_KEYSLOT_NV_FEK, SE_MODE_PKT_AESMODE_KEY128, NvAesFuseDecryptionKey, 0, 0);
    FI_counter1 += COUNTER1;

    // Load into SE2, Slot 0
    LoadKeyIVIntoSlotSE2(AES_DEVICE_KEYSLOT_NV_FEK, SE_MODE_PKT_AESMODE_KEY128, NvAesFuseDecryptionKey, 0, 0);
    FI_counter1 += COUNTER1;

    NvBootUtilMemset(NvAesFuseDecryptionKey, 0, NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
    FI_counter1 += COUNTER1;

    //OEM KEY
    //need to decide which one to load
    if(NvBootFuseIsOemFuseEncryptionEnabled() )
    {
        //key selection is an offset mode,
        //start from armiscreg offset 0x1080 is the test key and every value increase
        //in fuse register increase 1 key slot (0x10 byte) in register space
        //starting from test key
        
        uint32_t KeyAddress;
        if(NvBootFuseGetOemFekBankSelect()==FUSE_RESERVED_PRODUCTION_0_OEM_FEK_BANK_SELECT_BANK_0)
            KeyAddress = NV_ADDRESS_MAP_APB_MISC_BASE + FEK_FUSEROMENCRYPTIONTESTKEY0_0_0;
        else
            KeyAddress = NV_ADDRESS_MAP_APB_MISC_BASE + FEK_FUSEROMENCRYPTIONTESTKEY1_0_0;
        KeyAddress = KeyAddress + (NvBootFuseGetFuseDecryptionKeySelection() * NVBOOT_SE_AES_KEY128_LENGTH_BYTES);

        // Read lock SE key slot before loading.
        NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_OEM_FEK, NvBootSeInstance_Se1);
        FI_counter1 += COUNTER1;
        NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_OEM_FEK, NvBootSeInstance_Se2);
        FI_counter1 += COUNTER1;

        //load FEK and clear out the IRAM right away
        NvBootCryptoMgrCopyArrayFromRegToBuffer(NvAesFuseDecryptionKey, KeyAddress, NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
        // Load OEM FEK into SE.
        LoadKeyIVIntoSlot(AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128, NvAesFuseDecryptionKey, 0, 0);
        FI_counter1 += COUNTER1;

        // Load OEM FEK into SE2.
        LoadKeyIVIntoSlotSE2(AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128, NvAesFuseDecryptionKey, 0, 0);
        FI_counter1 += COUNTER1;

        NvBootUtilMemset(NvAesFuseDecryptionKey, 0, NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
        FI_counter1 += COUNTER1;
    }


    // lock FEK right away after loaded into key slot.
    NvU32 MiscLock = NV_DRF_NUM(APB_MISC, PP_FEK_RD_DIS, FEK_RD_DIS, 1);
    NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_FEK_RD_DIS_0, MiscLock);
    FI_counter1 += COUNTER1;

    return NvBootError_Success;
}

NvBootError NvBootCryptoMgrLoadOemAesKeys()
{
    // Decryption Buffer for AES keys
    NvU8 AesKeyDecryptionBuffer[128] __attribute__((aligned(4)));

    // Disable key slot read locks before loading.
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_KEK, NvBootSeInstance_Se1);
    FI_counter1 += COUNTER1;
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_KEK, NvBootSeInstance_Se2);
    FI_counter1 += COUNTER1;
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_SBK, NvBootSeInstance_Se1);
    FI_counter1 += COUNTER1;
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_SBK, NvBootSeInstance_Se2);
    FI_counter1 += COUNTER1;
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_BEK, NvBootSeInstance_Se1);
    FI_counter1 += COUNTER1;
    NvBootSeDisableAesKeySlotRead(AES_DEVICE_KEYSLOT_BEK, NvBootSeInstance_Se2);
    FI_counter1 += COUNTER1;

    // OEM assets are SBK, KEK, BEK which could optionally be encrypted.
    if( NvBootFuseIsOemFuseEncryptionEnabled() )
    {
        //decrypt each key
        //KEK0
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
                NV_ADDRESS_MAP_FUSE_BASE + FUSE_KEK00_0,
                NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
        // Decrypt into SE1 instance
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(
                NvBootSeInstance_Se1,
                AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128,
                AES_DEVICE_KEYSLOT_KEK, SE_MODE_PKT_AESMODE_KEY128,
                (uint8_t*)AesKeyDecryptionBuffer);
        NvBootSeInstanceKeySlotWriteKeyIV( NvBootSeInstance_Se1,
        AES_DEVICE_KEYSLOT_KEK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
        
        // Decrypt into SE2 instance
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(
                NvBootSeInstance_Se2,
                AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128,
                AES_DEVICE_KEYSLOT_KEK, SE_MODE_PKT_AESMODE_KEY128,
                (uint8_t*)AesKeyDecryptionBuffer);
        NvBootSeInstanceKeySlotWriteKeyIV( NvBootSeInstance_Se2,
        AES_DEVICE_KEYSLOT_KEK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));
        
        //SBK
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
                NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY0_0,
                NVBOOT_SE_AES_KEY128_LENGTH_BYTES);

        // Decrypt into SE1 instance
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(
                NvBootSeInstance_Se1,
                AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128,
                AES_DEVICE_KEYSLOT_SBK, SE_MODE_PKT_AESMODE_KEY128,
                (uint8_t*)AesKeyDecryptionBuffer);
        NvBootSeInstanceKeySlotWriteKeyIV( NvBootSeInstance_Se1,
        AES_DEVICE_KEYSLOT_SBK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
        
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(
                NvBootSeInstance_Se2,
                AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128,
                AES_DEVICE_KEYSLOT_SBK, SE_MODE_PKT_AESMODE_KEY128,
                (uint8_t*)AesKeyDecryptionBuffer);
        NvBootSeInstanceKeySlotWriteKeyIV( NvBootSeInstance_Se2,
        AES_DEVICE_KEYSLOT_SBK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
        
        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));
        
        //BEK
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
                NV_ADDRESS_MAP_FUSE_BASE + FUSE_BEK0_0,
                NVBOOT_SE_AES_KEY128_LENGTH_BYTES);

        // Decrypt into SE1
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(
                NvBootSeInstance_Se1,
                AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128,
                AES_DEVICE_KEYSLOT_BEK, SE_MODE_PKT_AESMODE_KEY128,
                (uint8_t*)AesKeyDecryptionBuffer);
        NvBootSeInstanceKeySlotWriteKeyIV( NvBootSeInstance_Se1,
        AES_DEVICE_KEYSLOT_BEK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
        
        // Decrypt into SE2
        NvBootSeInstanceAesDecryptKeyIntoKeySlot(
                NvBootSeInstance_Se2,
                AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY128,
                AES_DEVICE_KEYSLOT_BEK, SE_MODE_PKT_AESMODE_KEY128,
                (uint8_t*)AesKeyDecryptionBuffer);
        NvBootSeInstanceKeySlotWriteKeyIV( NvBootSeInstance_Se2,
        AES_DEVICE_KEYSLOT_BEK, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
        
        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));

        // Best practice is to clear out the assets as soon as it is no
        // longer needed, so clear OEM FEKs from SE here. Note OEM FEK is 128-bits, but
        // we zero out all 256-bits of the key slot here.
        LoadKeyIVIntoSlot(AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY256, 0, 0, 0);
        LoadKeyIVIntoSlotSE2(AES_DEVICE_KEYSLOT_OEM_FEK, SE_MODE_PKT_AESMODE_KEY256, 0, 0, 0);
    }
    else //no fuse encryption
    {
        //KEK0
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
                NV_ADDRESS_MAP_FUSE_BASE + FUSE_KEK00_0,
                NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
        // Write SEL_KEY, IV=0, UIV=0
        LoadKeyIVIntoSlot(AES_DEVICE_KEYSLOT_KEK, SE_MODE_PKT_AESMODE_KEY128, (uint8_t*)AesKeyDecryptionBuffer, 0, 0);
        
        LoadKeyIVIntoSlotSE2(AES_DEVICE_KEYSLOT_KEK, SE_MODE_PKT_AESMODE_KEY128, (uint8_t*)AesKeyDecryptionBuffer, 0, 0);
        
        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));

        // SBK
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
                NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY0_0,
                NVBOOT_SE_AES_KEY128_LENGTH_BYTES);

        // Write SEL_KEY, IV=0, UIV=0
        LoadKeyIVIntoSlot(AES_DEVICE_KEYSLOT_SBK, SE_MODE_PKT_AESMODE_KEY128, (uint8_t*)AesKeyDecryptionBuffer, 0, 0);
        
        LoadKeyIVIntoSlotSE2(AES_DEVICE_KEYSLOT_SBK, SE_MODE_PKT_AESMODE_KEY128, (uint8_t*)AesKeyDecryptionBuffer, 0, 0);

        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));
        
        // BEK
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
                NV_ADDRESS_MAP_FUSE_BASE + FUSE_BEK0_0,
                NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
        // Write SEL_KEY, IV=0, UIV=0
        LoadKeyIVIntoSlot(AES_DEVICE_KEYSLOT_BEK, SE_MODE_PKT_AESMODE_KEY128, (uint8_t*)AesKeyDecryptionBuffer, 0, 0);
        
        LoadKeyIVIntoSlotSE2(AES_DEVICE_KEYSLOT_BEK, SE_MODE_PKT_AESMODE_KEY128, (uint8_t*)AesKeyDecryptionBuffer, 0, 0);
        
        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));
    }
    FI_counter1 += COUNTER1;

#if TEST_SEC_KEYS
    uint8_t KEK_readback[NVBOOT_SE_AES_KEY128_LENGTH_BYTES];
    uint8_t SBK_readback[NVBOOT_SE_AES_KEY128_LENGTH_BYTES];
    uint8_t BEK_readback[NVBOOT_SE_AES_KEY128_LENGTH_BYTES];
    
    NvBootSeKeySlotReadKeyIV(AES_DEVICE_KEYSLOT_KEK,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)KEK_readback);
    NvBootSeKeySlotReadKeyIV(AES_DEVICE_KEYSLOT_SBK,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)SBK_readback);
    NvBootSeKeySlotReadKeyIV(AES_DEVICE_KEYSLOT_BEK,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)BEK_readback);
#endif

    FI_counter1 += COUNTER1;

    //clear out buffer.
    NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));
    FI_counter1 += COUNTER1;
    return NvBootError_Success;
}

void NvBootCryptoMgrLoadOemKEK1As256b()
{
}

void NvBootCryptoMgrLoadFSKP(uint8_t KeySelection, uint8_t KeySlot)
{
    //declare structure point to secure IROM copy in secure IROM
    NvBootSecureIromLayout *KeysLayout = (NvBootSecureIromLayout *)(NVBOOT_SEC_IROM_START);

    //copy secure IROM to SYSRAM
    NvBootUtilMemcpy((uint8_t*) &FskpKeyBuf, &KeysLayout->FSPK[KeySelection], sizeof(FskpKeyBuf));

    //decrypt key into slot
    NvBootSeAesDecryptKeyIntoKeySlot(
            AES_DEVICE_KEYSLOT_NV_FEK, SE_MODE_PKT_AESMODE_KEY128,
            KeySlot, SE_MODE_PKT_AESMODE_KEY256,
            (uint8_t*)&FskpKeyBuf);
    // clear out IV in key slot.
    NvBootSeKeySlotWriteKeyIV(KeySlot, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    //clear out the temporary buffer
    NvBootUtilMemset((uint8_t*)&FskpKeyBuf, 0, sizeof(FskpKeyBuf));
}

NvBootError NvBootCryptoMgrAuthBct(const NvBootConfigTable *Bct)
{
    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_RSA_RSASSA_PSS)
    {
        // Setup RSASSA-PSS context.
        s_CryptoMgr_Buffers.RsaPssContext.RsaKeySlot = CRYPTOMGR_RSA_PUBLIC_KEY_SLOT;
        s_CryptoMgr_Buffers.RsaPssContext.RsaKey = &s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageIsHashed = false;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessage = (uint32_t *) ((uint32_t) Bct +  OFFSET_BCT_SIGNED_SECT(Bct));
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageLengthBytes = SIZE_BCT_SIGNED_SECT(Bct);
        s_CryptoMgr_Buffers.RsaPssContext.InputSignature = (NvBootCryptoRsaSsaPssSig *) &Bct->Signatures.RsaSsaPssSig;

        e = NvBootCryptoRsaSsaPssVerify(&s_CryptoMgr_Buffers.RsaPssContext, &ShaDevMgr, &RsaDevMgr);
        return e;
    }
    else if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_AES_CMAC)
    {
        // Setup AES-CMAC context.
        s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
        s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
        s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) ((uint32_t) Bct + OFFSET_BCT_SIGNED_SECT(Bct));
        s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
        s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_128;
        s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(SIZE_BCT_SIGNED_SECT(Bct), NVBOOT_AES_BLOCK_LENGTH_BYTES);
        s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_SBK;

        // Generate subkey.
        AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                        s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK2);

        // Calcaulte AES-CMAC hash.
        AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

        FI_bool compare_result = FI_FALSE;
        compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &Bct->Signatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        if(compare_result == FI_TRUE)
            return NvBootError_Success;
        else
            return NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;
    }
    else
    {
        return NvBootError_CryptoMgr_InvalidAuthScheme;
    }
}

NvBootError NvBootCryptoMgrDecryptBct(NvBootConfigTable *Bct)
{
    if (s_CryptoMgrContext.EncryptionScheme != CryptoAlgo_AES)
    {
        // return success if BCT encryption is disabled.
        return NvBootError_Success;
    }

    // BCT location is fixed via linker script. Allow input of BCT as pointer
    // for unit testing.
    uint8_t * start_decrypt_address = (uint8_t *) ((uint32_t) Bct + OFFSET_BCT_ENCRYPTED_SECT(Bct));
    uint8_t * decrypt_output_address = (uint8_t *) ((uint32_t) Bct + OFFSET_BCT_ENCRYPTED_SECT(Bct));
    size_t bct_encrypted_section_size = SIZE_BCT_ENCRYPTED_SECT(Bct);

    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    e = AesDevMgr.AesDevMgrCallbacks->AesDecryptCBC(start_decrypt_address,
                                                decrypt_output_address,
                                                AES_DEVICE_KEYSLOT_BEK,
                                                s_CryptoMgrContext.AesKeySize,
                                                bct_encrypted_section_size);
    return e;
}

NvBootError NvBootCryptoMgrAuthOemBootBinaryHeader(const NvBootOemBootBinaryHeader *OemHeader)
{
    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_RSA_RSASSA_PSS)
    {
        // Setup RSASSA-PSS context.
        s_CryptoMgr_Buffers.RsaPssContext.RsaKeySlot = CRYPTOMGR_RSA_PUBLIC_KEY_SLOT;
        s_CryptoMgr_Buffers.RsaPssContext.RsaKey = &s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageIsHashed = false;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessage = (uint32_t *) ((uint32_t) OemHeader +  OFFSET_OEMBOOT_SIGNED_SECT(OemHeader));
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageLengthBytes = SIZE_OEMBOOT_SIGNED_SECT(OemHeader);
        s_CryptoMgr_Buffers.RsaPssContext.InputSignature = (NvBootCryptoRsaSsaPssSig *) &OemHeader->OemSignatures.RsaSsaPssSig;
        // Run RSASSA-PSS verify.
        e = NvBootCryptoRsaSsaPssVerify(&s_CryptoMgr_Buffers.RsaPssContext, &ShaDevMgr, &RsaDevMgr);
    }
    else if (s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_AES_CMAC)
    {
        // Setup AES cmac context.
        s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
        s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
        s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) ((uint32_t) OemHeader +  OFFSET_OEMBOOT_SIGNED_SECT(OemHeader));
        s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
        s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_128;
        s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(SIZE_OEMBOOT_SIGNED_SECT(OemHeader), NVBOOT_AES_BLOCK_LENGTH_BYTES);
        s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_SBK;

        // Generate subkey.
        AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                        s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK2);

        // Calculate AES-CMAC hash.
        AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

        FI_bool compare_result = FI_FALSE;
        compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &OemHeader->OemSignatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        if(compare_result == FI_TRUE)
            e = NvBootError_Success;
        else
            e = NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;
    }
    else if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_FSKP_CMAC)
    {
        // Setup AES cmac context.
        s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
        s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
        s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) ((uint32_t) OemHeader +  OFFSET_OEMBOOT_SIGNED_SECT(OemHeader));
        s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
        s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_256;
        s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(SIZE_OEMBOOT_SIGNED_SECT(OemHeader), NVBOOT_AES_BLOCK_LENGTH_BYTES);
        s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE;

        // Generate subkey.
        AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                        s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK2);

        // Calculate AES-CMAC hash.
        AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

        FI_bool compare_result = FI_FALSE;
        compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &OemHeader->OemSignatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        if(compare_result == FI_TRUE)
            e = NvBootError_Success;
        else
            e = NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;
    }
    else
    {
        return NvBootError_CryptoMgr_InvalidAuthScheme;
    }

    if(e == NvBootError_Success)
    {
        // If successfully authenticated, save the OemBootBinaryHash for future
        // comparison once BL1 is read from secondary storage.
        NvBootUtilMemcpy(&s_CryptoMgr_Buffers.OemBootBinaryHash, &OemHeader->OemBootBinaryHash, sizeof(NvBootSha256HashDigest));
        s_CryptoMgrContext.IsOemBootBinaryHeaderAuthenticated = OEM_HEADER_AUTHENTICATED;
        return e;
    }
    else
    {
        return e;
    }
}

NvBootError NvBootCryptoMgrAuthNvBinary(const NvBinarySignHeader *NvHeader)
{
    (void)NvHeader;
    return NvBootError_Success;
}

NvBootError NvBootCryptoMgrAuthBlPackage(const NvBootOemBootBinaryHeader *OemHeader, uint32_t *BlBinary)
{
    if(s_CryptoMgrContext.IsOemBootBinaryHeaderAuthenticated != OEM_HEADER_AUTHENTICATED)
    {
        return NvBootError_CryptoMgr_OemBootBinaryHeader_NotAuthenticated;
    }

    // Sanintize the load address and length, even though the header
    // is authenticated.
    NvBootError e_SdramBlCheck = NvBootValidateAddress(DramRange, OemHeader->LoadAddress, OemHeader->Length);
    NvBootError e_IramBlCheck = NvBootValidateAddress(BlRamRange, OemHeader->LoadAddress, OemHeader->Length);

    // If all of the range checks above fail, then return error and don't copy.
    if((e_SdramBlCheck != NvBootError_Success) && (e_IramBlCheck != NvBootError_Success))
        return NvBootError_Invalid_Bl_Size_And_Or_Destination;

    // Calculate the SHA hash of the BlBinary
    ShaDevMgr.ShaDevMgrCallbacks->ShaHash(BlBinary,
                                          OemHeader->Length,
                                          (uint32_t *) &s_CryptoMgr_Buffers.Calculated_BlHash,
                                          &ShaDevMgr.ShaConfig);

    // Compare the calculated hash with the OemBinaryHash in the OemHeader.
    FI_bool compare_result = FI_FALSE;
    compare_result = NvBootUtilCompareConstTimeFI(&s_CryptoMgr_Buffers.Calculated_BlHash,
                                                &s_CryptoMgr_Buffers.OemBootBinaryHash,
                                                sizeof(NvBootSha256HashDigest));
    if(compare_result == FI_TRUE)
    {
        return NvBootError_Success;
    }
    else
    {
        return NvBootError_CryptoMgr_BlBinaryPackage_Auth_Error;
    }
}

NvBootError NvBootCryptoMgrDecryptBlPackage(const NvBootOemBootBinaryHeader *OemHeader, uint32_t *BlBinary)
{
    if (s_CryptoMgrContext.EncryptionScheme != CryptoAlgo_AES)
    {
        // return success and skip decyrption.
        return NvBootError_Success;
    }

    if(s_CryptoMgrContext.IsOemBootBinaryHeaderAuthenticated != OEM_HEADER_AUTHENTICATED)
    {
        return NvBootError_CryptoMgr_OemBootBinaryHeader_NotAuthenticated;
    }

    // Sanintize the load address and length, even though the header
    // is authenticated.
    NvBootError e_SdramBlCheck = NvBootValidateAddress(DramRange, (uint32_t) BlBinary, OemHeader->Length);
    NvBootError e_IramBlCheck = NvBootValidateAddress(BlRamRange, (uint32_t) BlBinary, OemHeader->Length);

    // If all of the range checks above fail, then return error and don't copy.
    if((e_SdramBlCheck != NvBootError_Success) && (e_IramBlCheck != NvBootError_Success))
        return NvBootError_Invalid_Bl_Size_And_Or_Destination;

    uint8_t * start_decrypt_address = (uint8_t *) BlBinary;
    uint8_t * decrypt_output_address = (uint8_t *) BlBinary;

    const uint8_t KeySlot = Context.FactorySecureProvisioningMode == false ? AES_DEVICE_KEYSLOT_BEK : AES_DEVICE_KEYSLOT_FSKP_DECRYPT;
    NvBootError e;
    // Clear IVs of keyslot before decrypt.
    e = NvBootInitializeNvBootError();
    NV_BOOT_CHECK_ERROR(AesDevMgr.AesDevMgrCallbacks->ClearOriginalIv(KeySlot));
    e = NvBootInitializeNvBootError();
    NV_BOOT_CHECK_ERROR(AesDevMgr.AesDevMgrCallbacks->ClearUpdatedIv(KeySlot));

    // Default to "fail", subsequent functions can set to pass.
     e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    e = AesDevMgr.AesDevMgrCallbacks->AesDecryptCBC(start_decrypt_address,
                                                decrypt_output_address,
                                                KeySlot,
                                                s_CryptoMgrContext.AesKeySize,
                                                OemHeader->Length);
    return e;
}

NvBootError NvBootCryptoMgrOemAuthSc7Fw(const NvBootWb0RecoveryHeader *Sc7Header)
{
    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    // Sanitize address and size, if it can fit into NVBOOT_SC7_FW_START to NVBOOT_SC7_FW_END + 1.
    uint32_t AuthAddrStart = (uint32_t) Sc7Header +  OFFSET_SC7_SIGNED_SECT(Sc7Header);
    // The function NvBootWb0CopyHeaderAndFirmware() has checked that LengthInsecure
    // is greater than sizeof(NvBootWb0RecoveryHeader) + 4 bytes (i.e. smallest possible SC7 firmware is
    // an ARM mode branch to self).
    uint32_t AuthSize = Sc7Header->LengthInsecure - sizeof(NvBootWb0RecoveryHeader) + SIZE_SC7_SIGNED_SECT(Sc7Header);

    NV_BOOT_CHECK_ERROR(NvBootValidateAddress(Sc7FwRamRange, AuthAddrStart, AuthSize));

    // Re-init default error code to a fail value.
    e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_RSA_RSASSA_PSS)
    {
        // Setup RSASSA-PSS context.
        s_CryptoMgr_Buffers.RsaPssContext.RsaKeySlot = CRYPTOMGR_RSA_PUBLIC_KEY_SLOT;
        s_CryptoMgr_Buffers.RsaPssContext.RsaKey = &s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageIsHashed = false;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessage = (uint32_t *) AuthAddrStart;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageLengthBytes = AuthSize;
        s_CryptoMgr_Buffers.RsaPssContext.InputSignature = (NvBootCryptoRsaSsaPssSig *) &Sc7Header->Signatures.RsaSsaPssSig;

        e = NvBootCryptoRsaSsaPssVerify(&s_CryptoMgr_Buffers.RsaPssContext, &ShaDevMgr, &RsaDevMgr);
        return e;
    }
    else if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_AES_CMAC)
    {
        // Setup AES cmac context.
        s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
        s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
        s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) AuthAddrStart;
        s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
        s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_128;
        s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(AuthSize, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_SBK;

        // Generate subkey.
        AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                        s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK2);

        // Calculate AES-CMAC hash.
        AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

        FI_bool compare_result = FI_FALSE;
        compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &Sc7Header->Signatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        if(compare_result == FI_TRUE)
            e = NvBootError_Success;
        else
            e = NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;

        return e;
    }
    else
    {
        return NvBootError_CryptoMgr_InvalidAuthScheme;
    }
}

NvBootError NvBootCryptoMgrOemDecryptSc7Fw(const NvBootWb0RecoveryHeader *Sc7Header)
{
    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    if (s_CryptoMgrContext.EncryptionScheme != CryptoAlgo_AES)
    {
        // return success and skip decyrption.
        return NvBootError_Success;
    }

    uint32_t start_decrypt_address = (uint32_t) Sc7Header +  OFFSET_SC7_SIGNED_SECT(Sc7Header);
    // The function NvBootWb0CopyHeaderAndFirmware() has checked that LengthInsecure
    // is greater than sizeof(NvBootWb0RecoveryHeader) + 4 bytes (i.e. smallest possible SC7 firmware is
    // an ARM mode branch to self).
    uint32_t decrypt_size = Sc7Header->LengthInsecure - sizeof(NvBootWb0RecoveryHeader) + SIZE_SC7_SIGNED_SECT(Sc7Header);

    // Sanitize address and size, if it can fit into NVBOOT_SC7_FW_START to NVBOOT_SC7_FW_END + 1.
    NV_BOOT_CHECK_ERROR(NvBootValidateAddress(Sc7FwRamRange, start_decrypt_address, decrypt_size));

    // Re-init error to fail before decryption.
    e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    e = AesDevMgr.AesDevMgrCallbacks->AesDecryptCBC((uint8_t *) start_decrypt_address,
                                                (uint8_t *) start_decrypt_address,
                                                AES_DEVICE_KEYSLOT_BEK,
                                                s_CryptoMgrContext.AesKeySize,
                                                decrypt_size);
    return e;
}

NvBootError NvBootCryptoMgrOemAuthRcmPayload(const NvBootRcmMsg *RcmMsg)
{
    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    uint32_t AuthSize = 0;
    // Calculate the length to authenticate

    NV_BOOT_CHECK_ERROR(NvBootValidateAddress(RcmMsgRange, (uint32_t) RcmMsg, RcmMsg->LengthInsecure));

    AuthSize = RcmMsg->LengthInsecure - OFFSET_RCM_SIGNED_SECT(RcmMsg);

    // Re-init error to fail before authentication.
    e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_RSA_RSASSA_PSS)
    {
        // Setup RSASSA-PSS context.
        s_CryptoMgr_Buffers.RsaPssContext.RsaKeySlot = CRYPTOMGR_RSA_PUBLIC_KEY_SLOT;
        s_CryptoMgr_Buffers.RsaPssContext.RsaKey = &s_CryptoMgr_Buffers.Pcp.RsaPublicParams.RsaPublicKey;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageIsHashed = false;
        s_CryptoMgr_Buffers.RsaPssContext.InputMessage = (uint32_t *) ((uint32_t) RcmMsg + OFFSET_RCM_SIGNED_SECT(RcmMsg));
        s_CryptoMgr_Buffers.RsaPssContext.InputMessageLengthBytes = AuthSize;
        s_CryptoMgr_Buffers.RsaPssContext.InputSignature = (NvBootCryptoRsaSsaPssSig *) &RcmMsg->Signatures.RsaSsaPssSig;

        e = NvBootCryptoRsaSsaPssVerify(&s_CryptoMgr_Buffers.RsaPssContext, &ShaDevMgr, &RsaDevMgr);
        return e;
    }
    else if(s_CryptoMgrContext.AuthenticationScheme == CryptoAlgo_AES_CMAC)
    {
        // Setup AES-CMAC context.
        s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
        s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
        s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) ((uint32_t) RcmMsg + OFFSET_RCM_SIGNED_SECT(RcmMsg));
        s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
        s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_128;
        s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(AuthSize, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
        s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_SBK;

        // Generate subkey.
        AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                        s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                        s_CryptoMgr_Buffers.AesCmacContext.pK2);

        // Calcaulte AES-CMAC hash.
        AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

        FI_bool compare_result = FI_FALSE;
        compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &RcmMsg->Signatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
        if(compare_result == FI_TRUE)
            return NvBootError_Success;
        else
            return NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;
    }
    else
    {
        return NvBootError_CryptoMgr_InvalidAuthScheme;
    }

    return NvBootError_Success;
}

NvBootError NvBootCryptoMgrOemDecryptRcmPayload(const NvBootRcmMsg *RcmMsg)
{
    // Default to "fail", subsequent functions can set to pass.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    if (s_CryptoMgrContext.EncryptionScheme != CryptoAlgo_AES)
    {
        // return success and skip decyrption.
        return NvBootError_Success;
    }

    // Sanitize address and size, if it can fit into RCM buffer.
    uint32_t start_decrypt_address = (uint32_t) RcmMsg +  OFFSET_RCM_SIGNED_SECT(RcmMsg);
    uint32_t decrypt_size = RcmMsg->LengthInsecure - SIZE_RCM_SIGNED_SECT(RcmMsg);

    NV_BOOT_CHECK_ERROR(NvBootValidateAddress(RcmMsgRange, start_decrypt_address, decrypt_size));

    // Re-init error to fail before authentication.
    e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    e = AesDevMgr.AesDevMgrCallbacks->AesDecryptCBC((uint8_t *) start_decrypt_address,
                                                (uint8_t *) start_decrypt_address,
                                                AES_DEVICE_KEYSLOT_BEK,
                                                s_CryptoMgrContext.AesKeySize,
                                                decrypt_size);
    return e;
}

/**
 *  Load 'default' SE keys into AES Slots 0-11 for SE and SE2
 *  Special handling required for SE Slot 0 as it contains Nv FEK for decrypting these keys.
 */
NvBootError NvBootCryptoMgrLoadDefaultSEKeys()
{
    uint8_t AesKeyDecryptionBuffer[NVBOOT_SE_AES_KEY128_LENGTH_BYTES] __attribute__((aligned(4)));
    NvBootSecureIromLayout *SecureIrom = (NvBootSecureIromLayout*)(NVBOOT_SEC_IROM_START);
 
    TODO // does this have to be protected by NV key. 
    // Special handling for slot 0. Start from 11 and work down to 0.
    for(int i=AES_DEVICE_KEYSLOT_11; i >= AES_DEVICE_KEYSLOT_0;i--)
    {
        // Copy encrypted key from IROM blob and decrypt into slot i.
        // PSK @ Slot i
        NvBootCryptoMgrCopyArrayFromRegToBuffer((uint8_t*)AesKeyDecryptionBuffer,
            (NvU32)&SecureIrom->PSK[i],
            NVBOOT_SE_AES_KEY128_LENGTH_BYTES);
        
        // Decrypt key from buffer into slot in SE
        NvBootSeAesDecryptKeyIntoKeySlot(
            AES_DEVICE_KEYSLOT_NV_FEK, SE_MODE_PKT_AESMODE_KEY128,
            i, SE_MODE_PKT_AESMODE_KEY128, // Decrypt Into Slot i
            (uint8_t*)AesKeyDecryptionBuffer);
        
        // Decrypt key from buffer into slot in SE2
        NvBootSe2AesDecryptKeyIntoKeySlot(
            AES_DEVICE_KEYSLOT_NV_FEK, SE_MODE_PKT_AESMODE_KEY128,
            i, SE_MODE_PKT_AESMODE_KEY128, // Decrypt Into Slot i
            (uint8_t*)AesKeyDecryptionBuffer);
        
        // Zero out buffer just in case before next use.
        NvBootUtilMemset((uint8_t*)AesKeyDecryptionBuffer, 0, sizeof(AesKeyDecryptionBuffer));
    }
#if 0
    uint8_t PSK1_readback[NVBOOT_SE_AES_KEY128_LENGTH_BYTES];
    uint8_t PSK2_readback[NVBOOT_SE_AES_KEY128_LENGTH_BYTES];
    
    for(int i=AES_DEVICE_KEYSLOT_11; i >= AES_DEVICE_KEYSLOT_0;i--)
    {
        NvBootSeKeySlotReadKeyIV(i,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)PSK1_readback);
        NvBootSe2KeySlotReadKeyIV(i,
            SE_MODE_PKT_AESMODE_KEY128,
            SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
            (NvU32*)PSK2_readback);

        if(!NvBootUtilCompareBytes(PSK2_readback, PSK1_readback, 16))
            return -1;
    }
#endif
    return NvBootError_Success;
}

NvBootError NvBootCryptoMgrFskpInit(FskpKeyNum KeyNum, uint8_t *KeyWrapKey)
{
    Context.FactorySecureProvisioningMode = true;
    // Force FSKP AES-CMAC authentication and encryption.
    s_CryptoMgrContext.AuthenticationScheme = CryptoAlgo_FSKP_CMAC;
    s_CryptoMgrContext.EngineForAuthentication = CryptoAlgo_AES;
    s_CryptoMgrContext.EncryptionScheme = CryptoAlgo_AES;
    s_CryptoMgrContext.AesKeySize = AES_KEY_256;

    if(NvBootFuseGetSecureProvisioningIndexValidity() == NvBootError_Success)
    {
        Context.ProvisioningKeyNum = NvBootFuseGetSecureProvisionIndex();
    }
    else
    {
        // Else we use the _insecure value. No need to worry about
        // NvBootError_SecProvisioningInvalidAntiCloningKey, since we reached
        // this path after secure provisioning is confirmed enabled.
        Context.ProvisioningKeyNum = KeyNum;
    }
    // Load the prospective FSPK from secure IROM.
    // If the ProvisioningKeyNum is the Key Wrap Key, decrypt it directly into the
    // key slot.
    if(Context.ProvisioningKeyNum == FSKP_KEY_WRAP_KEY)
    {
        // Load the prospective FSPK from secure IROM into the KeyWrapKey_Decrypt
        // key slot.
        NvBootCryptoMgrLoadFSKP(Context.ProvisioningKeyNum, AES_DEVICE_KEYSLOT_FSKP_KWK_DECRYPT);

        // Decrypt the encrypted user-specified AES provisioning key directly into
        // the SE key slots for FSPK.
        NvBootSeAesDecryptKeyIntoKeySlot(AES_DEVICE_KEYSLOT_FSKP_KWK_DECRYPT,
                                         SE_MODE_PKT_AESMODE_KEY256,
                                         AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE,
                                         SE_MODE_PKT_AESMODE_KEY256,
                                         KeyWrapKey);
    }
    else
    {
        // Load the prospective FSPK from secure IROM.
        NvBootCryptoMgrLoadFSKP(Context.ProvisioningKeyNum, AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE);
    }
    return NvBootError_Success;
}

NvBootError NvBootCryptoMgrAuthBctFskp(const NvBootConfigTable *Bct)
{
    // Setup AES-CMAC context.
    s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
    s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
    s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) ((uint32_t) Bct + OFFSET_BCT_SIGNED_SECT(Bct));
    s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
    s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_256;
    s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(SIZE_BCT_SIGNED_SECT(Bct), NVBOOT_AES_BLOCK_LENGTH_BYTES);
    s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
    s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
    s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE;

    // Generate subkey.
    AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                    s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                    s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                    s_CryptoMgr_Buffers.AesCmacContext.pK2);

    // Calcaulte AES-CMAC hash.
    AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

    FI_bool compare_result = FI_FALSE;
    compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &Bct->Signatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
    if(compare_result == FI_TRUE)
        return NvBootError_Success;
    else
        return NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;
}

NvBootError NvBootCryptoMgrDecryptBctFskp(NvBootConfigTable *Bct)
{
    if (s_CryptoMgrContext.EncryptionScheme != CryptoAlgo_AES)
    {
        // return success if BCT encryption is disabled.
        return NvBootError_Success;
    }

    // BCT location is fixed via linker script. Allow input of BCT as pointer
    // for unit testing.
    uint8_t * start_decrypt_address = (uint8_t *) ((uint32_t) Bct + OFFSET_BCT_ENCRYPTED_SECT(Bct));
    uint8_t * decrypt_output_address = (uint8_t *) ((uint32_t) Bct + OFFSET_BCT_ENCRYPTED_SECT(Bct));
    size_t bct_encrypted_section_size = SIZE_BCT_ENCRYPTED_SECT(Bct);

    NvBootError e;
    e = NvBootInitializeNvBootError();
    NV_BOOT_CHECK_ERROR(AesDevMgr.AesDevMgrCallbacks->ClearOriginalIv(AES_DEVICE_KEYSLOT_FSKP_DECRYPT));
    e = NvBootInitializeNvBootError();
    NV_BOOT_CHECK_ERROR(AesDevMgr.AesDevMgrCallbacks->ClearUpdatedIv(AES_DEVICE_KEYSLOT_FSKP_DECRYPT));
    // Default to "fail", subsequent functions can set to pass.
    e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    e = AesDevMgr.AesDevMgrCallbacks->AesDecryptCBC(start_decrypt_address,
                                                decrypt_output_address,
                                                AES_DEVICE_KEYSLOT_FSKP_DECRYPT,
                                                AES_KEY_256,
                                                bct_encrypted_section_size);

    // Compare the Header->SecProvisioningKeyNum_Insecure or the Anti-Cloning fuse value to the
    // secure provisioningkey number after decryption. If _Insecure number or
    // Anti-Cloning fuse value doesn't match the Secure field, someone has tampered
    // with the BCT.
    if(Context.ProvisioningKeyNum != Bct->SecProvisioningKeyNum_Secure)
        return NvBootError_SecProvisioningBctKeyMismatch;

    return e;
}

NvBootError NvBootCryptoMgrAuthRcmPayloadFskp(const NvBootRcmMsg *RcmMsg)
{
    NvBootError e;

    uint32_t AuthSize = 0;

    // Validate the length of the payload.
        NV_BOOT_CHECK_ERROR(NvBootValidateAddress(RcmMsgRange, (uint32_t) RcmMsg, RcmMsg->LengthInsecure));

    // Calculate the length to authenticate.
        AuthSize = RcmMsg->LengthInsecure - OFFSET_RCM_SIGNED_SECT(RcmMsg);

    // Setup AES-CMAC context.
    s_CryptoMgr_Buffers.AesCmacContext.pK1 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK1;
    s_CryptoMgr_Buffers.AesCmacContext.pK2 = (uint32_t *) &s_CryptoMgr_Buffers.CmacK2;
    s_CryptoMgr_Buffers.AesCmacContext.pInputMessage = (uint32_t *) ((uint32_t) RcmMsg + OFFSET_RCM_SIGNED_SECT(RcmMsg));
    s_CryptoMgr_Buffers.AesCmacContext.pHash = (uint32_t*) &s_CryptoMgr_Buffers.AesCmacHashResult;
    s_CryptoMgr_Buffers.AesCmacContext.KeySize = AES_KEY_256;
    s_CryptoMgr_Buffers.AesCmacContext.NumBlocks = NV_ICEIL(AuthSize, NVBOOT_AES_BLOCK_LENGTH_BYTES);
    s_CryptoMgr_Buffers.AesCmacContext.FirstChunk = true;
    s_CryptoMgr_Buffers.AesCmacContext.LastChunk = true;
    s_CryptoMgr_Buffers.AesCmacContext.KeySlot = AES_DEVICE_KEYSLOT_FSKP_AUTHENTICATE;

    // Generate subkey.
    AesDevMgr.AesDevMgrCallbacks->AesGenerateCmacSubkey(s_CryptoMgr_Buffers.AesCmacContext.KeySlot,
                                                    s_CryptoMgr_Buffers.AesCmacContext.KeySize,
                                                    s_CryptoMgr_Buffers.AesCmacContext.pK1,
                                                    s_CryptoMgr_Buffers.AesCmacContext.pK2);

    // Calcaulte AES-CMAC hash.
    AesDevMgr.AesDevMgrCallbacks->AesCmacHash(&s_CryptoMgr_Buffers.AesCmacContext);

    FI_bool compare_result = FI_FALSE;
    compare_result = NvBootUtilCompareConstTimeFI(s_CryptoMgr_Buffers.AesCmacContext.pHash, &RcmMsg->Signatures.AesCmacHash, NVBOOT_AES_BLOCK_LENGTH_BYTES);
    if(compare_result == FI_TRUE)
        return NvBootError_Success;
    else
        return NvBootError_CryptoMgr_AesCmacHash_Compare_Failure;
}

NvBootError NvBootCryptoMgrDecryptRcmPayloadFskp(const NvBootRcmMsg *RcmMsg)
{
    NvBootError e;

    if (s_CryptoMgrContext.EncryptionScheme != CryptoAlgo_AES)
    {
        // return success and skip decyrption.
        return NvBootError_Success;
    }

    // Sanitize address and size, if it can fit into RCM buffer.
    uint32_t start_decrypt_address = (uint32_t) RcmMsg +  OFFSET_RCM_SIGNED_SECT(RcmMsg);
    uint32_t decrypt_size = RcmMsg->LengthInsecure - SIZE_RCM_SIGNED_SECT(RcmMsg);

    NV_BOOT_CHECK_ERROR(NvBootValidateAddress(RcmMsgRange, start_decrypt_address, decrypt_size));

    e = AesDevMgr.AesDevMgrCallbacks->AesDecryptCBC((uint8_t *) start_decrypt_address,
                                                (uint8_t *) start_decrypt_address,
                                                AES_DEVICE_KEYSLOT_FSKP_DECRYPT,
                                                AES_KEY_256,
                                                decrypt_size);

    if(Context.ProvisioningKeyNum != RcmMsg->SecProvisioningKeyNum_Secure)
        return NvBootError_SecProvisioningBctKeyMismatch;

    return e;
}


