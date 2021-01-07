/*
 * Copyright (c) 2006 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 *  @file nvboot_se_aes.c
 *
 *  Implement SE key read and write, as well as the context restore of the SE
 *  after LP0 exit.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arse.h"
#include "project.h"
#include "nvboot_aes.h"
#include "nvboot_clocks_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_lp0_context.h"
#include "nvboot_se_rsa.h"
#include "nvboot_se_defs.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_reset_int.h"
#include "nvboot_se_int.h"
#include "nvboot_ssk_int.h"
#include "nvboot_util_int.h"


//---------------------MACRO DEFINITIONS--------------------------------------

/**
 *  SE Linked List size used; 1 entry
 *            -----------------------------------------------------------
 * LL_ADDR -> | Last Buffer number (n) (32-bits)                        |
 *            -----------------------------------------------------------
 *            | Buffer 0 Start Byte Address (32-bits)                   |
 *            -----------------------------------------------------------
 *            | Buffer 0 Reserved (8-bits), Buffer Byte Size (24-bits)  |
 *            -----------------------------------------------------------
 */
#define SeLLSize 1+(1*2)
        
// Long winded way of saying 0x0. Set each keyslot to SECURE mode only. 
#define SE_CRYPTO_SECURITY_PERKEY_0_ALL_SECURE \
        ( ARSE_SECURE << 0 \
        | ARSE_SECURE << 1 \
        | ARSE_SECURE << 2 \
        | ARSE_SECURE << 3 \
        | ARSE_SECURE << 4 \
        | ARSE_SECURE << 5 \
        | ARSE_SECURE << 6 \
        | ARSE_SECURE << 7 \
        | ARSE_SECURE << 8 \
        | ARSE_SECURE << 9 \
        | ARSE_SECURE << 10 \
        | ARSE_SECURE << 11 \
        | ARSE_SECURE << 12 \
        | ARSE_SECURE << 13 \
        | ARSE_SECURE << 14 \
        | ARSE_SECURE << 15)
        
// 200000 microseconds timeout (= 200 ms). If we timeout something really bad
// went wrong. 
// SE data rate = SE clock frequency / (N + 1), where N is the rate attenuation
// read from the encryption rate fuse bits. At worst case, N = 15. Assuming
// SE clock to use the same clock as the AVP, it should be 108Mhz. 
// 108Mhz / 16 = 6 Mbps. To decrypt our 1072 bytes of SE context, 6 Mbps and
// 200 ms should be more than enough. 
#define NVBOOT_SE_OP_TIMEOUT_US 200000

// Define the SE_OPERATION IDLE value we expect. 
#define NVBOOT_SE_OP_STATUS_IDLE (NV_DRF_DEF(SE, STATUS, STATE, IDLE))

// Number of linked list buffers * 2 entries each, plus one word for the last
// buffer number
NV_ALIGN (4) static SeLinkedList s_InputLinkedList;


// ---------------------Private Functions Declaration---------------------------
static void
NvBootSeLeftShiftOneBit(NvU8 *in_buf, NvU32 size)
{
    NvU8    carry;
    NvU32   i;

    /* left shift one bit */
    in_buf[0] <<= 1;
    for (carry = 0, i = 1; i < size; i++) {
        carry = NVBOOT_SE_AES_CMAC_IS_MSB_SET(in_buf[i]);
        in_buf[i-1] |= carry;
        in_buf[i] <<= 1;
    }
}

static void
NvBootSeClearSrkSecureScratch()
{
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH4_0, 0);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH5_0, 0);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH6_0, 0);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH7_0, 0);

    return;
}

static void
NvBootSeClearAllKeySlots() 
{
    NvU32 Slot;
    // Clear all key slots to zero. 
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeKeySlotWriteKeyIV(Slot, SE_MODE_PKT_AESMODE_KEY256, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, 0);
        NvBootSeKeySlotWriteKeyIV(Slot, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 0);
        NvBootSeKeySlotWriteKeyIV(Slot, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
    }

    return;
}

static void
NvBootSeDisableAllKeySlotReadWrite() 
{
    NvU32 Slot = 0;

    // Disable key slot read and write for all key slots. 
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeDisableKeySlotReadAccess(Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
        NvBootSeDisableKeySlotWriteAccess(Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
            
        NvBootSeDisableKeySlotReadAccess(Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS);
        NvBootSeDisableKeySlotWriteAccess(Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS);

        NvBootSeDisableKeySlotReadAccess(Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS);
        NvBootSeDisableKeySlotWriteAccess(Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS);
    }
    
    // Set each keyslot as SECURE (can be accessed in secure mode only). 
    // Not absolutely necessary after we cleared and locked down the key slots, 
    // but lets be paranoid. 
    NvBootSetSeReg(SE_CRYPTO_SECURITY_PERKEY_0, SE_CRYPTO_SECURITY_PERKEY_0_ALL_SECURE);

    return;
}

static void
NvBootSeDisableTzram()
{
    NvU32 SeConfigReg = 0;
    // Set TZRAM_SETTING to ARSE_SECURE. Can't use DRF macros because
    // ARSE_SECURE is separately defined from SE_TZRAM_SECURITY_0. 
    SeConfigReg = (ARSE_SECURE << SE_TZRAM_SECURITY_0_TZRAM_SETTING_SHIFT);
    // Set TZRAM_ENG_DIS to TRUE (All TZRAM writes will be ignored and all 
    // read data masked). 
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, TZRAM_SECURITY, TZRAM_ENG_DIS, TRUE, SeConfigReg);
    NvBootSetSeReg(SE_TZRAM_SECURITY_0, SeConfigReg);

    return;
}

static void
NvBootSeDisableSe()
{
    NvU32 SeConfigReg = 0;

    // Disable all crypto ops with SE_ENG_DIS.
    SeConfigReg = NV_DRF_DEF(SE, SE_SECURITY, SE_ENG_DIS, TRUE);
    // Can't use DRF macros because ARSE_SECURE is separately defined from
    // SE_SE_SECURITY_0.
    // No need to set SE_SE_SECURITY_0_SE_SOFT_SETTING. SE_SE_SECURITY_0_SE_HARD_SETTING
    // overrides the SOFT_SETTING. T210 See SE IAS, section 3.1 Modes of Operation.
    SeConfigReg &= ~(SE_SE_SECURITY_0_SE_HARD_SETTING_FIELD | SE_SE_SECURITY_0_PERKEY_SETTING_FIELD);
    SeConfigReg |= (ARSE_SECURE << SE_SE_SECURITY_0_SE_HARD_SETTING_SHIFT) |
                   (ARSE_SECURE << SE_SE_SECURITY_0_PERKEY_SETTING_SHIFT);
    NvBootSetSeReg(SE_SE_SECURITY_0, SeConfigReg);

    return;
}


static NvBool NvBootSeCompareBytes(NvU8 *Value1, NvU8 *Value2, NvU32 ValueSizeBytes) 
{
    int i;

    for(i= 0; i < ValueSizeBytes; i++)
    {
        if(*Value1++ != *Value2++) 
        {
            return NV_FALSE;
        }

    }
    return NV_TRUE;
}

static NvU32 NvBootSeConvertRsaKeySize(NvU32 RsaKeySizeBits)
{
    if(RsaKeySizeBits == 2048) {
        return  SE_RSA_KEY_SIZE_0_VAL_WIDTH_2048;
    } else if (RsaKeySizeBits == 1536) {
        return  SE_RSA_KEY_SIZE_0_VAL_WIDTH_1536;
    } else if (RsaKeySizeBits == 1024) {
        return  SE_RSA_KEY_SIZE_0_VAL_WIDTH_1024;
    } else if (RsaKeySizeBits == 512) {
        return SE_RSA_KEY_SIZE_0_VAL_WIDTH_512;
    } else {
        return 0;
    }
}

/**
 * Generate the correct value of SE_CRYPTO_KEYIV_PKT for SE
 * AES key operations.
 *
 * @param KeySlot SE AES key slot number.
 * @param KeyIvSel Select a Key or IV for the operation. Use
 *                 SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_*.
 * @param KeyWord Select the word number to access in the AES key for the current
 *                operation. Only used for non-IV operations.
 * @param IvSel SE_CRYPTO_KEYIV_PKT_IV_SEL value for IV operations. Ignored
 *              for regular AES key operations.
 * @param IvWord SE_CRYPTO_KEYIV_PKT_IV_WORD value for IV operations. Ignored
 *               for regular AES key operations.
 *
 */
static NvU32 SeCreateCryptoKeyIvPkt(NvU8 KeySlot, NvU8 KeyIvSel, NvU8 KeyWord, NvU8 IvSel, NvU8 IvWord)
{
    NvU32 SeCryptoKeyIvPkt = 0;
    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT( (KeyIvSel == SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_IV) ||
               (KeyIvSel == SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_KEY) );
    // KeyWord must be between 0 and 7.
    NV_ASSERT(KeyWord < ARSE_CRYPTO_KEYS_PER_KEYCHUNK);
    NV_ASSERT( (IvSel == SE_CRYPTO_KEYIV_PKT_IV_SEL_ORIGINAL) ||
               (IvSel == SE_CRYPTO_KEYIV_PKT_IV_SEL_UPDATED) );
    // Since IVs are 128-bits, IvWord can be one of four 32-bit words.
    NV_ASSERT(IvWord < ARSE_CRYPTO_IVS_PER_KEYCHUNK);

    if(KeyIvSel == SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_KEY)
    {
        // IV_SEL and IV_WORD aren't use in the key case, and overlap with
        // KEY_WORD in SE_CRYPTO_KEYIV_PKT.
        SeCryptoKeyIvPkt = (
            (KeySlot << SE_CRYPTO_KEYIV_PKT_KEY_INDEX_SHIFT) |
            (KeyIvSel << SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_SHIFT) |
            (KeyWord << SE_CRYPTO_KEYIV_PKT_KEY_WORD_SHIFT)
            );
    }
    else if (KeyIvSel == SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_IV)
    {

        // IV_SEL and IV_WORD overlap with KEY_WORD in SE_CRYPTO_KEYIV_PKT.
        // KEY_WORD isn't touched here.
        SeCryptoKeyIvPkt = (
            (KeySlot << SE_CRYPTO_KEYIV_PKT_KEY_INDEX_SHIFT) |
            (KeyIvSel << SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_SHIFT) |
            (IvSel << SE_CRYPTO_KEYIV_PKT_IV_SEL_SHIFT) |
            (IvWord << SE_CRYPTO_KEYIV_PKT_IV_WORD_SHIFT)
            );
   }
   // No else condition. Asserts above should take care of any fuzzed input
   // for KeyIvSel.

   return SeCryptoKeyIvPkt;
}
#if 0
static NvBool NvBootSeCompareWords(NvU8 *Value1, NvU8 *Value2, NvU32 ValueSizeWords) 
{
    int i;

    for(i= 0; i < ValueSizeWords; i++)
    {
        if(*Value1++ != *Value2++) 
        {
            return NV_FALSE;
        }

    }
    return NV_TRUE;
}
#endif

#if 0
static void SeDebugWriteScratch(NvU32 FirstScratchReg, NvU32 *InputBuffer, NvU32 BufSize) 
{

    NvU32 i = 0;
    for(i = 0; i < BufSize; i++) {
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + FirstScratchReg + (i*4), *InputBuffer++);
    }
    return;
}
#endif

// Worst case for the octet string T is the largest key size with 
// the smallest possible Hash algorithm output size (2048 bit key, SHA1 
// hash). This should be:
// Max possible counter C = Ceil(maskLen/hLen) - 1
//                        = Ceil( (emLen - hLen -1) / hLen) - 1
//                        = Ceil( (256 - 20 - 1) / 20) - 1
//                        = Ceil(235/20) - 1
//                        = 11. 
//
//            Where emLen = Ceil(modBits-1/8)
//                        = Ceil(2048-1/8)
//                        = Ceil(2047/8)
//                        = 256
//
// Max possible number of loops is from 0 to 11, which is 12 loops. 
//
// So, worst case is 12 loops of the MGF function, and the largest
// hash size supported is 512-bits. 
//
// (512/8) * 12 = 768 bytes
//
// In reality we shouldn't go over 252 bytes (SHA224 and 9 loops). 
//
#define MAX_MGF_MASKLEN (256 - 20 - 1)
#define MAX_MGF_COUNTER (NV_ICEIL(MAX_MGF_MASKLEN, 20) - 1)
#define MAX_MGF_COUNTER_LOOPS (MAX_MGF_COUNTER + 1)

static NvBootError NvBootSeMaskGenerationFunction(NvU8 *mgfSeed, NvU32 maskLen, NvU8 *dbMaskBuffer, NvU8 HashAlgorithm, NvU32 hLen) 
{
    NvU32 counter = 0;
    NvU32 T[((ARSE_SHA512_HASH_SIZE/8) * MAX_MGF_COUNTER_LOOPS) / 4]; 
    NvU32 hashInputBuffer[(ARSE_SHA512_HASH_SIZE / 8 / 4) +  1]; // +1 for the octet string C of 4 octets
    // Step 1. If maskLen > 2^32 hLen, output "mask too long" and stop. 

    // Step 2. Let T be the empty octet string. 
    
    // Step 3. For counter from 0 to Ceiling(maskLen / hLen) - 1, do the 
    //         following:
    //         a. Convert counter to an octet string C of length 4 octets:
    //              C = I2OSP(counter, 4)
    //         b. Concatentate the hash of the seed mgfSeed and C to the octet
    //         string T:
    //         T = T || Hash(mgfSeed||C). 
    
    for (counter = 0; counter <= (NV_ICEIL(maskLen, hLen) - 1); counter++) 
    {
        // counter is already a NvU32, so no need to convert to 
        // length of four octets

        NvBootUtilMemcpy(&hashInputBuffer[0], mgfSeed, hLen);
        hashInputBuffer[(hLen / 4) + 1 - 1] = NvBootUtilSwapBytesInNvU32(counter);
        NvBootSeSHAHash(&hashInputBuffer[0], hLen + 4, NULL, &T[0], HashAlgorithm);

        while(NvBootSeIsEngineBusy())
            ;

        NvBootUtilMemcpy(&dbMaskBuffer[counter*hLen], &T[0], hLen);

    }

    // Step 4. Output the leading maskLen octets of T as the octet string mask. 

    return NvBootError_Success;
}

static void SeReverseList(NvU8 *original, NvU32 listSize)
{
    NvU8 i, j, temp;

    for(i = 0, j = listSize - 1; i < j; i++, j--) 
    {
        temp = original[i];
        original[i] = original[j];
        original[j] = temp;
    }

}

/**
 * Generate a linked list structure for consumption by the SE. 
 *
 * @param SeLinkedList  Pointer to SeLinkedList struct
 * @param pStartAddress Pointer to start address of data to be processed by the SE
 * @param MessageSize   Size in bytes of the data to be processed by the SE
 */
static NvBootSeGenerateLinkedList(SeLinkedList *pLinkedList, NvU32 *pStartAddress, NvU32 MessageSize) 
{
    
    NvU32 i;
    const NvU32 NumBuffers = NV_ICEIL(MessageSize, NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES);

    NV_ASSERT(NumBuffers > 0);
    NV_ASSERT(MessageSize <= NVBOOT_SE_LL_MAX_SIZE_BYTES);

    // The first entry of the linked list is the "last buffer number", which
    // begins with buffer 0. 
    // For example, if there are two linked list buffers, the last buffer number
    // is buffer 1. 
    pLinkedList->LastBufferNumber = NumBuffers - 1;
    
    for(i = 0; i < NumBuffers; i++)
    {
        // Fill linked list buffer element with address of message and size. 
        pLinkedList->LLElement[i].StartByteAddress = (NvU32) pStartAddress;
        pLinkedList->LLElement[i].BufferByteSize = MessageSize > NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES ?
                    NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES : MessageSize;

        pStartAddress += NVBOOT_SE_LL_MAX_BUFFER_SIZE_WORDS;

        // Subtract the maximum buffer size for every buffer except the last. 
        if(MessageSize > NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES)
            MessageSize -= NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES;
    }
    return;
}

// ---------------------Public Functions Definitions----------------------------

/**
 *  Get SE config register
 */
NvU32
NvBootGetSeReg(NvU32 Reg)
{   
    return NV_READ32(NV_ADDRESS_MAP_SE_BASE + Reg);
}

/**
 *  Set SE config register
 */
void
NvBootSetSeReg(NvU32 Reg, NvU32 Data)
{
    NV_WRITE32(NV_ADDRESS_MAP_SE_BASE + Reg, Data);
}
    
/**
 *  Enable clocks to the SE. 
 */
void
NvBootSeInitializeSE(void)
{
    NvU32   RegData;
    NvU32   SclkSource;
    NvU32   Divider;

    // Default to safe SE frequency for now.
    Divider = NVBOOT_CLOCKS_7_1_DIVIDER_BY_4;

    // Read the frequency at which the AVP(SCLK) is running at.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                 CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0);
    SclkSource = NV_DRF_VAL(CLK_RST_CONTROLLER,
                            SCLK_BURST_POLICY,
                            SWAKEUP_RUN_SOURCE,
                            RegData);

    // See the Boot GFD for details on the SE/AVP(SCLK) frequency to be
    // used. BR is supposed to set the SE clock to 102Mhz at coldboot and LP0.
    // This initialization code detects what clock speed
    // AVP(SCLK) is running at, and adjusts the SE clock source and divider
    // accordingly.
    // PLLP_OUT2 = 204Mhz, PLLP_OUT4 = 102Mhz.
    // Since SE can only use PLLP_OUT0 as a source and not PLLP_OUT2
    // or PLLP_OUT4, we need to divide down PLLP_OUT0 to match the target
    // frequency.
    // Effectively for T210, we will always divide PLLP_OUT0 by 4.
    // PLLP_OUT4 is the CYA frequency for AVP(SCLK).
    switch (SclkSource)
    {
        case CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0_SWAKEUP_RUN_SOURCE_PLLP_OUT2:
                Divider = NVBOOT_CLOCKS_7_1_DIVIDER_BY_4;
                break;
        default:
            NV_ASSERT(0);
    }

    // Set the clock source for SE.
    NvBootClocksConfigureClock(NvBootClocksClockId_SeId,
        Divider,
        CLK_RST_CONTROLLER_CLK_SOURCE_SE_0_SE_CLK_SRC_PLLP_OUT0);

    // Enable clock to the controller
    NvBootClocksSetEnable(NvBootClocksClockId_SeId, NV_TRUE);

    // Reset the controllers
    NvBootResetSetEnable(NvBootResetDeviceId_SeId, NV_TRUE);
    NvBootResetSetEnable(NvBootResetDeviceId_SeId, NV_FALSE);

    return;
}

NvBool 
NvBootSeIsEngineBusy(void)
{
    NvU8 Status = 0;
    NvU32 SeConfigReg = 0;

    SeConfigReg = NvBootGetSeReg(SE_STATUS_0);
    Status = NV_DRF_VAL(SE, STATUS, STATE, SeConfigReg);

    if(Status == NVBOOT_SE_OP_STATUS_IDLE)
    {
        return  NV_FALSE;
    } else {
        return  NV_TRUE;
    }
}

void
NvBootSeClearTzram(void) 
{
    NvBootUtilMemset((void *) NV_ADDRESS_MAP_TZRAM_BASE, 0, ARSE_TZRAM_BYTE_SIZE); 
    return;
}

void
NvBootSeSetupOpMode(NvBootSeOperationMode Mode, NvBool Encrypt, NvBool UseOrigIv, NvU8 Dst, NvU8 KeySlot, NvU8 KeySize)
{
    NvU32   SeConfigReg;

    NV_ASSERT(Mode < SE_OP_MODE_MAX);
    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );
    NV_ASSERT( (Dst == SE_CONFIG_0_DST_MEMORY) ||
               (Dst == SE_CONFIG_0_DST_HASH_REG) ||
               (Dst == SE_CONFIG_0_DST_KEYTABLE) ||
               (Dst == SE_CONFIG_0_DST_SRK) ||
               (Dst == SE_CONFIG_0_DST_RSA_REG) );

    switch (Mode)
    {
        case SE_OP_MODE_AES_CBC:
        case SE_OP_MODE_AES_CMAC_HASH:
        /**
         * 1. Configure SE for AES-CBC encrypt operation. The message input is one
         *    AES block of zeroes.
         * SE_CONFIG.DEC_ALG = NOP for encrypt, AES_DEC for decrypt.
         * SE_CONFIG.ENC_ALG = NOP for decrypt, AES_ENC for encrypt.
         * SE_CONFIG.DEC_MODE = KeySize
         * SE_CONFIG.ENC_MODE = KeySize
         * SE_CONFIG.DST: Must be memory if doing AES encrypt/decrypt. Can be
         *                HASHREG if doing AES-CMAC hash.
         *
         */
        SeConfigReg = 0;
        if(Encrypt)
        {
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, AES_ENC, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_NUM(SE, CONFIG, ENC_MODE, KeySize, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, NOP, SeConfigReg);
        }
        else
        {
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, AES_DEC, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_NUM(SE, CONFIG, DEC_MODE, KeySize, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, NOP, SeConfigReg);
        }

        SeConfigReg = NV_FLD_SET_DRF_NUM(SE, CONFIG, DST, Dst, SeConfigReg);
        NvBootSetSeReg(SE_CONFIG_0, SeConfigReg);
        /**
         * 2. Program cipher mode to CBC mode / Encrypt
         *
         * Set SE_CRYPTO_CONFIG.HASH_ENB = ENABLE for CMAC
         *
         * Set SE_CRYPTO_CONFIG.KEY_INDEX = 0
         *
         * SE_CRYPTO_CONFIG Fields:
         *      HASH_ENB = ENABLE for AES-CMAC.
         *      XOR_POS = TOP for Encrypt, BOTTOM for Decrypt.
         *      INPUT_SEL = AHB (input data)
         *      IV_SELECT = UseOrigIv
         *      VCTRAM_SEL = INIT_AESOUT for Encrypt, INIT_PREVAHB
         *                   (IV + previous input) for Decrypt.
         *      CORE_SEL = ENCRYPT or DECRYPT.
         *
         */

        // Can initialize this to zero because we touch all of the fields in
        // SE_CRYPTO_CONFIG
        SeConfigReg = 0;
        SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, HASH_ENB, DISABLE, SeConfigReg);
        if(Encrypt)
        {
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, XOR_POS, TOP, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, MEMIF, AHB, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, VCTRAM_SEL, INIT_AESOUT, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, CORE_SEL, ENCRYPT, SeConfigReg);
        }
        else
        {
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, XOR_POS, BOTTOM, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, MEMIF, AHB, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, VCTRAM_SEL, INIT_PREV_MEMORY, SeConfigReg);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, CORE_SEL, DECRYPT, SeConfigReg);
        }
        if(UseOrigIv)
        {
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, IV_SELECT, ORIGINAL, SeConfigReg);
        }
        else
        {
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, IV_SELECT, UPDATED, SeConfigReg);
        }
        SeConfigReg &= ~(SE_CRYPTO_CONFIG_0_KEY_INDEX_FIELD);
        SeConfigReg |= (KeySlot << SE_CRYPTO_CONFIG_0_KEY_INDEX_SHIFT);
        NvBootSetSeReg(SE_CRYPTO_CONFIG_0, SeConfigReg);

        if(Mode == SE_OP_MODE_AES_CMAC_HASH)
        {
            SeConfigReg = NvBootGetSeReg(SE_CRYPTO_CONFIG_0);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, HASH_ENB, ENABLE, SeConfigReg);
            NvBootSetSeReg(SE_CRYPTO_CONFIG_0, SeConfigReg);

        }
        break;
    }
}

/*
 *  Read RSA key slot.
 *  
 *  @param pKeyBuffer Pointer to the input key buffer. 
 *  @param RsaKeySizeBits Specifies the RSA key size in bits.
 *  @param RsaKeySlot Specifies the RSA key slot number.
 *  @param ExpModSel Specify SE_RSA_KEY_PKT_EXPMOD_SEL_EXPONENT or SE_RSA_KEY_PKT_EXPMOD_SEL_MODULUS.
 * 
 */
void NvBootSeRsaReadKey(NvU32 *pKeyBuffer, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot, NvU8 ExpModSel)
{
    NvU32 SeRsaKeytableAddr = 0;
    NvU32 SeRsaKeyPkt = 0;
    NvU32 i = 0;

    NV_ASSERT(pKeyBuffer != NULL);
    NV_ASSERT(RsaKeySizeBits <= NVBOOT_SE_RSA_MODULUS_LENGTH_BITS);
    NV_ASSERT(RsaKeySlot <  NvBootSeRsaKeySlot_Num);
    NV_ASSERT((ExpModSel == SE_RSA_KEY_PKT_EXPMOD_SEL_EXPONENT) || 
              (ExpModSel == SE_RSA_KEY_PKT_EXPMOD_SEL_MODULUS));

    // Create SE_RSA_KEY_PKT
    // Set EXPMOD_SEL to either MODULUS or EXPONENT
    // Set WORD_ADDR to one of the 64 exponent/modulus words for access
    SeRsaKeyPkt = ( \
            (RsaKeySlot << SE_RSA_KEY_PKT_KEY_SLOT_SHIFT) | \
            (ExpModSel << SE_RSA_KEY_PKT_EXPMOD_SEL_SHIFT) );
    SeRsaKeytableAddr |= SeRsaKeyPkt;

    for (i = 0; i < (RsaKeySizeBits / 32); i++) 
    {

        // Set WORD_ADDR field
        SeRsaKeyPkt &= ~SE_RSA_KEY_PKT_WORD_ADDR_FIELD;
        SeRsaKeyPkt |= (i << SE_RSA_KEY_PKT_WORD_ADDR_SHIFT);

        SeRsaKeytableAddr &= ~SE_RSA_KEYTABLE_ADDR_0_PKT_FIELD;
        SeRsaKeytableAddr |= SeRsaKeyPkt;

        // Start Rsa key slot read.
        NvBootSetSeReg(SE_RSA_KEYTABLE_ADDR_0, SeRsaKeytableAddr);

        *pKeyBuffer++ = NvBootGetSeReg(SE_RSA_KEYTABLE_DATA_0);
    }

    return;
}

/*
 *  Write RSA key to a key slot via DMA / memory buffer method. 
 *  
 *  When using DMA / memory buffer input method, the HW
 *  expects the modulus first, and then the exponent in the input
 *  linked list buffer. 
 *
 *  The modulus and exponent are to be loaded in one operation. 
 *
 *  @param pKeyBuffer Pointer to the input key buffer. 
 *  @param RsaModulusSizeBits Specifies the RSA modulus size in bits.
 *  @param RsaKeySizeBits Specifies the RSA key size in bits.
 *  @param RsaKeySlot Specifies the RSA key slot number.
 *
 */
void NvBootSeRsaWriteKey(NvU32 *pKeyBuffer, NvU32 RsaModulusSizeBits, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot)
{
    NvU32 SeInLL[SeLLSize];
    NvU32 SeRsaKeytableAddr = 0;
    NvU32 SeRsaKeyPkt = 0;
    NvU32 SeConfigReg = 0;

    NV_ASSERT(pKeyBuffer != NULL);
    NV_ASSERT(RsaModulusSizeBits <= NVBOOT_SE_RSA_MODULUS_LENGTH_BITS);
    NV_ASSERT(RsaKeySizeBits <= NVBOOT_SE_RSA_EXPONENT_LENGTH_BITS);
    NV_ASSERT(RsaKeySlot <  NvBootSeRsaKeySlot_Num);

    // If using DMA / memory buffer as input, set SE_CONFIG.ENC_ALG
    // to RSA.
    SeConfigReg = NV_DRF_DEF(SE, CONFIG, ENC_ALG, RSA);
    NvBootSetSeReg(SE_CONFIG_0, SeConfigReg);

    // Create input linked list data structure
    // Assumes a contiguous buffer, so there is only one entry
    // in the LL. Therefore, n = 0.
    SeInLL[0] = 0;
    SeInLL[1] = (NvU32) pKeyBuffer;
    SeInLL[2] = (RsaModulusSizeBits + RsaKeySizeBits) / 8;

    // Program SE_IN_LL_ADDR with pointer to SeInLLAddr
    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &SeInLL[0]);

    // Program size of key to be written into SE_RSA_EXP_SIZE or SE_RSA_KEY_SIZE
    // SE_RSA_*_SIZE is specified in units of 4-byte / 32-bits blocks.
    NvBootSetSeReg(SE_RSA_EXP_SIZE_0, RsaKeySizeBits / 32);
    NvBootSetSeReg(SE_RSA_KEY_SIZE_0, NvBootSeConvertRsaKeySize(RsaKeySizeBits));

    // Create SE_RSA_KEY_PKT
    SeRsaKeyPkt = ( \
            (SE_RSA_KEY_PKT_INPUT_MODE_DMA << SE_RSA_KEY_PKT_INPUT_MODE_SHIFT) | \
            (RsaKeySlot << SE_RSA_KEY_PKT_KEY_SLOT_SHIFT));
            // | \
            //(ExpModSel << SE_RSA_KEY_PKT_EXPMOD_SEL_SHIFT) );

    SeRsaKeytableAddr |= SeRsaKeyPkt;

    // Start Rsa key slot write. Begins the transfer of the contents of the
    // input memory buffer to the indended key slot.
    NvBootSetSeReg(SE_RSA_KEYTABLE_ADDR_0, SeRsaKeytableAddr);

    // Wait until engine becomes IDLE.
    while(NvBootSeIsEngineBusy())
        ;

    return;
}

/*
 * Set an RSA key slot to zero. 
 *
 * Assumes: The RSA key slot does not have its WRITE protection bit set. 
 *
 * @param RsaKeySlot  A valid SE RSA key slot. 
 *
 */
void NvBootSeRsaClearKeySlot(NvU8 RsaKeySlot)
{
    NvBootSeRsaKey2048  PublicKey __attribute__ ((aligned (4)));

    NV_ASSERT(RsaKeySlot < NvBootSeRsaKeySlot_Num);

    NvBootUtilMemset(&PublicKey.Modulus[0], 0, ARSE_RSA_MAX_MODULUS_SIZE / 8);
    NvBootUtilMemset(&PublicKey.Exponent[0], 0, ARSE_RSA_MAX_EXPONENT_SIZE / 8);

    NvBootSeRsaWriteKey((NvU32 *) &PublicKey, 
                        ARSE_RSA_MAX_MODULUS_SIZE, 
                        ARSE_RSA_MAX_EXPONENT_SIZE, 
                        RsaKeySlot);            

    return;
}

/**
 *
 *  Calls the HW RSA engine and computes the modular exponentiation 
 *  of InputMessage ^ Exponent (mod n). 
 *
 *  The exponent and modulus is expected to have already been loaded
 *  into an SE RSA key slot before calling this function. As well, 
 *  SE_RSA_KEY_SIZE and SE_RSA_EXP_SIZE should have been programmed
 *  in the key write operation.
 *
 *  Input:
 *
 *  @param  RsaKeySizeBits Specifies the RSA key size in bits.
 *                         Assumes both the modulus and the exponent are 
 *                         the same key size. 
 *
 */
NvBootError NvBootSeRsaModularExp(NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 InputMessageLengthBytes, NvU32 *pInputMessage, NvU32 *pOutputDestination)
{
    NvU32   SeConfigReg = 0;
    // Define the input and output SE linked lists static. Because this is a
    // non-blocking function, we need to ensure that the contents of
    // the linked lists live on even after this function returns.
    // This is required so that if the SE is still processing the RSA
    // calculation after this function returns, the proper linked list values
    // are used. See http://nvbugs/1207660.
    static NvU32    SeInLL[SeLLSize];
    static NvU32    SeOutLL[SeLLSize];
    //NvU32   BeginTime = 0;
    //NvU32   Status = 0;

    // Program size of key to be written into SE_RSA_EXP_SIZE or SE_RSA_KEY_SIZE
    // SE_RSA_*_SIZE is specified in units of 4-byte / 32-bits blocks. 
    NvBootSetSeReg(SE_RSA_EXP_SIZE_0, RsaKeySizeBits / 32);
    NvBootSetSeReg(SE_RSA_KEY_SIZE_0, NvBootSeConvertRsaKeySize(RsaKeySizeBits));            
    
    // Create input linked list data structure
    // Assumes a contiguous buffer, so there is only one entry 
    // in the LL. Therefore, n = 0.
    SeInLL[0] = 0;
    SeInLL[1] = (NvU32) pInputMessage;
    SeInLL[2] = InputMessageLengthBytes;

    // Program SE_IN_LL_ADDR with pointer to SeInLLAddr
    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &SeInLL[0]);
    
    /**
     * 1. Set SE to RSA modular exponentiation operation 
     * SE_CONFIG.DEC_ALG = NOP
     * SE_CONFIG.ENC_ALG = RSA
     * SE_CONFIG.DEC_MODE = DEFAULT (use SE_MODE_PKT)
     * SE_CONFIG.ENC_MODE = Don't care (using SW_DEFAULT value)
     * SE_CONFIG.DST = RSA_REG or MEMORY depending on OutputDestination
     *
     */
    // Can initialize this to zero because we touch all of the fields in
    // SE_CONFIG
    SeConfigReg = 0;
    if (pOutputDestination == NULL)
    {
        SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, RSA_REG, SeConfigReg);    
    } else {
        SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, MEMORY, SeConfigReg);    

        // Setup Output Lined List
        // Only one entry in LL, so n = 0
        SeOutLL[0] = 0;
        SeOutLL[1] = (NvU32) pOutputDestination;
        SeOutLL[2] = InputMessageLengthBytes;

        // Program SE_OUT_LL_ADDR with pointer from SeOutLLAddr
        NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &SeOutLL[0]);
    }
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, NOP, SeConfigReg);    
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, RSA, SeConfigReg);    
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_MODE, DEFAULT, SeConfigReg);    
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_MODE, DEFAULT, SeConfigReg);    

    // Write to SE_CONFIG register
    NvBootSetSeReg(SE_CONFIG_0, SeConfigReg);

    /** 
     * Set key slot number
     * 
     * SE_RSA_CONFIG Fields:
     *      KEY_SLOT = RsaKeySlot
     *      
     */
    SeConfigReg = 0;
    SeConfigReg = NV_FLD_SET_DRF_NUM(SE, RSA_CONFIG, KEY_SLOT, RsaKeySlot, SeConfigReg);
    
    // Write to SE_RSA_CONFIG register
    NvBootSetSeReg(SE_RSA_CONFIG_0, SeConfigReg);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

    return NvBootError_Success;
}

/**
 *
 * HashAlgorithm must be a supported algorithm. Use SE_MODE_PKT_SHAMODE to
 * specify.
 *
 * No splitting of SHA hash into multiple operations supported. Hash
 * should be done in one operation. This is a blocking function. 
 *
 * InputMessageSizeBytes specifies the size of the message in bytes. 
 * Although SHA input message length registers are specified in bits, 
 * all anticipated use-cases of this function will be in byte-multiples
 * (See also SE IAS section 3.2.3.3 on SHA Input Data Size).
 * The maximum anticipated size needed to be hashed by the Boot ROM
 * is a Boot Loader of size 2^32 as specified in the BootLoaderInfo's 
 * Length member (of NvU32 size), therefore InputMessageSizeBytes is
 * a NvU32. 
 *
 * pInputLinkedList is a pointer to a pre-generated SE input linked list. This
 * is used when the caller wants to hash a message that is spread throughout
 * multiple non-contiguous memory blocks. Set to NULL if your input message
 * is in a contiguous block. If the message is in a contiguous block, this 
 * function will then create the input linked list automatically. 
 * If an input linked list is specified as an input, this function assumes 
 * the input linked list is correctly formatted. If pInputLinkList is specified, 
 * pInputMessage is effectively a don't care, but must still be non-NULL.
 *
 * pOutDestination should be a buffer of appropriate digest size or NULL if
 * the result is to be output to the SE_HASH_RESULT* registers. 
 *
 */
void
NvBootSeSHAHash(NvU32 *pInputMessage, NvU32 InputMessageSizeBytes, NvU32 *pInputLinkedList, NvU32 *pOutputDestination, NvU8 HashAlgorithm)
{
    NvU32   SeConfigReg = 0;
    NvU32   SeOutLL[SeLLSize];
    NvU64   InputMessageSizeBitsLeft = InputMessageSizeBytes * 8;
    NvU32   * const pInputMessageSizeBitsLeft0 = (NvU32 *) &InputMessageSizeBitsLeft;
    NvU32   * const pInputMessageSizeBitsLeft1 = pInputMessageSizeBitsLeft0 + 1;
    NvU32   ChunksRemainingForHash = NV_ICEIL(InputMessageSizeBytes, NVBOOT_SE_LL_MAX_SIZE_BYTES);
    NvU32   InputMessageBytesLeft = InputMessageSizeBytes;
    NvBool  First = NV_TRUE;
    NvBool  Last = NV_FALSE;

    NV_ASSERT(pInputMessage != NULL);
    NV_ASSERT(InputMessageSizeBytes > 0);
    NV_ASSERT( (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA1) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA224) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA256) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA384) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA512) );

    // SHA operation is specified by programming SE_CONFIG.DEC_ALG to NOP and 
    // SE_CONFIG.ENC_ALG to SHA. 
    // Program the hash algorithm into SE_CONFIG_0_ENC_MODE.
    SeConfigReg = NV_FLD_SET_DRF_NUM(SE, CONFIG, ENC_MODE, HashAlgorithm, SeConfigReg);
    // Set DST to HASH_REG first, then override later to MEMORY if 
    // pOutputDestination is not NULL. 
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, HASH_REG, SeConfigReg);
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, NOP, SeConfigReg);
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, SHA, SeConfigReg);
    
    // Write to SE_CONFIG register.
    NvBootSetSeReg(SE_CONFIG_0, SeConfigReg);

    // Set up total message length (SE_SHA_MSG_LENGTH is specified in bits). 
    NvBootSetSeReg(SE_SHA_MSG_LENGTH_0, *pInputMessageSizeBitsLeft0);
    NvBootSetSeReg(SE_SHA_MSG_LENGTH_1, *pInputMessageSizeBitsLeft1);

    // Zero out MSG_LENGTH2-3 and MSG_LEFT2-3, since the maximum size handled by the
    // BR is <= 4GB. LEFT_0 and LEFT_1 handled later in the loop below. 
    NvBootSetSeReg(SE_SHA_MSG_LENGTH_2, 0);
    NvBootSetSeReg(SE_SHA_MSG_LENGTH_3, 0);
    NvBootSetSeReg(SE_SHA_MSG_LEFT_2, 0);    
    NvBootSetSeReg(SE_SHA_MSG_LEFT_3, 0); 

    // Process the hash operation in multiple chunks if necessary. 
    // Maximum chunk size is the maximum size of the linked list structure
    // supported by the Boot ROM. To save IRAM space, maximum size of the 
    // linked list structure is limited by NVBOOT_SE_LL_MAX_SIZE_BYTES. 
    // To break up a message into multiple SHA operations, the intermediate
    // results in SE_HASH_RESULT* are untouched between operations. 
    // BR will poll SE_OP_DONE and then start a new hash operation. 
    while(ChunksRemainingForHash > 0) 
    {
        if(First)
        {
            First = NV_FALSE;
            // Set SE_SHA_CONFIG_0 to HW_INIT_HASH_ENABLE. If there is only 
            // one total chunk to hash, HW_INIT_HASH should be ENABLE. 
            SeConfigReg = NV_DRF_DEF(SE, SHA_CONFIG, HW_INIT_HASH, ENABLE);
            NvBootSetSeReg(SE_SHA_CONFIG_0, SeConfigReg);
            
        } else {
            // Set SE_SHA_CONFIG_0 to HW_INIT_HASH_DISABLE for second
            // chunk and onwards.
            SeConfigReg = NV_DRF_DEF(SE, SHA_CONFIG, HW_INIT_HASH, DISABLE);
            NvBootSetSeReg(SE_SHA_CONFIG_0, SeConfigReg);
        }
        
        ChunksRemainingForHash--;
        Last = (ChunksRemainingForHash == 0);

        if(pInputLinkedList == NULL)
        {
            // Set up input the linked list, up to the maximum input
            // linked list size specified by NVBOOT_SE_LL_MAX_SIZE_BYTES. 
            NvBootSeGenerateLinkedList(&s_InputLinkedList, 
                                       pInputMessage, 
                                       InputMessageBytesLeft > NVBOOT_SE_LL_MAX_SIZE_BYTES ?
                                       NVBOOT_SE_LL_MAX_SIZE_BYTES : InputMessageBytesLeft);

            // Program SE_IN_LL_ADDR with pointer to s_InputLinkedList
            NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &s_InputLinkedList);
        } else {
            // Program SE_IN_LL_ADDR with pointer to a pre-generated linked
            // list.
            NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) pInputLinkedList);
        }

        // Calculate input message size left in bits. InputMessageSizeBitsLeft
        // is a NvU64. 
        InputMessageSizeBitsLeft = InputMessageBytesLeft * 8;

        NvBootSetSeReg(SE_SHA_MSG_LEFT_0, *pInputMessageSizeBitsLeft0);
        NvBootSetSeReg(SE_SHA_MSG_LEFT_1, *pInputMessageSizeBitsLeft1);
        // LEFT_2 and LEFT_3 already zeroed out 

        // If this is the last chunk to hash, set appropriate output 
        // destination. 
        if(Last)
        {
            SeConfigReg = NvBootGetSeReg(SE_CONFIG_0);
            // If doing a multiple operation hash, send intermediate output to 
            // HASH_REG first, then send last hash operation to either the 
            // HASH_RESULT registers or to memory, depending on 
            // pOutputDestination.
            //
            // SHA output destination is specified by programming SE_CONFIG.DST to 
            // either MEMORY or HASH_REG. 
            if (pOutputDestination == NULL)
            {
                SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, HASH_REG, SeConfigReg);    
            } else {
                // If a non-NULL address is specified in pOutputDestination, 
                // set up hash output to memory by building the output linked
                // list. 
                SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DST, MEMORY, SeConfigReg);    

                // Setup Output Linked List
                // Only one entry in LL, so n = 0
                SeOutLL[0] = 0;
                SeOutLL[1] = (NvU32) pOutputDestination; 
                switch(HashAlgorithm) 
                {
                    case SE_MODE_PKT_SHAMODE_SHA1:
                        SeOutLL[2] = ARSE_SHA1_HASH_SIZE / 8;
                        break;
                    case SE_MODE_PKT_SHAMODE_SHA224:
                        SeOutLL[2] = ARSE_SHA224_HASH_SIZE / 8;
                        break;
                    case SE_MODE_PKT_SHAMODE_SHA256:
                        SeOutLL[2] = ARSE_SHA256_HASH_SIZE / 8;
                        break;
                    case SE_MODE_PKT_SHAMODE_SHA384:
                        SeOutLL[2] = ARSE_SHA384_HASH_SIZE / 8;
                        break;
                    case SE_MODE_PKT_SHAMODE_SHA512:
                        SeOutLL[2] = ARSE_SHA512_HASH_SIZE / 8;
                        break;
                    default:
                        NV_ASSERT(0);
                }
                // Program SE_OUT_LL_ADDR with pointer from SeOutLLAddr
                NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &SeOutLL[0]);
            }
            // Write to SE_CONFIG register
            NvBootSetSeReg(SE_CONFIG_0, SeConfigReg);
        }

        // Subtract the number of bytes left to be processed by the maximum
        // chunk size (in this case, the maximum LL size), except for the 
        // last chunk. 
        if(InputMessageBytesLeft > NVBOOT_SE_LL_MAX_SIZE_BYTES)
            InputMessageBytesLeft -= NVBOOT_SE_LL_MAX_SIZE_BYTES;

        /**
         * Issue START command in SE_OPERATION.OP
         */
        SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
        NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

        // Poll for OP_DONE. 
        while(NvBootSeIsEngineBusy())        
            ;
    }

    return;
}


/**
 *  NvBootSeRsaPssSignatureVerify():
 *      Performs an RSASSA-PSS-VERIFY signature verification operation.
 *
 *  Input:
 *      The key must be loaded into a RSA key slot before calling
 *      this function.
 *
 *  @param RsaKeySlot Specifies the SE RSA key slot to use.
 *  @param pInputMessage Specifies the pointer to the message whose
 *          signature is to be verified
 *  @param pMessageHash Allows caller to specify the SHA hash of the
 *                      input message already completed.
 *  @param pSignature Signature to be verified. The length of the Signature
 *                    pointed to by pSignature must be of length k, where
 *                    k is the length in octets of the RSA modulus n.
 *  @param HashAlgorithm Hash algorithm to be used (used for both MGF and mHash)
 *  @param sLen Salt length to be used. Should be either 0 or hLen.
 *
 *  Returns:
 *  @retval NV_TRUE Valid signature calculated.
 *  @retval NV_FALSE Invalid signature calculated.
 *
 *
 */
NvBootError
NvBootSeRsaPssSignatureVerify(NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 *pInputMessage, NvU32 *pMessageHash, NvU32 InputMessageLengthBytes, NvU32 *pSignature, NvU8 HashAlgorithm, NvS8 sLen)
{
    NvU32   *pMessageRepresentative;
    NvU32   MessageRepresentative[NVBOOT_SE_RSA_SIGNATURE_LENGTH_BITS / 32];
    NvU32   mHash[ARSE_SHA512_HASH_SIZE / 32]; // declare mHash to be maximum digest size supported
    NvU8    *EM, *H;//, *salt;
    NvU8    *maskedDB;
    NvU32   maskedDBLen;
    NvU32   hLen = 0;
    const   NvU32   emLen = NV_ICEIL(RsaKeySizeBits - 1, 8);
    const   NvU32   maxemLen = NV_ICEIL(ARSE_RSA_MAX_MODULUS_SIZE - 1, 8);
    NvU8    LowestBits;
    NvU8    FirstOctetMaskedDB = 0;
    NvBootError MGFReturn;
    NvU8    M_Prime[8 + (ARSE_SHA512_HASH_SIZE/8) + (ARSE_SHA512_HASH_SIZE/8)];
    NvU32   H_Prime[(ARSE_SHA512_HASH_SIZE/32)];

    // dbMaskBuffer size declared to be maximum maskLen supported:
    // emLen - hLen - 1 = (Ceiling(2048-1)/8) - SHA512_HASH_SIZE in bytes - 1
    //                  = 256 - 64 - 1 = 191 bytes.
    NvU8    dbMask[maxemLen - (ARSE_SHA1_HASH_SIZE/8) - 1]; // max possible size of dbMask
    NvU8    DB[maxemLen - (ARSE_SHA1_HASH_SIZE/8) - 1]; // max possible size of DB
                                                        // (maximal emLen - smallest hash)
    NvU32   i;

    NV_ASSERT(RsaKeySlot < NvBootSeRsaKeySlot_Num);
    NV_ASSERT(RsaKeySizeBits <= NVBOOT_SE_RSA_MODULUS_LENGTH_BITS);
    NV_ASSERT(pInputMessage != NULL);
    NV_ASSERT(pSignature != NULL);
    NV_ASSERT( (sLen == 0) ||
               (sLen == ARSE_SHA1_HASH_SIZE / 8) ||
               (sLen == ARSE_SHA224_HASH_SIZE / 8) ||
               (sLen == ARSE_SHA256_HASH_SIZE / 8) ||
               (sLen == ARSE_SHA384_HASH_SIZE / 8) ||
               (sLen == ARSE_SHA512_HASH_SIZE / 8) );

    switch(HashAlgorithm)
    {
        case SE_MODE_PKT_SHAMODE_SHA1:
            hLen =  ARSE_SHA1_HASH_SIZE / 8;
            break;
        case SE_MODE_PKT_SHAMODE_SHA224:
            hLen = ARSE_SHA224_HASH_SIZE / 8;
            break;
        case SE_MODE_PKT_SHAMODE_SHA256:
            hLen = ARSE_SHA256_HASH_SIZE / 8;
            break;
        case SE_MODE_PKT_SHAMODE_SHA384:
            hLen = ARSE_SHA384_HASH_SIZE / 8;
            break;
        case SE_MODE_PKT_SHAMODE_SHA512:
            hLen = ARSE_SHA512_HASH_SIZE / 8;
            break;
        default:
            NV_ASSERT(0);
    }

    pMessageRepresentative = &MessageRepresentative[0];

    // Calculate m = s^e mod n
    NvBootSeRsaModularExp(RsaKeySlot, RsaKeySizeBits, RsaKeySizeBits / 8, pSignature, pMessageRepresentative);

    while(NvBootSeIsEngineBusy())
        ;

    // After the RSAVP1 step, the message representative m is stored in
    // in ascending order in a byte array, i.e. the 0xbc trailer field is the
    // first value of the array, when it is the "last" vlaue in the spec.
    // Reversing the byte order in the array will match the endianness
    // of the PKCS #1 spec and make for code that directly matches the spec.
    SeReverseList((NvU8 *) pMessageRepresentative, emLen);

    /**
     * Convert the message representative m to an encoded message EM of
     *       length emLen = Ceiling( (modBits - 1) / 8) octets, where modBits
     *       is the length in bits of the RSA modulus n:
     *          EM = I2OSP(m, emLen)
     *
     */
    EM = (NvU8 *) pMessageRepresentative;

    /**
     * EMSA-PSS verification: Apply the EMSA-PSS verification operation
     *    to the message M and the encoded message EM to determine whether
     *    they are consistent:
     *      Result = EMSA-PSS-VERIFY(M, EM, modBits -1)
     *
     *    if(Result == "consistent")
     *          output "Valid Signature"
     *    else
     *          output "Invlaid Signature"
     */

    // Step 2. Let mHash = Hash(M), an octet string of length hLen.
    // Allow calling functions to specify the hash of the message as a parameter.
    if(pMessageHash == NULL)
    {
        NvBootSeSHAHash(pInputMessage, InputMessageLengthBytes, NULL, &mHash[0], HashAlgorithm);
    } else {
        NvBootUtilMemcpy(&mHash[0], pMessageHash, hLen);
    }

    // Step 3. If emLen < hLen + sLen + 2, output "inconsistent" and stop.
    // emLen < hLen + sLen + 2
    // Ceiling(emBits/8) < hLen + sLen + 2
    // Ceiling(modBits-1/8) < hLen + sLen + 2
    // 256 octets < 32 octets + 32 octets + 2 (assuming SHA256 as the hash function)
    // Salt length is equal to hash length
    if((emLen*8) < hLen + NVBOOT_SE_RSA_PSS_SALT_LENGTH_BITS + 2)
    {
        return NvBootError_SE_RsaPssVerify_Inconsistent;
    }

    // Step 4. If the rightmost octet of EM does not have hexadecimal
    // value 0xbc, output "inconsistent" and stop.
    if(EM[emLen - 1] != 0xbc)
    //if(EM[0] != 0xbc)
    {
        return NvBootError_SE_RsaPssVerify_Inconsistent;
    }

    // Step 5. Let maskedDB be the leftmost emLen - hLen - 1 octets
    // of EM, and let H be the next hLen octets.
    maskedDBLen = emLen - hLen - 1;
    maskedDB = EM;
    H = EM + maskedDBLen;

    // Step 6. If the leftmost 8emLen - emBits bits of the leftmost
    // octet in maskedDB are not all equal to zero, output "inconsistent"
    // and stop.
    // 8emLen - emBits = 8*256 - 2047 = 1 (assuming 2048 key size and sha256)
    LowestBits = (8*emLen) - (RsaKeySizeBits - 1);
    FirstOctetMaskedDB = maskedDB[0] & (0xFF << (8 - LowestBits));
    if(FirstOctetMaskedDB > 0)
    {
        return NvBootError_SE_RsaPssVerify_Inconsistent;
    }

    // Step 7. Let dbMask = MGF(H, emLen - hLen - 1).
    MGFReturn = NvBootSeMaskGenerationFunction(H, emLen - hLen - 1, &dbMask[0], HashAlgorithm, hLen);
    if (MGFReturn != NvBootError_Success)
    {
        return NvBootError_SE_RsaPssVerify_Inconsistent;
    }

    // Step 8. Let DB = maskedDB XOR dbMask
    for(i = 0; i < maskedDBLen; i++)
    {
        DB[i] = maskedDB[i] ^ dbMask[i];
    }

    // Step 9. Set the leftmost 8emLen - emBits bits of the leftmost
    // octet in DB to zero.
    DB[0] &= ~(0xFF << (8 - LowestBits));

    // Step 10. If the emLen - hLen - sLen - 2 leftmost octets of DB are not
    // zero or if the octet at position emLen - hLen - sLen - 1 (the leftmost
    // or lower position is "position 1") does not have hexadecimal value
    // 0x01, output "inconsistent" and stop.)

    for(i = 0; DB[i] == 0 && i < (emLen - hLen - sLen - 2); i++)
    {
        if(DB[i] != 0)
        {
            return NvBootError_SE_RsaPssVerify_Inconsistent;
        }
    }
    // if octet at position emLen - hLen - sLen - 1
    // e.g. 256 - 32 - 32 - 1 = 191th position
    // position 191 is 190th element of the array, so subtract by 1 more.
    if(DB[emLen - hLen - sLen - 1 - 1] != 0x1)
    {
        return NvBootError_SE_RsaPssVerify_Inconsistent;
    }

    // Step 11. Let salt be the last sLen octets of DB.

    // Step 12. Let M' = 0x 00 00 00 00 00 00 00 00 || mHash || salt;
    // Set eight initial octets to 0.
    NvBootUtilMemset(&M_Prime[0], 0, 8);
    NvBootUtilMemcpy(&M_Prime[8], &mHash[0], hLen);
    // Copy salt to M_Prime. Note: DB is an octet string of length
    // emLen - hLen - 1. Subtract sLen from DB length to get salt location.
    NvBootUtilMemcpy(&M_Prime[8+hLen], &DB[(emLen - hLen - 1) - sLen], hLen);

    // Step 13. Let H' = Hash(M')
    //NvBootSeSHAHash(NvU32 *pInputMessage, NvU32 InputMessageSizeBytes, NvU32 *pOutputDestination, NvU8 HashAlgorithm)
    NvBootSeSHAHash((NvU32 *) &M_Prime, 8 + hLen + sLen, NULL, &H_Prime[0], HashAlgorithm);

    while(NvBootSeIsEngineBusy())
        ;

    // Step 14. If H = H' output "consistent". Otherwise, output "inconsistent".
    if(NvBootSeCompareBytes(H, (NvU8 *) &H_Prime[0], hLen) == NV_TRUE)
    {
        return NvBootError_SE_Signature_Valid;
    } else {
        return NvBootError_SE_RsaPssVerify_Inconsistent;
    }
}

/**
 *
 * Disable SE from accepting all OPERATION.OP commands. All key table
 * reads are masked. All other reg reads allowed and all reg writes allowed.
 * SE is disabled until the next chip reset.
 *
 */
void
NvBootSeDisableEngine(void) 
{
    // Don't do read-modify-write for fear of bad uninitialized values. 
    // Unless required for proper operations immediately after reset, most
    // reigsters aren't reset to a known value. 
    // Use the default reset value here. We can exploit the fact that 
    // the bits in SE_SE_SECURITY are sticky to DISABLE/SECURE/TRUE so if we write 
    // them to their non-sticky value the write won't take effect. 
    NvU32 SeConfigReg = SE_SE_SECURITY_0_RESET_VAL;

    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, SE_SECURITY, SE_ENG_DIS, TRUE, SeConfigReg);   
    NvBootSetSeReg(SE_SE_SECURITY_0, SeConfigReg);

    return;
}

/**
 * Load SRK from Secure Scratch Registers
 *
 */
void
NvBootSePmcLoadSrkFromSecureScratch(NvU32 *pSrk)
{
    /**
     * Per SE IAS and ASIC, a CTX_SAVE operation with DST = SRK will save the 
     * SRK at PMC Secure Scratch Registers SCRATCH4-7. 
     */
    // APBDEV_PMC_SECURE_SCRATCH4_0_SRK_0_SRK0_RANGE
    *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH4_0);
    // APBDEV_PMC_SECURE_SCRATCH5_0_SRK_0_SRK1_RANGE
    *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH5_0);
    // APBDEV_PMC_SECURE_SCRATCH6_0_SRK_0_SRK2_RANGE
    *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH6_0);
    // APBDEV_PMC_SECURE_SCRATCH7_0_SRK_0_SRK3_RANGE
    *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH7_0);

    return;
}

/**
 * Disable read access for a particular key slot, OIV, or UIV. 
 *
 */
void NvBootSeDisableKeySlotReadAccess(NvU8 KeySlot, NvU8 KeySlotType)
{
    NvU8 DisableValue = 0;

    // Since KEYTABLE_ACCESS_* is the same for slots 0-15, we can use
    // slot 0's reset value. 
    // Use the default reset value here. We can exploit the fact that 
    // the bits in KEYTABE_ACCESS are sticky to DISABLE so if we write 
    // them to their non-sticky value the write won't take effect. 
    DisableValue = SE_CRYPTO_KEYTABLE_ACCESS_0_RESET_VAL; 

    switch(KeySlotType) 
    {   
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3:
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_4_7:
            DisableValue = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_ACCESS, KEYREAD, DISABLE, DisableValue);
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS:
            DisableValue = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_ACCESS, OIVREAD, DISABLE, DisableValue);
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS:
            DisableValue = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_ACCESS, UIVREAD, DISABLE, DisableValue);
            break;
            
    }

    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ACCESS_0 + (KeySlot * 4), DisableValue);

    return;
}

/**
 * Disable write access for a particular key slot, OIV, or UIV. 
 *
 */
void NvBootSeDisableKeySlotWriteAccess(NvU8 KeySlot, NvU8 KeySlotType)
{
    NvU8 DisableValue = 0;

    // Since KEYTABLE_ACCESS_0 is the same for slots 0-15, we can use
    // slot 0's reset value. 
    // Use the default reset value here. We can exploit the fact that 
    // the bits in KEYTABE_ACCESS are sticky to DISABLE so if we write 
    // them to their non-sticky value the write won't take effect. 
    DisableValue = SE_CRYPTO_KEYTABLE_ACCESS_0_RESET_VAL; 

    switch(KeySlotType) 
    {   
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3:
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_4_7:
            DisableValue = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_ACCESS, KEYUPDATE, DISABLE, DisableValue);
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS:
            DisableValue = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_ACCESS, OIVUPDATE, DISABLE, DisableValue);
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS:
            DisableValue = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_ACCESS, UIVUPDATE, DISABLE, DisableValue);
            break;
            
    }

    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ACCESS_0 + (KeySlot * 4), DisableValue);

    return;
}

/**
 * Read an AES Key or IV to an AES key slot in the SE.
 *
 * @param KeySlot SE key slot number.
 * @param KeySize AES keysize. Use SE_MODE_PKT_AESMODE to specify KeySize.
 *                (KeySize can only be 128-bit if IV is selected).
 * @param KeyType Specify if this is a key, original IV or updated IV. Use SE_CRYPTO_KEYIV_PKT
 *                to specify the type. WORD_QUAD_KEYS_0_3 and WORD_QUAD_KEYS_4_7 for keys,
 *                SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS for original IV,
 *                SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS for updated IV.
 * @param KeyData Pointer to key data. Must be valid memory location.
 *
 * KeySize must be 128-bit for IVs.
 *
 */
void NvBootSeKeySlotReadKeyIV(NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData)
{
    NvU32   SeCryptoKeytableAddr = 0;
    NvU8    KeyIvSel = 0;
    NvU8    IvSel = 0;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);

    switch(KeyType)
    {
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3:
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_4_7:
            KeyIvSel = SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_KEY;
            IvSel = 0; // Don't care in this case.
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS:
            KeyIvSel = SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_IV;
            IvSel = SE_CRYPTO_KEYIV_PKT_IV_SEL_ORIGINAL;
            KeySize = SE_MODE_PKT_AESMODE_KEY128; // Force 128-bit key size
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS:
            KeyIvSel = SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_IV;
            IvSel = SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS;
            KeySize = SE_MODE_PKT_AESMODE_KEY128; // Force 128-bit key size
            break;
        default:
            NV_ASSERT(0);
    }

    /**
     * First, SE_CRYPTO_KEYTABLE_ADDR has to be written with the keyslot
     * configuration for the particular KEY_WORD/IV_WORD you are reading
     * from. The data to be read from the keyslot KEY_WORD/IV_WORD is then
     * read from SE_CRYPTOI_KEYTABLE_DATA.
     */
    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  0,
                                                  IvSel,
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  1,
                                                  IvSel,
                                                  1);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  2,
                                                  IvSel,
                                                  2);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  3,
                                                  IvSel,
                                                  3);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    // if writing a 128-bit key or IV, return. Writes to IV must specify 128-bit
    // key size.
    if (KeySize == SE_MODE_PKT_AESMODE_KEY128)
    {
        return;
    }

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  4,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  5,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    if (KeySize == SE_MODE_PKT_AESMODE_KEY192)
    {
        return;
    }
    // Must be a 256-bit key now.

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  6,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  7,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeReg(SE_CRYPTO_KEYTABLE_DATA_0);

    return;
}

/**
 * Write an AES Key or IV to an AES key slot in the SE.
 *
 * @param KeySlot SE key slot number.
 * @param KeySize AES keysize. Use SE_MODE_PKT_AESMODE to specify KeySize.
 *                (KeySize can only be 128-bit if IV is selected).
 * @param KeyType Specify if this is a key, original IV or updated IV. Use SE_CRYPTO_KEYIV_PKT
 *                to specify the type. WORD_QUAD_KEYS_0_3 and WORD_QUAD_KEYS_4_7 for keys,
 *                SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS for original IV,
 *                SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS for updated IV.
 * @param KeyData Pointer to key data. Must be valid memory location with valid
 *                data. If KeyData == 0, this function will set the particular
 *                KeySlot to all zeroes.
 *
 * KeySize must be 128-bit for IVs.
 *
 * If KeyData = 0, this function will set the particular KeySlot or IV to all zeroes.
 *
 */
void NvBootSeKeySlotWriteKeyIV(NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData)
{

    NvU32   SeCryptoKeytableAddr = 0;
    NvU8    KeyIvSel = 0;
    NvU8    IvSel = 0;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);

    switch(KeyType)
    {
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3:
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_4_7:
            KeyIvSel = SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_KEY;
            IvSel = 0; // Don't care in this case.
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS:
            KeyIvSel = SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_IV;
            IvSel = SE_CRYPTO_KEYIV_PKT_IV_SEL_ORIGINAL;
            KeySize = SE_MODE_PKT_AESMODE_KEY128; // Force 128-bit key size
            break;
        case    SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS:
            KeyIvSel = SE_CRYPTO_KEYIV_PKT_KEYIV_SEL_IV;
            IvSel = SE_CRYPTO_KEYIV_PKT_IV_SEL_UPDATED;
            KeySize = SE_MODE_PKT_AESMODE_KEY128; // Force 128-bit key size
            break;
        default:
            NV_ASSERT(0);
    }

    /**
     * First, SE_CRYPTO_KEYTABLE_ADDR has to be written with the keyslot
     * configuration for the particular KEY_WORD/IV_WORD you are writing to.
     * The data to be written to the keyslot KEY_WORD/IV_WORD is then
     * written to SE_CRYPTOI_KEYTABLE_DATA.
     */
    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  0,
                                                  IvSel,
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  1,
                                                  IvSel,
                                                  1);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  2,
                                                  IvSel,
                                                  2);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  3,
                                                  IvSel,
                                                  3);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    // if writing a 128-bit key or IV, return. Writes to IV must specify 128-bit
    // key size.
    if (KeySize == SE_MODE_PKT_AESMODE_KEY128)
    {
        return;
    }

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  4,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  5,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    if (KeySize == SE_MODE_PKT_AESMODE_KEY192)
    {
        return;
    }

    // Must be a 256-bit key now.

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  6,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  7,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    return;
}

void 
NvBootSeLockSbk(void)
{
    // Disable read access to the SE SBK key slot. SSK key permissions for the 
    // reserved SE key slot handled in NvBootSskLockSsk. 
    NvBootSeDisableKeySlotReadAccess(NvBootSeAesKeySlot_SBK, 
                                     SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
    return;
}

/**
 *  Compare Known Pattern to the pattern decrypted by the SE.
 */
NvBool  NvBootSeCheckKnownPattern(NvU8 *pPattern)
{   
    NvU32   i;

    for (i = 0; i < NVBOOT_SE_KNOWN_PATTERN_LENGTH_BYTES; i++)
    {
        if((*pPattern++) != NvBootSeContextKnownPattern[i])
        {
            return NV_FALSE;
        }
    }
    return NV_TRUE;
}


/**
 * Parse decrypted context in IRAM into usable format.
 */
static void NvBootSeParseDecryptedContext(
            NvBootSeLp0Context *NvBootSeDecryptedContext,
            NvBootSeContextStickyBitsBuffer *pNvBootSeContextStickyBitsBuffer)
{
    NvBootSeLp0ContextStickyBits *pSeLp0ContextStickyBits = &NvBootSeDecryptedContext->SeContextStickyBits;

    /**
     * Parse sticky bits information into NvBootSeContextStickyBitsBuffer.
     *
     * Assume the following format for the decrypted sticky bits:
     * (descending address)
     *
     * { 114'd0,
     * SE_RSA_KEYTABLE_ACCESS_1[2:0],
     * SE_RSA_KEYTABLE_ACCESS_0[2:0],
     * SE_RSA_SECURITY_PERKEY_0[1:0],
     * SE_CRYPTO_KEYTABLE_ACCESS_15[6:0],
     * SE_CRYPTO_KEYTABLE_ACCESS_14[6:0],
     * ... ,
     * SE_CRYPTO_KEYTABLE_ACCESS_0[6:0],
     * SE_CRYPTO_SECURITY_PERKEY_0[15:0],
     * SE_TZRAM_SECURITY_0[1:0],
     * SE_SECURITY_0[16],
     * SE_SECURITY_0[2:0] }
     *
     * Assumes: We have decrypted the encrypted context into
     *          a contiguous block in IRAM-A starting at offset 0
     *
     *  Note: Stack should be at top 8KB of 64KB IRAM bank A
     *        on FPGA
     */

    // Parse the packed sticky bits blocks into useable variables for later restoration
    // according to the above table
    pNvBootSeContextStickyBitsBuffer->SE_SE_SECURITY =
        (SE_SECURITY_0_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[0]) >>
         SE_SECURITY_0_LP0_CONTEXT_SHIFT_0;

    // If bit 3 of the sticky first dword of sticky bits is set, put it in the correct
    // position (bit 16 of SE_SE_SECURITY_0)
    if(pNvBootSeContextStickyBitsBuffer->SE_SE_SECURITY & SE_SECURITY_0_SE_SOFT_SETTING_STICKY_BIT_FIELD)
    {
        pNvBootSeContextStickyBitsBuffer->SE_SE_SECURITY &= (SE_SE_SECURITY_0_PERKEY_SETTING_FIELD | SE_SE_SECURITY_0_SE_ENG_DIS_FIELD | SE_SE_SECURITY_0_SE_HARD_SETTING_FIELD);
        pNvBootSeContextStickyBitsBuffer->SE_SE_SECURITY |= 0x01 << SE_SE_SECURITY_0_SE_SOFT_SETTING_SHIFT;
    }

    pNvBootSeContextStickyBitsBuffer->SE_TZRAM_SECURITY =
        (SE_TZRAM_SECURITY_0_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[0]) >>
         SE_TZRAM_SECURITY_0_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_SECURITY_PERKEY =
        (SE_CRYPTO_SECURITY_PERKEY_0_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[0]) >>
         SE_CRYPTO_SECURITY_PERKEY_0_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[0] =
        (SE_CRYPTO_KEYTABLE_ACCESS_0_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[0]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_0_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[1] =
        ((SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[0]) >>
          SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_SHIFT_0) |
        ((SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_MASK_1 & pSeLp0ContextStickyBits->StickyBits[1]) <<
          SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_SHIFT_1);

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[2] =
        (SE_CRYPTO_KEYTABLE_ACCESS_2_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[1]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_2_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[3] =
        (SE_CRYPTO_KEYTABLE_ACCESS_3_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[1]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_3_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[4] =
        (SE_CRYPTO_KEYTABLE_ACCESS_4_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[1]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_4_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[5] =
        (SE_CRYPTO_KEYTABLE_ACCESS_5_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[1]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_5_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[6] =
        (SE_CRYPTO_KEYTABLE_ACCESS_6_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[2]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_6_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[7] =
        (SE_CRYPTO_KEYTABLE_ACCESS_7_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[2]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_7_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[8] =
        (SE_CRYPTO_KEYTABLE_ACCESS_8_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[2]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_8_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[9] =
        (SE_CRYPTO_KEYTABLE_ACCESS_9_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[2]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_9_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[10] =
        ((SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[2]) >>
          SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_SHIFT_0) |
        ((SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_MASK_1 & pSeLp0ContextStickyBits->StickyBits[3]) <<
          SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_SHIFT_1);

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[11] =
        (SE_CRYPTO_KEYTABLE_ACCESS_11_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[3]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_11_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[12] =
        (SE_CRYPTO_KEYTABLE_ACCESS_12_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[3]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_12_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[13] =
        (SE_CRYPTO_KEYTABLE_ACCESS_13_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[3]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_13_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[14] =
        (SE_CRYPTO_KEYTABLE_ACCESS_14_LP0_CONTEXT_MASK_0  & pSeLp0ContextStickyBits->StickyBits[3]) >>
         SE_CRYPTO_KEYTABLE_ACCESS_14_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_CRYPTO_KT_ACCESS[15] =
        ((SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[3]) >>
          SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_SHIFT_0) |
        ((SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_MASK_1 & pSeLp0ContextStickyBits->StickyBits[4]) <<
          SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_SHIFT_1);

    pNvBootSeContextStickyBitsBuffer->SE_RSA_SECURITY_PERKEY =
        (SE_RSA_SECURITY_PERKEY_0_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[4]) >>
         SE_RSA_SECURITY_PERKEY_0_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_RSA_KT_ACCESS[0] =
        (SE_RSA_KEYTABLE_ACCESS_0_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[4]) >>
         SE_RSA_KEYTABLE_ACCESS_0_LP0_CONTEXT_SHIFT_0;

    pNvBootSeContextStickyBitsBuffer->SE_RSA_KT_ACCESS[1] =
        (SE_RSA_KEYTABLE_ACCESS_1_LP0_CONTEXT_MASK_0 & pSeLp0ContextStickyBits->StickyBits[4]) >>
         SE_RSA_KEYTABLE_ACCESS_1_LP0_CONTEXT_SHIFT_0;

    return;
}

/**
 *  Program the SE to decrypt our saved context.
 *
 *  pEncryptedContext = location of encrypted context
 *  pDecryptedContext = location where SE should deecrypt to
 */
void    NvBootSeAesDecryptContext(NvU32 *pEncryptedContext, NvU32 *pDecryptedContext)
{
    NvBootSeAesDecrypt(NvBootSeAesKeySlot_0,
                       SE_MODE_PKT_AESMODE_KEY128,
                       NV_TRUE,
                       sizeof(NvBootSeLp0Context) / NVBOOT_SE_AES_BLOCK_LENGTH_BYTES,
                       (NvU8 *) pEncryptedContext,
                       (NvU8 *) pDecryptedContext);

    return;
}

/** 
 *  Routine to erase the decrypted data after restoration into the SE. 
 */
void NvBootSeEraseDecryptedData(NvU8 *pData, NvU32 Size) 
{
    NvBootUtilMemset(pData, 0, Size);
}

NvBootError NvBootLP0ContextRestore(void)
{
    NvU32 Srk[NVBOOT_SE_SRK_KEY_LENGTH];
    NvBootSeContextStickyBitsBuffer s_NvBootSeContextStickyBitsBuffer;
    NvBootSeLp0Context *pNvBootSeDecryptedContext = (NvBootSeLp0Context *) NVBOOT_SE_DECRYPTED_CONTEXT_START;
    NvU32  Slot; 

    /** 
     * 1. SW copies the 128-bit SRK from PMC secure scratch registers to some 
     * arbitrary key-slot n in SE and sets the original and updated IVs in this key-slot to
     * zeroes. 
     */

    // Load SRK from PMC secure scratch registers
    NvBootSePmcLoadSrkFromSecureScratch(&Srk[0]);

    // Write the SRK into key slot 0
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_0, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, &Srk[0]);

    // Initialize OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_0, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    // Initialize UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_0, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 0);

    /**
     * 2. Proram SE to decrypt the encrypted context in memory using the SRK in 
     * key-slot 0 and output it to some arbitrary memory location
     */

    // APBDEV_PMC_SCRATCH43_0_SE_ENCRYPTED_CONTEXT_RANGE
    NvBootSeAesDecryptContext((NvU32 *) NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH43_0), (NvU32 *) pNvBootSeDecryptedContext);

    // Parse the decrypted context into a more easily usable format
    NvBootSeParseDecryptedContext( 
            pNvBootSeDecryptedContext,
            &s_NvBootSeContextStickyBitsBuffer);

    /**
     * 3. SW checks decyrpted known pattern. If it matches the expected known 
     * pattern, proceed. 
     */
    // if the known pattern check fails, disable SE as much as possible and
    // skip the restore of the SE. Let OS decide what to do after failed SE
    // context restore. 
    if  (!NvBootSeCheckKnownPattern(&pNvBootSeDecryptedContext->KnownPattern[0]))
    {
        // NOTE: Order matters here!!
        // For safety, clear all key slots. Even after an LP0 transition, 
        // the keyslots may not be fully cleared if the time spent in LP0
        // was very short. 
        NvBootSeClearAllKeySlots();

        // Disable read/write access to the key slots. Set each key slot as 
        // ARSE_SECURE. 
        NvBootSeDisableAllKeySlotReadWrite();

        // Disable TZRAM (Set TZRAM as ARSE_SECURE and disable reads and writes).
        NvBootSeDisableTzram();

        // Disable all crypto operations and expanded key table access. Force 
        // access to PERKEY_SETTING and SE register accesses/operations
        // in secure mode only. 
        NvBootSeDisableSe();

        // Even though known pattern check failed, let's clear the bad 
        // decrypted data from IRAM. 
        NvBootSeEraseDecryptedData((NvU8 *) pNvBootSeDecryptedContext, sizeof(NvBootSeLp0Context));

        return NvBootError_SE_Context_Restore_Failure;
    }
    
    /**
     * 4. SW restores keys to key-slot n in SE key table
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeKeySlotWriteKeyIV(Slot, 
                                  SE_MODE_PKT_AESMODE_KEY256,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                                  &pNvBootSeDecryptedContext->Aes256Key[Slot].Key[0]); 
    }

    /**
     * 5. SW restores original IV to key-slot n in SE key table
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeKeySlotWriteKeyIV(Slot, 
                                  SE_MODE_PKT_AESMODE_KEY128, 
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 
                                  &pNvBootSeDecryptedContext->OriginalIv[Slot].Iv[0]);
    }

    /**
     * 6. SW restores updated/current IV to key-slot n in SE key table
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeKeySlotWriteKeyIV(Slot, 
                                  SE_MODE_PKT_AESMODE_KEY128, 
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 
                                  &pNvBootSeDecryptedContext->UpdatedIv[Slot].Iv[0]);
    }

    /**
     * 7. Restore SSK from key slot "SeAesKeySlot_SSK" to BSEA and BSEV engines
     *
     * This step is no longer necessary for T148 since the BSEA/BSEV engines
     * have been removed for BR use.
     */

    /**
     * 8. Restore the RSA keys to their respective key slots. 
     *
     */
    for (Slot = NvBootSeRsaKeySlot_1; Slot < NvBootSeRsaKeySlot_Num; Slot++) 
    {
        NvBootSeRsaWriteKey(&pNvBootSeDecryptedContext->RsaKey2048[Slot].Modulus[0], 
                            ARSE_RSA_MAX_MODULUS_SIZE, 
                            ARSE_RSA_MAX_EXPONENT_SIZE, 
                            Slot);        
    }
    
    /**
     * 9. SW must erase the decrypted data stored in memory.
     */
    NvBootSeEraseDecryptedData((NvU8 *) pNvBootSeDecryptedContext, sizeof(NvBootSeLp0Context));    
    
    /**
     * 10. SW restores decrypted key table access control sticky-bits via 
     * register writes to the respective SE registers. 
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSetSeReg( 
            SE_CRYPTO_KEYTABLE_ACCESS_0 + (Slot * 4), 
            s_NvBootSeContextStickyBitsBuffer.SE_CRYPTO_KT_ACCESS[Slot]);
    }

    /**
     * 10. SW restores SE_RSA_KEYTABLE_ACCESS_0 and SE_RSA_KEYTABLE_ACCESS_1
     * registers.
     */
    for (Slot = NvBootSeRsaKeySlot_1; Slot < NvBootSeRsaKeySlot_Num; Slot++)
    {
        NvBootSetSeReg(
            SE_RSA_KEYTABLE_ACCESS_0 + (Slot * 4),
            s_NvBootSeContextStickyBitsBuffer.SE_RSA_KT_ACCESS[Slot]);
    }

    // Restore SE_CRYPTO_SECURITY_PERKEY_0
    NvBootSetSeReg(
        SE_CRYPTO_SECURITY_PERKEY_0,
        s_NvBootSeContextStickyBitsBuffer.SE_CRYPTO_SECURITY_PERKEY);

    // Restore SE_RSA_SECURITY_PERKEY_0
    NvBootSetSeReg(
        SE_RSA_SECURITY_PERKEY_0,
        s_NvBootSeContextStickyBitsBuffer.SE_RSA_SECURITY_PERKEY);

    // Restore SE_TZRAM_SECURITY_0
    NvBootSetSeReg(
        SE_TZRAM_SECURITY_0,
        s_NvBootSeContextStickyBitsBuffer.SE_TZRAM_SECURITY);

    // Restore SE_SE_SECURITY_0
    NvBootSetSeReg(
        SE_SE_SECURITY_0,
        s_NvBootSeContextStickyBitsBuffer.SE_SE_SECURITY);

    // Clear the sticky bits buffer memory
    NvBootUtilMemset(&s_NvBootSeContextStickyBitsBuffer, 0, sizeof(NvBootSeContextStickyBitsBuffer));

    /**
     * 11. Clear the secure scratch registers associated with the SRK to 0. 
     *
     * Even though the SRK is generated at every LP0 entry, we don't want any
     * untrusted SW after the Boot ROM to read it. 
     *
     */ 
    NvBootSeClearSrkSecureScratch();

    return NvBootError_Success;
}

void NvBootSeAesCmacGenerateSubkey(NvU8 KeySlot, NvU8 KeySize, NvU32 *pK1, NvU32 *pK2)
{
    NvU32   SeConfigReg = 0;
    const NvU32   ConstZero[NVBOOT_SE_AES_BLOCK_LENGTH];
    NvU32   L[NVBOOT_SE_AES_BLOCK_LENGTH];
    NvU32   msbL;
    NvU8    *pK;
    SingleSeLinkedList InputLL;
    SingleSeLinkedList OutputLL;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT(pK1 != NULL);
    NV_ASSERT(pK2 != NULL);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );

    // AES block of zeroes.
    NvBootUtilMemset(&ConstZero, 0, NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);

    // Set up SE engine for AES encrypt + CMAC hash.
    NvBootSeSetupOpMode(SE_OP_MODE_AES_CMAC_HASH,
                        NV_TRUE,
                        NV_TRUE,
                        SE_CONFIG_0_DST_MEMORY,
                        KeySlot,
                        KeySize);

    // Setup SE input and output linked lists.
    InputLL.LastBufferNumber = 0;
    InputLL.LLElement.StartByteAddress = (NvU32) &ConstZero;
    InputLL.LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
    OutputLL.LastBufferNumber = 0;
    OutputLL.LLElement.StartByteAddress = (NvU32) &L;
    OutputLL.LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLL);
    NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &OutputLL);

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // 16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // SE_CRYPTO_LAST_BLOCK = 0
    NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, 0);

    // Initialize key slot OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(KeySlot,
                              KeySize,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                              0);

    // Initialize key slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(KeySlot,
                              KeySize,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                              0);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

    // Poll for OP_DONE.
    while(NvBootSeIsEngineBusy())
        ;

    // L := AES-{128,192,256}(K, const_Zero);
    // if MSB(L) is equal to 0,
    // then K1 := L << 1;
    // else K1 := (L << 1) XOR const_Rb;
    msbL = NVBOOT_SE_AES_CMAC_IS_MSB_SET((NvU8) L[0]);
    NvBootUtilMemcpy(pK1, &L[0], NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);
    NvBootSeLeftShiftOneBit((NvU8 *)pK1, NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);

    pK = (NvU8 *) pK1;
    if(msbL)
        *(pK + NVBOOT_SE_AES_BLOCK_LENGTH_BYTES - 1) ^= NVBOOT_SE_AES_CMAC_CONST_RB;

    // if MSB(K1) is equal to 0
    // then K2 := K1 << 1;
    // else K2 := (K1 << 1) XOR const_Rb;
    NvBootUtilMemcpy(pK2, pK1, NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);
    msbL = NVBOOT_SE_AES_CMAC_IS_MSB_SET((NvU8) *pK2);
    NvBootSeLeftShiftOneBit((NvU8 *)pK2, NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);

    pK = (NvU8 *) pK2;
    if(msbL)
        *(pK + NVBOOT_SE_AES_BLOCK_LENGTH_BYTES - 1) ^= NVBOOT_SE_AES_CMAC_CONST_RB;

    return;
}

void
NvBootSeAesCmacHashBlocks (NvU32 *pK1, NvU32 *pK2, NvU32 *pInputMessage, NvU8 *pHash, NvU8 KeySlot, NvU8 KeySize, NvU32 NumBlocks, NvBool FirstChunk, NvBool LastChunk)
{

    NvU32   *Src = (NvU32 *) pInputMessage;
    const NvU32 WordOffsetToLastBlock = (NumBlocks-1) * NVBOOT_SE_AES_BLOCK_LENGTH;
    const NvU32 ByteOffsetToLastBlock = (NumBlocks-1) * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
    NvU32   i;
    NvU32   LastBlockBuffer[NVBOOT_SE_AES_BLOCK_LENGTH];
    NvU32   SeConfigReg;
    SeLinkedList InputLinkedList;
    SingleSeLinkedList OutputLinkedList;

    NV_ASSERT(pK1 != NULL);
    NV_ASSERT(pK2 != NULL);
    NV_ASSERT(pInputMessage != NULL);
    NV_ASSERT(pHash != NULL);
    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );

    if(NumBlocks)
    {
        if(FirstChunk)
        {
            // Clear IVs of SE keyslot.
            // Initialize key slot OriginalIv[127:0] to zero
            NvBootSeKeySlotWriteKeyIV(KeySlot,
                                      KeySize,
                                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                                      0);

            // Initialize key slot UpdatedIV[127:0] to zero
            NvBootSeKeySlotWriteKeyIV(KeySlot,
                                      KeySize,
                                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                                      0);
        }

        if(LastChunk)
        {
            if(NumBlocks > 1)
            {
                // Check if SE is idle.
                while(NvBootSeIsEngineBusy())
                    ;

                // Encrypt the input data for blocks zero to NumBLocks - 1.
                // Setup SE engine parameters.
                // Use the SE_HASH_RESULT_* output registers to store the intermediate result.
                // Use UPDATED IV. Even if this is the first block, we can use UPDATED IV
                // instead of ORIGINAL IV since we just cleared the IVs above.
                NvBootSeSetupOpMode(SE_OP_MODE_AES_CMAC_HASH, NV_TRUE, NV_FALSE, SE_CONFIG_0_DST_HASH_REG, KeySlot, KeySize);
                // Generate an input linked list.
                NvBootSeGenerateLinkedList(&InputLinkedList, pInputMessage, ByteOffsetToLastBlock);
                // Set address of input linked list.
                NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

                // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
                // given in the SE IAS section 3.2.3.1 AES Input Data Size.
                // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
                // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
                // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
                // Since we want to encrypt NumBlocks - 1 blocks, it should be NumBlocks-1-1.
                NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, (NumBlocks-1)-1);

                /**
                 * Issue START command in SE_OPERATION.OP
                 */
                SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
                NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

                // Poll for OP_DONE.
                while(NvBootSeIsEngineBusy())
                    ;
            }

            // Process the last block with the correct subkey K1 or K2.
            for(i = 0; i < NVBOOT_SE_AES_BLOCK_LENGTH; i++)
            {
                LastBlockBuffer[i] = *(pK1+i) ^ *(Src + WordOffsetToLastBlock + i);
            }

            // Setup SE engine parameters.
            // Set output destination to MEMORY.
            // Set IV to UPDATED_IV because we are continuing a previous AES
            // operation.
            NvBootSeSetupOpMode(SE_OP_MODE_AES_CMAC_HASH, NV_TRUE, NV_FALSE, SE_CONFIG_0_DST_MEMORY, KeySlot, KeySize);
            // Generate an input linked list.
            //static NvBootSeGenerateLinkedList(SeLinkedList *pLinkedList, NvU32 *pStartAddress, NvU32 MessageSize)
            NvBootSeGenerateLinkedList(&InputLinkedList, LastBlockBuffer, NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);
            // Set address of input linked list.
            NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

            // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
            // given in the SE IAS section 3.2.3.1 AES Input Data Size.
            // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
            // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
            // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
            // Since we are only encrypting one block, set SE_CRYPTO_LAST_BLOCK_0 to 0.
            NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, 0);

            OutputLinkedList.LastBufferNumber = 0;
            OutputLinkedList.LLElement.StartByteAddress = (NvU32) pHash;
            OutputLinkedList.LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
            NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &OutputLinkedList);

            /**
             * Issue START command in SE_OPERATION.OP
             */
            SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
            NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

            // Poll for OP_DONE.
            while(NvBootSeIsEngineBusy())
                ;

        }
        else
        {
            // Check if SE is idle.
            while(NvBootSeIsEngineBusy())
                ;

            // Encrypt the input data for blocks zero to NumBLocks.
            // Setup SE engine parameters.
            // Use the SE_HASH_RESULT_* output registers to store the intermediate result.
            // Set IV to UPDATED_IV because we are continuing a previous AES
            // operation.
            NvBootSeSetupOpMode(SE_OP_MODE_AES_CMAC_HASH, NV_TRUE, NV_FALSE, SE_CONFIG_0_DST_HASH_REG, KeySlot, KeySize);
            // Generate an input linked list.
            //static NvBootSeGenerateLinkedList(SeLinkedList *pLinkedList, NvU32 *pStartAddress, NvU32 MessageSize)
            NvBootSeGenerateLinkedList(&InputLinkedList, pInputMessage, NumBlocks*NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);
            // Set address of input linked list.
            NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

            // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
            // given in the SE IAS section 3.2.3.1 AES Input Data Size.
            // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
            // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
            // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
            NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, NumBlocks-1);

            /**
             * Issue START command in SE_OPERATION.OP
             */
            SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
            NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);
        }
    }
}

void NvBootSeAesDecrypt (
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst)
{
    SingleSeLinkedList InputLinkedList;
    SingleSeLinkedList OutputLinkedList;
    NvU32   SeConfigReg;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT(NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES < NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES);
    NV_ASSERT(NumBlocks > 0);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );

    // Setup SE engine parameters for AES decrypt operation.
    NvBootSeSetupOpMode(SE_OP_MODE_AES_CBC,
                        NV_FALSE,
                        First,
                        SE_CONFIG_0_DST_MEMORY,
                        KeySlot,
                        KeySize);

    // Setup input linked list.
    InputLinkedList.LastBufferNumber = 0;
    InputLinkedList.LLElement.StartByteAddress = (NvU32) Src;
    InputLinkedList.LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of input linked list.
    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

    OutputLinkedList.LastBufferNumber = 0;
    OutputLinkedList.LLElement.StartByteAddress = (NvU32) Dst;
    OutputLinkedList.LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of output linked list.
    NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &OutputLinkedList);

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
    NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, NumBlocks-1);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

    // When called in the reader code, the UpdateCryptoStatus2 function will make sure no new
    // operations will start before the engine is idle.
    // Poll for OP_DONE.
    while(NvBootSeIsEngineBusy())
        ;

    return;
}

void NvBootSeAesEncrypt (
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst)
{
    SingleSeLinkedList InputLinkedList;
    SingleSeLinkedList OutputLinkedList;
    NvU32   SeConfigReg;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT(NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES < NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES);
    NV_ASSERT(NumBlocks > 0);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );

    // Setup SE engine parameters for AES encrypt operation.
    NvBootSeSetupOpMode(SE_OP_MODE_AES_CBC,
                        NV_TRUE,
                        First,
                        SE_CONFIG_0_DST_MEMORY,
                        KeySlot,
                        KeySize);

    // Setup input linked list.
    InputLinkedList.LastBufferNumber = 0;
    InputLinkedList.LLElement.StartByteAddress = (NvU32) Src;
    InputLinkedList.LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of input linked list.
    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

    OutputLinkedList.LastBufferNumber = 0;
    OutputLinkedList.LLElement.StartByteAddress = (NvU32) Dst;
    OutputLinkedList.LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of output linked list.
    NvBootSetSeReg(SE_OUT_LL_ADDR_0, (NvU32) &OutputLinkedList);

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
    NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, NumBlocks-1);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

    // When called in the reader code, the UpdateCryptoStatus2 function will make sure no new
    // operations will start before the engine is idle.
    // Poll for OP_DONE.
    while(NvBootSeIsEngineBusy())
        ;

    return;
}

void NvBootSeAesDecryptKeyIntoKeySlot (
        NvU8    SourceKeySlot,
        NvU8    SourceKeySize,
        NvU8    TargetKeySlot,
        NvU8    TargetKeySize,
        NvU8   *Src)
{
    SingleSeLinkedList InputLinkedList;
    NvU32   SeConfigReg;
    NvU32   SeCryptoKeytableDst = 0;

    NV_ASSERT(SourceKeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT(TargetKeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT( (SourceKeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (SourceKeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (SourceKeySize == SE_MODE_PKT_AESMODE_KEY256) );
    NV_ASSERT( (TargetKeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (TargetKeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (TargetKeySize == SE_MODE_PKT_AESMODE_KEY256) );
    NV_ASSERT(Src != NULL);

    // Initialize SE source IV slot OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(SourceKeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                      0);

    // Initialize SE source IV slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(SourceKeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      0);

    // Setup SE engine parameters for AES decrypt operation.
    NvBootSeSetupOpMode(SE_OP_MODE_AES_CBC,
                        NV_FALSE,
                        NV_TRUE,
                        SE_CONFIG_0_DST_KEYTABLE,
                        SourceKeySlot,
                        SourceKeySize);

    // Setup input linked list.
    InputLinkedList.LastBufferNumber = 0;
    InputLinkedList.LLElement.StartByteAddress = (NvU32) Src;
    InputLinkedList.LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
    // Only 1 block at a time here, so set this to 0.
    NvBootSetSeReg(SE_CRYPTO_LAST_BLOCK_0, 0);

    // Set address of input linked list.
    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

    // Set operation to decrypt into lower 128-bits of the key slot.
    SeCryptoKeytableDst = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_DST, WORD_QUAD, KEYS_0_3, SeCryptoKeytableDst);
    SeCryptoKeytableDst = NV_FLD_SET_DRF_NUM(SE, CRYPTO_KEYTABLE_DST, KEY_INDEX, TargetKeySlot, SeCryptoKeytableDst);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DST_0, SeCryptoKeytableDst);

     // Issue START command in SE_OPERATION.OP
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

    // Wait until engine becomes IDLE.
    while(NvBootSeIsEngineBusy())
        ;

    // If the target key size to be loaded directly into the SE key slot is
    // 128-bits, we are done.
    if(TargetKeySize == SE_MODE_PKT_AESMODE_KEY128)
        return;

    // Setup SE engine parameters for AES decrypt operation.
    NvBootSeSetupOpMode(SE_OP_MODE_AES_CBC,
                        NV_FALSE,
                        NV_FALSE,
                        SE_CONFIG_0_DST_KEYTABLE,
                        SourceKeySlot,
                        SourceKeySize);

    // Setup input linked list.
    InputLinkedList.LastBufferNumber = 0;
    InputLinkedList.LLElement.StartByteAddress = (NvU32) Src + NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
    InputLinkedList.LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of input linked list.
    NvBootSetSeReg(SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList);

    // Set operation to decrypt into upper 128-bits of the key slot.
    SeCryptoKeytableDst = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_DST, WORD_QUAD, KEYS_4_7, SeCryptoKeytableDst);
    SeCryptoKeytableDst = NV_FLD_SET_DRF_NUM(SE, CRYPTO_KEYTABLE_DST, KEY_INDEX, TargetKeySlot, SeCryptoKeytableDst);
    NvBootSetSeReg(SE_CRYPTO_KEYTABLE_DST_0, SeCryptoKeytableDst);

     // Issue START command in SE_OPERATION.OP
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeReg(SE_OPERATION_0, SeConfigReg);

    // Wait until engine becomes IDLE.
    while(NvBootSeIsEngineBusy())
        ;

    return;
}
