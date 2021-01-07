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
#include "arahb_arbc.h"
#include "arpka1.h"
#include "project.h"
#include "nvboot_bit.h"
#include "nvboot_context_int.h"
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
#include "nvboot_pka_int.h"
#include "nvboot_ssk_int.h"
#include "nvboot_util_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_rng_int.h"
#include "nvboot_crypto_sha_param.h"
#include "nvboot_kcv_int.h"
#include "nvboot_ahb_int.h"

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

extern NvBootInfoTable BootInfoTable;
extern NvBootContext     Context;

/**
 *  Need 2 instances for for parallel execution of SE1/SE2
 */
NV_ALIGN (4) static SingleSeLinkedList InputLinkedList[NvBootSeInstance_Se2+1];
NV_ALIGN (4) static SingleSeLinkedList OutputLinkedList[NvBootSeInstance_Se2+1];

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
NvBootSeClearSrkSecureScratch(NvBootSeInstance SeInstance)
{
    if(SeInstance ==  NvBootSeInstance_Se1)
    {
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH4_0, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH5_0, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH6_0, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH7_0, 0);
    }
    else
    {
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH120_0, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH121_0, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH122_0, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH123_0, 0);
    }

    return;
}

static void
NvBootSeInstanceClearAllKeySlots(NvBootSeInstance SeInstance) 
{
    NvU32 Slot;
    // Clear all key slots to zero. 
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeInstanceKeySlotWriteKeyIV(SeInstance, Slot, SE_MODE_PKT_AESMODE_KEY256, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, 0);
        NvBootSeInstanceKeySlotWriteKeyIV(SeInstance, Slot, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 0);
        NvBootSeInstanceKeySlotWriteKeyIV(SeInstance, Slot, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);
    }

    return;
}

static void
NvBootSeInstanceDisableAllKeySlotReadWrite(NvBootSeInstance SeInstance) 
{
    NvU32 Slot = 0;

    // Disable key slot read and write for all key slots. 
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeInstanceDisableKeySlotReadAccess(SeInstance, Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
        NvBootSeInstanceDisableKeySlotWriteAccess(SeInstance, Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
            
        NvBootSeInstanceDisableKeySlotReadAccess(SeInstance, Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS);
        NvBootSeInstanceDisableKeySlotWriteAccess(SeInstance, Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS);

        NvBootSeInstanceDisableKeySlotReadAccess(SeInstance, Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS);
        NvBootSeInstanceDisableKeySlotWriteAccess(SeInstance, Slot, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS);
    }
    
    // Set each keyslot as SECURE (can be accessed in secure mode only). 
    // Not absolutely necessary after we cleared and locked down the key slots, 
    // but lets be paranoid. 
    NvBootSetSeInstanceReg(SeInstance, SE_CRYPTO_SECURITY_PERKEY_0, SE_CRYPTO_SECURITY_PERKEY_0_ALL_SECURE);

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
NvBootSeInstanceDisableSe(NvBootSeInstance Instance)
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
    NvBootSetSeInstanceReg(Instance, SE_SE_SECURITY_0, SeConfigReg);

    return;
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

/**
 * Generate a linked list structure for consumption by the SE. 
 *
 * @param SeLinkedList  Pointer to SeLinkedList struct
 * @param pStartAddress Pointer to start address of data to be processed by the SE
 * @param MessageSize   Size in bytes of the data to be processed by the SE
 */
static void NvBootSeGenerateLinkedList(SeLinkedList *pLinkedList, NvU32 *pStartAddress, NvU32 MessageSize) 
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
}
// --------------------- Static Buffers ----------------------------------------

static NvBootSe1Lp0Context Se1DecryptedContext;
static NvBootSe2Lp0Context Se2DecryptedContext;

// ---------------------Public Functions Definitions----------------------------

/**
 *  Get SE config register
 */
NvU32 
NvBootGetSeInstanceReg(NvBootSeInstance Instance, NvU32 Reg)
{   
    // Default to legacy instance i.e. SE
    NvU32 BaseAddress = NV_ADDRESS_MAP_SE_BASE;
    
    if(Instance == NvBootSeInstance_Se2)
    {
        BaseAddress =  NV_ADDRESS_MAP_SE2_BASE;
    }
    return NV_READ32(BaseAddress + Reg);
}

/**
 *  Set SE config register
 */
void
NvBootSetSeInstanceReg(NvBootSeInstance Instance, NvU32 Reg, NvU32 Data)
{
    // Default to legacy instance i.e. NvBootSeInstance_Se1
    NvU32 BaseAddress = NV_ADDRESS_MAP_SE_BASE;
    
    if(Instance == NvBootSeInstance_Se2)
    {
        BaseAddress =  NV_ADDRESS_MAP_SE2_BASE;
    }
    NV_WRITE32(BaseAddress + Reg, Data);
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

    // NVSE IAS section "SE in Cold Boot".
    // "Override SLCG by writing 0x1 to SE_CLK_VR_ON in
    // CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0 register."
    // Need to clear this back to 0x0 at BR exit.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRB, SE_CLK_OVR_ON, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0,
               RegData);

    return;
}

NvBool NvBootSeIsAtomicSeContextSaveEnabled(NvBootSeInstance Instance)
{

    NvU8 Status = 0;
    NvU32 SeConfigReg = 0;

    SeConfigReg = NvBootGetSeInstanceReg(Instance, SE_CTX_SAVE_AUTO_0);
    Status = NV_DRF_VAL(SE, CTX_SAVE_AUTO, ENABLE, SeConfigReg);

    if(Status == SE_CTX_SAVE_AUTO_0_ENABLE_YES)
    {
        return  NV_TRUE;
    } else {
        return  NV_FALSE;
    }
}

void NvBootSeInstanceEnableAtomicSeContextSave(NvBootSeInstance Instance)
{
    // Random delay before enabling SE atomic context save.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    if( (NvBootGetSwCYA() & NVBOOT_SW_CYA_ATOMIC_SE_CONTEXT_SAVE_ENABLE) ||
        (NvBootFuseIsSeContextAtomicSaveEnabled()) )
    {
        // Enable atomic SE context save for Instance.
        uint32_t CtxSaveAutoReg = NvBootGetSeInstanceReg(Instance, SE_CTX_SAVE_AUTO_0);
        CtxSaveAutoReg = NV_FLD_SET_DRF_DEF(SE, CTX_SAVE_AUTO, ENABLE, YES, CtxSaveAutoReg);
        CtxSaveAutoReg = NV_FLD_SET_DRF_DEF(SE, CTX_SAVE_AUTO, LOCK, YES, CtxSaveAutoReg);
        NvBootSetSeInstanceReg(Instance, SE_CTX_SAVE_AUTO_0, CtxSaveAutoReg);

        // Program SE context blob locations into the assigned secure scratch
        // registers.
        // If SE1, program Secure scratch 117 <-- ARSE_TZRAM_CARVEOUT_ADDR_SE1 (0x7C04C000)
        // If SE2/PKA1, program Secure scratch 116 <-- ARSE_TZRAM_CARVEOUT_ADDR_SE2 (0x7C04D000)
        uint32_t PmcScratchLock = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE8_0);
        uint32_t PmcScratchLockNs = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE8_NS_0);
        if(Instance == NvBootSeInstance_Se1)
        {
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH117_0, ARSE_TZRAM_CARVEOUT_ADDR_SE1);
            PmcScratchLock = NV_FLD_SET_DRF_DEF(APBDEV_PMC, SEC_DISABLE8, WRITE117, ON, PmcScratchLock);
            PmcScratchLockNs = NV_FLD_SET_DRF_DEF(APBDEV_PMC, SEC_DISABLE8_NS, WRITE117, ON, PmcScratchLockNs);
        }
        else
        {
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH116_0, ARSE_TZRAM_CARVEOUT_ADDR_SE2);
            PmcScratchLock = NV_FLD_SET_DRF_DEF(APBDEV_PMC, SEC_DISABLE8, WRITE116, ON, PmcScratchLock);
            PmcScratchLockNs = NV_FLD_SET_DRF_DEF(APBDEV_PMC, SEC_DISABLE8_NS, WRITE116, ON, PmcScratchLockNs);
        }

        // Set both the non-secure disables and the TZ disables for completeness.
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE8_0, PmcScratchLock);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE8_NS_0, PmcScratchLockNs);
    }
}

NvBootError NvBootSeEnableAtomicSeContextSave()
{
    NvBootSeInstanceEnableAtomicSeContextSave(NvBootSeInstance_Se1);
    NvBootSeInstanceEnableAtomicSeContextSave(NvBootSeInstance_Se2);

    // Must return an NvBootError for use in a dispatcher table.
    return NvBootError_Success;
}

// PKA1 ref manual not in Tegra format. Also THRESHOLD field isn't defined.
#define PKA1_SLCG_CTRL_0_THRESHOLD_SHIFT _MK_SHIFT_CONST(22)
#define PKA1_SLCG_CTRL_0_THRESHOLD_FIELD _MK_FIELD_CONST(0x3ff, PKA1_SLCG_CTRL_0_THRESHOLD_SHIFT)
#define PKA1_SLCG_CTRL_0_THRESHOLD_RANGE 31:22
#define PKA1_SLCG_CTRL_0_CLK_OVR_ON_SHIFT _MK_SHIFT_CONST(0)
#define PKA1_SLCG_CTRL_0_CLK_OVR_ON_FIELD _MK_FIELD_CONST(0x1, PKA1_SLCG_CTRL_0_CLK_OVR_ON_SHIFT)
#define PKA1_SLCG_CTRL_0_CLK_OVR_ON_RANGE 0:0

extern int32_t FI_counter1;

NvBootError NvBootSeHousekeepingBeforeBRExit()
{
    // BIT available for all boot paths. Per http://nvbugs/1859111, do this
    // for all non-SC7 boot paths.
    if(BootInfoTable.BootType != NvBootType_Sc7)
    {
        // Clear RSA key slots.
        for (uint32_t seRsaSlot = NvBootSeRsaKeySlot_1; seRsaSlot < NvBootSeRsaKeySlot_Num; seRsaSlot++)
        {
            // Clear the RSA key slots for both SE instances.
            NvBootSeInstanceRsaClearKeySlot(NvBootSeInstance_Se1, seRsaSlot);
            NvBootSeInstanceRsaClearKeySlot(NvBootSeInstance_Se2, seRsaSlot);
        }

        // Clear all PKA key slots.
        NvBootPkaClearAllKeySlots();
    }
    FI_counter1 += COUNTER1;

    // [BR] Write 32 to PKA1_CTRL_CG_SLCG_THRESHOLD.
    // Per JerryZ, and SE IAS, write PKA1_CTRL_CG_SLCG_THRESHOLD to 32.
    uint32_t RegData = NvBootPkaGetPka0Reg(PKA1_CTRL_CG);
    RegData = NV_FLD_SET_DRF_NUM(PKA1, SLCG_CTRL, THRESHOLD, 32, RegData);
    RegData = NV_FLD_SET_DRF_NUM(PKA1, SLCG_CTRL, CLK_OVR_ON, 0, RegData);
    NvBootPkaSetPka0Reg(PKA1_CTRL_CG, RegData);
    FI_counter1 += COUNTER1;

    // Clear ERR_STATUS/INT_STATUS in SE1
    // Clear out AES0, RSA INT_STATUS, write 1 to clear.
    NvBootSetSeInstanceReg(NvBootSeInstance_Se1, SE_INT_STATUS_0, 0xffffffff);
    // [BR] Clear ERR_STATUS/INT_STATUS in SE2
    NvBootSetSeInstanceReg(NvBootSeInstance_Se2, SE_INT_STATUS_0, 0xffffffff);

    // [BR] Write 0x8000_0000 to PKA1_NVSECURE_GROUP to avoid misusing NV group. 
    NvBootPkaSetPka0Reg(PKA1_PKA1_NVSECURE_GROUP, 0x80000000);

    // Should be last step
    // NVSE IAS section "SE in Cold Boot".
    // "Restore default value of SE_CLK_OVR_ON" in
    // CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0 register."
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRB, SE_CLK_OVR_ON, 0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB_0,
               RegData);

    FI_counter1 += COUNTER1;

    return NvBootError_Success;
}

NvBool 
NvBootSeInstanceIsEngineBusy(NvBootSeInstance Instance, NvU8 *DestAddr)
{
    NvU8 Status = 0;
    NvU8 MemStatus = 0;
    NvU32 SeConfigReg = 0;
    NvBool Busy = NV_TRUE;
    NvU32 MstId = ARAHB_MST_ID_SE;

    SeConfigReg = NvBootGetSeInstanceReg(Instance, SE_STATUS_0);
    Status = NV_DRF_VAL(SE, STATUS, STATE, SeConfigReg);

    if(Status == NVBOOT_SE_OP_STATUS_IDLE)
    {
        Busy = NV_FALSE;
    } else {
        Busy = NV_TRUE;
    }
    
    /**
     *  SE operations that write to memory (IRAM/DRAM/TZRAM) need to check this bit
     *  in addition to SE engine status to confirm completion of operation.
     *  http://nvbugs/200279346
     */
    if(DestAddr)
    {
        MemStatus = NV_DRF_VAL(SE, STATUS, MEM_INTERFACE, SeConfigReg);
        if(MemStatus != SE_STATUS_0_MEM_INTERFACE_IDLE)
        {
            Busy |= NV_TRUE;
        }
        
        if(NvBootAhbCheckIsExtMemAddr(DestAddr))
        {
            if(Instance == NvBootSeInstance_Se2)
            {
                MstId = ARAHB_MST_ID_SE2;
            }
            NvBootAhbWaitCoherency(MstId); // 200000 microseconds timeout (= 200 ms)
        }
    }
    return Busy;
}

/**
 * Is the particular SE instance enabled or disabled?
 * True if enabled, false if disabled.
 */
NvBool NvBootSeInstanceIsEngineEnabled(NvBootSeInstance Instance)
{
    NvU32 SeConfigReg = NvBootGetSeInstanceReg(Instance, SE_SE_SECURITY_0);
    if (NV_DRF_VAL(SE, SE_SECURITY, SE_ENG_DIS, SeConfigReg) ==
            SE_SE_SECURITY_0_SE_ENG_DIS_TRUE)
    {
        return NV_FALSE;
    }
    else
    {
        return  NV_TRUE;
    }
}

void
NvBootSeClearTzram(void) 
{
    uint32_t SizeToClear = ARSE_TZRAM_BYTE_SIZE;
    if(NvBootSeIsAtomicSeContextSaveEnabled(NvBootSeInstance_Se1) == NV_TRUE ||
       NvBootSeIsAtomicSeContextSaveEnabled(NvBootSeInstance_Se2) == NV_TRUE )
    {
        // Both carveouts will always be enabled by Boot ROM.
        SizeToClear = ARSE_TZRAM_BYTE_SIZE - ARSE_TZRAM_CARVEOUT_BYTE_SIZE;
    }
    else
    {
        SizeToClear = ARSE_TZRAM_BYTE_SIZE;
    }
    NvBootUtilMemset((void *) NV_ADDRESS_MAP_TZRAM_BASE, 0, SizeToClear);
}

void
NvBootSeInstanceSetupOpMode(NvBootSeInstance Instance, NvBootSeOperationMode Mode, NvBool Encrypt, NvBool UseOrigIv, NvU8 Dst, NvU8 KeySlot, NvU8 KeySize)
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
        case SE_OP_MODE_AES_ECB:
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
        NvBootSetSeInstanceReg(Instance, SE_CONFIG_0, SeConfigReg);
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
        NvBootSetSeInstanceReg(Instance, SE_CRYPTO_CONFIG_0, SeConfigReg);

        if(Mode == SE_OP_MODE_AES_CMAC_HASH)
        {
            SeConfigReg = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_CONFIG_0);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, HASH_ENB, ENABLE, SeConfigReg);
            NvBootSetSeInstanceReg(Instance, SE_CRYPTO_CONFIG_0, SeConfigReg);

        }
        else if(Mode == SE_OP_MODE_AES_ECB)
        {
            SeConfigReg = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_CONFIG_0);
            SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CRYPTO_CONFIG, XOR_POS, BYPASS, SeConfigReg);
            NvBootSetSeInstanceReg(Instance, SE_CRYPTO_CONFIG_0, SeConfigReg);
        }
        break;
        default:
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

#if 0
/*
 *  Archiving SE instance RSA read key slot code. Not needed since SE key slots
 *  are read locked at POR.
 *  Read RSA key slot.
 *  
 *  @param SeInstance SE instance to read from.
 *  @param pKeyBuffer Pointer to the input key buffer. 
 *  @param RsaKeySizeBits Specifies the RSA key size in bits.
 *  @param RsaKeySlot Specifies the RSA key slot number.
 *  @param ExpModSel Specify SE_RSA_KEY_PKT_EXPMOD_SEL_EXPONENT or SE_RSA_KEY_PKT_EXPMOD_SEL_MODULUS.
 * 
 */
void NvBootSeInstanceRsaReadKey(NvBootSeInstance SeInstance, NvU32 *pKeyBuffer, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot, NvU8 ExpModSel)
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
        NvBootSetSeInstanceReg(SeInstance, SE_RSA_KEYTABLE_ADDR_0, SeRsaKeytableAddr);

        *pKeyBuffer++ = NvBootGetSeInstanceReg(SeInstance, SE_RSA_KEYTABLE_DATA_0);
    }

    return;
}
#endif

/*
 *  Write RSA key to a key slot via DMA / memory buffer method. 
 *  
 *  When using DMA / memory buffer input method, the HW
 *  expects the modulus first, and then the exponent in the input
 *  linked list buffer. 
 *
 *  The modulus and exponent are to be loaded in one operation. 
 *  @param SeInstance SE1/SE2
 *  @param pKeyBuffer Pointer to the input key buffer. 
 *  @param RsaModulusSizeBits Specifies the RSA modulus size in bits.
 *  @param RsaKeySizeBits Specifies the RSA key size in bits.
 *  @param RsaKeySlot Specifies the RSA key slot number.
 *
 */
void NvBootSeInstanceRsaWriteKey(NvBootSeInstance SeInstance, NvU32 *pKeyBuffer, NvU32 RsaModulusSizeBits, NvU32 RsaKeySizeBits, NvU8 RsaKeySlot)
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
    NvBootSetSeInstanceReg(SeInstance, SE_CONFIG_0, SeConfigReg);

    // Create input linked list data structure
    // Assumes a contiguous buffer, so there is only one entry
    // in the LL. Therefore, n = 0.
    SeInLL[0] = 0;
    SeInLL[1] = (NvU32) pKeyBuffer;
    SeInLL[2] = (RsaModulusSizeBits + RsaKeySizeBits) / 8;

    // Program SE_IN_LL_ADDR with pointer to SeInLLAddr
    NvBootSetSeInstanceReg(SeInstance, SE_IN_LL_ADDR_0, (NvU32) &SeInLL[0]);

    // Program size of key to be written into SE_RSA_EXP_SIZE or SE_RSA_KEY_SIZE
    // SE_RSA_*_SIZE is specified in units of 4-byte / 32-bits blocks.
    NvBootSetSeInstanceReg(SeInstance, SE_RSA_EXP_SIZE_0, RsaKeySizeBits / 32);
    NvBootSetSeInstanceReg(SeInstance, SE_RSA_KEY_SIZE_0, NvBootSeConvertRsaKeySize(RsaKeySizeBits));

    // Create SE_RSA_KEY_PKT
    SeRsaKeyPkt = ( \
            (SE_RSA_KEY_PKT_INPUT_MODE_DMA << SE_RSA_KEY_PKT_INPUT_MODE_SHIFT) | \
            (RsaKeySlot << SE_RSA_KEY_PKT_KEY_SLOT_SHIFT) /*| \
            (ExpModSel << SE_RSA_KEY_PKT_EXPMOD_SEL_SHIFT) */ );

    SeRsaKeytableAddr |= SeRsaKeyPkt;

    // Start Rsa key slot write. Begins the transfer of the contents of the
    // input memory buffer to the indended key slot.
    NvBootSetSeInstanceReg(SeInstance, SE_RSA_KEYTABLE_ADDR_0, SeRsaKeytableAddr);

    // Wait until engine becomes IDLE.
    while(NvBootSeInstanceIsEngineBusy(SeInstance, NULL)) 
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

/*
 * Set an RSA key slot to zero.
 *
 * Assumes: The RSA key slot does not have its WRITE protection bit set.
 *
 * @param RsaKeySlot  A valid SE RSA key slot.
 *
 */
void NvBootSeInstanceRsaClearKeySlot(NvBootSeInstance SeInstance, NvU8 RsaKeySlot)
{
    NvBootSeRsaKey2048  PublicKey __attribute__ ((aligned (4)));

    NV_ASSERT(RsaKeySlot < NvBootSeRsaKeySlot_Num);

    NvBootUtilMemset(&PublicKey.Modulus[0], 0, ARSE_RSA_MAX_MODULUS_SIZE / 8);
    NvBootUtilMemset(&PublicKey.Exponent[0], 0, ARSE_RSA_MAX_EXPONENT_SIZE / 8);

    NvBootSeInstanceRsaWriteKey(SeInstance,
                        (NvU32 *) &PublicKey,
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
NvBootError NvBootSeInstanceRsaModularExp(NvBootSeInstance SeInstance, NvU8 RsaKeySlot, NvU32 RsaKeySizeBits, NvU32 InputMessageLengthBytes, NvU32 *pInputMessage, NvU32 *pOutputDestination)
{
    NvU32   SeConfigReg = 0;

    // Program size of key to be written into SE_RSA_EXP_SIZE or SE_RSA_KEY_SIZE
    // SE_RSA_*_SIZE is specified in units of 4-byte / 32-bits blocks. 
    NvBootSetSeInstanceReg(SeInstance, SE_RSA_EXP_SIZE_0, RsaKeySizeBits / 32);
    NvBootSetSeInstanceReg(SeInstance, SE_RSA_KEY_SIZE_0, NvBootSeConvertRsaKeySize(RsaKeySizeBits));            
    
    // Create input linked list data structure
    // Assumes a contiguous buffer, so there is only one entry 
    // in the LL. Therefore, n = 0.
    InputLinkedList[SeInstance].LastBufferNumber = 0;
    InputLinkedList[SeInstance].LLElement.StartByteAddress  = (NvU32) pInputMessage;
    InputLinkedList[SeInstance].LLElement.BufferByteSize = InputMessageLengthBytes;

    // Program SE_IN_LL_ADDR with pointer to SeInLLAddr
    NvBootSetSeInstanceReg(SeInstance, SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList[SeInstance]);
    
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
        OutputLinkedList[SeInstance].LastBufferNumber = 0;
        OutputLinkedList[SeInstance].LLElement.StartByteAddress = (NvU32) pOutputDestination;
        OutputLinkedList[SeInstance].LLElement.BufferByteSize = InputMessageLengthBytes;

        // Program SE_OUT_LL_ADDR with pointer from SeOutLLAddr
        NvBootSetSeInstanceReg(SeInstance, SE_OUT_LL_ADDR_0, (NvU32) &OutputLinkedList[SeInstance]);
    }
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_ALG, NOP, SeConfigReg);    
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_ALG, RSA, SeConfigReg);    
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, DEC_MODE, DEFAULT, SeConfigReg);    
    SeConfigReg = NV_FLD_SET_DRF_DEF(SE, CONFIG, ENC_MODE, DEFAULT, SeConfigReg);    

    // Write to SE_CONFIG register
    NvBootSetSeInstanceReg(SeInstance, SE_CONFIG_0, SeConfigReg);

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
    NvBootSetSeInstanceReg(SeInstance, SE_RSA_CONFIG_0, SeConfigReg);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeInstanceReg(SeInstance, SE_OPERATION_0, SeConfigReg);

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
    /*
     * [Coverity] overflow_before_widen: Potentially overflowing expression
     * "InputMessageBytesLeft * 8UL" with type "unsigned long" (32 bits, unsigned)is
     * evaluated using 32-bit arithmetic, and then used in a context that expects
     * an expression of type "NvU64" (64 bits, unsigned).
     * remediation: To avoid overflow, cast either "InputMessageBytesLeft" or "8UL" to type "NvU64".
     */
    NvU64   InputMessageSizeBitsLeft = (NvU64) InputMessageSizeBytes * 8;
    NvU32   * const pInputMessageSizeBitsLeft0 = (NvU32 *) &InputMessageSizeBitsLeft;
    NvU32   * const pInputMessageSizeBitsLeft1 = pInputMessageSizeBitsLeft0 + 1;
    NvU32   ChunksRemainingForHash = NV_ICEIL(InputMessageSizeBytes, NVBOOT_SE_LL_MAX_SIZE_BYTES);
    NvU32   InputMessageBytesLeft = InputMessageSizeBytes;
    NvBool  First = NV_TRUE;
    NvBool  Last = NV_FALSE;

    NV_ASSERT(pInputMessage != NULL);
    NV_ASSERT( (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA1) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA224) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA256) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA384) ||
               (HashAlgorithm == SE_MODE_PKT_SHAMODE_SHA512) );

    // T214 Specific WAR, http://nvbugs/1788437. Fixed in T194.
    if(InputMessageSizeBytes == 0)
    {
        NvBootUtilMemcpy(pOutputDestination, &Sha256NullStringDigest, NVBOOT_SHA256_LENGTH_BYTES);
        return;
    }

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
        /*
         * [Coverity] overflow_before_widen: Potentially overflowing expression
         * "InputMessageBytesLeft * 8UL" with type "unsigned long" (32 bits, unsigned)is
         * evaluated using 32-bit arithmetic, and then used in a context that expects
         * an expression of type "NvU64" (64 bits, unsigned).
         * remediation: To avoid overflow, cast either "InputMessageBytesLeft" or "8UL" to type "NvU64".
         */
        InputMessageSizeBitsLeft = (NvU64) InputMessageBytesLeft * 8;

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
        while(NvBootSeIsEngineBusy((NvU8*)pOutputDestination))        
            ;
    }

    return;
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
NvBootSePmcLoadSrkFromSecureScratch(NvBootSeInstance Instance, NvU32 *pSrk)
{
    /**
     * Per SE IAS and ASIC, a CTX_SAVE operation with DST = SRK will save the 
     * SRK at PMC Secure Scratch Registers SCRATCH4-7. 
     */
     if(Instance == NvBootSeInstance_Se1)
     {
        // APBDEV_PMC_SECURE_SCRATCH4_0_SRK_0_SRK0_RANGE
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH4_0);
        // APBDEV_PMC_SECURE_SCRATCH5_0_SRK_0_SRK1_RANGE
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH5_0);
        // APBDEV_PMC_SECURE_SCRATCH6_0_SRK_0_SRK2_RANGE
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH6_0);
        // APBDEV_PMC_SECURE_SCRATCH7_0_SRK_0_SRK3_RANGE
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH7_0);
     }
     else
     {
         
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH120_0);
        
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH121_0);
        
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH122_0);
        
        *pSrk++ = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH123_0);
     }

    return;
}

/**
 * Disable read access for a particular key slot, OIV, or UIV. 
 *
 */
void NvBootSeInstanceDisableKeySlotReadAccess(NvBootSeInstance SeInstance, NvU8 KeySlot, NvU8 KeySlotType)
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

    NvBootSetSeInstanceReg(SeInstance, SE_CRYPTO_KEYTABLE_ACCESS_0 + (KeySlot * 4), DisableValue);

    return;
}

/**
 * Disable write access for a particular key slot, OIV, or UIV. 
 *
 */
void NvBootSeInstanceDisableKeySlotWriteAccess(NvBootSeInstance SeInstance, NvU8 KeySlot, NvU8 KeySlotType)
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

    NvBootSetSeInstanceReg(SeInstance, SE_CRYPTO_KEYTABLE_ACCESS_0 + (KeySlot * 4), DisableValue);

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

void NvBootSeInstanceKeySlotReadKeyIV(NvBootSeInstance Instance, NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData)
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
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  1,
                                                  IvSel,
                                                  1);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  2,
                                                  IvSel,
                                                  2);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  3,
                                                  IvSel,
                                                  3);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

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
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  5,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

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
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  7,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    *KeyData++ = NvBootGetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0);

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

void NvBootSeInstanceKeySlotWriteKeyIV(NvBootSeInstance Instance, NvU8 KeySlot, NvU8 KeySize, NvU8 KeyType, NvU32 *KeyData)
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
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  1,
                                                  IvSel,
                                                  1);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  2,
                                                  IvSel,
                                                  2);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  3,
                                                  IvSel,
                                                  3);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

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
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  5,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

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
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

    SeCryptoKeytableAddr = SeCreateCryptoKeyIvPkt(KeySlot,
                                                  KeyIvSel,
                                                  7,
                                                  IvSel,
                                                  // IvWord always zero since
                                                  // it must be a key only.
                                                  0);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_ADDR_0, SeCryptoKeytableAddr);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DATA_0, KeyData == 0 ? 0 : *KeyData++);

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

void
NvBootSeDisableAesKeySlotRead(NvBootSeInstance Instance, uint8_t KeySlot)
{
    NvBootSeInstanceDisableKeySlotReadAccess(Instance,
                                             KeySlot,
                                             SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3);
    NvBootSeInstanceDisableKeySlotReadAccess(Instance,
                                             KeySlot,
                                             SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS);
    NvBootSeInstanceDisableKeySlotReadAccess(Instance,
                                             KeySlot,
                                             SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS);
}

/**
 * Parse decrypted context in IRAM into usable format.
 */
void NvBootSeInstanceParseDecryptedContext(
            NvBootSeInstance    Instance,
            void *pSeDecryptedContext,
            NvBootSeContextStickyBitsRegBuf *pNvBootSeContextStickyBitsRegBuf)
{
    NvBootSe1Lp0Context *pSeInstancedDecryptedContext = (NvBootSe1Lp0Context *)pSeDecryptedContext;
    NvBootSeLp0ContextStickyBits *pSeLp0ContextStickyBits;
    
    pSeLp0ContextStickyBits= &(pSeInstancedDecryptedContext->SeContextStickyBits);
    

    /**
     * Parse sticky bits information into NvBootSeContextStickyBitsRegBuf.
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
    // Hard Setting, Eng Disable, Per Key Setting grouped.
    NvU32 SeSecurity0=pSeLp0ContextStickyBits->SeSecurity0_2_0;
    // Soft Setting
    SeSecurity0 = NV_FLD_SET_DRF_NUM(SE, SE_SECURITY, SE_SOFT_SETTING, pSeLp0ContextStickyBits->SeSecurity0_SoftSetting, SeSecurity0);
    SeSecurity0 = NV_FLD_SET_DRF_NUM(SE, SE_SECURITY, CTX_SAVE_TZ_LOCK, pSeLp0ContextStickyBits->SeSecurity0_4, SeSecurity0);
    pNvBootSeContextStickyBitsRegBuf->SE_SE_SECURITY = SeSecurity0;

    pNvBootSeContextStickyBitsRegBuf->SE_TZRAM_SECURITY = pSeLp0ContextStickyBits->SeTzramSecurity0_1_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_SECURITY_PERKEY = pSeLp0ContextStickyBits->SeCryptoSecurityPerKey15_0;

    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[0] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess0_7_0;
        
    /// Warrants special handling as sticky bits are across word boundary
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[1] =  pSeLp0ContextStickyBits->SeCryptoKeyTableAccess1_1_0 | \
                                                                (pSeLp0ContextStickyBits->SeCryptoKeyTableAccess1_7_2<<0x2);

    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[2] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess2_7_0;

    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[3] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess3_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[4] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess4_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[5] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess5_1_0 | \
                                                               (pSeLp0ContextStickyBits->SeCryptoKeyTableAccess5_7_2<<0x2);
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[6] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess6_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[7] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess7_7_0;

    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[8] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess8_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[9] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess9_1_0 | \
                                                               (pSeLp0ContextStickyBits->SeCryptoKeyTableAccess9_7_2 << 0x2);
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[10] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess10_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[11] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess11_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[12] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess12_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[13] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess13_1_0 | \
                                                                (pSeLp0ContextStickyBits->SeCryptoKeyTableAccess13_7_2<<0x2);
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[14] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess14_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[15] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess15_7_0;

    pNvBootSeContextStickyBitsRegBuf->SE_CRYPTO_KT_ACCESS[15] = pSeLp0ContextStickyBits->SeCryptoKeyTableAccess15_7_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_RSA_SECURITY_PERKEY = pSeLp0ContextStickyBits->SeRsaSecurityPerKey0_1_0;

    pNvBootSeContextStickyBitsRegBuf->SE_RSA_KT_ACCESS[0] = pSeLp0ContextStickyBits->SeRsaKeyTableAccess0_2_0;
    
    pNvBootSeContextStickyBitsRegBuf->SE_RSA_KT_ACCESS[1] = pSeLp0ContextStickyBits->SeRsaKeyTableAccess1_2_0;
    
    if(Instance ==  NvBootSeInstance_Se2)
    {
        NvBootSe2Lp0Context *pSeInstancedDecryptedContext = (NvBootSe2Lp0Context *)pSeDecryptedContext;
        NvBootSePka1Lp0ContextStickyBits *pSePka1Lp0ContextStickyBits = &(pSeInstancedDecryptedContext->SePka1Lp0ContextStickyBits);
        // Handle PKA sticky bits for SE2.
        
        pNvBootSeContextStickyBitsRegBuf->PKA1_SECURITY_PERKEY_OWNER[0] = pSePka1Lp0ContextStickyBits->Pka1SecurityPerKey0_2_0;
        pNvBootSeContextStickyBitsRegBuf->PKA1_SECURITY_PERKEY_OWNER[1] = pSePka1Lp0ContextStickyBits->Pka1SecurityPerKey1_2_0;
        pNvBootSeContextStickyBitsRegBuf->PKA1_SECURITY_PERKEY_OWNER[2] = pSePka1Lp0ContextStickyBits->Pka1SecurityPerKey2_2_0;
        pNvBootSeContextStickyBitsRegBuf->PKA1_SECURITY_PERKEY_OWNER[3] = pSePka1Lp0ContextStickyBits->Pka1SecurityPerKey3_2_0;
        
        pNvBootSeContextStickyBitsRegBuf->PKA1_KEYTABLE_ACCESS[0] = pSePka1Lp0ContextStickyBits->Pka1KeyTableAccess0_2_0;
        pNvBootSeContextStickyBitsRegBuf->PKA1_KEYTABLE_ACCESS[1] = pSePka1Lp0ContextStickyBits->Pka1KeyTableAccess1_2_0;
        pNvBootSeContextStickyBitsRegBuf->PKA1_KEYTABLE_ACCESS[2] = pSePka1Lp0ContextStickyBits->Pka1KeyTableAccess2_2_0;
        pNvBootSeContextStickyBitsRegBuf->PKA1_KEYTABLE_ACCESS[3] = pSePka1Lp0ContextStickyBits->Pka1KeyTableAccess3_2_0;

        NvU32 Pka1Security = 0;
        // PKA1 manual is not compatible with NV_DRF macros so use identical SE register with same
        // bit mapping for concerned
        Pka1Security = NV_FLD_SET_DRF_NUM(SE, SE_SECURITY, SE_HARD_SETTING, pSePka1Lp0ContextStickyBits->Pka1Security0_SeHardSetting, Pka1Security);
        Pka1Security = NV_FLD_SET_DRF_NUM(SE, SE_SECURITY, SE_ENG_DIS, pSePka1Lp0ContextStickyBits->Pka1Security0_SeEngDis, Pka1Security);
        Pka1Security = NV_FLD_SET_DRF_NUM(SE, SE_SECURITY, PERKEY_SETTING, pSePka1Lp0ContextStickyBits->Pka1Security0_PerKeySetting, Pka1Security);
        Pka1Security = NV_FLD_SET_DRF_NUM(SE, SE_SECURITY, SE_SOFT_SETTING, pSePka1Lp0ContextStickyBits->Pka1Security0_SeSoftSetting, Pka1Security);
        pNvBootSeContextStickyBitsRegBuf->PKA1_SECURITY = Pka1Security;
        
        NvU32 Pka1MutexRRTmout = 0;
        Pka1MutexRRTmout = NV_FLD_SET_DRF_NUM(PKA1, CTRL_PKA_MUTEX_RR_TMOUT, LOCK, pSePka1Lp0ContextStickyBits->Pka1MutexRRTmoutLock, Pka1MutexRRTmout);
        /// Warrants special attention as sticky bits cross boundary.
        Pka1MutexRRTmout = NV_FLD_SET_DRF_NUM(PKA1, CTRL_PKA_MUTEX_RR_TMOUT, VAL, \
                                             pSePka1Lp0ContextStickyBits->Pka1MutexRRTmoutVal_2_0 | \
                                            (pSePka1Lp0ContextStickyBits->Pka1MutexRRTmoutVal_19_3 << 0x3),\
                                             Pka1MutexRRTmout);
        pNvBootSeContextStickyBitsRegBuf->CTRL_PKA_MUTEX_RR_TMOUT = Pka1MutexRRTmout;
        
        /// Warrants special attention as sticky bits cross boundary.
        pNvBootSeContextStickyBitsRegBuf->PKA1_NVSECURE_GROUP  = pSePka1Lp0ContextStickyBits->Pka1NvSecureGroup_13_0 | \
                                                                 pSePka1Lp0ContextStickyBits->Pka1NvSecureGroup_31_14 << 14;

    }

    return;
}

/**
 *  Stage 1 starts the decryption process of SE blob. It is not blocking so make sure to poll for
 *  completion after exiting the routine.
 */
void NvBootLP0ContextRestoreStage1(NvBootSeInstance SeInstance, void *pSeDecryptedContext)
{
    NvU32 Srk[NVBOOT_SE_SRK_KEY_LENGTH];

    /**************************** Start decrypt SE1 Context first *********************************/

    /** 
     * 1. SW copies the 128-bit SRK from PMC secure scratch registers to some 
     * arbitrary key-slot n in SE and sets the original and updated IVs in this key-slot to
     * zeroes. 
     */
    // Load SRK from PMC secure scratch registers
    NvBootSePmcLoadSrkFromSecureScratch(SeInstance, &Srk[0]);

    // Write the SRK into key slot 0
    NvBootSeInstanceKeySlotWriteKeyIV(SeInstance, NvBootSeAesKeySlot_0, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, &Srk[0]);

    /**
     *  Clear SRK on stack to 0.
     */
    NvBootUtilMemset(&Srk[0], 0, sizeof(Srk));
    
    /**
     * Clear the secure scratch registers associated with the SRK to 0. 
     *
     * Even though the SRK is generated at every LP0 entry, we don't want any
     * untrusted SW after the Boot ROM to read it. 
     *
     */ 
    NvBootSeClearSrkSecureScratch(SeInstance);

    // Initialize OriginalIv[127:0] to zero
    NvBootSeInstanceKeySlotWriteKeyIV(SeInstance, NvBootSeAesKeySlot_0, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 0);

    // Initialize UpdatedIV[127:0] to zero
    NvBootSeInstanceKeySlotWriteKeyIV(SeInstance, NvBootSeAesKeySlot_0, SE_MODE_PKT_AESMODE_KEY128, SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 0);

    /**
     * 2. Proram SE to decrypt the encrypted context in memory using the SRK in 
     * key-slot 0 and output it to some arbitrary memory location
     */

    // Start decryption
    NvU32 EncryptedSize;
    NvU32 EncryptedContextAddr;
    // See //hw/ap/chip_tools/global/Tegra_PMC_Scratch_Registers.csv for scratch allocation.
    if(SeInstance == NvBootSeInstance_Se1)
    {
        EncryptedSize = sizeof(NvBootSe1Lp0Context)/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
        EncryptedContextAddr = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH117_0);
    }
    else
    {
        EncryptedSize = sizeof(NvBootSe2Lp0Context)/NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
        EncryptedContextAddr = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH116_0);
    }
    // Note: This only starts the decrypt. Poll to check complete.
    NvBootSeInstanceAesDecryptStart(SeInstance,
                        NvBootSeAesKeySlot_0,
                       SE_MODE_PKT_AESMODE_KEY128,
                       NV_TRUE,
                       EncryptedSize,
                       (NvU8 *) EncryptedContextAddr,
                       (NvU8 *) pSeDecryptedContext);
}

void NvBootLP0ContextRestoreDisable(NvBootSeInstance SeInstance, void *pSeDecryptedContext)
{
    // NOTE: Order matters here!!
    // For safety, clear all key slots. Even after an LP0 transition, 
    // the keyslots may not be fully cleared if the time spent in LP0
    // was very short. 
    NvU32 ContextSize = sizeof(NvBootSe1Lp0Context);
    
    NvBootSeInstanceClearAllKeySlots(SeInstance);

    // Disable read/write access to the key slots. Set each key slot as 
    // ARSE_SECURE. 
    NvBootSeInstanceDisableAllKeySlotReadWrite(SeInstance);
    
    if(SeInstance == NvBootSeInstance_Se2)
    {
        // Clear PKA engine key slots.
        NvBootPkaClearAllKeySlots();

        // Disable PKA engine.
        NvBootPkaDisablePka();
        
        // Adjust Context Size
        ContextSize = sizeof(NvBootSe2Lp0Context);
    
    }
    
    if(SeInstance == NvBootSeInstance_Se1)
    {
        // SE1 controls TZRAM. The register is dummy in SE2.
        // Disable TZRAM (Set TZRAM as ARSE_SECURE and disable reads and writes).
        NvBootSeDisableTzram();
    }
    // Disable all crypto operations and expanded key table access. Force 
    // access to PERKEY_SETTING and SE register accesses/operations
    // in secure mode only. 
    NvBootSeInstanceDisableSe(SeInstance);

    // Even though known pattern check failed, let's clear the bad 
    // decrypted data from IRAM. 
    NvBootUtilMemset(pSeDecryptedContext, 0, ContextSize);
}

void NvBootSeRsaKeySlotSwapModExp(NvBootSeRsaKey2048 *RsaKey)
{
    // NvBootSeRsaKey2048 RsaKey2048[NvBootSeRsaKeySlot_Num]; // 32 Blks * 2 slots = 64 Blks
    // NvU32 Modulus[2048 / 8 / 4];
    // The exponent size is 2048-bits.
    //NvU32 Exponent[2048 / 8 / 4];
    NvU32 TempModulus[NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS];

    // Save modulus into temp buffer, source from the Exponent field.
    NvBootUtilMemcpy(&TempModulus, &RsaKey->Exponent, sizeof(TempModulus));
    // Copy data Modulus field (which contains exponent data) to Exponent
    // field.
    NvBootUtilMemcpy(&RsaKey->Exponent, &RsaKey->Modulus, sizeof(TempModulus));
    // Copy temporarily saved modulus into real Modulus field.
    NvBootUtilMemcpy(&RsaKey->Modulus, &TempModulus, sizeof(TempModulus));

    // Clear temporary buffer.
    NvBootUtilMemset(&TempModulus, 0, sizeof(TempModulus));
}

void NvBootLP0ContextRestoreStage2(NvBootSeInstance SeInstance, void *pSeDecryptedContext)
{
    NvBootSeContextStickyBitsRegBuf s_NvBootSeContextStickyBitsRegBuf;
    NvU32 Slot;
    NvU32 ContextSize;
    // It is ok to use SE1 context for AES, RSA keys i.e. the common part.
    NvBootSe1Lp0Context *pSeInstancedDecryptedContext = (NvBootSe1Lp0Context *)pSeDecryptedContext;

    // Parse the decrypted context into a more easily usable format
    NvBootSeInstanceParseDecryptedContext(SeInstance,
            pSeDecryptedContext,
            &s_NvBootSeContextStickyBitsRegBuf);
        
    /**
     * SW restores keys to key-slot n in SE key table
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeInstanceKeySlotWriteKeyIV(SeInstance,
                                  Slot, 
                                  SE_MODE_PKT_AESMODE_KEY256,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                                  &pSeInstancedDecryptedContext->AesKeys[Slot].Aes256Key.Key[0]); 
    }
    /**
     * SW restores original IV to key-slot n in SE key table
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeInstanceKeySlotWriteKeyIV(SeInstance,
                                  Slot, 
                                  SE_MODE_PKT_AESMODE_KEY128, 
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 
                                  &pSeInstancedDecryptedContext->AesKeys[Slot].OriginalIv.Iv[0]);
    }

    /**
     * SW restores updated/current IV to key-slot n in SE key table
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSeInstanceKeySlotWriteKeyIV(SeInstance,
                                  Slot, 
                                  SE_MODE_PKT_AESMODE_KEY128, 
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 
                                  &pSeInstancedDecryptedContext->AesKeys[Slot].UpdatedIv.Iv[0]);
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
        // Swap context of modulus and exponent; WAR for atomic save issue.
        NvBootSeRsaKeySlotSwapModExp(&pSeInstancedDecryptedContext->RsaKey2048[Slot]);

        NvBootSeInstanceRsaWriteKey(SeInstance, 
                            &pSeInstancedDecryptedContext->RsaKey2048[Slot].Modulus[0], 
                            ARSE_RSA_MAX_MODULUS_SIZE, 
                            ARSE_RSA_MAX_EXPONENT_SIZE, 
                            Slot);        
    }

    if(SeInstance == NvBootSeInstance_Se2)
    {
        /**
         * Restore PKA1 key slots
         */
        NvBootSe2Lp0Context *pSeInstancedDecryptedContext = (NvBootSe2Lp0Context *)pSeDecryptedContext;
        for (Slot = 0; Slot < ARSE_PKA1_NUM_KEY_SLOTS; Slot++)
        {
            NvBootPkaWriteKeySlot(&pSeInstancedDecryptedContext->Pka1KeySlot[Slot].SlotData[0], Slot);
        }
        /**
         * Restore key slot security related info, PKA1_KEYTABLE_ACCESS, 
         * PKA1_SECURITY_PERKEY, PKA1_MUTEX_RR_TMOUT
         */
        for (Slot = 0; Slot < ARSE_PKA1_NUM_KEY_SLOTS; Slot++)
        {
            NvBootPkaSetPka0Reg(
                PKA1_PKA1_KEYTABLE_ACCESS(Slot),
                s_NvBootSeContextStickyBitsRegBuf.PKA1_KEYTABLE_ACCESS[Slot]);

            NvBootPkaSetPka0Reg(
               PKA1_PKA1_SECURITY_PERKEY(Slot),
                s_NvBootSeContextStickyBitsRegBuf.PKA1_SECURITY_PERKEY_OWNER[Slot]);
        }

        NvBootPkaSetPka0Reg(PKA1_CTRL_PKA_MUTEX_RR_TMOUT, s_NvBootSeContextStickyBitsRegBuf.CTRL_PKA_MUTEX_RR_TMOUT);

        /**
         * Restore PKA1 other seecurity registers, PKA1_SECURITY.
         * Move to later in SC7 exit sequence in Stage 3.
         */
        Context.Pka1Security = s_NvBootSeContextStickyBitsRegBuf.PKA1_SECURITY;
    }

    /**
     * SW must erase the decrypted data stored in memory.
     */
    if(SeInstance == NvBootSeInstance_Se1)
    {
        ContextSize = sizeof(NvBootSe1Lp0Context);
        
    }
    else
    {
        ContextSize = sizeof(NvBootSe2Lp0Context);
    }

    NvBootUtilMemset(pSeInstancedDecryptedContext, 0, ContextSize);
    
    /**
     * 10. SW restores decrypted key table access control sticky-bits via 
     * register writes to the respective SE registers. 
     */
    for (Slot = 0; Slot < NvBootSeAesKeySlot_Num; Slot++) 
    {
        NvBootSetSeInstanceReg( 
            SeInstance,
            SE_CRYPTO_KEYTABLE_ACCESS_0 + (Slot * 4), 
            s_NvBootSeContextStickyBitsRegBuf.SE_CRYPTO_KT_ACCESS[Slot]);
    }

    /**
     * 10. SW restores SE_RSA_KEYTABLE_ACCESS_0 and SE_RSA_KEYTABLE_ACCESS_1
     * registers.
     */
    for (Slot = NvBootSeRsaKeySlot_1; Slot < NvBootSeRsaKeySlot_Num; Slot++)
    {
        NvBootSetSeInstanceReg(
            SeInstance,
            SE_RSA_KEYTABLE_ACCESS_0 + (Slot * 4),
            s_NvBootSeContextStickyBitsRegBuf.SE_RSA_KT_ACCESS[Slot]);
    }
    // Restore SE_CRYPTO_SECURITY_PERKEY_0
    NvBootSetSeInstanceReg(
        SeInstance,
        SE_CRYPTO_SECURITY_PERKEY_0,
        s_NvBootSeContextStickyBitsRegBuf.SE_CRYPTO_SECURITY_PERKEY);

    // Restore SE_RSA_SECURITY_PERKEY_0
    NvBootSetSeInstanceReg(
        SeInstance,
        SE_RSA_SECURITY_PERKEY_0,
        s_NvBootSeContextStickyBitsRegBuf.SE_RSA_SECURITY_PERKEY);
    
    // TZRAM Bits apply only to SE1
    if(SeInstance == NvBootSeInstance_Se1)
    {
        // These are global and apply to SE1 only.
        // Save value to restore to SE_TZRAM_SECURITY_0 for later restoration.
        Context.SeTzramSecurity = s_NvBootSeContextStickyBitsRegBuf.SE_TZRAM_SECURITY;
    }

    // Re-enable SE atomic context save feature before programming
    // SE_SECURITY_0.
    NvBootSeInstanceEnableAtomicSeContextSave(SeInstance);

    // Save SE_SE_SECURITY_0 for restoration at the end of BR SC7 exit.
    // Do a read/modify/write here, since not all of the bits in this register are
    // saved/restored by this sequence.
    uint32_t SeSecurity0 = NvBootGetSeInstanceReg(SeInstance, SE_SE_SECURITY_0);
    // Bit 5, SE_TZ_LOCK_SOFT is not saved/restored by SE context save/restore,
    // preserve POR value.
    SeSecurity0 &= SE_SE_SECURITY_0_SE_TZ_LOCK_SOFT_FIELD;
    SeSecurity0 |= s_NvBootSeContextStickyBitsRegBuf.SE_SE_SECURITY;

    if(SeInstance == NvBootSeInstance_Se1)
        Context.SeSecuritySe1 = SeSecurity0;
    else
        Context.SeSecuritySe2 = SeSecurity0;
    
    // Clear the sticky bits buffer memory
    NvBootUtilMemset(&s_NvBootSeContextStickyBitsRegBuf, 0, sizeof(NvBootSeContextStickyBitsRegBuf));
}

NvBootError NvBootLP0ContextRestore(void)
{

    /**
     * Set Pointers to respective buffers.
     * Note: Buffers can't be shared. The decrypt operations run in parallel. 
     */
    NvBootSe1Lp0Context *pSe1DecryptedContext = (NvBootSe1Lp0Context *)&Se1DecryptedContext;
    NvBootSe2Lp0Context *pSe2DecryptedContext = (NvBootSe2Lp0Context *)&Se2DecryptedContext;
    
    /**
     *  Stage 1: Start decryption of both blobs using SE1 and SE2 in parallel
     */

    NvBootLP0ContextRestoreStage1(NvBootSeInstance_Se1, pSe1DecryptedContext);
    NvBootLP0ContextRestoreStage1(NvBootSeInstance_Se2, pSe2DecryptedContext);
     
    /**
     *  Poll for completion of both engines
     */
    // Wait till SE1 is idle.
    while(NvBootSeInstanceIsEngineBusy(NvBootSeInstance_Se1, (NvU8*)pSe1DecryptedContext));

    // Wait till SE2 is idle.
    while(NvBootSeInstanceIsEngineBusy(NvBootSeInstance_Se2, (NvU8*)pSe2DecryptedContext));

    /**
     * SW checks decyrpted known pattern. If it matches the expected known 
     * pattern, proceed. if the known pattern check fails, disable SE as much as possible and
     * skip the restore of the SE. Let OS decide what to do after failed SE
     * context restore. If either blob fails to decrypt correctly, disable both.
     */
    if  ( (NvBootUtilCompareConstTimeFI(&pSe1DecryptedContext->KnownPattern[0], &NvBootSeContextKnownPattern[0], sizeof(NvBootSeContextKnownPattern)) == FI_FALSE) ||
          (NvBootUtilCompareConstTimeFI(&pSe2DecryptedContext->KnownPattern[0], &NvBootSeContextKnownPattern[0], sizeof(NvBootSeContextKnownPattern)) == FI_FALSE) )
    {
        // First disable SE2.
        NvBootLP0ContextRestoreDisable(NvBootSeInstance_Se2, pSe2DecryptedContext);
        
        // SE1 contains the master security bits.
        NvBootLP0ContextRestoreDisable(NvBootSeInstance_Se1, pSe1DecryptedContext);

        return NvBootError_SE_Context_Restore_Failure;
    }

    // Restore SE2/PKA1 first followed by SE1
    NvBootLP0ContextRestoreStage2(NvBootSeInstance_Se2, pSe2DecryptedContext);

    NvBootLP0ContextRestoreStage2(NvBootSeInstance_Se1, pSe1DecryptedContext);

    return NvBootError_Success;
}

NvBootError NvBootLP0ContextRestoreStage3()
{
    if(BootInfoTable.BootType == NvBootType_Sc7)
    {
        // If the engines are disabled these register writes are
        // silently dropped. They might cause an issue in RTL sim (flagged with an error/warning)
        // , when the engine is is set to secure (a step BR does in when the known pattern
        // check fails), engine is disabled after SE context restore fails, and BPMP
        // (non-secure client) tries to write this register.

        // Follow the original restoration order from Stage2.
        // First restore SE2. PKA1_SECURITY, then SE_SECURITY.
        // Only one PKA engine.
        NvBootPkaSetPka0Reg(PKA1_PKA1_SECURITY, Context.Pka1Security);
        NvBootSetSeInstanceReg(NvBootSeInstance_Se2, SE_SE_SECURITY_0, Context.SeSecuritySe2);
    
        // Secondly, restore SE1. TZRAM_SECURITY, then SE_SECURITY.
        // Only one copy of SE_TZRAM_SECURITY_0
        NvBootSetSeInstanceReg(NvBootSeInstance_Se1, SE_TZRAM_SECURITY_0, Context.SeTzramSecurity);
        NvBootSetSeInstanceReg(NvBootSeInstance_Se1, SE_SE_SECURITY_0, Context.SeSecuritySe1);
    }
    // Dispatcher routines need NvBootError returned.
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
    while(NvBootSeIsEngineBusy((NvU8*)L))
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

    // flag == false when the length is not 16-byte aligned. true otherwise.
    // In this implementation, it will only be false when length = NumBlocks = 0.
    bool flag = false;
    /**
     * Step 3 of AES-CMAC hash algorithm.
     * Flag == false;
     * NumBlocks = 1;
     * M_Last := padding(M_n) XOR K2;
     */
    if(NumBlocks == 0)
    {
        NumBlocks = 1;

        // Create one block using the padding scheme from the spec.
        // "For an input string x of r-octets, where 0 <= r < 16, the padding
        // function, padding(x), is defined as follows:
        // padding(x) = x || 10^i      where i is 128-8*r-1, and
        // 10^i means 1 followed by i-times repeated zeros."
        NvBootUtilMemset(&LastBlockBuffer, 0, NVBOOT_SE_AES_BLOCK_LENGTH_BYTES);
        LastBlockBuffer[0] = 0x00000080;

        // The only input is this padded last block now.
        Src = (uint32_t *) &LastBlockBuffer;

        flag = false;
    }
    else
    {
        // Since the input length is always NumBlocks, length is always aligned
        // to 16-bytes.
        flag = true;
    }
    const NvU32 WordOffsetToLastBlock = (NumBlocks-1) * NVBOOT_SE_AES_BLOCK_LENGTH;
    const NvU32 ByteOffsetToLastBlock = (NumBlocks-1) * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

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
                while(NvBootSeIsEngineBusy(NULL))
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
                while(NvBootSeIsEngineBusy(NULL))
                    ;
            }

            // "Step 4" of RFC 4493. Choose which subkey to use depending on flag.
            uint32_t *SubKey = (flag == true) ? pK1 : pK2;

            // Process the last block with the correct subkey K1 or K2.
            for(i = 0; i < NVBOOT_SE_AES_BLOCK_LENGTH; i++)
            {
                LastBlockBuffer[i] = *(SubKey+i) ^ *(Src + WordOffsetToLastBlock + i);
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
            while(NvBootSeIsEngineBusy(pHash))
                ;

        }
        else
        {
            // Check if SE is idle.
            while(NvBootSeIsEngineBusy(NULL))
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
    while(NvBootSeIsEngineBusy(Dst));
        ;

    return;
}

void NvBootSeInstanceAesDecryptStart (
        NvBootSeInstance SeInstance,
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst)
{
    NvU32   SeConfigReg;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT(NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES < NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES);
    NV_ASSERT(NumBlocks > 0);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );

    // Setup SE engine parameters for AES decrypt operation.
    NvBootSeInstanceSetupOpMode(SeInstance,
                        SE_OP_MODE_AES_CBC,
                        NV_FALSE,
                        First,
                        SE_CONFIG_0_DST_MEMORY,
                        KeySlot,
                        KeySize);

    // Setup input linked list.
    InputLinkedList[SeInstance].LastBufferNumber = 0;
    InputLinkedList[SeInstance].LLElement.StartByteAddress = (NvU32) Src;
    InputLinkedList[SeInstance].LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of input linked list.
    NvBootSetSeInstanceReg(SeInstance, SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList[SeInstance]);

    OutputLinkedList[SeInstance].LastBufferNumber = 0;
    OutputLinkedList[SeInstance].LLElement.StartByteAddress = (NvU32) Dst;
    OutputLinkedList[SeInstance].LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of output linked list.
    NvBootSetSeInstanceReg(SeInstance, SE_OUT_LL_ADDR_0, (NvU32) &OutputLinkedList[SeInstance]);

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
    NvBootSetSeInstanceReg(SeInstance, SE_CRYPTO_LAST_BLOCK_0, NumBlocks-1);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeInstanceReg(SeInstance, SE_OPERATION_0, SeConfigReg);

    // When called in the reader code, the UpdateCryptoStatus2 function will make sure no new
    // operations will start before the engine is idle.
    // Poll for OP_DONE.

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
    while(NvBootSeIsEngineBusy(Dst))
        ;

    return;
}

void NvBootSeInstanceAesEncryptECBStart (
        NvBootSeInstance SeInstance,
        NvU8     KeySlot,
        NvU8     KeySize,
        NvBool   First,
        NvU32    NumBlocks,
        NvU8    *Src,
        NvU8    *Dst)
{
    NvU32   SeConfigReg;

    NV_ASSERT(KeySlot < NvBootSeAesKeySlot_Num);
    NV_ASSERT(NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES < NVBOOT_SE_LL_MAX_BUFFER_SIZE_BYTES);
    NV_ASSERT(NumBlocks > 0);
    NV_ASSERT( (KeySize  == SE_MODE_PKT_AESMODE_KEY128) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY192) ||
               (KeySize == SE_MODE_PKT_AESMODE_KEY256) );

    // Setup SE engine parameters for AES ECB encrypt operation.
    NvBootSeInstanceSetupOpMode(SeInstance,
                        SE_OP_MODE_AES_ECB,
                        NV_TRUE,
                        First,
                        SE_CONFIG_0_DST_MEMORY,
                        KeySlot,
                        KeySize);

    // Setup input linked list.
    InputLinkedList[SeInstance].LastBufferNumber = 0;
    InputLinkedList[SeInstance].LLElement.StartByteAddress = (NvU32) Src;
    InputLinkedList[SeInstance].LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of input linked list.
    NvBootSetSeInstanceReg(SeInstance, SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList[SeInstance]);

    OutputLinkedList[SeInstance].LastBufferNumber = 0;
    OutputLinkedList[SeInstance].LLElement.StartByteAddress = (NvU32) Dst;
    OutputLinkedList[SeInstance].LLElement.BufferByteSize = NumBlocks * NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of output linked list.
    NvBootSetSeInstanceReg(SeInstance, SE_OUT_LL_ADDR_0, (NvU32) &OutputLinkedList[SeInstance]);

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
    NvBootSetSeInstanceReg(SeInstance, SE_CRYPTO_LAST_BLOCK_0, NumBlocks-1);

    /**
     * Issue START command in SE_OPERATION.OP
     */
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeInstanceReg(SeInstance, SE_OPERATION_0, SeConfigReg);

    // When called in the reader code, the UpdateCryptoStatus2 function will make sure no new
    // operations will start before the engine is idle.
    // Calling function should poll for OP_DONE.

    return;
}
        
void NvBootSeInstanceAesDecryptKeyIntoKeySlot (
        NvBootSeInstance Instance,
        NvU8    SourceKeySlot,
        NvU8    SourceKeySize,
        NvU8    TargetKeySlot,
        NvU8    TargetKeySize,
        NvU8   *Src)
{
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
    NvBootSeInstanceKeySlotWriteKeyIV(Instance, SourceKeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                      0);

    // Initialize SE source IV slot UpdatedIV[127:0] to zero
    NvBootSeInstanceKeySlotWriteKeyIV(Instance, SourceKeySlot,
                      SE_MODE_PKT_AESMODE_KEY128,
                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                      0);

    // Setup SE engine parameters for AES decrypt operation.
    NvBootSeInstanceSetupOpMode(Instance,
                        SE_OP_MODE_AES_CBC,
                        NV_FALSE,
                        NV_TRUE,
                        SE_CONFIG_0_DST_KEYTABLE,
                        SourceKeySlot,
                        SourceKeySize);

    // Setup input linked list.
    InputLinkedList[Instance].LastBufferNumber = 0;
    InputLinkedList[Instance].LLElement.StartByteAddress = (NvU32) Src;
    InputLinkedList[Instance].LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // The SE_CRYPTO_LAST_BLOCK_0 value is calculated by the following formula
    // given in the SE IAS section 3.2.3.1 AES Input Data Size.
    // Input Bytes = 16 bytes * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks*16 = 16 * (1 + SE_CRYPTO_LAST_BLOCK)
    // NumBlocks - 1 = SE_CRYPTO_LAST_BLOCK
    // Only 1 block at a time here, so set this to 0.
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_LAST_BLOCK_0, 0);

    // Set address of input linked list.
    NvBootSetSeInstanceReg(Instance, SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList[Instance]);

    // Set operation to decrypt into lower 128-bits of the key slot.
    SeCryptoKeytableDst = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_DST, WORD_QUAD, KEYS_0_3, SeCryptoKeytableDst);
    SeCryptoKeytableDst = NV_FLD_SET_DRF_NUM(SE, CRYPTO_KEYTABLE_DST, KEY_INDEX, TargetKeySlot, SeCryptoKeytableDst);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DST_0, SeCryptoKeytableDst);

     // Issue START command in SE_OPERATION.OP
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeInstanceReg(Instance, SE_OPERATION_0, SeConfigReg);

    // Wait until engine becomes IDLE.
    while(NvBootSeInstanceIsEngineBusy(Instance, NULL))
        ;

    // If the target key size to be loaded directly into the SE key slot is
    // 128-bits, we are done.
    if(TargetKeySize == SE_MODE_PKT_AESMODE_KEY128)
        return;

    // Setup SE engine parameters for AES decrypt operation.
    NvBootSeInstanceSetupOpMode(Instance,
                        SE_OP_MODE_AES_CBC,
                        NV_FALSE,
                        NV_FALSE,
                        SE_CONFIG_0_DST_KEYTABLE,
                        SourceKeySlot,
                        SourceKeySize);

    // Setup input linked list.
    InputLinkedList[Instance].LastBufferNumber = 0;
    InputLinkedList[Instance].LLElement.StartByteAddress = (NvU32) Src + NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;
    InputLinkedList[Instance].LLElement.BufferByteSize = NVBOOT_SE_AES_BLOCK_LENGTH_BYTES;

    // Set address of input linked list.
    NvBootSetSeInstanceReg(Instance, SE_IN_LL_ADDR_0, (NvU32) &InputLinkedList[Instance]);

    // Set operation to decrypt into upper 128-bits of the key slot.
    SeCryptoKeytableDst = NV_FLD_SET_DRF_DEF(SE, CRYPTO_KEYTABLE_DST, WORD_QUAD, KEYS_4_7, SeCryptoKeytableDst);
    SeCryptoKeytableDst = NV_FLD_SET_DRF_NUM(SE, CRYPTO_KEYTABLE_DST, KEY_INDEX, TargetKeySlot, SeCryptoKeytableDst);
    NvBootSetSeInstanceReg(Instance, SE_CRYPTO_KEYTABLE_DST_0, SeCryptoKeytableDst);

     // Issue START command in SE_OPERATION.OP
    SeConfigReg = NV_DRF_DEF(SE, OPERATION, OP, START);
    NvBootSetSeInstanceReg(Instance, SE_OPERATION_0, SeConfigReg);

    // Wait until engine becomes IDLE.
    while(NvBootSeInstanceIsEngineBusy(Instance, NULL))
        ;

    return;
}

void NvBootSeGetSeStickyBits(NvBootSePkaState *pSePkaState)
{
    // SE1
    for(uint32_t i = 0; i < sizeof(NvBootSeStickyBitsOffsets) / 4; i++)
    {
        pSePkaState->Se1StickyBits[i] = NvBootGetSeInstanceReg(NvBootSeInstance_Se1, NvBootSeStickyBitsOffsets[i]);
    }

    // SE2
    for(uint32_t i = 0; i < sizeof(NvBootSeStickyBitsOffsets) / 4; i++)
    {
        pSePkaState->Se2StickyBits[i] = NvBootGetSeInstanceReg(NvBootSeInstance_Se2, NvBootSeStickyBitsOffsets[i]);

    }

    // PKA1
    for(uint32_t i = 0; i < sizeof(NvBootPkaStickyBitsOffsets) / 4; i++)
    {
        pSePkaState->Pka1StickyBits[i]= NvBootPkaGetPka0Reg(NvBootPkaStickyBitsOffsets[i]);
    }
}

void NvBootSeGetSePkaState(NvBootSePkaState *pSePkaState)
{
    NvBootUtilMemset(pSePkaState, 0, sizeof(NvBootSePkaState));

    // Calculate KCVs of SE key slots and save.
    for(uint8_t KeySlot = 0; KeySlot < NvBootSeAesKeySlot_Num; KeySlot++)
    {
        // Save KCVs to TZRAM for comparison at SC7 exit.
        NvBootKcvComputeKcvSE(&pSePkaState->SE1_KCV[KeySlot], NvBootSeInstance_Se1, KeySlot, AES_KEY_128);
    }

    for(uint8_t KeySlot = 0; KeySlot < NvBootSeAesKeySlot_Num; KeySlot++)
    {
        // Save KCVs to TZRAM for comparison at SC7 exit.
        NvBootKcvComputeKcvSE(&pSePkaState->SE2_KCV[KeySlot], NvBootSeInstance_Se2, KeySlot, AES_KEY_128);
    }

    // Save PKA key slots
    for (uint8_t Slot = 0; Slot < ARSE_PKA1_NUM_KEY_SLOTS; Slot++)
    {
        NvBootPkaReadKeySlot((NvU32 *)&pSePkaState->Pka1KeySlot[Slot], Slot);
    }

    // Save all sticky bits to the buffer
    NvBootSeGetSeStickyBits(pSePkaState);
}

