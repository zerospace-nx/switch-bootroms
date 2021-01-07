/*
 * Copyright (c) 2006 - 2011 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_se_lp0_context.h
 *
 * Defines the SE LP0 context data structures. 
 *
 */
#ifndef INCLUDED_NVBOOT_SE_LP0_CONTEXT_H
#define INCLUDED_NVBOOT_SE_LP0_CONTEXT_H

#include "nvboot_se_aes.h"
#include "nvboot_se_rsa.h"
#include "nvboot_se_pka.h"
#include "arse.h"
#include "arpka1.h"

#if defined(__cplusplus)
extern "C"
{
#endif

typedef union NvBootSe1Lp0ContextStickyBitsRec
{
    struct 
    {
    NvU32 SeSecurity0_2_0:3;
    NvU32 SeSecurity0_SoftSetting:1; // SoftSetting@Bit16
    NvU32 SeTzramSecurity0_1_0:2;
    NvU32 SeCryptoSecurityPerKey15_0:16;
    NvU32 SeCryptoKeyTableAccess0_7_0:8;
    NvU32 SeCryptoKeyTableAccess1_1_0:2;
    /** Word Boundary **/
    NvU32 SeCryptoKeyTableAccess1_7_2:6;
    NvU32 SeCryptoKeyTableAccess2_7_0:8;
    NvU32 SeCryptoKeyTableAccess3_7_0:8;
    NvU32 SeCryptoKeyTableAccess4_7_0:8;
    NvU32 SeCryptoKeyTableAccess5_1_0:2;
    /** Word Boundary **/
    NvU32 SeCryptoKeyTableAccess5_7_2:6;
    NvU32 SeCryptoKeyTableAccess6_7_0:8;
    NvU32 SeCryptoKeyTableAccess7_7_0:8;
    NvU32 SeCryptoKeyTableAccess8_7_0:8;
    NvU32 SeCryptoKeyTableAccess9_1_0:2;
    /** Word Boundary **/
    NvU32 SeCryptoKeyTableAccess9_7_2:6;
    NvU32 SeCryptoKeyTableAccess10_7_0:8;
    NvU32 SeCryptoKeyTableAccess11_7_0:8;
    NvU32 SeCryptoKeyTableAccess12_7_0:8;
    NvU32 SeCryptoKeyTableAccess13_1_0:2;
    /** Word Boundary **/
    NvU32 SeCryptoKeyTableAccess13_7_2:6;
    NvU32 SeCryptoKeyTableAccess14_7_0:8;
    NvU32 SeCryptoKeyTableAccess15_7_0:8;
    NvU32 SeRsaSecurityPerKey0_1_0:2;
    NvU32 SeRsaKeyTableAccess0_2_0:3;
    NvU32 SeRsaKeyTableAccess1_2_0:3;
    NvU32 SeSecurity0_4:1;
    };
    NvU32 Force2Blocks[8]; // 32 bytes = 2 blocks.
} NvBootSeLp0ContextStickyBits;



typedef union NvBootSe2Pka1Lp0ContextStickyBitsRec
{
    struct {
    NvU32 Pka1SecurityPerKey0_2_0:3;
    NvU32 Pka1SecurityPerKey1_2_0:3;
    NvU32 Pka1SecurityPerKey2_2_0:3;
    NvU32 Pka1SecurityPerKey3_2_0:3;
    NvU32 Pka1KeyTableAccess0_2_0:3;
    NvU32 Pka1KeyTableAccess1_2_0:3;
    NvU32 Pka1KeyTableAccess2_2_0:3;
    NvU32 Pka1KeyTableAccess3_2_0:3;
    NvU32 Pka1Security0_SeHardSetting:1;
    NvU32 Pka1Security0_SeEngDis:1;
    NvU32 Pka1Security0_PerKeySetting:1;
    NvU32 Pka1Security0_SeSoftSetting:1;
    NvU32 Pka1MutexRRTmoutLock:1;
    NvU32 Pka1MutexRRTmoutVal_2_0:3;
    /** Word Boundary **/
    NvU32 Pka1MutexRRTmoutVal_19_3:17;
    NvU32 Pka1NvSecureGroup_13_0:14;
    /** Word Boundary **/
    NvU32 Pka1NvSecureGroup_31_14:18;
    };
    NvU32 Force1Block[4];
} NvBootSePka1Lp0ContextStickyBits;

/*
 * Offsets of all the SE sticky bits. SE1 and SE2 are the same.
 */
static const NvU32 NvBootSeStickyBitsOffsets[] =
{
    SE_SE_SECURITY_0,
    SE_RSA_KEYTABLE_ACCESS_1,
    SE_RSA_KEYTABLE_ACCESS_0,
    SE_RSA_SECURITY_PERKEY_0,
    SE_CRYPTO_KEYTABLE_ACCESS_15,
    SE_CRYPTO_KEYTABLE_ACCESS_14,
    SE_CRYPTO_KEYTABLE_ACCESS_13,
    SE_CRYPTO_KEYTABLE_ACCESS_12,
    SE_CRYPTO_KEYTABLE_ACCESS_11,
    SE_CRYPTO_KEYTABLE_ACCESS_10,
    SE_CRYPTO_KEYTABLE_ACCESS_9,
    SE_CRYPTO_KEYTABLE_ACCESS_8,
    SE_CRYPTO_KEYTABLE_ACCESS_7,
    SE_CRYPTO_KEYTABLE_ACCESS_6,
    SE_CRYPTO_KEYTABLE_ACCESS_5,
    SE_CRYPTO_KEYTABLE_ACCESS_4,
    SE_CRYPTO_KEYTABLE_ACCESS_3,
    SE_CRYPTO_KEYTABLE_ACCESS_2,
    SE_CRYPTO_KEYTABLE_ACCESS_1,
    SE_CRYPTO_KEYTABLE_ACCESS_0,
    SE_CRYPTO_SECURITY_PERKEY_0,
    SE_TZRAM_SECURITY_0
    // SE_SECURITY_0[16] and SE_SECURITY_0[2:0] included in first entry above.
};

/*
 * Offsets of all the PKA sticky bits.
 */
static const NvU32 NvBootPkaStickyBitsOffsets[] =
{
    // PKA1_SECURITY_PERKEY_0_OWNER
    PKA1_PKA1_SECURITY_PERKEY(0),
    // PKA1_SECURITY_PERKEY_1_OWNER
    PKA1_PKA1_SECURITY_PERKEY(1),
    // PKA1_SECURITY_PERKEY_2_OWNER
    PKA1_PKA1_SECURITY_PERKEY(2),
    // PKA1_SECURITY_PERKEY_3_OWNER
    PKA1_PKA1_SECURITY_PERKEY(3),
    // PKA1_KEYTABLE_ACCESS_0
    PKA1_PKA1_KEYTABLE_ACCESS(0),
    // PKA1_KEYTABLE_ACCESS_1
    PKA1_PKA1_KEYTABLE_ACCESS(1),
    // PKA1_KEYTABLE_ACCESS_2
    PKA1_PKA1_KEYTABLE_ACCESS(2),
    // PKA1_KEYTABLE_ACCESS_3
    PKA1_PKA1_KEYTABLE_ACCESS(3),
    /**
    PKA1_SECURITY_0_SE_HARD_SETTING
    PKA1_SECURITY_0_SE_ENG_DIS
    PKA1_SECURITY_0_PERKEY_SETTING
    PKA1_SECURITY_0_SE_SOFT_SETTING
    */
    PKA1_PKA1_SECURITY,
    /**
    PKA1_MUTEX_RR_TMOUT_COUNTER_LOCK
    PKA1_MUTEX_RR_TMOUT_COUNTER_VAL
    */
    PKA1_CTRL_PKA_MUTEX_RR_TMOUT,
    // 80:49	0	PKA1_NVSECURE_GROUP	
    PKA1_PKA1_NVSECURE_GROUP
};

/**
 * Defines the length of the SRK 32-bit words.
 */
enum {NVBOOT_SE_SRK_KEY_LENGTH = 4};

/**
 * Defines the length of the SRK in units of bytes.
 */
enum {NVBOOT_SE_SRK_KEY_LENGTH_BYTES = 16};

/**
 * Defines the length of the Known Pattern in units of 32 bit words.
 */
enum {NVBOOT_SE_KNOWN_PATTERN_LENGTH = 4};

/**
 * Defines the length of the Known Pattern in units of bytes.
 */
enum {NVBOOT_SE_KNOWN_PATTERN_LENGTH_BYTES = 16};

enum {NVBOOT_SE_STICKY_BITS_CONTEXT_LENGTH = 4};

enum {NVBOOT_SE_STICKY_BITS_CONTEXT_LENGTH_BYTES = 16};

/**
 * Define the 128-bit known pattern as defined in the SE IAS. 
 * 
 * Declared static so that any module can include this file without
 * getting "multiply defined" errors. 
 */
static const NvU8 NvBootSeContextKnownPattern[NVBOOT_SE_KNOWN_PATTERN_LENGTH_BYTES] = 
{
    0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08,
    0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};


/**
 * Define a struct for the SE sticky bits / security config temp buffer. 
 */
typedef struct NvBootSeContextStickyBitsRegBufRec
{
    NvU32   SE_SE_SECURITY;
    NvU8    SE_TZRAM_SECURITY;    
    NvU8    SE_CRYPTO_KT_ACCESS[NvBootSeAesKeySlot_Num];
    NvU16   SE_CRYPTO_SECURITY_PERKEY;
    NvU8    SE_RSA_SECURITY_PERKEY;
    NvU8    SE_RSA_KT_ACCESS[NvBootSeRsaKeySlot_Num];
    /**** PKA registers ****/
    NvU32   PKA1_SECURITY_PERKEY_OWNER[ARSE_PKA1_NUM_KEY_SLOTS];
    NvU32   PKA1_KEYTABLE_ACCESS[ARSE_PKA1_NUM_KEY_SLOTS];
    NvU32   PKA1_SECURITY;
    NvU32   CTRL_PKA_MUTEX_RR_TMOUT;
    NvU32   PKA1_NVSECURE_GROUP;
} NvBootSeContextStickyBitsRegBuf;


typedef struct NvBootSeContextAesKeySlotRec
{
    NvBootAes256Key                 Aes256Key;
    NvBootAes128Iv                  OriginalIv;
    NvBootAes128Iv                  UpdatedIv;
} NvBootSeContextAesKeySlot;

/**
 * The scope of the previous #pragma directive from above applies until
 * the end of file or any subsequent pragma pack(n) directive is encountered. 
 * Reset alignment of subsequent struct to 4-byte alignment. 
 * The default is #pragma(8). 
 */

/**
 *  SE1 context blob format
 *  Encrypted / Decrypted context size is:
 *
 *  1 AES block of random data = 16 bytes
 *  2 AES block of sticky bits and security status of each key-slot = 32 bytes
 *  16 256-bit AES keys = 512 bytes
 *  16 128-bit Original IVs = 256 bytes
 *  16 128-bit Updated IVs = 256 bytes
 *  2 4096-bit RSA keys = 1024 bytes
 *  1 128-bit Known Pattern = 16 bytes
 *  ----------------------------------
 *  Total: 2112 bytes
 *
 *  SE_CRYPTO_LAST_BLOCK_0 specifies the lengh of AES data for AES crypto operation
 *  in units of 16-byte blocks. 
 *
 */
typedef struct NvBootSe1Lp0ContextRec
{
    NvBootAes128Key                 RandomAesBlock;
    NvBootSeLp0ContextStickyBits    SeContextStickyBits;
    NvBootSeContextAesKeySlot       AesKeys[NvBootSeAesKeySlot_Num];
    NvBootSeRsaKey2048              RsaKey2048[NvBootSeRsaKeySlot_Num];
    NvU8                            KnownPattern[NVBOOT_SE_KNOWN_PATTERN_LENGTH_BYTES];
} NvBootSe1Lp0Context;


typedef struct NvBootPka1KeySlotRec
{
    // Exponent is words 0-127
    // Modulus is words 128-255
    // m' is words 256-383
    // r2 is words 384-511
    // Maximum key size is 4096-bit RSA key.
    NvU32 SlotData[512];
} NvBootPka1KeySlot;

/**
 *  SE2/PKA Context blob.
 *  
 */
typedef struct NvBootSe2Lp0ContextRec
{
    NvBootAes128Key                 RandomAesBlock;                     // 1 Blk
    NvBootSeLp0ContextStickyBits    SeContextStickyBits;                // 2 Blks
    NvBootSeContextAesKeySlot       AesKeys[NvBootSeAesKeySlot_Num];    // 4 Blks* 16 slots = 64 Blks   
    NvBootSeRsaKey2048              RsaKey2048[NvBootSeRsaKeySlot_Num]; // 32 Blks * 2 slots = 64 Blks
    NvBootSePka1Lp0ContextStickyBits SePka1Lp0ContextStickyBits;        //  1 Blk
    NvBootPka1KeySlot               Pka1KeySlot[ARSE_PKA1_NUM_KEY_SLOTS]; // 128 Blks * 4 slots = 512 Blks
    NvU8                            KnownPattern[NVBOOT_SE_KNOWN_PATTERN_LENGTH_BYTES]; // 1 Blk
} NvBootSe2Lp0Context;


/**
 * SE1, SE2, PKA "State" blob.
 * Used to store all the KCVs of each SE, plus the PKA key slot data, and
 * the values of the sticky bits.
 */
typedef struct NvBootSePkaStateRec
{
    NvU32 SE1_KCV[NvBootSeAesKeySlot_Num];
    NvU32 SE2_KCV[NvBootSeAesKeySlot_Num];
    NvBootPka1KeySlot Pka1KeySlot[ARSE_PKA1_NUM_KEY_SLOTS]; // 128 Blks * 4 slots = 512 Blks
    NvU32 Se1StickyBits[sizeof(NvBootSeStickyBitsOffsets)/4];
    NvU32 Se2StickyBits[sizeof(NvBootSeStickyBitsOffsets)/4];
    NvU32 Pka1StickyBits[sizeof(NvBootPkaStickyBitsOffsets)/4];
} NvBootSePkaState;

typedef union NvBootSeLp0ContextRec
{
    NvBootSe1Lp0Context Se1Context;
    NvBootSe2Lp0Context Se2Context;
}   NvBootSeLp0Context;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SE_LP0_CONTEXT_H
