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

#if defined(__cplusplus)
extern "C"
{
#endif

// Define the masks and shifts for the SE sticky bits to be
// restored by BR at LP0. 
// SE_SECURITY_0[16] is bit 3, SE_SECURITY_0[2:0] are bits 2:0.
#define SE_SECURITY_0_LP0_CONTEXT_MASK_0                    0x0000000F
#define SE_SECURITY_0_LP0_CONTEXT_SHIFT_0                   0

#define SE_SECURITY_0_SE_SOFT_SETTING_STICKY_BIT_FIELD      0x00000008

#define SE_TZRAM_SECURITY_0_LP0_CONTEXT_MASK_0              0x00000030
#define SE_TZRAM_SECURITY_0_LP0_CONTEXT_SHIFT_0             4

#define SE_CRYPTO_SECURITY_PERKEY_0_LP0_CONTEXT_MASK_0      0x003FFFC0
#define SE_CRYPTO_SECURITY_PERKEY_0_LP0_CONTEXT_SHIFT_0     6

#define SE_CRYPTO_KEYTABLE_ACCESS_0_LP0_CONTEXT_MASK_0      0x1FC00000
#define SE_CRYPTO_KEYTABLE_ACCESS_0_LP0_CONTEXT_SHIFT_0     22

#define SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_MASK_0      0xE0000000
#define SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_SHIFT_0     29
#define SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_MASK_1      0x0000000F
#define SE_CRYPTO_KEYTABLE_ACCESS_1_LP0_CONTEXT_SHIFT_1     3

#define SE_CRYPTO_KEYTABLE_ACCESS_2_LP0_CONTEXT_MASK_0      0x000007F0
#define SE_CRYPTO_KEYTABLE_ACCESS_2_LP0_CONTEXT_SHIFT_0     4

#define SE_CRYPTO_KEYTABLE_ACCESS_3_LP0_CONTEXT_MASK_0      0x0003F800
#define SE_CRYPTO_KEYTABLE_ACCESS_3_LP0_CONTEXT_SHIFT_0     11

#define SE_CRYPTO_KEYTABLE_ACCESS_4_LP0_CONTEXT_MASK_0      0x01FC0000
#define SE_CRYPTO_KEYTABLE_ACCESS_4_LP0_CONTEXT_SHIFT_0     18

#define SE_CRYPTO_KEYTABLE_ACCESS_5_LP0_CONTEXT_MASK_0      0xFE000000
#define SE_CRYPTO_KEYTABLE_ACCESS_5_LP0_CONTEXT_SHIFT_0     25

#define SE_CRYPTO_KEYTABLE_ACCESS_6_LP0_CONTEXT_MASK_0      0x0000007F
#define SE_CRYPTO_KEYTABLE_ACCESS_6_LP0_CONTEXT_SHIFT_0     0

#define SE_CRYPTO_KEYTABLE_ACCESS_7_LP0_CONTEXT_MASK_0      0x00003F80
#define SE_CRYPTO_KEYTABLE_ACCESS_7_LP0_CONTEXT_SHIFT_0     7

#define SE_CRYPTO_KEYTABLE_ACCESS_8_LP0_CONTEXT_MASK_0      0x001FC000
#define SE_CRYPTO_KEYTABLE_ACCESS_8_LP0_CONTEXT_SHIFT_0     14

#define SE_CRYPTO_KEYTABLE_ACCESS_9_LP0_CONTEXT_MASK_0      0x0FE00000
#define SE_CRYPTO_KEYTABLE_ACCESS_9_LP0_CONTEXT_SHIFT_0     21

#define SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_MASK_0     0xF0000000
#define SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_SHIFT_0    28
#define SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_MASK_1     0x00000007
#define SE_CRYPTO_KEYTABLE_ACCESS_10_LP0_CONTEXT_SHIFT_1    4

#define SE_CRYPTO_KEYTABLE_ACCESS_11_LP0_CONTEXT_MASK_0     0x000003F8
#define SE_CRYPTO_KEYTABLE_ACCESS_11_LP0_CONTEXT_SHIFT_0    3

#define SE_CRYPTO_KEYTABLE_ACCESS_12_LP0_CONTEXT_MASK_0     0x0001FC00
#define SE_CRYPTO_KEYTABLE_ACCESS_12_LP0_CONTEXT_SHIFT_0    10

#define SE_CRYPTO_KEYTABLE_ACCESS_13_LP0_CONTEXT_MASK_0     0x00FE0000
#define SE_CRYPTO_KEYTABLE_ACCESS_13_LP0_CONTEXT_SHIFT_0    17

#define SE_CRYPTO_KEYTABLE_ACCESS_14_LP0_CONTEXT_MASK_0     0x7F000000
#define SE_CRYPTO_KEYTABLE_ACCESS_14_LP0_CONTEXT_SHIFT_0    24

#define SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_MASK_0     0x80000000
#define SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_SHIFT_0    31
#define SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_MASK_1     0x0000003F
#define SE_CRYPTO_KEYTABLE_ACCESS_15_LP0_CONTEXT_SHIFT_1    1

#define SE_RSA_SECURITY_PERKEY_0_LP0_CONTEXT_MASK_0         0x000000C0
#define SE_RSA_SECURITY_PERKEY_0_LP0_CONTEXT_SHIFT_0        6

#define SE_RSA_KEYTABLE_ACCESS_0_LP0_CONTEXT_MASK_0         0x00000700
#define SE_RSA_KEYTABLE_ACCESS_0_LP0_CONTEXT_SHIFT_0        8

#define SE_RSA_KEYTABLE_ACCESS_1_LP0_CONTEXT_MASK_0         0x00003800
#define SE_RSA_KEYTABLE_ACCESS_1_LP0_CONTEXT_SHIFT_0        11

// The start location of the SE decrypted context.
enum {NVBOOT_SE_DECRYPTED_CONTEXT_START = NV_ADDRESS_MAP_IRAM_A_BASE};

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
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

/**
 * Define a struct for the SE sticky bits / security config temp buffer. 
 */
typedef struct NvBootSeContextStickyBitsBufferRec
{
    NvU32   SE_SE_SECURITY;
    NvU8    SE_TZRAM_SECURITY;    
    NvU8    SE_RSA_SECURITY_PERKEY;
    NvU8    SE_RSA_KT_ACCESS[NvBootSeRsaKeySlot_Num];
    NvU8    SE_CRYPTO_KT_ACCESS[NvBootSeAesKeySlot_Num];
    NvU16   SE_CRYPTO_SECURITY_PERKEY;
} NvBootSeContextStickyBitsBuffer;


/**
 * 32 bytes of sticky bits data
 */
typedef struct NvBootSeLp0ContextStickyBitsRec
{
    NvU32   StickyBits[8];
} NvBootSeLp0ContextStickyBits;

/**
 * The scope of the previous #pragma directive from above applies until
 * the end of file or any subsequent pragma pack(n) directive is encountered. 
 * Reset alignment of subsequent struct to 4-byte alignment. 
 * The default is #pragma(8). 
 */
#pragma pack(4)
/**
 *
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
typedef struct NvBootSeLp0ContextRec
{
    NvBootAes128Key                 RandomAesBlock;
    NvBootSeLp0ContextStickyBits    SeContextStickyBits;
    NvBootAes256Key                 Aes256Key[NvBootSeAesKeySlot_Num];
    NvBootAes128Iv                  OriginalIv[NvBootSeAesKeySlot_Num];
    NvBootAes128Iv                  UpdatedIv[NvBootSeAesKeySlot_Num];
    NvBootSeRsaKey2048              RsaKey2048[NvBootSeRsaKeySlot_Num];
    NvU8                            KnownPattern[NVBOOT_SE_KNOWN_PATTERN_LENGTH_BYTES];
} NvBootSeLp0Context;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_SE_LP0_CONTEXT_H
