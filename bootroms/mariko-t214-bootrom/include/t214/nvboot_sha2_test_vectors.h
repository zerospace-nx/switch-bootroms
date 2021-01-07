/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_SHA2_TEST_VECTORS_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_SHA2_TEST_VECTORS_H_

#if defined(__cplusplus)
extern "C"
{
#endif

/**
    Test vectors from http://csrc.nist.gov/groups/ST/toolkit/documents/Examples/SHA_All.pdf
    and http://csrc.nist.gov/groups/STM/cavp/documents/shs/shabytetestvectors.zip.
    Null string test vector is from SHA256ShortMsg.rsp from shabytetestvectors.zip.
*/

unsigned char sha256_test_input[3][56] =
{
    { "abc" },
    { "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" },
    { "" } // null string, length 0
};

uint32_t sha256_test_input_len[3] =
{
    3, 56, 0,
};

uint8_t sha256_test_digests[3][32] =
{
    {   // "abc"
        0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA,
        0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
        0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C,
        0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD,
    },
    {   // "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
        0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8,
        0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39,
        0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
        0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1,
    },
    {   // ""; null string, length 0
        0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
        0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
        0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
        0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
    },

};
#if defined(__cplusplus)
}
#endif

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_SHA2_TEST_VECTORS_H_ */

