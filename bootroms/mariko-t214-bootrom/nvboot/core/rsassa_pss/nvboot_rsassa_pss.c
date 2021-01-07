/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <stddef.h>
#include "nvboot_crypto_pkc_rsassa_pss_int.h"
#include "nvboot_rsa_devmgr_int.h"
#include "nvboot_sha_devmgr_int.h"
#include "nvboot_util_int.h"
#include "nvboot_rng_int.h"
//NV debug #include "nvboot_printf_int.h"
#include "nvboot_bpmp_int.h"

static void ReverseList(uint8_t *original, uint32_t listSize)
{
    uint8_t i, j, temp;

    for(i = 0, j = listSize - 1; i < j; i++, j--)
    {
        temp = original[i];
        original[i] = original[j];
        original[j] = temp;
    }
}

/**
MGF1 is a Mask Generation Function based on a hash function.

   MGF1 (mgfSeed, maskLen)
   where mgfSeed = H
   maskLen = emLen - hLen - 1

   Options:
   Hash     hash function (hLen denotes the length in octets of the hash
            function output)

   Input:
   mgfSeed  seed from which mask is generated, an octet string
   maskLen  intended length in octets of the mask, at most 2^32 hLen

   Output:
   mask     mask, an octet string of length maskLen

   Error:   "mask too long"
*/
//
                          // emLen                          - hLen    - 1 (0xbc)
                          // emLen is Ceil(modBits-1/8), we will use modulus size
                          // to get the same.
#define MAX_MGF_MASKLEN (NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES - RSASSA_PSS_HASH_FUNCTION_SIZE_BYTES - 1)

/**
 * The size of the T buffer which holds the calculated mask should be of size
 * Ceil(maskLen / hLen) * hLen bytes (i.e. the number of iterations of the loop possible
 * times the hLen size).
 * In the case of a 2048-bit RSA key and SHA256, this should be
 * Ceil(223/32) * 32 bytes = 224 bytes.
 *
 * Note, maskLen can be less than 224, so only place maskLen bytes into the
 * T buffer.
 */
#define T_BUF_SIZE_BYTES (NV_ICEIL(MAX_MGF_MASKLEN, RSASSA_PSS_HASH_FUNCTION_SIZE_BYTES) * RSASSA_PSS_HASH_FUNCTION_SIZE_BYTES)
#define T_BUF_SIZE_WORDS NV_ICEIL(T_BUF_SIZE_BYTES, 4)
uint8_t VT_CRYPTO_BUFFER T_Buf[T_BUF_SIZE_BYTES] __attribute__((aligned(4)));

NvBootError NvBootCryptoRsaSsaPssMGF(uint8_t *mgfSeed, uint32_t maskLen, uint8_t *dbMaskBuffer, NvBootShaDevMgr *ShaDevMgr)
{
    // Step 1. If maskLen > 2^32 hLen, output "mask too long" and stop.
    // maskLen is uint32_t, can not be > 2^32 hLen.

    // Step 2. Let T be the empty octet string.

    // Step 3. For counter from 0 to Ceiling(maskLen / hLen) - 1, do the
    //         following:
    //         a. Convert counter to an octet string C of length 4 octets:
    //              C = I2OSP(counter, 4)
    //         b. Concatentate the hash of the seed mgfSeed and C to the octet
    //         string T:
    //         T = T || Hash(mgfSeed||C).

    // Step 3a calls for counter to be an octet string of length 4, which
    // is implicit with uint32_t.
    uint32_t counter = 0;
    const uint32_t hLen = RsaSsaPssHashFunction.ShaDigestSize / 8;
    // The MGF1 spec defines the number of iterations as "For counter from 
    // 0 to Ceil(maskLen/hLen)-1", i.e. 0 to 6 inclusive. 
    // Therefore "loops" below is NOT the number of iterations.
    // The number of iterations is loops + 1.
    const uint32_t loops = (NV_ICEIL(maskLen, hLen)) - 1;
    //NV debug NvBootPrintf("NvBootCryptoRsaSsaPssMGF(). hLen: %d. loop iterations: %d. maskLen: %d\r\n", hLen, loops+1, maskLen);
    // hashInputBuffer is the buffer for (mgfSeed || C), C accounts for the +1.
    uint32_t hashInputBuffer[RsaSsaPssHashFunctionSizeWords + 1];
    // Initialze e to a non-success value. Double check that it is not success before proceeding.
    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

#if 1
    NvBootUtilMemset(T_Buf, 0, T_BUF_SIZE_BYTES);
    T_Buf[0xbe] = 0x01;
#endif
    // MGF1 says "For counter from 0 to Ceil(maskLen/hLen)-1"
    // i.e.  loop from 0 to Ceil(223/32)-1 inclusive
    //       loop from 0 to 6 inclusive
    //       (i.e. 7 loops)
    for(counter = 0; counter <= loops; counter++)
    {
        NvBootUtilMemcpy((uint32_t *) &hashInputBuffer, mgfSeed, hLen);
        hashInputBuffer[(hLen / 4) + 1 - 1] = NvBootUtilSwapBytesInNvU32(counter);

        e = NvBootInitializeNvBootError();
        if(e == NvBootError_Success)
            do_exception();

        e = ShaDevMgr->ShaDevMgrCallbacks->ShaHash((uint32_t *) &hashInputBuffer,
                                                hLen + 4,
                                                (uint32_t *) &T_Buf[counter*hLen],
                                                &ShaDevMgr->ShaConfig);
        if(e != NvBootError_Success)
            return e;
        //NV debug NvBootPrintf("MGF: copying into offset %d of T_Buf. Iteration %d.\r\n", counter*hLen, counter);
    }
    // Fill the dbMaskBuffer from the T_Buf, up to maskLen bytes.
    NvBootUtilMemcpy(dbMaskBuffer, &T_Buf, maskLen);
    //NV debug NvBootPrintf("NvBootCryptoRsaSsaPssMGF(): T_Buf size: %d. bytes copied %d\r\n", sizeof(T_Buf), maskLen);
    return NvBootError_Success;
}

/**
                                  +-----------+
                                  |     M     |
                                  +-----------+
                                        |
                                        V
                                      Hash
                                        |
                                        V
                          +--------+----------+----------+
                     M' = |Padding1|  mHash   |   salt   |
                          +--------+----------+----------+
                                         |
               +--------+----------+     V
         DB =  |Padding2|maskedseed|   Hash
               +--------+----------+     |
                         |               |
                         V               |    +--+
                        xor <--- MGF <---|    |bc|
                         |               |    +--+
                         |               |      |
                         V               V      V
               +-------------------+----------+--+
         EM =  |    maskedDB       |maskedseed|bc|
               +-------------------+----------+--+
*/
// For ease of debugging, and future-proofing for future chips, declare
// the relevant buffers for RSASSA-PSS outside the verify function.
// ModExpResult is the buffer to hold the modular exponentiation result.
uint32_t VT_CRYPTO_BUFFER ModExpResult[NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS];
// EM is the result of I2OSP(m, emLen), where m is the result of RSAVP1.
// In other words, EM is the octet stream after the modular exponentiation
// operation.
// IMPORTANT NOTE: Pointers declared in VT_CRYPTO_BUFFER must be initialized
// manually, see the init function.
uint8_t VT_CRYPTO_BUFFER *EM;
// maskedDB is the leftmost emLen - hLen - 1 octets of EM.
uint8_t VT_CRYPTO_BUFFER *maskedDB; // Pointing to where EM points.
// H is the octets after maskedDB (maskedseed+0xbc above in EM)
uint8_t VT_CRYPTO_BUFFER *H; // Need to offset this by maskedDBLen before usage; See code below.
// DB is the result of maskedDB xor dbMask, it becomes masked DB in the encoding step.
uint8_t VT_CRYPTO_BUFFER DB[NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES] __attribute__((aligned(4))); // DB is a subset of EM.
// The buffer for the output of the mask generation function. The buffer size
// required is maskLen = maskedDBLen = emLen - hLen - 1. Use emLen as the buffer size,
// since the size of the MGF output will always be smaller than emLen. The PKCS spec
// calls the MGF output the "dbMask", with length of maskLen.
uint8_t VT_CRYPTO_BUFFER dbMask[NVBOOT_RSA_MAX_MODULUS_SIZE_BYTES] __attribute__((aligned(4)));
// mHash is the result of Hash(M).
uint32_t VT_CRYPTO_BUFFER mHash[RSASSA_PSS_HASH_FUNCTION_SIZE_WORDS];

NvBootError NvBootCryptoRsaSsaPssInit(void)
{
    // Initialize pointers in the VT_CRYPTO_BUFFER region, these are not
    // done automatically by the startup code.
    EM = (uint8_t *) &ModExpResult;
    // maskedDB is the leftmost emLen - hLen - 1 octets of EM.
    maskedDB = (uint8_t *) &ModExpResult; //EM;
    // H is the octets after maskedDB (maskedseed+0xbc above in EM)
    H = (uint8_t *) &ModExpResult; // Need to offset this by maskedDBLen before usage; See code below.
    return NvBootError_Success;
}

/**
 * Glossary of terms:
 *
 * modBits = length in bits of the RSA modulus n.
 * emBits = modBits - 1.
 * emLen = Ceil(emBits/8)
 * EM = encoded message, octet string of length emLen
 * sLen = Salt length; usually equal to your chosen hash function digest length, or 0.
 *        See explanation in the RSASSA-PSS spec.
 * hLen = size in bytes of your chosen hash function digest length
 *
 */
NvBootError NvBootCryptoRsaSsaPssVerify(NvBootCryptoRsaSsaPssContext *RsaSsaPssContext, NvBootShaDevMgr *ShaDevMgr, NvBootRsaDevMgr *RsaDevMgr)
{
    if(RsaDevMgr->RsaDevMgrCallbacks->IsValidKeySize(RsaSsaPssContext->RsaKey->KeySize) == false)
        return NvBootError_Unsupported_RSA_Key_Size;

    NvBootError e = NvBootInitializeNvBootError();
    if(e == NvBootError_Success)
        do_exception();

    // emLen = \ceil ((modBits - 1)/8) octets, where modBits
    // is the length in bits of the RSA modulus n.
    const uint32_t emLen = NV_ICEIL(RsaSsaPssContext->RsaKey->KeySize - 1, 8);
    const uint8_t hLen = NV_ICEIL(RsaSsaPssHashFunction.ShaDigestSize, 8);
    const uint32_t sLen = NvBootCryptoRsaSsaPssSaltLength;

    const uint32_t RsaKeyBytes = NV_ICEIL(RsaSsaPssContext->RsaKey->KeySize, 8);

    /**
        1. Length checking: If the length of the signature S is not k octets,
           output "invalid signature" and stop.

           InputSignature in NvBootCryptoRsaSsaPssContext is of
           NvBootCryptoRsaSsaPssSig type, which is the necessary size.
           So, the length of the signature is always k octets, where k is the size of
           public key modulus.
    */

    /**
        2. RSA verification:
          a. Convert the signature S to an integer signature representative
             s (see Section 4.2):
                s = OS2IP (S).

             The OS2IP step is skipped here (to convert from octet string to
             non-negative integer), as the modular exponentiation step is done
             directly on the octet string.
    */

    /**
     *
     *  b. Apply the RSAVP1 verification primitive (Section 5.2.2) to the
         RSA public key (n, e) and the signature representative s to
         produce an integer message representative m:

            m = RSAVP1 ((n, e), s).

         If RSAVP1 output "signature representative out of range,"
         output "invalid signature" and stop.
         RSAVP1 defines in range as "message representative, an integer between 0 and n - 1"

         RSAVP1 is the modular exponentiation calculation, m = s^e mod n.

         s in our case is InputMessage, e = 0x10001 (the public exponent),
         n is the public key modulus.

         The exponent and modulus is expected to be loaded into the key slot
         specified in the RsaSsaPssContext.
     */

    // First step of RSAVP1. Check if signature representative s (an integer)
    // is between 0 and n-1 inclusive.
    if(NvBootUtilCmpBigUnsignedInt((uint8_t *) RsaSsaPssContext->InputSignature,
                                   (uint8_t *) RsaSsaPssContext->RsaKey->KeyData.RsaKeyMaxSizeNvU8.Modulus,
                                   NV_ICEIL(RsaSsaPssContext->RsaKey->KeySize, 8)) != NVBOOT_BIGINT_LT)
    {
        return NvBootError_RSA_SSA_PSS_Signature_Out_Of_Range;
    }

    // Modular exponentiation step, using s as the input. Public key is loaded
    // into key slot already.
    e = NvBootInitializeNvBootError(); // reset e to non-success.
    if(e == NvBootError_Success)
        do_exception();

#if 1
    NvBootUtilMemset(ModExpResult, 0, sizeof(uint32_t)*NVBOOT_RSA_MAX_EXPONENT_SIZE_WORDS);
    ModExpResult[0] = 0xbc;
#endif
    e = RsaDevMgr->RsaDevMgrCallbacks->RsaModularExponentiation((uint32_t *) RsaSsaPssContext->InputSignature,
                                                            (uint32_t *) &ModExpResult,
                                                            RsaSsaPssContext->RsaKeySlot,
                                                            RsaSsaPssContext->RsaKey->KeySize);
    if(e != NvBootError_Success)
        return e;

    // Wait for modular exponentiation calculation to finish.
    while(RsaDevMgr->RsaDevMgrCallbacks->IsDeviceBusy() == true)
        ;

    // After the RSAVP1 step, the message representative m is stored in
    // in ascending order in a byte array, i.e. the 0xbc trailer field is the
    // first value of the array, when it is the "last" value in the spec.
    // Reversing the byte order in the array will match the byte ordering
    // of the PKCS #1 spec and make for code that directly matches the spec.
    ReverseList((uint8_t *) &ModExpResult, RsaKeyBytes);

    /**
        c. Convert the message representative m to an encoded message EM
           of length emLen = \ceil ((modBits - 1)/8) octets, where modBits
           is the length in bits of the RSA modulus n (see Section 4.1):

           EM = I2OSP (m, emLen).

           Note that emLen will be one less than k if modBits - 1 is
           divisible by 8 and equal to k otherwise.  If I2OSP outputs
           "integer too large," output "invalid signature" and stop.

           Since the result "ModExpResult" from above is already in octet format,
           skip the conversion from integer to octet per spec. As well, see the first
           check for valid key size at the start of this function, which will check
           if the input key size is a valid RSA key size.
    */

    /**
     * 3. EMSA-PSS verification: Apply the EMSA-PSS verification operation
      (Section 9.1.2) to the message M and the encoded message EM to
      determine whether they are consistent:

         Result = EMSA-PSS-VERIFY (M, EM, modBits - 1), where the third
         parameter is "emBits".

                                  +-----------+
                                  |     M     |
                                  +-----------+
                                        |
                                        V
                                      Hash
                                        |
                                        V
                          +--------+----------+----------+
                     M' = |Padding1|  mHash   |   salt   |
                          +--------+----------+----------+
                                         |
               +--------+----------+     V
         DB =  |Padding2|maskedseed|   Hash
               +--------+----------+     |
                         |               |
                         V               |    +--+
                        xor <--- MGF <---|    |bc|
                         |               |    +--+
                         |               |      |
                         V               V      V
               +-------------------+----------+--+
         EM =  |    maskedDB       |maskedseed|bc|
               +-------------------+----------+--+
    __________________________________________________________________

    Figure 2: EMSA-PSS encoding operation.  Verification operation
    follows reverse steps to recover salt, then forward steps to
    recompute and compare H.

    */
    const uint32_t emBits = RsaSsaPssContext->RsaKey->KeySize - 1;

    /**
     * Step 1 of EMSA-PSS verification.
     * If the length of M is greater than the input limitation for the hash
     * function, output "inconsistent" and stop. In our case, the maximum
     * input message length is a uint32_t, which is smaller than the maximum
     * SHA2 message size.
     */

    /**
     * Step 2 of EMSA-PSS verification.
     * Calculate mHash = Hash(M), an octet string of length hLen, if InputMessageIsHashed
     * is false. It is possible to pass in the hash of the message, precalculated.
     */
    if(RsaSsaPssContext->InputMessageIsHashed == false)
    {
        ShaDevMgr->ShaDevMgrCallbacks->ShaHash(RsaSsaPssContext->InputMessage,
                                               RsaSsaPssContext->InputMessageLengthBytes,
                                               (uint32_t *)&mHash,
                                               &ShaDevMgr->ShaConfig);
    }
    else
    {
        NvBootUtilMemcpy(&mHash, RsaSsaPssContext->InputMessageShaHash, sizeof(mHash));
    }

    /**
        Step 3 of EMSA-PSS verification.
        If emLen < hLen + sLen + 2, output "inconsistent" and stop.
    */
    if(emLen < (hLen + sLen + 2))
        return NvBootError_RsaPssVerify_Inconsistent_1;

    /**
        Step 4 of EMSA-PSS verification.
        If the rightmost octet of EM does not have hexadecimal value
        0xbc, output "inconsistent" and stop.

        EM = encoded message, an octet string of length emLen = \ceil (emBits/8)
     */
    if(EM[emLen - 1] != 0xbc)
        return NvBootError_RsaPssVerify_Inconsistent_bc;

    /**
        Step 5 of EMSA-PSS verification.
        Let maskedDB be the leftmost emLen - hLen - 1 octets of EM, and
        let H be the next hLen octets.
     */

    const uint32_t maskedDBLen = emLen - hLen - 1;

    /**
        Point H to the proper location in EM (after maskedDB).
              +-------------------+----------+--+
        EM =  |    maskedDB       |maskedseed|bc|
              +-------------------+----------+--+
     *                            ^
     *                            H starts here.
    */
    H = (uint8_t *) &ModExpResult + maskedDBLen;

    // Step 6 of EMSA-PSS verification.
    // If the leftmost 8emLen - emBits bits of the leftmost
    // octet in maskedDB are not all equal to zero, output "inconsistent"
    // and stop.
    // 8emLen - emBits = 8*256 - 2047 = 1 (assuming 2048 key size and sha256)
    uint8_t LowestBits = (8*emLen) - (emBits);
    uint8_t FirstOctetMaskedDB = maskedDB[0] & (0xFF << (8 - LowestBits));
    if(FirstOctetMaskedDB > 0)
    {
        return NvBootError_RsaPssVerify_Inconsistent_2;
    }

    // Step 7. Run mask generation function.
    // Let dbMask = MGF(H, emLen - hLen - 1), where H is the mgfSeed and
    // emLen - hLen - 1 is the maskLen.
    //
    // emLen = Ceil((modBits-1) / 8)
    //       = 2048-1/8
    //       = 256.
    //       modBits is the length in bits of the RSA modulus n.
    //
    // maskLen = emLen - hLen - 1 = 256 - 32 - 1 = 223 bytes
    // worst case maskLen (3072-bit key) =  (max emLen) - (smallest hLen) - 1
    //                    = (Ceil (3072-1/8) = 384) - (sha256 digest length = 32) - 1
    //                    = 384 - 32 - 1
    //                    = 351
    // worst case maskLen (2048-bit key) =  (max emLen) - (smallest hLen) - 1
    //                    = (Ceil (2048-1/8) = 256) - (sha256 digest length = 32) - 1
    //                    = 256 - 32 - 1
    //                    = 223
    // mgfSeed = H = hLen = 32 octets
    // maskedDBLen is also maskLen = emLen - hLen - 1.
    // dbMaskBuffer = maskLen
    //uint8_t dbMask[maskedDBLen] __attribute__((aligned(4)));
    e = NvBootInitializeNvBootError(); // default to non-success.
    if(e == NvBootError_Success)
        do_exception();

    // Zero out the whole dbMask buffer, not just maskedDBLen.
    NvBootUtilMemset(dbMask, 0, sizeof(dbMask));
    e = NvBootCryptoRsaSsaPssMGF(H, maskedDBLen, (uint8_t *) &dbMask, ShaDevMgr);
    if(e != NvBootError_Success)
    {
        return NvBootError_RsaPssVerify_Inconsistent_MGF;
    }

    // Step 8. Let DB = maskedDB XOR dbMask
    //anw uint8_t DB[maskedDBLen] __attribute__((aligned(4)));
    for(uint32_t i = 0; i < maskedDBLen; i++)
    {
        DB[i] = maskedDB[i] ^ dbMask[i];
    }

    // Step 9. Set the leftmost 8emLen - emBits bits of the leftmost
    // octet in DB to zero.
    DB[0] &= ~(0xFF << (8 - LowestBits));

    // Step 10. If the emLen - hLen - sLen - 2 leftmost octets of DB are not
    // zero or if the octet at position emLen - hLen - sLen - 1 (the leftmost
    // or lower position is "position 1") does not have hexadecimal value
    // 0x01, output "inconsistent" and stop.
    uint8_t step10_result = 0;
    for(uint32_t i = 0; i < (emLen - hLen - sLen - 2); i++)
    {
        // Always run all iterations, to make this function as
        // near const runtime as possible.
        step10_result |= DB[i];
    }
    if(step10_result != 0)
    {
        return NvBootError_RsaPssVerify_Inconsistent_3;
    }

    // if octet at position emLen - hLen - sLen - 1 does not have
    // the value 0x01, return inconsistent.
    // e.g. 256 - 32 - 32 - 1 = 191th position
    // position 191 is 190th element of the array, so subtract by 1 more.
    if(DB[emLen - hLen - sLen - 1 - 1] != 0x1)
    {
        return NvBootError_RsaPssVerify_Inconsistent_4;
    }

    // Step 11. Let salt be the last sLen octets of DB.
    // Step 12. Let M' = 0x 00 00 00 00 00 00 00 00 || mHash || salt;
    // Set eight initial octets to 0.
    uint8_t M_Prime[8 + RsaSsaPssHashFunctionSizeBytes + sLen];
    // Only the first 8 bytes need be zeroed, just zero out the whole buffer
    // anyway.
    NvBootUtilMemset(&M_Prime, 0, sizeof(M_Prime));
    NvBootUtilMemcpy(&M_Prime[8], &mHash[0], hLen);
    // Copy salt to M_Prime. Note: DB is an octet string of length
    // emLen - hLen - 1. Subtract sLen from DB length to get salt location.
    NvBootUtilMemcpy(&M_Prime[8+hLen], &DB[(emLen - hLen - 1) - sLen], sLen);

    // Step 13. Let H' = Hash(M')
    uint8_t H_Prime[RsaSsaPssHashFunctionSizeBytes] __attribute__((aligned(4)));
    e = NvBootInitializeNvBootError(); // default to non-success.
    if(e == NvBootError_Success)
        do_exception();

    e = ShaDevMgr->ShaDevMgrCallbacks->ShaHash((uint32_t *) &M_Prime,
                                                8 + hLen + sLen,
                                                (uint32_t *) &H_Prime,
                                                &ShaDevMgr->ShaConfig);

    if(e != NvBootError_Success)
    {
        return NvBootError_RsaPssVerify_ShaHash_Error;
    }

    // Add random delay to the end of the function to help mitigate against
    // attacks on r0 immediately at function return.
    // For example:
    //
    // 1036ea:	f004 fd89 	bl	108200 <NvBootCryptoRsaSsaPssVerify>
    // 1036ee:	2800      	cmp	r0, #0 <-- if an attacker can somehow glitch here
    //                                     any mitigations after this would not be
    //                                     effective.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    // Step 14. If H = H' output "consistent". Otherwise, output "inconsistent".
    if(NvBootUtilCompareConstTimeFI(H, &H_Prime, hLen) != FI_TRUE)
    {
        return NvBootError_RsaPssVerify_Inconsistent_5;
    }
    else
    {
        return NvBootError_Success;
    }
}
