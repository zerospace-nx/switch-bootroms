/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_se_rsa_dev_int.h"
#include "nvboot_se_int.h"
#include "nvboot_error.h"
#include "nvboot_crypto_rsa_param.h"
#include "arse.h"

/**
 * Function for any required initialization sequence for SE.
 * Note, clock init should be taken care of in the non-secure
 * dispatcher. This function should take care of other non-clock
 * programming.
 *
 * @return NvBootError.
 */
NvBootError NvBootSeRsaDevInit(void)
{
    return NvBootError_Success;
}

/**
 * SE implementation of the NvBootRsaDeviceNumKeySlots function
 * of the nvboot_rsa_device_int.h interface.
 *
 * @return The maximal number of RSA key slots.
 */
uint8_t NvBootSeRsaDevNumKeySlots(void)
{
    return  (uint8_t) ARSE_RSA_NUM_KEY_SLOTS;
}

/**
 * SE implementation of the NvBootAesDeviceIsValid function
 * of the nvboot_aes_device_int.h interface
 *
 * @param[in] KeySlot The KeySlot number to validate.
 *
 * @return true if KeySlot is a valid key slot number. false otherwise.
 */
bool NvBootSeRsaDevIsValidKeySlot(uint8_t KeySlot)
{
    if(KeySlot < NvBootSeRsaDevNumKeySlots())
        return true;
    else
        return false;
}

bool NvBootSeRsaDevIsValidKeySize(NvBootCryptoRsaKeySize KeySize)
{
    if(KeySize == RSA_KEY_2048)
        return true;
    else
        return false;
}

/**
 * Function to read an RSA key from a SE RSA key slot.
 * @param[out] Key The buffer to read the RSA key into.
 * @param[in] KeySlot The key slot number.
 * @param[in] The key size to read.
 *
 * @return NvBootError. Returns NvBootError_InvalidSeKeySlotNum if invalid
 * key slot number used. Returns NvBootError_Success if the operation was
 * successful.
 */
NvBootError NvBootSeRsaDevGetKey(NvBootCryptoRsaKey *Key,
                                 const uint8_t KeySlot,
                                 const NvBootCryptoRsaKeySize KeySize)
{
    if(NvBootSeRsaDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    if(NvBootSeRsaDevIsValidKeySize(KeySize) == false)
        return NvBootError_InvalidSeKeySize;

    NvBootSeRsaReadKey((uint32_t *)&Key->KeyData.RsaKeyMaxSizeSe0NvU32.Modulus, KeySize, KeySlot, SE_RSA_KEY_PKT_EXPMOD_SEL_MODULUS);
    NvBootSeRsaReadKey((uint32_t *)&Key->KeyData.RsaKeyMaxSizeSe0NvU32.Exponent, KeySize, KeySlot, SE_RSA_KEY_PKT_EXPMOD_SEL_EXPONENT);

    Key->KeySize = KeySize;

    return NvBootError_Success;
}


/**
 * Function to load an RSA key to a key slot.
 * @param[in] Key The RSA key to load into the key slot
 * @param[in] KeySlot The key slot number.
 *
 * Note: Key size information is embedded in the NvBootCryptoRsaKey
 * struct.
 */
NvBootError NvBootSeRsaDevSetKey(const NvBootCryptoRsaKey *Key,
                                 const uint8_t KeySlot)
{
    if(NvBootSeRsaDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    if(NvBootSeRsaDevIsValidKeySize(Key->KeySize) == false)
        return NvBootError_InvalidSeKeySize;

    NvBootSeRsaWriteKey((uint32_t *)&Key->KeyData.RsaKeyMaxSizeSe0NvU32, Key->KeySize, Key->KeySize, KeySlot);

    return NvBootError_Success;
}


/**
 * Function to calculate the exponentiation of a base with modulus, of the form
 * c = b ^ e (mod m), where
 * c is the result
 * b is the base, the integer that is repeatedly multiplied
 * e is the exponent
 * m is the modulus
 *
 * @param[in] Base, what is to be repeatedly multiplied
 * @param[out] Result he result of the calculation.
 * @param[in] KeySlot The key slot number of the RSA key to use, which specifies
 *                    the modulus and exponent.
 * @param[in] KeySize A valid RSA key size.
 *
 */
NvBootError NvBootSeRsaDevModularExponentiation(const uint32_t *Base,
                                                uint32_t *Result,
                                                const uint8_t KeySlot,
                                                const NvBootCryptoRsaKeySize KeySize)
{
    if(NvBootSeRsaDevIsValidKeySlot(KeySlot) == false)
        return NvBootError_InvalidSeKeySlotNum;

    if(NvBootSeRsaDevIsValidKeySize(KeySize) == false)
        return NvBootError_InvalidSeKeySize;

    // InputMessageLengthBytes in this case is equal to the key size.
    NvBootSeRsaModularExp(KeySlot, KeySize, KeySize / 8, (uint32_t *) Base, (uint32_t *) Result);

    return NvBootError_Success;
}

bool NvBootSeRsaDevIsEngineBusy(void)
{
    // RSA in t214 will not write to DRAM so we don't need AHB coherency check.
    // Pass a dummy address to force mem check alone.
    return (bool) NvBootSeIsEngineBusy((NvU8*)NV_ADDRESS_MAP_IRAM_A_BASE);
}

/**
 *  Shutdown the Rsa device and clean up state.
 */
void (*NvBootSeRsaDevShutdown)(void);

