/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_KCV_INT_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_KCV_INT_H_

#include "nvboot_crypto_aes_param.h"

/**
 * Function to compute the Key Check(sum) Value of an AES key
 * already loaded into an SE key slot.
 * @param[in] Kcv Pointer to uint32_t, where the Kcv will be stored.
 * @param[in] Instance SE instance.
 * @param[in] KeySlot SE key slot number
 * @param[in] Key slot size.
 *
 * @return 32-bit value, the first 3 octets will be the KCV. The fourth octet will be zero.
 *
 * KCV calculation procedure:
 * Encrypts (using AES-ECB) a 16-byte buffer of zeroes using an AES key loaded into an
 * SE key slot.
 * The first 3 bytes of the Kcv Buffer will be the KCV, which can be used
 * to compare against an expected KCV. This procedure checks if
 * the key slot was loaded correctly.
 *
 */
void NvBootKcvComputeKcvSE(uint32_t *Kcv, const NvBootSeInstance SeInstance, const uint8_t KeySlot, const NvBootAesKeySize KeySize);

#endif /* NVBOOT_INCLUDE_T214_NVBOOT_KCV_INT_H__ */

