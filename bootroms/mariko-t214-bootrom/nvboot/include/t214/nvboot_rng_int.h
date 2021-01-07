/* Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_RNG_INT_H
#define NVBOOT_RNG_INT_H

/**
 *  @file nvboot_rng_int.h
 *
 */
 
/**
 *  RNG State.
 */
typedef enum 
{
    RNG_ENABLED=0x454e42, // ENB
    RNG_DISABLED=0x444953, // DIS
    RNG_FORCE32=0xffffffff
} NvBootRngState;

/**
 *  @brief Generate Random Number in the range 0 - (Max-1)
 *  @param Max Upper limit of random number. Should be power of 2. Will be enforced by routine 
 *         regardless.
 *  @return Random Number 0 - (Max-1)
 */
NvU32 NvBootRngGenerateRandom(NvU32 Max);

/**
 *  @brief Wait Random
 *  @param MaxDelay Delay in Range 0 - MaxDelay-1
 *  @return void
 */
void NvBootRngWaitRandom(NvU32 MaxDelay);

/**
 *  @brief Initialize RNG
 *  @return NvBootError
 */
NvBootError NvBootRngInit(void);

/**
 *  @brief Wait Random
 *  @param MaxCycles in Range 0 - MaxCycles-1
 *  @return void
 */
void NvBootRngWaitRandomLoop(NvU32 MaxCycles);

#endif // NVBOOT_RNG_INT_H
