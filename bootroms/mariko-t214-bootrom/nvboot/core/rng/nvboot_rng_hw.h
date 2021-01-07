/* Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#ifndef NVBOOT_RNG_HW_H
#define NVBOOT_RNG_HW_H

/**
 *  @file nvboot_rng_hw.h
 *
 *  SE Random number generator h/w routines.
 */
 #include "arse.h"
 #include "nvboot_rng_int.h"
 
typedef enum
{
    GENERATE=SE_RNG_CONFIG_0_MODE_NORMAL,
    INSTANTIATE=SE_RNG_CONFIG_0_MODE_FORCE_INSTANTION,
    FORCE_RESEED=SE_RNG_CONFIG_0_MODE_FORCE_RESEED
} NvBootSeRngOp;

/**
*  @brief SE Random Number Generator Operations
 *  @param RngOp Instantitate, Generate or Reseed.
 *         Instantiate      Seeds RNG with Entropy/Nonce/Personalization String from ECID. Also returns random number
 *         Generate         Generate Random Number
 *         Force_reseed     Force_reseed. Will be forced if Reseed_Cntr_Exhausted
 *  @return 32 bit Random Number
 */
NvU32 NvBootSeRngOperation(NvBootSeRngOp RngOp);

#define NvBootSeRngGenerateRandom() NvBootSeRngOperation(GENERATE)

/**
 *  @brief Initialize SE RNG
 *  @param RngState Pass back RngState.
 *         It is possible for RngState to be disabled yet not an error (INT boards) 
 *  @return NvBootError
 */
NvBootError NvBootSeRngHwInit(NvBootRngState*);

#endif // NVBOOT_RNG_HW_H