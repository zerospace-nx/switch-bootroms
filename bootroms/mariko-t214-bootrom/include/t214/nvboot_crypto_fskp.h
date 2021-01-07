/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_CRYPTO_FSKP_H
#define INCLUDED_NVBOOT_CRYPTO_FSKP_H

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum
{
    FSKP_0, // 0 means Secure Provisioning is Disabled
    FSKP_1,
    FSKP_2,
    FSKP_3,
    FSKP_4,
    FSKP_5,
    FSKP_6,
    FSKP_7,
    FSKP_8,
    FSKP_9,
    FSKP_10,
    FSKP_11,
    FSKP_12,
    FSKP_13,
    FSKP_14,
    FSKP_15,
    FSKP_16,
    FSKP_17,
    FSKP_18,
    FSKP_19,
    FSKP_20,
    FSKP_21,
    FSKP_22,
    FSKP_23,
    FSKP_24,
    FSKP_25,
    FSKP_26,
    FSKP_27,
    FSKP_28,
    FSKP_29,
    FSKP_30,
    FSKP_31,
    FSKP_32,
    FSKP_33,
    FSKP_34,
    FSKP_35,
    FSKP_36,
    FSKP_37,
    FSKP_38,
    FSKP_39,
    FSKP_40,
    FSKP_41,
    FSKP_42,
    FSKP_43,
    FSKP_44,
    FSKP_45,
    FSKP_46,
    FSKP_47,
    FSKP_48,
    FSKP_49,
    FSKP_50,
    FSKP_51,
    FSKP_52,
    FSKP_53,
    FSKP_54,
    FSKP_55,
    FSKP_56,
    FSKP_57,
    FSKP_58,
    FSKP_59,
    FSKP_60,
    FSKP_61,
    FSKP_62,
    FSKP_63,

    FSKP_Num,

    FSKP_DISABLED = FSKP_0,
    FSKP_ANTI_CLONING_KEY_START = FSKP_1,
    FSKP_ANTI_CLONING_KEY_END = FSKP_15,
    FSKP_REGULAR_KEY_START = FSKP_16,
    FSKP_REGULAR_KEY_END = FSKP_63,
    FSKP_KEY_WRAP_KEY = FSKP_62,
    FSKP_NVIDIA_DEBUG_KEY = FSKP_63,

    FSKP_FORCE32 = 0x7FFFFFFF,
} FskpKeyNum;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_CRYPTO_FSKP_H
