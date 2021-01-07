/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_ADDRESS_OFFSETS_SIZES_H
#define INCLUDED_ADDRESS_OFFSETS_SIZES_H


#define OFFSET_SIGNED_SECT(x) \
    ((uint32_t)&(x->RandomAesBlock) - (uint32_t)x)

#define OFFSET_BCT_SIGNED_SECT(x) OFFSET_SIGNED_SECT(x)
#define SIZE_BCT_SIGNED_SECT(x) \
    (sizeof(NvBootConfigTable) - OFFSET_BCT_SIGNED_SECT(x))

#define OFFSET_SC7_SIGNED_SECT(x) OFFSET_SIGNED_SECT(x)
#define SIZE_SC7_SIGNED_SECT(x) \
    (sizeof(NvBootWb0RecoveryHeader) - OFFSET_SC7_SIGNED_SECT(x))

#define OFFSET_RCM_SIGNED_SECT(x) OFFSET_SIGNED_SECT(x)
#define SIZE_RCM_SIGNED_SECT(x) \
    (sizeof(NvBootRcmMsg) - OFFSET_RCM_SIGNED_SECT(x))


#define OFFSET_BCT_ENCRYPTED_SECT(x) \
    ((uint32_t)&(x->RandomAesBlock2) - (uint32_t)x)

#define SIZE_BCT_ENCRYPTED_SECT(x) \
    (sizeof(NvBootConfigTable) - OFFSET_BCT_ENCRYPTED_SECT(x))

#define OFFSET_OEMBOOT_SIGNED_SECT(x) \
    ((uint32_t)&(x->Salt) - (uint32_t)x)

#define SIZE_OEMBOOT_SIGNED_SECT(x) \
    (sizeof(NvBootOemBootBinaryHeader) - OFFSET_OEMBOOT_SIGNED_SECT(x))

#endif //INCLUDED_ADDRESS_OFFSETS_SIZES_H
