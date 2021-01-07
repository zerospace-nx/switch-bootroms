/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_INCLUDE_T214_NVBOOT_DEBUG_H_
#define NVBOOT_INCLUDE_T214_NVBOOT_DEBUG_H_

#include "nvboot_fuse.h"
#include "nvboot_error.h"

#define APBDEV_PMC_DEBUG_AUTHENTICATION_0_NOT_ECID_CHECKED_RANGE 5:4
#define APBDEV_PMC_DEBUG_AUTHENTICATION_0_NOT_ECID_CHECKED_SHIFT _MK_SHIFT_CONST(4)
#define APBDEV_PMC_DEBUG_AUTHENTICATION_0_NOT_ECID_CHECKED_FIELD _MK_FIELD_CONST(0x3, APBDEV_PMC_DEBUG_AUTHENTICATION_0_NOT_ECID_CHECKED_SHIFT)

#define APBDEV_PMC_DEBUG_AUTHENTICATION_0_ECID_CHECKED_RANGE 3:0
#define APBDEV_PMC_DEBUG_AUTHENTICATION_0_ECID_CHECKED_SHIFT _MK_SHIFT_CONST(0)
#define APBDEV_PMC_DEBUG_AUTHENTICATION_0_ECID_CHECKED_FIELD _MK_FIELD_CONST(0xF, APBDEV_PMC_DEBUG_AUTHENTICATION_0_ECID_CHECKED_SHIFT)

#define DEBUG_ALL_DISABLED (APBDEV_PMC_DEBUG_AUTHENTICATION_0_SW_DEFAULT_VAL) // 0 is all disabled
#define DEBUG_ALL_ENABLED (APBDEV_PMC_DEBUG_AUTHENTICATION_0_RESET_VAL) //0x3F is all enabled.

/**
 * @param[in] InputEcid from the BCT header or RCM header.
 * @param[in] Field from the BCT header or RCM header. Not ECID checked.
 * @param[in] Field from the BCT header or RCM header. ECID checked. If mismatched with
 *            the current chip, this value will not be set into the debug authentication register.
 * @retval NvBootError. Returns NvBootError_Success if in preproduction mode or "nv production mode"
 *                      and FSKP is disabled. Returns Success in ODM production mode if the ECID
 *                      check matched. Returns NvBootError_ECID_Mismatch if the Input ECID mismatches
 *                      with the chip ECID. The value in SecureDebugControl_Not_ECID_Checked will always
 *                      be set to the debug authentication register.
 */
NvBootError
NvBootDebugSetDebugFeatures(const NvBootECID *InputEcid,
                            uint32_t SecureDebugControl_Not_ECID_Checked,
                            uint32_t SecureDebugControl_ECID_Checked);


#endif /* NVBOOT_INCLUDE_T214_NVBOOT_DEBUG_H_ */
