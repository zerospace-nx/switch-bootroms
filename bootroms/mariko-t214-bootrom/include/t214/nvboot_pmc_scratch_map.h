/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * Defines fields in the PMC scratch registers used by the Boot ROM code.
 */

#ifndef INCLUDED_NVBOOT_PMC_SCRATCH_MAP_H
#define INCLUDED_NVBOOT_PMC_SCRATCH_MAP_H

/**
 * The PMC module in the Always On domain of the chip provides 43
 * scratch registers and 6 secure scratch registers. These offer SW a
 * storage space that preserves values across LP0 transitions.
 *
 * These are used by to store state information for on-chip controllers which
 * are to be restored during Warm Boot, whether WB0 or WB1.
 *
 * Scratch registers offsets are part of PMC HW specification - "arapbpm.spec".
 *
 * This header file defines the allocation of scratch register space for
 * storing data needed by Warm Boot.
 *
 * Each of the scratch registers has been sliced into bit fields to store
 * parameters for various on-chip controllers. Every bit field that needs to
 * be stored has a matching bit field in a scratch register. The width matches
 * the original bit fields.
 *
 * Scratch register fields have been defined with self explanatory names
 * and with bit ranges compatible with nvrm_drf macros.
 *
 * Ownership Issues: Important!!!!
 *
 * Register APBDEV_PMC_SCRATCH0_0 is the *only* scratch register cleared on
 * power-on-reset. This register will also be used by RM and OAL. This holds
 * several important flags for the Boot ROM:
 *     WARM_BOOT0_FLAG: Tells the Boot ROM to perform WB0 upon reboot instead
 *                    of a cold boot.
 *     FORCE_RECOVERY_FLAG:
 *         Forces the Boot ROM to enter RCM instead of performing a cold
 *         boot or WB0.
 *     BL_FAIL_BACK_FLAG:
 *         One of several indicators that the the Boot ROM should fail back
 *         older generations of BLs if the newer generations fail to load.
 *     FUSE_ALIAS_FLAG:
 *         Indicates that the Boot ROM should alias the fuses with values
 *         stored in PMC SCRATCH registers and restart.
 *     STRAP_ALIAS_FLAG:
 *         Same as FUSE_ALIAS_FLAG, but for strap aliasing.
 *
 * The assignment of bit fields used by BootROM is *NOT* to be changed.
 */

/**
 * FUSE_ALIAS:
 *   Desc: Enables fuse aliasing code when set to 1.  In Pre-Production mode,
 *     the Boot ROM will alias the fuses and re-start the boot process using
 *     the new fuse values. Note that the Boot ROM clears this flag when it
 *     performs the aliasing to avoid falling into an infinite loop.
 *     Unlike AP15/AP16, the alias values are pulled from PMC scratch registers
 *     to survive an LP0 transition.
 * STRAP_ALIAS:
 *   Desc: Enables strap aliasing code when set to 1.  In Pre-Production mode,
 *     the Boot ROM will alias the straps and re-start the boot process using
 *     the new strap values. Note that the Boot ROM does not clear this flag
 *     when it aliases the straps, as this cannot lead to an infinite loop.
 *     Unlike AP15/AP16, the alias values are pulled from PMC scratch registers
 *     to survive an LP0 transition.
 */
#define APBDEV_PMC_SCRATCH0_0_WARM_BOOT0_FLAG_RANGE                        0: 0
#define APBDEV_PMC_SCRATCH0_0_FORCE_RECOVERY_FLAG_RANGE                    1: 1
#define APBDEV_PMC_SCRATCH0_0_BL_FAIL_BACK_FLAG_RANGE                      2: 2
#define APBDEV_PMC_SCRATCH0_0_BR_HALT_AT_WB0_FLAG_RANGE                    4: 4
// Set BR_HALT_AFTER_WB0_FLAG to stop BR execution after any SC7 error.
// This bit is only effective in preproduction mode.
#define APBDEV_PMC_SCRATCH0_0_BR_HALT_AFTER_WB0_FLAG_RANGE                 5: 5
// Bits 31:5 reserved for SW

// Likely needed for SW in same place
#define APBDEV_PMC_SECURE_SCRATCH119_0_PTR_TO_RECOVERY_CODE_RANGE          31: 0

// TODO: Update LP0 exit PLLM registers for T35
#define PLLM_DIVM_LOW_BIT   0
#define PLLM_DIVM_HIGH_BIT (PLLM_DIVM_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVM_RANGE)-1)
#define PLLM_DIVN_LOW_BIT (PLLM_DIVM_HIGH_BIT+1)
#define PLLM_DIVN_HIGH_BIT (PLLM_DIVN_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVN_RANGE)-1)
#define PLLM_DIVP_LOW_BIT (PLLM_DIVN_HIGH_BIT+1)
#define PLLM_DIVP_HIGH_BIT (PLLM_DIVP_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVP_RANGE)-1)
#define PLLM_KVCO_LOW_BIT (PLLM_DIVP_HIGH_BIT+1)
#define PLLM_KVCO_HIGH_BIT (PLLM_KVCO_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KVCO_RANGE)-1)
#define PLLM_KCP_LOW_BIT (PLLM_KVCO_HIGH_BIT+1)
#define PLLM_KCP_HIGH_BIT (PLLM_KCP_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KCP_RANGE)-1)

#define APBDEV_PMC_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVM_RANGE	PLLM_DIVM_HIGH_BIT: PLLM_DIVM_LOW_BIT
#define APBDEV_PMC_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVN_RANGE	PLLM_DIVN_HIGH_BIT: PLLM_DIVN_LOW_BIT
#define APBDEV_PMC_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVP_RANGE	PLLM_DIVP_HIGH_BIT: PLLM_DIVP_LOW_BIT
#define APBDEV_PMC_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KVCO_RANGE	PLLM_KVCO_HIGH_BIT: PLLM_KVCO_LOW_BIT
#define APBDEV_PMC_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KCP_RANGE	PLLM_KCP_HIGH_BIT: PLLM_KCP_LOW_BIT
// Bits 31:20 available

// Note: APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_CHOICE_RANGE identifies the choice
//       of PLL to start w/PLLX parameters: X or C.
// TODO: Update LP0 exit PLLX registers for T35
#define PLLX_DIVM_LOW_BIT   0
#define PLLX_DIVM_HIGH_BIT (PLLX_DIVM_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVM_RANGE)-1)
#define PLLX_DIVN_LOW_BIT (PLLX_DIVM_HIGH_BIT+1)
#define PLLX_DIVN_HIGH_BIT (PLLX_DIVN_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVN_RANGE)-1)
#define PLLX_DIVP_LOW_BIT (PLLX_DIVN_HIGH_BIT+1)
#define PLLX_DIVP_HIGH_BIT (PLLX_DIVP_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVP_RANGE)-1)
#define PLLX_KVCO_LOW_BIT (PLLX_DIVP_HIGH_BIT+1)
#define PLLX_KVCO_HIGH_BIT (PLLX_KVCO_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KVCO_RANGE)-1)
#define PLLX_KCP_LOW_BIT (PLLX_KVCO_HIGH_BIT+1)
#define PLLX_KCP_HIGH_BIT (PLLX_KCP_LOW_BIT + \
                NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KCP_RANGE)-1)
#define PLLX_ENABLE_LOW_BIT (PLLX_KCP_HIGH_BIT+1)
#define PLLX_ENABLE_SIZE 1
#define PLLX_ENABLE_HIGH_BIT (PLLX_ENABLE_LOW_BIT+PLLX_ENABLE_SIZE-1)

#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVM_RANGE	PLLX_DIVM_HIGH_BIT: PLLX_DIVM_LOW_BIT
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVN_RANGE	PLLX_DIVN_HIGH_BIT: PLLX_DIVN_LOW_BIT
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVP_RANGE	PLLX_DIVP_HIGH_BIT: PLLX_DIVP_LOW_BIT
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KVCO_RANGE	PLLX_KVCO_HIGH_BIT: PLLX_KVCO_LOW_BIT
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KCP_RANGE	PLLX_KCP_HIGH_BIT: PLLX_KCP_LOW_BIT
// Note: APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_ENABLE_RANGE need to
//          be set explicitly by BL/OS code in order to enable PLLX  in LP0 by
//          BR, by default PLLX is turned OFF.
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_ENABLE_RANGE              PLLX_ENABLE_HIGH_BIT: PLLX_ENABLE_LOW_BIT
// Bits 25:24 available
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_CHOICE_RANGE                         26:26
// Bits 31:27 available

#define APBDEV_PMC_SCRATCH4_0_PLLM_STABLE_TIME_RANGE                       9: 0
#define APBDEV_PMC_SCRATCH4_0_PLLX_STABLE_TIME_RANGE                      19:10
// Bits 31:20 available

// PLLM extra params
// Bits 31:24 available
#define PLLM_SETUP_LOW_BIT 0
#define PLLM_SETUP_HIGH_BIT (PLLM_SETUP_LOW_BIT + \
                            NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_SETUP_RANGE)-1)
#define APBDEV_PMC_SCRATCH35_0_CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_SETUP_RANGE PLLM_SETUP_HIGH_BIT: PLLM_SETUP_LOW_BIT

// PLLX extra params
#define PLLX_SETUP_LOW_BIT 0
#define PLLX_SETUP_HIGH_BIT (PLLX_SETUP_LOW_BIT + \
                            NV_FIELD_SIZE(CLK_RST_CONTROLLER_PLLX_MISC_1_0_PLLX_SETUP_RANGE)-1)

#define APBDEV_PMC_SCRATCH36_0_CLK_RST_CONTROLLER_PLLX_MISC_1_0_PLLX_SETUP_RANGE    PLLX_SETUP_HIGH_BIT: PLLX_SETUP_LOW_BIT
// Bits 31:24 available

// Storage for the location of the encrypted SE context
#define APBDEV_PMC_SCRATCH43_0_SE_ENCRYPTED_CONTEXT_RANGE                 31: 0

// Storage location for bits that Boot ROM needs to restore at LP0.
#define APBDEV_PMC_SCRATCH49_0_AHB_SPARE_REG_0_OBS_OVERRIDE_EN_RANGE        0:0
#define APBDEV_PMC_SCRATCH49_0_AHB_SPARE_REG_0_APB2JTAG_OVERRIDE_EN_RANGE   1:1

// Storage for the SRK
/**
 * The SE will save the SRK key to SCRATCH4-7 when a CTX_SAVE operation with
 * destination SRK is started.
 */
#define APBDEV_PMC_SECURE_SCRATCH4_0_SRK_0_SRK0_RANGE                     31: 0
#define APBDEV_PMC_SECURE_SCRATCH5_0_SRK_0_SRK1_RANGE                     31: 0
#define APBDEV_PMC_SECURE_SCRATCH6_0_SRK_0_SRK2_RANGE                     31: 0
#define APBDEV_PMC_SECURE_SCRATCH7_0_SRK_0_SRK3_RANGE                     31: 0

// Keep MC code drop at last. Use AS-IS
#include "nvboot_wb0_sdram_scratch_list.h"

#endif // INCLUDED_NVBOOT_PMC_SCRATCH_MAP_H
