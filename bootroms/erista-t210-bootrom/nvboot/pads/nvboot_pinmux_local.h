/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVRM_PINMUX_UTILS_H
#define NVRM_PINMUX_UTILS_H

/*
 * nvrm_pinmux_utils.h defines the pinmux macros to implement for the resource
 * manager.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "nvboot_util_int.h"
#include "arapb_misc.h"

/*  The pin mux code supports run-time trace debugging of all updates to the
 *  pin mux & tristate registers by embedding strings (cast to NvU32s) into the
 *  control tables.
 */
#define NVRM_PINMUX_DEBUG_FLAG 0
#define NVRM_PINMUX_SET_OPCODE_SIZE_RANGE 3:1


#if NVRM_PINMUX_DEBUG_FLAG
NV_CT_ASSERT(sizeof(NvU32)==sizeof(const char*));
#endif

//  The extra strings bloat the size of Set/Unset opcodes
#define NVRM_PINMUX_SET_OPCODE_SIZE ((NVRM_PINMUX_DEBUG_FLAG)?NVRM_PINMUX_SET_OPCODE_SIZE_RANGE)

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

typedef enum {
    PinMuxConfig_OpcodeExtend = 0,
    PinMuxConfig_Set = 1,
    PinMuxConfig_Unset = 2,
    PinMuxConfig_BranchLink = 3,
} PinMuxConfigStates;

typedef enum {
    PinMuxOpcode_ConfigEnd = 0,
    PinMuxOpcode_ModuleDone = 1,
    PinMuxOpcode_SubroutinesDone = 2,
} PinMuxConfigExtendOpcodes;

//
// Note to the RM team: The per-pin pin control registers collect all the
// relevant pin control state into single registers per pin.
// Therefore, a single offset is needed for both pinmux and tristrate controls.
// Further, the bit positions for the pinmux controls and tristates are 
// are identical for all pins.
// Based on these observations:
//    * There is a single offset whose range has been expanded to handle 
//      the larger number of entries needed.
//    * The TS_SHIFT, MUX_CTL_SHIFT, and MUX_CTL_MASK fields have been
//      deprecated.
//
// Presumably, the GPIO port-to-pin mapping goes away, but that
// is outside the purview of the Boot ROM.

// When the state is BranchLink, this is the number of words to increment
// the current "PC"
#define MUX_ENTRY_0_BRANCH_ADDRESS_RANGE 31:2

// The incr1 offset from PINMUX_AUX_SDMMC1_CLK_0 to the pin's control register
#define MUX_ENTRY_0_OFFSET_RANGE 31:12

// This value to set PULLUP or not
#define MUX_ENTRY_0_PULL_CTL_SET_RANGE 11:8

// When a pad group needs to be owned (or disowned), this value is applied
#define MUX_ENTRY_0_MUX_CTL_SET_RANGE 7:5

// This value is compared against, to determine if the pad group should be
// disowned
#define MUX_ENTRY_0_MUX_CTL_UNSET_RANGE 4:2

// for extended opcodes, this field is set with the extended opcode
#define MUX_ENTRY_0_OPCODE_EXTENSION_RANGE 3:2

// The state for this entry
#define MUX_ENTRY_0_STATE_RANGE 1:0

#define MAX_NESTING_DEPTH 4

/*
 * This macro is used to generate 32b value to program the tristate & pad mux
 * control registers for config/unconfig for a padgroup
 */
#define PIN_MUX_ENTRY(PINOFFS,PUPDSET,MUXSET,MUXUNSET,STAT) \
    (NV_DRF_NUM(MUX, ENTRY, OFFSET, PINOFFS) | \
    NV_DRF_NUM(MUX, ENTRY, PULL_CTL_SET, PUPDSET) | \
    NV_DRF_NUM(MUX, ENTRY, MUX_CTL_SET, MUXSET) | \
    NV_DRF_NUM(MUX, ENTRY, MUX_CTL_UNSET,MUXUNSET) | \
    NV_DRF_NUM(MUX, ENTRY, STATE,STAT))

/*
 * This is used to program the tristate & pad mux control registers for a
 * pad group.
 */
#define CONFIG_VAL(REG, MUX, PUPD) \
    (PIN_MUX_ENTRY(((PINMUX_AUX_##REG##_0 - PINMUX_AUX_SDMMC1_CLK_0)>>2), \
                PINMUX_AUX_##REG##_0_PUPD_##PUPD,PINMUX_AUX_##REG##_0_PM_##MUX, \
                0, PinMuxConfig_Set))

/*
 * This macro is used to compare a pad group against a potentially conflicting
 * enum (where the conflict is caused by setting a new config), and to resolve
 * the conflict by setting the conflicting pad group to a different,
 * non-conflicting option.
 * Read this as: if the pin's mux is equal to (CONFLICTMUX), replace it with
 * (RESOLUTIONMUX)
 */
#define UNCONFIG_VAL(REG, CONFLICTMUX, RESOLUTIONMUX) \
    (PIN_MUX_ENTRY(((PINMUX_AUX_##REG##_0 - PINMUX_AUX_SDMMC1_CLK_0)>>2), \
                  PINMUX_AUX_##REG##_0_PM_##RESOLUTIONMUX, \
                  PINMUX_AUX_##REG##_0_PM_##CONFLICTMUX, \
                  PinMuxConfig_Unset))

#if NVRM_PINMUX_DEBUG_FLAG
#define CONFIG(REG, MUX) \
    (CONFIG_VAL(REG, MUX)), \
    (NvU32)(const void*)(#REG "_0_" "_PM to " #MUX),

#define UNCONFIG(REG, CONFLICTMUX, RESOLUTIONMUX) \
    (UNCONFIG_VAL(REG, CONFLICTMUX, RESOLUTIONMUX)), \
    (NvU32)(const void*)(#REG "_0_PM from " #CONFLICTMUX " to " #RESOLUTIONMUX), \
    (NvU32)(const void*)(NULL)
#else
#define CONFIG(REG, MUX, PUPD)     (CONFIG_VAL(REG, MUX,PUPD))

#define UNCONFIG(REG, CONFLICTMUX, RESOLUTIONMUX) \
    (UNCONFIG_VAL(REG, CONFLICTMUX, RESOLUTIONMUX))
#endif

/*  This macro is used for opcode entries in the tables */
#define PIN_MUX_OPCODE(_OP_) \
    (NV_DRF_NUM(MUX,ENTRY,STATE,PinMuxConfig_OpcodeExtend) | \
     NV_DRF_NUM(MUX,ENTRY,OPCODE_EXTENSION,(_OP_)))

/*
 * This is a dummy entry in the array which indicates that all
 * setting/unsetting for a configuration is complete.
 */
#define CONFIGEND() PIN_MUX_OPCODE(PinMuxOpcode_ConfigEnd)

/*
 * This is a dummy entry in the array which indicates that the last 
 * configuration
 * for the module instance has been passed.
 */
#define MODULEDONE()  PIN_MUX_OPCODE(PinMuxOpcode_ModuleDone)

/*
 * This is a dummy entry in the array which indicates that all "extra"
 * configurations used by sub-routines have been passed.
 */
#define SUBROUTINESDONE() PIN_MUX_OPCODE(PinMuxOpcode_SubroutinesDone)

/*
 * This macro is used to insert a branch-and-link from one configuration
 * to another
 */
#define BRANCH(_ADDR_) \
     (NV_DRF_NUM(MUX,ENTRY,STATE,PinMuxConfig_BranchLink) | \
      NV_DRF_NUM(MUX,ENTRY,BRANCH_ADDRESS,(_ADDR_)))

/*
 * The below entries define the table format for GPIO Port/Pin-to-Tristate
 * register mappings. Each table entry is 16b, and one is stored for every 
 * GPIO Port/Pin on the chip.
 */
#define MUX_GPIOMAP_0_OFFSET_RANGE 15:10

#define TRISTATE_ENTRY(PINOFFS) \
    ((NvU16)(NV_DRF_NUM(MUX,GPIOMAP,OFFSET,(PINOFFS))))             

#define GPIO_TRISTATE(REG) \
    (TRISTATE_ENTRY(((PINMUX_AUX_##REG##_0 - PINMUX_AUX_SDMMC1_CLK_0)>>2)))



void
NvBootPadSetTriStates(
    const NvU32* Module,
    NvU32 Config,
    NvBool EnableTristate);

void
NvBootPadSetPinMuxCtl(
    const NvU32* Module,
    NvU32 Config,
    NvBool DoPinMux,
    NvBool DoPullUpPullDown);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif // NVRM_PINMUX_UTILS_H
