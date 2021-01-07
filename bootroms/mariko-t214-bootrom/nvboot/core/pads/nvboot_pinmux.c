/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_pinmux_local.h"
#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include "project.h"

/*
 *  Each of the pin mux configurations defined in the pin mux spreadsheet are
 *  stored in chip-specific tables.  For each configuration, every pin
 *  that must be programmed is stored as a single 32b entry, where the register
 *  offset for the pin controls and new pin mux state are programmed.
 *
 *  The tables are microcode for a simple state machine.  The state machine
 *  supports subroutine call/return (up to 4 levels of nesting), so that
 *  pin mux configurations which have substantial repetition can be
 *  represented compactly by separating common portion of the configurations
 *  into a subroutine.  Additionally, the state machine supports
 *  "unprogramming" of the pin mux registers, so that pins which are
 *  incorrectly programmed to mux from a controller may be safely disowned,
 *  ensuring that no conflicts exist where multiple pins are muxing
 *  the same set of signals.
 *
 *  Each module instance array has a reserved "reset" configuration at index
 *  zero.  This special configuration is used in order to disown all pins
 *  whose reset state refers to the module instance.  When a module
 *  instance configuration is to be applied, the reset configuration will
 *  first be applied, to ensure that no conflicts will arise between register
 *  reset values and the new configuration, followed by the application of
 *  the requested configuration.
 *
 *  Furthermore, for controllers which support dynamic pinmuxing (i.e.,
 *  the "Multiplexed" pin map option), the last table entry is reserved for
 *  a "global unset," which will ensure that all configurations are disowned.
 *  This Multiplexed configuration should be applied before transitioning
 *  from one configuration to a second one.
 *
 *  The table data has been packed into a single 32b entry to minimize code
 *  footprint using macros similar to the hardware register definitions, so
 *  that all of the shift and mask operations can be performed with the DRF
 *  macros.
 *
 *  Note that all pins have the same representation.
 *  The code uses bitfield definitions for PINMUX_AUX_SDMMC1_CLK_0.
 */


/*
 *  FindConfigStart searches through an array of configuration data to find the
 *  starting position of a particular configuration in a module instance array.
 *  The stop position is programmable, so that sub-routines can be placed after
 *  the last valid true configuration
 */

static const NvU32*
NvBootPadFindConfigStart(const NvU32* Instance,
    NvU32 Config,
    NvU32 EndMarker)
{
    NvU32 Cnt = 0;
    while ((Cnt < Config) && (*Instance!=EndMarker))
    {
        switch (NV_DRF_VAL(MUX, ENTRY, STATE, *Instance))
        {
            case PinMuxConfig_BranchLink:
            case PinMuxConfig_OpcodeExtend:
                if (*Instance==CONFIGEND())
                    Cnt++;
                Instance++;
                break;
            default:
                Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
                break;
        }
    }

    //  Ugly postfix.  In modules with bonafide subroutines, the last
    //  configuration CONFIGEND() will be followed by a MODULEDONE()
    //  token, with the first Set/Unset/Branch of the subroutine 
    //  following that.  To avoid leaving the "PC" pointing to a
    //  MODULEDONE() in the case where the first subroutine should be
    //  executed, fudge the "PC" up by one, to point to the subroutine.
    if (EndMarker==SUBROUTINESDONE() && *Instance==MODULEDONE())
        Instance++;

    if (*Instance==EndMarker)
        Instance = NULL;

    return Instance;
}


void
NvBootPadSetTriStates(
    const NvU32* Module,
    NvU32 Config,
    NvBool EnableTristate)
{
    int          StackDepth = 0;
    const NvU32 *Instance = NULL;
    const NvU32 *ReturnStack[MAX_NESTING_DEPTH+1];

    Instance = NvBootPadFindConfigStart(Module, Config, MODULEDONE());
    /*
     * The first stack return entry is NULL, so that when a ConfigEnd is
     * encountered in the "main" configuration program, we pop off a NULL
     * pointer, which causes the configuration loop to terminate.
     */
    ReturnStack[0] = NULL;

    /*
     * This loop iterates over all of the pad groups that need to be updated,
     * and updates the reference count for each appropriately.
     */
    while (Instance)
    {
        switch (NV_DRF_VAL(MUX,ENTRY, STATE, *Instance))
        {
            case PinMuxConfig_OpcodeExtend:
              /* 
               * Pop the most recent return address off of the return stack
               * (which will be NULL if no values have been pushed onto the
               * stack)
               */
              if (NV_DRF_VAL(MUX,ENTRY, OPCODE_EXTENSION, *Instance) ==
                  PinMuxOpcode_ConfigEnd)
              {
                  Instance = ReturnStack[StackDepth--];
              }
              /*
               * ModuleDone & SubroutinesDone should never be encountered
               * during execution, for properly-formatted tables.
               */
              else
              {
                  NV_ASSERT(0); // Logical entry in table
              }
              break;

            case PinMuxConfig_BranchLink:
                /*
                 * Push the next instruction onto the return stack if nesting
                 * space is available, and jump to the target.
                 */
                NV_ASSERT(StackDepth<MAX_NESTING_DEPTH);

                ReturnStack[++StackDepth] = Instance+1;

                Instance = NvBootPadFindConfigStart(
                    Module,
                    NV_DRF_VAL(MUX,ENTRY,BRANCH_ADDRESS,*Instance),
                    SUBROUTINESDONE());

                NV_ASSERT(Instance); // Invalid branch configuration in table
                break;

            case PinMuxConfig_Set:
                {
                    NvU32 Offs = NV_DRF_VAL(MUX, ENTRY, OFFSET, *Instance);
        
                    // Perform the udpate
                    NvU32 Curr = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + 
                                           PINMUX_AUX_SDMMC1_CLK_0 +
                                           4*Offs);

                    Curr = NV_FLD_SET_DRF_NUM(PINMUX_AUX,
                                              SDMMC1_CLK,
                                              TRISTATE,
                                              EnableTristate?1:0,
                                              Curr);

#if 0 //FIXME
                    // Clearing the IO_RESET
                    if((NV_DRF_VAL(MUX, ENTRY, PULL_CTL_SET, *Instance)) || 
                        (NV_DRF_VAL( PINMUX_AUX, SDMMC4_CLK, IO_RESET, Curr)))
                    {
                        if(NV_DRF_VAL( PINMUX_AUX, SDMMC4_CLK, IO_RESET, Curr))
                             Curr = NV_FLD_SET_DRF_DEF( PINMUX_AUX, SDMMC4_CLK,
                                        IO_RESET,NORMAL, Curr);
                    }
#endif

                    NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + 
                               PINMUX_AUX_SDMMC1_CLK_0 + 4*Offs,
                               Curr);

                }

                //  fall through.

                /*
                 * The "Unset" configurations are not applicable to tristate
                 * configuration, so skip over them.
                 */
            case PinMuxConfig_Unset:
                Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
                break;       
        }
    }
}

/*  NvRmSetPinMuxCtl will apply new pin mux configurations to the pin mux
 *  control registers.  */
void
NvBootPadSetPinMuxCtl(
    const NvU32* Module,
    NvU32 Config,
    NvBool DoPinMux,
    NvBool DoPullUpPullDown)
{
    NvU32 Curr;
    NvU32 MuxCtlSet;
    NvU32 PullCtlSet;
    NvU32 MuxCtlUnset;
    NvU32 Offset;
    NvU32 State;
    const NvU32 *ReturnStack[MAX_NESTING_DEPTH+1];
    const NvU32 *Instance;
    int StackDepth = 0;

    ReturnStack[0] = NULL;

    Instance = NvBootPadFindConfigStart(Module, Config, MODULEDONE());

    // Apply the new configuration, setting / unsetting as appropriate
    while (Instance)
    {
        State = NV_DRF_VAL(MUX,ENTRY, STATE, *Instance);
        switch (State)
        {
            case PinMuxConfig_OpcodeExtend:
                if (NV_DRF_VAL(MUX,ENTRY, OPCODE_EXTENSION, *Instance) ==
                    PinMuxOpcode_ConfigEnd)
                {
                    Instance = ReturnStack[StackDepth--];
                }
                else
                {
                    NV_ASSERT(0); //Logical entry in table
                }
                break;
            case PinMuxConfig_BranchLink:
                NV_ASSERT(StackDepth<MAX_NESTING_DEPTH);
                ReturnStack[++StackDepth] = Instance+1;
                Instance = NvBootPadFindConfigStart(Module,
                                                    NV_DRF_VAL(MUX,
                                                               ENTRY,
                                                               BRANCH_ADDRESS,
                                                               *Instance),
                                                    SUBROUTINESDONE());
                NV_ASSERT(Instance); // Invalid branch configuration in table
                break;

            default:
            {
                MuxCtlUnset = NV_DRF_VAL(MUX, ENTRY, MUX_CTL_UNSET, *Instance);
                MuxCtlSet   = NV_DRF_VAL(MUX, ENTRY, MUX_CTL_SET,   *Instance);
                PullCtlSet   = NV_DRF_VAL(MUX, ENTRY, PULL_CTL_SET,   *Instance);
                Offset      = NV_DRF_VAL(MUX, ENTRY, OFFSET, *Instance);

         
                // Perform the udpate
                Curr = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + 
                                 PINMUX_AUX_SDMMC1_CLK_0 +
                                 4*Offset);

                if (State == PinMuxConfig_Set)
                {
                    if(DoPinMux)
                    {

                        Curr = NV_FLD_SET_DRF_NUM(PINMUX_AUX,
                                                  SDMMC1_CLK,
                                                  PM,
                                                  MuxCtlSet,
                                                  Curr);

			// ENABLE E_OD
			// E_IO_HV field in PINMUX_AUX_USB_VBUS_ENX_0 register to ENABLE, 
			// E_OD field in PINMUX_AUX_USB_VBUS_ENX_0 register to ENABLE, 
			// E_INPUT field in PINMUX_AUX_USB_VBUS_ENX_0 register to ENABLE, and 
			// PDPU field in PINMUX_AUX_USB_VBUS_ENX_0 register to NONE
			if((NV_DRF_VAL(MUX, ENTRY, PULL_CTL_SET, *Instance)) || 
			    (NV_DRF_VAL( PINMUX_AUX, USB_VBUS_EN0, E_INPUT, Curr)))
			{
			    if(NV_DRF_VAL( PINMUX_AUX, USB_VBUS_EN0, E_INPUT, Curr))
			         Curr = NV_FLD_SET_DRF_DEF( PINMUX_AUX, USB_VBUS_EN0,
			                    E_OD,DISABLE, Curr);
    				Curr = NV_FLD_SET_DRF_DEF( PINMUX_AUX, USB_VBUS_EN0,
    						   E_IO_HV,ENABLE, Curr);
    				Curr = NV_FLD_SET_DRF_DEF( PINMUX_AUX, USB_VBUS_EN0,
    						   E_INPUT,ENABLE, Curr);
    				Curr = NV_FLD_SET_DRF_DEF( PINMUX_AUX, USB_VBUS_EN0,
    						   PARK,NORMAL, Curr);
    				Curr = NV_FLD_SET_DRF_DEF( PINMUX_AUX, USB_VBUS_EN0,
    						   E_LPDR,DISABLE, Curr);
			}
                    }

                    if(DoPullUpPullDown) 
                    {
			Curr = NV_FLD_SET_DRF_NUM(PINMUX_AUX,
						  SDMMC1_CLK,
						  PUPD,
						  PullCtlSet,
						  Curr);
                    }
                }
                else if (NV_DRF_VAL(PINMUX_AUX, SDMMC1_CLK, PM, Curr) ==
                         MuxCtlUnset)
                {

                    NV_ASSERT(State == PinMuxConfig_Unset);
                    if(DoPinMux)
                    {
                        Curr = NV_FLD_SET_DRF_NUM(PINMUX_AUX,
                                                  SDMMC1_CLK,
                                                  PM,
                                                  MuxCtlSet,
                                                  Curr);
                    }
                }

                NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + 
                           PINMUX_AUX_SDMMC1_CLK_0 + 4*Offset,
                           Curr);

            }

            Instance += NVRM_PINMUX_SET_OPCODE_SIZE;
            break;
        }
    }
}
