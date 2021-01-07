/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_clocks.c - Implementation of Clocks support.
 *
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "arclk_rst.h"
#include "artimerus.h"
#include "nvboot_clocks_int.h"
#include "nvboot_error.h"
#include "nvboot_hacks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_util_int.h"
#include "project.h"


// set of initialization values for usec counter */
// there are no define for the values themselves, only comment in the spec file
// the order must be the order of the NvBootClocksOscFreq enum
//
//    osc clock freq.   dividend/divisor        USEC_DIVIDEND/USEC_DIVISOR
//    --------------------------------------------------------------------
//        13MHz                1/13                     0x00 / 0x0c
//        19.2MHz             5/96                     0x04 / 0x5f
//        12MHz                1/12                     0x00 / 0x0b
//        26MHz                1/26                     0x00 / 0x19
//        16.8MHz             5/84                     0x04 / 0x53
//        38.4MHz             5/192                   0x04 / 0xbf
//        48MHz                1/48                     0x00 / 0x2f


// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
static const NvU32 s_UsecCfgTable[(int) NvBootClocksOscFreq_MaxVal] =
{
    // 13MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (1-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (13-1)),

    // 16.8MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (5-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (84-1)),

    // dummy field
    0,

    // dummy field
    0,

    // 19.2MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (5-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (96-1)),

    // 38.4MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (5-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (192-1)),

    // dummy field
    0,

    // dummy field
    0,

    // 12MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (1-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (12-1)),

    // 48MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (1-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (48-1)),

    // dummy field
    0,

    // dummy field
    0,

    // 26MHz
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVIDEND, (1-1)) |
    NV_DRF_NUM(TIMERUS_USEC, CFG, USEC_DIVISOR, (26-1))
};

// Put PLL in bypass to insure an active clock (CYA)
void
NvBootClocksBypassPll(NvBootClocksPllId PllId)
{
    NvU32 RegData;

    NVBOOT_CLOCKS_CHECK_PLLID(PllId);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        NVBOOT_CLOCKS_PLL_BASE(PllId));

    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 PLLP_BASE,
                                 PLLP_BYPASS,
                                 ENABLE,
                                 RegData);

    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId),
               RegData);
}

// Check for Pll stability, i.e. is current time after expected stable time
// we cheat by treating all PLLs in the same fashion This work because 
// PLL LOCK field offset is same for all PLLs

NvBool
NvBootClocksIsPllStable(NvBootClocksPllId PllId, NvU32 StableTime)
{
    NvU32 DeltaTime;
    NvU32 RegData;
    NvBool Lock;

    DeltaTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE + TIMERUS_CNTR_1US_0) - 
        StableTime;

    /*
     * WAR: For all PLLs used by BootROM in the past chips as well as in T210,
     * the PLL*_LOCK bit was bit 27:27 of the PLL*_BASE register.
     * However, this is not true for PLLE and PLLREFE. PLLREFE lock bit is
     * PLLREFE_MISC_0[27:27] and PLLE lock bit is PLLE_MISC[11:11]. 
     */
    switch(PllId) {
        case NvBootClocksPllId_PllE:
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                NVBOOT_CLOCKS_PLL_MISC(PllId));
            /// Note: PLLE_LOCK(lock bit for phase as well as frequency) 
            /// is PLLE_MISC_0[11:11].
            Lock = NV_DRF_VAL(CLK_RST_CONTROLLER,
                             PLLE_MISC,
                             PLLE_LOCK,
                             RegData);
            break;
        case NvBootClocksPllId_PllREFE:
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                NVBOOT_CLOCKS_PLL_MISC(PllId));
            /// Note: PLLREFE_LOCK(lock bit for phase as well as frequency) 
            /// is PLLREFE_MISC_0[27:27].
            Lock = NV_DRF_VAL(CLK_RST_CONTROLLER,
                             PLLREFE_MISC,
                             PLLREFE_LOCK,
                             RegData);
            break;
        default:
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                NVBOOT_CLOCKS_PLL_BASE(PllId));
            /// Note: PLLC_FREQ_LOCK is PLLC_BASE_0[27:27](same as PLLC_LOCK) and
            /// therefore can be used as base for checking LOCK for other PLLs.
            Lock = NV_DRF_VAL(CLK_RST_CONTROLLER,
                             PLLC_BASE,
                             PLLC_FREQ_LOCK,
                             RegData);
            break;
    }
#ifdef TARGET_ASIM
    Lock = 1;
#endif
    if (Lock ||!(DeltaTime & (1U << 31)))
    {
        // some delay required aftet the lock bit indicates the PLL is lock 
        if(Lock)
            NvBootUtilWaitUS(NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY_AFTER_LOCK);

        // Disable OUTx divider reset (Bug 954159)
        // 0 - Reset Enable, 1 - Reset Disable
        NvBootClocksPllDivRstCtrl(PllId, 1);

        return NV_TRUE;
    }
    // sign bit set, so DeltaTime is negative, not ready yet  (or)
    // lock bit is not set, not ready yet 
    return NV_FALSE;
}

// Stop PLL, will shut off the output if PLL was not in bypass before that
void
NvBootClocksStopPll(NvBootClocksPllId PllId) {
    
    NvU32 RegData;

    NVBOOT_CLOCKS_CHECK_PLLID(PllId);

    // again relying on common format
    RegData = NV_DRF_DEF(CLK_RST_CONTROLLER, PLLC_BASE, PLLC_ENABLE, DISABLE);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId),
               RegData);
}

// Configure the clock source and divider
void
NvBootClocksConfigureClock(NvBootClocksClockId ClockId, 
                           NvU32 Divider,
                           NvU32 Source ) {

    NvU32 RegOffset;
    NvU32 RegData;

    NVBOOT_CLOCKS_CHECK_CLOCKID(ClockId);

    // Process source first, starting with the standard one
    RegOffset = NVBOOT_CLOCKS_SOURCE_OFFSET(ClockId);
    if(RegOffset != 0)
    {
        // we do abuse a little bit, always using the 16 bit format for the
        // divider, extra bits dropped by HW when needed, this is why we use
        // I2C1 as a template
        RegData = NV_DRF_NUM(CLK_RST_CONTROLLER,
                             CLK_SOURCE_I2C1,
                             I2C1_CLK_SRC,
                             Source) |
                  NV_DRF_NUM(CLK_RST_CONTROLLER,
                             CLK_SOURCE_I2C1,
                             I2C1_CLK_DIVISOR,
                             Divider);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + RegOffset, RegData);
    } 
    else
    { 
        // we need to work more
        switch(ClockId)
        {
        case NvBootClocksClockId_SclkId:  
            RegData = NV_DRF_DEF(CLK_RST_CONTROLLER,
                                 SCLK_BURST_POLICY,
                                 SYS_STATE,
                                 RUN) |
                      NV_DRF_NUM(CLK_RST_CONTROLLER,
                                 SCLK_BURST_POLICY,
                                 SWAKEUP_RUN_SOURCE,
                                 Source);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                       CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0,
                       RegData);

            // we reprogram PLLP_OUT4, better be that the source, otherwise
            // no effect, RMW here 
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                CLK_RST_CONTROLLER_PLLP_OUTB_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                         PLLP_OUTB,
                                         PLLP_OUT4_RATIO,
                                         Divider,
                                         RegData);
            RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                         PLLP_OUTB,
                                         PLLP_OUT4_OVRRIDE,
                                         ENABLE,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                       CLK_RST_CONTROLLER_PLLP_OUTB_0,
                       RegData);
            break;
            
        case NvBootClocksClockId_CclkId :
            RegData = NV_DRF_DEF(CLK_RST_CONTROLLER,
                                 CCLK_BURST_POLICY,
                                 CPU_STATE,
                                 RUN) |
                      NV_DRF_NUM(CLK_RST_CONTROLLER, 
                                 CCLK_BURST_POLICY,
                                 CWAKEUP_RUN_SOURCE,
                                 Source);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                       CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0,
                       RegData);

            RegData = NV_DRF_DEF(CLK_RST_CONTROLLER,
                                 SUPER_CCLK_DIVIDER,
                                 SUPER_CDIV_ENB,
                                 ENABLE) |
                      NV_DRF_NUM(CLK_RST_CONTROLLER,
                                 SUPER_CCLK_DIVIDER,
                                 SUPER_CDIV_DIVIDEND,
                                 (1- 1)) | 
                      NV_DRF_NUM(CLK_RST_CONTROLLER,
                                 SUPER_CCLK_DIVIDER,
                                 SUPER_CDIV_DIVISOR,
                                 Divider);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + 
                       CLK_RST_CONTROLLER_SUPER_CCLK_DIVIDER_0,
                       RegData);
            break;
            
        case NvBootClocksClockId_HclkId : /* cannot change the source */
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                CLK_RST_CONTROLLER_CLK_SYSTEM_RATE_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                         CLK_SYSTEM_RATE,
                                         AHB_RATE,
                                         Divider,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                       CLK_RST_CONTROLLER_CLK_SYSTEM_RATE_0,
                       RegData);
            break;

        case NvBootClocksClockId_PclkId : /* cannot change the source */
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                CLK_RST_CONTROLLER_CLK_SYSTEM_RATE_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                         CLK_SYSTEM_RATE,
                                         APB_RATE,
                                         Divider,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                       CLK_RST_CONTROLLER_CLK_SYSTEM_RATE_0,
                       RegData);
            break;

        case NvBootClocksClockId_EmcId :
            // special mux type and two clock enable present in the source
            // for 1x and 2x
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                                CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                         CLK_SOURCE_EMC,
                                         EMC_2X_CLK_SRC,
                                         Source,
                                         RegData);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                         CLK_SOURCE_EMC,
                                         EMC_2X_CLK_DIVISOR,
                                         Divider,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                       CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0,
                       RegData); 
            break; 

        default :
            // nothing for other enums here, make that clear to the compiler
            break;
        };
    };
    NvBootUtilWaitUS(NVBOOT_CLOCKS_CLOCK_STABILIZATION_TIME);
}

// Set the AVP clock to 204MHz, called before scatter loading so
// must not rely on any non const static, stack variables OK, static const
// variables OK.  Assumes PLLP is stable.
void
NvBootClocksSetAvpClockBeforeScatterLoad()
{
    NvU32 RegData, RegRstSts, SclkDivider;

    // switch system clock to PLLP_out2 (204 MHz) MHz, AVP will now run
    // at 204 MHz.  This is glitch free as only the source is changed,
    // no special precaution needed
    RegData = NV_DRF_DEF(CLK_RST_CONTROLLER,
                         SCLK_BURST_POLICY,
                         SYS_STATE,
                         RUN) |

              NV_DRF_DEF(CLK_RST_CONTROLLER,
                         SCLK_BURST_POLICY, 
                         SWAKEUP_RUN_SOURCE,
                         PLLP_OUT2);

    // Disable AVP fast clock per Boot GFD
    RegRstSts = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);

    if ( (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegRstSts) == APBDEV_PMC_RST_STATUS_0_RST_SOURCE_WATCHDOG) || \
         (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegRstSts) == APBDEV_PMC_RST_STATUS_0_RST_SOURCE_SENSOR) || \
         (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegRstSts) == APBDEV_PMC_RST_STATUS_0_RST_SOURCE_AOTAG) || \
         (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegRstSts) == APBDEV_PMC_RST_STATUS_0_RST_SOURCE_LP0) || \
         (NvBootGetSwCYA() & NVBOOT_SW_CYA_AVP_FAST_CLOCK_DISABLE) )
    {
        // SCLK = PLLP_OUT2/2 = 102 Mhz
        // All other bits of this register are undefined. Simply write the divider.
        SclkDivider = NV_DRF_NUM(CLK_RST_CONTROLLER,
                         CLK_SOURCE_SYS,
                         SYS_CLK_DIVISOR,
                         2);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_SYS_0,
               SclkDivider);
        NvBootUtilWaitUS(2);

    }
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0,
               RegData);
}

void
NvBootClocksPllDivRstCtrl(NvBootClocksPllId  PllId,
        NvU32 EnDis)
{
    NvU32 RegData;

    if (NvBootGetSwCYA() & NVBOOT_SW_CYA_DIVRST_DISABLE)
        return;

    switch (PllId) {

        case NvBootClocksPllId_PllP:
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_OUTA_0);

            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_OUTA, PLLP_OUT1_RSTN,
                    EnDis, RegData);
            /// PLLP_OUT2 Branch is enabled by default.
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_OUTA_0, RegData);

            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_OUTB_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_OUTB, PLLP_OUT3_RSTN,
                    EnDis, RegData);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_OUTB, PLLP_OUT4_RSTN,
                    EnDis, RegData);

            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_OUTB_0, RegData);

            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_OUTC_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLP_OUTC, PLLP_OUT5_RSTN,
                    EnDis, RegData);

            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_OUTC_0, RegData);
            break;

        case NvBootClocksPllId_PllM:
            /// PLLM has no external dividers or branches. 
            // <placeholder if they come back>
            break;

        case NvBootClocksPllId_PllREFE:
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_OUT_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_OUT, PLLREFE_OUT1_RSTN,
                     EnDis, RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_OUT_0, RegData);
            break;
    }
}

// Start PLLP and configure microsecond timer, called before scatter loading so
// must not rely on any non const static, stack variables OK, static const
// variables OK
void
NvBootClocksStartPllpBeforeScatterLoad(NvBootClocksOscFreq OscFreq)
{
    // The purpose of this is to put the system clock on PLLP / 4 to
    // accelerate as fast as possible the C library functions that build
    // the C runtime environment.
    // This implies restrictions on what can be done here, especially any
    // global dynamic variables should be avoided, they would be
    // overwritten during the scatter loading

    NvU32 RegData;

    NV_ASSERT((NvU32) OscFreq <(NvU32) NvBootClocksOscFreq_MaxVal);

    // The definitive PLLP startup steps are contained in the libola spec for the
    // respective PLL type. PLLP is of type PLL700_LP_ESD.
    // Notes:
    // According to http://nvbugs/1391832, IDDQ transition from 1->0 is not needed.
    // Also, the 5us delay listed in the PLL datasheet is not needed.
    // The POR value of PLLP_SETUP is 0, so we need not program this either.
    // PLL LOCK_ENABLE sequencing is no longer needed.

    // Programing Mdiv / Ndiv and setup registers not needed in this case,
    // as we are relying on PLLP auto setup parameters by default.

    // PLL power and Control signals must be stable and input reference clock
    // must be running. Start PLL by setting ENABLE=1.
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLP_BASE_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 PLLP_BASE,
                                 PLLP_ENABLE,
                                 ENABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_BASE_0,
               RegData);

    // Wait for LOCK assertion. Make this a non-blocking function.
    // Caller must wait for lock assertion.
}

void
NvBootClocksEnablePllSpreadSpectrum(NvBootClocksPllId PllId,
                           NvBootClocksSSParams *SSCoefficientTable)
{
    NvU32 RegData;
    NvU32 StateVariable1 = 0;
    switch(PllId) {
        case NvBootClocksPllId_PllE:
               /*
                * If (enable SSD)
                * Then
                *     Save state of PLLE_CML output enable in StateVariable1
                *     Disable PLLE_CML0_OEN and PLLE_CML1_OEN
                *     Disable PLLE_ENABLE
                *     Disable PLLE_LOCK_ENABLE
                *
                *     Program SDM Min, Max, Step, DIN for SDM from static table
                *     Set PLLE_BYPASS_SS to 0
                *     Program PLLE_MISC1 to ENABLE SDM
                *  
                *     Enable PLLE_LOCK_ENABLE
                *     Enable PLLE_ENABLE
                *     Enable PLLE_CML0_OEN and PLLE_CML1_OEN
                *     Wait for PLLE to lock
                *     Enable SSC
                *
                * Else
                *     We need to enable SSA
                */
                if ( !SSCoefficientTable->IsSSA )
                {
                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0);
                    StateVariable1 = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML0_OEN, RegData);
                    StateVariable1 = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN, 
                           NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN, RegData), StateVariable1);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML0_OEN, 0, RegData);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN, 0, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0, RegData);
                    
                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
                    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK_ENABLE, DISABLE, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);
                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
                    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE, DISABLE, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegData);

                    RegData = 0;
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL1, PLLE_SDM_DIN, 
                              SSCoefficientTable->Sdmdin , RegData);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL1, PLLE_SDM_SSC_STEP,
                              SSCoefficientTable->Sdmsscstep, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL1_0, RegData);
                    RegData = 0;
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL2, PLLE_SDM_SSC_MAX,
                              SSCoefficientTable->Sdmsscmax , RegData);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL2, PLLE_SDM_SSC_MIN,
                              SSCoefficientTable->Sdmsscmin , RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL2_0, RegData);

                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL, PLLE_BYPASS_SS,
                              0 , RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);

                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_MISC1_0);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC1, PLLE_EN_SDM,
                              1 , RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);

                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
                    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK_ENABLE, ENABLE, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
                    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE, ENABLE, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegData);

                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML0_OEN, 
                              NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML0_OEN, StateVariable1), RegData);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN,
                              NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN, StateVariable1), RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0, RegData);

                    while (!NvBootClocksIsPllStable(NvBootClocksPllId_PllE, NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY));
                    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_MISC1_0);
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC1, PLLE_EN_SSC,
                              1 , RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);
                }
                else
                {
                    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0);
                    // Set PLLE_SSCINCINTRV = 0x1d default else programmable
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL,
                               PLLE_SSCINCINTRV, SSCoefficientTable->Sscincintrvl, RegData);

                    // Set PLLE_SSCINC = 0x1 default else programmable
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL, PLLE_SSCINC,
                               SSCoefficientTable->Sscinc, RegData);

                    // Set PLLE_SSCBYP=0
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL, PLLE_SSCBYP, 0, RegData);

                    // Set PLLE_BYPASS_SS=0
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL, PLLE_BYPASS_SS,
                               0, RegData);

                    // Set PLLE_SSCMAX = 0x0 default else programmable
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL,
                               PLLE_SSCMAX, SSCoefficientTable->Sscmax, RegData);

                    // Set PLLE_SSCCENTER = 0x1(center spread) default else determined by PLLE_SSCINVERT
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL,
                               PLLE_SSCCENTER, SSCoefficientTable->Ssccenter, RegData);

                    // Set PLLE_SSCINVERT = 0x0(down spread) default else programmable
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL,
                               PLLE_SSCINVERT, SSCoefficientTable->Sscinvert, RegData);

                    // Set PLLE_INTEGOFFSET = 0x0 default else programmable
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL,
                               PLLE_INTEGOFFSET, SSCoefficientTable->Integoffset, RegData);
                    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);

                    // Wait 300ns per PLLE Spec before writing 0 to INTERP_RESET
                    NvBootUtilWaitUS(1);

                    // Set PLLE_INTERP_RESET=0
                    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_SS_CNTL, PLLE_INTERP_RESET,
                               0, RegData);

                    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);

                }
            break;
       default:
                break;
    }

    
}
//  Start PLL using the provided configuration parameters
void
NvBootClocksStartPll(NvBootClocksPllId PllId,
                     NvU32 M,
                     NvU32 N,
                     NvU32 P,
                     NvU32 Misc1,
                     NvU32 Misc2,
                     NvU32 *StableTime ) {
    NvU32 RegData = 0;
    NvU32 RegPllBaseData = 0;
    NvU32 RegPllAuxData = 0;

    NVBOOT_CLOCKS_CHECK_PLLID(PllId);
    NV_ASSERT (StableTime != NULL);

    switch (PllId) {
        /* We can now handle each PLL explicitly by making each PLL its own case
         * construct. Unsupported PLL will fall into NV_ASSERT(0). Misc1 and Misc2
         * are flexible additional arguments for programming up to 2 32-bit registers.
         * If more is required, one or both can be used as pointer to struct.
         */


    case NvBootClocksPllId_PllU:
        /* PLLU: Misc1 - CPCON - Not used in T210
         *       Misc2 - LFCON - Not used in T210
         */

        // Enable Lock detect.
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLU_MISC_0);
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                     PLLU_MISC,
                                     PLLU_EN_LCKDET,
                                     ENABLE,
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLU_MISC_0,
               RegData);

        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLU_BASE_0);

        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_DIVM, M, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_DIVN, N, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_DIVP, P, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_OVERRIDE, 1, RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLU_BASE_0,
                   RegData);

        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_ENABLE, ENABLE,
                    RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLU_BASE_0,
                   RegData);

        // PLLU typical lock time is 1ms - bug 1018795
        *StableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE + TIMERUS_CNTR_1US_0) +
                      NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY + 1000;
        // wait for PLL lock    
        while (!NvBootClocksIsPllStable(PllId, *StableTime));

        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_PLLU_BASE_0);

        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_CLKENABLE_48M, 1, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_CLKENABLE_USB, 1, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_CLKENABLE_HSIC, 1, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, PLLU_CLKENABLE_ICUSB, 1, RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLU_BASE_0,
                RegData);
        break;
        
    case NvBootClocksPllId_PllM:
        // 0 - Reset Enable, 1 - Reset Disable
        // Note: On t210, this function does nothing as there are no external dividers
        // or branches for PLLM.
        NvBootClocksPllDivRstCtrl(PllId, 0);

        // Disable PLL as precaution
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLM_BASE_0);
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLM_BASE, 
                            PLLM_ENABLE, DISABLE, RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLM_BASE_0, RegData);

        // Enable Lock detect.
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLM_MISC2_0);
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                     PLLM_MISC2,
                                     PLLM_EN_LCKDET,
                                     ENABLE,
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLM_MISC2_0,
               RegData);

        // Program Setup, KVCO, KCP
        RegData =   NV_DRF_NUM(CLK_RST_CONTROLLER, PLLM_MISC1, PLLM_SETUP, \
                    NV_DRF_VAL(MISC1, CLK_RST_CONTROLLER_PLLM_MISC1, PLLM_SETUP, Misc1) );
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + 
                CLK_RST_CONTROLLER_PLLM_MISC1_0, RegData);

        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_PLLM_MISC2_0);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
                                     PLLM_MISC2,
                                     PLLM_KVCO,
                  NV_DRF_VAL(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KVCO, Misc2), 
                                     RegData );

        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                     PLLM_MISC2,
                                     PLLM_KCP,
                  NV_DRF_VAL(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KCP, Misc2 ),
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + 
                   CLK_RST_CONTROLLER_PLLM_MISC2_0, RegData);

        // Program M, N, P dividers.
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLM_BASE, PLLM_DIVM, M, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLM_BASE, PLLM_DIVN, N, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLM_BASE, PLLM_DIVP, P, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegData);

        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLM_BASE, PLLM_ENABLE, ENABLE,
                RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId),
                RegData);

        break;

    case NvBootClocksPllId_PllX:
        // IDDQ 1->0
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLX_MISC_3_0);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                     PLLX_MISC_3,
                                     PLLX_IDDQ,
                                     0,
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLX_MISC_3_0, RegData);
        // Wait 2us as in spec
        NvBootUtilWaitUS(2);

        // Set lock enable
        RegData = NV_DRF_DEF(CLK_RST_CONTROLLER, PLLX_MISC, \
                  PLLX_LOCK_ENABLE, ENABLE);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLX_MISC_0, RegData);

        // Program setup
        RegData = NV_DRF_NUM(CLK_RST_CONTROLLER, PLLX_MISC_1, PLLX_SETUP, \
                    NV_DRF_VAL(MISC1, CLK_RST_CONTROLLER_PLLX_MISC_1, PLLX_SETUP, Misc1) );
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLX_MISC_1_0, RegData);

        // Program KCP, KVCO
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLX_MISC_3_0);

        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLX_MISC_3, PLLX_KVCO, \
                    NV_DRF_VAL(MISC2, CLK_RST_CONTROLLER_PLLX_MISC_3, PLLX_KVCO, Misc2), RegData);

        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLX_MISC_3, PLLX_KCP, \
                    NV_DRF_VAL(MISC2, CLK_RST_CONTROLLER_PLLX_MISC_3, PLLX_KCP, Misc2), RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLX_MISC_3_0, RegData);

        // Program M, N, P
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLX_BASE, PLLX_DIVM, M, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLX_BASE, PLLX_DIVN, N, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLX_BASE, PLLX_DIVP, P, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId),
                RegData);

        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLX_BASE, PLLX_ENABLE, ENABLE,
                RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId),
                RegData);
        break;

    case NvBootClocksPllId_PllC4:
        // First disable LCKDET and ENABLE, in case this is a restart
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_MISC, PLLC4_EN_LCKDET, DISABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_ENABLE, DISABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegData);

        // Now begin starting the PLL
        // IDDQ 1 -> 0
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_IDDQ, OFF, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegData);

        // Wait 5us
        NvBootUtilWaitUS(5);

        // Program PLL Registers
        RegData = Misc1;
        // Always set LCKDET enable, we need this to start
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_MISC, PLLC4_EN_LCKDET, ENABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_DIVM, M, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_DIVN, N, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_DIVP, P, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId),
                RegData);

        // Enable 0 -> 1
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_ENABLE, ENABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegData);

        break;

    case NvBootClocksPllId_PllREFE:

        // Remove pll from iddq
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
        // POR value of Setup[23:0] is 0(as required by PLLREFE spec). However, bit 15 of SETUP is
        // "observe clockin on clkout" which should help us in case we need osc freq to be directly input
        // to a PLL(for example PLLE in this case) fed from PLLREFE.
        // ORing the value here allows us to program SETUP while keeping the lock detection enable to POR (disabled)
        // and also to program PLLREFE_LOCK_OVERRIDE and PLLREFE_PTS to allow debug capability as well as
        // override lock detect in case of issues.
        RegData |= Misc1;
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLREFE_MISC, PLLREFE_IDDQ, OFF, RegData);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        // PLL spec defined delay after iddq removal
        NvBootUtilWaitUS(5);

        // Disable lock detect enable before programming the divisors so that previous lock status does not
        // interfere with locking the pll with the new divisor values.
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLREFE_MISC, PLLREFE_EN_LCKDET, DISABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        // PLL now powered up. Set M/N/P divisors
        RegPllBaseData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegPllBaseData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_DIVM, M, RegPllBaseData);
        RegPllBaseData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_DIVN, N, RegPllBaseData);
        RegPllBaseData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_DIVP, P, RegPllBaseData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegPllBaseData);

        // Enable lock detect and PLLREFE
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLREFE_MISC, PLLREFE_EN_LCKDET, ENABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        // Enable PLLREFE output
        RegPllBaseData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLREFE_BASE, PLLREFE_ENABLE, ENABLE, RegPllBaseData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegPllBaseData);
        
        break;

   case NvBootClocksPllId_PllE:

        /*
         * Misc1 = VAL[PLLE_MISC_0] 
         */

        /*
         * Misc2 for PLLE = [PLLE_AUX_0] (bits PLLE_REF_SEL_PLLREFE, PLLE_REF_SRC, PLLE_CML0_OEN, PLLE_CML1_OEN)
         */
        // IDDQ_OVERRIDE_VAL = 1 and IDDQ_SWCTL = 1
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_IDDQ_OVERRIDE_VALUE, 0x1, RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_IDDQ_SWCTL, 0x1, RegData);
        RegData = NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);
        // Set PlleRefSource, SelectPllRefe in PLLE_AUX
        RegPllAuxData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0);
        RegData = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_REF_SRC, Misc2);
        RegPllAuxData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_REF_SRC, RegData, RegPllAuxData);
        RegData = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_REF_SEL_PLLREFE, Misc2);
        RegPllAuxData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_REF_SEL_PLLREFE, RegData, RegPllAuxData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0, RegPllAuxData);

        // LOCK_ENABLE = DISABLE, PLLE_ENABLE = DISABLE, PLLE_EXT_SETUP_23_16 = 0, PLLE_SETUP = 0
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_SETUP, 0, RegData);
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK_ENABLE, DISABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        RegPllBaseData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId));
        RegPllBaseData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE, DISABLE, RegPllBaseData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegPllBaseData);

        // PLLE_IDDQ_OVERRIDE_VAL = 0
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_IDDQ_OVERRIDE_VALUE, 0x0, RegData);
        RegData = NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        // Wait for 5 usec per PLLE spec
        NvBootUtilWaitUS(5);

        // Program M/N/PLDIV_CML divisors
        RegPllBaseData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_MDIV, M, RegPllBaseData);
        RegPllBaseData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_NDIV, N, RegPllBaseData);
        RegPllBaseData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_PLDIV_CML, P, RegPllBaseData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegPllBaseData);

        // PLLE_READY bit doesn't exist. Therefore, wait for training to complete else wait longer 
        // in the end for the lock bit to be set.
        // For SSA, Disable SS
        // SSC_BYP=1, BYPASS_SS=1 INTERP_RESET=1 EN_SDM=0 EN_SSC=0 - All are default


        // Program SETUP (usually 0)
        // RegData contains the updated value of register PLLE_MISC
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId));
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_SETUP,
                        NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_SETUP, Misc1) , RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_KCP,
                        NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_KCP, Misc1) , RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_VREG_BG_CTRL,
                        NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_VREG_BG_CTRL, Misc1) , RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_VREG_CTRL,
                        NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_VREG_CTRL, Misc1) , RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_KVCO,
                        NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_KVCO, Misc1) , RegData);


        // LOCK_ENABLE = ENABLE, PLLE_ENABLE = ENABLE
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK_ENABLE, ENABLE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(PllId), RegData);

        // RegPllBaseData contains the updated value of register PLLE_BASE 
        RegPllBaseData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLE_BASE, PLLE_ENABLE, ENABLE, RegPllBaseData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(PllId), RegPllBaseData);

        // PLLE_CML1_OEN = ENABLE for SATA, PLLE_CML0_OEN = ENABLE for PCIE
        // RegPllAuxData contains the updated value of register PLLE_AUX
        RegPllAuxData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN, 
                             NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML1_OEN, Misc2), RegPllAuxData);
        RegPllAuxData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML0_OEN, 
                             NV_DRF_VAL(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_CML0_OEN, Misc2), RegPllAuxData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0, RegPllAuxData);
        // For PLLE, Stable Time returned from this function may need to be overridden with a wait time
        // which is configurable in the calling function.
        // Program SSA/SDM coefficients and enable SSD before PLLE is locked or enable SSA after PLLE
        // has locked, as the case may be.
        break;

    default:
        NV_ASSERT(0);
        break;
    }
    // calculate the stable time
    *StableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE + TIMERUS_CNTR_1US_0) +
                  NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY;
}

// this code is mapped in the non secure part of the ROM
#ifdef __arm__
// NOTE: This pragma only makes sense when building for the ARM compilers.
#pragma arm section rodata = "ClocksNonsecure", code = "ClocksNonsecure" 
#endif

static const NvBootClocksMiscParams s_PllMiscTbl[] =
{
    /*  setup,  kcp     kvco,  divm,   divn,   divp    */
        0x00,   0x00,   0x00,   2,     0x32,    0   // PLLC
};

NvBootClocksMiscParams *
NvBootClocksGetPllMiscParams(NvBootClocksPllId PllId)
{
    NvBootClocksMiscParams *ret;

    switch(PllId) {
        
        case NvBootClocksPllId_PllC4:
            ret = (NvBootClocksMiscParams *)&s_PllMiscTbl[0];
            break;

        default:
            ret = NULL;
            NV_ASSERT(0);
            break;
    }
    return ret;
}

// Get the oscillator frequency, from the corresponding HW configuration field
NvBootClocksOscFreq 
NvBootClocksGetOscFreq(void)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + 
                        CLK_RST_CONTROLLER_OSC_CTRL_0);

    return (NvBootClocksOscFreq) NV_DRF_VAL(CLK_RST_CONTROLLER,
                                            OSC_CTRL,
                                            OSC_FREQ,
                                            RegData); 
}

// Get Actual oscillator frequency number
NvU32 
NvBootClocksGetOscFreqMhz(void)
{

#ifdef NV_DEBUG
    // These assertions ensure the hard-coded values in NvBootClocksOscRefFreqMhz
    //   matches enum definitions of CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC13 == 0);
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC16P8 == 1);
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC19P2 == 4);
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC38P4 == 5);
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC12 == 8);
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC48 == 9);
    NV_CT_ASSERT(CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC26 == 12);
#endif

    static NvU32 const NvBootClocksOscRefFreqMhz[NvBootClocksOscFreq_MaxVal] = 
    {13,16,0,0,19,38,0,0,12,48,0,0,26};

    return NvBootClocksOscRefFreqMhz[NvBootClocksGetOscFreq()];
}

static const NvU32 ClocksRegSetClrTbl[ClocksReg_Num] = {
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_L_SET_0,
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_H_SET_0,
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_U_SET_0,
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_X_SET_0,
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_V_SET_0,
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_W_SET_0,
    NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_ENB_Y_SET_0,
};

// Enable the clock, this corresponds generally to level 1 clock gating
void
NvBootClocksSetEnable(NvBootClocksClockId ClockId, NvBool Enable) {
    NvU32 RegData;
    NvU32 RegAddr;

    NVBOOT_CLOCKS_CHECK_CLOCKID(ClockId);
    NV_ASSERT(((int) Enable == 0) ||((int) Enable == 1));

    // The simplest case is via bits in register ENB_CLK
    // But there are also special cases
    if(NVBOOT_CLOCKS_HAS_STANDARD_ENB(ClockId))
    {
        // Take advantage of the the SET and CLR registers to enable/disable
        // the clocks. No longer need read-modify-writes.
        RegAddr = ClocksRegSetClrTbl[NVBOOT_CLOCKS_REG_OFFSET(ClockId)];

        if (!Enable)
        {
            RegAddr += 4;// CLR register is always immediately after SET register
        }
        NV_WRITE32(RegAddr, 1 << NVBOOT_CLOCKS_BIT_OFFSET(ClockId));
    }
    else
    {
        // there is no bit in CLK_ENB, less regular processing needed
        switch(ClockId)
        {
        // Note for NvBootClocksClockId_SclkId:
        // There is no way to stop Sclk, for documentation purpose

        case NvBootClocksClockId_SataAuxId:
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                   CLK_RST_CONTROLLER_CLK_SOURCE_SATA_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                         CLK_SOURCE_SATA,
                                         SATA_AUX_CLK_ENB,
                                         Enable,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
                   CLK_RST_CONTROLLER_CLK_SOURCE_SATA_0,
                   RegData);
            break;

        default :
            // nothing for other enums, make that explicit for compiler
            break;
        };
    };

    NvBootUtilWaitUS(NVBOOT_CLOCKS_CLOCK_STABILIZATION_TIME);
}

void
NvBootClocksSetOscFreq(NvBootClocksOscFreq OscFreq)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_OSC_CTRL_0);

    // Previous chips like T124 required the setting of PLL_REF_DIV (DIV2
    // for 38.4Mhz oscillators and DIV4 for 48Mhz) for oscillator frequencites
    // greater than 26Mhz. This requirement no longer applies to T210. See
    // T210_Clocks_Delta_IAS.docx (Section 3.1.2 Pll reference clock) and bugs
    // 1407864, 1272505.

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 OSC_CTRL,
                                 OSC_FREQ,
                                (int)OscFreq,
                                 RegData);

    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_OSC_CTRL_0,
               RegData);
}

NvBool
NvBootClocksIsPllC4Stable(NvU32 StableTime)
{
    NvU32 DeltaTime;
    NvU32 RegData;
    NvBool Lock;

    DeltaTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE + TIMERUS_CNTR_1US_0) - 
        StableTime;
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLC4_BASE_0);
    Lock = NV_DRF_VAL(CLK_RST_CONTROLLER,
                      PLLC4_BASE,
                      PLLC4_LOCK,
                      RegData);
#ifdef TARGET_ASIM
    Lock = 1;
#endif
    if (Lock ||!(DeltaTime & (1U << 31)))
    {
        // some delay required aftet the lock bit indicates the PLL is lock 
        if(Lock)
            NvBootUtilWaitUS(NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY_AFTER_LOCK);

        // Disable OUTx divider reset (Bug 954159)
        // 0 - Reset Enable, 1 - Reset Disable
//        NvBootClocksPllDivRstCtrl(PllId, 1);

        return NV_TRUE;
    }
    // sign bit set, so DeltaTime is negative, not ready yet  (or)
    // lock bit is not set, not ready yet 
    return NV_FALSE;
}

//  Start PLL using the provided configuration parameters
void
NvBootClocksStartPllC4(NvU32 M,
                       NvU32 N,
                       NvU32 P,
                       NvU32 Misc,
                       NvU32 *StableTime ) {
    NvU32 RegData;

    // First disable LCKDET and ENABLE, in case this is a restart
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(NvBootClocksPllId_PllC4));
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_MISC, PLLC4_EN_LCKDET, DISABLE, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(NvBootClocksPllId_PllC4), RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(NvBootClocksPllId_PllC4));
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_ENABLE, DISABLE, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(NvBootClocksPllId_PllC4), RegData);

    // Now begin starting the PLL
    // IDDQ 1 -> 0
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(NvBootClocksPllId_PllC4));
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_IDDQ, OFF, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(NvBootClocksPllId_PllC4), RegData);

    // Wait 5us
    NvBootUtilWaitUS(5);

    // Program PLL Registers
    // Always set LCKDET enable, we need this to start
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_MISC, PLLC4_EN_LCKDET, ENABLE, Misc);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_MISC(NvBootClocksPllId_PllC4),
               RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(NvBootClocksPllId_PllC4));
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_DIVM, M, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_DIVN, N, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_DIVP, P, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + NVBOOT_CLOCKS_PLL_BASE(NvBootClocksPllId_PllC4), RegData);

    // Enable 0 -> 1
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_BASE, PLLC4_ENABLE, ENABLE, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLC4_BASE_0, RegData);

    // calculate the stable time
    *StableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE + TIMERUS_CNTR_1US_0) +
                  NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY;
}

NvBootClocksOscFreq
NvBootClocksMeasureOscFreq(void)
{
    NvU32 Cnt;

#if NVBOOT_TARGET_RTL && NVBOOT_SKIP_OSC_FREQ_IN_RTL
    Cnt = NVBOOT_CLOCKS_MIN_CNT_38_4;
#else
    NvU32 RegData;
    // start measurement, window size uses n-1 coding
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_OSC_FREQ_DET_0, 
                NV_DRF_DEF(CLK_RST_CONTROLLER,
                           OSC_FREQ_DET,
                           OSC_FREQ_DET_TRIG,
                           ENABLE) |
                NV_DRF_NUM(CLK_RST_CONTROLLER,
                           OSC_FREQ_DET,
                           REF_CLK_WIN_CFG,
                           (1-1)));

    // wait until the measurement is done
    do
    {
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_OSC_FREQ_DET_STATUS_0);
    } 
    while(NV_DRF_VAL(CLK_RST_CONTROLLER,
                     OSC_FREQ_DET_STATUS,
                     OSC_FREQ_DET_BUSY,
                     RegData));

    Cnt = NV_DRF_VAL(CLK_RST_CONTROLLER,
                     OSC_FREQ_DET_STATUS,
                     OSC_FREQ_DET_CNT,
                     RegData);  
#endif

    if((Cnt >= NVBOOT_CLOCKS_MIN_CNT_12) &&
       (Cnt <= (NVBOOT_CLOCKS_MAX_CNT_12 + ((NVBOOT_CLOCKS_MIN_CNT_13 - NVBOOT_CLOCKS_MAX_CNT_12)/2))))
    {
        return NvBootClocksOscFreq_12;
    }
    if((Cnt >= (NVBOOT_CLOCKS_MIN_CNT_13 - ((NVBOOT_CLOCKS_MIN_CNT_13 - NVBOOT_CLOCKS_MAX_CNT_12)/2))) &&
       (Cnt <= (NVBOOT_CLOCKS_MAX_CNT_13 + ((NVBOOT_CLOCKS_MIN_CNT_16_8 - NVBOOT_CLOCKS_MAX_CNT_13)/2))))
    {
        return NvBootClocksOscFreq_13;
    }
    if((Cnt >= (NVBOOT_CLOCKS_MIN_CNT_16_8 - ((NVBOOT_CLOCKS_MIN_CNT_16_8 - NVBOOT_CLOCKS_MAX_CNT_13)/2))) &&
       (Cnt <= (NVBOOT_CLOCKS_MAX_CNT_16_8 + ((NVBOOT_CLOCKS_MIN_CNT_19_2 - NVBOOT_CLOCKS_MAX_CNT_16_8)/2))))
    {
        return NvBootClocksOscFreq_16_8;
    }
    if((Cnt >= (NVBOOT_CLOCKS_MIN_CNT_19_2 - ((NVBOOT_CLOCKS_MIN_CNT_19_2 - NVBOOT_CLOCKS_MAX_CNT_16_8)/2))) &&
       (Cnt <= (NVBOOT_CLOCKS_MAX_CNT_19_2 + ((NVBOOT_CLOCKS_MIN_CNT_26 - NVBOOT_CLOCKS_MAX_CNT_19_2)/2))))
    {
        return NvBootClocksOscFreq_19_2;
    }
    if((Cnt >= (NVBOOT_CLOCKS_MIN_CNT_26 - ((NVBOOT_CLOCKS_MIN_CNT_26 - NVBOOT_CLOCKS_MAX_CNT_19_2)/2)+1)) &&
       (Cnt <= (NVBOOT_CLOCKS_MAX_CNT_26 + ((NVBOOT_CLOCKS_MIN_CNT_38_4 - NVBOOT_CLOCKS_MAX_CNT_26)/2))))
    {
        return NvBootClocksOscFreq_26;
    }
    if((Cnt >= (NVBOOT_CLOCKS_MIN_CNT_38_4 - ((NVBOOT_CLOCKS_MIN_CNT_38_4 - NVBOOT_CLOCKS_MAX_CNT_26)/2))) &&
       (Cnt <= (NVBOOT_CLOCKS_MAX_CNT_38_4 + ((NVBOOT_CLOCKS_MIN_CNT_48 - NVBOOT_CLOCKS_MAX_CNT_38_4)/2))))
    {
        return NvBootClocksOscFreq_38_4;
    }
    if((Cnt >= (NVBOOT_CLOCKS_MIN_CNT_48 - ((NVBOOT_CLOCKS_MIN_CNT_48 - NVBOOT_CLOCKS_MAX_CNT_38_4)/2))) &&
       (Cnt <= NVBOOT_CLOCKS_MAX_CNT_48))
    {
        return NvBootClocksOscFreq_48;
    }
    return NvBootClocksOscFreq_Unknown;
}

void
NvBootClocksConfigureUsecTimer(NvBootClocksOscFreq OscFreq)
{
    NvU32 UsecCfg;

    if(OscFreq == NvBootClocksOscFreq_Unknown)
    {
        // Note: This case should never happen.
        // Discussion has raged about how to handle this case.  The final
        // consensus was to force operation as if the oscillator
        // frequency was 38.4 MHz. Rejected (but somehow reluctantly)
        // was an alternate proposal to calculate
        // approximately correct M,N,P based on the measured frequency.
        // We do remember the error by returning an error code (but
        // currently dropped)
        UsecCfg = s_UsecCfgTable[(NvU32) NvBootClocksOscFreq_Default_If_Unknown];
        OscFreq = NvBootClocksOscFreq_Default_If_Unknown;
    }
    else
    {
        UsecCfg = s_UsecCfgTable[(NvU32) OscFreq];
    }
    NV_WRITE32(NV_ADDRESS_MAP_TMRUS_BASE + TIMERUS_USEC_CFG_0, UsecCfg);
}

// revert to normal mapping again if new code added below
#ifdef __arm__
// NOTE: This pragma only makes sense when building for the ARM compilers.
#pragma arm section
#endif
