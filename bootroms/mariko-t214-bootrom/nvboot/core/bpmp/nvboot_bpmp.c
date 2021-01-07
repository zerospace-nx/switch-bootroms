/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <nvboot_bit.h>
#include <nvboot_section_defs.h>
#include <nvboot_tasks_ns.h>
#include <nvboot_bpmp_int.h>
#include <nvboot_pmc_int.h>
#include <nvboot_clocks_int.h>
#include <nvboot_reset_int.h>
#include "nvboot_se_aes.h"
#include "nvboot_crypto_param.h"
#include "nvboot_pka_ecc.h"
#include "nvboot_pka_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_context_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_strap_int.h"
#include "nvboot_wdt_int.h"
#include "nvboot_rcm_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_util_int.h"
#include "nvboot_sdram_int.h"
//#include "nvboot_apb2jtag_int.h"
#include "nvboot_se_int.h"
#include "nvboot_irom_patch_int.h"

#include "nvboot_crypto_sec_irom.h"

#include "nvtypes.h"
//#include "address_map_new.h"
#include <nvboot_bct.h>
//#include "arpmc_impl.h"
//#include "arscratch.h"
#include "nvrm_drf.h"
//#include "arbpmp_atcmcfg.h"
//#include "armiscreg.h"
//#include "artsa.h"
//#include "araonrmu_br.h"
//#include "araonrmu_br_scr.h"
//#include "arpmc_impl.h"
//#include "arpmc_misc.h"
#include "arapb_misc.h"
#include "arsecure_boot.h"
#include "arevp.h"
#include "artimerus.h"
#include "arse.h"
#include "nvboot_version_defs.h"
#include "nvboot_crypto_fskp.h"

/*
 * The following bit flags indicate various actions to be carried out upon exit
 * from the Secure Section of the Boot ROM code.
 */

/* reset the full chip */
#define NVBOOT_SECURE_EXIT_OPTION_FULLRESET (0x1 << 0)
/* zero out the Boot Information Table (BIT) */
#define NVBOOT_SECURE_EXIT_OPTION_CLEAR_BIT (0x1 << 1)
/* zero out the Boot Configuration Table (BCT) */
#define NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT (0x1 << 2)
/* enable JTAG (the corresponding fuse still has override power) */
#define NVBOOT_SECURE_EXIT_OPTION_JTAG_DIS  (0x1 << 3)

extern NvBootInfoTable   BootInfoTable;
extern NvBootContext     Context;
extern NvBootConfigTable *pBootConfigTable;
//extern NvBootConfigTable BootConfigTable; // Actual BCT storage in IRAM at .MainBCT.
extern uint32_t * __bit_start;
extern uint32_t * __bct_start;
extern uint32_t * __stack_top;
extern uint32_t * __data_after_bct;
extern uint32_t * __data_start;
extern uint32_t * __crypto_buffer_start;
extern uint32_t * __crypto_buffer_end;

/* BPMP exception loop in SYSRAM */
NV_ALIGN(4) NvU32 VT_NOZI ExcpLoop;

void FT_NONSECURE SetupGlobalClockOverride()
{
#if 0
    NvU32 GlobalClkOvr = 0;

    // 8.1.5     Global clock override See 7.3.7  for cold boot reference. There's nothing to be done for warm boot code in this step
    if(NvBootQueryRstStatusWarmBootFlag())
        return;

    // Global second level clock gate override turned on by default at coldboot and AO for Warmboot
    //	http://nvbugs/200044415 (http://nvbugs/200038440)
    // If the fuse is burnt, set CLK_RST_CONTROLLER_MISC_REG0[GLOBAL_CLK_OVR_ON]= 0

    GlobalClkOvr = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
    				CLK_RST_CONTROLLER_MISC_REG0_0);
    GlobalClkOvr = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
					 MISC_REG0,
					 GLOBAL_CLK_OVR_ON,
					 !(NvBootFuseGlobalClkOverrideEnabled()),// setting GLOBAL_CLK_OVR_ON = 0
					 GlobalClkOvr);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_MISC_REG0_0,
                    GlobalClkOvr);
#endif
}


void NvBootBpmpAonXusbUfsLogicReset()
{
#if 0
    // 8.1.5     Global clock override See 7.3.7  for cold boot reference. There's nothing to be done for warm boot code in this step
    if(NvBootQueryRstStatusWarmBootFlag())
        return;

    // 7.3.19	Bring AO-logic out of reset
    // http://nvbugs/200070837
    // Refer Section 7.2 of the SC8 GFD for register details on applying these resets.
    // a.	XUSBAO is reset through SW control bit in CAR. RST_DEV_XUSB_AON bit in SWR_XUSB_VAUX_AON_RST will be used for this purpose. (200065305
    // CLK_RST_CONTROLLER_RST_DEV_XUSB_AON_0
    // DO NOT use Reset API for this register/.
    // NvBootResetSetEnable(NvBootResetDeviceId_XusbAonId, NV_FALSE);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_XUSB_AON_CLR_0, 1);

    // a.	 CLK_RST_CONTROLLER_RST_DEV_UFS_AON register is written to assert the master reset to the UFSHC. This SW control resets all logic UFSHC
    // ufs is not clear with reset/assertion logic..
    // NvBootResetSetEnable(NvBootResetDeviceId_UfsAonId, NV_FALSE);
		#ifdef NV_CAR_HAS_UFS
			NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_UFS_AON_CLR_0, 1);
		#endif
#endif
}

NvBootError NvBootBpmpSubSystemInit(void)
{
    NvU32 RegData;

    // Check if the reset status is LP0 or not. This check is important
    // because this function will also be utilized outside of Boot ROM (uartmon).
    // If called in uartmon during an SC7 exit the NvBootEnableMemClk function call
    // below with PreserveDRAM = false will cause loss of DRAM contents.
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);
    if (NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) !=
            APBDEV_PMC_RST_STATUS_0_RST_SOURCE_LP0)
    {
        // For T214, this function should only be called
        // in a NON-SC7 path. SC7 path will do its own function call of
        // NvBootEnableMemClk because you will need the pointer to the unpacked SDRAM parameters.
        //
        // Enable Mem clk per Boot GFD. Reference section 1.6, 1.11.
        // Preserve DRAM = true for SC7.
        // If PreserveDRAM is true, the following function can only be called after
        // unpacking SDRAM parameters.
        NvBootEnableMemClk (NULL, NV_FALSE);
        }

    return NvBootError_Success;
}

void FT_NONSECURE NvBootBpmpFabricInit(void)
{
#if 0
    NvBool IsSc7orSc8 = 0;
    NvBool SkipAonCpu = 0;
    NvBool SkipAonApb = 0;
    NvU32 RegData;

    // Boot clocks setup for BPMP Cpu/Fabric/SE/EMC(ROC) with PLLP_out0 for coldboot/warmboot path
    // Check coldboot/warmboot  path
    uint8_t switch_divider_cb_204 = NVBOOT_CLOCKS_N_DIVIDER_BY_3; //main cpu clk/SE/EMC divisor
    /**
     *  BPMP_APB, AXI_CBB and AON_APB use an (N/2) + 1 (lsb denoting 1.0x) divider
     *  Therefore, if PLLP_OUT0 (=408MHz) is used as clock source to these bridges,
     *  in order to run them at 102MHz, N = 6.
     *  Other IPs needing to derive 102MHz may not necessarily need this divider based
     *  on the arclk_rst spec file.
     *
     **/
    uint8_t switch_divider_cb_102 = NVBOOT_CLOCKS_7_1_DIVIDER_BY(3, 0); //Apb / axi cbb /fabric bridges

    // clock divisor for coldboot & warmboot same

    // if osc clk fuse select enabled, skip PLLP source switch for fabric
    if(!NvBootFuseIsOscClkFallBackEnabled())
    {
        //log BIT
        //bpmp CPU clock @ 204 Mhz @ coldboot
        NvBootClocksConfigureClock(NvBootClocksClockId_BpmpCclkId,
            NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_204, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_BPMP_CPU_NIC_0_BPMP_CPU_NIC_CLK_SRC_PLLP_OUT0);

        // Set SE engine clocks/enables
        NvBootClocksConfigureClock(NvBootClocksClockId_SeId,
            NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_204, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_SE_0_SE_CLK_SRC_PLLP_OUT0);

        //set up clocks/enables for Bpmp Apb/Fabric
        //APB
        NvBootClocksConfigureClock(NvBootClocksClockId_BpmpApbId,
            NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_102, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_BPMP_APB_0_BPMP_APB_CLK_SRC_PLLP_OUT0);


        // AXI_CBB
        NvBootClocksConfigureClock(NvBootClocksClockId_BpmpAxiCbbId,
            NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_102, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_AXI_CBB_0_AXI_CBB_CLK_SRC_PLLP_OUT0);

        // Bug 200116031. Skip configuring AON CPU
#define SKIP_WARMBOOT_AON_CPU_CLK    0x1
#define SKIP_WARMBOOT_AON_APB_CLK    0x2
        IsSc7orSc8 = NvBootQueryRstStatusSC8Status() || NvBootQueryRstStatusWarmBootFlag();
        if(IsSc7orSc8)
        {
            RegData = NV_READ32(NV_ADDRESS_MAP_SCRATCH_BASE + SCRATCH_SCRATCH_11);
            if(RegData & SKIP_WARMBOOT_AON_CPU_CLK)
                SkipAonCpu = 1;
            if(RegData & SKIP_WARMBOOT_AON_APB_CLK)
                SkipAonApb = 1;
        }
        //AON clock @ 204 Mhz @ coldboot
        //Aon CPU/NIC
        if(!SkipAonCpu)
        {
            NvBootClocksConfigureClock(NvBootClocksClockId_AonCclkId,
                NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_204, 0),
                CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_0_AON_CPU_NIC_CLK_SRC_PLLP_OUT0);
        }

        //Aon APB
        if(!SkipAonApb)
        {
            NvBootClocksConfigureClock(NvBootClocksClockId_AonApbId,
                NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_102, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_0_AON_APB_CLK_SRC_PLLP_OUT0);
        }

    }



    // set bpmp NIC (nic Rate) clock handling here..

    // Enable the cpu & nic clock.
    NvBootClocksSetEnable(NvBootClocksClockId_BpmpCclkId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_BpmpCNicclkId, NV_TRUE);



    // Enable Se  clock to the controller
    NvBootClocksSetEnable(NvBootClocksClockId_SeId, NV_TRUE);

    // Enable the APB clk.
    NvBootClocksSetEnable(NvBootClocksClockId_BpmpApbId, NV_TRUE);


    // Enable the AXI CBB clk.
    NvBootClocksSetEnable(NvBootClocksClockId_BpmpAxiCbbId, NV_TRUE);

    // skip the following setting, by default the ratio is 1/1
/****
    AXI_CBB_CENTRAL,
    AXI_CBB_AON,
    AXI_CBB_PCIE,
    AXI_CBB_CCPLEX
    AXI_CBB_GPU
    AXI_CBB_APE
    AXI2APB_1
    AXI2APB_2
    AXI2APB_3
    AXI2APB_4
    AXI2APB_5
***/

    // set AON (Nic Rate) clock handling here..

    // Enable the Aon cpu & nic clock.
    if(!SkipAonCpu)
    {
        NvBootClocksSetEnable(NvBootClocksClockId_AonCclkId, NV_TRUE);
        NvBootClocksSetEnable(NvBootClocksClockId_AonCNicclkId, NV_TRUE);
    }

    if(!SkipAonApb)
    {
        // Enable the Aon APB clk.
        NvBootClocksSetEnable(NvBootClocksClockId_AonApbId, NV_TRUE);
    }

    // if skip emc/roc clk source switch to PLLP enabled, skip PLLP source switch for emc/roc clock.
    if((!NvBootFuseIsFallBackClkEnabled()) && (!NvBootFuseIsOscClkFallBackEnabled()) )
    {
        //log BIT

        //EMC clk src PLLP_OUT0@ coldboot
        NvBootClocksConfigureClock(NvBootClocksClockId_EmcId,
            NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_204, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLP_OUT0);

        //EMCSB clk src PLLP_OUT0@ coldboot
        NvBootClocksConfigureClock(NvBootClocksClockId_EmcsbId,
            NVBOOT_CLOCKS_7_1_DIVIDER_BY(switch_divider_cb_204, 0),
            CLK_RST_CONTROLLER_CLK_SOURCE_EMCSB_0_EMC_2X_CLK_SRC_PLLP_OUT0);

    }
    //log BIT
    BootInfoTable.BootROMtracker = NvBootFlowStatus_FabricInitialized;
#endif
}

void
NvBootBpmpSecureRomEnterBeforeScatter()
{
    NvBootClocksOscFreq OscFreq;
    NvU32 RegData, pllPStableTime;

    // Assert PLL*_OUT*_RSTN field to PLL*_OUT* divider. This step
    // must be done regardless if starting up PLLP manually or via auto-restart.
    // This step must be done before startup of the PLL. The reset disable
    // will be done in NvBootClocksIsPllStable.
    // Enable OUTx divider reset (Bug 954159)
    // 0 - Reset Enable, 1 - Reset Disable
    NvBootClocksPllDivRstCtrl(NvBootClocksPllId_PllP, 0);

    // Only start PLLP if it did not happen automatically on a warm boot.
    // Note that GetOscFreq() will always return a reasonable value.
    if (!NvBootPmcIsPllpOverrideEnabled())
    {
        OscFreq = NvBootClocksGetOscFreq();
        NvBootClocksStartPllpBeforeScatterLoad(OscFreq);
    }
    // If the above NvBootClocksStartPllpBeforeScatterLoad function was not
    // called, it is assumed that PLLP has been auto-started by
    // programming the appropriate registers in PMC. In this case, the
    // hardware will sequence the PLL except for enabling the lock detection
    // circuitry.

    // Make sure the lock detection circuitry is enabled. This step is for
    // safety only, as the lock detection circuitry should already be enabled
    // at POR for the PLLs that can be auto-restarted (i.e. PLLP/U/M; i.e. POR reset
    // value for PLLP_EN_LCKDET is ENABLE).
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLP_MISC_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                         PLLP_MISC,
                         PLLP_EN_LCKDET,
                         ENABLE,
                         RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_MISC_0,
               RegData);

    pllPStableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE +
                               TIMERUS_CNTR_1US_0) + NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY;
    // Poll for PLLP lock bit and timeout on polling loop
    while (!NvBootClocksIsPllStable(NvBootClocksPllId_PllP, pllPStableTime));

    NvBootClocksSetAvpClockBeforeScatterLoad();
}

/**
 *  t210 bug fix http://nvbugs/1867566
 *  Tegra should not re-sample strapping option on PMC.MAIN_RST and Tegra WDT reset
 */
void FT_NONSECURE NvBootOverrideNonPORStraps(void)
{
    NvU32 RegData;
    NvU32 Straps;

    // Check if Coldboot (POR) or any other boot.
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_RST_STATUS_0);

    if(NV_DRF_VAL(APBDEV_PMC, RST_STATUS, RST_SOURCE, RegData) == \
        APBDEV_PMC_RST_STATUS_0_RST_SOURCE_POR)
    {
        // Sample straps on POR and store in Secure Scratch 111 after clearing RCM straps.
        // Note: RCM straps are active low so OR with 0x7 to disable.
        Straps = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_STRAPPING_OPT_A_0);
        Straps |=  APB_MISC_PP_STRAPPING_OPT_A_0_RCM_STRAPS_FIELD;
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH111_0, Straps);
    }
    else{
        // Non PoR boot; Override with straps sampled in POR.
        Straps =  NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH111_0);
        NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_STRAPPING_OPT_A_0, Straps);
    }

    // Write lock Secure Scratch 111
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE8_0);
    RegData = NV_FLD_SET_DRF_DEF(APBDEV_PMC, SEC_DISABLE8, WRITE111, ON, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SEC_DISABLE8_0, RegData);
}

// The T210 function name was NvBootMainNonsecureConfigureClocks.
void FT_NONSECURE NvBootBpmpSetupOscClk(void)
{
    NvBootClocksOscFreq OscFreq;

    if (NvBootPmcIsPllpOverrideEnabled())
    {
        // The override is enabled, so only copy the OscFreq to CAR.
        OscFreq = NvBootPmcGetPllpOverrideOscFreq();
    }
    else
    {
        // The override is not enabled, so measure the oscillator,
        // store it in CAR, store it in the Pllp Override fields.
        // PLLP will actually be started later if needed.
        OscFreq = NvBootClocksMeasureOscFreq();

        // If the frequency is unknown, we assume 38.4 MHz, the primary
        // oscillator frequency expected on T210 platforms.
        if (OscFreq == NvBootClocksOscFreq_Unknown)
        {
            OscFreq = NvBootClocksOscFreq_Default_If_Unknown;
        }

        // Set up the BootROM portion of the PllpOverride.
        NvBootPmcSetPllpOverride(NV_TRUE, OscFreq);
    }

    // Store the oscillator frequency in CAR.
    NvBootClocksSetOscFreq(OscFreq);

    // Configure the microsecond timer.
    NvBootClocksConfigureUsecTimer(OscFreq);

    /**
     * Set up PLLU auto-restart on WB0. Note Boot ROM is only
     * responsible for enabling the PLL auto-restart via PLLx_ENABLE.
     * PLLx_OVERRIDE_ENABLE is the domain of the BL, OS, or LP0 entry code.
     * Assumes OscFreq has the correct value
     * If PllU override is enabled, then do nothing.
     */
    if (!NvBootPmcIsPlluOverrideEnabled())
    {
        NvBootPmcSetPlluOverride(NV_TRUE, OscFreq);
    }

	//log BIT
    BootInfoTable.BootROMtracker = NvBootFlowStatus_SetupOscClk;
}


/**
 * Replaces NvBootMainNonsecureRomEnter from T210.
 */
void FT_NONSECURE
NvBootBpmpNonsecureRomEnter(void)
{
    NvBootClocksOscFreq OscFreq;
    NvU32 RegData;
    NvBool IsPreproductionUartBoot = NV_FALSE;

    // Osc frequency detect and programming into CAR done in NvBootBpmpSetupOscClk().
    OscFreq = NvBootClocksGetOscFreq();

    // check for Failure Analysis (FA) Mode
    //
    // FA Mode is the highest priority; it overrides all other modes
    // In FA and Preproduction, the only possible path is to the UART
    // bootloader
#if NVBOOT_TARGET_QT
    // Bit 29:26 are BOOT_SELECT. Bit 9 is BOOT_FAST_UART.
    // Use UART boot and select slow UART
    NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE +
                    APB_MISC_PP_STRAPPING_OPT_A_0, 0x0);
#endif
    if ( NvBootFuseIsFailureAnalysisMode() || NvBootFuseIsPreproductionMode() )
    {
        if (NvBootFuseIsPreproductionMode())
        {
            // Ignore strap on LP0 exit
            RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH0_0);
            if (RegData & NvBootPmcFlagId_Wb0)
                return;
            // Or Enter UART Boot if BOOT_SELECT = NvBootStrapDevSel_UartBoot_Preproduction
            RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                    APB_MISC_PP_STRAPPING_OPT_A_0);
            if (NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A,
                           BOOT_SELECT, RegData) == NvBootStrapDevSel_UartBoot_PreProduction)
                IsPreproductionUartBoot = NV_TRUE;

            if (IsPreproductionUartBoot == NV_FALSE)
                return;

        }

        // By design in RTL, the Protected PIROM cannot be read when the FA
        // fuse is blown. However, we must ensure that the SB_PIROM_START_0
        // register is also write protected to ensure that the Protected
        // ROM start address is not programmed to some non-expected value
        // by a mallicious entity such that the Protected IROM can be read even
        // though we are in FA mode and the PRIOM_DISABLE bit is set.
        //
        // The SB_PIROM_START_0 register is only programmable while
        // SECURE_BOOT_FLAG is ENABLE. Thus, in FA mode, before downloading
        // and executing the UART payload, set SECURE_BOOT_FLAG to DISABLE
        // and PIROM_DISABLE to DISABLE. Setting SECURE_BOOT_FLAG to DISABLE
        // write protects SB_PIROM_START_0, making it impossible to change the
        // start address to some non-expected value such that the PIROM is
        // able to be read. In addition, PIROM_DISABLE is set to DISABLE to
        // disable the reading of the PIROM region.
        //
        // Any entity should not be able to read the protected IROM except for
        // the Boot ROM starting in T132. This is to protect the MTS Decryption
        // Key which will be stored in IROM for T132.
        // See http://nvbugs/1185573 and http://nvbugs/1034867.
        //
        // Notes: Setting PIROM_DISABLE is just a guarantee mechanism. If the
        // FA fuse is burnt, PIROM shouldn't be readable regardless.
        if (NvBootFuseIsFailureAnalysisMode())
        {
            RegData = NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0);
            RegData = NV_FLD_SET_DRF_DEF(SB,
                                         CSR,
                                         PIROM_DISABLE,
                                         DISABLE,
                                         RegData);
            RegData = NV_FLD_SET_DRF_DEF(SB,
                                         CSR,
                                         SECURE_BOOT_FLAG,
                                         DISABLE,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0, RegData);

            // Ensure FEK is read locked in FA mode.
            RegData = NV_DRF_NUM(APB_MISC, PP_FEK_RD_DIS, FEK_RD_DIS, 1);
            NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_FEK_RD_DIS_0, RegData);

    		BootInfoTable.BootROMtracker = NvBootFlowStatus_FAMode;
        }

        // boot from UART
        // NOTE: this routine should never return
        NvBootUartDownload(OscFreq);

        // if we got here, something went terribly wrong ... reset the chip
        NvBootResetFullChip();
    }
}

void FT_NONSECURE NvBootBpmpCheckSc7WakeStatus(void)
{
    TODO
#if 0
    if(NvBootQueryRstStatusSC7AowakeFlag())
    {
        // Infinite loop awaiting L2 reset via SPE WDT's 4th expiry
        NvBootMainNonSecureBootLoader();
    }
#endif
}

void NvBootBpmpEnableWdt(void)
{
    // odm production mode and watchdog enable fuse are set
    if (NvBootFuseIsOdmProductionMode() && NvBootFuseIsWatchdogEnabled()) {
        NvBootWdtInit();
        NvBootWdtStart();
    }
}

void NvBootBpmpEnablePllp(void)
{
    NvBootClockEnablePllp();
	//log BIT
    BootInfoTable.BootROMtracker = NvBootFlowStatus_PllpEnabled;

}

NvBootError NvBootBpmpEnableApb2jtag(void)
{
    //TODO
#if 0
    NvU32 arg0 = 0;
    NvU32 arg1 = 0;
    NvU32 arg2 = 0;
    NvU32 arg3 = 0;
    NvU32 arg4 = 0;
    NvU32 arg5 = 0;
    NvU32 arg6 = 0;
    NvU32 arg7 = 0;

    NvBootApb2jtag_Update(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7 );
#endif
    return NvBootError_Success;
}

/*
 * Do not use this anymore. FI attacks possible.
void FT_NONSECURE NvBootMainNonSecureBootLoader()
{
    while(1);
}
*/

static const NvU32 EvpTable[] = {
//    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_RESET_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_UNDEF_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_SWI_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_PREFETCH_ABORT_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_DATA_ABORT_VECTOR_0,
//    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_RSVD_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_IRQ_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_FIQ_VECTOR_0
};

extern int32_t FI_counter1;

void
NvBootMainSecureInit()
{
    NvBootUtilMemset(&Context, 0, sizeof(NvBootContext));
    NvBootUtilMemset( (void*) &__bct_start, 0, sizeof(NvBootConfigTable) );
    NvBootUtilMemset( (void*) &__bit_start, 0, sizeof(NvBootInfoTable) );

    // Init security related context.
    Context.FactorySecureProvisioningMode = 0;
    Context.ProvisioningKeyNum = FSKP_DISABLED;
    Context.Rcm_CopyKeysToSysram = NV_FALSE;

    FI_counter1 = 0;
}

/* Bare minimum secure rom exit routine **/
void
NvBootMainSecureRomExit(NvBool IsWarmBoot,
                        NvU32 BootRomExitBranchTargetBpmp,
                        NvU32 BootRomExitOptionsBitmap)
{
    NvU32 SecureRegData;
    // Clear out "RAM" region in linker script.
    uint32_t StartClearArea = (NvU32)(&__data_start);
    uint32_t ClearAreaEnd = (NvU32)(&__stack_top);

    // Check if we encountered an unrecoverable problem.  If so, then reset
    // the chip and start over.
    if (BootRomExitOptionsBitmap & NVBOOT_SECURE_EXIT_OPTION_FULLRESET) {
        NvBootResetFullChip();
    }

    // disable access to SBK and DK fuses, depending on operating mode
    //
    // Operating Mode        SBK/DK access
    // ------------------    -------------
    // Preproduction Mode*       read
    // Failure Analysis*         none
    // Nv Production Mode        read
    // ODM Non-secure            none
    // ODM Secure                none
    //
    // * Processing for these operating modes is not performed here.  The FA
    //   Mode functionality is handled by hardware directly (the FA Fuse
    //   permanently disables access to SBK and DK).  The Preproduction Mode
    //   functionality is performed in the non-secure section of the Boot ROM
    //   code.
    //
    // Note: Read access to the SBK and DK fuses is left enabled in NV
    //       Production Mode so that, as a risk mitigation, we can burn and
    //       verify these fuses from a bootloader or Recovery Mode applet (i.e.,
    //       from outside the Boot ROM) so long as the chip is in NV Production
    //       Mode.  In reality, the ability the burn fuses in this mode has
    //       always existed; it's the ability to read back the fuse values and
    //       check whether the desired settings have been achieved that is
    //       enabled here.

    if ( ! NvBootFuseIsNvProductionMode() ) {
        NvBootFuseHideKeys();
    }

    // If we are in NvProductionMode/Secure provisioning mode, hide the SBK
    // if the KEY_HIDE fuse has already been burned.
    // This is to protect against an attacker from disconnecting the device
    // after the SBK has been burned but before the SECURITY_MODE fuse
    // has been burned. See the T210_Tegra_Security_GFD.docx for more details.
    if ( NvBootFuseIsNvProductionMode() && NvBootFuseIsSecureProvisionKeyHideFuseBurned() ) {
        NvBootFuseHideKeys();
    }

    // Zero out the BCT for WB0 and forced Recovery boot types,
    if (BootRomExitOptionsBitmap & NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT) {
        NvBootUtilMemset( (void*) &__bct_start, 0, sizeof(NvBootConfigTable) );
    }

    //lock down / clear keyslots
    //only need to happen in cold boot/rcm
    TODO
    (void)IsWarmBoot;
    // Fix when SE simulation model is available.
    //LoadLockCryptoKeys(IsWarmBoot);

    SecureRegData = NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0);
    SecureRegData = NV_FLD_SET_DRF_DEF(SB,
    				CSR,
    				PIROM_DISABLE,
    				DISABLE,
    				SecureRegData);
    SecureRegData = NV_FLD_SET_DRF_DEF(SB,
    				CSR,
    				SECURE_BOOT_FLAG,
    				DISABLE,
    				SecureRegData);

    //BootROM patch clean-up
    NvBootIRomPatchCleanup();

    BootInfoTable.BootTimeLog.NvBootTimeLogExit = SecureRegData;

    // Update Boot ROM version.
    BootInfoTable.BootRomVersion = CONST_NVBOOT_BOOTROM_VERSION;

#ifndef TARGET_ASIM // WDT timer is not supported by ASIM yet
    //Reload watchdog counter before handing off to bootloader
    NvBootWdtReload(WDT_TIMEOUT_VAL_NONRCM);
#endif

    // Final BIT logger
    BootInfoTable.BootROMtracker = NvBootFlowStatus_SecureBootExit;
    // NOTE: this routine should never return
    NvBootBpmpSecureExit(BootRomExitBranchTargetBpmp,
                            (NvU32) (StartClearArea),
                            (ClearAreaEnd),
                            (NvU32) &(BootInfoTable.BootTimeLog.NvBootTimeLogExit)) ;

    // if we got here, something went terribly wrong ... reset the chip
    NvBootResetFullChip();
}

NvBootError NvBootBpmpSecureExitStart()
{
    NvBool IsWarmBoot = Context.BootFlowStatus.NvBootFlowSc7Exit;// updated in warmboot entry - Javed,,
    NvU32 BootRomExitBranchTargetAvp = (NvU32)(Context.BootLoader);
    NvU32 BootRomExitOptionsBitmap = 0;

    // if any kind of error occurred and we weren't able to fix it before we
    // got here, then there's no way to recover; reset the chip and try again
    if ( Context.BootFlowStatus.NvBootFlowChipStatus ) {
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_FULLRESET;
    }

    // The Boot Configuration Table (BCT) and Boot Information Table (BIT)
    // data is left undisturbed in memory after the Boot ROM exits, under
    // some circumstances as defined below --
    //
    // Boot Type                   BIT             BCT
    // ----------------------      ------          ------
    // Pre-producion*              valid           valid
    // Failure Analysis*           valid           zeroed
    // Warm Boot 0                 valid           zeroed
    // Cold Boot                   valid           valid
    // Forced Recovery Mode        valid           zeroed
    // Unforced Recovery Mode      valid           present, valid or invalid
    //
    // * Processing for these boot types is not performed here; it's performed
    //   in the non-secure section of the Boot ROM code
    //
    // Note: an Unforced Recovery occurs when an error prevents a Cold Boot
    //       from succeeding.  In this case, it is not possible to guarantee
    //       that the BCT is valid, however it is left in memory intact in
    //       case it will be useful in identifying why the Cold Boot failed.

    if (Context.BootFlowStatus.NvBootFlowXusbForceRcm )
    {
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT;
    }

    if ( IsWarmBoot ) // updated in warmboot entry - Javed,,
    {
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT;
    }

    // Flush .CryptoBuffer section in IRAM.
    NvBootUtilMemset((uint32_t *) &__crypto_buffer_start, 0, (uint32_t)&__crypto_buffer_end - (uint32_t)&__crypto_buffer_start);

    // Clear SE TZRAM for coldboot only.
    if(BootInfoTable.BootType == NvBootType_Cold)
    {
        NvBootSeClearTzram();
    }

    FI_counter1 = 0;
    // Re-run the SE housekeeping function out of an abundance of caution.
    // It should have been run already in the NvBootTaskListId_SecureExit task table.
    NvBootSeHousekeepingBeforeBRExit();
    FI_counter1 -= SE_HOUSEKEEPING_STEPS*COUNTER1;
    // If the increment count is not what we expect, some instruction skipping
    // might have happened. Reset the chip.
    if(FI_counter1 != 0) do_exception();

    // flush all secure information in preparation for exiting Secure Section
    // of boot ROM --
    // 1. disable access to SBK and DK fuses ??
    // 2. disable read access to SSK always-on register
    // 3. flush SBK from SE key slot used for AES CMAC hashing ??
    // Describe exit conditions -- Anthony.
    // 4. over-write the Sysram(iRAM) area containing Boot ROM variables and code
    // 5. Enable/Disable JTAG
    //
    // exit Secure Section of iROM, disable Secure Section of iROM, then set
    // up chip according to desired exit criteria
    // NOTE: this routine should never return

    NvBootMainSecureRomExit(IsWarmBoot,
    						BootRomExitBranchTargetAvp,
    						BootRomExitOptionsBitmap);

    // this should never reach here..
    // if we got here, something went terribly wrong ... reset the chip
    NvBootResetFullChip();

    return NvBootError_HwTimeOut;
}

#if 0
//Stuff for Power Ungating
//Used in xusb, uphy, xusb drivers only slightly modified. We should move it to util
static NvU32 PollField200us(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue)
{
    NvU32 Timeout = 200;
    NvU32 RegData;
    do {
        RegData = NV_READ32(RegAddr);
        if((RegData & Mask) == ExpectedValue)
            return NvBootError_Success;
        NvBootUtilWaitUS(10);
        Timeout-=10;
    } while(Timeout);
    return NvBootError_HwTimeOut;
}
#endif

#if 0
//Abstract out common unpowergating to clear power control bits
#define POWER_UNGATE(dev) \
do { \
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_##dev##_POWER_GATE_CONTROL_0); \
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_##dev##_POWER_GATE_CONTROL, LOGIC_SLEEP, OFF, RegData); \
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_##dev##_POWER_GATE_CONTROL, SRAM_SD, OFF, RegData); \
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_##dev##_POWER_GATE_CONTROL, SRAM_SLP, OFF, RegData); \
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_##dev##_POWER_GATE_CONTROL, SRAM_DSLP, OFF, RegData); \
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_##dev##_POWER_GATE_CONTROL, INTER_PART_DELAY_EN, ENABLE, RegData); \
    RegData = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_##dev##_POWER_GATE_CONTROL, START, PENDING, RegData); \
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_##dev##_POWER_GATE_CONTROL_0, RegData); \
} while(0)
#endif

#if 0
NvBootError NvBootBpmpUnPGL2Rst()
{
    NvBootError e = NvBootError_Success;
    NvU32 RegData;

    // Nothing to do if this is not a L2 reset
    if (!NvBootQueryRstStatusWDTFourthExpiryIdleSc0Flag())
        return NvBootError_Success;
    /*
    An ungraceful L2 reset can leave the Power Control Registers
    in a non-optimal state, for eg, the boot device partitions may be PG.
    BR assumes all partitions are unPG on cold and L1 reset.
    For L2 reset, the unPGing needs to be specifically attempted.
    To reduce code complexity, all boot device partitions are being powered up
    without checking if the partition was PG i.e. force powerup
    Since this is debug path, the added power penalty is not a concern
    */
    // Potential of deadlock is XUSB powering up fails. Because USB RCM mode will
    // start to return errors from fabric. This will then enter into Abort Handler
    // and the Abort handler will loop for NV pre-production.
    // For NV production it will hit the MAIN_RST after a large delay.

    // Step 1 of Sequence 9.8 : UnPG XUSB A/B/C, Sata, UFS
    // UnPowergate XUSB A
    POWER_UNGATE(XUSBA);
    // Poll for Start = 0
    PollField200us(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_XUSBA_POWER_GATE_CONTROL_0,
                   PMC_IMPL_PART_XUSBA_POWER_GATE_CONTROL_0_START_FIELD,
                   0);
    // Removed Padam's error checking here. Couldn't come up with a point in checking errors
    // also removed from later sections at same location
    // Remove power clamp. Only bit 0 present so no need of RMW
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_XUSBA_CLAMP_CONTROL_0,
               PMC_IMPL_PART_XUSBA_CLAMP_CONTROL_0_CLAMP_OFF);
    NvBootUtilWaitUS(1); // Requirement is to wait for 200ns, but 1us is closest available.

    // UnPowergate XUSB B
    POWER_UNGATE(XUSBB);
    // Poll for Start = 0
    PollField200us(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_XUSBB_POWER_GATE_CONTROL_0,
                   PMC_IMPL_PART_XUSBB_POWER_GATE_CONTROL_0_START_FIELD,
                   0);

    // Remove power clamp. Only bit 0 present so no need of RMW
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_XUSBB_CLAMP_CONTROL_0,
               PMC_IMPL_PART_XUSBB_CLAMP_CONTROL_0_CLAMP_OFF);
    NvBootUtilWaitUS(1); // Requirement is to wait for 200ns,closest available.

    // UnPowergate XUSB C
    POWER_UNGATE(XUSBC);
    // Poll for Start = 0
    PollField200us(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_XUSBC_POWER_GATE_CONTROL_0,
                   PMC_IMPL_PART_XUSBC_POWER_GATE_CONTROL_0_START_FIELD,
                   0);

    // Remove power clamp. Only bit 0 present so no need of RMW
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_XUSBC_CLAMP_CONTROL_0,
               PMC_IMPL_PART_XUSBC_CLAMP_CONTROL_0_CLAMP_OFF);
    NvBootUtilWaitUS(1); // Requirement is to wait for 200ns, this is closest available.

    // UnPowergate Sata
    POWER_UNGATE(SAX);
    // Poll for Start = 0
    PollField200us(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_SAX_POWER_GATE_CONTROL_0,
                   PMC_IMPL_PART_SAX_POWER_GATE_CONTROL_0_START_FIELD,
                   0);

    // Remove power clamp. Only bit 0 present so no need of RMW
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_SAX_CLAMP_CONTROL_0,
               PMC_IMPL_PART_SAX_CLAMP_CONTROL_0_CLAMP_OFF);
    NvBootUtilWaitUS(1); // Requirement is to wait for 200ns, closest available.

    // Indicate Sata is out of PG
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_MISC_BASE + PMC_MISC_SATA_PWRGT_0);
    // Arun to double check - Default =0.
    RegData = NV_FLD_SET_DRF_DEF(PMC_MISC, SATA_PWRGT, PG_INFO, DEFAULT, RegData);

    // UnPowergate UFS
    POWER_UNGATE(UFS);
    // Poll for Start = 0
    PollField200us(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_UFS_POWER_GATE_CONTROL_0,
                   PMC_IMPL_PART_UFS_POWER_GATE_CONTROL_0_START_FIELD,
                   0);

    // Remove power clamp. Only bit 0 present so no need of RMW
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_PART_UFS_CLAMP_CONTROL_0,
               PMC_IMPL_PART_UFS_CLAMP_CONTROL_0_CLAMP_OFF);
    NvBootUtilWaitUS(1); // Requirement is to wait for 200ns, this is closest available.

    // Step 2 of Sequence 9.8 : 2.  BR writes to IO_DPD_REQ
    // Only need to write to IO_DPD_RE and IO_DPD2_REQ. Write DPD  ON followed by OFF
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_IO_DPD_REQ_0, 0x7FFFFFFF);
    NV_WRITE32(NV_ADDRESS_MAP_PMC_IMPL_BASE + PMC_IMPL_IO_DPD2_REQ_0, 0x7FFFFFFF);

    return e; // No error if we reach here
}
#endif

void NvBootBpmpSecureExit(NvU32 BootloaderEntryAddress,
	NvU32 StartClearAddress,
	NvU32 StopClearAddress,
	NvU32 SecureRegisterValueAddr)
{
#ifdef DO_COVERAGE
#if DO_COVERAGE
    void codecov_dump(void);
    codecov_dump();
#endif
#endif
    NvBootMainAsmSecureExit(BootloaderEntryAddress,
                            StartClearAddress,
                            StopClearAddress,
                            SecureRegisterValueAddr);
}

/**
 *  Minimal secure exit that can be achieved during early boot for errors encountered in non-secure
 *  region code.
 */
void FT_NONSECURE NvBootMinimalAssetLockDownExit()
{
    NvU32 RegData;
    NvU32 SBCSRMask = NV_DRF_DEF(SB, CSR, PIROM_DISABLE, DISABLE) | \
                      NV_DRF_DEF(SB, CSR, SECURE_BOOT_FLAG, DISABLE);
    NvU32 FekRdDisMask = NV_DRF_NUM(APB_MISC, PP_FEK_RD_DIS, FEK_RD_DIS,1);

    // Set Secure Boot flag
    do
    {
        // Do once
        RegData = NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0);
        RegData = NV_FLD_SET_DRF_DEF(SB,
                                     CSR,
                                     PIROM_DISABLE,
                                     DISABLE,
                                     RegData);
        RegData = NV_FLD_SET_DRF_DEF(SB,
                                     CSR,
                                     SECURE_BOOT_FLAG,
                                     DISABLE,
                                     RegData);
        NV_WRITE32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0, RegData);

        // Do twice
        NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0);

        NV_WRITE32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0, RegData);

        RegData = NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0);
    } while((RegData &  SBCSRMask) != SBCSRMask);

    // Ensure FEK is read locked.
    do
    {
        // Do once
        RegData = NV_DRF_NUM(APB_MISC, PP_FEK_RD_DIS, FEK_RD_DIS, 1);
        NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_FEK_RD_DIS_0, RegData);

        // Do twice
        NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_FEK_RD_DIS_0);
        NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_FEK_RD_DIS_0, RegData);
        // RegData = NV_DRF_NUM(APB_MISC, PP_FEK_RD_DIS, FEK_RD_DIS, 1);

        RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_PP_FEK_RD_DIS_0);
    }
    while((RegData & FekRdDisMask) != FekRdDisMask);
}
