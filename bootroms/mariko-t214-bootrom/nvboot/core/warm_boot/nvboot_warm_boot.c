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
 * nvboot_warmboot0.c - Implementation of WarmBoot0 functions.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arahb_arbc.h"
#include "arapb_misc.h"
#include "arclk_rst.h"
#include "aremc.h"
#include "armc.h"
#include "arrtc.h"
#include "arse.h"
#include "artimerus.h"
#include "nvboot_clocks_int.h"
#include "nvboot_config_int.h"
#include "nvboot_context_int.h"
#include "nvboot_error.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_reset_int.h"
#include "nvboot_sdram_int.h"
#include "nvboot_sdram_param.h"
//#include "nvboot_se_aes.h"
#include "nvboot_se_int.h"
//#include "nvboot_se_rsa.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_crypto_param.h"
#include "nvboot_address_int.h"
#include "nvboot_util_int.h"
#include "nvboot_warm_boot_0.h"
#include "nvboot_warm_boot_0_int.h"
#include "nvboot_platform_int.h"
#include "nvboot_rng_int.h"
#include "nvboot_bpmp_int.h"
#include "nvboot_bct.h"
#include "project.h"

/**
 * SCRATCHNAME - Compose the scratch field name
 *
 *   @param n scratch register number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 */
#define SCRATCHNAME(n, d, r, f) \
    APBDEV_PMC_SCRATCH##n##_0_##d##_##r##_0_##f##_RANGE

/**
 * PMC_TO_REG -  Set the register field from the pmc register field.
 *
 *   @param n scratch register number
 *   @param d register domain (hardware block)
 *   @param r register name
 *   @param f register field
 *   @param e element of SDRAM parameter structure
 *
 * 1. Extract field value from given source register value
 *       t = NV_DRF_VAL(PMC, SCRATCHn, d_r_0_f, RegVal);
 * 2. Set the specified field in the destination register
 *       pData->e = NV_FLD_SET_DRF_NUM(d, r, f, t, pData->e);
 */
#define PMC_TO_REG(n, d, r, f, e)                                           \
    do {                                                                    \
       pData->e = NV_FLD_SET_DRF_NUM(d, r, f,                               \
                                     NV_DRF_VAL(APBDEV_PMC, SCRATCH##n,     \
                                                d##_##r##_0_##f, RegVal),   \
                                     pData->e);                             \
    } while (0)

/* Shorthand for several sets of parameters, all build on PMC_TO_REG(). */

#define PMC_TO_FBIO(n, r, f, e) PMC_TO_REG(n, EMC, FBIO_##r, f, EmcFbio##e)

#define PMC_TO_EMC(n, r, f, e) PMC_TO_REG(n, EMC, r, f, Emc##e)

#define PMC_TO_MC(n, r, f, e)  PMC_TO_REG(n, MC,  r, f, Mc##e)

#define PMC_TO_PAD(n, rp, fs, e) \
    PMC_TO_REG(n, APB_MISC_GP, rp##PADCTRL, CFG2TMC_##rp##fs,  \
               ApbMiscGp##e##PadCtrl)

#define PMC_TO_PAD2(n, rp, fs, e) \
    PMC_TO_REG(n, APB_MISC_GP, rp##PADCTRL2, CFG2TMC_##rp##fs, \
               ApbMiscGp##e##PadCtrl2)

#define PMC_TO_PLL(n, p, f, e) \
    pData->Pll##p##e = NV_DRF_VAL(APBDEV_PMC, SCRATCH##n,      \
                                  CLK_RST_PLL##p##_##f, RegVal)

#define PMC_TO_TYPE(n, f, t, e) \
    pData->e = (t)NV_DRF_VAL(APBDEV_PMC, SCRATCH##n, f, RegVal)

#define PMC_TO_VAL(n, f, e) \
    pData->e = NV_DRF_VAL(APBDEV_PMC, SCRATCH##n, f, RegVal)

extern NvBootContext Context;

extern int32_t FI_counter1;

//structure holds data needed for MC/EMC initialization.
extern NvBootConfigTable BootConfigTable;
static NvBootSdramParams *s_sdRamParamData = (NvBootSdramParams*)(&(BootConfigTable.SdramParams[0]));
/*
 * Compile-time assertion that the header size is a multiple of 16 bytes.
 */
//NV_CT_ASSERT((sizeof(NvBootWb0RecoveryHeader) & 0xf) == 0);

/*
 * This function reads the recovery code header and firmware
 * into IRAM-B.
 */
NvBootError
NvBootWb0CopyHeaderAndFirmware()
{
    // FI counter that is incremented after every critical step in the function.
    FI_counter1 = 0;

    // Read the recovery code header start and the header
    uint32_t Sc7FwSaveAddress = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SECURE_SCRATCH119_0);

    // Check if the FW header and binary was saved in a valid location.
    NvBootError e_Tzram_Source = NvBootValidateAddress(Sc7FwTzramRange, Sc7FwSaveAddress, 1);
    FI_counter1 += COUNTER1;
    NvBootError e_Sdram_Source = NvBootValidateAddress(DramRange, Sc7FwSaveAddress, 1);
    FI_counter1 += COUNTER1;

    // If all of the range checks above fail, then return error and don't copy.
    if ((e_Tzram_Source != NvBootError_Success) && (e_Sdram_Source != NvBootError_Success))
        return NvBootError_Invalid_SC7_FW_Save_Address;

    if (Sc7FwSaveAddress & 0xF)
    {
        return NvBootError_MemoryNotAligned;
    }

    // Copy header.
    NvBootUtilMemcpy((void*) NVBOOT_SC7_FW_START,
                     (void*)(Sc7FwSaveAddress),
                     sizeof(NvBootWb0RecoveryHeader));
    FI_counter1 += COUNTER1;

    // Get length of SC7 firmware binary in header, and sanitize it. The size
    // shouldn't be larger than the size of IRAM-B, C, and D (64KB * 3 = 196KB) minus the
    // header size. The size of LengthInsecure must be at least sizeof(NvBootWb0RecoveryHeader) + 4.
    NvBootWb0RecoveryHeader *pSc7Header = (NvBootWb0RecoveryHeader *) NVBOOT_SC7_FW_START;

    // Before subtracting sizeof(NvBootWb0RecoveryHeader) from LengthInsecure, check to see if it
    // is at least sizeof(NvBootWb0RecoveryHeader) + 4 (i.e. recovery code must be at least 4
    // bytes, sufficient to do an ARM 0xEAFFFFFE branch to self).
    if(pSc7Header->LengthInsecure < (sizeof(NvBootWb0RecoveryHeader) + 4))
        return NvBootError_SC7_FW_Size_Too_Small;
    FI_counter1 += COUNTER1;

    // RecoveryCodeLength is the unpadded size of SC7 firmware. The SC7 firmware needs to be
    // padded to a multiple AES block length to be encrypted, and then it is signed.
    // We use LengthInsecure here because this is the total size of the SC7 firmware
    // including padding and header.
    uint32_t SizeToCopy = pSc7Header->LengthInsecure - sizeof(NvBootWb0RecoveryHeader);
    NvBootError e_SC7_FW_Target = NvBootValidateAddress(Sc7FwRamRange, (NVBOOT_SC7_FW_START + sizeof(NvBootWb0RecoveryHeader)), SizeToCopy);
    FI_counter1 += COUNTER1;

    if(e_SC7_FW_Target != NvBootError_Success)
        return NvBootError_SC7_FW_Size_Too_Large;

    // Add random delay before copy to mitigate against FI attack against
    // address/size.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    // Copy SC7 firmware.
    NvBootUtilMemcpy((void*) NVBOOT_SC7_FW_START+sizeof(NvBootWb0RecoveryHeader),
                     (void*)(Sc7FwSaveAddress+sizeof(NvBootWb0RecoveryHeader)),
                     SizeToCopy);
    FI_counter1 += COUNTER1;

    // Check if function counter is the expected value. If not, some instruction skipping might
    // have occurred.
    if(FI_counter1 != COUNTER1 * NvBootWb0CopyHeaderAndFirmware_COUNTER_STEPS)
        do_exception();

    // Decrement function counter.
    FI_counter1 -= COUNTER1 * NvBootWb0CopyHeaderAndFirmware_COUNTER_STEPS;

    // Add random delay to mitigate against temporal glitching.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    // Re-check counter.
    if(FI_counter1 != 0)
        do_exception();

    return NvBootError_Success;
}

NvBootError NvBootHaltAtWarmboot()
{
    NvU32 HaltAtWb0;
    HaltAtWb0 = NvBootPmcQueryFlag(NvBootPmcFlagId_HaltAtWb0);
    if(HaltAtWb0 && NvBootFuseIsPreproductionMode())
    {
        // Clear the HaltAtWb0 flag.
        NvBootPmcSetFlag(NvBootPmcFlagId_HaltAtWb0, NV_FALSE);
        while(HaltAtWb0);
    }
    return NvBootError_Success;
}

/**
    HCLK is running at 102mhz for SC7 since BR sets
    CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0_SWAKEUP_RUN_SOURCE = PLLP_OUT2,
    and CLK_RST_CONTROLLER_CLK_SOURCE_SYS_0_SYS_CLK_DIVISOR = 2 for SC7.
    Per JerryZ, the TZRAM restoration time is calculated as:
    4 bytes per cycle by the copy engine, 256KB TZRAM is 262,114 bytes.
    Number of cycles to copy is 262,114 / 4 = 65,536 cycles.
    Given HCLK = 102Mhz, and assuming 3x 65,536 cycles to be the timeout value:
    65536*3/102000000 = 0.001927 seconds = 1927 microseconds = 1.927 milliseconds.
*/
#define NVBOOT_TZRAM_RESTORE_TIMEOUT_US 1927
/**
    After profiling this function on the T214 FPGA system, this function requires
    around 5052 us to complete the copy. Setting the timeout for FPGA at 10000us to
    give some margin.
*/
#define NVBOOT_TZRAM_RESTORE_TIMEOUT_US_FPGA 10000

NvBootError NvBootWb0TzramInit(void)
{
    // Read AON Shadow TZRAM powergating control.
    uint32_t RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_TZRAM_PWR_CNTRL_0);
    RegData = NV_DRF_VAL(APBDEV_PMC, TZRAM_PWR_CNTRL, TZRAM_SD, RegData);
    // If powergating control is 0, i.e. not powergated, trigger the A0 TZRAM
    // restore sequence and wait for the restore to complete.

    // Set timeout based on platform.
    uint32_t Timeout = 0;
    if(NvBootIsPlatformFpga())
        Timeout = NVBOOT_TZRAM_RESTORE_TIMEOUT_US_FPGA;
    else
        Timeout = NVBOOT_TZRAM_RESTORE_TIMEOUT_US;

    if(RegData == 0)
    {
        uint32_t TzramReg = 0;
        TzramReg |= NV_DRF_DEF(SE, TZRAM_OPERATION, MODE, RESTORE);
        TzramReg |= NV_DRF_DEF(SE, TZRAM_OPERATION, REQ, INITIATE);
        NV_WRITE32(NV_ADDRESS_MAP_SE_BASE + SE_TZRAM_OPERATION_0, TzramReg);

        uint32_t Busy = SE_TZRAM_OPERATION_0_BUSY_YES;
        uint32_t StartTime = NvBootUtilGetTimeUS();
        uint32_t ElapsedTime = 0;
        while(Busy == SE_TZRAM_OPERATION_0_BUSY_YES)
        {
            Busy = NV_READ32(NV_ADDRESS_MAP_SE_BASE + SE_TZRAM_OPERATION_0);
            Busy = NV_DRF_VAL(SE, TZRAM_OPERATION, BUSY, Busy);
            ElapsedTime = NvBootUtilElapsedTimeUS(StartTime);
            if(ElapsedTime > Timeout)
                return NvBootError_Sc7TzramRestoreTimeout;
        }
    }
    return NvBootError_Success;
}

NvBootError NvBootWarmBootUnPackSdramStartPllm()
{
    NvU32 M, N, P, Misc1, Misc2;
    NvU32 RegVal;
    NvU32 pllXStabilizationDelay;
    NvU32 pllMStabilizationDelay;
    NvU32 pllXStableTime;
    NvU32 pllMStableTime;
    NvU32 pllXEnable = 0;
    NvBootError e;

    NV_BOOT_CHECK_ERROR(NvBootWb0UnpackSdramParams(s_sdRamParamData));

    // Enable Mem clk per Boot GFD. Reference section 1.6, 1.11.
    // Preserve DRAM = true for SC7.
    // If PreserveDRAM is true, the following function can only be called after 
    // unpacking SDRAM parameters.
    NvBootEnableMemClk (s_sdRamParamData, NV_TRUE);
    
    // Only start PLLM if required
    RegVal = NV_DRF_VAL(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_EMC,
                                 EMC_2X_CLK_SRC,
                                 s_sdRamParamData->EmcClockSource);
    
    if((RegVal == CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_UD)   ||
       (RegVal == CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0_EMC_2X_CLK_SRC_PLLM_OUT0))
    {
    
    // PLLM KVCO, KCP etc.
    Misc2 = NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KVCO, \
                    s_sdRamParamData->PllMKVCO) | \
            NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLM_MISC2, PLLM_KCP, \
                    s_sdRamParamData->PllMKCP);

    Misc1 = NV_DRF_NUM(MISC1, CLK_RST_CONTROLLER_PLLM_MISC1, PLLM_SETUP, \
                    s_sdRamParamData->PllMSetupControl);

        pllMStabilizationDelay = s_sdRamParamData->PllMStableTime;

    // If PLLM auto-restart is enabled, skip the starting of PLLM.
    if(!(NvBootPmcIsPllmOverrideEnabled()))
    {
        // Start PLLM for EMC/MC
            NvBootClocksStartPll(NvBootClocksPllId_PllM, s_sdRamParamData->PllMInputDivider,
                                                         s_sdRamParamData->PllMFeedbackDivider,
                                                         s_sdRamParamData->PllMPostDivider, Misc1, Misc2, \
                             &pllMStableTime);
    }
    else
    {
        // This function *currently* does nothing as PLLM does not have external dividers.
        // 0 - Reset Enable, 1 - Reset Disable
        NvBootClocksPllDivRstCtrl(NvBootClocksPllId_PllM, 0);
    }

    // Overwrite the stable time returned by clocks API to use the supplied
    // delay.
    pllMStableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE +
                               TIMERUS_CNTR_1US_0) +
        pllMStabilizationDelay;
        // Poll for PLLM lock bit and timeout on polling loop
        while (!(NvBootClocksIsPllStable(NvBootClocksPllId_PllM, pllMStableTime)));
    }

    // start PLLX using data in PMC
    // extract the PLLX data from PMC
    RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH3_0);

    pllXEnable = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_ENABLE, RegVal);
    if(pllXEnable)
    {
        RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH4_0);
        pllXStabilizationDelay = NV_DRF_VAL(APBDEV_PMC, SCRATCH4, PLLX_STABLE_TIME,
                                       RegVal);
    
        M = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVM, RegVal);
        N = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVN, RegVal);
        P = NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVP, RegVal);

        Misc2 = NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLX_MISC_3, PLLX_KVCO, \
                    NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KVCO, RegVal)) | \
                NV_DRF_NUM(MISC2, CLK_RST_CONTROLLER_PLLX_MISC_3, PLLX_KCP, \
                    NV_DRF_VAL(APBDEV_PMC, SCRATCH3, CLK_RST_CONTROLLER_PLLX_MISC_3_0_PLLX_KCP, RegVal));

        RegVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH36_0);
        Misc1 = NV_DRF_NUM(MISC1, CLK_RST_CONTROLLER_PLLX_MISC_1, PLLX_SETUP, \
                    NV_DRF_VAL(APBDEV_PMC, SCRATCH36, CLK_RST_CONTROLLER_PLLX_MISC_1_0_PLLX_SETUP, RegVal));

        // Start PLLX and record the default PLL start time in pllXStableTime.
        NvBootClocksStartPll(NvBootClocksPllId_PllX, M, N, P, Misc1, Misc2, \
                             &pllXStableTime);

        // Overwrite the stable time returned by clocks API to use the supplied
        // delay.
        pllXStableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE +
                                   TIMERUS_CNTR_1US_0) +
                                   pllXStabilizationDelay;

    }


    return NvBootError_Success;
}

NvBootError
NvBootWb0Start()
{
    // Clear the warm boot flag so that if the warm boot fails, the iROM
    // will follow the cold boot path on the next reboot.
    NvBootPmcSetFlag(NvBootPmcFlagId_Wb0, NV_FALSE);

    return NvBootError_Success;
}

NvBootError
NvBootWarmBootSdramInit()
{
    NvBootSdramInitWarmBoot0(s_sdRamParamData);
    return NvBootError_Success;
}

NvBootError NvBootWarmBootOemProcessRecoveryCode()
{
    // FI counter that is incremented after every critical step in the function.
    FI_counter1 = 0;

    // Setup a pointer to the OEM header and the Pcp.
    NvBootWb0RecoveryHeader *OemSc7Header = (NvBootWb0RecoveryHeader *) NVBOOT_SC7_FW_START;
    NvBootPublicCryptoParameters *Pcp = &OemSc7Header->Pcp;

    // Load OEM Pcp if necessary.
    volatile NvBootError e = NvBootInitializeNvBootError();
    e = NvBootCryptoMgrSetOemPcp(Pcp);
    FI_counter1 += COUNTER1;

    if(e != NvBootError_Success &&
       e != NvBootError_CryptoMgr_Pcp_Not_Loaded_Not_PK_Mode)
    {
        return NvBootError_CryptoMgr_Pcp_Hash_Mismatch;
    }

    // Initialize e to non success value.
    e = NvBootInitializeNvBootError();
    volatile NvBootError e_auth_result = NvBootInitializeNvBootError();

    // Authenticate SC7 payload.
    // If the return value in r0 can be attacked, then the mitigations below may
    // not be effective. However, the NvBootCryptoRsaSsaPssVerify function
    // adds a random delay before returning, which should make it harder to attack r0.
    // 108a1c:	f7fb f882 	bl	103b24 <NvBootCryptoMgrOemAuthSc7Fw>
    // 108a20:	9000      	str	r0, [sp, #0] <-- if you can change r0 somehow
    //                                           then mitigation is not useful.
    e_auth_result = NvBootCryptoMgrOemAuthSc7Fw(OemSc7Header);
    if(e_auth_result != NvBootError_Success)
    {
        /// Introduces code distance between error detection and response.
        FI_ADD_DISTANCE_STEP(2);
        return e_auth_result;
    }
    FI_counter1 += COUNTER1;

    // Random delay before re-checking e. Should never return an error. No response sent
    // if so.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    // FI mitigation, double check e_auth_result. It must be success.
    if(e_auth_result != NvBootError_Success)
    {
        /// Introduces code distance between error detection and response.
        FI_ADD_DISTANCE_STEP(2);
        return e_auth_result;
    }
    FI_counter1 += COUNTER1;

    // Re-initialize e to a non success value.
    e = NvBootInitializeNvBootError();
    NV_BOOT_CHECK_ERROR(NvBootCryptoMgrOemDecryptSc7Fw(OemSc7Header));
    FI_counter1 += COUNTER1;

    // Check if Length in the non signed portion equals the signed portion
    if (OemSc7Header->LengthInsecure != OemSc7Header->LengthSecure)
    {
        return NvBootError_SC7_Insecure_To_Secure_Size_Mismatch;
    }
    FI_counter1 += COUNTER1;

    if (OemSc7Header->RecoveryCodeLength >
        (OemSc7Header->LengthSecure - sizeof(NvBootWb0RecoveryHeader)))
    {
        return NvBootError_ValidationFailure;
    }
    FI_counter1 += COUNTER1;

    // Set entry point to fixed location.
    Context.BootLoader = (uint8_t *) (NVBOOT_SC7_FW_START + sizeof(NvBootWb0RecoveryHeader));
    FI_counter1 += COUNTER1;

    // Check if function counter is the expected value. If not, some instruction skipping might
    // have occurred.
    if(FI_counter1 != COUNTER1 * NvBootWarmBootOemProcessRecoveryCode_COUNTER_STEPS)
    {
        /// Introduces code distance between error detection and response.
        FI_ADD_DISTANCE_STEP(2);
        return NvBootError_Fault_Injection_Detection;
    }

    // Decrement function counter.
    FI_counter1 -= COUNTER1 * NvBootWarmBootOemProcessRecoveryCode_COUNTER_STEPS;

    // Add random delay to mitigate against temporal glitching.
    NvBootRngWaitRandomLoop(INSTRUCTION_DELAY_ENTROPY_BITS);

    // Re-check counter.
    if(FI_counter1 != 0)
    {
        /// Introduces code distance between error detection and response.
        FI_ADD_DISTANCE_STEP(2);
        return NvBootError_Fault_Injection_Detection;
    }

    return (e|e_auth_result);
}

