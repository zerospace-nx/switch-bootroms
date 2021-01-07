/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "address_map_new.h"
#include "nvrm_drf.h"

#include "nvboot_bit.h"
#include "nvboot_error.h"

#include "nvboot_buffers_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_config_int.h"
#include "nvboot_device_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_hardware_access_int.h"

#include "nvboot_sata_int.h"
#include "nvboot_sata_context.h"
#include "nvboot_sata_local.h"
#include "nvboot_sata_uphy.h"
#include "nvboot_sata_ahci_struct.h"

#include "arpadctl_PEX_CTL.h"

#include "arsata.h"
// includes from hw/ap/manuals
#include "ardev_t_fpci_sata0.h"
#include "ardev_t_sata.h"
#include "ardev_t_ahci.h"

// Internal SATA context object needed to ensure driver programs controller for
// data transfer as per the mode within the params
NvBootSataContext *SataContext;

extern NvBootInfoTable BootInfoTable;

static NvBootSataParams DefaultParams;
static SataFuseParam FuseData;

static const SataSrcPllDivisors gs_PllDiv[] = {
    // 12 MHz, no refplle
    { .PllrefeDivM = 0, .PllrefeDivN = 0, .PllrefeDivP = 0,
      .PlleDivM = 1, .PlleDivN = 200, .PlleDivPlCml = 24},
    // 38.4 MHz, no refplle
    { .PllrefeDivM = 0, .PllrefeDivN = 0, .PllrefeDivP = 0,
      .PlleDivM = 2, .PlleDivN = 125, .PlleDivPlCml = 24},
    // 12 MHz
    { .PllrefeDivM = 1, .PllrefeDivN = 52, .PllrefeDivP = 2,
      .PlleDivM = 26, .PlleDivN = 100, .PlleDivPlCml = 24},
    // 38.4 MHz
    { .PllrefeDivM = 4, .PllrefeDivN = 65, .PllrefeDivP = 2,
      .PlleDivM = 26, .PlleDivN = 100, .PlleDivPlCml = 24},
#if NVBOOT_TARGET_FPGA
//Temp, 19.2
    { .PllrefeDivM = 0x0, .PllrefeDivN = 0x0, .PllrefeDivP = 0x0,
      .PlleDivM = 1, .PlleDivN = 125, .PlleDivPlCml = 24}
#endif
};

void NvBootSataGetParams(const NvU32 ParamIndex, NvBootSataParams **Params)
{
    NV_ASSERT(Params != NULL);

    /// Specifies the clock source for SATA controller.
    DefaultParams.SataClockSource = NvBootSataClockSource_PllPOut0;

    /// Specifes the clock divider to be used.
    DefaultParams.SataClockDivider = 2;

    /// Specifies the clock source for SATA controller.
    DefaultParams.SataOobClockSource = NvBootSataClockSource_PllPOut0;

    /// Specifes the clock divider to be used.
    DefaultParams.SataOobClockDivider = 0;

    // Read the fuse parameters into FuseData
    FuseData.ComInitWaitIndex = NV_DRF_VAL(SATA, FUSE_PARAM, COMINIT_WAIT_INDEX, ParamIndex);
    FuseData.ComresetRetries = NV_DRF_VAL(SATA, FUSE_PARAM, COMRESET_RETRIES, ParamIndex);
    FuseData.AllowGen2 = NV_DRF_VAL(SATA, FUSE_PARAM, ALLOW_GEN2, ParamIndex);
    FuseData.PageSizeMultLog2 = NV_DRF_VAL(SATA, FUSE_PARAM, PAGE_SIZE_MULT_LOG2, ParamIndex);
    DefaultParams.PlleUsePllRefe = NV_DRF_VAL(SATA, FUSE_PARAM, PLLE_USE_PLLREFE, ParamIndex);
    DefaultParams.SataMode = (NV_DRF_VAL(SATA, FUSE_PARAM, MODE, ParamIndex) ?
                              NvBootSataMode_AhciDma :
                              NvBootSataMode_LegacyPio);

    NvBootClocksOscFreq OscFreq = NvBootClocksGetOscFreq();

    //Default Osc is 38.4
    NvU32 divisorTableIdx = 1;
    if (OscFreq == NvBootClocksOscFreq_12) {
        divisorTableIdx = 0;
    } else if (OscFreq == NvBootClocksOscFreq_38_4) {
        divisorTableIdx = 1;
    }
    // refplle table is second half of table
    divisorTableIdx += DefaultParams.PlleUsePllRefe ? 2 : 0;
#if NVBOOT_TARGET_FPGA
    divisorTableIdx = 4;
#endif
    const SataSrcPllDivisors *pPllDiv = &gs_PllDiv[divisorTableIdx];

    DefaultParams.PlleDivM = pPllDiv->PlleDivM;
    DefaultParams.PlleDivN = pPllDiv->PlleDivN;
    DefaultParams.PlleDivPlCml = pPllDiv->PlleDivPlCml;

    DefaultParams.PllRefeDivM = pPllDiv->PllrefeDivM;
    DefaultParams.PllRefeDivN = pPllDiv->PllrefeDivN;
    DefaultParams.PllRefeDivP = pPllDiv->PllrefeDivP;

    switch (DefaultParams.SataMode)
    {
    // AHCI DMA Mode
    case NvBootSataMode_AhciDma:
        // IRAM_SATA_AHCI_HBAMEM_BASE - Required only for AHCI DMA
        // HBAMem base for SATA - default value. Takes up 1KB 
        // this was 4K aligned in the past, but I think it only NEEDS to be 1KB aligned, l
        DefaultParams.SataBuffersBase = ALIGN_ADDR(NVBOOT_DEV_DS_START, 0x1000);
        break;
    // 0 or not set = PIO mode
    default:
        // Start address for command and FIS structures. - Not used for pio mode
        DefaultParams.SataBuffersBase = 0;
        break;
    }

    // Get the sata parameters for mode and timing.
    *Params = &DefaultParams;
}

NvBool NvBootSataValidateParams(const NvBootSataParams *Params)
{
    NvBool ParamsValid = NV_FALSE;
    NvU32 PageSizeLog2 = SECTOR_SIZE_LOG2;
    // Default : Max page size supported is 32KB for DMA, 16KB for PIO
    NvU32 MaxPageSizeSupportedLog2 = NVBOOT_SATA_BLOCK_SIZE_LOG2;

    NV_ASSERT(Params);

    if (Params != NULL) {
        ParamsValid = (Params->SataClockSource <= NvBootSataClockSource_Num);
        ParamsValid &= (Params->SataOobClockSource <= NvBootSataClockSource_Num);
        ParamsValid &= (Params->SataMode <= NvBootSataMode_Num);
        ParamsValid &= (Params->SataMode > 0);

        /* Supported Page Size Log2 values <= 16K*/
        PageSizeLog2 = SECTOR_SIZE_LOG2 + FuseData.PageSizeMultLog2;
        //UNNECCESARY, remove in next chip
        if (Params->SataMode == NvBootSataMode_LegacyPio) {
            // Max page size supported is changed to 16KB
            MaxPageSizeSupportedLog2 = (NVBOOT_SATA_BLOCK_SIZE_LOG2 - 1);
        }
        ParamsValid &= (PageSizeLog2 <= MaxPageSizeSupportedLog2);
    }
    SET_SATA_BOOT_INFO_BITFLD(InitStatus, (ParamsValid ? SataInitStatus_ParamsValid : NV_FALSE));
    return ParamsValid;
}

void NvBootSataGetBlockSizes(const NvBootSataParams *Params __attribute__((unused)),
                             NvU32 *BlockSizeLog2,
                             NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);

    *BlockSizeLog2 = NVBOOT_SATA_BLOCK_SIZE_LOG2;
    *PageSizeLog2 = SataContext->PageSizeLog2;
}

NvBootError NvBootSataInit(const NvBootSataParams *ParamData,
                           NvBootSataContext *pSataContext)
{
    NvBootError Error = NvBootError_NotInitialized;
    static NvU32 s_AhciPxImplemented;
    static NvU32 s_AhciPxCmdSlotsSupported;

    /* 1. Hold SATA and SATA_OOB in reset.
     * 2.
     *    (a) PLLP_OUT0 is required as CLK Source to SATA controller and SATA OOB.
     *         Sata OOB needs to be fed with 216MHz.
     *    (b) Sata controller needs to be fed with 108MHz(frequency upto which
     *         SATA controller operation is known to be stable). Select source
     *         and divisor accordingly.
     *    (c) Enable clock to SATA controller and SATA OOB.
     * 3. Put SATA PAD PLL in IDDQ mode.
     * 4. Set SATA pad PLL reset overrides.
     * 5. Override iddq for SATA PHY.
     * 6. Select PLLE as input ref clock to IO PHY.
     * 7. Get SATA pad PLL out of IDDQ mode. Wait for slumber delay of 3 us and
            de-assert iddq signal to SATA PHY.
     * 8. Train and Enable PLLE.
     * 9. Release Resets
     *    (a) Release Reset for SATA pad PLL and check if SATA pad PLL is locked.
     *    (b) Release cold reset for SATA controller.
     * 10. Clear sw cntrl for SATA pad PLL, SATA Phy and PLLE.
     * 11. Make sure interrupts are disabled.
     * 12. Enable FPCI accesses in SATA IPFS.
     * 13. Check SATA aux misc cntrl register to ensure two COMRESETs are not
     *       sent on warm reset. - !! Driver doesn't implement this because
     *        regression test doesn't perform this step.
     * 14. Program Configuration Space Registers
     *    (a) Disable native mode
     *    (b) Enable IO space
     *    (c) Enable memory space
     *    (d) Enable ADMA space
     * 15. Configure IO space registers
     *    (a) Program Bars 0/1/5
     *       i.   BAR 0 = 0xFFFFFFFF
     *       ii.  BAR 1 = 0xFFFFFFFF
     *       iii. BAR 5 = 0xFFFFFFFF
     *    (b) Read back all registers to do what????
     *    (c) Program Bars 0/1/5
     *       i.   BAR 0 = 0x000001F0
     *       ii.  BAR 1 = 0x000003F4
     *       iii BAR 5 = 0x40020000
     * 16. Program corresponding BAR register in IPFS with upper 20bits of the
     *       address
     *    (a) FPCI_BAR0_START=0x0 and
     *         FPCI_BAR0_ACCESS_TYPE = NV_SATA_IPFS_IO_TYPE (1).
     *    (b) FPCI_BAR1_START=0x0 and
     *         FPCI_BAR1_ACCESS_TYPE = NV_SATA_IPFS_IO_TYPE (1).
     *    (c) FPCI_BAR5_START=0x40020 and
     *         FPCI_BAR5_ACCESS_TYPE = NV_SATA_IPFS_MEM_TYPE (0).
     * 17. Initialize Extended Configuration Space Registers
     *        i. T_SATA0_CHX_PHY_CTRL1_GEN2_RX_EQ
     * 18. Clear warm reset.
     * 19. Check if device is busy.
     * 20. Issue a COMRESET.
     * 21. After 1ms, write 0b0 to PxSCTL.DET.
     * 22 Poll PxSSTS.DET until the returned data is 0x3.
     * Note 1: There could be certain steps that are required only during first
     *            time initialization.
     * Note 2: Need to distinguish between init paths for legacy PIO and AHCI
     *            DMA wherever required.
     * Note 3: Fuse bits are not being checked right now and alternative fixed
     *            delays have been provided.
     */
    NV_ASSERT(pSataContext);
    NV_ASSERT(ParamData);

    SataContext = pSataContext;

    SataContext->SataMode = ParamData->SataMode;
    SataContext->PageSizeLog2 = SECTOR_SIZE_LOG2 + FuseData.PageSizeMultLog2;
    SataContext->PagesPerBlock = 1 << (NVBOOT_SATA_BLOCK_SIZE_LOG2 - SataContext->PageSizeLog2);
    SataContext->SectorsPerPage = 1 << FuseData.PageSizeMultLog2;

    if (SataContext->SataMode == NvBootSataMode_AhciDma) {
        if (AllocateHBAMemBuffers(ParamData) != NvBootError_Success) {
            // Fallback to legacy pio mode
            SataContext->SataMode = NvBootSataMode_LegacyPio;
        }
    }

    SET_SATA_BOOT_INFO_FLD(AhciSataBuffersBase, SataContext->SataBuffersBase);

    SET_SATA_BOOT_INFO_BITFLD(SataMode, (SataContext->SataMode << 16));

    // Once only Initialization of SATA controller
    // Step 1 This is warm reset.
    NvBootResetSetEnable(NvBootResetDeviceId_SataId, NV_TRUE);

    // There's no signal corresponding to SWR_SATA_OOB_RST. Therefore, don't
    // program SWR_SATA_OOB_RST

    // Cold reset bit for sata init
    NvBootResetSetEnable(NvBootResetDeviceId_SataColdRstId, NV_TRUE);

    // Step 2(a) SATA OOB clock should be 204MHz using PLLP_OUT0 as source
    ConfigureClockSource(NvBootClocksClockId_SataOobId, ParamData);

    //SATA controller clock should be 102MHz using PLLP_OUT0 as source
    // nvboot_clocks not yet tested for Sata clock source programming
    // Step 2(b)
    ConfigureClockSource(NvBootClocksClockId_SataId, ParamData);

    // Step 2(c) Enable Clocks to SATA controller and SATA OOB
    NvBootClocksSetEnable(NvBootClocksClockId_SataId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_SataAuxId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_SataOobId, NV_TRUE);

    if (!SataContext->IsSataInitialized) {
        // Clear Reset for SATA USB UPHY (PAD PLL)
    //SET_SATA_USB_UPHY_RST, CLK_RST_CONTROLLER_RST_DEV_Y_SET_0_SET_SATA_USB_UPHY_RST_ENABLE, RegValue);

        // Step 6 Select internal CML ref clk
        // Select PLLE as input to IO phy
        // Register SATA_AUX_PAD_PLL_CNTL_1_0 is RESERVED in T210. 
        // REFCLK_SEL = SATA_AUX_PAD_PLL_CTRL_0_0[15:12], default 0(INT_CML)

        // Step 8 Enable PLLE - This code should go in nvboot_clocks.h
        Error = EnablePLLE(ParamData);
        if (Error) {
            // Don't return error. Record the error in the BIT.
            SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_PlleInitFailed);
        }
        SataUPhyInit();
    }

    // Release cold reset
    NvBootResetSetEnable(NvBootResetDeviceId_SataColdRstId, NV_FALSE);

    //De-assert warm reset
    // need to de-assert warm reset to access sata controller registers
    NvBootResetSetEnable(NvBootResetDeviceId_SataId, NV_FALSE);

    // There's no signal corresponding to CLR_SATA_OOB_RST. Therefore, don't
    // program CLR_SATA_OOB_RST
    NvBootUtilWaitUS(62);

    // Step 12
    // Enable SATA FPCI accesses in SATA IPFS logic -
    // NV_ADDRESS_MAP_APB_SATA_BASE = NV_SATA_IPFS_REGS
    IPFS_RMW_FLD_NUM(CONFIGURATION, EN_FPCI, 1);

    ProgramSquelch();

    // Step 14 Enable the IO, MEM and ADMA space
    EnableSpaces();
    // Step 15(a) Program BARs 0,1 and 5
    EnableBARs();

    // Step 17. Initialize Extended Configuration Space Registers
    // For boot, only program GEN2_RX_RQ
    /* For GEN1 drives,
     *  - Step 17(a) has no effect.
     *  - Step 17(b) is also not going to adversely affect
     *
     * For GEN2 drives
     *  - Step 17(a) is required
     *  - Step 17(b) : if fuse bit is high, then GEN1 will be forced else continue to negotiate as GEN2
    */
    // Step 17(a)

    if (!FuseData.AllowGen2) {
        // Backdoor write to Implementation specifc bit in HBA_CAP register to set speed support to GEN1
        SATA0_RMW_FLD(AHCI_HBA_CAP_BKDR, INTF_SPD_SUPP, GEN1);

        // Force GEN1
        AHCI_RMW_FLD(PORT_PXSCTL, SPD, GEN1);
    }

    //UphySataCalibPadCntrl();

    // Make sure SATA interrupts are not forwarded by SATA IPFS
    NV_WRITE32(NV_ADDRESS_MAP_SATA_BASE + SATA_INTR_MASK_0, 0x0);

    // Step 14(a) Disable NATIVE mode for Legacy PIO - after CFG 4,5,9 registers
    // otherwise CFG_4,5,9 registers will become non-writable and read as zeroes
    SATA0_RMW_FLD(CFG_SATA, FORCE_NATIVE, DISABLED);
    // Update class code for AHCI DMA mode only
    if (SataContext->SataMode == NvBootSataMode_AhciDma) {
        UpdateClasscode();
        AhciEnable();

        s_AhciPxImplemented = PortImplemented();
        if (s_AhciPxImplemented != AHCI_HBA_PI_0_PI_FIRST) {
            Error = NvBootError_DeviceUnsupported;
            goto end;
        }

        s_AhciPxCmdSlotsSupported = GetNumCmdSlots();
        if ((s_AhciPxCmdSlotsSupported > AHCI_HBA_CAP_0_NCS_32)) {
            Error = NvBootError_DeviceUnsupported;
            goto end;
        }

        Error = CheckPortCommandErr();
        if (Error) {
            goto end;
        }

        ProgramCommandAndFISReceive();
        ClearPortErrors();
    }

    //Disable Interrupts
    AHCI_RMW_FLD(HBA_GHC, IE, FALSE);

    Error = IssueComReset(FuseData.ComInitWaitIndex, FuseData.ComresetRetries);
    if (Error != NvBootError_Success) {
        SET_SATA_BOOT_INFO_BITFLD(InitStatus,SataInitStatus_ComResetFailed);
        goto end;
    }

    if (SataContext->SataMode == NvBootSataMode_AhciDma) {
        ClearPortErrors();
        // Step 24(s)
        AHCI_RMW_FLD(PORT_PXCMD, ST, SET);
    }

    if (SataContext->IsSataInitialized) {
        SET_SATA_BOOT_INFO_BITFLD(InitStatus,SataInitStatus_SataReinitialized);
    }

    SataContext->IsSataInitialized = NV_TRUE;

    Error = NvBootError_Success;

end:
    SET_SATA_BOOT_INFO_BITFLD(InitStatus, (NvU32)Error);

    return Error;
}

NvBootError NvBootSataReadPage(const NvU32 Block,
                               const NvU32 Page,
                               const NvU32 Len,
                               uint8_t *Dest)
{
    NvBootError e = NvBootError_IllegalParameter;

    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < SataContext->PagesPerBlock);
    // ASSUMPTION : Everything is already configured. These are the two supported modes.
    if (SataContext->SataMode == NvBootSataMode_LegacyPio) {
        e = NvBootSataLegacyPioRead(Block, Page, Len, Dest);
    }
    else if (SataContext->SataMode == NvBootSataMode_AhciDma) {
        e = NvBootSataAhciDmaRead(Block, Page, Len, Dest);
    }

    if (e == NvBootError_Success) {
        SET_SATA_BOOT_INFO_FLD(LastBlockRead, Block);
        SET_SATA_BOOT_INFO_FLD(LastPageRead, Page);
        //TODO this will not be accurate, maybe fix
        SET_SATA_BOOT_INFO_FLD(NumPagesRead, GET_SATA_BOOT_INFO_FLD(NumPagesRead) + 1);
    }
    return e;
}

NvBootError NvBootSataWritePage(const NvU32 Block,
                                const NvU32 Page,
                                const NvU32 Len __attribute__ ((unused)),
                                uint8_t *Dest)
{
    //TODO Respect Len. Currently just writes 1 page
    return NvBootSataLegacyPioWrite(Block, Page, Dest);
}

NvBootDeviceStatus NvBootSataQueryStatus(void)
{
    return NvBootDeviceStatus_Idle;
}

void NvBootSataShutdown(void)
{
    //Disable Clocks to SATA controller and SATA OOB
    NvBootClocksSetEnable(NvBootClocksClockId_SataId, NV_FALSE);
    NvBootClocksSetEnable(NvBootClocksClockId_SataAuxId, NV_FALSE);
    NvBootClocksSetEnable(NvBootClocksClockId_SataOobId, NV_FALSE);

    NvBootUtilWaitUS(2);
}

NvBootError NvBootSataGetReaderBuffersBase(uint8_t** ReaderBuffersBase __attribute__((unused)),
                                           const NvU32 __attribute__((unused)) Alignment,
                                           const NvU32 __attribute__((unused)) Bytes)
{
    // This routine is no longer called
    return NvBootError_Unimplemented;
}

NvBootDeviceStatus NvBootSataPinMuxInit(const void *Params __attribute__((unused)))
{
    //There is another function to do this, but didn't have the time in t186 to evaluate
    NvU32 ClkReqPmx = NV_READ32(NV_ADDRESS_MAP_PADCTL_A7_BASE + PADCTL_PEX_CTL_PEX_L2_CLKREQ_N_0);
    ClkReqPmx = NV_FLD_SET_DRF_DEF(PADCTL_PEX_CTL, PEX_L2_CLKREQ_N, TRISTATE, PASSTHROUGH, ClkReqPmx);
    ClkReqPmx = NV_FLD_SET_DRF_DEF(PADCTL_PEX_CTL, PEX_L2_CLKREQ_N, PM, SATA, ClkReqPmx);
    NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A7_BASE + PADCTL_PEX_CTL_PEX_L2_CLKREQ_N_0, ClkReqPmx);
    
    NvU32 RstPmx = NV_READ32(NV_ADDRESS_MAP_PADCTL_A7_BASE + PADCTL_PEX_CTL_PEX_L2_RST_N_0);
    RstPmx = NV_FLD_SET_DRF_DEF(PADCTL_PEX_CTL, PEX_L2_RST_N, TRISTATE, PASSTHROUGH, RstPmx);
    RstPmx = NV_FLD_SET_DRF_DEF(PADCTL_PEX_CTL, PEX_L2_RST_N, PM, SATA, RstPmx);
    NV_WRITE32(NV_ADDRESS_MAP_PADCTL_A7_BASE + PADCTL_PEX_CTL_PEX_L2_RST_N_0, RstPmx);

    return NvBootError_Success;
}
