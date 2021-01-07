/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_sata_local.h"
#include "nvboot_sata_ahci_struct.h"
#include "nvboot_sata_context.h"
#include "nvboot_sata_int.h"

#include "nvrm_drf.h"

#include "arsata.h"
#include "arapb_misc.h"
#include "arclk_rst.h"

#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_sdram_int.h"
#include "nvboot_arc_int.h"
#include "nvboot_buffers_int.h"
#include "nvboot_limits_int.h"
#include "project.h"

// All includes from hw/ap/manuals
#include "dev_t_fpci_sata0.h"
#include "dev_t_sata.h"
#include "dev_t_ahci.h"
#include "ardev_t_fpci_sata0.h"
#include "ardev_t_sata.h"
#include "ardev_t_ahci.h"
#include "arxusb_padctl.h"

// Sector size is 512 bytes. - By definition in SATA spec
#define SECTOR_SIZE_LOG2    9

// Internal SATA context object needed to ensure driver programs controller for
// data transfer as per the mode within the params
static NvBootSataContext gs_SataContext;

// Boot Info table.
extern NvBootInfoTable BootInfoTable;

// Pointer to Sata Bit info. - Unused till cold boot from sata is tested.
static NvBootSataStatus* s_pSataBitInfo =
              &BootInfoTable.SecondaryDevStatus.SataStatus;

static NvBootSataParams s_DefaultSataParams;
static SataFuseParam s_SataFuseParamData;

static NvU32 s_AhciPxImplemented;
static NvU32 s_AhciPxCmdSlotsSupported;

static const SataPadCntrlReg s_SataCalibPadVal[] =
    {
        {
            /* SATA_CALIB[1:0]  = 00'b*/
            0x15,
            0x07,
            0x0F,
            0x18,
            0x0A,
            0x8F
        },
        {
            /* SATA_CALIB[1:0]  = 01'b*/
            0x12,
            0x05,
            0x0F,
            0x15,
            0x0A,
            0x4F
        },
        {
            /* SATA_CALIB[1:0]  = 10'b*/
            0x0F,
            0x04,
            0x0F,
            0x12,
            0x0A,
            0x0F
        },
        {
            /* SATA_CALIB[1:0]  = 11'b*/
            0x18,
            0x0A,
            0x0F,
            0x1B,
            0x0E,
            0xCD
        }


    };

static const NvU8 gs_OscFreqIndex[] = {
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0x00,  // 19.2MHz
    0x01,  // 38.4MHz
    0xFF,
    0xFF,
    0x02,  // 12MHz
    0x03,  // 48MHz
    0xFF,
    0xFF,
    0xFF 
};
static const SataSrcPllDivisors gs_PllDiv[] = {
    /* 
     * pllrefe_divm, 	pllrefe_divn, 	pllrefe_divp,
     * plle_divm, 	plle_divn, 	plle_divpl,
     * pllc4_divm, 	pllc4_divn, 	pllc4_divp
     */
    // 19.2 MHz
    { 0x2, 0x41, 0x2, 0x1a, 0x64, 0xe, 0x1, 0x32, 0x1 },
    // 38.4 MHz
    { 0x4, 0x41, 0x2, 0x1a, 0x64, 0xe, 0x2, 0x32, 0x1 },
    // 12 MHz
    { 0x1, 0x33, 0x2, 0x1a, 0x64, 0xe, 0x1, 0x50, 0x1 },
    // 48 MHz
    { 0x4, 0x33, 0x2, 0x1a, 0x64, 0xe, 0x4, 0x50, 0x1 },
};

// Refer T210_SATA_BootRom.doc (by Michael Hopgood)
static const NvU32 gs_UphyPllCfg[NUM_SATA_XUSB_PAD_PLL_REGS] = {
    0x01e00017, // PLL_S0_CTL_1_0 (POR) PLL0_PWR_OVRD = 1
    0x00001364, // PLL_S0_CTL_2_0 (POR) PLL0_CAL_OVRD = 1, PLL0_CAL_CTRL = 0x000136
    0x00037850, // PLL_S0_CTL_3_0 (POR)
    0x00508100, // PLL_S0_CTL_4_0 (POR) 
    0x002A0021, // PLL_S0_CTL_5_0 (POR) PLL0_DCO_CTRL = 0x2A
    0x04840000, // PLL_S0_CTL_6_0 (POR)
    0x00000333, // PLL_S0_CTL_7_0 (POR)
    0x00078333, // PLL_S0_CTL_8_0 (POR) PLL0_RCAL_OVRD = 1
    0x00000000, // PLL_S0_CTL_9_0 (POR)
    0x08000000, // PLL_S0_CTL_10_0 (POR)
    0x00000000  // PLL_S0_CTL_11_0 (RO) SATA Delta IAS states programming this register
};

// Refer T210_SATA_BootRom.doc (by Michael Hopgood)
static const NvU32 gs_UphyMiscPadCfg[NUM_SATA_XUSB_MISC_PAD_REGS] = {
    0x10402000, // MISC_PAD_S0_CTL_1_0 (POR) AUX_RX_MODE_OVRD=1, AUX_RX_IDLE_EN=1
    0x00053030, // MISC_PAD_S0_CTL_2_0 (POR)
    0x00000000, // MISC_PAD_S0_CTL_3_0 (POR)
    0x00a00002, // MISC_PAD_S0_CTL_4_0 (POR) RX_TERM_EN=1, RX_TERM_OVRD=1
    0x00000000, // MISC_PAD_S0_CTL_5_0 (POR)
    0x00000000, // MISC_PAD_S0_CTL_6_0 (POR)
    0x00000000, // MISC_PAD_S0_CTL_7_0 (POR)
    0x08000000, // MISC_PAD_S0_CTL_8_0 (POR)
    0x00000000  // MISC_PAD_S0_CTL_9_0 (RO) SATA Delta IAS states programming this register

};

// Refer T210_SATA_BootRom.doc (by Michael Hopgood) for electrical controls
static const SataExtElectricalControls gs_UphyRxEqCfg = {
    // RX_EQ_CTRL_L_GEN1
    0x55010000,
    // RX_EQ_CTRL_L_GEN2
    0x55010000,
    // RX_EQ_CTRL_H_GEN1	
    0x00000001,
    // RX_EQ_CTRL_H_GEN2
    0x00000001,
};
static const NvBootPlleSsaCoeff gs_SsaCoeffCfg = { 0x25, 0x1, 0x20, 0x0, 0x0, 0x0};
static const NvBootPlleSsdCoeff gs_SsdCoeffCfg = { 0x0, 0x0, 0x0, 0x0};

static
NvBootError NvBootSataLegacyPioRead(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest);


static
NvBootError NvBootSataAhciDmaRead(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest);

static
void ProgramUPhyInit(void);

static
void ProgramUPhyPadPllRegs(void);

static
void ProgramUPhyPadCntrlRegs(void);

static
void ProgramUPhyRegsAfterPadPllLock(void);

static
void EnableUphyPadPll(void);


static
NvU32 * GetTable(ConstantTableIdx Idx)
{
    NvU32 *pTable = NULL;
    switch(Idx)
    {
        case ConstantTableIdx_SataCalibPadVal:
                pTable = (NvU32 *)&s_SataCalibPadVal[0];
                break;
        case ConstantTableIdx_OscFreqIndex:
                pTable = (NvU32 *)&gs_OscFreqIndex[0];
                break;
        case ConstantTableIdx_PllDiv:
                pTable = (NvU32 *)&gs_PllDiv[0];
                break;
        case ConstantTableIdx_UphyPllCfg:
                pTable = (NvU32 *)&gs_UphyPllCfg[0];
                break;
        case ConstantTableIdx_UphyMiscPadCfg:
                pTable = (NvU32 *)&gs_UphyMiscPadCfg[0];
                break;
        case ContantTableIdx_SsaCoeffCfg:
                pTable = (NvU32 *)(&gs_SsaCoeffCfg);
                break;
        case ContantTableIdx_SsdCoeffCfg:
                pTable = (NvU32 *)(&gs_SsdCoeffCfg);
                break;
	 case ConstantTableIdx_SataExtElectricalCntrls:
	 	  pTable = (NvU32 *)(&gs_UphyRxEqCfg);
		  break;
        default:
                break;
    }
    return pTable;
}

/*
 * Public Function Definitions.
 */
void
NvBootSataGetParams(
    const NvU32 ParamIndex,
    NvBootSataParams **Params)
{
    NvBootClocksOscFreq OscFreq;
    NvU8 *pOscFreqIndex = (NvU8 *)GetTable(ConstantTableIdx_OscFreqIndex);
    SataSrcPllDivisors *pPllDiv = (SataSrcPllDivisors *)GetTable(ConstantTableIdx_PllDiv);
    NvBootSataPlleSSCoeff *pSSCoeff = NULL;
    NvU32  HBAMemSizeBytes = ((MAX_COMMAND_LIST_SIZE + 0x80) & ~(0x7f) ) +
	                          ((MAX_COMMAND_TABLES_SIZE + 0x100) & ~(0xff)) +
	                          TOTAL_FIS_SIZE;
    NvU32 BytesRequired = 0;

    NV_ASSERT(Params != NULL);

    /// Specifies the clock source for SATA controller.
    s_DefaultSataParams.SataClockSource = NvBootSataClockSource_PllPOut0;

    /// Specifes the clock divider to be used.
    s_DefaultSataParams.SataClockDivider = 6;

    /// Specifies the clock source for SATA controller.
    s_DefaultSataParams.SataOobClockSource = NvBootSataClockSource_PllPOut0;

    /// Specifes the clock divider to be used.
    s_DefaultSataParams.SataOobClockDivider = 2;

    s_DefaultSataParams.isSdramInitialized = NV_FALSE;


    if (s_SataFuseParamData.UsePllpSrcForPlle)
    {
        // Divisors for ref_clk = 408MHz
        s_DefaultSataParams.PlleDivM = 0x11;
        s_DefaultSataParams.PlleDivN = 0x64;
        s_DefaultSataParams.PlleDivPlCml = 0xe;
    }
    else
    {
        // Divisors for ref_clk = osc frequency
        OscFreq = NvBootClocksGetOscFreq();
        s_DefaultSataParams.PlleDivM = pPllDiv[pOscFreqIndex[OscFreq]].PlleDivM;
        s_DefaultSataParams.PlleDivN = pPllDiv[pOscFreqIndex[OscFreq]].PlleDivN;
        s_DefaultSataParams.PlleDivPlCml = pPllDiv[pOscFreqIndex[OscFreq]].PlleDivPlCml;

        s_DefaultSataParams.PllRefeDivM = pPllDiv[pOscFreqIndex[OscFreq]].PllrefeDivM;
        s_DefaultSataParams.PllRefeDivN = pPllDiv[pOscFreqIndex[OscFreq]].PllrefeDivN;
        s_DefaultSataParams.PllRefeDivP = pPllDiv[pOscFreqIndex[OscFreq]].PllrefeDivP;

    }

    s_DefaultSataParams.EnablePlleSS = NV_FALSE;
    s_DefaultSataParams.IsPlleSSA = 0x1;
    // Zero initialization may not be needed for global static. 
    // s_DefaultSataParams.PlleMisc = 0x0;
    // s_DefaultSataParams.PlleMisc = 0x0;
    // TBD: Retaining POR values till Hardware team provides actual values.
    if (s_DefaultSataParams.IsPlleSSA)
    {
        pSSCoeff = (NvBootSataPlleSSCoeff *)GetTable(ContantTableIdx_SsaCoeffCfg);
        s_DefaultSataParams.PlleSSCoeff.PlleSsaCoeff.PlleSSCoeffSscmax = pSSCoeff->PlleSsaCoeff.PlleSSCoeffSscmax;
        s_DefaultSataParams.PlleSSCoeff.PlleSsaCoeff.PlleSSCoeffSscinc = pSSCoeff->PlleSsaCoeff.PlleSSCoeffSscinc;
        s_DefaultSataParams.PlleSSCoeff.PlleSsaCoeff.PlleSSCoeffSscincintrvl = 
                            pSSCoeff->PlleSsaCoeff.PlleSSCoeffSscincintrvl;
        s_DefaultSataParams.PlleSSCoeff.PlleSsaCoeff.PlleSSCoeffIntegoffset =
                            pSSCoeff->PlleSsaCoeff.PlleSSCoeffIntegoffset;
        s_DefaultSataParams.PlleSSCoeff.PlleSsaCoeff.PlleSSCoeffSsccenter = pSSCoeff->PlleSsaCoeff.PlleSSCoeffSsccenter;
        s_DefaultSataParams.PlleSSCoeff.PlleSsaCoeff.PlleSSCoeffSscinvert = pSSCoeff->PlleSsaCoeff.PlleSSCoeffSscinvert;
    }
    else
    {
        pSSCoeff = (NvBootSataPlleSSCoeff *)GetTable(ContantTableIdx_SsdCoeffCfg);
        s_DefaultSataParams.PlleSSCoeff.PlleSsdCoeff.PlleSSCoeffSdmsscmax = pSSCoeff->PlleSsdCoeff.PlleSSCoeffSdmsscmax;
        s_DefaultSataParams.PlleSSCoeff.PlleSsdCoeff.PlleSSCoeffSdmsscmin = pSSCoeff->PlleSsdCoeff.PlleSSCoeffSdmsscmin;
        s_DefaultSataParams.PlleSSCoeff.PlleSsdCoeff.PlleSSCoeffSdmsscstep =
                            pSSCoeff->PlleSsdCoeff.PlleSSCoeffSdmsscstep;
        s_DefaultSataParams.PlleSSCoeff.PlleSsdCoeff.PlleSSCoeffSdmdin = pSSCoeff->PlleSsdCoeff.PlleSSCoeffSdmdin;
    }

    // Read the fuse parameters into s_SataFuseParamData
    GET_FUSE_PARAM(ParamIndex, USE_PLLP_SOURCE, UsePllpSrcForPlle);
    GET_FUSE_PARAM(ParamIndex, COMRESET_RETRIES, NumComresetRetries);
    GET_FUSE_PARAM(ParamIndex, FORCE_GEN1, ForceGen1);

    GET_FUSE_PARAM(ParamIndex, MODE, Mode);
    GET_FUSE_PARAM(ParamIndex, COMINIT_WAIT_IND, ComInitWaitIndex);
    GET_FUSE_PARAM(ParamIndex, PAGE_SIZE_MULT_LOG2, PageSizeMultLog2);

    switch (s_SataFuseParamData.Mode)
    {
        case 1: // AHCI DMA Mode
             /// Specifies the SATA mode to be used.
             s_DefaultSataParams.SataMode = NvBootSataMode_Ahci;
             /// Specifies mode of transfer from device to memory via controller.
             /// Note: transfer mode doesn't imply SW intervention.
             s_DefaultSataParams.TransferMode = NvBootSataTransferMode_Dma;

             // IRAM_DATA_AHCI_BUFFER_BASE
             // IRAM D limit MINUS twice page size 
             // Should be 1 word aligned base addr (keeping it atleast 16 bytes aligned)
             // Data buffer should be twice the page size (need two data buffers for Reader code to work with)
             // Ideally, only min of of reader buffer size and the required data buffers' size should be allocated
             // and checked for in NvBootGetReaderBuffersBase. But we don't want to touch NvBootGetReaderBuffersBase
             // anymore (it's common code to all modes of operation).
             BytesRequired = NV_MAX(NVBOOT_MAX_BUFFER_SIZE, (1 << 
                                                   (SECTOR_SIZE_LOG2 + s_SataFuseParamData.PageSizeMultLog2 + 1)));
             s_DefaultSataParams.DataBufferBase = NVBOOT_BL_IRAM_END -  BytesRequired;
	      if (s_DefaultSataParams.DataBufferBase & 0xf)
                 s_DefaultSataParams.DataBufferBase = (s_DefaultSataParams.DataBufferBase - 0x10 ) & (~0xf);

	      // IRAM_SATA_AHCI_HBAMEM_BASE - Required only for AHCI DMA
	      // HBAMem base for SATA - default value
             // IRAM D limit MINUS  Max Data buffers size (1KB aligned base addr) MINUS 
             // HBA Memory Size (1KB aligned base addr) 
             s_DefaultSataParams.SataBuffersBase =  s_DefaultSataParams.DataBufferBase - HBAMemSizeBytes;
             if (s_DefaultSataParams.SataBuffersBase & 0xfff)
			 s_DefaultSataParams.SataBuffersBase = (s_DefaultSataParams.SataBuffersBase - 0x1000) & (~0xfff);

             break;

        default: // 0 or not set = PIO mode
             /// Specifies the SATA mode to be used.
             s_DefaultSataParams.SataMode = NvBootSataMode_Legacy;
             /// Specifies mode of transfer from device to memory via controller.
             /// Note: transfer mode doesn't imply SW intervention.
             s_DefaultSataParams.TransferMode = NvBootSataTransferMode_Pio;

             // Valid Start address(in IRAM) for data buffers. - This will be considered invalid
             // by NvBootSataGetReaderBuffersBase and reader buffers will be assigned by
             // reader code for pio mode reads.
             s_DefaultSataParams.DataBufferBase = (NvU32)Buffer[0];

	      // Start address for command and FIS structures. - Not used for
	      // pio mode
             s_DefaultSataParams.SataBuffersBase = 0;

             break;

    }
  

    // Get the sata parameters for mode and timing.
     *Params = &s_DefaultSataParams;

}

NvBool
NvBootSataValidateParams(
    const NvBootSataParams *Params)
{
    NvBool ParamsValid = NV_FALSE;
    NvU32 PageSizeLog2 = SECTOR_SIZE_LOG2;
    // Default : Max page size supported is 16KB
    // Max page size currently supported is 16KB because the reader code uses
    // buffers that are 16KB max.
    NvU32 MaxPageSizeSupportedLog2 = NVBOOT_SATA_BLOCK_SIZE_LOG2;

    NV_ASSERT(Params);

    if (Params != NULL)
    {
        ParamsValid = (Params->SataClockSource <= NvBootSataClockSource_Num);
        ParamsValid &= (Params->SataOobClockSource <= NvBootSataClockSource_Num);

        ParamsValid &= ((Params->SataMode <= NvBootSataMode_Num) &&
                                        (Params->SataMode > 0));

        // AHCI PIO not supported in T210 bootrom SATA driver.
        ParamsValid &= ((Params->SataMode == NvBootSataMode_Ahci) ?
                    ((Params->TransferMode == NvBootSataTransferMode_Pio) ?
                    NV_FALSE : NV_TRUE) : NV_TRUE);
        // Assumption: Legacy DMA will not be supported in T210 bootrom SATA
        // driver
        ParamsValid &= ((Params->SataMode == NvBootSataMode_Legacy) ?
                    ((Params->TransferMode == NvBootSataTransferMode_Dma) ?
                    NV_FALSE : NV_TRUE) : NV_TRUE);
        /* Supported Page Size Log2 values <= 16K*/
        PageSizeLog2 = SECTOR_SIZE_LOG2 + s_SataFuseParamData.PageSizeMultLog2;

        if ((NvBootGetSwCYA() & NVBOOT_SW_CYA_SATA_32KB_PAGE_SIZE_SUPPORT_DISABLE) ||
			((Params->SataMode == NvBootSataMode_Legacy) && (Params->TransferMode == NvBootSataTransferMode_Pio)))
        {
            // Max page size supported is changed to 16KB
            MaxPageSizeSupportedLog2 =  (NVBOOT_SATA_BLOCK_SIZE_LOG2 - 1);
        }
        ParamsValid &= (PageSizeLog2 <= MaxPageSizeSupportedLog2);
    }
    SET_SATA_BOOT_INFO_BITFLD(InitStatus, (ParamsValid ? SataInitStatus_ParamsValid : ParamsValid));
    return ParamsValid;
}

void
NvBootSataGetBlockSizes(
    const NvBootSataParams *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);

    *BlockSizeLog2 = NVBOOT_SATA_BLOCK_SIZE_LOG2;
    *PageSizeLog2 = gs_SataContext.PageSizeLog2;
}

static void
WriteByteEnable(NvBool ByteEnable, LegacyPioBe Val)
{
    NvU32 RegValue = 0;

    IPFS_REG_WRITE_FLD(SATA, DFPCI_BEN, EN_DFPCI_BEN, ByteEnable, RegValue);
    IPFS_REG_WRITE_FLD(SATA, DFPCI_BEN, DFPCI_BYTE_ENABLE_N, Val, RegValue);
}

static NvBootError
EnablePLLE(const NvBootSataParams *Params)
{
    NvU32 RegValue = 0;
    // TBD: Error handling when Lock bit is not set
    NvBootError e = NvBootError_PllNotLocked;
    NvU32 StableTime = 0;
    NvBool SelectRefPlle = NV_FALSE;
    NvBootClocksSSParams SSCoefficientTable;

    if (!Params->PlleRefSrc)
    {
        // Default: Allow Plle to be driven from PllREFE and osc freq input to PllREFGE
        SelectRefPlle = NV_TRUE;
        //NvBootClocksPllDivRstCtrl(NvBootClocksPllId_PllREFE, 0x1);
	// Misc1 is Setup[23:0] for PLLREFE
	NvBootClocksStartPll(NvBootClocksPllId_PllREFE,
	                     Params->PllRefeDivM,
	                     Params->PllRefeDivN,
	                     Params->PllRefeDivP,
	                     Params->PllRefeMisc,
	                     0,
                             &StableTime);
        // TBD: If lock override is set, then delay of 300 us + 100 us to be provided here.
        WAIT_FOR_PLLLOCK(NvBootClocksPllId_PllREFE, StableTime, SataInitStatus_PllRefeInitFailed);

    }
    // Set Pllp as source for Plle. This assumes that Pllp is locked and enabled.
    // This path hasn't been tested.

    /* Misc1 = VAL[PLLE_MISC_0]
     * Misc1[31:16] = PLLE_SETUP
     * Misc1[7:6] = PLLE_KCP
     * Misc1[5:4] = PLLE_VREG_BG_CTRL
     * Misc1[3:2] = PLLE_VREG_CTRL
     * Misc[0:0] = PLLE_KVCO
     * Misc2 = VAL[PLLE_AUX_0] Only bits PLLE_REF_SEL_PLLREFE, PLLE_REF_SRC, PLLE_CML1_OEN, PLLE_CML0_OEN
     * Misc2[0:0] = PLLE_CML0_OEN
     * Misc2[1:1] = PLLE_CML1_OEN
     * Misc2[28:28] = PLLE_REF_SEL_PLLREFE
     * Misc2[2:2] = PLLE_REF_SRC
     */
    RegValue = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0);
    RegValue = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX, PLLE_REF_SRC,
                            Params->PlleRefSrc, RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX,PLLE_REF_SEL_PLLREFE, SelectRefPlle, RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLE_AUX,PLLE_CML1_OEN, 0x1,RegValue);
    NvBootClocksStartPll(NvBootClocksPllId_PllE,
	                   Params->PlleDivM,
	                   Params->PlleDivN,
	                   Params->PlleDivPlCml,
	                   Params->PlleMisc,
	                   RegValue,
                           &StableTime );
    // TBD: If lock override is set, then delay of 300 us + 100 us to be provided here.
    WAIT_FOR_PLLLOCK(NvBootClocksPllId_PllE, StableTime, SataInitStatus_PlleInitFailed);

    // If BCT params require spread spectrum to be enabled and 
    // sata has been initialized once, then program spread spectrum coefficients 
    // and enable spread spectrum else do not.
    // ASSUMPTION: params used for second init shall have the right spread spectrum
    // coefficients.
    // DO NOT ENABLE SS during the first time sata initialization. Reduces risk.
    if ((gs_SataContext.IsSataInitialized) && (Params->EnablePlleSS))
    {
         SET_PLLE_SS_COEFF(Sdmdin, d);
         SET_PLLE_SS_COEFF(Sdmsscstep, d);
         SET_PLLE_SS_COEFF(Sdmsscmax, d);
         SET_PLLE_SS_COEFF(Sdmsscmin, d);

         SET_PLLE_SS_COEFF(Sscmax, a);
         SET_PLLE_SS_COEFF(Sscinc, a);
         SET_PLLE_SS_COEFF(Sscincintrvl, a);
         SET_PLLE_SS_COEFF(Integoffset, a);
         SET_PLLE_SS_COEFF(Ssccenter, a);
         SET_PLLE_SS_COEFF(Sscinvert, a);

         SSCoefficientTable.IsSSA = Params->IsPlleSSA;

         NvBootClocksEnablePllSpreadSpectrum(NvBootClocksPllId_PllE,
                               &SSCoefficientTable);
         // TBD: If lock override is set, then delay of 300 us + 100 us to be provided here.
         WAIT_FOR_PLLLOCK(NvBootClocksPllId_PllE, StableTime, SataInitStatus_PlleSSFailed);
         SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_PlleReinitialized);
    }

    return e;
}

static void ConfigureClockSource(const NvBootClocksClockId Id, const NvBootSataParams *Params)
{
    NvBootClocksOscFreq OscFreq;
    NvBootSataClockSource ClockSource = NvBootSataClockSource_PllPOut0;
    NvU8 ClockDivider = 0;
    NvU32 StableTime = 0;
    NvU32 RegVal = 0;
    NvU8 *pOscFreqIndex = (NvU8 *)GetTable(ConstantTableIdx_OscFreqIndex);
    SataSrcPllDivisors *pPllDiv = (SataSrcPllDivisors *)GetTable(ConstantTableIdx_PllDiv);

    NV_ASSERT((Id == NvBootClocksClockId_SataId) || (Id == NvBootClocksClockId_SataOobId));

    switch(Id) {
        case NvBootClocksClockId_SataId:
                ClockSource = Params->SataClockSource;
                SET_SATA_BOOT_INFO_BITFLD(SataClockSource, ClockSource<<16);
                ClockDivider = Params->SataClockDivider;
                SET_SATA_BOOT_INFO_FLD(SataClockDivider, ClockDivider);
                NV_ASSERT((ClockSource < NvBootSataClockSource_Num));
                break;
        case NvBootClocksClockId_SataOobId:
                ClockSource = Params->SataOobClockSource;
                ClockDivider = Params->SataOobClockDivider;
                SET_SATA_BOOT_INFO_BITFLD(SataClockSource, ClockSource);
                SET_SATA_BOOT_INFO_FLD(SataOobClockDivider, ClockDivider);
                NV_ASSERT((ClockSource < NvBootSataOobClockSource_Num));
                break;

    }

    switch (ClockSource)
    {
        case NvBootSataClockSource_PllC4Out0:
        case NvBootSataClockSource_PllC4Out1:
            // Divisors for ref_clk = osc frequency
            OscFreq = NvBootClocksGetOscFreq();
            // Enable Pllc4 or re-program it in case already enabled(eg. in UARTA use for ATE)
            NvBootClocksStartPll(NvBootClocksPllId_PllC4,
                     pPllDiv[pOscFreqIndex[OscFreq]].Pllc4DivM /*M*/,
                     pPllDiv[pOscFreqIndex[OscFreq]].Pllc4DivN /*N*/,
                     pPllDiv[pOscFreqIndex[OscFreq]].Pllc4DivP /*P*/,
                     NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLC4_MISC, PLLC4_EN_LCKDET, 1, RegVal),
                     0x0,
                     &StableTime );
            while(!NvBootClocksIsPllStable(NvBootClocksPllId_PllC4, StableTime));
        case NvBootSataClockSource_ClkM:
        case NvBootSataClockSource_PllPOut0:
        default:
            NvBootClocksConfigureClock(Id, 
                           ClockDivider,
                           ClockSource );
            break;
    }
}

static void ProgramPadCntrlRegisters(void)
{

    NvU32 RegVal = 0;
    NvU32 SataCalibValue = 0;
    SataPadCntrlReg *pSataCalibPadVal = (SataPadCntrlReg *)GetTable(ConstantTableIdx_SataCalibPadVal);

    NvBootFuseGetSataCalib(&SataCalibValue);

    // Enable updates to Phy Ctrl Reg fields TX_AMP/TX_PEAK
     SATA0_DFPCI_READ32(INDEX, RegVal);
     SATA0_DFPCI_SET_FLD(INDEX, CH1, T_SATA0_INDEX_CH1_SELECTED, RegVal);
     SATA0_DFPCI_WRITE32(INDEX, RegVal);

    // Program TX_AMP and TX_PEAK values for GEN1 as per SATA_CALIB[1:0]
     SATA0_DFPCI_READ32(CHX_PHY_CTRL1_GEN1, RegVal);
     SATA0_DFPCI_SET_FLD(CHX_PHY_CTRL1_GEN1, TX_AMP, pSataCalibPadVal[SataCalibValue].Gen1TxAmp, RegVal);
     SATA0_DFPCI_SET_FLD(CHX_PHY_CTRL1_GEN1, TX_PEAK, pSataCalibPadVal[SataCalibValue].Gen1TxPeak, RegVal);
     SATA0_DFPCI_WRITE32(CHX_PHY_CTRL1_GEN1, RegVal);

	 
    // Program TX_AMP and TX_PEAK values for GEN2 as per SATA_CALIB[1:0]
     SATA0_DFPCI_READ32(CHX_PHY_CTRL1_GEN2, RegVal);
     SATA0_DFPCI_SET_FLD(CHX_PHY_CTRL1_GEN2, TX_AMP, pSataCalibPadVal[SataCalibValue].Gen2TxAmp, RegVal);
     SATA0_DFPCI_SET_FLD(CHX_PHY_CTRL1_GEN2, TX_PEAK, pSataCalibPadVal[SataCalibValue].Gen2TxPeak, RegVal);
     SATA0_DFPCI_WRITE32(CHX_PHY_CTRL1_GEN2, RegVal);

     SATA0_DFPCI_READ32(CHX_PHY_CTRL11, RegVal);
     SATA0_DFPCI_SET_FLD(CHX_PHY_CTRL11, GEN1_RX_EQ, pSataCalibPadVal[SataCalibValue].Gen1RxCtle, RegVal);
     SATA0_DFPCI_SET_FLD(CHX_PHY_CTRL11, GEN2_RX_EQ, pSataCalibPadVal[SataCalibValue].Gen2RxCtle, RegVal);
     SATA0_DFPCI_WRITE32(CHX_PHY_CTRL11, RegVal);
	 
    // Disable updates to Phy Ctrl Reg fields TX_AMP/TX_PEAK
     SATA0_DFPCI_READ32(INDEX, RegVal);
     SATA0_DFPCI_SET_FLD(INDEX, CH1, T_SATA0_INDEX_CH1_UNSELECTED, RegVal);
     SATA0_DFPCI_WRITE32(INDEX, RegVal);
}

static
void ProgramUPhyInit()
{
    NvU32 RegValue = 0;
    // Release reset to XUSB_PADCTL
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbId, NV_FALSE);

    // Clear Reset for SATA USB UPHY (PAD PLL)
    RegValue = 0x00001000;
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_Y_CLR_0, RegValue);

}
#if NVBOOT_TARGET_FPGA
static
void ProgramUPhyPadPllRegs()
{
}
static
void ProgramUPhyPadCntrlRegs()
{
}
static
void ProgramUPhyRegsAfterPadPllLock()
{
}
static void EnableUphyPadPll()
{
}
#else
/* Refer T210_SATA_BootRom.doc by Michael Hopgood */
static
void ProgramUPhyPadPllRegs()
{
    NvU32 iterator = 0;
    NvU32 * pUphyPllCfg = GetTable(ConstantTableIdx_UphyPllCfg);

    NvU32 NumRegs = NUM_SATA_XUSB_PAD_PLL_REGS;
    NvU32 BaseReg = NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0;
    // Program Pll config registers for SATA USB pads
    for (iterator = 0; iterator < NumRegs; iterator++)
    {
        NV_WRITE32(BaseReg + (sizeof(NvU32) * iterator), pUphyPllCfg[iterator]);
    }
}

/* Refer T210_SATA_BootRom.doc by Michael Hopgood */
static
void ProgramUPhyPadCntrlRegs()
{
    NvU32 iterator = 0;
    NvU32 *pUphyMiscPadCfg = GetTable(ConstantTableIdx_UphyMiscPadCfg);

    // Program pad control registers for SATA USB pads
    NvU32 NumRegs = NUM_SATA_XUSB_MISC_PAD_REGS;
    NvU32 BaseReg = NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_MISC_PAD_S0_CTL_1_0;
    for (iterator = 0; iterator < NumRegs; iterator++)
    {
        NV_WRITE32(BaseReg + (sizeof(NvU32) * iterator), pUphyMiscPadCfg[iterator]);
    }
}

/* Refer T210_SATA_BootRom.doc by Michael Hopgood */
static
void ProgramUPhyRegsAfterPadPllLock()
{

    NvU32 RegValue = 0;

    // Lane0 is set default to SATA and no other code snippet in BootRom is expected
    // to use Lane0 for xusb. Therefore, no need to program XUSB_PADCTL_USB3_PAD_MUX_0[SATA_PAD_LANE0]
    /// IDDQ to be disabled for SATA IO PHY to be out of IDDQ
    /// Set FORCE_SATA_PAD_IDDQ_DISABLE_MASK0 = 1'b (DISABLED) - remove iddq override
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, USB3_PAD_MUX, FORCE_SATA_PAD_IDDQ_DISABLE_MASK0,         
          XUSB_PADCTL_USB3_PAD_MUX_0_FORCE_SATA_PAD_IDDQ_DISABLE_MASK0_DISABLED, RegValue);

 //===========
 
    // Program clamps
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, ELPG_PROGRAM_1, AUX_MUX_LP0_CLAMP_EN,         
          XUSB_PADCTL_ELPG_PROGRAM_1_0_AUX_MUX_LP0_CLAMP_EN_NO, RegValue);

    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, ELPG_PROGRAM_1, AUX_MUX_LP0_CLAMP_EN_EARLY,         
          XUSB_PADCTL_ELPG_PROGRAM_1_0_AUX_MUX_LP0_CLAMP_EN_EARLY_NO, RegValue);
	 
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, ELPG_PROGRAM_1, AUX_MUX_LP0_VCORE_DOWN,         
          XUSB_PADCTL_ELPG_PROGRAM_1_0_AUX_MUX_LP0_VCORE_DOWN_NO, RegValue);
   
 //===========
    // Wait for 200us recommended
    NvBootUtilWaitUS(200);

}

static
void PerformUphyPllCalibration()
{
    NvU32 TimeOut = 100;
    NvU32 RegValue = 0;

    // Enable PLL0 CAL
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_2, PLL0_CAL_EN, 1, RegValue);
    // Wait for XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0_PLL0_CAL_DONE to turn 1 with a timeout of
    // 200 us.
    TimeOut = NVBOOT_SATA_PAD_PLL_CAL_DONE_WAIT;
    // Wait until cal done
    do
    {
       NvBootUtilWaitUS(1);
       TimeOut--;
       RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0);
    }while ((!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_2,PLL0_CAL_DONE, RegValue)) && (TimeOut));

    if ((!TimeOut) && (!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_2,PLL0_CAL_DONE, RegValue)))
    {
         NvBootUtilWaitUS(NVBOOT_SATA_PAD_PLL_WAIT_MARGIN);
	  RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0);
	  if (!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_2,PLL0_CAL_DONE, RegValue))
	  {
	      // Set error flag. Don't return error. Record the error in the BIT.
	      SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_UphyPllCalFailed);
	  }
	  
    }
	
    // Clear CAL EN
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_2, PLL0_CAL_EN, 0, RegValue);
    TimeOut = NVBOOT_SATA_PAD_PLL_CAL_DONE_WAIT;
    
    // Wait until cal done cleared
    do
    {
       NvBootUtilWaitUS(1);
       TimeOut--;
       RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0);
    }while ((NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_2,PLL0_CAL_DONE, RegValue)) && (TimeOut));

    if ((!TimeOut) && (NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_2,PLL0_CAL_DONE, RegValue)))
    {
         NvBootUtilWaitUS(NVBOOT_SATA_PAD_PLL_WAIT_MARGIN);
	  RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0);
	  if (NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_2,PLL0_CAL_DONE, RegValue))
	  {
	      // Set error flag. Don't return error. Record the error in the BIT.
	      SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_UphyPllCalNotCleared);
	  }
	  
    }    
}

static void PerformUphyPllRCalibration()
{
    NvU32 TimeOut = 100;
    NvU32 RegValue = 0;
    // Enable RCAL and RCAL CLK
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_8, PLL0_RCAL_EN, 1, RegValue);
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_8, PLL0_RCAL_CLK_EN, 1, RegValue);

    TimeOut = NVBOOT_SATA_PAD_PLL_RCAL_DONE_WAIT;

    // Wait until rcal done
    do
    {
       NvBootUtilWaitUS(1);
       TimeOut--;
       RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_8_0);
    }while ((!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_8,PLL0_RCAL_DONE, RegValue)) && (TimeOut));

    if ((!TimeOut) && (!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_8,PLL0_RCAL_DONE, RegValue)))
    {
         NvBootUtilWaitUS(NVBOOT_SATA_PAD_PLL_WAIT_MARGIN);
	  RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_8_0);
	  if (!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_8,PLL0_RCAL_DONE, RegValue))
	  {
	      // Set error flag. Don't return error. Record the error in the BIT.
	      SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_UphyPllRCalFailed);
	  }
	  
    }
    // Clear RCAL EN
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_8, PLL0_RCAL_EN, 0, RegValue);
    TimeOut = NVBOOT_SATA_PAD_PLL_RCAL_DONE_WAIT;

    // Wait until RCAL DONE cleared
    do
    {
       NvBootUtilWaitUS(1);
       TimeOut--;
       RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_8_0);
    }while ((NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_8,PLL0_RCAL_DONE, RegValue)) && (TimeOut));

   if ((!TimeOut) && (NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_8,PLL0_RCAL_DONE, RegValue)))
    {
         NvBootUtilWaitUS(NVBOOT_SATA_PAD_PLL_WAIT_MARGIN);
	  RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_8_0);
	  if (NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_8,PLL0_RCAL_DONE, RegValue))
	  {
	      // Set error flag. Don't return error. Record the error in the BIT.
	      SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_UphyPllRCalNotCleared);
	  }
	  
    }
   // Disable RCAL CLK
   XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_8, PLL0_RCAL_CLK_EN, 0, RegValue);
}

/* Refer T210_SATA_BootRom.doc by Michael Hopgood */
static void EnableUphyPadPll()
{
    NvU32 TimeOut = 100;
    NvU32 RegValue = 0;


    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_1, PLL0_PWR_OVRD, 1, RegValue);
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_2, PLL0_CAL_OVRD, 1, RegValue);
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_8, PLL0_RCAL_OVRD, 1, RegValue);

    // Before enabling calibration/enabling pad pll, M/N/P divisors and PLL mode should be programmed.
    // This is taken care of by function ProgramUPhyPadPllRegs
    // These values can be patched if required (specifically for PLL mode - default ring pll is being considered
    // LC pll is only as a backup option in case failures seen with ring pll
 //===========
    // Clear PLL0_IDDQ and PLL0_SLEEP
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_1, PLL0_IDDQ, 0, RegValue);
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_1, PLL0_SLEEP, 0, RegValue);  
    // Wait for 100ns required here(usually absorbed in later register writes). 
    // Still explicit delay provided here. Timer granularity doesn't allow less than 1us wait. 
    NvBootUtilWaitUS(1);

//===========
    // Do PLL Calibration
    PerformUphyPllCalibration();

//===========
    // Enable Pad Pll
    XUSB_PADCTL_WRITE_FLD(XUSB_PADCTL, UPHY_PLL_S0_CTL_1, PLL0_ENABLE, 1, RegValue);
	
    // Wait for SATA_AUX_PAD_PLL_CNTRL_0_0_LOCKDET to turn 1 with a timeout of
    // 15 us.
    TimeOut = NVBOOT_SATA_PAD_PLL_LOCK_WAIT;

    // Wait until lockdet is set
    do
    {
       NvBootUtilWaitUS(1);
       TimeOut--;
       RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0);
    }while ((!NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_1,PLL0_LOCKDET_STATUS, RegValue)) && (TimeOut));

    if ((!TimeOut) && (! NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_1,PLL0_LOCKDET_STATUS, RegValue) ))
    {
        NvBootUtilWaitUS(NVBOOT_SATA_PAD_PLL_WAIT_MARGIN);
	 RegValue = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0);
	 if (! NV_DRF_VAL(XUSB_PADCTL, UPHY_PLL_S0_CTL_1, PLL0_LOCKDET_STATUS, RegValue) )
	 {
            // Don't return error. Record the error in the BIT.
            SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_PadPllNotLocked);
         }
    }
//============
    // Do Resistor Calibration
    PerformUphyPllRCalibration();

}
#endif

static void ProgramSquelch(){

    // Code snippet initially provided by Gaurav
    // NV_WRITE32(0x70021120,0x3328A5AC);

    // POR value programming, option to program SATA0_CFG_PHY_0_0_USE_7BIT_ALIGN_DET_FOR_SPD_NO later subject to SwCya bit
    // Option to patch the value here.
    NvU32 RegValue = 0;
    SATA0_REF_DFPCI_READ32(CFG_PHY_0_0 , RegValue);
    SATA0_REF_DFPCI_WRITE32(CFG_PHY_0_0 , RegValue);

    // Code snippet initially provided by Gaurav
    // NV_WRITE32(0x70021114,0x4d000000);
    // POR value programming, Option to patch the value here.
    // Program Comma Count
    SATA0_REF_DFPCI_READ32(NVOOB_0 , RegValue);
    SATA0_REF_DFPCI_WRITE32(NVOOB_0 , RegValue);


}

static void ProgramChPhyCtrl() {
    SataExtElectricalControls *pUphyRxEqCfg = (SataExtElectricalControls *)GetTable(ConstantTableIdx_SataExtElectricalCntrls);

    NvU32 RegValue = 0;
    // Code snippet initially provided by Gaurav
    // POR value programming - option to patch the value here.

    SATA0_REF_DFPCI_READ32(CFG_LINK_0_0, RegValue);
    SATA0_REF_DFPCI_WRITE32(CFG_LINK_0_0, RegValue);


    /* Refer T210_SATA_BootRom.doc by Michael Hopgood */
    // RX_EQ_CTRL_L_GEN1
    SATA0_DFPCI_WRITE32(CHX_PHY_CTRL17, pUphyRxEqCfg->Gen1RxEqCtrlL);
    // RX_EQ_CTRL_L_GEN2
    SATA0_DFPCI_WRITE32(CHX_PHY_CTRL18, pUphyRxEqCfg->Gen2RxEqCtrlL);

    // RX_EQ_CTRL_H_GEN1
    SATA0_DFPCI_WRITE32(CHX_PHY_CTRL20, pUphyRxEqCfg->Gen1RxEqCtrlH);
    // RX_EQ_CTRL_H_GEN2
    SATA0_DFPCI_WRITE32(CHX_PHY_CTRL21, pUphyRxEqCfg->Gen2RxEqCtrlH);

}

static void EnableSpaces()
{
    NvU32 RegVal = 0;
    // Steps 14(b) and 14(c) Enable IO and MEM space
    SATA0_DFPCI_READ32(CFG_1, RegVal);
    SATA0_DFPCI_SET_FLD(CFG_1, IO_SPACE, T_SATA0_CFG_1_IO_SPACE_ENABLED, RegVal);
    SATA0_DFPCI_SET_FLD(CFG_1, MEMORY_SPACE, T_SATA0_CFG_1_MEMORY_SPACE_ENABLED, RegVal);

    // Step 24(a) For AHCI mode, enable bus master
    if ((gs_SataContext.SataMode == NvBootSataMode_Ahci) &&
        (gs_SataContext.XferMode == NvBootSataTransferMode_Dma))
    {
        SATA0_DFPCI_SET_FLD(CFG_1, BUS_MASTER, T_SATA0_CFG_1_BUS_MASTER_ENABLED, RegVal);
    }
    SATA0_DFPCI_WRITE32(CFG_1, RegVal);

    // Step 14(d) Enable ADMA space
    SATA0_DFPCI_READ32(CTRL, RegVal);
    SATA0_DFPCI_SET_FLD(CTRL, SEC_CHANNEL_EN, SATA0_CTRL_0_SEC_CHANNEL_EN_YES, RegVal);
    SATA0_DFPCI_SET_FLD(CTRL, PRI_CHANNEL_EN, SATA0_CTRL_0_PRI_CHANNEL_EN_YES, RegVal);
    SATA0_DFPCI_SET_FLD(CTRL, ADMA_SPACE_EN, SATA0_CTRL_0_ADMA_SPACE_EN_YES, RegVal);
    SATA0_DFPCI_SET_FLD(CTRL, CH3_EN, SATA0_CTRL_0_CH3_EN_YES, RegVal);
    SATA0_DFPCI_WRITE32(CTRL, RegVal);
}

static void EnableBARs()
{
    NvU32 RegVal = 0;

    RegVal = 0xFFFFFFFF;
    if (!((gs_SataContext.SataMode == NvBootSataMode_Ahci) &&
        (gs_SataContext.XferMode == NvBootSataTransferMode_Dma)))
    {

        SATA0_DFPCI_WRITE32(CFG_4, RegVal);
        SATA0_DFPCI_READ32(CFG_4, RegVal);      
        RegVal = 0xFFFFFFFF;
        SATA0_DFPCI_WRITE32(CFG_5, RegVal);
    }
    SATA0_DFPCI_WRITE32(CFG_9, RegVal);

    // Step 15(c)
    if (!((gs_SataContext.SataMode == NvBootSataMode_Ahci) &&
        (gs_SataContext.XferMode == NvBootSataTransferMode_Dma)))
    {
        RegVal = 0x000001F0;
        SATA0_DFPCI_WRITE32(CFG_4, RegVal);
        // Step 16 Program the BARs with upper 20 bits of the address
        // Step 16(a)
        IPFS_REG_WRITE_FLD(SATA, FPCI_BAR0, FPCI_BAR0_ACCESS_TYPE, 0, RegVal);

        RegVal = 0x000003F4;
        SATA0_DFPCI_WRITE32(CFG_5, RegVal);
        
        // Step 16 Program the BARs with upper 20 bits of the address
        // Step 16(b)
        IPFS_REG_WRITE_FLD(SATA, FPCI_BAR1, FPCI_BAR1_ACCESS_TYPE, 0, RegVal);
    }

    RegVal = 0x40020000;
    SATA0_DFPCI_WRITE32(CFG_9, RegVal);

    // Step 16(c)
    IPFS_REG_WRITE_FLD(SATA, FPCI_BAR5, FPCI_BAR5_START, 0x40020, RegVal);
    IPFS_REG_WRITE_FLD(SATA, FPCI_BAR5, FPCI_BAR5_ACCESS_TYPE, 1, RegVal);

}

static void
UpdateClasscode()
{
    NvU32 RegVal = 0;

    // Step 24(b)Enable Class code Backdoor programming
    SATA0_DFPCI_READ32(CFG_SATA, RegVal);
    SATA0_DFPCI_SET_FLD(CFG_SATA, BACKDOOR_PROG_IF_EN, SATA0_CFG_SATA_0_BACKDOOR_PROG_IF_EN_YES, RegVal);
    SATA0_DFPCI_WRITE32(CFG_SATA, RegVal);

    // Step 24(c) Program Class code
    SATA0_DFPCI_READ32(BKDOOR_CC, RegVal);
    SATA0_DFPCI_SET_FLD(BKDOOR_CC, CLASS_CODE, 0x0106, RegVal);
    SATA0_DFPCI_SET_FLD(BKDOOR_CC, PROG_IF, SATA0_BKDOOR_CC_0_PROG_IF__NOPRDCHK, RegVal);
    SATA0_DFPCI_WRITE32(BKDOOR_CC, RegVal);
}

static void
AhciEnable()
{
    NvU32 RegValue = 0;
    // Step 24(d) Indicate that SW is ahci aware
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, HBA_GHC,
        AE, AHCI_HBA_GHC_0_AE_YES,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0, RegValue);
    // 1ms suggested. 10ms provided for test purposes
#if NVBOOT_TARGET_RTL
    NvBootUtilWaitUS(10);
#else
    NvBootUtilWaitUS(1000);
#endif
#if 0
    // Step 24(e) Reset all state machines
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, HBA_GHC,
        HR, AHCI_HBA_GHC_0_HR_TRUE,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0, RegValue);
     // Step 24(f) Wait for reset to complete
     do
     {
         RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0);
     } while(RegValue & AHCI_HBA_GHC_0_HR_FIELD);

    // Step 24(g) Enable AHCI
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, HBA_GHC,
        AE, AHCI_HBA_GHC_0_AE_YES,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0, RegValue);
#endif
}

static NvU32
PortImplemented()
{
    NvU32 RegValue = 0;
    // Step 24(h) Find the port implemented
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_PI_0);
    return RegValue;
}

static NvU32
GetNumCmdSlots()
{
    NvU32 RegValue = 0;
    // Step 24(i) Find the number of command slots implemented
    // This driver needs only 1 command slot
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_CAP_0);
    RegValue= NV_DRF_VAL(AHCI, HBA_CAP, NCS, RegValue);
    return RegValue;
}

static NvBootError
CheckPortCommandErr()
{
    NvBootError Error = NvBootError_NotInitialized;
    NvU32 RegValue = 0;
    NvU32 Cr = 0;
    NvU32 Fr = 0;
    NvU32 Fre = 0;
    NvU32 St = 0;
    // Step 24(j) Check for errors
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCMD_0);
    Cr = NV_DRF_VAL(AHCI, PORT_PXCMD, CR, RegValue);
    Fr = NV_DRF_VAL(AHCI, PORT_PXCMD, FR, RegValue);
    Fre = NV_DRF_VAL(AHCI, PORT_PXCMD, FRE, RegValue);
    St = NV_DRF_VAL(AHCI, PORT_PXCMD, ST, RegValue);
    if ((Cr != AHCI_PORT_PXCMD_0_CR_NOTRCVD) ||
        (Fr != AHCI_PORT_PXCMD_0_FR_NOTRCVD) ||
        (Fre != AHCI_PORT_PXCMD_0_FRE_CLEAR) ||
        (St != AHCI_PORT_PXCMD_0_ST_CLEAR))
    {
        SET_SATA_BOOT_INFO_BITFLD(PortStatus, SataStatus_PortBsy);
        goto fail;
    }
    Error = NvBootError_Success;
fail:
    return Error;
}


static NvBootError
WaitForCommandCompletion()
{
    NvU32 Timeout =  NVBOOT_SATA_COMMAND_COMPLETE_TIMEOUT;
    NvU32 RegValue = 0;
    NvBootError Error = NvBootError_HwTimeOut;

    do
    {
        NvBootUtilWaitUS(1);
        Timeout --;
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START +
                AHCI_PORT_PXCI_0);
        RegValue = NV_DRF_VAL(AHCI, PORT_PXCI, CI0, RegValue);
    }while ((RegValue) && (Timeout));
    if (!RegValue)
    {
        Error = NvBootError_Success;
    }

    return Error;

}

static NvBootError
WaitForPortBusy()
{
    NvU32 Cr = 0;
    NvU32 TimeOut = NVBOOT_SATA_BUSY_TIMEOUT;
    NvBootError Error = NvBootError_Busy;
    NvU32 RegValue = 0;

    // Check PxCMD CR bit. (Command list running)
    do
    {
        NvBootUtilWaitUS(1);
        TimeOut--;

        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCMD_0);
        Cr = NV_DRF_VAL(AHCI, PORT_PXCMD, CR, RegValue);

    }while(Cr & TimeOut);
    if ((!TimeOut) && (Cr))
    {
        SET_SATA_BOOT_INFO_BITFLD(PortStatus, SataStatus_CommandListRunning);
        goto fail;
    }


    // Wait for BSY and DRQ to go low
    TimeOut = NVBOOT_SATA_BUSY_TIMEOUT;
    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    do
    {
        NvBootUtilWaitUS(1);
        TimeOut--;

        //Poll for Status Register
        RegValue = NV_READ32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_1);
        // Can't use DRF macros to read STS_DRQ and STS_BSY fields
        // bit 27 is STS_DRQ and bit 31 is STS_BSY
    }while((RegValue & 0x88000000) && (TimeOut));

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    if ((!TimeOut) && (RegValue & 0x88000000))
    {
        if (RegValue & 0x08000000)
            SET_SATA_BOOT_INFO_BITFLD(PortStatus, SataStatus_DrqHigh);
        if (RegValue & 0x80000000)
            SET_SATA_BOOT_INFO_BITFLD(PortStatus, SataStatus_BsyHigh);

        goto fail;

    }


    Error = NvBootError_Success;
fail:
    return Error;
}

static void
ProgramCommandAndFISReceive()
{
    NvU32 RegValue = 0;
    // Step 24(k) Program Command list and FIS receive memory regions
    RegValue = (NvU32)gs_SataContext.SataBuffersBase;
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCLB_0, RegValue);

    RegValue = (NvU32)gs_SataContext.FisBase;
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXFB_0, RegValue);

}

static void
ClearPortErrors()
{
    NvU32 RegValue = 0;
    // Step 24(l) Set FRE to 1
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCMD_0);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXCMD,
        FRE, AHCI_PORT_PXCMD_0_FRE_SET,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCMD_0, RegValue);

    // Step 24(m) Clear all valid  Err bits
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXSERR_0);

    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        ERR_I, AHCI_PORT_PXSERR_0_ERR_I_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        ERR_M, AHCI_PORT_PXSERR_0_ERR_M_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        ERR_T, AHCI_PORT_PXSERR_0_ERR_T_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        ERR_C, AHCI_PORT_PXSERR_0_ERR_C_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        ERR_P, AHCI_PORT_PXSERR_0_ERR_P_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        ERR_E, AHCI_PORT_PXSERR_0_ERR_E_CLEAR,
        RegValue);

    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_N, AHCI_PORT_PXSERR_0_DIAG_N_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_I, AHCI_PORT_PXSERR_0_DIAG_I_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_W, AHCI_PORT_PXSERR_0_DIAG_W_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_B, AHCI_PORT_PXSERR_0_DIAG_B_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_D, AHCI_PORT_PXSERR_0_DIAG_D_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_C, AHCI_PORT_PXSERR_0_DIAG_C_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_H, AHCI_PORT_PXSERR_0_DIAG_H_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_S, AHCI_PORT_PXSERR_0_DIAG_S_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_T, AHCI_PORT_PXSERR_0_DIAG_T_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_F, AHCI_PORT_PXSERR_0_DIAG_F_CLEAR,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSERR,
        DIAG_X, AHCI_PORT_PXSERR_0_DIAG_X_CLEAR,
        RegValue);

    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXSERR_0, RegValue);

}

static NvBootError
IssueComReset()
{
    // There may be a second COMRESET required in case the first one is sent
    // too early. In case the COMRESET doesn't finish in 1 sec, send another
    // COMRESET.
    NvU32 RegValue = 0;
    NvBootError Error = NvBootError_DeviceNotResponding;
    NvU64 TimeOut = 200 * 1000 * (s_SataFuseParamData.ComInitWaitIndex + 1);
    NvU32 ComResetsSent = 0;
    NvBool ComInit = NV_FALSE;

#if 1
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXTFD_0);
        NvBootUtilWaitUS(1);
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXTFD_0);
        // Test read for PxSSTS
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSSTS);
#endif
    do {
        // Step 20 Issue COMRESET
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSCTL);
        // [3:0] are DET bits
        RegValue = RegValue | 0xF;
        RegValue = RegValue & (0xFFFFFFF0 | T_AHCI_PORT_PXSCTL_DET_INTF_INIT);
        NV_WRITE32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSCTL,
            RegValue);

        // Step 21
        // A 1ms delay
#if NVBOOT_TARGET_RTL
        NvBootUtilWaitUS(1);
#else
        NvBootUtilWaitUS(1000);
#endif
        // [3:0] are DET bits
        RegValue = RegValue & (0xFFFFFFF0);// | (~T_AHCI_PORT_PXSCTL_DET_NO_DEV_RQD));
        NV_WRITE32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSCTL,
            RegValue);
        // Step 22 Poll PxSSTS.DET until the returned data is 0x3
        while (TimeOut)
        {
            RegValue = NV_READ32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSSTS);
            if ((RegValue & 0x3) == 0x3)
            {
                // Device detected and PHY communication established
                Error = NvBootError_Success;
                ComInit = NV_TRUE;
                break;
            }
            #if NVBOOT_TARGET_RTL
            NvBootUtilWaitUS(20);
            TimeOut -= 20;
            #else
            NvBootUtilWaitUS(20000);
            TimeOut -= 20000;
            #endif
        }
        if (ComInit)
            break;
        ComResetsSent++;
    } while(ComResetsSent < s_SataFuseParamData.NumComresetRetries);

    return Error;
}

static NvBootError AllocateHBAMemBuffers(const NvBootSataParams *Params)
{
    NvBootError error = NvBootError_IllegalParameter;
    NvU32 BuffAddr = 0;
    NvU32 HBAMemSizeBytes = 0;
    // In T210, for ahci dma reads through SATA controller, 
    // HBA Mem structures and Data buffers can be in IRAM or in DRAM
    // Pio mode also works well for either : IRAM and DRAM
    // Reader Buffers Base is now determined through the parameters provided
    // for Sata Buffers Base and Data Buffer Base. Therefore, we need to check
    // if Sata Buffers Base is a valid memory address and then initialize FIS base 
    // and Command List and Command Table base.
    // NOTE: If Sata Buffers Base is Zero, then it's not a valid DRAM/IRAM
    // address and the default/previous memory address for Sata Buffers Base will be used (which 
    // is a DRAM address and DRAM should therefore be initialized. If DRAM is not initialized,
    // then the caller of the function will have to set the Sata Buffers Base to an IRAM address)
    // If Data Buffers Base is set to 0, then the function NvBootSataGetReaderBuffersBase
    // will ensure that the reader code uses default reader buffers in IRAM.

    // Command List Base. BootROM populates only one command in Command List
    BuffAddr = (NvU32) Params->SataBuffersBase;
    HBAMemSizeBytes = ((MAX_COMMAND_LIST_SIZE + 0x80) & ~(0x7f) ) +
	                          ((MAX_COMMAND_TABLES_SIZE + 0x100) & ~(0xff)) +
	                          TOTAL_FIS_SIZE;
    // Check if Sata Buffers base is valid, If valid, then only assign FIS base and Data Buffers Base
    // (Valid IRAM address) or (Valid DRAM address and DRAM initialized)
    // This will allow HBA Mem to be allocated within the reader buffer area with page size less than 16K and
    // still be able to use ahci dma when BL is in IRAM and needs all space till NVBOOT_BL_IRAM_END to be reclaimed.
    //    if ( ( (BuffAddr >= NVBOOT_BL_IRAM_START) && ((BuffAddr+HBAMemSizeBytes) < NVBOOT_BL_IRAM_END) )  ||
    if ( ( (BuffAddr >= NVBOOT_LIMITS_ADDRESS_MAP_DATAMEM_BASE) && ((BuffAddr+HBAMemSizeBytes) <= NVBOOT_BL_IRAM_END) )  ||
        ( (BuffAddr >= NVBOOT_BL_SDRAM_START) && ((BuffAddr - NVBOOT_BL_SDRAM_START + HBAMemSizeBytes) <= NvBootSdramQueryTotalSize()) && 
          (BootInfoTable.SdramInitialized == NV_TRUE)) )
    {
        // Alignment of Command list should be 1KB. (Refer AHCI1.3 Specification)
        if (!((NvU32)BuffAddr & 0x3ff))
        {
		gs_SataContext.SataBuffersBase = Params->SataBuffersBase;

              gs_SataContext.CmdTableBase = gs_SataContext.SataBuffersBase +
                       MAX_COMMAND_LIST_SIZE;
		// Command table has to be 128byte aligned. (Refer AHCI1.3 Specification)
              gs_SataContext.CmdTableBase =  (gs_SataContext.CmdTableBase + 0x80) & ~(0x7f);
              gs_SataContext.FisBase = gs_SataContext.CmdTableBase +
                       MAX_COMMAND_TABLES_SIZE;
	       // FIS receive region has to be 256byte aligned. (Refer AHCI1.3 Specification)
		gs_SataContext.FisBase = (gs_SataContext.FisBase + 0x100) & ~(0xff);
		error = NvBootError_Success;
          }
	   else
	   {
	       error = NvBootError_MemoryNotAligned;
	   }
	   	
    	}
    return error;
}

NvBootError
NvBootSataInit(
    const NvBootSataParams *Params,
    NvBootSataContext *pSataContext)
{
    NvBootError Error = NvBootError_NotInitialized;
    NvU32 RegValue = 0;
    NvBool ParamsValid = NV_FALSE;

    /*
     *
     * 1. Hold SATA and SATA_OOB in reset.
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
    NV_ASSERT(Params);

    ParamsValid = NvBootSataValidateParams(Params);
    if (!ParamsValid)
    {
        Error = NvBootError_InvalidDevParams;
        goto end;
    }

    gs_SataContext.SataMode = Params->SataMode;
    gs_SataContext.XferMode = Params->TransferMode;
    gs_SataContext.PageSizeLog2 = SECTOR_SIZE_LOG2 + s_SataFuseParamData.PageSizeMultLog2;
    gs_SataContext.PagesPerBlock = 1 << (NVBOOT_SATA_BLOCK_SIZE_LOG2 - gs_SataContext.PageSizeLog2);
    gs_SataContext.SectorsPerPage = 1 << s_SataFuseParamData.PageSizeMultLog2;



     if ((gs_SataContext.SataMode == NvBootSataMode_Ahci) &&
         (gs_SataContext.XferMode == NvBootSataTransferMode_Dma))
     {
 
            if (AllocateHBAMemBuffers(Params) != NvBootError_Success)
            {
                // Fallback to legacy pio mode
	         gs_SataContext.SataMode = NvBootSataMode_Legacy;
                gs_SataContext.XferMode = NvBootSataTransferMode_Pio;
	         // Record it in the BIT			
            }
	 
     }

    // Valid Data Buffer base required for both modes: pio and dma
    // This check is done through function NvBootSataGetReaderBuffersBase
    gs_SataContext.DataBufferBase = Params->DataBufferBase;
    SET_SATA_BOOT_INFO_FLD(AhciSataBuffersBase, gs_SataContext.SataBuffersBase);
    SET_SATA_BOOT_INFO_FLD(AhciDataBuffersBase, gs_SataContext.DataBufferBase);    
			
    SET_SATA_BOOT_INFO_BITFLD(SataMode, (gs_SataContext.XferMode << 16));        

    s_AhciPxImplemented = 0;
    s_AhciPxCmdSlotsSupported = 0;

    // Once only Initialization of SATA controller
    // Step 1 This is warm reset.
    NvBootResetSetEnable(NvBootResetDeviceId_SataId, NV_TRUE);

    // There's no signal corresponding to SWR_SATA_OOB_RST. Therefore, don't
    // program SWR_SATA_OOB_RST
    NvBootResetSetEnable(NvBootResetDeviceId_SataOobId, NV_TRUE);

    // Cold reset bit for sata init
    NvBootResetSetEnable(NvBootResetDeviceId_SataColdRstId, NV_TRUE);

    // Step 2(a) SATA OOB clock should be 216MHz using PLLP_OUT0 as source
    ConfigureClockSource(NvBootClocksClockId_SataOobId, Params);

    //SATA controller clock should be 108MHz using PLLP_OUT0 as source
    // nvboot_clocks not yet tested for Sata clock source programming
    // Step 2(b)
    ConfigureClockSource(NvBootClocksClockId_SataId, Params);

    // Step 2(c) Enable Clocks to SATA controller and SATA OOB
    NvBootClocksSetEnable(NvBootClocksClockId_SataId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_SataAuxId, NV_TRUE);
    NvBootClocksSetEnable(NvBootClocksClockId_SataOobId, NV_TRUE);

    if ((!gs_SataContext.IsSataInitialized) || (Params->EnablePlleSS))
    {
        // Clear Reset for SATA USB UPHY (PAD PLL)
        RegValue = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_Y_SET_0);
        RegValue = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, RST_DEV_Y_SET,
                SET_SATA_USB_UPHY_RST, CLK_RST_CONTROLLER_RST_DEV_Y_SET_0_SET_SATA_USB_UPHY_RST_ENABLE, RegValue);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_RST_DEV_Y_SET_0, RegValue);

        // Release reset to XUSB_PADCTL
        NvBootResetSetEnable(NvBootResetDeviceId_XUsbId, NV_TRUE);
	
        // Step 6 Select internal CML ref clk
        // Select PLLE as input to IO phy
        // Register SATA_AUX_PAD_PLL_CNTL_1_0 is RESERVED in T210. 
        // REFCLK_SEL = SATA_AUX_PAD_PLL_CTRL_0_0[15:12], default 0(INT_CML)


        // Step 8 Enable PLLE - This code should go in nvboot_clocks.h
        Error = EnablePLLE(Params);
        if (Error)
        {
            // Don't return error. Record the error in the BIT.
            SET_SATA_BOOT_INFO_BITFLD(InitStatus, SataInitStatus_PlleInitFailed);
        }

        ProgramUPhyInit();

        ProgramUPhyPadPllRegs();

        // On T210, XUSB PAD control block needs to be brought out of reset
        // and to be programmed for SATA usage. These pad controls are common
        // for USB3, SATA and PCIE.
        // Placed this before SATA reset release and pad pll locking because:
        // a. these values consist of overrides and non-POR values for proper functionality. 
        // On top of these values, the progrmaming for calibration, pad pll enable and resistor calibration  
        // per PLL Sequencer document needs to be done.
        // a. no dependence of XUSB Pad controls on sata/sata oob clks/resets/config
        // b. PLLE is up and IDDQs programmed.
        // c. PadPhy iddq not required to be programmed.
        ProgramUPhyPadCntrlRegs();

        EnableUphyPadPll();        

        ProgramUPhyRegsAfterPadPllLock();
    }

    // Release cold reset
    NvBootResetSetEnable(NvBootResetDeviceId_SataColdRstId, NV_FALSE);

    //De-assert warm reset
    // need to de-assert warm reset to access sata controller registers
    NvBootResetSetEnable(NvBootResetDeviceId_SataId, NV_FALSE);

    // There's no signal corresponding to CLR_SATA_OOB_RST. Therefore, don't
    // program CLR_SATA_OOB_RST
    NvBootResetSetEnable(NvBootResetDeviceId_SataOobId, NV_FALSE);
    NvBootUtilWaitUS(62);

    // Step 12
    // Enable SATA FPCI accesses in SATA IPFS logic -
    // NV_ADDRESS_MAP_APB_SATA_BASE = NV_SATA_IPFS_REGS
    IPFS_REG_WRITE_FLD(SATA, CONFIGURATION, EN_FPCI, 1, RegValue);

    ProgramChPhyCtrl();
    ProgramSquelch();

    /*
      * SW WAR for bug#748271
      * Comma detection counter does not get reset while going from one link
      * speed to another during speed negotiation. Bug found in MCP89 post
      * silicon.
      * SW WAR enables the 40 bit ALIGN detection mode is required for GEN3
      * not to be detected as GEN1 drive.
      * In case of T30, GEN3 drives are not supported and in case they are
      * used, they should settle down to GEN2 speed.
      * In T210, this bug has been fixed. However, keeping this code subject to SwCya.
      */
     if (NvBootGetSwCYA() & NVBOOT_SW_CYA_SATA_WAR_BUG748271_EN)
     {
         SATA0_REF_DFPCI_READ32( CFG_PHY_0_0, RegValue);
         SATA0_DFPCI_SET_FLD(CFG_PHY_0, USE_7BIT_ALIGN_DET_FOR_SPD, SATA0_CFG_PHY_0_0_USE_7BIT_ALIGN_DET_FOR_SPD_NO, RegValue);
         SATA0_REF_DFPCI_WRITE32(CFG_PHY_0_0 , RegValue);

     }
     if (!gs_SataContext.IsSataInitialized)
     {

        // Enable AHB Redirection Controller to allow access to IRAM.
        NvBootArcEnable();
     }
    
    // Step 14 Enable the IO, MEM and ADMA space
    EnableSpaces();

    // Step 15(a) Program BARs 0,1 and 5
    EnableBARs();

#if NVBOOT_TARGET_RTL
    // Read back the value of the register for test purposes.
    RegValue = NV_READ32(NV_ADDRESS_MAP_MISC_BASE + SATA_AUX_MISC_CNTL_1_0);
#endif

#if NVBOOT_TARGET_RTL
    // Read back the value of the register for test purposes.
    RegValue = NV_READ32(NV_ADDRESS_MAP_MISC_BASE + SATA_AUX_RX_STAT_INT_0 );
#endif

    // Step 17. Initialize Extended Configuration Space Registers
    // For boot, only program GEN2_RX_RQ
    /*
     * For GEN1 drives,
     *  - Step 17(a) has no effect.
     *  - Step 17(b) is also not going to adversely affect
     *
     * For GEN2 drives
     *  - Step 17(a) is required
     *  - Step 17(b) : if fuse bit is high, then GEN1 will be forced else continue to negotiate as GEN2
    */
    // Step 17(a)
  
      if (s_SataFuseParamData.ForceGen1)
    {
        // Backdoor write to Implementation specifc bit in HBA_CAP register to set speed support to GEN1
        SATA0_DFPCI_READ32(AHCI_HBA_CAP_BKDR , RegValue);
        SATA0_DFPCI_SET_FLD(AHCI_HBA_CAP_BKDR, INTF_SPD_SUPP, SATA0_AHCI_HBA_CAP_BKDR_0_INTF_SPD_SUPP_GEN1, RegValue);
        SATA0_DFPCI_WRITE32(AHCI_HBA_CAP_BKDR , RegValue);

        // Force GEN1
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSCTL);
        RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXSCTL,
            SPD, T_AHCI_PORT_PXSCTL_SPD_GEN1,
            RegValue);
        NV_WRITE32(NV_SATA_APB_BAR5_START + T_AHCI_PORT_PXSCTL, RegValue);
    }

    ProgramPadCntrlRegisters();

    // Make sure SATA interrupts are not forwarded by SATA IPFS
    NV_WRITE32(NV_SATA_IPFS_REGS + SATA_INTR_MASK_0, 0x0);

    // Step 14(a) Disable NATIVE mode for Legacy PIO - after CFG 4,5,9 registers
    // otherwise CFG_4,5,9 registers will become non-writable and read as zeroes
    SATA0_DFPCI_READ32(CFG_SATA, RegValue);
    SATA0_DFPCI_SET_FLD(CFG_SATA, FORCE_NATIVE, SATA0_CFG_SATA_0_FORCE_NATIVE_DISABLED, RegValue);
    SATA0_DFPCI_WRITE32(CFG_SATA, RegValue);
#if NVBOOT_TARGET_RTL
    // Read back register for test purpose
    SATA0_DFPCI_READ32(CFG_SATA, RegValue);
#endif
    // Update class code for AHCI DMA mode only
    if ((gs_SataContext.SataMode == NvBootSataMode_Ahci) &&
    (gs_SataContext.XferMode == NvBootSataTransferMode_Dma))
    {
        UpdateClasscode();
        AhciEnable();
        s_AhciPxImplemented = PortImplemented();
        if (s_AhciPxImplemented != AHCI_HBA_PI_0_PI_FIRST)
        {
            Error = NvBootError_DeviceUnsupported;
            goto end;
        }
        s_AhciPxCmdSlotsSupported = GetNumCmdSlots();
        if ((s_AhciPxCmdSlotsSupported > AHCI_HBA_CAP_0_NCS_32))
        {
            Error = NvBootError_DeviceUnsupported;
            goto end;
        }

        Error = CheckPortCommandErr();
        if (Error)
            goto end;

        ProgramCommandAndFISReceive();
        ClearPortErrors();

    }

    //Disable Interrupts
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, HBA_GHC,
        IE, AHCI_HBA_GHC_0_IE_FALSE,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0, RegValue);
    // Read back register for test purposes
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_HBA_GHC_0);


    Error = IssueComReset();
    if (Error != NvBootError_Success)
    {
        SET_SATA_BOOT_INFO_BITFLD(InitStatus,SataInitStatus_ComResetFailed);
        goto end;
    }

    if ((gs_SataContext.SataMode == NvBootSataMode_Ahci) &&
    (gs_SataContext.XferMode == NvBootSataTransferMode_Dma))
    {
        ClearPortErrors();
        // Step 24(s)
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCMD_0);
        RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXCMD,
            ST, AHCI_PORT_PXCMD_0_ST_SET,
            RegValue);
        NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCMD_0, RegValue);
    }

    if (gs_SataContext.IsSataInitialized)
    {
        SET_SATA_BOOT_INFO_BITFLD(InitStatus,SataInitStatus_SataReinitialized);
    }

    gs_SataContext.IsSataInitialized = NV_TRUE;
    *pSataContext = gs_SataContext;

    Error = NvBootError_Success;

end:
    SET_SATA_BOOT_INFO_BITFLD(InitStatus, (NvU32)Error);

    return Error;
}

static
NvBootError NvBootSataLegacyPioRead(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest)
{

    NvU32 RegValue = 0;
    NvU32 SectorCount = 0;
    NvU32 LBALow = 0;
    NvU32 LBAMid = 0;
    NvU32 LBAHi = 0;
    NvU32 BytesToRead = 0;
    NvU32 TimeOut = 0;
    static NvBool s_IsSsdDetected = NV_FALSE;
    NvU32 *Addr = (NvU32 *)Dest;

    NvBootError e = NvBootError_Success;

    //Step 1: Enable the primary channel
    SATA0_DFPCI_READ32(CTRL, RegValue);
    SATA0_DFPCI_SET_FLD(CTRL, PRI_CHANNEL_EN, SATA0_CTRL_0_PRI_CHANNEL_EN_YES, RegValue);
    SATA0_DFPCI_WRITE32(CTRL, RegValue);

    // Step 2: Configure L2P Fifo depth to 0 - omitted
    // Regression test sets the fifo depth to 6 the default

    // Step 3: Set dev bit to 0 and set bit 6(ADDR_MODE) to 1
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    RegValue = 0x00400000;
    NV_WRITE32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_1, RegValue);

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    //Poll for Status Register
    // Wait for BSY and DRQ to go low
    TimeOut = NVBOOT_SATA_BUSY_TIMEOUT;
    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    do
    {
        NvBootUtilWaitUS(1);
        TimeOut--;

        RegValue = NV_READ32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_1);
        // bit 27 is STS_DRQ and bit 31 is STS_BSY
    }while((RegValue & 0x88000000) && (TimeOut));

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    if ((!TimeOut) && (RegValue & 0x88000000))
    {
        e = NvBootError_Busy;
        goto fail;
    }

    NvBootUtilWaitUS(10);

    // Step 4
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    RegValue = 0x00400000;
    NV_WRITE32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_1, RegValue);

    if (!s_IsSsdDetected)
    {
        // Step 5
        // Read Sector count, LBA Mid and LBA High to determine the type of drive
        // connected
        // ByteEnable = 0xB to read sector count.
        RegValue = NV_READ32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_0);
        SectorCount= NV_DRF_VAL(SATA, PRI_COMMAND_0, SECTOR_COUNT, RegValue);
        WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

        // ByteEnable = 0xE to read LBA mid.
        WriteByteEnable(NV_TRUE, ENABLE_BYTE0);
        // Can't use DRF macros here - because an _0 doesn't follow the
        RegValue = NV_READ32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_1);
        LBAMid= NV_DRF_VAL(SATA, PRI_COMMAND_1, LBA_MID, RegValue);
        WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

        // ByteEnable = 0xD to read LBA high.
        WriteByteEnable(NV_TRUE, ENABLE_BYTE1);
        RegValue = NV_READ32(NV_SATA_APB_BAR0_START + T_SATA_PRI_COMMAND_1);
        LBAHi= NV_DRF_VAL(SATA, PRI_COMMAND_1, LBA_HIGH, RegValue);
        WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);
        // This is to be done only once along with the comreset
        if ((SectorCount == 0x01) && (LBAMid == 0x00) && (LBAHi == 0x00))
        {
            s_IsSsdDetected = NV_TRUE;
        }
        else
        {
            SET_SATA_BOOT_INFO_BITFLD(InitStatus,SataInitStatus_SsdNotdetected);
            e = NvBootError_IdentificationFailed;
            goto fail;
        }
    }

    // Type of drive is SSD/HDD. Proceed.
    // Step 6 - Write Features register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE1);
    RegValue = NV_FLD_SET_DRF_NUM(SATA, PRI_COMMAND_0,
        FEATURE_ERR, 0x03,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_0, RegValue);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 7 - Write Sector Count register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    RegValue = NV_READ32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_0);
    // Read Page Size bytes.
    // Sector count needs to be calculated from Page size and Sector size
    RegValue = NV_FLD_SET_DRF_NUM(SATA, PRI_COMMAND_0,
        SECTOR_COUNT, gs_SataContext.SectorsPerPage,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_0, RegValue);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 8 - Write LBA Low and Mid register
    // Will not need to write LBA Hi register because of 32 bit addresses used.
    // Calculate LBALow and LBAMid
   // Calculate the Page number first
    RegValue = (Block * gs_SataContext.PagesPerBlock) + Page;
    // Calculate the sector number next
    RegValue = (RegValue * gs_SataContext.SectorsPerPage);

    LBALow = RegValue & 0xFF;
    LBAMid = ((RegValue & 0xFF00) >> 8);
    LBAHi = ((RegValue & 0xFF0000) >> 16);

    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    RegValue = NV_READ32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_0);
    // Set LBA Low
    RegValue = NV_FLD_SET_DRF_NUM(SATA, PRI_COMMAND_0,
        LBA_LOW, LBALow,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_0, RegValue);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);


    WriteByteEnable(NV_TRUE, ENABLE_BYTE0);
    RegValue = NV_READ32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_1);
    RegValue = RegValue & 0xFFFFFF00;
    RegValue = RegValue | LBAMid; //LBAMid is [7:0]
    NV_WRITE32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_1, RegValue);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 9 -Write LBAHi register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE1);
    RegValue = NV_READ32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_1);
    RegValue = RegValue & 0xFFFF00FF;
    RegValue = RegValue | LBAHi; //LBAHi is [15:8]
    NV_WRITE32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_1, RegValue);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);


    // Step 10 -Write Command register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    // Read command
    RegValue = 0;
    RegValue = NV_FLD_SET_DRF_NUM(SATA, PRI_COMMAND_1,
            COMMAND_STATUS, CMD_READ_SECTOR, RegValue);
    NV_WRITE32(NV_SATA_APB_BAR0_START +
        T_SATA_PRI_COMMAND_1, RegValue);


    // Poll for Status Register
    do
    {
        RegValue = NV_READ32(NV_SATA_APB_BAR0_START +
                T_SATA_PRI_COMMAND_1);
    }while((RegValue & 0x48000000) != 0x48000000);

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Read data - The page size is the same as the sector size.
    BytesToRead = 1 << gs_SataContext.PageSizeLog2;
 
   // Byte enable is 0x0 for reading 32 bit data
   WriteByteEnable(NV_TRUE, ENABLE_ALL_BYTES);
   while (BytesToRead)
   {
      RegValue = NV_READ32(NV_SATA_APB_BAR0_START +
          T_SATA_PRI_COMMAND_0);
      *Addr = RegValue;
      Addr += 1;
      BytesToRead -= 4;
   }

fail:
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);
    return e;
}

static void
CreateCommandList()
{
    NvU32 RegValue = 0;
    NvU32 BytesToRead = 1 << gs_SataContext.PageSizeLog2;

    // Step 24(u) Create Command List with 1 command
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CMD0,
        DWORD0_PRDTL, MAX_PRDT_ENTRIES,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CMD0,
        DWORD0_CFL, NVBOOT_SATA_CFIS_0_WORD_COUNT,
        RegValue);
    NV_WRITE32(gs_SataContext.SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD0),
                    RegValue);

    NV_WRITE32(gs_SataContext.SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD1),
                    BytesToRead);

    NV_WRITE32(gs_SataContext.SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD2),
                    gs_SataContext.CmdTableBase);

    // Set Command Table Base higher 32 bit to 0    
    NV_WRITE32(gs_SataContext.SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD3),
                    0);
    // Set all reserved words to 0 - Not required here because Only first four DWORDs 
    // in Command List processed

}

static void
CreateCommandTable(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest)
{
    NvU32 RegValue = 0;
    NvU32 BytesToRead = 1 << gs_SataContext.PageSizeLog2;
    NvU32 LBAToReadFrom = 0;

    // Step 24(v) Create CFIS - H2D FIS

    // Command will be MCP_CMD_READ_DMA
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD0_COMMAND, CMD_READ_DMA,
        RegValue);
    // Indicates register update is due to Command Register (PxCMD)
    // If this field is set to 0, indicated register update is due to Device Control Register
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD0_C, 1,
        RegValue);
    // Set FIS type to 0x27
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD0_FISTYPE, 0x27,
        RegValue);

    NV_WRITE32(gs_SataContext.CmdTableBase +
                BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD0),
                    RegValue);

   // Calculate the Page number first
    LBAToReadFrom = (Block * gs_SataContext.PagesPerBlock) + Page;
    // Calculate the sector number next
    LBAToReadFrom = (LBAToReadFrom * gs_SataContext.SectorsPerPage);
    RegValue = 0;
    /*
    LBALow = RegValue & 0xFF;
    LBAMid = ((RegValue & 0xFF00) >> 8);
    LBAHi = ((RegValue & 0xFF0000) >> 16);
    LBALowExp = ((RegValue & 0xFF000000) >> 24);
    */
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD1_LBA_LOW, LBAToReadFrom & 0xFF,
        RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD1_LBA_MID, ((LBAToReadFrom & 0xFF00) >> 8),
    RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD1_LBA_HIGH, ((LBAToReadFrom & 0xFF0000) >> 16),
        RegValue);

    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD1_DEVICE, (1 << 6),
        RegValue);

    NV_WRITE32(gs_SataContext.CmdTableBase +
                BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD1),
                    RegValue);
    RegValue = 0;
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD2_LBA_LOW_EXP, ((LBAToReadFrom & 0xFF000000) >> 24),
        RegValue);
    NV_WRITE32(gs_SataContext.CmdTableBase +
                BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD2),
                    RegValue);

    RegValue = 0;

    // Sector count is set to number of sectors in a page because a page fill
    // of data is being read.
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
        DWORD3_SECTOR_COUNT, gs_SataContext.SectorsPerPage,
        RegValue);
    NV_WRITE32(gs_SataContext.CmdTableBase +
                BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD3),
                    RegValue);

    NV_WRITE32(gs_SataContext.CmdTableBase +
                BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD4),
                    0);
    // CFIS can be from 5 to 16 DWORDs- therefore, set DWORD5-DWORD15 should
    // ideally be set to 0's. But only 5 DWORDs for CFIS are programmed in Command
    // List

    // Create ACMD
    // bit A in command header is not set. Therefore, this region of
    // of command table is not relevant.
    // No need to set it to zeroes

    // Create PRDT
    NV_WRITE32(gs_SataContext.CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                 BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD0),
                    (NvU32)(Dest));
    RegValue = 0;
    NV_WRITE32(gs_SataContext.CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD1),
                    RegValue);
    NV_WRITE32(gs_SataContext.CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD2),
                    RegValue);
    // Bit 0 should be 1 to indicate an even byte count,  value of 1 means 2
    // bytes, 3 means 4 bytes etc. A maximum length of 4MB can be specified per
    // PRD table.
    // Page size can't exceed 4KB in the case of T30-SATA driver, therefore, 1
    // PRDT entry is enough
    // Interrupt on Completion (bit 31) set to 0
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, PRDT0, DWORD3_IOC, 1, RegValue);
    RegValue = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, PRDT0, DWORD3_DBC, (BytesToRead - 1), RegValue);    
    NV_WRITE32(gs_SataContext.CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD3),
                    RegValue);
}

static void
IssueAhciCommand()
{
    NvU32 RegValue = 0;

    // Step 24(w) Issue the AHCI command
    RegValue = NV_READ32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCI_0);
    RegValue = NV_FLD_SET_DRF_NUM(AHCI, PORT_PXCI,
        CI0, 0x1,
        RegValue);
    NV_WRITE32(NV_SATA_APB_BAR5_START + AHCI_PORT_PXCI_0, RegValue);
}

static NvBootError
WaitForDataTransferComplete()
{
    NvU32 Timeout =  NVBOOT_SATA_DATA_TRANSFER_TIMEOUT;
    NvU32 RegValue = 0;
    NvBootError Error = NvBootError_HwTimeOut;

    do
    {
        NvBootUtilWaitUS(1);
        Timeout --;
        RegValue = NV_READ32(NV_SATA_APB_BAR5_START +
                AHCI_PORT_PXIS_0);
        RegValue= NV_DRF_VAL(AHCI, PORT_PXIS, DPS, RegValue);
    }while ((!RegValue) && (Timeout));
    if (RegValue)
    {
        Error = NvBootError_Success;
    }

    return Error;

}

static NvBootError
CheckDataXmissionErrors()
{
    NvU32 RegValue = 0;
    NvBootError Error = NvBootError_HwTimeOut;

    /* Check for the following errors:
     * Task File Error
     * Host Bus Fatal Error
     * Host Bus Data Error
     * Interface Fatal Error
     * Interface Non Fatal Error
     * Overflow Error
     */
    NvU32 ErrorMask = (1 << AHCI_PORT_PXIS_0_TFES_SHIFT) |
                        (1 << AHCI_PORT_PXIS_0_HBFS_SHIFT) |
                        (1 << AHCI_PORT_PXIS_0_HBDS_SHIFT) |
                        (1 << AHCI_PORT_PXIS_0_IFS_SHIFT) |
                        (1 << AHCI_PORT_PXIS_0_INFS_SHIFT) |
                        (1 << AHCI_PORT_PXIS_0_OFS_SHIFT);

    RegValue = NV_READ32(NV_SATA_APB_BAR5_START +
            AHCI_PORT_PXIS_0);
    if (!(RegValue & ErrorMask))
        Error = NvBootError_Success;

    return Error;
}

static NvBootError
CheckTaskFileDataError()
{
    NvU32 RegValue = 0;
    NvBootError Error = NvBootError_Success;

    RegValue = NV_READ32(NV_SATA_APB_BAR5_START +
            AHCI_PORT_PXTFD_0);
    RegValue= NV_DRF_VAL(AHCI, PORT_PXTFD, STS_ERR, RegValue);

    if (RegValue)
    {
        // Status shows that there's an error.
        // Set Bit Status for Task File data error
        RegValue= NV_DRF_VAL(AHCI, PORT_PXTFD, STS_ERR, RegValue);
        Error = NvBootError_DeviceError;
    }

    return Error;

}
static
NvBootError NvBootSataAhciDmaRead(
    const NvU32 Block,
    const NvU32 Page,
    NvU8 *Dest)
{

    NvBootError Error = NvBootError_NotInitialized;
    // Buffer should be word aligned and should be a valid memory 
    // address in IRAM or DRAM
    // This is guaranteed by function NvBootSataGetReaderBuffersBase
    // which is called by Reader code and the buffer addresses are 
    // configured in the function NvBootSataInit
    NV_ASSERT(!((NvU32)(Dest) & 0xF));

    Error = WaitForPortBusy();
    if (Error)
        goto fail;

    // Specifically clear DIAGX
    ClearPortErrors();

    CreateCommandList();

    CreateCommandTable(Block, Page, Dest);

    IssueAhciCommand();

    Error = WaitForDataTransferComplete();
    if (Error)
    {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciDmaNotComplete);
        goto fail;
    }

    Error = WaitForCommandCompletion();
    if (Error)
    {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_DmaCmdNotComplete);
        goto fail;
    }
    Error = CheckDataXmissionErrors();
    if (Error)
    {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciDataXmissionError);
        goto fail;
    }

    Error = CheckTaskFileDataError();
    if (Error)
    {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciTfdError);
        goto fail;
    }

    Error = NvBootError_Success;

fail:
    return Error;

}


NvBootError NvBootSataReadPage(const NvU32 Block, const NvU32 Page, NvU8 *Dest)
{
    NvBootError e = NvBootError_IllegalParameter;

    NV_ASSERT(Dest != NULL);
    NV_ASSERT(Page < gs_SataContext.PagesPerBlock);
    // ASSUMPTION : The mode is already configured and reads as per the mode
    // may proceed directly
    // Only Legacy PIO and AHCI DMA are supported
    if ((gs_SataContext.SataMode == NvBootSataMode_Legacy)
        && (gs_SataContext.XferMode == NvBootSataTransferMode_Pio))
    {
        e = NvBootSataLegacyPioRead(Block, Page, Dest);
    }
    else if ((gs_SataContext.SataMode == NvBootSataMode_Ahci)
        && (gs_SataContext.XferMode == NvBootSataTransferMode_Dma))
    {
        e = NvBootSataAhciDmaRead(Block, Page, Dest);
    }
    if (e == NvBootError_Success)
    {
        SET_SATA_BOOT_INFO_FLD(LastBlockRead, Block);
        SET_SATA_BOOT_INFO_FLD(LastPageRead, Page);
        SET_SATA_BOOT_INFO_FLD(NumPagesRead, GET_SATA_BOOT_INFO_FLD(NumPagesRead) + 1);
    }
    return e;
}

NvBootDeviceStatus NvBootSataQueryStatus(void)
{
    // This API doesn't have any significance for bootrom SATA driver.
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


NvBootError NvBootSataGetReaderBuffersBase(NvU8** ReaderBuffersBase,
                            const NvU32 Alignment, const NvU32 Bytes)
{
/*
 * error = NotInitialized;
 * if (Parameter Pointer is NULL)
 *   if (Buffer allocated is within valid memory region)
 *     if (Buffer alignment restriction met)
 *       assign address to pointer;
 *       error = Success;
 *     else
 *       error = MemoryNotAligned
 *   else
 *     error = MemoryNotAllocated
 * else
 *     error = IllegalParameter
 *
 * return error
 */
    NvBootError error = NvBootError_NotInitialized;

    if (*ReaderBuffersBase == NULL)
    {
        /// No sata driver function should be called unless sata is initialized.
        /// A call to sata init function will ensure updated values for Sata Context and Data
        /// buffers address.
        if (gs_SataContext.IsSataInitialized)
        {
            NvU32 BuffAddr = (NvU32)(gs_SataContext.DataBufferBase);
            /// Check if address is valid.
            /// if (within range) then error = Success and set reader buffer base
            /// Check for IRAM allowed for BL
            /// <=NVBOOT_BL_IRAM_END because NVBOOT_BL_IRAM_END is not the last byte address in IRAM
            /// but equal to the size of IRAM (= IRAM D END + 1).
            if ( ( (BuffAddr >= NVBOOT_BL_IRAM_START) && ((BuffAddr+Bytes) <= NVBOOT_BL_IRAM_END) )  ||
            ( (BuffAddr >= NVBOOT_BL_SDRAM_START) && ((BuffAddr - NVBOOT_BL_SDRAM_START + Bytes) <=  NvBootSdramQueryTotalSize()) && 
              (BootInfoTable.SdramInitialized == NV_TRUE)) )
            {
                error = ((BuffAddr & (Alignment - 1)) == 0) ?  NvBootError_Success : NvBootError_MemoryNotAligned;
            }
            else
                error = NvBootError_MemoryNotAllocated;
    
            if (error == NvBootError_Success)
                *ReaderBuffersBase = (NvU8 *)(gs_SataContext.DataBufferBase);
        }
    }
    else
    {
        error = NvBootError_IllegalParameter;
    }
        
    return error;
}
