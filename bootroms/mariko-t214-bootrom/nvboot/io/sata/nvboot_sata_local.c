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
#include "nvboot_hardware_access_int.h"
#include "nvboot_clocks_int.h"

#include "nvboot_sata_local.h"
#include "nvboot_sata_ahci_struct.h"
#include "nvboot_sata_context.h"
#include "nvboot_sata_uphy.h"

#include "arsata.h"
// includes from hw/ap/manuals
#include "ardev_t_fpci_sata0.h"
#include "ardev_t_sata.h"
#include "ardev_t_ahci.h"

// Boot Info table. Used by SATA_BOOT_INFO macros
extern NvBootInfoTable BootInfoTable;

extern NvBootSataContext *SataContext;

static NvBootError LegacyPioPrelude(const NvU32 Block, const NvU32 Page);

static void
WriteByteEnable(NvBool ByteEnable, LegacyPioBe Val)
{
    IPFS_RMW_FLD_NUM(DFPCI_BEN, EN_DFPCI_BEN, ByteEnable);
    IPFS_RMW_FLD_NUM(DFPCI_BEN, DFPCI_BYTE_ENABLE_N, Val);
}

//Coding guidline for sata driver: All values in SataContext
//will be in terms of Syram mapped at 0x40000000. We will do the physical address
//conversion at the moment it enters an AHCI struct or AHCI register. Phy addresses
//should not be passed around, convert at the moment before write.
//however, BR passes in the Dest value already converted to a PHY address
static NvU32 IfSysramConvertToPhyAddr(NvU32 a)
{
    if ((a >= NVBOOT_BPMP_R5_SYSRAM) && (a < NVBOOT_BL_SYSRAM_END)) {
        return a - NVBOOT_SYSRAM_DIFFERENCE;
    } else {
        return a;
    }
}

static NvU32 IfSysramConvertToBpmpAddr(NvU32 a)
{
    if ((a >= NVBOOT_BPMP_CLIENT_SYSRAM) &&
        (a < NVBOOT_BPMP_CLIENT_SYSRAM + NV_ADDRESS_MAP_SYSRAM_0_IMPL_SIZE)) {
        return a + NVBOOT_SYSRAM_DIFFERENCE;
    } else {
        return a;
    }
}

NvBootError EnablePLLE(const NvBootSataParams *Params)
{
    // TBD: Error handling when Lock bit is not set
    NvBootError e = NvBootError_PllNotLocked;
    NvU32 StableTime = 0;

    if (Params->PlleUsePllRefe)
    {
        // Allow Plle to be driven from PllREFE and osc freq input to PllREFE

        //FOR NEXT CHIP: This should not have been commented out, but PLLREFE sourcing PLLE
        //is NEVER going to be used, so since it is close to tapeout, leaving bug it
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

    // Because PLLE runs through PllRefE for UPHY, analog rail needs to be on
    NvU32 PllRefEMisc = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_MISC_0);
    PllRefEMisc = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                     PLLREFE_MISC,
                                     PLLREFE_IDDQ,
                                     OFF,
                                     PllRefEMisc);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_MISC_0, PllRefEMisc);

    NvBootUtilWaitUS(5);

    // This mux in REFPLLE makes PLLE drive the UPHY Lane/PLL
    NvU32 temp = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_MISC_0);
    temp = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                              PLLREFE_MISC,
                              PLLREFE_SEL_CLKIN_PEX,
                              ENABLE,
                              temp);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLREFE_MISC_0, temp);

    /* Misc1 = VAL[PLLE_MISC_0]
     * Misc1[31:16] = PLLE_SETUP
     * Misc1[7:6] = PLLE_KCP
     * Misc1[5:4] = PLLE_VREG_BG_CTRL
     * Misc1[3:2] = PLLE_VREG_CTRL
     * Misc[0:0] = PLLE_KVCO
     * Misc2 = VAL[PLLE_AUX_0]
     * only changes bits PLLE_REF_SEL_PLLREFE, PLLE_REF_SRC, PLLE_CML1_OEN, PLLE_CML0_OEN
     */
    NvU32 PlleAux = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLE_AUX_0);
//TODO: check this over with PLL folks, manual is ambiguous
    PlleAux = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 PLLE_AUX,
                                 PLLE_REF_SRC,
                                 Params->PlleUsePllRefe,
                                 PlleAux);
    PlleAux = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 PLLE_AUX,
                                 PLLE_REF_SEL_PLLREFE,
                                 Params->PlleUsePllRefe,
                                 PlleAux);
    PlleAux = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 PLLE_AUX,
                                 PLLE_CML1_OEN,
                                 0x1,
                                 PlleAux);
    NvBootClocksStartPll(NvBootClocksPllId_PllE,
                         Params->PlleDivM,
                         Params->PlleDivN,
                         Params->PlleDivPlCml,
                         Params->PlleMisc,
                         PlleAux,
                         &StableTime);
    // TBD: If lock override is set, then delay of 300 us + 100 us to be provided here.
    WAIT_FOR_PLLLOCK(NvBootClocksPllId_PllE, StableTime, SataInitStatus_PlleInitFailed);

    return e;
}

void ConfigureClockSource(const NvBootClocksClockId Id, const NvBootSataParams *Params)
{
    NvBootSataClockSource ClockSource = NvBootSataClockSource_PllPOut0;
    uint8_t ClockDivider = 0;

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
        //Asserts agains getting here, quiet warning by inclusion
        default: break;
    }

    NvBootClocksConfigureClock(Id, 
                               ClockDivider,
                               ClockSource);
}

void ProgramSquelch()
{
    // Code snippet initially provided by Gaurav
    // NV_WRITE32(0x70021120,0x3328A5AC);

    // POR value programming, option to program SATA0_CFG_PHY_0_0_USE_7BIT_ALIGN_DET_FOR_SPD_NO
    // later subject to SwCya bit
    // Option to patch the value here.
    NvU32 CfgPhy = SATA0_READ32(CFG_PHY_0);
    SATA0_WRITE32(CFG_PHY_0, CfgPhy);

    // Code snippet initially provided by Gaurav
    // NV_WRITE32(0x70021114,0x4d000000);
    // POR value programming, Option to patch the value here.
    // Program Comma Count
    NvU32 NvOOB = SATA0_READ32(NVOOB);
    SATA0_WRITE32(NVOOB, NvOOB);
}

void EnableSpaces()
{
    // Steps 14(b) and 14(c) Enable IO and MEM space
    NvU32 Cfg1 = SATA0_READ32(CFG_1);
    Cfg1 = NV_FLD_SET_DRF_DEF(SATA0, CFG_1, IO_SPACE, ENABLED, Cfg1);
    Cfg1 = NV_FLD_SET_DRF_DEF(SATA0, CFG_1, MEMORY_SPACE, ENABLED, Cfg1);

    // Step 24(a) For AHCI mode, enable bus master
    if (SataContext->SataMode == NvBootSataMode_AhciDma) {
        Cfg1 = NV_FLD_SET_DRF_DEF(SATA0, CFG_1, BUS_MASTER, ENABLED, Cfg1);
    }
    SATA0_WRITE32(CFG_1, Cfg1);

    // Step 14(d) Enable ADMA space
    NvU32 Ctrl = SATA0_READ32(CTRL);
    Ctrl = NV_FLD_SET_DRF_DEF(SATA0, CTRL, SEC_CHANNEL_EN, YES, Ctrl);
    Ctrl = NV_FLD_SET_DRF_DEF(SATA0, CTRL, PRI_CHANNEL_EN, YES, Ctrl);
    Ctrl = NV_FLD_SET_DRF_DEF(SATA0, CTRL, ADMA_SPACE_EN, YES, Ctrl);
    Ctrl = NV_FLD_SET_DRF_DEF(SATA0, CTRL, CH3_EN, YES, Ctrl);
    SATA0_WRITE32(CTRL, Ctrl);
}

void EnableBARs()
{
    if (SataContext->SataMode == NvBootSataMode_LegacyPio) {
        SATA0_WRITE32(CFG_4, 0xFFFFFFFF);
        SATA0_WRITE32(CFG_5, 0xFFFFFFFF);
    }
    SATA0_WRITE32(CFG_9, 0xFFFFFFFF);

    // Step 15(c)
    if (SataContext->SataMode == NvBootSataMode_LegacyPio) {
        SATA0_WRITE32(CFG_4, 0x000001F0);
        // Step 16 Program the BARs with upper 20 bits of the address
        // Step 16(a)
        IPFS_RMW_FLD_NUM(FPCI_BAR0, FPCI_BAR0_ACCESS_TYPE, 0);

        SATA0_WRITE32(CFG_5, 0x000003F4);
        
        // Step 16 Program the BARs with upper 20 bits of the address
        // Step 16(b)
        IPFS_RMW_FLD_NUM(FPCI_BAR1, FPCI_BAR1_ACCESS_TYPE, 0);
    }
    SATA0_WRITE32(CFG_9, 0x40020000);

    // Step 16(c)
    IPFS_RMW_FLD_NUM(FPCI_BAR5, FPCI_BAR5_START, 0x40020);
    IPFS_RMW_FLD_NUM(FPCI_BAR5, FPCI_BAR5_ACCESS_TYPE, 1);
}

void UpdateClasscode()
{
    // Step 24(b)Enable Class code Backdoor programming
    SATA0_RMW_FLD(CFG_SATA, BACKDOOR_PROG_IF_EN, YES);

    // Step 24(c) Program Class code
    NvU32 BkdoorCC = SATA0_READ32(BKDOOR_CC);
    BkdoorCC = NV_FLD_SET_DRF_NUM(SATA0, BKDOOR_CC, CLASS_CODE, 0x0106, BkdoorCC);
    BkdoorCC = NV_FLD_SET_DRF_DEF(SATA0, BKDOOR_CC, PROG_IF, _NOPRDCHK, BkdoorCC);
    SATA0_WRITE32(BKDOOR_CC, BkdoorCC);
}

void AhciEnable()
{
    // Step 24(d) Indicate that SW is ahci aware
    AHCI_RMW_FLD(HBA_GHC, AE, YES);
    // 1ms suggested. 10us provided for test purposes
    NvBootUtilWaitUS(NVBOOT_TARGET_RTL ? 10 : 1000);
}

NvU32 PortImplemented()
{
    // Step 24(h) Find the port implemented
    return AHCI_READ32(HBA_PI);
}

NvU32 GetNumCmdSlots()
{
    // Step 24(i) Find the number of command slots implemented
    // This driver needs only 1 command slot
    return NV_DRF_VAL(AHCI, HBA_CAP, NCS, AHCI_READ32(HBA_CAP));
}

NvBootError CheckPortCommandErr()
{
    NvBootError Error = NvBootError_NotInitialized;
    NvU32 Cr = 0;
    NvU32 Fr = 0;
    NvU32 Fre = 0;
    NvU32 St = 0;
    // Step 24(j) Check for errors
    NvU32 PxCmd = AHCI_READ32(PORT_PXCMD);
    Cr = NV_DRF_VAL(AHCI, PORT_PXCMD, CR, PxCmd);
    Fr = NV_DRF_VAL(AHCI, PORT_PXCMD, FR, PxCmd);
    Fre = NV_DRF_VAL(AHCI, PORT_PXCMD, FRE, PxCmd);
    St = NV_DRF_VAL(AHCI, PORT_PXCMD, ST, PxCmd);
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
    NV_ASSERT((Timeout % 10) == 0);
    NV_ASSERT((Timeout % 100) == 0);
    NvU32 LoopDelayAmount = (NVBOOT_TARGET_RTL ? 10 : 100);

    NvBootError Error = NvBootError_HwTimeOut;

    NvU32 CommandIssue0;
    while (Timeout) {
        CommandIssue0 = AHCI_READ32(PORT_PXCI);
        if (NV_DRF_VAL(AHCI, PORT_PXCI, CI0, CommandIssue0) == 0) {
            Error = NvBootError_Success;
            break;
        }

        NvBootUtilWaitUS(LoopDelayAmount);
        Timeout -= LoopDelayAmount;
    }

    return Error;
}

static NvBootError WaitForPortBusy()
{
    // Poll for BSY and DRQ in Task File status to go low, timeout at
    // NVBOOT_SATA_BUSY_TIMEOUT, which must be divis by timeout step
    // Why the rigor? There were bugs here in t210 one due to 1us steps in timeout
    // So let's be careful, but still try to let the sim finish more quickly
    NvU32 TimeOut = NVBOOT_SATA_BUSY_TIMEOUT;
    NV_ASSERT((TimeOut % 10) == 0);
    NV_ASSERT((TimeOut % 100) == 0);
    NvU32 LoopDelayAmount = (NVBOOT_TARGET_RTL ? 10 : 100);

    NvBootError Error = NvBootError_Busy;

    NvU32 TaskFile;
    while(TimeOut) {
        TaskFile = AHCI_READ32(PORT_PXTFD);
        if (NV_DRF_VAL(AHCI, PORT_PXTFD, STS_BSY, TaskFile) == 0 &&
            NV_DRF_VAL(AHCI, PORT_PXTFD, STS_DRQ, TaskFile) == 0)
        {
            Error = NvBootError_Success;
            break;
        }

        NvBootUtilWaitUS(LoopDelayAmount);
        TimeOut -= LoopDelayAmount;
    }

    if (Error != NvBootError_Success)
    {
        if (NV_DRF_VAL(AHCI, PORT_PXTFD, STS_DRQ, TaskFile)) {
            SET_SATA_BOOT_INFO_BITFLD(PortStatus, SataStatus_DrqHigh);
        }
        if (NV_DRF_VAL(AHCI, PORT_PXTFD, STS_BSY, TaskFile)) {
            SET_SATA_BOOT_INFO_BITFLD(PortStatus, SataStatus_BsyHigh);
        }
    }
    return Error;
}

void ProgramCommandAndFISReceive()
{
    // Step 24(k) Program Command list and FIS receive memory regions
    AHCI_WRITE32(PORT_PXCLB,IfSysramConvertToPhyAddr((NvU32)SataContext->SataBuffersBase));

    AHCI_WRITE32(PORT_PXFB, IfSysramConvertToPhyAddr((NvU32)SataContext->FisBase));
}

void ClearPortErrors()
{
    // Step 24(l) Set FRE to 1
    AHCI_RMW_FLD(PORT_PXCMD, FRE, SET);

    // Step 24(m) Clear all valid  Err bits
    NvU32 SataErrors = AHCI_READ32(PORT_PXSERR);

    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, ERR_I, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, ERR_M, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, ERR_T, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, ERR_C, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, ERR_P, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, ERR_E, CLEAR, SataErrors);

    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_N, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_I, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_W, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_B, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_D, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_C, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_H, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_S, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_T, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_F, CLEAR, SataErrors);
    SataErrors = NV_FLD_SET_DRF_DEF(AHCI, PORT_PXSERR, DIAG_X, CLEAR, SataErrors);

    AHCI_WRITE32(PORT_PXSERR, SataErrors);
}

NvBootError IssueComReset(uint8_t ComInitWaitIndex, uint8_t ComresetRetries)
{
    // There may be a second COMRESET required in case the first one is sent
    // too early. In case the COMRESET doesn't finish in 1 sec, send another
    // COMRESET.
    NvBootError Error = NvBootError_DeviceNotResponding;
    NvU32 ComResetsSent = 0;
    NvBool ComInit = NV_FALSE;

    do {
        // Step 20 Issue COMRESET
        // Set DET to 1 to start COMRESET
        AHCI_RMW_FLD(PORT_PXSCTL, DET, INTF_INIT);

        // Step 21
        // A 1ms delay
        NvBootUtilWaitUS(NVBOOT_TARGET_RTL ? 1 : 1000);

        // Clear DET to COMRESET
        AHCI_RMW_FLD(PORT_PXSCTL, DET, NO_DEV_RQD);

        // Step 22 Poll PxSSTS.DET until the returned data is 0x3
        NvU64 TimeOut = 200 * 1000 * (ComInitWaitIndex + 1);
        while (TimeOut)
        {
            NvU32 Status = AHCI_READ32(PORT_PXSSTS);
            if (NV_DRF_VAL(AHCI, PORT_PXSSTS, DET, Status) ==
                    AHCI_PORT_PXSSTS_0_DET_DEV_PRSNT_PHY_COMM)
            {
                // Device detected and PHY communication established
                Error = NvBootError_Success;
                ComInit = NV_TRUE;
                break;
            }
            NvU32 LoopDelayAmount = (NVBOOT_TARGET_RTL ? 20 : 20000);
            NvBootUtilWaitUS(LoopDelayAmount);
            TimeOut -= (LoopDelayAmount);
        }
        if (ComInit)
            break;
        ComResetsSent++;
    } while(ComResetsSent < ComresetRetries);

    return Error;
}

NvBootError AllocateHBAMemBuffers(const NvBootSataParams *Params)
{
    NvBootError error = NvBootError_IllegalParameter;
    // In T186, for ahci dma reads through SATA controller HBA Mem structures
    // and Data buffers will be in SYSRAM. Pio mode also works for TCM in addition.
    // Reader Buffers Base is now determined through the parameters provided
    // for Sata Buffers Base and Data Buffer Base. Therefore, we need to check
    // if Sata Buffers Base is a valid memory address and then initialize FIS base 
    // and Command List and Command Table base.
    NvU32 BuffAddr = (NvU32) Params->SataBuffersBase;
    NvU32 HBAMemSizeBytes = ((MAX_COMMAND_LIST_SIZE + 0x80) & ~(0x7f) ) +
                             ((MAX_COMMAND_TABLES_SIZE + 0x100) & ~(0xff)) +
                             TOTAL_FIS_SIZE;
    // Check if Sata Buffers base is valid, If valid, then only assign FIS base and Data Buffers
    // Base (Valid IRAM address)
    // This will allow HBA Mem to be allocated within the reader buffer area with page size
    // less than 16K and still be able to use ahci dma when BL is in IRAM and needs all
    // space till NVBOOT_BL_IRAM_END to be reclaimed.
    if (((BuffAddr >= NVBOOT_BPMP_R5_SYSRAM) &&
            ((BuffAddr + HBAMemSizeBytes) <= NVBOOT_BL_SYSRAM_END)))
    {
        // Alignment of Command list should be 1KB. (Refer AHCI1.3 Specification)
        if (!((NvU32)BuffAddr & 0x3ff)) {
            SataContext->SataBuffersBase = Params->SataBuffersBase;

            NvU32 CmdListEnd = SataContext->SataBuffersBase + MAX_COMMAND_LIST_SIZE;
            // Command table has to be 128byte aligned. (Refer AHCI1.3 Specification)
            SataContext->CmdTableBase = (CmdListEnd + 0x80) & ~(0x7f);

            NvU32 CmdTblEnd = SataContext->CmdTableBase + MAX_COMMAND_TABLES_SIZE;
            // FIS receive region has to be 256byte aligned. (Refer AHCI1.3 Specification)
            SataContext->FisBase = (CmdTblEnd + 0x100) & ~(0xff);

            error = NvBootError_Success;
        } else {
            error = NvBootError_MemoryNotAligned;
        }
    }
    return error;
}

//Both Pio Reads and Write share a great deal of setup
//This should be refactored into a Setup Task file routine
//and an other stuff routine.
static NvBootError LegacyPioPrelude(const NvU32 Block, const NvU32 Page)
{
    //Step 1: Enable the primary channel
    SATA0_RMW_FLD(CTRL, PRI_CHANNEL_EN, YES);

    // Step 2: Configure L2P Fifo depth to 0 - omitted
    // Regression test sets the fifo depth to 6 the default

    // Step 3: Set dev bit to 0 and set bit 6(ADDR_MODE) to 1
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    SATA_PRI_WRITE32(COMMAND_1, 0x00400000);

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    //Poll for Status Register
    // Wait for BSY and DRQ to go low
    NvU32 TimeOut = NVBOOT_SATA_D2HFIS_TIMEOUT;
    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    //will contain unshifted high byte of PRI_COMMAND_1 aka command status
    NvU32 CommandStatus;
    do
    {
        NvBootUtilWaitUS(100);
        TimeOut-=100;

        CommandStatus = SATA_PRI_READ32(COMMAND_1);
        // bit 27 is STS_DRQ and bit 31 is STS_BSY
    }while((CommandStatus & 0x88000000) && (TimeOut));

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    if ((!TimeOut) && (CommandStatus & 0x88000000))
    {
        return NvBootError_Busy;
    }

    NvBootUtilWaitUS(10);

    // Step 4
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    SATA_PRI_WRITE32(COMMAND_1, 0x00400000);

    NvU32 LBAMid = 0;
    NvU32 LBAHi = 0;
    // Step 6 - Write Features register
    //FOR NEXT CHIP - This doesn't cause a bug, but is a little weird.
    //t210 actually reused an unrelated regval. This is better than that
    WriteByteEnable(NV_TRUE, ENABLE_BYTE1);
    SATA_PRI_WRITE32(COMMAND_0, NV_FLD_SET_DRF_NUM(SATA,
                                                   PRI_COMMAND_0,
                                                   FEATURE_ERR,
                                                   0x03,
                                                   0));
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 8 - Write LBA Low and Mid register
    // Will not need to write LBA Hi register because of 32 bit addresses used.
    // Calculate LBALow and LBAMid
    // Calculate the Page number first
    NvU32 PageNumber = (Block * SataContext->PagesPerBlock) + Page;
    // Calculate the sector number next
    NvU32 SectorNumber = (PageNumber * SataContext->SectorsPerPage);

    NvU32 LBALow = SectorNumber & 0xFF;
    LBAMid = ((SectorNumber & 0xFF00) >> 8);
    LBAHi = ((SectorNumber & 0xFF0000) >> 16);

    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    // Set LBA Low
    SATA_PRI_RMW_FLD_NUM(COMMAND_0, LBA_LOW, LBALow);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    WriteByteEnable(NV_TRUE, ENABLE_BYTE0);
    SATA_PRI_RMW_FLD_NUM(COMMAND_1, LBA_MID, LBAMid);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 9 -Write LBAHi register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE1);
    SATA_PRI_RMW_FLD_NUM(COMMAND_1, LBA_HIGH, LBAHi);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    return NvBootError_Success;
}

NvBootError NvBootSataLegacyPioRead(const NvU32 Block,
                                    const NvU32 Page,
                                    const NvU32 Length,
                                    uint8_t *Dest)
{
    NvBootError e = LegacyPioPrelude(Block, Page);
    if (e != NvBootError_Success) {
        goto fail;
    }

    // (Former Step 7, order for creating task file doesn't matter)
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    // Read Length can span multiple pages, we will round up to the nearest sector
    // Sector count needs to be calculated from Page size and Sector size
    NvU32 SectorsToRead = CEIL_SECTOR(Length);
    NvU32 BytesToRead = CEIL_SECTOR(Length) * SECTOR_SIZE;
    SATA_PRI_RMW_FLD_NUM(COMMAND_0, SECTOR_COUNT, SectorsToRead);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 10 -Write Command register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    // Read command
    SATA_PRI_WRITE32(COMMAND_1, NV_FLD_SET_DRF_NUM(SATA,
                                                   PRI_COMMAND_1,
                                                   COMMAND_STATUS,
                                                   CMD_READ_SECTOR,
                                                   0));
    // Poll for Status Register
    NvU32 CommandStatus;
    do {
        CommandStatus = SATA_PRI_READ32(COMMAND_1);
    } while((CommandStatus & 0x48000000) != 0x48000000);

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // DevMgr addresses are always physical, so if sysram, to BPMP Addr
    NvU32 *Addr = (NvU32 *)IfSysramConvertToBpmpAddr((NvU32)Dest);

    // Byte enable is 0x0 for reading 32 bit data
    WriteByteEnable(NV_TRUE, ENABLE_ALL_BYTES);
    while (BytesToRead)
    {
        *Addr = SATA_PRI_READ32(COMMAND_0);
        Addr += 1;
        BytesToRead -= 4;
    }

fail:
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);
    return e;
}

NvBootError NvBootSataLegacyPioWrite(const NvU32 Block, const NvU32 Page, uint8_t *Src)
{
    NvU32 BytesToRead = 0;
    NvBool Enb32BitWrites = NV_TRUE;
    
    NvBootError e = LegacyPioPrelude(Block, Page);
    if (e != NvBootError_Success) {
        goto fail;
    }

    // (Former Step 7, order for creating task file doesn't matter)
    // Write Sector Count register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE2);
    // Read Page Size bytes.
    // Sector count needs to be calculated from Page size and Sector size
    SATA_PRI_RMW_FLD_NUM(COMMAND_0, SECTOR_COUNT, SataContext->SectorsPerPage);
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Step 10 -Write Command register
    WriteByteEnable(NV_TRUE, ENABLE_BYTE3);
    // Write command
    SATA_PRI_WRITE32(COMMAND_1, NV_FLD_SET_DRF_NUM(SATA,
                                                   PRI_COMMAND_1,
                                                   COMMAND_STATUS,
                                                   CMD_WRITE_SECTOR,
                                                   0));

    // Poll for Status Register
    NvU32 CommandStatus;
    do
    {
        CommandStatus = SATA_PRI_READ32(COMMAND_1);
    }while((CommandStatus & 0x48000000) != 0x48000000);

    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);

    // Write data - The page size is the same as the sector size.
    BytesToRead = 1 << SataContext->PageSizeLog2;
    if (Enb32BitWrites)
    {
        NvU32 *Addr = (NvU32 *)IfSysramConvertToBpmpAddr((NvU32)Src);
        // Byte enable is 0x0 for reading 32 bit data
        WriteByteEnable(NV_TRUE, ENABLE_ALL_BYTES);
        while (BytesToRead)
        {
            SATA_PRI_WRITE32(COMMAND_0, *Addr);
            Addr += 1;
            BytesToRead -= 4;
        }
    }
    else
    {
        uint16_t *Addr = (uint16_t *)IfSysramConvertToBpmpAddr((NvU32)Src);
        // Byte enable is 0xc for reading 16 bit data
        WriteByteEnable(NV_TRUE, ENABLE_BYTE1_BYTE0);
        while (BytesToRead)
        {
            SATA_PRI_WRITE32(COMMAND_0, *Addr);
            Addr += 1;
            BytesToRead -= 2;
        }
    }

fail:
    WriteByteEnable(NV_FALSE, ENABLE_ALL_BYTES);
    return e;
}


static void
CreateCommandList(NvU32 BytesToRead)
{
    // Step 24(u) Create Command List with 1 command
    NvU32 DWord0 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CMD0,
                                      DWORD0_PRDTL, MAX_PRDT_ENTRIES,
                                      0);
    DWord0 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CMD0,
                                DWORD0_CFL, NVBOOT_SATA_CFIS_0_WORD_COUNT,
                                DWord0);
    NV_WRITE32(SataContext->SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD0),
               DWord0);

    NV_WRITE32(SataContext->SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD1),
               BytesToRead);

    NV_WRITE32(SataContext->SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD2),
               IfSysramConvertToPhyAddr(SataContext->CmdTableBase));

    // Set Command Table Base higher 32 bit to 0    
    NV_WRITE32(SataContext->SataBuffersBase + BITS_TO_BYTES(NVBOOT_SATA_CMD0_0_DWORD3), 0);
    // Set all reserved words to 0 - Not required here because Only first four DWORDs 
    // in Command List processed
}

static void
CreateCommandTable(
    const NvU32 Block,
    const NvU32 Page,
    const NvU32 SectorsToRead,
    const NvU32 BytesToRead,
    uint8_t *Dest)
{
    // Step 24(v) Create CFIS - H2D FIS
    // Command will be MCP_CMD_READ_DMA
    NvU32 CfisDword0 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                         DWORD0_COMMAND, CMD_READ_DMA,
                                         0);
    // Indicates register update is due to Command Register (PxCMD)
    // If this field is set to 0, indicated register update is due to Device Control Register
    CfisDword0 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                    DWORD0_C, 1,
                                    CfisDword0);
    // Set FIS type to 0x27
    CfisDword0 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                    DWORD0_FISTYPE, 0x27,
                                    CfisDword0);

    NV_WRITE32(SataContext->CmdTableBase +
                   BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD0),
               CfisDword0);

    // Calculate the Page number first
    NvU32 LBAToReadFrom = (Block * SataContext->PagesPerBlock) + Page;
    // Calculate the sector number next
    LBAToReadFrom = (LBAToReadFrom * SataContext->SectorsPerPage);
    /*
    LBALow = RegValue & 0xFF;
    LBAMid = ((RegValue & 0xFF00) >> 8);
    LBAHi = ((RegValue & 0xFF0000) >> 16);
    LBALowExp = ((RegValue & 0xFF000000) >> 24);
    */
    NvU32 CfisDword1 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                          DWORD1_LBA_LOW, LBAToReadFrom & 0xFF,
                                          0);
    CfisDword1 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                    DWORD1_LBA_MID, ((LBAToReadFrom & 0xFF00) >> 8),
                                    CfisDword1);
    CfisDword1 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                    DWORD1_LBA_HIGH, ((LBAToReadFrom & 0xFF0000) >> 16),
                                    CfisDword1);
    CfisDword1 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                   DWORD1_DEVICE, (1 << 6),
                                   CfisDword1);
    NV_WRITE32(SataContext->CmdTableBase +
                   BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD1),
               CfisDword1);

    NvU32 CfisDword2 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                          DWORD2_LBA_LOW_EXP,
                                          ((LBAToReadFrom & 0xFF000000) >> 24),
                                          0);
    NV_WRITE32(SataContext->CmdTableBase +
                   BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD2),
               CfisDword2);

    // Sector count is set to number of sectors in a page because a page fill
    // of data is being read.
    NvU32 CfisDword3 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, CFIS,
                                  DWORD3_SECTOR_COUNT, SectorsToRead,
                                  0);
    NV_WRITE32(SataContext->CmdTableBase +
                  BITS_TO_BYTES(NVBOOT_SATA_CFIS_0 + NVBOOT_SATA_CFIS_0_DWORD3),
               CfisDword3);

    NV_WRITE32(SataContext->CmdTableBase +
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
    NV_WRITE32(SataContext->CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                 BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD0),
                   IfSysramConvertToPhyAddr((NvU32)(Dest))); //maybe undo this

    NV_WRITE32(SataContext->CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                   BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD1),
               0);
    NV_WRITE32(SataContext->CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                   BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD2),
               0);
    // Bit 0 should be 1 to indicate an even byte count,  value of 1 means 2
    // bytes, 3 means 4 bytes etc. A maximum length of 4MB can be specified per
    // PRD table.
    // Page size can't exceed 4KB in the case of T30-SATA driver, therefore, 1
    // PRDT entry is enough
    // Interrupt on Completion (bit 31) set to 0
    NvU32 PrdtDword3 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA, PRDT0, DWORD3_IOC, 1, 0);
    PrdtDword3 = NV_FLD_SET_DRF_NUM(NVBOOT_SATA,
                                    PRDT0,
                                    DWORD3_DBC,
                                    (BytesToRead - 1),
                                    PrdtDword3);
    NV_WRITE32(SataContext->CmdTableBase + BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0) +
                   BITS_TO_BYTES(NVBOOT_SATA_PRDT0_0_DWORD3),
               PrdtDword3);
}

static void
IssueAhciCommand()
{
    // Step 24(w) Issue the AHCI command
    AHCI_RMW_FLD_NUM(PORT_PXCI, CI0, 0x1);
}

static NvBootError
WaitForDataTransferComplete()
{
    NvU32 Timeout =  NVBOOT_SATA_DATA_TRANSFER_TIMEOUT;
    NV_ASSERT((Timeout % 10) == 0);
    NV_ASSERT((Timeout % 100) == 0);
    NvU32 LoopDelayAmount = (NVBOOT_TARGET_RTL ? 10 : 100);

    NvBootError Error = NvBootError_HwTimeOut;

    // The DPS bit will set when all data is transfered
    while (Timeout) {
        NvU32 InterruptStatus = AHCI_READ32(PORT_PXIS);
        if (NV_DRF_VAL(AHCI, PORT_PXIS, DPS, InterruptStatus)) {
            Error = NvBootError_Success;
            break;
        }

        NvBootUtilWaitUS(LoopDelayAmount);
        Timeout -= LoopDelayAmount;
    }

    return Error;
}

static NvBootError
CheckDataXmissionErrors()
{
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

    NvU32 InterruptStatus = AHCI_READ32(PORT_PXIS);

    if (!(InterruptStatus & ErrorMask)) {
        Error = NvBootError_Success;
    }
    return Error;
}

static NvBootError
CheckTaskFileDataError()
{
    NvU32 TaskFileData = 0;
    NvBootError Error = NvBootError_Success;

    TaskFileData = AHCI_READ32(PORT_PXTFD);
    TaskFileData = NV_DRF_VAL(AHCI, PORT_PXTFD, STS_ERR, TaskFileData);

    if (TaskFileData) {
        // Status shows that there's an error.
        Error = NvBootError_DeviceError;
    }
    return Error;
}

NvBootError NvBootSataAhciDmaRead(const NvU32 Block,
                                  const NvU32 Page,
                                  const NvU32 Length,
                                  uint8_t *Dest)
{

    NvBootError Error = NvBootError_NotInitialized;
    // Buffer should be word aligned and should be a valid memory 
    // address in SYSRAM.
    NV_ASSERT(!((NvU32)(Dest) & 0xF));

    Error = WaitForPortBusy();
    if (Error)
        goto fail;

    // Specifically clear DIAGX
    ClearPortErrors();

    NvU32 SectorsToRead = CEIL_SECTOR(Length);
    NvU32 BytesToRead = CEIL_SECTOR(Length) * SECTOR_SIZE;

    CreateCommandList(BytesToRead);

    CreateCommandTable(Block,
                       Page,
                       SectorsToRead,
                       BytesToRead,
                       Dest);

    IssueAhciCommand();

    Error = WaitForDataTransferComplete();
    if (Error) {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciDmaNotComplete);
        goto fail;
    }

    Error = WaitForCommandCompletion();
    if (Error) {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_DmaCmdNotComplete);
        goto fail;
    }

    Error = CheckDataXmissionErrors();
    if (Error) {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciDataXmissionError);
        goto fail;
    }

    Error = CheckTaskFileDataError();
    if (Error) {
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciError);
        SET_SATA_BOOT_INFO_BITFLD(AhciDmaStatus, SataAhciStatus_AhciTfdError);
        goto fail;
    }

    Error = NvBootError_Success;

fail:
    return Error;
}
