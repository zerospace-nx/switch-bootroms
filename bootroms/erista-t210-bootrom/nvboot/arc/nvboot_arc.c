/*
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "armc.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "project.h"

#define HW_REGR(d, p, r)  NV_READ32(NV_ADDRESS_MAP_##d##_BASE + p##_##r##_0)
#define HW_REGW(b, p, r, v) NV_WRITE32(b + p##_##r##_0, v)

extern NvBootInfoTable BootInfoTable;

static NvBool s_ArcEnabled;

void NvBootArcEnable(void)
{
    NvU32 RegData;
    if(!s_ArcEnabled)
    {
        // ARC enabling - see nvbug 1170178
     
        // Enable the clocks for EMC and MC
        NvBootClocksSetEnable(NvBootClocksClockId_EmcId, NV_TRUE);
        NvBootClocksSetEnable(NvBootClocksClockId_McId,  NV_TRUE);

        // Remove the EMC and MC controllers from Reset.
        // Must be done after clk enable to complete to reset correctly
        NvBootResetSetEnable(NvBootResetDeviceId_EmcId, NV_FALSE);
        NvBootResetSetEnable(NvBootResetDeviceId_McId,  NV_FALSE);
        
        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0);
        RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_EMC, 
                            EMC_2X_CLK_SRC, PLLP_OUT0, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_EMC_0, RegData); 

        NvBootUtilWaitUS(5); 

        // also enable ARC_CLK_OVR_ON of CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0
        // Bug 1239022

        RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                            CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0);

        //(1 << 19);
        RegData |= NV_DRF_NUM(CLK_RST_CONTROLLER, 
                            LVL2_CLK_GATE_OVRD, ARC_CLK_OVR_ON, 1);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + 
               CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0,
               RegData);

        // Enable ARC (AHB redirection ) path
        RegData = HW_REGR(MC, MC, IRAM_REG_CTRL);
        // update for ARC(AHB redirection ) path
        HW_REGW(NV_ADDRESS_MAP_MC_BASE, MC, IRAM_BOM, 
                (NV_ADDRESS_MAP_IRAM_A_BASE & MC_IRAM_BOM_0_WRITE_MASK));
        HW_REGW(NV_ADDRESS_MAP_MC_BASE, MC, IRAM_TOM, 
                (NV_ADDRESS_MAP_IRAM_D_LIMIT & MC_IRAM_TOM_0_WRITE_MASK));

        // Read MC register to flush all writes.
        RegData = HW_REGR(MC, MC, IRAM_REG_CTRL);
        s_ArcEnabled = 1;
    }   
}

void NvBootArcDisable(NvU32 TargetAddr)
{
    NvU32 RegData;

    if (BootInfoTable.SdramInitialized) {

        // Check if ARC was already locked-down
        RegData = HW_REGR(MC, MC, IRAM_REG_CTRL);
        if (NV_DRF_VAL(MC, IRAM_REG_CTRL, IRAM_CFG_WRITE_ACCESS, RegData) == \
            MC_IRAM_REG_CTRL_0_IRAM_CFG_WRITE_ACCESS_DISABLED )
        return;

        if ( (TargetAddr >= NV_ADDRESS_MAP_EMEM_LO_BASE) &&
             (TargetAddr < NV_ADDRESS_MAP_EMEM_LO_LIMIT) ) {
            // Disable ARC
            // Set MC_IRAM_TOM_0[IRAM_TOM] to 0x0000_0000
            // Set MC_IRAM_BOM_0[IRAM_BOM] to 0xFFFF_F000
            HW_REGW(NV_ADDRESS_MAP_MC_BASE, MC, IRAM_BOM, MC_IRAM_BOM_0_RESET_VAL);
            HW_REGW(NV_ADDRESS_MAP_MC_BASE, MC, IRAM_TOM, MC_IRAM_TOM_0_RESET_VAL);

            // Lock-down ARC
            RegData = NV_DRF_DEF(MC, IRAM_REG_CTRL, IRAM_CFG_WRITE_ACCESS, DISABLED);
            HW_REGW(NV_ADDRESS_MAP_MC_BASE, MC, IRAM_REG_CTRL, RegData);
            
            // Turn off override which un-gated lvl2 clk gate for ARC. Save Power!
            // Bug 1239022
            RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0);
            RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, LVL2_CLK_GATE_OVRD, ARC_CLK_OVR_ON, 0, RegData);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + 
            CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0,
            RegData);
            
        }
    }
}
