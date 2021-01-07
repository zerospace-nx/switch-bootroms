/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "arclk_rst.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_sku_int.h"
#include "project.h"

/*
 * nvboot_sku.c: Implementation of the SKU API.
 */

void
NvBootSkuEnforceSku(NvBootSku_SkuId Sku)
{
#if 0
    // TODO: Update with the correct SKU implementation.

    // BOND_OUT registers are write to one only => no need for RMW
    switch (Sku)
    {
        case NvBootSku_IdeOn_PcieOn:
            // do nothing
            break;

        case NvBootSku_IdeOn_PcieOff:
            // Disable the PCIe controller
            NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE +
                       CLK_RST_CONTROLLER_BOND_OUT_U_0,
                       NV_DRF_NUM(CLK_RST_CONTROLLER,
                                  BOND_OUT_U,
                                  BOND_OUT_PCIE,
                                  1));
            break;

        case NvBootSku_IdeOff_PcieOn:
            // Disable the IDE controller
            NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE + 
                       CLK_RST_CONTROLLER_BOND_OUT_L_0,
                       NV_DRF_NUM(CLK_RST_CONTROLLER,
                                  BOND_OUT_L,
                                  BOND_OUT_IDE,
                                  1));
            break;

        case NvBootSku_IdeOff_PcieOff:
            // Disable the IDE controller
            NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE + 
                       CLK_RST_CONTROLLER_BOND_OUT_L_0,
                       NV_DRF_NUM(CLK_RST_CONTROLLER, 
                                  BOND_OUT_L,
                                  BOND_OUT_IDE,
                                  1));

            // Disable the PCIe controller
            NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE + 
                       CLK_RST_CONTROLLER_BOND_OUT_U_0,
                       NV_DRF_NUM(CLK_RST_CONTROLLER,
                                  BOND_OUT_U,
                                  BOND_OUT_PCIE,
                                  1));
            break;

        default:
            // Do nothing.
            ;
    }
#endif
}
