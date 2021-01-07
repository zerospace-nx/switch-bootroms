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
#include "arclk_rst.h"

const NvU32 g_TegraMux_Sdio4[] =
{
    CONFIGEND(),    // config 1 POPSDMMC4 set of pins not supported

    CONFIGEND(),    // config 2 - x8 on SDMMC4 set of pins
    BRANCH(3),
#if 0 //FIXME
    CONFIG(SDMMC4_DAT4,  SDMMC4, PULL_UP), // SDMMC4_DAT4 
    CONFIG(SDMMC4_DAT5,  SDMMC4, PULL_UP), // SDMMC4_DAT5
    CONFIG(SDMMC4_DAT6,  SDMMC4, PULL_UP), // SDMMC4_DAT6
    CONFIG(SDMMC4_DAT7,  SDMMC4, PULL_UP), // SDMMC4_DAT7
#endif
    CONFIGEND(),    // config 3 - x4 on SDMMC4 set of pins
#if 0 //FIXME
    CONFIG(SDMMC4_CLK,   SDMMC4, PULL_UP), // SDMMC4_CLK
    CONFIG(SDMMC4_CMD,   SDMMC4, PULL_UP), // SDMMC4_CMD
    CONFIG(SDMMC4_DAT0,  SDMMC4, PULL_UP), // SDMMC4_DAT0
    CONFIG(SDMMC4_DAT1,  SDMMC4, PULL_UP), // SDMMC4_DAT1
    CONFIG(SDMMC4_DAT2,  SDMMC4, PULL_UP), // SDMMC4_DAT2
    CONFIG(SDMMC4_DAT3,  SDMMC4, PULL_UP), // SDMMC4_DAT3
#endif

    CONFIGEND(),
    SUBROUTINESDONE(),
    MODULEDONE(),
};

// Use the new pinmux descriptions
const NvU32 g_TegraMux_Sflash[] =
{
    CONFIGEND(),

    CONFIG(QSPI_SCK,   QSPI, NONE), // QSPI_SCK    
    CONFIG(QSPI_CS_N,  QSPI, NONE), // QSPI_CS_N
    CONFIG(QSPI_IO0,   QSPI, NONE), // QSPI_IO0
    CONFIG(QSPI_IO1,   QSPI, NONE), // QSPI_IO1

    CONFIGEND(),
    MODULEDONE(),
};
#if NVENABLE_NAND_SUPPORT
const NvU32 g_TegraMux_Nand[] = 
{
    CONFIGEND(),    // config 1 - x16, standard pins
    BRANCH(2),      // Configure the standard x8 pins

    // Add the extra pins for x16.
    CONFIG(GMI_AD8,   NAND, NONE),    // NAND_D8
    CONFIG(GMI_AD9,   NAND, NONE),    // NAND_D9
    CONFIG(GMI_AD10,  NAND, NONE),    // NAND_D10
    CONFIG(GMI_AD11,  NAND, NONE),    // NAND_D11
    CONFIG(GMI_AD12,  NAND, NONE),    // NAND_D12
    CONFIG(GMI_AD13,  NAND, NONE),    // NAND_D13
    CONFIG(GMI_AD14,  NAND, NONE),    // NAND_D14
    CONFIG(GMI_AD15,  NAND, NONE),    // NAND_D15

    CONFIGEND(),    // config 2 - x8, standard pins
    CONFIG(GMI_AD0,   NAND, NONE),    // NAND_D0
    CONFIG(GMI_AD1,   NAND, NONE),    // NAND_D1
    CONFIG(GMI_AD2,   NAND, NONE),    // NAND_D2
    CONFIG(GMI_AD3,   NAND, NONE),    // NAND_D3
    CONFIG(GMI_AD4,   NAND, NONE),    // NAND_D4
    CONFIG(GMI_AD5,   NAND, NONE),    // NAND_D5
    CONFIG(GMI_AD6,   NAND, NONE),    // NAND_D6
    CONFIG(GMI_AD7,   NAND, NONE),    // NAND_D7
    CONFIG(GMI_WAIT,  NAND, NONE),    // NAND_BSY0
    CONFIG(GMI_ADV_N, NAND, NONE),    // NAND_ALE
    CONFIG(GMI_CLK,   NAND, NONE),    // NAND_CLE
    CONFIG(GMI_CS0_N, NAND, NONE),    // NAND_CE0
    CONFIG(GMI_WR_N,  NAND, NONE),    // NAND_WE
    CONFIG(GMI_OE_N,  NAND, NONE),    // NAND_RE
    CONFIG(GMI_DQS_P, NAND, NONE),    // NAND_DQS
    CONFIG(NAND_GMI_CLK_LB, NAND, NONE),    // NAND_GMI_CLK_LB
    CONFIGEND(),    // config 3 - alternate pins, not supported

    CONFIGEND(),
    SUBROUTINESDONE(),
    MODULEDONE(),
};
#endif
const NvU32 g_TegraMux_Snor[] =
{
    CONFIGEND(),    // Config 1, x16 Mux
    BRANCH(5), // Setup Common, AD[15:0]
    BRANCH(6), // Setup AD[27:16]
    
    CONFIGEND(),    // Config 2, x16 Non Mux
    BRANCH(5), // Setup Common, AD[15:0] (Data pins)
    BRANCH(7), // Setup A[27:0]
    
    CONFIGEND(), // Config 3 x32 Mux
    BRANCH(1), // Do x16 Mux Config.
    BRANCH(8), // Setup extra Data pins D[31:28]
    
    CONFIGEND(), // Config 4 x32 Non Mux
    BRANCH(3), // Do x32 Mux Config
    BRANCH(7), // Setup Address pins A[27:0]
    
    CONFIGEND(), // Config 5

#if 0 //FIXME
    CONFIG(GPIO_PJ0,     GMI, NORMAL), // NOR_CS0
    CONFIG(GPIO_PJ2,     GMI, NORMAL), // NOR_CS1
    CONFIG(GPIO_PK1,     GMI, NORMAL), // NOR_CLK
    CONFIG(GPIO_PK0,     GMI, NORMAL), // NOR_ADV_N
    CONFIG(GPIO_PI1,     GMI, NORMAL), // NOR_HIOR
    CONFIG(GPIO_PI0,     GMI, NORMAL), // NOR_HIOW

    CONFIG(GPIO_PI4,     GMI, NORMAL), // NOR_RST_N
    CONFIG(GPIO_PI7,     GMI, NORMAL), // NOR_WAIT
    CONFIG(SDMMC4_CLK,   GMI, NORMAL), // NOR_DPD
    CONFIG(GPIO_PC7,     GMI, NORMAL), // NOR_WP_N

    // SNOR AD[15:0] pins (Address/Data pins in Mux mode)
    CONFIG(GPIO_PG0,     GMI, NORMAL), // NOR_AD00
    CONFIG(GPIO_PG1,     GMI, NORMAL), // NOR_AD01
    CONFIG(GPIO_PG2,     GMI, NORMAL), // NOR_AD02
    CONFIG(GPIO_PG3,     GMI, NORMAL), // NOR_AD03
    CONFIG(GPIO_PG4,     GMI, NORMAL), // NOR_AD04
    CONFIG(GPIO_PG5,     GMI, NORMAL), // NOR_AD05
    CONFIG(GPIO_PG6,     GMI, NORMAL), // NOR_AD06
    CONFIG(GPIO_PG7,     GMI, NORMAL), // NOR_AD07
    CONFIG(GPIO_PH0,     GMI, NORMAL), // NOR_AD08
    CONFIG(GPIO_PH1,     GMI, NORMAL), // NOR_AD09
    CONFIG(GPIO_PH2,     GMI, NORMAL), // NOR_AD10
    CONFIG(GPIO_PH3,     GMI, NORMAL), // NOR_AD11
    CONFIG(GPIO_PH4,     GMI, NORMAL), // NOR_AD12
    CONFIG(GPIO_PH5,     GMI, NORMAL), // NOR_AD13
    CONFIG(GPIO_PH6,     GMI, NORMAL), // NOR_AD14
    CONFIG(GPIO_PH7,     GMI, NORMAL), // NOR_AD15
    
    CONFIGEND(), // Config 6 - AD[27:16]
    CONFIG(GPIO_PJ7,     GMI, NORMAL), // NOR_AD16
    CONFIG(GPIO_PB0,     GMI, NORMAL), // NOR_AD17
    CONFIG(GPIO_PB1,     GMI, NORMAL), // NOR_AD18
    CONFIG(GPIO_PK7,     GMI, NORMAL), // NOR_AD19
    CONFIG(GPIO_PK2,     GMI, NORMAL), // NOR_AD20
    CONFIG(GPIO_PK3,     GMI, NORMAL), // NOR_AD21
    CONFIG(GPIO_PK4,     GMI, NORMAL), // NOR_AD22
    CONFIG(GPIO_PI2,     GMI, NORMAL), // NOR_AD23
    CONFIG(GPIO_PI3,     GMI, NORMAL), // NOR_AD24
    CONFIG(GPIO_PI6,     GMI, NORMAL), // NOR_AD25
    CONFIG(SDMMC4_DAT6,  GMI, NORMAL), // NOR_AD26
    CONFIG(SDMMC4_DAT7,  GMI, NORMAL), // NOR_AD27
  
    CONFIGEND(), // CONFIG 7 - SNOR A[27:0] pins 
    CONFIG(UART2_RTS_N,  GMI, NORMAL), // NOR_A00
    CONFIG(UART2_CTS_N,  GMI, NORMAL), // NOR_A01
    CONFIG(UART3_TXD,    GMI, NORMAL), // NOR_A02
    CONFIG(UART3_RXD,    GMI, NORMAL), // NOR_A03
    CONFIG(UART3_RTS_N,  GMI, NORMAL), // NOR_A04
    CONFIG(UART3_CTS_N,  GMI, NORMAL), // NOR_A05
    CONFIG(GPIO_PU0,     GMI, NORMAL), // NOR_A06
    CONFIG(GPIO_PU1,     GMI, NORMAL), // NOR_A07
    CONFIG(GPIO_PU2,     GMI, NORMAL), // NOR_A08
    CONFIG(GPIO_PU3,     GMI, NORMAL), // NOR_A09
    CONFIG(GPIO_PU4,     GMI, NORMAL), // NOR_A10
    CONFIG(GPIO_PU5,     GMI, NORMAL), // NOR_A11
    CONFIG(GPIO_PU6,     GMI, NORMAL), // NOR_A12
    CONFIG(DAP4_FS,      GMI, NORMAL), // NOR_A13
    CONFIG(DAP4_DIN,     GMI, NORMAL), // NOR_A14
    CONFIG(DAP4_DOUT,    GMI, NORMAL), // NOR_A15
    CONFIG(DAP4_SCLK  ,  GMI, NORMAL), // NOR_A16
    CONFIG(DAP2_FS,      GMI, NORMAL), // NOR_A17
    CONFIG(DAP2_SCLK,    GMI, NORMAL), // NOR_A18
    CONFIG(DAP2_DIN,     GMI, NORMAL), // NOR_A19
    CONFIG(DAP2_DOUT,    GMI, NORMAL), // NOR_A20
    CONFIG(DVFS_PWM,     GMI, NORMAL), // NOR_A21
    CONFIG(GPIO_X1_AUD,  GMI, NORMAL), // NOR_A22
    CONFIG(DVFS_CLK,     GMI, NORMAL), // NOR_A23
    CONFIG(GPIO_X3_AUD,  GMI, NORMAL), // NOR_A24
    CONFIG(GPIO_X4_AUD,  GMI, NORMAL), // NOR_A25
    CONFIG(GPIO_X5_AUD,  GMI, NORMAL), // NOR_A26
    CONFIG(GPIO_X6_AUD,  GMI, NORMAL), // NOR_A27
    
    CONFIGEND(), // Config 8 - D[31:28]
    CONFIG(DAP1_FS,      GMI, NORMAL), // NOR_D28
    CONFIG(DAP1_DIN,     GMI, NORMAL), // NOR_D29
    CONFIG(DAP1_DOUT,    GMI, NORMAL), // NOR_D30
    CONFIG(DAP1_SCLK,    GMI, NORMAL), // NOR_D31
#endif //FIXME
    CONFIGEND(),
  
    SUBROUTINESDONE(),
    MODULEDONE(),
};
#if NVENABLE_XUSB_SUPPORT
const NvU32 g_TegraMux_Usb3[] =
{

    CONFIGEND(), // config 1, otg0
    // PUPD field in PINMUX_AUX_USB_VBUS_ENX_0 register to NONE
    CONFIG(USB_VBUS_EN0, USB, NONE), // vbus_0

    CONFIGEND(), // config 2, otg1
    // PUPD field in PINMUX_AUX_USB_VBUS_ENX_0 register to NONE
    CONFIG(USB_VBUS_EN1, USB, NONE), // vbus_1
    
    CONFIGEND(),
    SUBROUTINESDONE(),
    MODULEDONE(),

};
#endif

const NvU32* g_TegraMuxControllers[] =
{
    &g_TegraMux_Sdio4[0],  // SDMMC
    &g_TegraMux_Sflash[0], // SPI
#if NVENABLE_NAND_SUPPORT
    &g_TegraMux_Nand[0],   // NAND
#else
    NULL,
#endif
    NULL,
#if NVENABLE_XUSB_SUPPORT
    &g_TegraMux_Usb3[0],   // USB3
#else
    NULL,
#endif
    NULL, 		// Reserved
    NULL,		// Reserved
    NULL		// Reserved
};

