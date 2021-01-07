/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#include "arfuse.h"
#include "nvrm_drf.h"
#include "nvboot_util_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_car_int.h"
#include "project.h"
#include "nvboot_config.h"
 
 
 /**
  *  These tables are here for reference to use with clocks table.
  *  Delete after porting to new table method.
  */
 // // UTMI PLL needs output = 960 Mhz with osc input
// // Bug 1398118
// // Note: CLKIN/M ratio should between 12 and 38.4 Mhz
// static const UtmiPllClockParams s_UtmiPllBaseInfo[NvBootClocksOscFreq_MaxVal] = 
// {
    // //DivN, DivM
    // {0x04A, 0x01}, // For NvBootClocksOscFreq_13, // Not P0.
    // {0x039,  0x1}, // For NvBootClocksOscFreq_16_8 // Not P0.
    // {    0,    0}, // dummy field
    // {    0,    0}, // dummy field
    // {0x032, 0x01}, // For NvBootClocksOscFreq_19_2 // Not P0.
    // {0x019, 0x01}, // For NvBootClocksOscFreq_38_4,
    // {    0,    0}, // dummy field
    // {    0,    0}, // dummy field
    // {0x050, 0x01}, // For NvBootClocksOscFreq_12
    // {0x028, 0x02}, // For NvBootClocksOscFreq_48, // Not P0.
    // {    0,    0}, // dummy field
    // {    0,    0}, // dummy field
    // {0x04A, 0x02}  // NvBootClocksOscFreq_26 // Not P0.
// };

// const NvU32 s_UsbHsicTrkDiv[NvBootClocksOscFreq_MaxVal] =
// {
    // // The frequency for the tracking circuit should be between 1 to 10 MHz. 
    // // osc_clk frequency is between 10 to 20 MHz, the clock divisor should be set to 0x2; 
    // // osc_clk frequency is between 20 to 30 MHz, the clock divisor should be set to 0x4; 
    // // osc_clk frequency is between 30 to 40 MHz, the clock divisor should be set to 0x6;
    // 2,  // For NvBootClocksOscFreq_13,
    // 2,  //  For NvBootClocksOscFreq_16_8
    // 0,  // dummy field
    // 0,  // dummy field
    // 2,  // For NvBootClocksOscFreq_19_2
    // 6,  //For NvBootClocksOscFreq_38_4,
    // 0,  // dummy field
    // 0,  // dummy field
    // 2,  // For NvBootClocksOscFreq_12
    // 6,  // For NvBootClocksOscFreq_48,
    // 0,  // dummy field
    // 0,  // dummy field
    // 4   // For NvBootClocksOscFreq_26
// };

// // dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
// ///////////////////////////////////////////////////////////////////////////////
// // PLL CONFIGURATION & PARAMETERS: refer to the arapb_misc_utmip.spec file.
// ///////////////////////////////////////////////////////////////////////////////
// // PLL CONFIGURATION & PARAMETERS for different clock generators:
// //-----------------------------------------------------------------------------
// // Reference frequency            13.0MHz       19.2MHz       12.0MHz      26.0MHz 
// // ----------------------------------------------------------------------------
// // PLLU_ENABLE_DLY_COUNT   02 (02h)       03 (03h)       02 (02h)     04 (04h)
// // PLLU_STABLE_COUNT          51 (33h)       75 (4Bh)       47 (2Fh)     102 (66h)
// // PLL_ACTIVE_DLY_COUNT     09 (09h)       12 (0C)        08 (08h)     17 (11h)
// // XTAL_FREQ_COUNT             118 (76h)     188 (BCh)     118 (76h)   254 (FEh)
// //-----------------------------------------------------------------------------
// // Reference frequency            16.8MHz        38.4MHz         48MHz 
// // ----------------------------------------------------------------------------
// // PLLU_ENABLE_DLY_COUNT   03 (03h)        05 (05h)         06 (06h)
// // PLLU_STABLE_COUNT          66 (42h)        150 (96h)       188 (BCh)
// // PLL_ACTIVE_DLY_COUNT     11 (0Bh)       24 (18h)         30 (1Eh)
// // XTAL_FREQ_COUNT             165 (A5h)      375 (177h)     469 (1D5h)
// ///////////////////////////////////////////////////////////////////////////////
// static const UsbPllDelayParams s_UsbPllDelayParams[NvBootClocksOscFreq_MaxVal] =
// {
    // //ENABLE_DLY,  STABLE_CNT,  ACTIVE_DLY,  XTAL_FREQ_CNT
    // {0x02,         0x33,        0x09,       0x7F}, // For NvBootClocksOscFreq_13,
    // {0x03,         0x42,        0x0B,       0xA5}, // For NvBootClocksOscFreq_16_8
    // {     0,              0,             0,             0}, // dummy field
    // {     0,              0,             0,             0}, // dummy field
    // {0x03,         0x4B,       0x0C,        0xBC}, // For NvBootClocksOscFreq_19_2
    // {0x05,         0x96,       0x18,      0x177},  //For NvBootClocksOscFreq_38_4
    // {     0,              0,             0,             0}, // dummy field
    // {     0,             0,              0,             0}, // dummy field
    // {0x02,         0x2F,       0x08,        0x76}, // For NvBootClocksOscFreq_12
    // {0x06,         0xBC,       0X1F,      0x1D5}, // For NvBootClocksOscFreq_48
    // {     0,              0,            0,              0}, // dummy field
    // {     0,              0,            0,              0}, // dummy field
    // {0x04,         0x66,        0x11,       0xFE}  // For NvBootClocksOscFreq_26
// };
 
 /***************************** Clock Tables ********************************************************/
/**
 *  Primary clock table for XUSB device.
 *  Used for Si.
 */
const ClockInst s_XUSBDeviceClkTable[] = 
{
     // Needed for HSIC_480 branch.
    Instc(Clk_Rmw_Def, PLLU_OUTA, PLLU_OUT1_RSTN, RESET_DISABLE),
    // Enable clocks for Device, SS and PadCtl
    Instc(Clk_W1b,  CLK_ENB_U_SET,  SET_CLK_ENB_XUSB_DEV),
    // Enable clocks for SS and PadCtl
    Instc(Clk_W1b,  CLK_ENB_W_SET,        SET_CLK_ENB_XUSB,
                                          SET_CLK_ENB_XUSB_SS),
    // Assert Reset to SS, PadCtl
    Instc(Clk_W1b,  RST_DEV_W_SET,    SET_XUSB_PADCTL_RST,
                                      SET_XUSB_SS_RST),
    // Assert Reset to Device, SS, Host (in case of Host boot), PadCtl
    Instc(Clk_W1b,  RST_DEV_U_SET,    SET_XUSB_DEV_RST),
    // Xusb Core Dev clock = 102 Mhz (Pllp_out0@408M/4)
    Instc(Clk_Src,  CLK_SOURCE_XUSB_CORE_DEV, XUSB_CORE_DEV_CLK_SRC, PLLP_OUT0,
                                              XUSB_CORE_DEV_CLK_DIVISOR, 6),
    // Xusb SS clock @120 Mhz (HSIC_480M/4)
    Instc(Clk_Src,  CLK_SOURCE_XUSB_SS, XUSB_SS_CLK_SRC, HSIC_480,
                                        XUSB_SS_CLK_DIVISOR, 6),
    // Xsub FS Clock @ FO_48M
    Instc(Clk_Rmw_Def,  CLK_SOURCE_XUSB_FS, XUSB_FS_CLK_SRC, FO_48M),
    // De-assert Reset to SS, PadCtl
    Instc(Clk_W1b,  RST_DEV_W_CLR,    CLR_XUSB_PADCTL_RST,
                                      CLR_XUSB_SS_RST),
    // De-assert Reset to Device, SS, Host (in case of Host boot), PadCtl
    Instc(Clk_W1b,  RST_DEV_U_CLR,    CLR_XUSB_DEV_RST,
                                      CLR_XUSB_HOST_RST),
    // Poll UTMI Pll lock
    Instc(Poll_Num, 100, UTMIPLL_HW_PWRDN_CFG0, UTMIPLL_LOCK, 1),
    // Enable tracking clock
    Instc(Clk_W1b,  CLK_ENB_Y_SET, SET_CLK_ENB_USB2_TRK),
    // Powerup and remove power down on Samplers
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_A_POWERDOWN, 0,
                                        UTMIP_FORCE_PD_SAMP_B_POWERDOWN, 0,
                                        UTMIP_FORCE_PD_SAMP_C_POWERDOWN, 0),
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_D_POWERDOWN, 0),
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_A_POWERUP, 1,
                                        UTMIP_FORCE_PD_SAMP_B_POWERUP, 1,
                                        UTMIP_FORCE_PD_SAMP_C_POWERUP, 1),
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_D_POWERUP, 1),
    {0} // Zero array Terminated.
};


/**
 *   Fpga clock table for XUSB device. The need to have separate table stems from:
 *  a. XUSB Host clock might be needed.
 *  b. SS and FS using PLLU branches results in getting FFs from XUSB config space and XUSB XHCI 
 *  register space.
 *  c. UTMIPLL won't lock. This might not be specific to UTMIPLL.
 */
 /*
const ClockInst s_XUSBDeviceFpgaClkTable[] = 
{
     // Needed for HSIC_480 branch.
    Instc(Clk_Rmw_Def, PLLU_OUTA, PLLU_OUT1_RSTN, RESET_DISABLE),
    // Enable clocks for Device, SS and PadCtl
    Instc(Clk_W1b,  CLK_ENB_U_SET,  SET_CLK_ENB_XUSB_DEV, 
                                    SET_CLK_ENB_XUSB_HOST),
    // Enable clocks for SS and PadCtl
    Instc(Clk_W1b,  CLK_ENB_W_SET,        SET_CLK_ENB_XUSB,
                                          SET_CLK_ENB_XUSB_SS),
    // Assert Reset to SS, PadCtl
    Instc(Clk_W1b,  RST_DEV_W_SET,    SET_XUSB_PADCTL_RST,
                                      SET_XUSB_SS_RST),
    // Assert Reset to Device, SS, Host (in case of Host boot), PadCtl
    Instc(Clk_W1b,  RST_DEV_U_SET,    SET_XUSB_DEV_RST,
                                      SET_XUSB_HOST_RST),
    // Xusb Core Dev clock = 102 Mhz (Pllp_out0@408M/4)
    Instc(Clk_Src,  CLK_SOURCE_XUSB_CORE_DEV, XUSB_CORE_DEV_CLK_SRC, PLLP_OUT0,
                                              XUSB_CORE_DEV_CLK_DIVISOR, 6),
    // Xusb SS clock @120 Mhz (HSIC_480M/4)
    Instc(Clk_Src,  CLK_SOURCE_XUSB_SS, XUSB_SS_CLK_SRC, HSIC_480,  // NOTE: Commenting this out
                                         XUSB_SS_CLK_DIVISOR, 6),    // on fpga because dividing fpga clk by 6 does n't make sense.
    // Xsub FS Clock @ FO_48M
    Instc(Clk_Rmw_Def,  CLK_SOURCE_XUSB_FS, XUSB_FS_CLK_SRC, FO_48M),
    // De-assert Reset to SS, PadCtl
    Instc(Clk_W1b,  RST_DEV_W_CLR,    CLR_XUSB_PADCTL_RST,
                                      CLR_XUSB_SS_RST),
    // De-assert Reset to Device, SS, Host (in case of Host boot), PadCtl
    Instc(Clk_W1b,  RST_DEV_U_CLR,    CLR_XUSB_DEV_RST,
                                      CLR_XUSB_HOST_RST),
    // Poll UTMI Pll lock
    TODO // Move to platform specific code.
    Instc(Poll_Num, 1000, UTMIPLL_HW_PWRDN_CFG0, UTMIPLL_LOCK, 1),
    // Enable tracking clock
    Instc(Clk_W1b,  CLK_ENB_Y_SET, SET_CLK_ENB_USB2_TRK),
    // Powerup and remove power down on Samplers
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_A_POWERDOWN, 0,
                                        UTMIP_FORCE_PD_SAMP_B_POWERDOWN, 0,
                                        UTMIP_FORCE_PD_SAMP_C_POWERDOWN, 0),
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_D_POWERDOWN, 0),
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_A_POWERUP, 1,
                                        UTMIP_FORCE_PD_SAMP_B_POWERUP, 1,
                                        UTMIP_FORCE_PD_SAMP_C_POWERUP, 1),
    Instc(Clk_Rmw_Num,  UTMIP_PLL_CFG2, UTMIP_FORCE_PD_SAMP_D_POWERUP, 1),
    {0} // Zero array Terminated.
};
*/

/**
 * The frequency for the tracking circuit should be between 1 to 10 MHz.
 * osc_clk frequency is between 10 to 20 MHz, the clock divisor should be set to 0x2; 
 * osc_clk frequency is between 20 to 30 MHz, the clock divisor should be set to 0x4; 
 * osc_clk frequency is between 30 to 40 MHz, the clock divisor should be set to 0x6;
 */
    // Osc 12
    // Only 38.4M needed.
/*
const ClockInst s_XUSBTrackingClockFrequency_12[] =  
{
    // ----------------------------------------  M     N    P  Misc1 Misc2
    Instc(Pll_Start, NvBootClocksPllId_PllU,    0x01, 0x28, 1, 0, 0),
    // ----------------------------------------  M     N    P  XtalCount, EnableDelay   StableCount, ActiveDelay
    Instc(Pll_Start, NvBootClocksPllId_UtmiPll, 0x01, 0x50, 0, Misc1_Utmi(0x76, 0x00), Misc2_Utmi(0x2f, 0x08)),
    Instc(Clk_Rmw_Num,  CLK_SOURCE_USB2_HSIC_TRK, USB2_HSIC_TRK_CLK_DIVISOR, 2),
};
*/
    // Osc 38_4
const ClockInst s_XUSBTrackingClockFrequency_38_4[] =
{
    // ----------------------------------------  M     N    P  Misc1 Misc2
    Instc(Pll_Start, NvBootClocksPllId_PllU,    0x02, 0x19, 1, 0, 0),
    // ----------------------------------------  M     N    P  XtalCount, EnableDelay   StableCount, ActiveDelay
    Instc(Pll_Start, NvBootClocksPllId_UtmiPll, 0x01, 0x19, 0, Misc1_Utmi(0x177, 0x00), Misc2_Utmi(0x96, 0x18)),
    Instc(Clk_Rmw_Num,  CLK_SOURCE_USB2_HSIC_TRK, USB2_HSIC_TRK_CLK_DIVISOR, 6),
    {0} // Zero array Terminated.
};

/**
 *  Collection of all tables required for xusb device.
 */
const ClockTable XusbClockTables[] = 
{
    // Oscillator freq dependent table, default to Osc 38_4
    s_XUSBTrackingClockFrequency_38_4,// DO NOT move this entry. Change OSC_TABLE macro defined in
                                       // nvboot_xusb_dev_hw.h if you do.
    // Common table
    s_XUSBDeviceClkTable,
    0 // Null pointer terminated. Note the difference from ClockInst termination with a zero array.
};

/*
ClockTable XusbFpgaClockTables[] = 
{
    // Oscillator freq dependent table, default to Osc 38_4
    s_XUSBTrackingClockFrequency_38_4, // DO NOT move this entry. Change OSC_TABLE macro defined in
                                       // nvboot_xusb_dev_hw.h if you do.
    // Common table
    s_XUSBDeviceFpgaClkTable,
    0 // Null pointer terminated. Note the difference from ClockInst termination with a zero array.
};
*/
/***************************** Clock Tables End ****************************************************/
