/*
 * Copyright (c) 2007 - 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "project.h"
#include "arapb_misc.h"
#include "aruart.h"

#include "nvrm_drf.h"
#include "nvboot_bct.h"
#include "nvboot_bit.h"
#include "nvboot_config.h"
#include "nvboot_clocks_int.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_version_rom.h"
#include "nvboot_util_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_strap_int.h"

/*
 * nvboot_uart.c: Implementation of the API for UART download
 */


// These defines are AT LEAST one. For fast uart they wait longer, thats OK
// 11 bits per character (1 start + 2 stop + 8 bits) / 115200 = 96 us at 115200 baud
#define ONE_CHARACTER_WAIT_US 96
// one bit wait = 9 us at 115200 baud
#define ONE_BIT_WAIT_US 9

// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
//Refer to ar/doc/t35/bootrom/uart_baud_clock_divisor_calc.xls for calculation details
static const VT_NONSECURE uint8_t s_UartDivider115200[(int) NvBootClocksOscFreq_MaxVal] =
{ 
	12, // 13 MHz
	16, // 16.8 MHz
	0,
	0,
	18, // 19.2 MHz
	39, // 38.4 MHz
	0,
	0,	
	11,	// 12 MHz
	50, // 48 MHz
	0,
	0,	
	26  // 26 MHz
};

// Macro that simplifies pin settings.
#define SET_PIN(pin, f, v)                                                   \
do {                                                                     \
       NvU32 RegData;                                                        \
       RegData = NV_READ32(NV_ADDRESS_MAP_MISC_BASE + PINMUX_AUX_##pin##_0); \
       RegData = NV_FLD_SET_DRF_DEF(PINMUX_AUX, pin, f, v, RegData);         \
       NV_WRITE32(NV_ADDRESS_MAP_MISC_BASE + PINMUX_AUX_##pin##_0, RegData); \
} while (0);


void FT_NONSECURE NvBootUartFifoFlush(void);


void FT_NONSECURE
NvBootUartInit(NvBootClocksOscFreq oscFreq, bool isNvProdMode)
{
    NvU32 RegData;
    NvU32 StrapBootFastUart, PllcStableTime = 0;
    NvBootClocksMiscParams *PllMiscTbl = NULL;

    // Initialization is responsible for correct pin mux configuration
    // It also needs to wait for the correct osc frequency to be known

    if(!isNvProdMode) {
        // IMPORTANT, there is some unspecified logic in the UART that always 
        // operate off PLLP_out3, mail from Robert Quan says that correct operation
        // of UART without starting PLLP requires
        // - to put PLLP in bypass
        // - to override the PLLP_ou3 divider to be 1 (or put in bypass)
        // This is done like that here to avoid any dependence on PLLP analog circuitry 
        // operating correctly

        RegData  = NV_READ32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE +
                             CLK_RST_CONTROLLER_PLLP_BASE_0);
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, PLLP_BASE, PLLP_BYPASS, ENABLE);
        NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE +
                   CLK_RST_CONTROLLER_PLLP_BASE_0, RegData);

        RegData  = NV_READ32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE +
                             CLK_RST_CONTROLLER_PLLP_OUTB_0);
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, PLLP_OUTB,
                              PLLP_OUT3_OVRRIDE, ENABLE);
        RegData |= NV_DRF_NUM(CLK_RST_CONTROLLER,
                              PLLP_OUTB, PLLP_OUT3_RATIO, 0);
        NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE +
                   CLK_RST_CONTROLLER_PLLP_OUTB_0, RegData);
    }

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                        APB_MISC_PP_STRAPPING_OPT_A_0);
    StrapBootFastUart = NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A,
                                   BOOT_FAST_UART, RegData);
    if (StrapBootFastUart == APB_MISC_PP_STRAPPING_OPT_A_0_BOOT_FAST_UART_FAST)
    {
        //Bug 1172661 : Support fast uart config in bootrom code for ATE test time savings
        //Configure PLLC for osc frequency 38.4MHz. This feature is for ATE only
        //so its ok to assume that OscFreq is running at 38.4Hz
        //Input frequency REF = 38.4MHz, Desired baud rate = 12Mbps
        //Assumption is that baud rate divisor would be set to divide by 1
        //So frequency for baud rate generator's input clock = 64 * Input frequency / 4 = 192 MHz 
        //Enable PLL C. Refer to arclk_rst.spec
        //Comparison frequency range CF = REF/DIVM. DIVM is input divider control 
        //DIVM = 1 (is selected such that CF is in valid range)
        //CF = 38.4/1 = 38.4MHz
        //VCO frequency range VCO = CF *DIVN where DIVN is feedback divider control 
        //DIVN = 15
        //VCO = 38.4 * 15 = 576 MHz
        //Final Pll output frequency FO = VCO / DIVP+1 where DIVP is post divide control
        //DIVP = 2
        // M, N,  P,
        // 1, 15, 3,
        // If required, change the values in s_PllMiscTbl
        PllMiscTbl = NvBootClocksGetPllMiscParams(NvBootClocksPllId_PllC4);

        NvBootClocksStartPllC4(PllMiscTbl->divm,
                               PllMiscTbl->divn,
                               PllMiscTbl->divp,
                               NV_DRF_DEF(CLK_RST_CONTROLLER, PLLC4_MISC, PLLC4_EN_LCKDET, ENABLE),
                               &PllcStableTime);

        while (!NvBootClocksIsPllC4Stable(PllcStableTime));
    }

    // Pinmux settings for uart
    // refer to include_chip/t210/arpinmux_aux.spec and ar/doc/t210/pinmux/t210_mpio_pinmux_gfd.docx
    // Configuration is not set to UARTA before programming the pin configuration as suggested by hw team.
    // Set pinmux to reset values, if for some reason register value is not reset, UART boot will still work
    SET_PIN(UART1_TX,       PM, DEFAULT) ; //Tx
    SET_PIN(UART1_RX,       PM, DEFAULT) ; //Rx

    SET_PIN(UART1_TX, TRISTATE, PASSTHROUGH) ;
    SET_PIN(UART1_RX, TRISTATE, PASSTHROUGH) ;

    SET_PIN(UART1_RX,     PUPD, PULL_UP);
    SET_PIN(UART1_TX,     PUPD, PULL_UP);
    
    SET_PIN(UART1_RX,  E_INPUT, ENABLE) ;
    SET_PIN(UART1_TX,  E_INPUT, ENABLE) ;

    // Then there is the specific set up for UART itself, including the clock configuration.
    // configure at top for source & configure the divider, internal to uart for obscure reasons
    // when UARTA_DIV_ENB is enable DLL/DLLM programming is not required
    // Refer to the CLK_SOURCE_UARTA reg description in arclk_rst
    
    //1)Assert device reset
    NvBootResetSetEnable(NvBootResetDeviceId_UartaId, NV_TRUE);
      
    if (StrapBootFastUart == APB_MISC_PP_STRAPPING_OPT_A_0_BOOT_FAST_UART_FAST )
    {
        //Only Switch if this is Preproduction UART boot or FA
        if(!isNvProdMode) {
            // Switch AVP/system clock to PLLC4_OUT1 = 320MHz
            RegData = NV_DRF_DEF(CLK_RST_CONTROLLER, SCLK_BURST_POLICY, SYS_STATE, RUN) |
                      NV_DRF_DEF(CLK_RST_CONTROLLER, SCLK_BURST_POLICY, SWAKEUP_RUN_SOURCE, PLLC4_OUT1);
            NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0,
                       RegData);
        }

        //To change device's clock source and divider after boot up
        //a)assert device reset              b)configure clock source,clock divider and wait 2 us
        //c)enable clock and wait 2 us       d)deassert reset (make sure there is 2us of wait between assert and deassert)
        
        //2)Configure clock source and divider

        // Set baud rate divisor to be 1 so that baud rate =  PLLC input clock /16 i.e. 16 * 12 /16 MHz
        // UARTA_CLK_DIVISOR is in 15.1 format
        // 15.1 format
        // [15 :1] = n
        // [0] = m
        // Divisor = (n+1) + m *.5 
        // For divide by 1 n = 0, m = 0
        RegData = 0;
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_CLK_SRC, PLLC4_OUT2);
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, ENABLE);
	NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE+CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0, RegData);
    }
    else
    {
#if !NVBOOT_TARGET_QT
        //Configure clock source and clock divider using method 2 (refer to arclk_rst.spec)
        //Since default clock source is osc, looks like method 2 is working fine. In general
        //bootrom follows method 1 to change a device's clock source/divider
        RegData = 0;
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_CLK_SRC, CLK_M);
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, ENABLE);
        RegData |= (NvU32)(s_UartDivider115200[(NvU32) oscFreq]);
        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE+CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0, RegData); 
        //wait 2 us after changing the divider. Refer to arclk_rst.spec
        //This method corresponds to method 2. Since default clk source to uart is osc,
        //it is ok not to use reset method i.e. Method 1 
#else
        // On QT, OSC's frequency is 115200 Hz
        // Baud rate should be 2400 bps
        // 115200/(16*2400) = 3
        // 15.1 format gives divisor (3-1)<<1 = 4
        RegData = 0;
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_CLK_SRC, CLK_M);
        RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, ENABLE);
        RegData |= (NvU32)(4);

        NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE+CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0, RegData); 
#endif
    }

    //wait 2 us after changing the divider. Refer to arclk_rst.spec
    //This method corresponds to method 2. Since default clk source to uart is osc,
    //it is ok not to use reset method i.e. Method 1 
    //Wait 2us to make sure clock source/device logic is stabilized    
    NvBootUtilWaitUS(NVBOOT_CLOCKS_CLOCK_STABILIZATION_TIME);
    
    //3)Enable UART A clock  
    NvBootClocksSetEnable(NvBootClocksClockId_UartaId, NV_TRUE);

    //4) Deassert device reset
    NvBootResetSetEnable(NvBootResetDeviceId_UartaId, NV_FALSE);

    // Configure for 8 bit, no parity, 2 stop bits, this also remove the DLAB enable bit
    NV_WRITE08(NV_UARTA_LCR, LCR_8BIT_NOPAR_2STOP);

    // Clear all Rx and trasnsmit
    NV_WRITE08(NV_UARTA_FCR, 0x00);
    // clear all interrupt
    NV_WRITE08(NV_UARTA_IER, 0x00);
    // Setup UART FIFO - Enable FIFO Mode
    NV_WRITE08(NV_UARTA_FCR, NV_DRF_DEF(UART, IIR_FCR, FCR_EN_FIFO, ENABLE));
    // Delay after writing FIFO is suggested - Bug 1432784
    NvBootUtilWaitUS(20);
    // initialize Fifo Control Register.
    NvBootUartFifoFlush();
}

NvBootError FT_NONSECURE
NvBootUartPollingWrite(const uint8_t *pSrc, size_t BytesRequested, size_t *BytesWritten)
{
    NvU32 BytesLeft = BytesRequested;
    NvU32 FifoSpace;
    NvU8  Status;

    *BytesWritten = 0;
    if (BytesRequested == 0)
    {
        return NvBootError_Success;
    }

    while (BytesLeft)
    {
        // Wait till transmitter empty.
        while (1)
        {
            Status = NV_READ8(NV_UARTA_LSR);
            if (NV_DRF_VAL(UART,LSR,FIFOE,Status) == UART_LSR_0_FIFOE_ERR )
            {
                NvBootUartFifoFlush();
                return NvBootError_ValidationFailure;  // error code not really important, placeholder
            }
            if ((NV_DRF_VAL(UART,LSR,TMTY,Status) == UART_LSR_0_TMTY_EMPTY) &&
                (NV_DRF_VAL(UART,LSR,THRE,Status) == UART_LSR_0_THRE_EMPTY) )
            {
                break;
            }
        }

        // Transmit up to 16 bytes
        FifoSpace = 16;
        while (BytesLeft && FifoSpace)
        {
            NV_WRITE08(NV_UARTA_THR, *pSrc);
            pSrc++;
            BytesLeft--;
            FifoSpace--;
        } // while
    } // while

    while (1)
    {
        Status = NV_READ8(NV_UARTA_LSR);
        if (NV_DRF_VAL(UART,LSR,FIFOE,Status) == UART_LSR_0_FIFOE_ERR )
        {
            NvBootUartFifoFlush();
            return NvBootError_ValidationFailure;  // error code not really important, placeholder 
        }
        if ((NV_DRF_VAL(UART,LSR,TMTY,Status) == UART_LSR_0_TMTY_EMPTY) &&
            (NV_DRF_VAL(UART,LSR,THRE,Status) == UART_LSR_0_THRE_EMPTY) )
        {
            break;
        }
    }

    *BytesWritten  = BytesRequested - BytesLeft;
    return NvBootError_Success;
}

NvBootError FT_NONSECURE
NvBootUartPollingRead(uint8_t *pDest, size_t BytesRequested, size_t *BytesRead)
{
    uint8_t Status;
    NvBootError Err;
    uint32_t BytesRemainToRead;


    *BytesRead = 0;
    if (BytesRequested == 0)
    {
        return NvBootError_Success;
    }

    BytesRemainToRead = BytesRequested;
    while(BytesRemainToRead)
    {
        // Wait for some time to get the data
        while (1)
        {
            Err = NvBootError_Success;
            Status = NV_READ8(NV_UARTA_LSR);
            if (Status & (UART_LSR_0_BRK_FIELD  | UART_LSR_0_FERR_FIELD |
                          UART_LSR_0_PERR_FIELD | UART_LSR_0_OVRF_FIELD))
            {
                Err = NvBootError_ValidationFailure;  // error code not really important, placeholder
                break;
            } else if (NV_DRF_VAL(UART,LSR,RDR,Status) == UART_LSR_0_RDR_DATA_IN_FIFO)
            {
                break;
            }
        }
        while (Status = NV_READ8(NV_UARTA_LSR),
               NV_DRF_VAL(UART,LSR,RDR,Status) == UART_LSR_0_RDR_DATA_IN_FIFO)
        {
            if (BytesRemainToRead)
            {
                *pDest++ = NV_READ8(NV_UARTA_RBR);
                BytesRemainToRead--;
            } else
            {
                // Exit the loop if data is still in the FIFO and no more
                // bytes remain to be read.
                break;
            }
            if ((BytesRemainToRead & 0xFF) == 0xFF)
            {
                //  RTRegWr32(NV_UARTA_THR, '#');
            }
        }
        if (Err == NvBootError_ValidationFailure)
        {
            // careful if modifying the error code
            // We used to only flush RX here, but the new flush function
            // will do both. That is OK.
            NvBootUartFifoFlush();
            *BytesRead = BytesRequested -BytesRemainToRead;
            return Err;
        }
    }                  
    *BytesRead = BytesRequested - BytesRemainToRead;
    return NvBootError_Success;
}

/**
 *  \brief Uart polling read that reads bytes till timeout
 *  \param pDest Buffer
 *  \param BytesRead Bytes received
 *  \param timeout between consecutive byte reads
 */
NvBootError FT_NONSECURE
NvBootUartPollingRead2(uint8_t *pDest, size_t *BytesRead, uint32_t timeout)
{
    uint8_t Status;
    NvBootError Err;
    size_t BytesReadTemp=0;
    uint32_t t0, t1;
    
    t0 = t1 = NvBootUtilGetTimeUS();

    while(!BytesReadTemp || (t1 - t0) <= timeout)
    {
        Err = NvBootError_Success;
        Status = NV_READ8(NV_UARTA_LSR);
        if (Status & (UART_LSR_0_BRK_FIELD  | UART_LSR_0_FERR_FIELD |
                      UART_LSR_0_PERR_FIELD | UART_LSR_0_OVRF_FIELD))
        {
            Err = NvBootError_ValidationFailure;  // error code not really important, placeholder
            break;
        }
        if (NV_DRF_VAL(UART,LSR,RDR,Status) == UART_LSR_0_RDR_DATA_IN_FIFO)
        {
                
                *pDest++ = NV_READ8(NV_UARTA_RBR);
                BytesReadTemp++;
                t0 = NvBootUtilGetTimeUS();
        }
        t1 = NvBootUtilGetTimeUS();
    }
    
    if (Err == NvBootError_ValidationFailure)
    {
        // careful if modifying the error code
        // We used to only flush RX here, but the new flush function
        // will do both. That is OK.
        NvBootUartFifoFlush();
    }
    *BytesRead = BytesReadTemp;
    return Err;    
}

void FT_NONSECURE
NvBootUartFifoFlush()
{
    // sequence indicates disabling and enabling HW flow control, we don't
    // use this in bootrom, but will ensure SW RTS is forced HI as specified.
    uint32_t mcr = NV_READ8(NV_UARTA_MCR);
    mcr = NV_FLD_SET_DRF_DEF(UART, MCR, RTS, FORCE_RTS_HI, mcr);
    NV_WRITE08(NV_UARTA_MCR, mcr);

    // wait one character time
    NvBootUtilWaitUS(ONE_CHARACTER_WAIT_US);

    // Program TX_CLR and RX_CLR
    // Doing a RMW with FCR would be meaningless, it has write only registers
    // For chips that change other FCR parameters we need to revisit this, it does
    // overwrite some stuff if you change it away from default. Maybe a shadow register?
    // But this way is fine for T214, and matches previous chips
    uint32_t fcr = 0x00; //NV_READ8(NV_UARTA_FCR); 
    fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, FCR_EN_FIFO, ENABLE, fcr);
    fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_CLR, CLEAR, fcr);
    fcr = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_CLR, CLEAR, fcr);
    NV_WRITE08(NV_UARTA_FCR, fcr);

    // UART Bug 1442321 means we have to delay for at least 2 bits time here
    NvBootUtilWaitUS(2*ONE_BIT_WAIT_US);

    // Poll for TMTY to be 1, timeout in 1ms
    for (uint_fast8_t i = 0; i < 10; i++) {
        NvBootUtilWaitUS(100);
        uint32_t lsr = NV_READ8(NV_UARTA_LSR);
        if (NV_DRF_VAL(UART, LSR, TMTY, lsr) == 1) {
            break;
        }
    }

    // Poll for RDR to be 0, timeout in 1ms
    for (uint_fast8_t i = 0; i < 10; i++) {
        NvBootUtilWaitUS(100);
        uint32_t lsr = NV_READ8(NV_UARTA_LSR);
        if (NV_DRF_VAL(UART, LSR, RDR, lsr) == 0) {
            break;
        }
    }
}
