/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "arapbpm.h"
#include "arfuse.h"
#include "aruart.h"
#include "artimerus.h"
#include "nvboot_bct.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_version_rom.h"
#include "project.h"
#include "nvboot_fuse_int.h"
#include "nvboot_util_int.h"
#include "nvboot_reset_int.h"

/*
 * nvboot_uart.c: Implementation of the API for UART download
 */

extern NvBootInfoTable   BootInfoTable;
extern NvBootConfigTable BootConfigTable;

// This code is in a specific section, mapped into the nonsecure ROM
#pragma arm section rodata = "Uart", code = "Uart"

#define ASCII_HEX_DIGIT(x) (((x) < 10) ? ((x) + '0') : ((x) - 10 + 'A')) 

// These defines are AT LEAST one. For fast uart they wait longer, thats OK
// 11 bits per character (1 start + 2 stop + 8 bits) / 115200 = 96 us at 115200 baud
#define ONE_CHARACTER_WAIT_US 96
// one bit wait = 9 us at 115200 baud
#define ONE_BIT_WAIT_US 9

// Hardcode index into PeomptMsg where BootROM version will be updated.
#define MSG_VERSION_INDEX	0x0F
const static NvU8 PromptMsgConst[] = 
{
    '\n',
    '\r',
    'N',
    'V',
    ' ',
    'B',
    'o',
    'o',
    't',
    ' ',
    'T',
    '2',
    '1',
    '0',
    ' ',
    'W',
    'X',
    'Y',
    'Z',
    '.',
    'H',
    'I',
    'J',
    'K',
    '\n',
    '\r'
}; 

static const NvU8 FailMsg[]    = "Fail\n\r";
static const NvU8 SuccessMsg[] = "Boot\n\r"; 

// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
//Refer to ar/doc/t35/bootrom/uart_baud_clock_divisor_calc.xls for calculation details
const static NvU8 s_UartDivider115200[(int) NvBootClocksOscFreq_MaxVal] =
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

void NvBootUartFifoFlush(void);

#pragma Ospace
void
NvBootUartInit(NvBootClocksOscFreq OscFreq, NvBool IsNvProdMode)
{
    NvU32 RegData;
    NvU32 StrapBootFastUart, PllcStableTime = 0;
    NvBootClocksMiscParams *PllMiscTbl = NULL;

    // Initialization is responsible for correct pin mux configuration
    // It also needs to wait for the correct osc frequency to be known

    if(!IsNvProdMode) {
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
        if(!IsNvProdMode) {
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
        RegData |= (NvU32)(s_UartDivider115200[(NvU32) OscFreq]);
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
} // Init

#pragma Ospace
NvBootError
NvBootUartPollingWrite(const NvU8 *pSrc, NvU32 BytesRequested,
    NvU32 *BytesWritten)
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

#pragma Ospace
NvBootError
NvBootUartPollingRead(NvU8 *pDest, NvU32 BytesRequested, NvU32 *BytesRead)
{
    NvU8 Status;
    NvBootError Err;
    NvU32 BytesRemainToRead;


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

// need same trick as for boot mainline code, need to encapsulate in function
// as otherwise cannot use explicit register reference
// other approach using cast to pointer could be possible
//__asm void NvBootUartJump(NvU32 TargetAddress) {
//    ENDP
//    PRESERVE8
//    AREA Uart_asm, CODE, READONLY
//NvBootUartJump PROC
//    bx r0 
//    ENDP
//    END
//}

extern void NvBootUartJump(NvU32 TargetAddress);


// function that implements the download protocol
#pragma Ospace
void
NvBootUartDownload_internal (NvBootClocksOscFreq OscFreq)
{
    // this is called when we are ready to receive data, with nothing ready yet

    NvU32 nRead, nWritten;
    NvU8 *RxBuf;
    NvU32 length;
    NvU32 Checksum, RxChecksum;
    NvU32 i;
    NvBool Success;
    NvBootUart_Header *pHeader;
    NvBootECID Uid;

    NvU8 PromptMsg[sizeof(PromptMsgConst)];
    //Cannot use NvBootUtilMemcpy in unsecured code.
    for (i=0; i < sizeof(PromptMsgConst); i++)
	PromptMsg[i] = PromptMsgConst[i];
    //Sanity check on location to be updated. To reduce code size impact on
    //debug build we only check the '.'
    i = MSG_VERSION_INDEX;
    NV_ASSERT(PromptMsg[MSG_VERSION_INDEX + 4] == '.');
    //Load BootROM version into PromptMsg
    i = MSG_VERSION_INDEX;
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 28) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 24) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 20) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 16) & 0x0F);
    i++; // '.' stays
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >> 12) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >>  8) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >>  4) & 0x0F);
    PromptMsg[i++] = ASCII_HEX_DIGIT( (NVBOOT_BOOTROM_VERSION >>  0) & 0x0F);

    // Repeat until success
    while (1)
    {
        // Initialize 
        NvBootUartInit(OscFreq, NV_FALSE);
        Success = NV_TRUE;

        // send the prompt
        // Ignore error code here, rely on checksum to force retry later on
        (void) NvBootUartPollingWrite(PromptMsg, sizeof(PromptMsg),  &nWritten);
  
        // receive the header
        RxBuf = (NvU8 *) &BootConfigTable;
        pHeader = (NvBootUart_Header *) &BootConfigTable;
        // Ignore error code here
        (void) NvBootUartPollingRead(RxBuf, sizeof(NvBootUart_Header), &nRead);

        length = pHeader->MainLength;
        // ignore error code here, rely on checksum
        (void) NvBootUartPollingRead((RxBuf + sizeof(NvBootUart_Header)), length, &nRead);

        // check checksum
        // the checksum is a 1's complement of the sum of all but the last 4 bytes
        RxBuf = (NvU8 *) &BootConfigTable;
        Checksum = 0;
        for (i=0; i < length+sizeof(NvBootUart_Header) - sizeof(Checksum); i++)
        {
            Checksum += RxBuf[i];
        }
        Checksum = ~Checksum;
        RxChecksum = *((NvU32 *) & RxBuf[length + sizeof(NvBootUart_Header) -
                                  sizeof(Checksum)]);
        Success = Success && ((NvBool) (RxChecksum == Checksum));

        // Check UID if in FA mode
        if (NvBootFuseIsFailureAnalysisMode())
        {
            // Get UID and compare
            NvBootFuseGetUniqueId(&Uid);
            Success = Success && (NvBool) (pHeader->UniqueId0 
                                           == (NvU32)(Uid.ECID_0));
            Success = Success && (NvBool) (pHeader->UniqueId1 
                                           == (NvU32)(Uid.ECID_1));
            Success = Success && (NvBool) (pHeader->UniqueId2 
                                           == (NvU32)(Uid.ECID_2));
            Success = Success && (NvBool) (pHeader->UniqueId3 
                                           == (NvU32)(Uid.ECID_3));
        }

        if (Success)
        { 
            break;
        } else
        {
            (void) NvBootUartPollingWrite (FailMsg, sizeof(FailMsg)-1, &nWritten);
        }
    };
    // send success message, update bit and jump to target address
    (void) NvBootUartPollingWrite (SuccessMsg, sizeof(SuccessMsg)-1, &nWritten);
  
    BootInfoTable.BootRomVersion   = NVBOOT_BOOTROM_VERSION;
    BootInfoTable.DataVersion      = NVBOOT_BOOTDATA_VERSION;
    BootInfoTable.RcmVersion       = NVBOOT_RCM_VERSION;
    BootInfoTable.BootType         = NvBootType_Uart;
    BootInfoTable.PrimaryDevice    = NvBootDevType_Irom;
    BootInfoTable.SecondaryDevice  = NvBootDevType_Uart;
    BootInfoTable.OscFrequency     = OscFreq;
    BootInfoTable.DevInitialized   = NV_FALSE;
    BootInfoTable.SdramInitialized = NV_FALSE;
    BootInfoTable.BctValid         = NV_FALSE;
    BootInfoTable.SafeStartAddr    = (NvU32) &BootConfigTable;
 
    // need same trick as for boot mainline code, need to encapsulate in line assembler function
    NvBootUartJump((NvU32) &BootConfigTable);

}

#pragma Ospace
void
NvBootUartFifoFlush()
{
    NvU32 RegData;
    NvU32 i;

    // sequence indicates disabling and enabling HW flow control, we don't
    // use this in bootrom, but will ensure SW RTS is forced HI as specified.
    RegData = NV_READ8(NV_UARTA_MCR);
    RegData = NV_FLD_SET_DRF_DEF(UART, MCR, RTS, FORCE_RTS_HI, RegData);
    NV_WRITE08(NV_UARTA_MCR, RegData);

    // wait one character time
    NvBootUtilWaitUS(ONE_CHARACTER_WAIT_US);

    // Program TX_CLR and RX_CLR
    RegData = NV_READ8(NV_UARTA_FCR); 
    RegData = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_CLR, CLEAR, RegData);
    RegData = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_CLR, CLEAR, RegData);
    NV_WRITE08(NV_UARTA_FCR, RegData);

    // UART Bug 1442321 means we have to delay for at least 2 bits time here
    NvBootUtilWaitUS(2*ONE_BIT_WAIT_US);

    // Poll for TMTY to be 1, timeout in 1ms
    for (i=0; i<10; i++) {
        NvBootUtilWaitUS(100);
        RegData = NV_READ8(NV_UARTA_LSR);
        if (NV_DRF_VAL(UART, LSR, TMTY, RegData) == 1) {
            break;
        }
    }

    // Poll for RDR to be 0, timeout in 1ms
    for (i=0; i<10; i++) {
        NvBootUtilWaitUS(100);
        RegData = NV_READ8(NV_UARTA_LSR);
        if (NV_DRF_VAL(UART, LSR, RDR, RegData) == 0) {
            break;
        }
    }
}

// entry point that adjusts the stack then call the real stuff,
// stack is put at top of IRAM, rest of IRAM used for downloaded data
// again need an asm function to allow explicit register reference
#pragma arm
__asm void
NvBootUartAsm (NvBootClocksOscFreq OscFreq)
{

    ENDP
    PRESERVE8
    AREA Uart_asm, CODE, READONLY
NvBootUartDownload PROC
    import NvBootUartDownload_internal
    export NvBootUartDownload

    ldr sp, =NVBOOT_UART_IRAM_BLDR_STACK 
    bl NvBootUartDownload_internal
    // this is called when we are ready to receive data, with nothing ready yet
    ENDP
 
NvBootUartJump PROC
    export NvBootUartJump

    bx r0
    ENDP

    END
}
