/*
 * Copyright (c) 2006 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file nvboot_uart.h
 *
 * Simple UART downloader for NvBoot
 *
 * NvBootUart is NVIDIA's interface for simple UART based downloading.
 * Used for both preproduction and failure analysis modes
 *
 *
 */

#ifndef INCLUDED_NVBOOT_UART_INT_H
#define INCLUDED_NVBOOT_UART_INT_H

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "nvtypes.h"
#include "nvrm_drf.h"

#include "nvboot_clocks_int.h"
#include "nvboot_error.h"

// shorter names
#define NV_UARTA_BASE   (NV_ADDRESS_MAP_APB_UARTA_BASE)
#define NV_UARTA_RBR    (NV_UARTA_BASE + UART_THR_DLAB_0_0) 
#define NV_UARTA_THR    (NV_UARTA_BASE + UART_THR_DLAB_0_0) 
#define NV_UARTA_DLL    (NV_UARTA_BASE + UART_THR_DLAB_0_0) 
#define NV_UARTA_IER    (NV_UARTA_BASE + UART_IER_DLAB_0_0) 
#define NV_UARTA_DLM    (NV_UARTA_BASE + UART_IER_DLAB_0_0)
#define NV_UARTA_IIR    (NV_UARTA_BASE + UART_IIR_FCR_0)
#define NV_UARTA_FCR    (NV_UARTA_BASE + UART_IIR_FCR_0) 
#define NV_UARTA_LCR    (NV_UARTA_BASE + UART_LCR_0) 
#define NV_UARTA_MCR    (NV_UARTA_BASE + UART_MCR_0)
#define NV_UARTA_LSR    (NV_UARTA_BASE + UART_LSR_0) 
#define NV_UARTA_MSR    (NV_UARTA_BASE + UART_MSR_0)
#define NV_UARTA_SPR    (NV_UARTA_BASE + UART_SPR_0)
#define NV_UARTA_CSR    (NV_UARTA_BASE + UART_IRDA_CSR_0) 

// configure uart for 8 bit characters, no parity, 1 extra stop bit
#define LCR_8BIT_NOPAR_2STOP \
  ( NV_DRF_DEF(UART,LCR,WD_SIZE,WORD_LENGTH_8) | \
    NV_DRF_DEF(UART,LCR,PAR,NO_PARITY) | \
    NV_DRF_DEF(UART,LCR,STOP,ENABLE) )

// Moved to nvboot_config.h; uart.S cannot include nvboot_uart_int.h.
// #define NVBOOT_UART_IRAM_BLDR_STACK (NV_ADDRESS_MAP_DATAMEM_IRAM_D_LIMIT + 1)

// This structure defines the header fields
// The other side must format for correct endianness on AP15 side
typedef struct {
    NvU32 EntryInstruction ;  // Entry point for the downloaded code
    NvU32 MainLength ;        // Length of the data following the header, including checksum
    NvU32 UniqueId0 ;
    NvU32 UniqueId1 ;
    NvU32 UniqueId2 ;
    NvU32 UniqueId3 ;
} NvBootUart_Header ;

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * nvboot_uart.h: Definition of the API for UART download
 */

/*
 * NvBootUartDownload(): implements the donwload protocol
 * 
 * @params OscFreq indicates the current operating frequency
 *
 * @return no return
 *
 */
void 
NvBootUartDownload (NvBootClocksOscFreq OscFreq);

/* NvBootUartInit(NvBootClocksOscFreq OscFreq, NvBool IsPllpRunning)
 *
 * @params OscFreq indicates the current operating frequency
 * @params IsNvProdMode Sets if AVP clock switches for ATE FAST UART, and if PllP
 *         is bypassed. True for Production Mode Uart, False for PreProd or FA
 *
 * @return no return
 */
void
NvBootUartInit(NvBootClocksOscFreq oscFreq, bool isNvProdMode);

NvBootError
NvBootUartPollingRead(uint8_t *pDest, size_t BytesRequested, size_t *BytesRead);

NvBootError NvBootUartReadOneByte(uint8_t *dest);

NvBootError
NvBootUartPollingWrite(const uint8_t *pSrc, size_t BytesRequested,
    size_t *BytesWritten);

/**
 *  \brief Uart polling read that reads bytes till timeout
 *  \param pDest Buffer
 *  \param BytesRead Bytes received
 *  \param timeout between consecutive byte reads
 */
NvBootError FT_NONSECURE
NvBootUartPollingRead2(uint8_t *pDest, size_t *BytesRead, uint32_t timeout);

/**
 * NvBootIsFAPreProductionUart : implements the decision process to 
 * to go to preproduction uart and handling failure during download over uart.
 * This function is implemented in main flow path and its header is made
 * made available through this header file as there's no dedicated header file to 
 * to expose functions in main flow path.
 */
void FT_NONSECURE NvBootIsFAPreProductionModeUart(void);
  
#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_UART_INT_H
