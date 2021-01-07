/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
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
#include "nvboot_hardware_access_int.h"
#include "nvboot_bct.h"
#include "nvboot_bit.h"
#include "nvboot_config.h"
#include "nvboot_clocks_int.h"
#include "nvboot_error.h"
#include "nvboot_uart_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_version_rom.h"
#include "nvboot_fuse_int.h"
#include "nvboot_util_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_strap_int.h"
#include "nvboot_bpmp_int.h"

/*
 * nvboot_uart.c: Implementation of the API for UART download
 */

extern NvBootInfoTable   BootInfoTable;
extern NvBootConfigTableBuffer BootConfigTable;

#define ASCII_HEX_DIGIT(x) (((x) < 10) ? ((x) + '0') : ((x) - 10 + 'A')) 

// Hardcode index into PeomptMsg where BootROM version will be updated.
#define MSG_VERSION_INDEX	15
// C std says implicit NUL at end of string is discarded if it makes string longer
// than struct, so this generates the same data as our older, more verbose layout
static const VT_NONSECURE uint8_t PromptMsgConst[26] = 
    "\n\rNV Boot T214 WXYZ.HIJK\n\r";

static const VT_NONSECURE uint8_t FailMsg[]    = "Fail\n\r";
static const VT_NONSECURE uint8_t SuccessMsg[] = "Boot\n\r"; 


void NvBootUartSetupStack(void);

extern void FT_NONSECURE NvBootUartJump(uintptr_t TargetAddress);


// function that implements the download protocol
void FT_NONSECURE NvBootUartDownload_internal (NvBootClocksOscFreq OscFreq)
{
    // this is called when we are ready to receive data, with nothing ready yet

    size_t nRead, nWritten;
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
        (void) NvBootUartPollingWrite(PromptMsg, sizeof(PromptMsg), &nWritten);

        // receive the header
        RxBuf = (NvU8 *) &BootConfigTable;
        pHeader = (NvBootUart_Header *) &BootConfigTable;
        // Ignore error code here
        (void) NvBootUartPollingRead(RxBuf, sizeof(NvBootUart_Header), &nRead);

        length = pHeader->MainLength;

        // limit our length to the size left in iram
        // stack is also at end of iram, so leave 0x2000 (8K) for stack
        // this is more than enough, and never a security issue since UART boot
        // is not availible outside NVIDIA.
        const NvU32 stackSpace = 0x2000;
        NvU32 maxLength = (NVBOOT_BL_IRAM_END - stackSpace) - (NvU32) RxBuf;
        if (length > maxLength) {
            length = maxLength;
        }

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
    // FI mitigation; recheck state of chip before jumping to UART payload.
    if ( NvBootFuseIsFailureAnalysisMode() || NvBootFuseIsPreproductionMode() )
    {

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
    BootInfoTable.SafeStartAddr    = (uintptr_t) &BootConfigTable;

    // need same trick as for boot mainline code, need to encapsulate in line assembler function
    NvBootUartJump((uintptr_t) &BootConfigTable);
        // should never get here.
        do_exception();
    }
    else
    {
        do_exception();
    }
}

void FT_NONSECURE NvBootUartDownload(NvBootClocksOscFreq OscFreq)
{
    // Adjust top of stack to end of IRAM
    // This is again required for BootROM because
    // the memory layout falls back to being similar to
    // prior chips and therefore, uart download will corrupt
    // BootROM stack. Placing BootROM stack towards end of memory
    // may leave memory allocation holes and unclaimed space.
    NvBootUartSetupStack();
    // NvBootUartDownload_internal never returns.
    NvBootUartDownload_internal(OscFreq);
}
