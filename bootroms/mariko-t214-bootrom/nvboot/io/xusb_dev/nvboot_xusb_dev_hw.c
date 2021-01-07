/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#include "nvboot_xusb_dev.h"
#include "nvboot_xusb_dev_int.h"
#include "ardev_t_xusb_dev_xhci.h"
#include "ardev_t_fpci_xusb_dev.h"
#include "arxusb_padctl.h"
#include "arxusb_dev.h"
#include "arfuse.h"
#include "nvrm_drf.h"
#include "nvboot_util_int.h"
#include "nvboot_arc_int.h"
#include "nvboot_dispatcher_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_wdt_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_car_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_platform_int.h"
#include "arapb_misc.h"
#include "armc.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_config.h"

extern XUSBDeviceContext_T XUSBDeviceContext;
extern EpContext_T *EpContext;

/**
 *  Setup Event ring.
 *  Note: Event ring region should be capable of holding NumTRB
 */
NvBootError NvBootXusbDeviceInitializeEventRing(NvU32 EventRing, NvU32 NumTRB)
{
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    NvU32 RegData;
    /* zero out event ring */
    NvBootUtilMemset((void*)EventRing, 0, NumTRB*sizeof(EventTRB_T));
    //NvBootUtilMemset((void*)0x80000000, 0, NumTRB*sizeof(EventTRB_T));
    /* Set event ring segment 0 and segment 1 */
    /* Segment 0 */
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_ERST0BALO_0);
    // SysRAM address decode caveat.
    RegData |= (NvU32)EventRing;
    
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST0BALO_0, RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST0BAHI_0, 0);

    /* Segment 1 */
    
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST1BALO_0, (EventRing+(NumTRB/2)*sizeof(EventTRB_T)));
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST1BAHI_0, 0);

    /* Write segment sizes */
    RegData = NV_DRF_NUM(XUSB_DEV_XHCI, ERSTSZ, ERST0SZ, NumTRB/2) |
              NV_DRF_NUM(XUSB_DEV_XHCI, ERSTSZ, ERST1SZ, NumTRB/2);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERSTSZ_0, RegData);
    
    /* Initialize Enqueue and Dequeue pointer of consumer context*/
    pXUSBDeviceContext->EventDequeuePtr =
    pXUSBDeviceContext->EventEnqueuePtr = 
    EventRing;
    pXUSBDeviceContext->EventCCS = 1;
    
    /* Set Enqueue/Producer Cycle State for controller */
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EREPLO_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 EREPLO,
                                 ECS,
                                 pXUSBDeviceContext->EventCCS,
                                 RegData);
    // Bits 3:0 are not used to indicate 16 byte aligned.
    // Shift the Enqueue Pointer before using DRF macro. 
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 EREPLO,
                                 ADDRLO,
                                 (pXUSBDeviceContext->EventEnqueuePtr)>> 4,
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EREPLO_0, RegData);

    // Clear bits 63:32 of enqueue pointer.
    RegData = 0;
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EREPHI_0, RegData);

    // Set the Dequeue Pointer
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_ERDPLO_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 ERDPLO,
                                 ADDRLO,
                                 (pXUSBDeviceContext->EventDequeuePtr) >> 4,
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERDPLO_0, RegData);
    
    // Clear bits 63:32 of Dequeue pointer.
    RegData = 0;
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERDPHI_0, RegData);

    return NvBootError_Success;
}

/**
 *  Poll for interrupt status and give Event enqueue pointer.
 */
NvBootError NvBootXusbDevicePollforInterrupt(NvU32 Timeout, NvU32 *EventEnqueuePtr)
{
    NvU32 RegData, ExpectedValue, IntPendingMask;
    NvBootError e;
    
    ExpectedValue = 1 << SHIFT(XUSB_DEV_XHCI, ST, IP);
    IntPendingMask = SHIFTMASK(XUSB_DEV_XHCI, ST, IP);

    // Poll for interrupt pending bit.
    e = NvBootPollField((XUSB_BASE+XUSB_DEV_XHCI_ST_0),
                        IntPendingMask,
                        ExpectedValue,
                        Timeout);
    if(e != NvBootError_Success) 
        return e;
    RegData = NV_READ32(XUSB_BASE+XUSB_DEV_XHCI_ST_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 ST,
                                 IP,
                                 1,
                                 RegData);
    NV_WRITE32(XUSB_BASE+XUSB_DEV_XHCI_ST_0, RegData);
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EREPLO_0);
    
    *EventEnqueuePtr = RegData & SHIFTMASK(XUSB_DEV_XHCI, EREPLO, ADDRLO);

    return e;

}

/**
 *  Clear Event handler busy and update dequeue pointer.
 */
void NvBootXusbDeviceUpdateEventDequeuePtr(NvU32 EventDequeuePtr)
{
    NvU32 RegData;
    
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_ERDPLO_0);
    // Clear Event Handler Busy bit
    if(NV_DRF_VAL(XUSB_DEV_XHCI, ERDPLO, EHB, RegData))
    {
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 ERDPLO,
                                 EHB,
                                 1,
                                 RegData);
    }

    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 ERDPLO,
                                 ADDRLO,
                                 EventDequeuePtr >> 4,
                                 RegData);

    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERDPLO_0, RegData);
}

/**
 *  Set run status
 */
void NvBootXusbDeviceSetRunStatus(NvU32 Run)
{
    NvU32 RegData;
    
    if(Run)
    {
        // Now set run
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0);
        RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                     CTRL,
                                     RUN,
                                     RUN,   
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);
    }
    else
    {
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0);
        RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                     CTRL,
                                     RUN,
                                     STOP,   
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);
    }
    // Also clear Run Change bit just in case to enable Doorbell register.
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_ST_0);
    RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                 ST,
                                 RC,
                                 CLEAR,
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ST_0, RegData);
}
 
 /**
  *  USB descriptor is updated to reflect chip specific data
  *  CHIP
  *  FAM (Handheld) | SKU (0-15)
  *  MAJORREV 
  *  MINORREV
  */
void NvBootXusbDeviceSetPidRev(uint8_t *pDevDescriptor)
{
    NvU32 RegData = 0;
    NvU32 Sku;

    // Read the Chip ID revision register
    // Bug 1679199
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_GP_HIDREV_0);

    // Product ID definition (2 bytes as per USB 2.0 Spec)
    // idProduct(LSB - 8bits) = Chip ID
    // idProduct(MSB - 8bits) = Family(4Bits:Bit4-Bit7) and Sku(4Bits:Bit0-Bit3)
    // idProduct(LSB) - Read chip ID from the chip ID register
    pDevDescriptor[10] = NV_DRF_VAL(APB_MISC_GP, HIDREV, CHIPID, RegData);
    // idProduct(MSB) - Read family and SKu value and define the MSB
    NvBootFuseGetSkuRaw(&Sku);
#define NVBOOT_USBF_DESCRIPTOR_SKU_MASK  0xF  // descriptor gets only 4 low bits
    pDevDescriptor[11] = (NV_DRF_VAL(APB_MISC_GP, HIDREV, HIDFAM, RegData) << 4) |
        (Sku & NVBOOT_USBF_DESCRIPTOR_SKU_MASK);
    // Device Release number definition (2 bytes as per USB 2.0 Spec)
    // bcd Device (LSB - 8bits) = (Bit4-Bit7 = 0) (Bit0-Bit3 = Minor Rev)
    // bcd Device (MSB - 8bits) = (Bit4-Bit7 = 0) (Bit0-Bit3 = Major Rev)
    // bcd Device (LSB) - Read minor revision
    pDevDescriptor[12] = NV_DRF_VAL(APB_MISC_GP, HIDREV, MINORREV, RegData);
    // bcd Device (MSB) - Read Major revision
    pDevDescriptor[13] = NV_DRF_VAL(APB_MISC_GP, HIDREV, MAJORREV, RegData);
}

/**
 *  Retrieve endpoint status and populate setup buffer
 */
NvBootError NvBootXusbDeviceEPGetStatus(uint16_t EpIndex, uint16_t* txLength, uint8_t* setupDataBuffer)
{
    NvU32 EpStatus;
    NvBootError e = NvBootError_Success;
    uint8_t EndpointStatus[2] = {0, 0};
    // Get the Ep number from 'wIndex' field and  get the 
    // Halt status. 

    /* EpIndex received from Host is of the form: 
     * Byte 1(Bit7 is Dir): Byte 0
     * 8: In : EpNum
     * 0: Out: EpNum
     */

    // EpNum = 2*(EpIndex&0xF);
    // EpNum += (EpIndex&0x80)?1:0;
    DEBUG_PRINT("   |__Endpoint Number ");
    DEBUG_PRINT(ITOA(EpIndex));

    EpStatus  = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0) >> EpIndex;
    // if Halted
    if (EpStatus == 1)
        EndpointStatus[0] = 1;
    else
        EndpointStatus[0] = 0;

    // Don't force EndpointStatus size (2) bytes on host. 
    // If a malformed host packet requests 0 bytes, send 0 bytes.
    *txLength = NV_MIN(*txLength, sizeof(EndpointStatus));
    memcpy((void*)setupDataBuffer, (void*)&EndpointStatus[0], sizeof(EndpointStatus));
    return e;
}


/**
 *  Stall endpoint.
 *  Used during unsupported control transfers or as per host request
 */
NvBootError NvBootXusbDeviceStallEndpoint(Endpoint_T EpIndex, NvU32 Stall)
{
    NvU32 RegData, ExpectedValue, IntPendingMask;
    NvBootError e;
    /* EIndex received from Host is of the form: 
     * Byte 1(Bit7 is Dir): Byte 0
     * 8: In : EpNum
     * 0: Out: EpNum
     */ 
    
    // EpNum = 2*(EpIndex&0xF);
    // EpNum += (EpIndex&0x80)?1:0;
    DEBUG_PRINT("   |__ Stalling Endpoint Number ");
    DEBUG_PRINT(ITOA(EpIndex));

    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0);
    if(Stall)
    {
        RegData |= (1 << EpIndex);
    }
    else
    {
        RegData &= ~(1 << EpIndex);
        // // TODO: Also make sure Endpoint is not paused. Check if this is reqd.
        // RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0);
        // RegData &= ~(1 << EpIndex);
        // NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0, RegData);
    }
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);
    // Poll for state change
    ExpectedValue = 1 << EpIndex;
    IntPendingMask = 1 << EpIndex;

    // Poll for interrupt pending bit.
    e = NvBootPollField(XUSB_BASE+XUSB_DEV_XHCI_EP_STCHG_0,
                        IntPendingMask,
                        ExpectedValue,
                        1000);
    if(e != NvBootError_Success) 
    {
        DEBUG_PRINT("Failed halting endpoint");
        return e;
    }
    NV_WRITE32(XUSB_BASE+XUSB_DEV_XHCI_EP_STCHG_0, 1 << EpIndex);
    return NvBootError_Success;
}

/**
 *  Reload endpoint context and poll for completion
 */
NvBootError NvBootXusbDeviceReloadContext(Endpoint_T EpIndex)
{
    NvU32 RegData, ExpectedValue, Mask;
    NvBootError e;

    RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                         EP_RELOAD,
                         DCI,
                         1 << EpIndex);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_RELOAD_0, RegData);
    Mask = 1 << EpIndex;
    ExpectedValue = 0;
    e = NvBootPollField(XUSB_BASE+XUSB_DEV_XHCI_EP_RELOAD_0,
                        Mask,
                        ExpectedValue,
                        1000);
    return e;
}
 
 /**
  *  Halt/Unhalt endpoint
  */
NvBootError NvBootXusbDeviceHaltEp(Endpoint_T EpIndex, NvU32 Halt)
{
    NvU32 RegData, ExpectedValue, IntPendingMask;
    NvBootError e;
    // Sanitize Halt
    ExpectedValue = (Halt & 0x1)<<EpIndex;
    IntPendingMask = 1<< EpIndex;
    
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0);
    
    if(!Halt)
    {
        RegData &= ~(1<< EpIndex);
    }
    else
    {
        RegData |= (1<< EpIndex);
    }
    
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);

    // Poll for interrupt pending bit.
    e = NvBootPollField((XUSB_BASE+XUSB_DEV_XHCI_EP_HALT_0),
                IntPendingMask,
                ExpectedValue,
                1000);
    return e;
}
  
 /**
  *  Handle port status. Quite a bit happens here.
  */
 NvBootError NvBootXusbDeviceHandlePortStatusChange(void)
{
    NvBootError e = NvBootError_Success;
    NvU32 PortSpeed, PortStatus, RegHalt, StatusBitsMask=0;
    NvU32 RegData, LinkState;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // Let's see why we got here.
    PortStatus = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
    RegHalt = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
    StatusBitsMask = NV_DRF_NUM(XUSB_DEV_XHCI, PORTSC, PRC,1) |
                     NV_DRF_NUM(XUSB_DEV_XHCI, PORTSC, PLC,1) |
                     NV_DRF_NUM(XUSB_DEV_XHCI, PORTSC, WRC,1) |
                     NV_DRF_NUM(XUSB_DEV_XHCI, PORTSC, CSC,1) |
                     NV_DRF_NUM(XUSB_DEV_XHCI, PORTSC, CEC,1);

    
    // Handle PORT RESET. PR indicates reset event received.
    // PR could get cleared (port reset complete) by the time we read it
    // so check on PRC.
    // See device mode IAS 5.1.4.5
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PR, PortStatus))
    {
            DEBUG_PRINT("|__Reset\n\r");
            // This is probably a good time to stop the watchdog timer.

            NvBootWdtStop();
            pXUSBDeviceContext->DeviceState = RESET;
    }
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PRC, PortStatus))
    {
            DEBUG_PRINT("|__Reset Complete\n\r");

            // Must clear PRC
            PortStatus &= ~StatusBitsMask;
            PortStatus = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         PRC,
                                         1,
                                         PortStatus);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, PortStatus);
    }

    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, WPR, PortStatus))
    {
            // This is probably a good time to stop the watchdog timer.
            NvBootWdtStop();
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTHALT,
                                         HALT_LTSSM,
                                         0,   
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0, RegData);
            pXUSBDeviceContext->DeviceState = RESET;
    }
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, WRC, PortStatus))
    {
            
            //*(volatile unsigned int*)0x80000000=PortStatus;
            PortStatus &= ~StatusBitsMask;
            PortStatus = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         WRC,
                                         1,
                                         PortStatus);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, PortStatus);
    }

    PortStatus = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
    // Connect Status Change and Current Connect Status should be 1
    // to indicate successful connection to downstream port.
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, CSC, PortStatus))
    {
            DEBUG_PRINT("|__Connect Status Change\n\r");
            if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, CCS, PortStatus)==1)
            {
                pXUSBDeviceContext->DeviceState = CONNECTED;
                //PortStatus = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
                PortSpeed = NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PS, PortStatus);
                pXUSBDeviceContext->PortSpeed = PortSpeed;
#if XUSB_DEV_SS_SUPPORTED
                // Reload Endpoint Context if not connected in superspeed
                // after changing packet size.
                if(pXUSBDeviceContext->PortSpeed != XUSB_SUPER_SPEED)
                {
                    pEpContext = &EpContext[EP0_IN];
                    pEpContext->AvgTRBLen = 8;
                    pEpContext->MaxPacketSize = 64;
                    RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                                 EP_RELOAD,
                                 DCI,
                                 1 << EP0_IN);
                    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_RELOAD_0, RegData);
                    Mask = 1 << EP0_IN;
                    ExpectedValue = 0;
                    e = NvBootPollField(XUSB_BASE+XUSB_DEV_XHCI_EP_RELOAD_0,
                                        Mask,
                                        ExpectedValue,
                                        2000);
                    if(e != NvBootError_Success)
                       return e;
                    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0);
                    RegData &= ~(1 << EP0_IN);
                NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0, RegData);
                    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0);
                    RegData &= ~(1 << EP0_IN);
                NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);
                }
    #endif
            }
        // if CCS=0, somebody pulled the plug.
        else
            {
                // This will never happen because Vbus is overriden to 1.
                pXUSBDeviceContext->DeviceState = DISCONNECTED;
            }
            PortStatus &= ~StatusBitsMask;
            PortStatus = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         CSC,
                                         1,
                                         PortStatus);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, PortStatus);
    }

    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTHALT, STCHG_REQ, RegHalt))
    {           
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTHALT,
                                         HALT_LTSSM,
                                         0,   
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0, RegData);
    }

    PortStatus = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
    // Port Link Status Change
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PLC, PortStatus))
    {
            DEBUG_PRINT("|__PLC\n\r");
            PortStatus &= ~StatusBitsMask;
            DEBUG_PRINT(
            ITOA(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PLS, PortStatus))
            );

            LinkState = NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PLS, PortStatus);
            if(LinkState == 0x3) // U3 or Suspend
            {
                pXUSBDeviceContext->DeviceState = SUSPENDED;
            }
            else if(LinkState == 0x0 && (pXUSBDeviceContext->DeviceState == SUSPENDED))
            {
                pXUSBDeviceContext->DeviceState = CONFIGURED;
                DEBUG_PRINT("|__EP Pause ");
                RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0);
                DEBUG_PRINT(ITOA(RegData));
                NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0, 0);
                NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_STCHG_0, RegData);
            }
            // This is SS stuff.
            //Check if this is resume from host. If so, put link back to U0.
            // if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PLS, PortStatus)==0xF)
                // PortStatus = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                            // PORTSC,
                                            // PLS,
                                            // 0,
                                            // PortStatus);
            PortStatus = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         PLC,
                                         1,
                                         PortStatus);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, PortStatus);
    }
    // Config Error Change
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, CEC, PortStatus))
    {
            DEBUG_PRINT("|__Error\n\r");
            PortStatus = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            PortStatus &= ~StatusBitsMask;
            PortStatus = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         CEC,
                                         1,
                                         PortStatus);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, PortStatus);
            e = NvBootError_XusbPortError;
    }
    return e;
}

/**
 *  Set xusb pad ownership. This defaults to XUSB post t210 but in previous chips, ownership was muxed
 *  betweeen XUSB and Synopsys.
 */
 void   NvBootXusbDeviceSetPadOwnership()
 {
    NvU32 RegData;
 
    // Setup OTG Pad and BIAS Pad ownership to XUSB
    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PAD_MUX_0);
                
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT0, XUSB, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PAD_MUX_0, RegData);
    
    // Set port cap to device
    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PORT_CAP_0);
                
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PORT_CAP, PORT0_CAP, DEVICE_ONLY, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PORT_CAP_0, RegData);

                            // Set port cap to device
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PORT_CAP_0);
    
    // Map ss port to hs port. This step is required even for HS.
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_SS_PORT_MAP_0);
                
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, SS_PORT_MAP, PORT0_MAP, USB2_PORT0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_SS_PORT_MAP_0, RegData);

 }
 
 /**
  *  Enable changes due to port changes and enabled interrupt.
  */
  void NvBootXusbDeviceInterruptSetup()
  {
    NvU32 RegData;
    
    // Make sure we get events due to port changes.
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                 CTRL,
                                 LSE,
                                 EN,   
                                 RegData);
    RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                 CTRL,
                                 IE,
                                 TRUE,   
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);

  }
  
 /**
  *  Ring doorbell w00t!
  */
  void NvBootXusbDeviceRingDoor(Endpoint_T EpIndex, NvU32 CntrlSeqNum)
  {
    NvU32 RegData;
    RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                                 DB,
                                 TARGET,
                                 EpIndex);
    if(EpIndex ==  EP0_IN)
    {
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     DB,
                                     STREAMID,
                                     CntrlSeqNum,
                                     RegData);
    }
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_DB_0, RegData);
  }
  /**
   *  All that is needed to start enumeration. 
   *  Turn off elpg (low power state)
   *  Turn off interrupt moderation because Bootrom does nothing between polling anyway
   *  Choose de-serializer algorithm
   *  Device enable.
   *  Set vbus override to avoid vbus detection.
   *  This part is done only once and 
   *  protected by a static variable in high level enumerate function.
   */
  void NvBootXusbDeviceEnumerateHw()
  {
      NvU32 RegData;
    NvU32 T210Algo = 0;
     if(NvBootIsPlatformRtl())
     {
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT0_0, 0x12c);  // : FS reset min
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT1_0, 0x2000);// : HS reset min
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT2_0, 0x400); // : HS reset state reset min
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT3_0, 0x2500); // : First chirp
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT4_0, 0x150); // : Chirp min
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT5_0, 0x1800); // : Chirp Max
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT6_0, 0x5400); // : Inactivity timeout //TODO add this under a different function only  for suspend resume/remwk OR program a greater value to consider 1ms frame for FS 
     }

      // Make interrupt moderation =0 to avoid delay between back 2 back interrrupts.
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_RT_IMOD_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     RT_IMOD,
                                     IMODI,
                                     0,   
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_RT_IMOD_0, RegData);

        // Set ELPG=0
        RegData = 0;
        NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_ELPG_PROGRAM_0_0, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_ELPG_PROGRAM_1_0, RegData);

        // Set sw override to vbus
        RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_USB2_VBUS_ID_0);
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_VBUS_ID, VBUS_SOURCE_SELECT, VBUS_OVERRIDE, RegData);
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_VBUS_ID, ID_SOURCE_SELECT, ID_OVERRIDE, RegData);
        NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_USB2_VBUS_ID_0, RegData);

        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     PORTHALT,
                                     HALT_LTSSM,
                                     0,   
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0, RegData);

        // Deserializer selection algorithm.
        // 0: T210, 1: MCP
    T210Algo = NvBootGetSwCYA() & NVBOOT_SW_CYA_DEVICE_DESERIALIZER_ENABLE;

    if(T210Algo)
    {
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_NVWRAP_DESER_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     HSFSPI_NVWRAP_DESER,
                                     MCP_MODE,
                                     0,
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_NVWRAP_DESER_0, RegData);
    }

        // Write Enable for device mode.
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                             CTRL,
                                             ENABLE,
                                             1,
                                             RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);

        
        // Force port reg
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CFG_DEV_FE_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     CFG_DEV_FE,
                                     PORTREGSEL,
                                     2, // HSFS
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CFG_DEV_FE_0, RegData);

        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     PORTSC,
                                     LWS,
                                     1,   
                                     RegData);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     PORTSC,
                                     PLS,
                                     5, // RxDetect
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, RegData);

        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CFG_DEV_FE_0);
        RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     CFG_DEV_FE,
                                     PORTREGSEL,
                                     0,   
                                     RegData);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CFG_DEV_FE_0, RegData);

    // Bug 1645744. No need of the chirp timer.
    // Don't need this for t214 http://nvbugs/1841934
        // RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT16_0);
        // RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     // HSFSPI_COUNT16,
                                     // CHIRP_FAIL,
                                     // 0,   
                                     // RegData);
        // NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_HSFSPI_COUNT16_0, RegData);

        if(NvBootIsPlatformFpga())
        {
            NV_WRITE32(0x700d8200,0x34000);
            RegData = NV_READ32(0x700d8200);
            NvBootUtilWaitUS(1000000);
            // NV_WRITE32(0x7009fc60,0x215441);
        }

        // Set sw override to vbus
        // ID sourced through VPIO is 0 which indicates otg_host.
        // Override ID bit alone to 1. Bug 1383185
        RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_USB2_VBUS_ID_0);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_VBUS_ID, VBUS_OVERRIDE, 1, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_VBUS_ID, ID_OVERRIDE, (1 << 3), RegData);
        NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_USB2_VBUS_ID_0, RegData);      

  }
  
/**
 *  Perform tracking for bias pad. Tracking clock expected to be configured during
 *  clock init.
 */
NvBootError NvBootXusbDevicePerformTracking(NvBootClocksOscFreq OscFreq)
{
    NvU32 RegData;
    (void)OscFreq; // Not used with move to clock tables.
    /* Perform tracking Bug 1440206 */

    // Setting TRK_DONE to PD_TRK assertion/TRK_START de-assertion time.
    // Uncomment this for si. There is an issue during sims so we are using reset val instead of reading hw reg.
    // RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                // XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0);
    RegData = XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0_RESET_VAL;
    // Setting PD_TRK de-assertion to TRK_START. 30 TRK clock cycles
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, TRK_START_TIMER, 0x1E, RegData);
    // Setting TRK_DONE to PD_TRK assertion. 10 TRK clock cycles
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, TRK_DONE_RESET_TIMER, 0xA, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);

    // Power up tracking circuit.
    // RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                // XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, PD_TRK, 0x0, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);

#if NVBOOT_TARGET_RTL
    NvBootUtilWaitUS(10);
#else
    NvBootUtilWaitUS(100);
#endif

    // No need to power down tracking circuit http://nvbugs/200115786
    // RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, PD_TRK, 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        // XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);

    // Giving some time for powerdown.(30 trk clk cycles @0.1us)
    NvBootUtilWaitUS(3);
    return NvBootError_Success;
}

/**
 *  Remove power down for pads.
 */
NvBootError NvBootXusbDeviceRemovePowerDownPad(void)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD_ZI, 0, RegData);
    // PD2 deasserted by hw
    // RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD2, 0, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD, 0, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0, RegData);

    // PD_DR field in XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0 and 
    // XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0 registers
    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, PD_DR, 0, RegData);
    NV_WRITE32( NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                          XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0, RegData);

    //PD fields in XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0
    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_0, PD, 0, RegData);    

    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0, RegData);


    return NvBootError_Success;
}

/**
 *  Set up electrical parameters for pad (parsed from fuses)
 */
NvBootError NvBootXusbDeviceSetupStaticParamsPad(void)
{
    NvU32 RegData, RpdCtrl, HSTermRangeAdj, HSCurrLevel, HSSquelchLevel;
   // Read FUSE_USB_CALIB_EXT_0,FUSE_USB_CALIB_0 register to parse RPD_CTRL, 
   // HS_TERM_RANGE_ADJ, and HS_CURR_LEVEL fields and set corresponding fields in 
   // XUSB_PADCTL_USB2_OTG_PAD0_CTL_0/1 registers
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_0);
    HSTermRangeAdj = NV_DRF_VAL(FUSE, USB_CALIB, TERM_RANGE_ADJ, RegData); //HS_TERM_RANGE_ADJ
    HSCurrLevel = NV_DRF_VAL(FUSE, USB_CALIB, HS_CURR_LEVEL, RegData); //HS_CURR_LEVEL
    HSSquelchLevel = NV_DRF_VAL(FUSE, USB_CALIB, HS_SQUELCH_LEVEL, RegData); //HS_SQUELCH_LEVEL
    
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_EXT_0);
    RpdCtrl = NV_DRF_VAL(FUSE, USB_CALIB_EXT, RPD_CTRL, RegData); //RPD_CTRL

    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, HS_CURR_LEVEL, HSCurrLevel, RegData);
    // Bug 1682605: TERM_SEL has to be flipped to 1 to get the proper values from the USB2 BIAS pad
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, TERM_SEL, 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, TERM_RANGE_ADJ, HSTermRangeAdj, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, RPD_CTRL, RpdCtrl, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0, RegData);

    // Program HS Squelch
    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                            XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_0, HS_SQUELCH_LEVEL, HSSquelchLevel, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0, RegData);


    // With 20 nm soc, pads voltage is 1.5v so enable pad protection circuit
    // against 3.3v
    RegData = NV_READ32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL1_0);
    // USB Pad protection circuit activation bug #1500052 , T186: 1536486 http://nvbugs/1536486/29
    // Decided this is not needed. T214: http://nvbugs/1841934
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD0_CTL1, PD_VREG, 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL1_0, RegData);

    return NvBootError_Success;
}

 void NvBootXusbDeviceBusInit()
 {
     NvU32 RegData;
     
    /// Arc enable needed for xusb device to access iram
    NvBootArcEnable();
    
    NvBootUtilWaitUS(1) ;
    
    // Enable access to fpci space.
    RegData = NV_READ32(NV_XUSB_DEV_IPFS_REGS +
                        XUSB_DEV_CONFIGURATION_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV,
                                 CONFIGURATION,
                                 EN_FPCI,
                                 1,
                                 RegData);
    NV_WRITE32(NV_XUSB_DEV_IPFS_REGS +
                        XUSB_DEV_CONFIGURATION_0, RegData);
                        
    NvBootUtilWaitUS(2) ;

    // BUS_MASTER, MEMORY_SPACE, IO_SPACE ENABLE
    RegData = NV_READ32(NV_XUSB_DEV_APB_DFPCI_CFG +
                        XUSB_DEV_CFG_1_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV,
                                 CFG_1,
                                 MEMORY_SPACE,
                                 1,
                                 RegData);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV,
                                 CFG_1,
                                 BUS_MASTER,
                                 1,
                                 RegData);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV,
                                 CFG_1,
                                 IO_SPACE,
                                 1,
                                 RegData);
    NV_WRITE32(NV_XUSB_DEV_APB_DFPCI_CFG +
                        XUSB_DEV_CFG_1_0, RegData);

    NvBootUtilWaitUS(1);

    // Set BAR Address.
    RegData = 0x700D0000;
    NV_WRITE32(NV_XUSB_DEV_APB_DFPCI_CFG +
                        XUSB_DEV_CFG_4_0, RegData);

 }
