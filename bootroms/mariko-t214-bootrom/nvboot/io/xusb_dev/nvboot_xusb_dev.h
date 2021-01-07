/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#ifndef __XUSB_DEV_H
#define __XUSB_DEV_H
#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_car_int.h"
#include "nvboot_uart_int.h"
#include "project.h"

/****************************** Util Macros ***************************************************/
#define SHIFT(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFT(PERIPH##_##REG##_0_##FIELD##_##RANGE)
#define SHIFTMASK(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFTMASK(PERIPH##_##REG##_0_##FIELD##_##RANGE)


#define XUSB_BASE NV_ADDRESS_MAP_XUSB_DEV_BASE

#define DEBUG_XUSB 0
#if !DEBUG_XUSB
#define DEBUG_PRINT(a) 
#define ITOA(a) 
#define UART_INIT(a,b) (void)a
#else
extern void DEBUG_PRINT(char* Msg);
char* ITOA(NvU32 num);
#endif // DEBUG_XUSB

/****************************** End Util Macros ***************************************************/

/* TRB Types */
#define NONE_TRB                0
#define NORMAL_TRB              1
#define DATA_STAGE_TRB          3
#define STATUS_STAGE_TRB        4
#define LINK_TRB                6
#define TRANSFER_EVENT_TRB      32
#define PORT_STATUS_CHANGE_TRB  34
#define SETUP_EVENT_TRB         63

/* Error codes */
#define TRB_ERR_CODE            5
#define SUCCESS_ERR_CODE        1
#define DATA_BUF_ERR_CODE       2
#define SHORT_PKT_ERR_CODE      13
#define CTRL_SEQ_NUM_ERR_CODE  223
#define CTRL_DIR_ERR_CODE      222

/* XUSB Speed */
#define XUSB_SUPERSPEED 0x4
#define XUSB_HIGHSPEED  0x3
#define XUSB_FULLSPEED  0x2

/* Endpoint ordering */
typedef enum
{
    EP0_IN = 0, // Bi-directional
    EP0_OUT, // Note: This is not used.
    EP1_OUT,
    EP1_IN,
    EPx_Max = 0xFF, // byte size
} Endpoint_T;

typedef enum
{
    DEFAULT,
    CONNECTED,
    DISCONNECTED,
    RESET,
    ADDRESSED_STATUS_PENDING,
    ADDRESSED,
    CONFIGURED_STATUS_PENDING,
    CONFIGURED,
    SUSPENDED,
}   DeviceState_T;

typedef struct
{
    /* As Producer (of Control TRBs) EP0_IN*/
    NvU32 CntrlEpEnqueuePtr;    
    NvU32 CntrlEpDequeuePtr;
    NvU32 CntrlPCS; // Producer Cycle State
    /* As Producer (of Transfer TRBs) for EP1_OUT*/
    NvU32 BulkOutEnqueuePtr;    
    NvU32 BulkOutDequeuePtr;
    NvU32 BulkOutPCS; // Producer Cycle State
    /* As Producer (of Transfer TRBs) for EP1_IN*/
    NvU32 BulkInEnqueuePtr;    
    NvU32 BulkInDequeuePtr;
    NvU32 BulkInPCS; // Producer Cycle State
    /* As Consumer (of Event TRBs) */
    NvU32 EventEnqueuePtr;
    NvU32 EventDequeuePtr;
    NvU32 EventCCS; // Consumer Cycle State
    DeviceState_T DeviceState;
    NvU32 Initialized;
    NvU32 Enumerated;
    NvU32 BytesTxfred;
    NvU32 TxCount;
    NvU32 CntrlSeqNum;
    NvU32 SetupPacketIndex;
    NvU32 ConfigurationNum;
    NvU32 InterfaceNum;
    NvU32 WaitForEvent;
    NvU32 PortSpeed;
}   XUSBDeviceContext_T;
/**
 * Template used to parse event TRBs 
 */
typedef volatile struct
{
    NvU32 Rsvd0;
    NvU32 Rsvd1;
    /* DWord2 begin */
    NvU32 Rsvd2:24;
    NvU32 CompCode:8;     // Completion Code
    /* DWord2 end */
    NvU32 C:1;          // Cycle Bit
    NvU32 Rsvd3:9;
    NvU32 TRBType:6;    // Identifies the type of TRB 
                        // (Setup Event TRB, Port Status Change TRB etc)
    NvU32 EndptId:5;         // Endpoint ID.
    NvU32 Rsvd4:11;
    /* DWord3 end */
} EventTRB_T;

/**
 * Setup Event TRB
 */
typedef volatile struct
{
    NvU32 Data[2];
    /* DWord2 begin */
    NvU32 CtrlSeqNum:16;    // Control Sequence Number
    NvU32 RsvdDW2_0:8;
    NvU32 CompCode:8;       // Completion Code
    /* DWord2 end */
    NvU32 C:1;              // Cycle bit
    NvU32 RsvdDW3_0:9;
    NvU32 TRBType:6;        // TrbType = 63 for Setup Event TRB
    NvU32 EndptId:5;        // Endpoint ID.
    NvU32 RsvdDW3_1:11;
    /* DWord3 end */
} SetupEventTRB_T;

/**
  * Status TRB 
  */
typedef volatile struct
{
    NvU32 RsvDW0;
    NvU32 RsvdDW1;
    /* DWord2 begin */
    NvU32 RsvdDW2_0:22;
    NvU32 IntTarget:10; // Interrupter Target
    /* DWord2 end */
    NvU32 C:1; // Cycle bit
    NvU32 ENT:1; // Evaluate Next TRB
    NvU32 RsvdDW3_0:2;
    NvU32 CH:1; // Chain bit
    NvU32 IOC:1; // Interrupt on Completion
    NvU32 Rsvd4:4;
    NvU32 TRBType:6;
    NvU32 Dir:1;
    NvU32 RsvdDW3_1:15;
    /* DWord3 end */
} StatusTRB_T;

/**
 * Data Status TRB
 */
typedef volatile struct
{
    NvU32 DataBufPtrLo;
    NvU32 DataBufPtrHi;
    /* DWord2 begin */
    NvU32 TRBTxLen:17;
    NvU32 TDSize:5;
    NvU32 IntTarget:10;
    /* DWord2 end */
    NvU32 C:1; // Cycle bit
    NvU32 ENT:1; // Evaluate Next TRB
    NvU32 ISP:1; // Interrupt on Short Packet
    NvU32 NS:1; // No Snoop
    NvU32 CH:1; // Chain bit
    NvU32 IOC:1; // Interrupt on Completion
    NvU32 RsvdDW3_0:4;
    NvU32 TRBType:6;
    NvU32 Dir:1;
    NvU32 RsvdDW3_1:15;
    /* DWord3 end */
} DataTRB_T;

/**
 * Normal TRB
 */
typedef volatile struct
{
    NvU32 DataBufPtrLo;
    NvU32 DataBufPtrHi;
    /* DWord2 begin */
    NvU32 TRBTxLen:17;
    NvU32 TDSize:5;
    NvU32 IntTarget:10;
    /* DWord2 end */
    NvU32 C:1; // Cycle bit
    NvU32 ENT:1; // Evaluate Next TRB
    NvU32 ISP:1; // Interrupt on Short Packet
    NvU32 NS:1; // No Snoop
    NvU32 CH:1; // Chain bit
    NvU32 IOC:1; // Interrupt on Completion
    NvU32 IDT:1; // Immediate Data
    NvU32 RsvdDW3_0:2;
    NvU32 BEI:1; // Block Event Interrupt
    NvU32 TRBType:6;
    NvU32 RsvdDW3_1:16;
    /* DWord3 end */
}   NormalTRB_T;


typedef volatile struct
{
    NvU32 TRBPointerLo;
    NvU32 TRBPointerHi;
    /* DWord1 end */
    NvU32 TRBTxLen:24;
    NvU32 CompCode:8;     // Completion Code
    /* DWord2 end */
    NvU32 C:1;          // Cycle Bit
    NvU32 RsvdDW3_0:1;
    NvU32 ED:1;         // Event Data. (Immediate data in 1st 2 words or not)
    NvU32 RsvdDW3_1:7;
    NvU32 TRBType:6;    // Identifies the type of TRB 
                        // (Setup Event TRB, Port Status Change TRB etc)
    NvU32 EndptId:5;         // Endpoint ID.
    NvU32 RsvdDW3_2:11;
    /* DWord3 end */
} TransferEventTRB_T;

typedef volatile struct
{
    /* DWORD0 */
    NvU32 RsvdDW0_0:4;
    NvU32 RingSegPtrLo:28;
    /* DWORD1 */
    NvU32 RingSegPtrHi;
    /* DWORD2 */
    NvU32 RsvdDW2_0:22;
    NvU32 IntTarget:10;
    /* DWORD3 */
    NvU32 C:1;
    NvU32 TC:1;
    NvU32 RsvdDW3_0:2;
    NvU32 CH:1;
    NvU32 IOC:1;
    NvU32 RsvdDW3_1:4;
    NvU32 TRBType:6;
    NvU32 RsvdDW3_2:16;
}   LinkTRB_T;
/*****************************************************************************/

typedef volatile struct
{
    /* DWORD0 */
    NvU32 EpState:3;
    NvU32 RsvdDW0_0:5;
    NvU32 Mult:2;
    NvU32 MaxPstreams:5;
    NvU32 LSA:1;
    NvU32 Interval:8;
    NvU32 RsvdDW0_1:8;
    /* DWORD1 */
    NvU32 RsvdDW1_0:1;
    NvU32 CErr:2;
    NvU32 EpType:3;
    NvU32 RsvdDW1_1:1;
    NvU32 HID:1;
    NvU32 MaxBurstSize:8;
    NvU32 MaxPacketSize:16;
    /* DWORD2 */
    NvU32 DCS:1;
    NvU32 RsvdDW2_0:3;
    NvU32 TRDequeuePtrLo:28;
    /* DWORD3 */
    NvU32 TRDequeuePtrHi;
    /* DWORD4 */
    NvU32 AvgTRBLen:16;
    NvU32 MaxESITPayload:16;
    /******* Nvidia specific from here ******/
    /* DWORD5 */
    NvU32 EventDataTxLenAcc:24;
    NvU32 RsvdDW5_0:1;
    NvU32 PTD:1;
    NvU32 SXS:1;
    NvU32 SeqNum:5;
    /* DWORD6 */
    NvU32 CProg:8;
    NvU32 SByte:7;
    NvU32 TP:2;
    NvU32 REC:1;
    NvU32 CEC:2;
    NvU32 CED:1;
    NvU32 HSP1:1;
    NvU32 RTY1:1;
    NvU32 STD:1;
    NvU32 Status:8;
    /* DWORD7 */
    NvU32 DataOffset:17;
    NvU32 RsvdDW6_0:4;
    NvU32 LPA:1;
    NvU32 NumTRB:5;
    NvU32 NumP:5;
    /* DWORD8 */
    NvU32 ScratchPad0;
    /* DWORD8 */
    NvU32 ScratchPad1;
    /* DWORD10 */
    NvU32 CPing:8;
    NvU32 SPing:8;
    NvU32 TC:2;
    NvU32 NS:1;
    NvU32 RO:1;
    NvU32 TLM:1;  
    NvU32 DLM:1;
    NvU32 HSP2:1;
    NvU32 RTY2:1;
    NvU32 StopRecReq:8;
    /* DWORD11 */
    NvU32 DeviceAddr:8;
    NvU32 HubAddr:8;
    NvU32 RootPortNum:8;
    NvU32 SlotID:8;
    /* DWORD12 */
    NvU32 RoutingString:20;
    NvU32 Speed:4;
    NvU32 LPU:1;
    NvU32 MTT:1;
    NvU32 Hub:1;
    NvU32 DCI:5;
    /* DWORD13 */
    NvU32 TTHubSlotID:8;
    NvU32 TTPortNum:8;
    NvU32 SSF:4;
    NvU32 SPS:2;
    NvU32 IntTarget:10;
    /* DWORD14 */
    NvU32 FRZ:1;
    NvU32 END:1;
    NvU32 ELM:1;
    NvU32 MRX:1;
    NvU32 EpLinkLo:28;
    /* DWORD15 */
    NvU32 EpLinkHi;
}   EpContext_T;

typedef struct
{
    NvU32 LastEventTick;
    NvU32 LastEventElapsed[128];
}   XUSBDeviceTime_T;

/**
 * Structure defining the fields for USB PLLU configuration Parameters.
 */
typedef struct UsbPllClockParamsRec
{
    //PLL feedback divider.
    NvU32 N;
    //PLL input divider.
    NvU32 M;
    //post divider
    NvU32 P;
} UsbPllClockParams;

/**
 * Structure defining the fields for UTMI PLL configuration Parameters.
 */
typedef struct UtmiPllClockParamsRec
{
    //PLL feedback divider.
    NvU32 N;
    //PLL input divider.
    NvU32 M;
} UtmiPllClockParams;

/**
 * Structure defining the fields for USB UTMI clocks delay Parameters.
 */
typedef struct UsbPllDelayParamsRec
{
    // Pll-U Enable Delay Count
    uint16_t EnableDelayCount;
    //PLL-U Stable count
    uint16_t StableCount;
    //Pll-U Active delay count
    uint16_t ActiveDelayCount;
    //PLL-U Xtal frequency count
    uint16_t XtalFreqCount;
} UsbPllDelayParams;

/************************** Chip independent /High level routines *********************************/
 NvBootError NvBootXusbDeviceQueueTRB(Endpoint_T EpIndex, NormalTRB_T* pTRB, NvU32 RingDoorBell);
 NvBootError NvBooXusbDeviceInitEpContext(Endpoint_T EpIndex);
 NvBootError NvBootXusbDeviceInitEndpoint(Endpoint_T EPIndex);
 NvBootError NvBootXusbDeviceStallEndpoint(Endpoint_T EPIndex, NvU32 Stall);
 NvBootError
 NvBootXusbDeviceCreateNormalTRB(NormalTRB_T *pNormalTRB, NvU32 Buffer,
                                 NvU32 Bytes, NvU32 Dir);
 NvBootError NvBootXusbDeviceCreateDataTRB(DataTRB_T *pDataTRB, NvU32 Buffer, 
                                           NvU32 Bytes, NvU32 Dir);
 NvBootError NvBootXusbDeviceCreateStatusTRB(StatusTRB_T *pStatusTRB, NvU32 Dir);
 NvBootError NvBootXusbDeviceHandleTransferEvent(TransferEventTRB_T *pTxEventTRB);
 NvBootError NvBootXusbDeviceDisableEndpoint(Endpoint_T EpIndex);
 
 NvBootError NvBootXusbDevicePollForEvent(NvU32 Timeout);
 NvBootError NvBootXusbDeviceSetConfiguration(uint8_t* pSetupData);
 NvBootError NvBootXusbDeviceSetAddress(uint8_t* pSetupData);
 NvBootError NvBootXusbDeviceSetInterface(uint8_t* pSetupData);
 NvBootError NvBootXusbDeviceGetDescriptor(uint8_t* pSetupData, uint16_t* txLength, uint8_t* SetupDataBuffer);
 NvBootError NvBootXusbDeviceEPGetStatus(uint16_t EpIndex, uint16_t* txLength, uint8_t* setupDataBuffer);
 NvBootError
 NvBootXusbDeviceIssueDataTRB(NvU32 Buffer, NvU32 Bytes, NvU32 Direction);
 NvBootError NvBootXusbDeviceIssueStatusTRB(NvU32 Direction);
 NvBootError NvBootXusbDeviceHandleSetupPacket( uint8_t* pSetupData);
 NvBootError NvBootXusbDeviceHandlePortStatusChange(void);
 NvBootError NvBootXusbDevicePadSetup(void);
 NvBootError NvBootXusbDeviceSetupHWControlPLLU(void);
 NvBootError NvBootXusbDeviceSetupStaticParamsPad(void);
 NvBootError NvBootXusbDeviceRemovePowerDownPad(void);
 NvBootError NvBootXusbDeviceClkInit(void);

/* End Chip independent /High level routines */
 
/********************************** Chip specific routines ****************************************/
/**
 *  Enable changes due to port changes and enabled interrupt.
 */
 void NvBootXusbDeviceInterruptSetup();
 
 /**
  *  Ring doorbell w00t!
  */
  void NvBootXusbDeviceRingDoor(Endpoint_T EpIndex, NvU32 CntrlSeqNum);
  
/**
 *  Set xusb pad ownership. This defaults to XUSB post t210 but in previous chips, ownership was muxed
 *  betweeen XUSB and Synopsys.
 */
 void NvBootXusbDeviceSetPadOwnership();
 
/**
 *  Handle port status. Quite a bit happens here.
 */
 NvBootError NvBootXusbDeviceHandlePortStatusChange();
 
/**
 *  Reload endpoint context and poll for completion
 */
 NvBootError NvBootXusbDeviceReloadContext(Endpoint_T EpIndex);
 
/**
 *  Stall endpoint.
 *  Used during unsupported control transfers or as per host request
 */
 NvBootError NvBootXusbDeviceStallEndpoint(Endpoint_T EpIndex, NvU32 Stall);

 /**
  *  Halt/Unhalt endpoint
  */
 NvBootError NvBootXusbDeviceHaltEp(Endpoint_T EpIndex, NvU32 Halt);

 /**
  *  USB descriptor is updated to reflect chip specific data
  *  CHIP
  *  FAM (Handheld) | SKU (0-15)
  *  MAJORREV 
  *  MINORREV
  */
 void NvBootXusbDeviceSetPidRev(uint8_t *pDevDescriptor);
 
/**
 *  Set run status
 */
 void NvBootXusbDeviceSetRunStatus(NvU32 Run);
 
/**
 *  Clear Event handler busy and update dequeue pointer.
 */
 void NvBootXusbDeviceUpdateEventDequeuePtr(NvU32 EventDequeuePtr);
 
/**
 *  Poll for interrupt status and give Event enqueue pointer.
 */
 NvBootError NvBootXusbDevicePollforInterrupt(NvU32 Timeout, NvU32 *EventEnqueuePtr);
 
 
/**
 *  Setup Event ring.
 *  Note: Event ring region should be capable of holding NumTRB
 */
 NvBootError NvBootXusbDeviceInitializeEventRing(NvU32 EventRing, NvU32 NumTRB);
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
void NvBootXusbDeviceEnumerateHw();

/**
 *  Remove power down for pads.
 */
 NvBootError NvBootXusbDeviceRemovePowerDownPad(void);
 
/**
 *  Set up electrical parameters for pad (parsed from fuses)
 */
NvBootError NvBootXusbDeviceSetupStaticParamsPad(void);
 
#endif
