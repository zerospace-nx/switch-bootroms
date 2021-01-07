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
#include "nvboot_xusb_dev_hw.h"
#include "nvboot_xusb_dev_int.h"
#include "ardev_t_xusb_dev_xhci.h"
#include "ardev_t_fpci_xusb_dev_0.h"
#include "arxusb_padctl.h"
#include "arfuse.h"
#include "nvrm_drf.h"
#include "nvboot_util_int.h"
#include "nvboot_dispatcher_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_wdt_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_car_int.h"
#include "nvboot_platform_int.h"
#include "nvboot_fuse_int.h"
#include "arapb_misc.h"
#include "armc.h"
#include "project.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_address_int.h"
#include "nvboot_config.h"


#define XUSB_DEV_SS_SUPPORTED 0
#define XUSB_FULL_SPEED  1
#define XUSB_HIGH_SPEED  3
#define XUSB_SUPER_SPEED 4
#define NV_TRUE 1
#define NV_FALSE 0

#define ALIGN_ADDR(ADDR, BYTES) ((((ADDR)+(BYTES)-1)/(BYTES)) * (BYTES))
/** We need 2 ring segments of size 16 each for event ring. 
 *  Use 1 contiguous segment for simplicity.
 */
#define NUM_TRB_EVENT_RING 32
#define NUM_TRB_TRANSFER_RING 16
#define NUM_EP_CONTEXT  4
#define EVENT_RING_WRAP_AROUND(a)      \
        (a==&EventRing[NUM_TRB_EVENT_RING-1]?&EventRing[0]:((a)+1))

#define SETUP_DATA_BUFFER_SIZE     (0x200)
             
/**************************** Globals **************************************/

uint8_t SetupData[8] __attribute__((aligned(4)));
// static EventTRB_T* EventRing = (EventTRB_T*)EVENT_RING_START;
static EventTRB_T EventRing[NUM_TRB_EVENT_RING] __attribute__((aligned(16)));
// static DataTRB_T* TxRingEp0 = (DataTRB_T*)TX_RING_EP0_START;
// Transfer Ring for Control Endpoint
#define NUM_TRB_TRANSFER_RING 16
static DataTRB_T TxRingEp0[NUM_TRB_TRANSFER_RING] __attribute__((aligned(16)));
// Transfer Ring for Bulk Out Endpoint
static DataTRB_T TxRingEp1Out[NUM_TRB_TRANSFER_RING]
                        __attribute__((aligned(16)));
// Transfer Ring for Bulk In Endpoint
static DataTRB_T TxRingEp1In[NUM_TRB_TRANSFER_RING]
                        __attribute__((aligned(16)));

// static DataTRB_T* TxRingEp1Out = (DataTRB_T*)TX_RING_EP1_OUT_START;
// static DataTRB_T* TxRingEp1In = (DataTRB_T*)TX_RING_EP1_IN_START;
//static uint8_t* SetupDataBuffer = (uint8_t*)SETUP_DATA_BUFFER_START;
static uint8_t SetupDataBuffer[SETUP_DATA_BUFFER_SIZE]  __attribute__((aligned(4)));;
/** Endpoint Context */
// EpContext_T *EpContext = (EpContext_T *)EP_CONTEXT_START;
// Endpoint descriptor
static EpContext_T EpContext[NUM_EP_CONTEXT] __attribute__((aligned(64)));
/** Holds all sorts of useful information */
XUSBDeviceContext_T XUSBDeviceContext;

/** End Globals */
#if DEBUG_XUSB
#define UART_INIT(a,b)    NvBootUartInit(a, b)

void DEBUG_PRINT(char* Msg)
{
    int BytesPrinted;
    char *Tmp=Msg;
    while(*Tmp!='\0')
    {
        Tmp++;
    }
    NvBootUartPollingWrite((const uint8_t*)Msg, Tmp-Msg,(size_t*)&BytesPrinted);
}
char* ITOA(NvU32 num)
{
    static char buffer[11];
    int n=0, i;
    char tmp;
    //NvBootUtilMemset(Buffer, 0 , 8);
    // We have reserved space for 8 ASCII chars, new line and carriage return
    do
    {
        buffer[n++]=(num%0x10)<0xA?48+(num%0x10):55+(num%0x10);
        num=num/0x10;
    } while(num && (n<8));
    for(i=0;i<n/2;i++)
    {
        tmp = buffer[n-i-1];
        buffer[n-i-1]=buffer[i];
        buffer[i]=tmp;
    }
    buffer[n++]='\n';
    buffer[n++]='\r';
    buffer[n]='\0';
    return (char*)&buffer[0];
}
#else
#define DEBUG_PRINT(a) 
#define ITOA(a) 
#define UART_INIT(a,b) (void)a
#endif // DEBUG_XUSB

/*********************************** USB protocol macros and globals ******************************/
#define EP_RUNNING  1
#define EP_DISABLED 0

#define DIR_OUT 0
#define DIR_IN 1

#define USB_DEV_DESCRIPTOR_SIZE 18
#define USB_BOS_DESCRIPTOR_SIZE 15
#define USB_CONFIG_DESCRIPTOR_SIZE 32
#define USB_MANF_STRING_LENGTH 26
#define USB_PRODUCT_STRING_LENGTH 8
#define USB_SERIAL_NUM_LENGTH  12
#define USB_LANGUAGE_ID_LENGTH 4
#define USB_DEV_QUALIFIER_LENGTH 10
#define USB_DEV_STATUS_LENGTH

#define USB_LANGUAGE_ID   0
#define USB_MANF_ID       1
#define USB_PROD_ID       2
#define USB_SERIAL_ID     3
#define USB_DEVICE_SELF_POWERED 1

// Feature Select
#define ENDPOINT_HALT        0
#define DEVICE_REMOTE_WAKEUP 1
#define TEST_MODE            2

// USB Setup Packet Byte Offsets
#define USB_SETUP_REQUEST_TYPE  0
#define USB_SETUP_REQUEST       1
#define USB_SETUP_VALUE         2
#define USB_SETUP_DESCRIPTOR    3
#define USB_SETUP_INDEX         4
#define USB_SETUP_LENGTH        6 

// USB Setup Packet Request Type
#define HOST2DEV_DEVICE     0x00
#define HOST2DEV_INTERFACE  0x01
#define HOST2DEV_ENDPOINT   0x02
#define DEV2HOST_DEVICE     0x80
#define DEV2HOST_INTERFACE  0x81
#define DEV2HOST_ENDPOINT   0x82

// USB Setup Packet Request
#define GET_STATUS        0
#define CLEAR_FEATURE     1
#define SET_FEATURE       3
#define SET_ADDRESS       5
#define GET_DESCRIPTOR    6
#define SET_DESCRIPTOR    7
#define GET_CONFIGURATION 8
#define SET_CONFIGURATION 9
#define GET_INTERFACE     10

// USB Descriptor Type
#define USB_DT_DEVICE             1
#define USB_DT_CONFIG             2
#define USB_DT_STRING             3
#define USB_DT_INTERFACE          4
#define USB_DT_ENDPOINT           5
#define USB_DT_DEVICE_QUALIFIER   6
#define USB_DT_OTHER_SPEED_CONFIG 7
#define USB_DT_BOS                15

#if XUSB_DEV_SS_SUPPORTED
#define BCDUSB_VERSION_LSB 0
#define BCDUSB_VERSION_MSB 3
#define EP0_PKT_SIZE 9
#else
#define BCDUSB_VERSION_LSB 0
#define BCDUSB_VERSION_MSB 2
#define EP0_PKT_SIZE 64
#endif
/**
 * USB Device Descriptor: 12 bytes as per the USB2.0 Specification
 * Stores the Device descriptor data must be word aligned
 */
__attribute__((aligned(4))) static uint8_t s_UsbDeviceDescriptor[USB_DEV_DESCRIPTOR_SIZE] =
{
    USB_DEV_DESCRIPTOR_SIZE,   // bLength - Size of this descriptor in bytes
    0x01,   // bDescriptorType - Device Descriptor Type
    BCDUSB_VERSION_LSB,   // bcd USB (LSB) - USB Spec. Release number
    BCDUSB_VERSION_MSB,   // bcd USB (MSB) - USB Spec. Release number (2.0)
    0x00,   // bDeviceClass - Class is specified in the interface descriptor.
    0x00,   // bDeviceSubClass - SubClass is specified in the interface descriptor.
    0x00,   // bDeviceProtocol - Protocol is specified in the interface descriptor.
    EP0_PKT_SIZE,   // bMaxPacketSize0 - Maximum packet size for EP0
    0x55,   // idVendor(LSB) - Vendor ID assigned by USB forum
    0x09,   // idVendor(MSB) - Vendor ID assigned by USB forum
    0x15,   // idProduct(LSB) - Product ID assigned by Organization
    0x71,   // idProduct(MSB) - Product ID assigned by Organization
    0x01,   // bcd Device (LSB) - Device Release number in BCD
    0x01,   // bcd Device (MSB) - Device Release number in BCD
    USB_MANF_ID,   // Index of String descriptor describing Manufacturer
    USB_PROD_ID,   // Index of String descriptor describing Product
    0,   // Index of String descriptor describing Serial number
    0x01   // bNumConfigurations - Number of possible configuration
};

#if XUSB_DEV_SS_SUPPORTED
/**
 * USB BOS Descriptor:
 * Stores the Device descriptor data must be word aligned
 */
__attribute__((aligned(4))) static uint8_t s_UsbBOSDescriptor[USB_BOS_DESCRIPTOR_SIZE] =
{
    0x5,   // bLength - Size of this descriptor in bytes
    0xF,    // bDescriptorType - BOS Descriptor Type
    0xF,    // wTotalLength LSB
    0x0,    // wTotalLength MSB
    0x1,    // NumDeviceCaps
    0xA,    // bLength - Size of Super Speed Device Capability Descriptor
    0x10,   // bDescriptorType - Device Capability Type
    0x3,    // bDevCapabilityType - SUPERSPEED USB
    0x2,    // bmAttributes - Bit 1 Device Capable of generating Latency Tolerace Msgs
    0xA,    // wSpeedsSupported LSB - Device Supports Full Speed and Super Speed *TODO* change to HS
    0x0,    // wSpeedsSupported MSB
    0x1,    // bFunctionalitySupport - All features available above FS.
    0xA,    // bU1DevExitLat - Less than 10us *TODO*
    0xFF,   // wU2DevExitLat LSB
    0x7,    // wU2DevExitLat MSB - Less than 2047us *TODO*
};
#endif

#if XUSB_DEV_SS_SUPPORTED
#define EP1_IN_PKT_SIZE_LSB 0
#define EP1_IN_PKT_SIZE_MSB 4
#define EP1_OUT_PKT_SIZE_LSB 0
#define EP1_OUT_PKT_SIZE_MSB 4
#else
#define EP1_IN_PKT_SIZE_LSB 64
#define EP1_IN_PKT_SIZE_MSB 0
#define EP1_OUT_PKT_SIZE_LSB 64
#define EP1_OUT_PKT_SIZE_MSB 0
#endif
__attribute__((aligned(4))) static uint8_t s_UsbConfigDescriptor[USB_CONFIG_DESCRIPTOR_SIZE] =
{
    /* Configuration Descriptor 32 bytes  */
    0x09,   // bLength - Size of this descriptor in bytes
    0x02,   // bDescriptorType - Configuration Descriptor Type
    0x20,   // WTotalLength (LSB) - Total length of data for this configuration
    0x00,   // WTotalLength (MSB) - Total length of data for this configuration
    0x01,   // bNumInterface - Nos of Interface supported by this configuration
    0x01,   // bConfigurationValue
    0x00,   // iConfiguration - Index of descriptor describing this configuration
    0xc0,   // bmAttributes - bitmap "D4-D0: Res, D6: Self Powered,D5: Remote Wakeup
    0x10,   // MaxPower in mA - Max Power Consumption of the USB device

    /* Interface Descriptor */
    0x09,   // bLength - Size of this descriptor in bytes
    0x04,   // bDescriptorType - Interface Descriptor Type
    0x00,   // bInterfaceNumber - Number of Interface
    0x00,   // bAlternateSetting - Value used to select alternate setting
    0x02,   // bNumEndpoints - Nos of Endpoints used by this Interface
    0xFF,   // bInterfaceClass - Class code "Vendor Specific Class."
    0xFF,   // bInterfaceSubClass - Subclass code "Vendor specific".
    0xFF,   // bInterfaceProtocol - Protocol code "Vendor specific".
    0x00,   // iInterface - Index of String descriptor describing Interface

    /* Endpoint Descriptor IN EP1 */
    0x07,   // bLength - Size of this descriptor in bytes
    0x05,   // bDescriptorType - ENDPOINT Descriptor Type
    0x81,   // bEndpointAddress - The address of EP on the USB device 
    0x02,   // bmAttributes - Bit 1-0: Transfer Type 10: Bulk, 
    EP1_IN_PKT_SIZE_LSB,   // wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    EP1_IN_PKT_SIZE_MSB,   // wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00,   // bInterval - Interval for polling EP, for Interrupt and Isochronous 

    /** Endpoint Descriptor OUT EP1 */
    0x07,   // bLength - Size of this descriptor in bytes
    0x05,   // bDescriptorType - ENDPOINT Descriptor Type
    0x01,   // bEndpointAddress - The address of EP on the USB device 
    0x02,   // bmAttributes - Bit 1-0: Transfer Type 10: Bulk, 
    EP1_OUT_PKT_SIZE_LSB,   // wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    EP1_OUT_PKT_SIZE_MSB,   // wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00    // bInterval - Interval for polling EP, for Interrupt and Isochronous 
};

__attribute__((aligned(4))) static uint8_t s_OtherSpeedConfigDesc[32]= {

    /** Other speed Configuration Descriptor */
    0x09,   /// bLength - Size of this descriptor in bytes
    0x07,   /// bDescriptorType - Other speed Configuration Descriptor Type
    0x20,   /// WTotalLength (LSB) - Total length of data for this configuration
    0x00,   /// WTotalLength (MSB) - Total length of data for this configuration
    0x01,   /// bNumInterface - Nos of Interface supported by this configuration
    0x01,   /// bConfigurationValue
    0x00,   /// iConfiguration - Index of String descriptor describing this configuration
    0xc0,   /// bmAttributes - Config Characteristcs bitmap "D4-D0: Res, D6: Self Powered,D5: Remote Wakeup
    0x10,   /// MaxPower in mA - Max Power Consumption of the USB device

    /**Interface Descriptor */
    0x09,   /// bLength - Size of this descriptor in bytes
    0x04,   /// bDescriptorType - Interface Descriptor Type
    0x00,   /// bInterfaceNumber - Number of Interface
    0x00,   /// bAlternateSetting - Value used to select alternate setting
    0x02,   /// bNumEndpoints - Nos of Endpoints used by this Interface
    0x08,   /// bInterfaceClass - Class code "MASS STORAGE Class."
    0x06,   /// bInterfaceSubClass - Subclass code "SCSI transparent command set"
    0x50,   /// bInterfaceProtocol - Protocol code "BULK-ONLY TRANSPORT."
    0x00,   /// iInterface - Index of String descriptor describing Interface

    /** Endpoint Descriptor IN EP2 */
    0x07,   /// bLength - Size of this descriptor in bytes
    0x05,   /// bDescriptorType - ENDPOINT Descriptor Type
    0x81,   /// bEndpointAddress - The address of EP on the USB device "Bit 7: Direction(0:OUT, 1: IN),Bit 6-4:Res,Bit 3-0:EP no"
    0x02,   /// bmAttributes - Bit 1-0: Transfer Type 00:Control,01:Isochronous,10: Bulk, 11: Interrupt
    0x40,   /// wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    0x00,   /// wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00,   /// bIntervel - Interval for polling EP, applicable for Interrupt and Isochronous data transfer only

    /** Endpoint Descriptor OUT EP1 */
    0x07,   /// bLength - Size of this descriptor in bytes
    0x05,   /// bDescriptorType - ENDPOINT Descriptor Type
    0x01,   /// bEndpointAddress - The address of EP on the USB device "Bit 7: Direction(0:OUT, 1: IN),Bit 6-4:Res,Bit 3-0:EP no"
    0x02,   /// bmAttributes - Bit 1-0: Transfer Type 00:Control,01:Isochronous,10: Bulk, 11: Interrupt
    0x40,   /// wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    0x00,   /// wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00    /// bIntervel - Interval for polling EP, applicable for Interrupt and Isochronous data transfer only
};


// Stores the Manufactures ID sting descriptor data
__attribute__((aligned(4))) static const uint8_t s_UsbManufacturerID[USB_MANF_STRING_LENGTH] =
{
    USB_MANF_STRING_LENGTH,  // Length of descriptor
    0x03,                    // STRING descriptor type.
    'N', 0,
    'V', 0,
    'I', 0,
    'D', 0,
    'I', 0,
    'A', 0,
    ' ', 0,
    'C', 0,
    'o', 0,
    'r', 0,
    'p', 0,
    '.', 0

};

// Stores the Product ID string descriptor data
__attribute__((aligned(4))) static const uint8_t s_UsbProductID[USB_PRODUCT_STRING_LENGTH] =
{
    USB_PRODUCT_STRING_LENGTH, // Length of descriptor
    0x03,                      // STRING descriptor type.
    'A', 0x00,
    'P', 0x00,
    'X', 0x00
};

// Stores the Serial Number String descriptor data (Not used for AP15 Bootrom)
__attribute__((aligned(4))) static const uint8_t s_UsbSerialNumber[USB_SERIAL_NUM_LENGTH] =
{
    USB_SERIAL_NUM_LENGTH, // Length of descriptor
    0x03,                  // STRING descriptor type.
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00,
    '0', 0x00
};

// Stores the Language ID Descriptor data
__attribute__((aligned(4))) static const uint8_t s_UsbLanguageID[USB_LANGUAGE_ID_LENGTH] =
{
    /* Language Id string descriptor */
    USB_LANGUAGE_ID_LENGTH,  // Length of descriptor
    0x03,                    // STRING descriptor type.
    0x09, 0x04               // LANGID Code 0: American English 0x409
};

// Stores the Device Qualifier Desriptor data
__attribute__((aligned(4))) static const uint8_t s_UsbDeviceQualifier[USB_DEV_QUALIFIER_LENGTH] =
{
    /* Device Qualifier descriptor */
    USB_DEV_QUALIFIER_LENGTH,   // Size of the descriptor
    6,    // Device Qualifier Type
    0x00, // USB specification version number: LSB
    0x02, //  USB specification version number: MSB
    0xFF, // Class Code
    0xFF, // Subclass Code
    0xFF, // Protocol Code
    0x40, //Maximum packet size for other speed
    0x01, //Number of Other-speed Configurations
    0x00  // Reserved for future use, must be zero
};


// Stores the Device Status descriptor data
__attribute__((aligned(4))) static const uint8_t s_UsbDeviceStatus[USB_DEV_STATUS_LENGTH] =
{
    USB_DEVICE_SELF_POWERED,
    0,
};

/** End USB protocol macros and globals **/

NvBootError NvBootXusbDeviceInitializeTransferRing(Endpoint_T EpIndex)
{
    /* zero out tx ring */
    if(EpIndex == EP0_IN || EpIndex == EP0_OUT)
        NvBootUtilMemset((void*)&TxRingEp0[0], 0, NUM_TRB_TRANSFER_RING*sizeof(EventTRB_T));
    else if(EpIndex == EP1_IN)
        NvBootUtilMemset((void*)&TxRingEp1In[0], 0, NUM_TRB_TRANSFER_RING*sizeof(EventTRB_T));
    else if(EpIndex == EP1_OUT)
        NvBootUtilMemset((void*)&TxRingEp1Out[0], 0, NUM_TRB_TRANSFER_RING*sizeof(EventTRB_T));
    else return NvBootError_InvalidParameter;

    return NvBootError_Success;
}

NvBootError NvBootXusbDevicePollForEvent(NvU32 Timeout)
{
    NvBootError e = NvBootError_Success;
    NvU32 EventEnqueuePtr;
    volatile EventTRB_T *pEventTRB;
    volatile SetupEventTRB_T *pSetupEventTRB;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext = &XUSBDeviceContext;

    e = NvBootXusbDevicePollforInterrupt(Timeout, &EventEnqueuePtr);
    if(e != NvBootError_Success)
        return e;

    pXUSBDeviceContext->EventEnqueuePtr = EventEnqueuePtr;

    pEventTRB = (EventTRB_T*)pXUSBDeviceContext->EventDequeuePtr;
    // Make sure cycle state matches
    if(pEventTRB->C !=  pXUSBDeviceContext->EventCCS)
    {
        return NvBootError_DeviceError;
    }

    
    while(pEventTRB->C ==  pXUSBDeviceContext->EventCCS)
    {        

        if(pEventTRB->TRBType == SETUP_EVENT_TRB)
        {
            DEBUG_PRINT("Setup Event\n\r");

            // Check if we are waiting for setup packet
            
            pSetupEventTRB = (SetupEventTRB_T*)pEventTRB;
                NvBootUtilMemcpy((void*)&SetupData[0],
                   (void*)&pSetupEventTRB->Data[0],
                    8);
                pXUSBDeviceContext->CntrlSeqNum = pSetupEventTRB->CtrlSeqNum;
                e = NvBootXusbDeviceHandleSetupPacket(&SetupData[0]);
        }
        else if(pEventTRB->TRBType == PORT_STATUS_CHANGE_TRB)
        {
            DEBUG_PRINT("Port Status\n\r");
            // Handle all port status changes here.
            e = NvBootXusbDeviceHandlePortStatusChange();
        }
        else if(pEventTRB->TRBType == TRANSFER_EVENT_TRB)
        {  
            DEBUG_PRINT("Transfer Event\n\r");
            // Handle tx event changes here.
            e = NvBootXusbDeviceHandleTransferEvent((TransferEventTRB_T*)pEventTRB);
        }
            // Increment Event Dequeue Ptr.
            // Check if last element of ring to wrap around and toggle cycle bit.
            if(pXUSBDeviceContext->EventDequeuePtr == (NvU32)&EventRing[NUM_TRB_EVENT_RING-1])
            {
                pXUSBDeviceContext->EventDequeuePtr = (NvU32)&EventRing[0];
                pXUSBDeviceContext->EventCCS ^=1;
            }
            else
                pXUSBDeviceContext->EventDequeuePtr += sizeof(EventTRB_T);
        pEventTRB = (EventTRB_T*)pXUSBDeviceContext->EventDequeuePtr;

            // Process only events posted when interrupt was triggered.
            // New posted events will be handled during the next interrupt handler call.
            if(pXUSBDeviceContext->EventDequeuePtr ==  pXUSBDeviceContext->EventEnqueuePtr)
                break;
    }

    NvBootXusbDeviceUpdateEventDequeuePtr(pXUSBDeviceContext->EventDequeuePtr);
    
    return e;   
}
NvBootError NvBootXusbDeviceSetConfiguration(uint8_t* pSetupData)
{
    uint16_t wValue;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    EpContext_T *pEpContext;
    NvBootError e;

    // Last stage of enumeration.
    wValue = pSetupData[USB_SETUP_VALUE] + (pSetupData[USB_SETUP_VALUE+1] << 8);
    DEBUG_PRINT("   |__Config Number ");
    DEBUG_PRINT(ITOA(wValue));

    // If we get a set config 0, then disable endpoints and remain in addressed
    // state.
    // If we had already set a config before this request, then do the same but 
    // also enable bulk endpoints after and set run bit.

    if(pXUSBDeviceContext->ConfigurationNum || wValue == 0)
    {
        // Disable Endpoint Ep1 In
        pEpContext = &EpContext[EP1_IN];
        pEpContext->EpState = EP_DISABLED;
        
        DEBUG_PRINT("   |__ Disabling Endpoint Number ");
        DEBUG_PRINT(ITOA(EP1_IN));
        e = NvBootXusbDeviceReloadContext(EP1_IN);
        if(e != NvBootError_Success)
            return e;

        // Disable Endpoint Ep1 Out
        DEBUG_PRINT("   |__ Disabling Endpoint Number ");
        DEBUG_PRINT(ITOA(EP1_OUT));
        pEpContext = &EpContext[EP1_OUT];
        pEpContext->EpState = EP_DISABLED;
        
        e = NvBootXusbDeviceReloadContext(EP1_OUT);
        if(e != NvBootError_Success)
            return e;

        // Stop controller i.e. Set RUN to stop
        NvBootXusbDeviceSetRunStatus(0);
    }

    if(wValue)
    {
        NvBootXusbDeviceInitEndpoint(EP1_OUT);
        NvBootXusbDeviceInitEndpoint(EP1_IN);

        // Now set run
        NvBootXusbDeviceSetRunStatus(1);
    }

    // Send status
    NvBootXusbDeviceIssueStatusTRB(DIR_IN);
    pXUSBDeviceContext->ConfigurationNum = wValue;

    // Change device state only for non-zero configuration number
    // Otherwise device remains in addressed state.
    if(wValue)
        pXUSBDeviceContext->DeviceState = CONFIGURED_STATUS_PENDING;
    else
        pXUSBDeviceContext->DeviceState = ADDRESSED_STATUS_PENDING;
    return NvBootError_Success;
}

NvBootError NvBootXusbDeviceSetInterface(uint8_t* pSetupData)
{
    uint16_t wValue;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    wValue = pSetupData[USB_SETUP_VALUE] + (pSetupData[USB_SETUP_VALUE+1] << 8);
    DEBUG_PRINT("   |__Interface Number ");
    DEBUG_PRINT(ITOA(wValue));

    pXUSBDeviceContext->InterfaceNum = wValue;
    // Send status
    NvBootXusbDeviceIssueStatusTRB(DIR_IN);
    return NvBootError_Success;
}

NvBootError NvBootXusbDeviceSetAddress(uint8_t* pSetupData)
{
    uint8_t bAddr;
    EpContext_T *pEpContext;
    //NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    
    bAddr = pSetupData[USB_SETUP_VALUE];
    pEpContext = &EpContext[EP0_IN];
    
    NvBootXusbDeviceSetAddressHw(bAddr);

    pEpContext->DeviceAddr = bAddr;
    // Send status
    NvBootXusbDeviceIssueStatusTRB(DIR_IN);
    pXUSBDeviceContext->DeviceState = ADDRESSED_STATUS_PENDING;
    return NvBootError_Success;
}

NvBootError
NvBootXusbDeviceGetDescriptor(uint8_t* pSetupData, uint16_t* txLength, uint8_t* setupDataBuffer)
{
    
    uint8_t bDescType, bDescIndex= 0;
    uint16_t wLength;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    NvBootError e = NvBootError_Success;

    bDescType = pSetupData[USB_SETUP_DESCRIPTOR];
    wLength = *(uint16_t*)&(pSetupData[USB_SETUP_LENGTH]);

    switch(bDescType)
    {
        case USB_DT_DEVICE:
            DEBUG_PRINT("   |__Device\n\r");
#if XUSB_DEV_SS_SUPPORTED
             if(pXUSBDeviceContext->PortSpeed != XUSB_SUPER_SPEED)
             {
                s_UsbDeviceDescriptor[2] = 0; // bcd USB LSB
                s_UsbDeviceDescriptor[3] = 2; // bcd USB MSB
                s_UsbDeviceDescriptor[7] = 64;
             }
#endif
             NvBootXusbDeviceSetPidRev(&s_UsbDeviceDescriptor[0]);
             *txLength = NV_MIN(wLength, sizeof(s_UsbDeviceDescriptor));
             memcpy((void*)setupDataBuffer, (void*)&s_UsbDeviceDescriptor[0],
                    *txLength);
            break;

        case USB_DT_CONFIG:
                        DEBUG_PRINT("   |__Config\n\r");
#if XUSB_DEV_SS_SUPPORTED
             if(pXUSBDeviceContext->PortSpeed == XUSB_SUPER_SPEED)
             {
                // EP1_IN
                s_UsbConfigDescriptor[22] = 0;
                s_UsbConfigDescriptor[23] = 4;
                // EP1_OUT
                s_UsbConfigDescriptor[29] = 0;
                s_UsbConfigDescriptor[30] = 4;
             } else
#endif
            if(pXUSBDeviceContext->PortSpeed == XUSB_HIGH_SPEED)
            {
                // EP1_IN
                s_UsbConfigDescriptor[22] = 0;
                s_UsbConfigDescriptor[23] = 2;
                // EP1_OUT
                s_UsbConfigDescriptor[29] = 0;
                s_UsbConfigDescriptor[30] = 2;
            }
            else // Apply full speed packet size
            {
                // EP1_IN
                s_UsbConfigDescriptor[22] = 64;
                s_UsbConfigDescriptor[23] = 0;
                // EP1_OUT
                s_UsbConfigDescriptor[29] = 64;
                s_UsbConfigDescriptor[30] = 0;
            }
            *txLength = NV_MIN(wLength, sizeof(s_UsbConfigDescriptor));
            memcpy((void*)setupDataBuffer, (void*)&s_UsbConfigDescriptor[0],
                    *txLength);
            break;

        case USB_DT_STRING:
            DEBUG_PRINT("   |__String\n\r");
            bDescIndex = pSetupData[USB_SETUP_VALUE];
            switch (bDescIndex)
            {
                case USB_MANF_ID: //Manufacture ID
                    DEBUG_PRINT("       |__Manf\n\r");
                    *txLength = NV_MIN(wLength, sizeof(s_UsbManufacturerID));
                    memcpy((void*)setupDataBuffer, 
                           (void*)&s_UsbManufacturerID[0],
                           *txLength);
                    break;
                case USB_PROD_ID:    // Product ID
                    DEBUG_PRINT("       |__Prod\n\r");
                    *txLength = NV_MIN(wLength, sizeof(s_UsbProductID));
                    memcpy((void*)setupDataBuffer, 
                           (void*)&s_UsbProductID[0],
                           *txLength);
                    break;
                case USB_SERIAL_ID:    // Serial Number
                    DEBUG_PRINT("       |__Serial\n\r");
                    *txLength = NV_MIN(wLength, sizeof(s_UsbSerialNumber));
                    memcpy((void*)setupDataBuffer, 
                           (void*)&s_UsbSerialNumber[0],
                           *txLength);
                    break;
                case USB_LANGUAGE_ID:    //Language ID
                DEBUG_PRINT("       |__Lang\n\r");
                //Default case return Language ID
                default: //Language ID
                    DEBUG_PRINT("       |__Unsupported\n\r");
                    *txLength = NV_MIN(wLength, sizeof(s_UsbLanguageID));
                    memcpy((void*)setupDataBuffer, 
                           (void*)&s_UsbLanguageID[0],
                           *txLength);
                    break;
            }
            break;

        case USB_DT_DEVICE_QUALIFIER:
            DEBUG_PRINT("   |__Device Qualifier\n\r");
            *txLength = NV_MIN(wLength, sizeof(s_UsbDeviceQualifier));
            memcpy((void*)setupDataBuffer, 
                   (void*)&s_UsbDeviceQualifier[0],
                   *txLength);
            break;
        case USB_DT_OTHER_SPEED_CONFIG:
            DEBUG_PRINT("   |__Other Speed\n\r");
            if(pXUSBDeviceContext->PortSpeed == XUSB_HIGH_SPEED)
            {
                // Full speed packet size as other speed.
                // EP1_IN
                s_OtherSpeedConfigDesc[22] = 64;
                s_OtherSpeedConfigDesc[23] = 0;
                // EP1_OUT
                s_OtherSpeedConfigDesc[29] = 64;
                s_OtherSpeedConfigDesc[30] = 0;
            }
            else
            {
                // High speed packet size as other speed.
                // EP1_IN
                s_OtherSpeedConfigDesc[22] = 0;
                s_OtherSpeedConfigDesc[23] = 2;
                // EP1_OUT
                s_OtherSpeedConfigDesc[29] = 0;
                s_OtherSpeedConfigDesc[30] = 2;
            }
            *txLength = NV_MIN(wLength, sizeof(s_OtherSpeedConfigDesc));
            memcpy((void*)setupDataBuffer, 
                   (void*)&s_OtherSpeedConfigDesc[0],
                   *txLength);
            break;
#if XUSB_DEV_SS_SUPPORTED
        case USB_DT_BOS:
            *txLength = NV_MIN(wLength, sizeof(s_UsbBOSDescriptor));
            memcpy((void*)setupDataBuffer, 
                   (void*)&s_UsbBOSDescriptor[0],
                   *txLength);
            break;
#endif
        default:
            DEBUG_PRINT("   |__Unsupported\n\r");
            // Stall if any Un supported request comes
             NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
            break;
    }

    return e;
}

NvBootError
NvBootXusbDeviceIssueNormalTRB(NvU32 Buffer, NvU32 Bytes, NvU32 Direction)
{
    NormalTRB_T NormalTRB;
    NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    Endpoint_T EpIndex;

    NvBootUtilMemset((void*)&NormalTRB, 0, sizeof(NormalTRB_T));
    e = NvBootXusbDeviceCreateNormalTRB(&NormalTRB, Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;
    EpIndex = (Direction == DIR_IN) ? EP1_IN: EP1_OUT;

    e = NvBootXusbDeviceQueueTRB(EpIndex, &NormalTRB, 1);
    if(e != NvBootError_Success)
        return e;

    pXUSBDeviceContext->WaitForEvent = NORMAL_TRB;
    return e;
}

NvBootError
NvBootXusbDeviceIssueDataTRB(NvU32 Buffer, NvU32 Bytes, NvU32 Direction)
{
    DataTRB_T DataTRB;
    NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // // Need to check if empty other wise don't issue. Will result in Seq Num Error
    if(pXUSBDeviceContext->CntrlEpEnqueuePtr != pXUSBDeviceContext->CntrlEpDequeuePtr)
    {
        return NvBootError_Success;
    }

    NvBootUtilMemset((void*)&DataTRB, 0, sizeof(DataTRB_T));
    e = NvBootXusbDeviceCreateDataTRB(&DataTRB, Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;
    // Note EP0_IN is bi-directional.
    e = NvBootXusbDeviceQueueTRB(EP0_IN, (NormalTRB_T*)&DataTRB, 1);
    if(e != NvBootError_Success)
        return e;

    pXUSBDeviceContext->WaitForEvent = DATA_STAGE_TRB;
    return e;
}
NvBootError NvBootXusbDeviceIssueStatusTRB(NvU32 Direction)
{
    NvBootError e;
    StatusTRB_T StatusTRB;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    if(pXUSBDeviceContext->CntrlEpEnqueuePtr != pXUSBDeviceContext->CntrlEpDequeuePtr && Direction == DIR_IN)
    {
        return NvBootError_Success;
    }
    NvBootUtilMemset((void*)&StatusTRB, 0, sizeof(StatusTRB_T));
    e = NvBootXusbDeviceCreateStatusTRB(&StatusTRB, Direction);
    if(e != NvBootError_Success)
        return e;
    // Note EP0_IN is bi-directional.
    
    e = NvBootXusbDeviceQueueTRB(EP0_IN, (NormalTRB_T*)&StatusTRB, 1);
    
    pXUSBDeviceContext->WaitForEvent = STATUS_STAGE_TRB;
    return e;
}

NvBootError NvBootXusbDeviceHandleSetupPacket(uint8_t* pSetupData)
{
    uint16_t wLength, txLength = 0;
    /* Wait for event to be posted */
    NvBootError e;
    Endpoint_T EpIndex;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    //GetStatus() Request to an Interface is always 0
    uint8_t InterfaceStatus[2] = {0, 0};

    wLength = *(uint16_t*)&pSetupData[USB_SETUP_LENGTH];

    // Unhalt Ep0
    e = NvBootXusbDeviceHaltEp(EP0_IN, 0);
    if(e != NvBootError_Success) 
        return e;

    switch(pSetupData[USB_SETUP_REQUEST_TYPE])
    {
        case HOST2DEV_DEVICE:
            // Process the Host -> device Device descriptor
            switch(pSetupData[USB_SETUP_REQUEST])
            {
                case SET_CONFIGURATION:
                    DEBUG_PRINT("|__Set Config\n\r");
                    e= NvBootXusbDeviceSetConfiguration(pSetupData);
                    if(e!=NvBootError_Success)
                        return e;
                break;
                case SET_ADDRESS:
                    DEBUG_PRINT("|__Set Address\n\r");
                    e= NvBootXusbDeviceSetAddress(pSetupData);
                    if(e!=NvBootError_Success)
                        return e;
                break;
                default:
                    DEBUG_PRINT("|__Unsupported Request\n\r");
                    // Stall if any Un supported request comes
                    NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;

        case HOST2DEV_INTERFACE:
            // Start the endpoint for zero packet acknowledgment
            // Store the interface number.
            NvBootXusbDeviceSetInterface(pSetupData);
            break;

        case DEV2HOST_DEVICE:
            switch(pSetupData[USB_SETUP_REQUEST])
            {
                case GET_STATUS:
                    DEBUG_PRINT("|__Get Status\n\r");
                     txLength = NV_MIN(wLength, sizeof(s_UsbDeviceStatus));
                     memcpy((void*)SetupDataBuffer, (void*)&s_UsbDeviceStatus[0],
                            txLength);
                    break;
                case GET_CONFIGURATION:
                    DEBUG_PRINT("|__Get Config\n\r");
                    txLength = NV_MIN(wLength, sizeof(pXUSBDeviceContext->ConfigurationNum));
                    memcpy((void*)SetupDataBuffer, &pXUSBDeviceContext->ConfigurationNum,
                            txLength);
                    break;
                case GET_DESCRIPTOR:
                    DEBUG_PRINT("|__Get Descriptor\n\r");
                    DEBUG_PRINT("   |__Length ");
                    DEBUG_PRINT(ITOA(wLength));
                    // Get Descriptor Request
                    // Enact Stall protocol on invalid requests.
                    e =  NvBootXusbDeviceGetDescriptor(pSetupData,
                                                       &txLength,
                                                       SetupDataBuffer);
                    
                    if(e != NvBootError_Success)
                        return e;
                    break;
                default:
                    DEBUG_PRINT("|__Unsupported D2H_D request\n\r");
                    // Stall if any Un supported request comes
                    NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;

        case DEV2HOST_INTERFACE:
            switch (pSetupData[USB_SETUP_REQUEST])
            {
                case GET_STATUS: // Get Status/Interface Request
                    DEBUG_PRINT("|__Get Status I/F\n\r");
                    // Just sending 0s.
                    txLength = NV_MIN(wLength, sizeof(InterfaceStatus));
                    memcpy((void*)SetupDataBuffer, &InterfaceStatus[0],
                            txLength);
                    break;
                case GET_INTERFACE: // Get Interface Request
                    // Just sending 0s.
                    // NOT USUALLY Supported so..lets decide on best course of action.
                    DEBUG_PRINT("|__Get Interface D2H_I/F\n\r");
                    txLength = NV_MIN(wLength, sizeof(pXUSBDeviceContext->InterfaceNum));
                    memcpy((void*)SetupDataBuffer, &pXUSBDeviceContext->InterfaceNum,
                            txLength);
                    break;
                default:
                    // Stall if any unsupported request comes
                    NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;

            // Stall here, as we don't support endpoint requests here
        case DEV2HOST_ENDPOINT:
            switch (pSetupData[USB_SETUP_REQUEST])
            {
                case GET_STATUS: // Get Status/Interface Request
                    DEBUG_PRINT("|__Get Status D2H_Ep\n\r");

                    EpIndex = 2*(pSetupData[USB_SETUP_INDEX]&0xF);
                    EpIndex += (pSetupData[USB_SETUP_INDEX]&0x80)?1:0; 
            // uint8_t EndpointStatus[2] = {0, 0};
            txLength = NV_MIN(wLength, 2);
                    e = NvBootXusbDeviceEPGetStatus(EpIndex,
                                                    &txLength,
                                                    SetupDataBuffer);
                    if(e != NvBootError_Success)
                        return e;
                    break;
                default:
                    DEBUG_PRINT("|__Unsupported D2H_Ep\n\r");
                    NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;

        case HOST2DEV_ENDPOINT:
            switch (pSetupData[USB_SETUP_REQUEST])
            {
                case SET_FEATURE:
                    DEBUG_PRINT("|__Set Feature H2D_Ep\n\r");
                    switch (pSetupData[USB_SETUP_VALUE])
                    {
                        case ENDPOINT_HALT:

                            EpIndex = 2*(pSetupData[USB_SETUP_INDEX]&0xF);
                            EpIndex += (pSetupData[USB_SETUP_INDEX]&0x80)?1:0; 
                            NvBootXusbDeviceStallEndpoint(EpIndex, NV_TRUE);
                            // Send status
                            NvBootXusbDeviceIssueStatusTRB(DIR_IN);
                            break;
                        default:
                            NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                            break;
                    }
                    break;
                case CLEAR_FEATURE:
                    DEBUG_PRINT("|__Clear Feature H2D_Ep\n\r");
                    switch (pSetupData[USB_SETUP_VALUE])
                    {
                        case ENDPOINT_HALT:
                            // Get once the EP status, to find wether Txfer is success or not
                            EpIndex = 2*(pSetupData[USB_SETUP_INDEX]&0xF);
                            EpIndex += (pSetupData[USB_SETUP_INDEX]&0x80)?1:0; 
                            NvBootXusbDeviceStallEndpoint(EpIndex, NV_FALSE);
                            // Send status
                            NvBootXusbDeviceIssueStatusTRB(DIR_IN);
                            break;
                        default:
                            NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                            break;
                    }
                    break;
                default:
                    DEBUG_PRINT("|__Unsupported H2D_Ep\n\r");
                    // Stall if any unsupported request comes
                   NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;
        default:
            DEBUG_PRINT("|__Unsupported Request\n\r");
            // Stall if any Un supported request comes
            NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
            break;
    }
    if(txLength){
        // Compensate buffer for xusb device view of sysram
        e = NvBootXusbDeviceIssueDataTRB((NvU32)(SetupDataBuffer),
                                               txLength,
                                               DIR_IN);
        if(e != NvBootError_Success)
            return e;
    }

    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceInitEndpoint(Endpoint_T EpIndex)
{
    NvBootError e = NvBootError_Success;

    NvBootXusbDeviceInitializeTransferRing(EpIndex);

    if(EpIndex == EP0_IN || EpIndex == EP0_OUT)
    {
        
        NvBooXusbDeviceInitEpContext(EP0_IN);
    }
    else if(EpIndex == EP1_IN || EpIndex == EP1_OUT)
    {
        NvBooXusbDeviceInitEpContext(EpIndex);
        e = NvBootXusbDeviceReloadContext(EpIndex);
        if(e!=NvBootError_Success)
            return e;
        
        // Make sure ep is not paused or halted.
        NvBootXusbDeviceUnpauseEp(EpIndex);
        NvBootXusbDeviceHaltEp(EpIndex, 0);
    }
    else
        e = NvBootError_InvalidParameter;
    return e;
}

NvBootError NvBootXusbDeviceHandleTransferEvent(TransferEventTRB_T *pTxEventTRB)
{
    /* Wait for event to be posted */
    NvBootError e = NvBootError_Success;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    LinkTRB_T *pLinkTRB;
    DataTRB_T *pNextTRB;

    // Make sure update local copy for dequeue ptr
    if(pTxEventTRB->EndptId == EP0_IN)
    {
        pXUSBDeviceContext->CntrlEpDequeuePtr+=sizeof(TransferEventTRB_T);
        pNextTRB = (DataTRB_T*)pXUSBDeviceContext->CntrlEpDequeuePtr;
        // Handle Link TRB
        if(pNextTRB->TRBType == LINK_TRB)
        {
            pLinkTRB = (LinkTRB_T*)pNextTRB;
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo <<4);
        }
        pXUSBDeviceContext->CntrlEpDequeuePtr = (NvU32)pNextTRB;
        
    }
    if(pTxEventTRB->EndptId == EP1_OUT)
    {
        pXUSBDeviceContext->BulkOutDequeuePtr+=sizeof(TransferEventTRB_T);
        pNextTRB = (DataTRB_T*)pXUSBDeviceContext->BulkOutDequeuePtr;
        // Handle Link TRB
        if(pNextTRB->TRBType == LINK_TRB)
        {
            pLinkTRB = (LinkTRB_T*)pNextTRB;
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo <<4);
        }
        pXUSBDeviceContext->BulkOutDequeuePtr = (NvU32)pNextTRB;
        
    }
    if(pTxEventTRB->EndptId == EP1_IN)
    {
        pXUSBDeviceContext->BulkInDequeuePtr+=sizeof(TransferEventTRB_T);
        pNextTRB = (DataTRB_T*)pXUSBDeviceContext->BulkInDequeuePtr;
        // Handle Link TRB
        if(pNextTRB->TRBType == LINK_TRB)
        {
            pLinkTRB = (LinkTRB_T*)pNextTRB;
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo <<4);
        }

        pXUSBDeviceContext->BulkInDequeuePtr = (NvU32)pNextTRB;
        
    }
    // Check for errors.
    if((pTxEventTRB->CompCode == SUCCESS_ERR_CODE) || (pTxEventTRB->CompCode == SHORT_PKT_ERR_CODE))
    {
        
        if(pTxEventTRB->EndptId == EP0_IN)
        {
            if(pXUSBDeviceContext->WaitForEvent == DATA_STAGE_TRB)
            {
                // Send status
                NvBootXusbDeviceIssueStatusTRB(DIR_OUT);
            }
            else if(pXUSBDeviceContext->WaitForEvent == STATUS_STAGE_TRB)
            {
                if(pXUSBDeviceContext->DeviceState == ADDRESSED_STATUS_PENDING)
                        pXUSBDeviceContext->DeviceState = ADDRESSED;
                if(pXUSBDeviceContext->DeviceState == CONFIGURED_STATUS_PENDING)
                        pXUSBDeviceContext->DeviceState = CONFIGURED;
            }
        }
        if(pTxEventTRB->EndptId == EP1_IN)
        {
            // TRB Tx Len will be 0 or remaining bytes.
            pXUSBDeviceContext->BytesTxfred -= pTxEventTRB->TRBTxLen;
            pXUSBDeviceContext->TxCount--;
            // For IN, we should not have remaining bytes. Flag error
            if(pTxEventTRB->TRBTxLen)
                e = NvBootError_DataUnderflow;
        }
        if(pTxEventTRB->EndptId == EP1_OUT)
        {
            // TRB Tx Len will be 0 or remaining bytes for short packet.
            pXUSBDeviceContext->BytesTxfred -= pTxEventTRB->TRBTxLen;
            pXUSBDeviceContext->TxCount--;
            // Short packet is not necessary an error because we prime for 4K bytes.
        }
        // This should be zero except in the case of a short packet.
    }
    else if(pTxEventTRB->CompCode == CTRL_DIR_ERR_CODE)
    {
        e = NvBootError_XusbControlDirError;
    }
    else if(pTxEventTRB->CompCode == CTRL_SEQ_NUM_ERR_CODE)
    {
        
        // This could mean 2 things.
        // 1.   The seq number in the data/status stage did not match the 
        //      setup event seq.
        // 2. A new setup packet was received when sending data/status packet.

        // We have a setup packet to process
        // Setuppacketindex always points to next slot. pop out last setup packet.
        // SetupPacketIndex = pXUSBDeviceContext->SetupPacketIndex-1;
        // pSetupData = &SetupPacketQueue[SetupPacketIndex*8];
        // e = NvBootXusbDeviceHandleSetupPacket(pSetupData);
        
        return NvBootError_XusbControlSeqNumError;
    }
    else
        e = NvBootError_TxferFailed;
    return e;
}
NvBootError NvBooXusbDeviceInitEpContext(Endpoint_T EpIndex)
{
    EpContext_T *pEpContext;
    XUSBDeviceContext_T *pXUSBDeviceContext;
    LinkTRB_T *pLinkTRB;

    pXUSBDeviceContext = &XUSBDeviceContext;

    if(EpIndex > EP1_IN)
        return NvBootError_InvalidParameter;
    if(EpIndex == EP0_OUT)
        EpIndex = EP0_IN; // One and the same

    /************ SET UP EP CONTEXT *************/
    pEpContext = &EpContext[EpIndex];

    // Control Endpoint 0.
    if(EpIndex == EP0_IN)
    {
        NvBootUtilMemset((void*)pEpContext, 0, sizeof(EpContext_T));
        // Set Endpoint State to running.

        pEpContext->EpState = EP_RUNNING;
        // Set error count to 3
        pEpContext->CErr = 3;
        // Set Burst size 0
        pEpContext->MaxBurstSize = 0;
        // Set Packet size as 64 bytes. USB 2.0
#if XUSB_DEV_SS_SUPPORTED
#define EP0_MAX_PACKET_SIZE 512
#else
#define EP0_MAX_PACKET_SIZE 64
#endif
        pEpContext->MaxPacketSize = EP0_MAX_PACKET_SIZE;
        // Set CCS for controller to 1. Cycle bit should be set to 1.
        pEpContext->DCS = 1;
        // CErr Count
        pEpContext->CEC = 0x3;
        // Initialize Producer Cycle State to 1.
        pXUSBDeviceContext->CntrlPCS = 1;
        // SW copy of Dequeue pointer for control endpoint.
        pXUSBDeviceContext->CntrlEpDequeuePtr =
        pXUSBDeviceContext->CntrlEpEnqueuePtr = 
        (NvU32)&TxRingEp0[0];
        
        // EP specific Context
        // Set endpoint type to Control.
#define EP_TYPE_CNTRL 4
        // Average TRB length. Setup data always 8 bytes.
        pEpContext->AvgTRBLen = 8;
        pEpContext->EpType = EP_TYPE_CNTRL;
        // Set the dequeue pointer for the consumer (i.e. XUSB Controller)
        pEpContext->TRDequeuePtrLo = (pXUSBDeviceContext->CntrlEpDequeuePtr) >> 4;
        pEpContext->TRDequeuePtrHi = 0;

        // Setup Link TRB. Last TRB of ring.
        pLinkTRB = (LinkTRB_T*)&TxRingEp0[NUM_TRB_TRANSFER_RING-1];
        pLinkTRB->TC = 1;
        pLinkTRB->RingSegPtrLo = ((NvU32)&TxRingEp0[0])>>4;
        pLinkTRB->RingSegPtrHi = 0;
        pLinkTRB->TRBType = LINK_TRB;
    }
    else
    {
        if(EpIndex == EP1_OUT)
        {
            NvBootUtilMemset((void*)pEpContext, 0, sizeof(EpContext_T));
            pEpContext->EpState = EP_RUNNING;
            // Set error count to 3
            pEpContext->CErr = 3;
            // Set Burst size 0
            pEpContext->MaxBurstSize = 0;
            // Set Packet size as 64 bytes. USB 2.0
            // Set CCS for controller to 1. Cycle bit should be set to 1.
            pEpContext->DCS = 1;
            // Set CCS for controller to 1. Cycle bit should be set to 1.
            pEpContext->DCS = 1;
            // CErr Count
            pEpContext->CEC = 0x3;

            // Initialize Producer Cycle State to 1.
            pXUSBDeviceContext->BulkOutPCS = 1;

            // SW copy of Dequeue pointer for control endpoint.
            pXUSBDeviceContext->BulkOutDequeuePtr =
            pXUSBDeviceContext->BulkOutEnqueuePtr = (NvU32)&TxRingEp1Out[0];

            // EP specific Context
            // Set endpoint type to Bulk.
#define EP_TYPE_BULK_OUT 2
            // Average TRB length
#if XUSB_DEV_SS_SUPPORTED
            if(pXUSBDeviceContext->PortSpeed == XUSB_SUPER_SPEED)
            {
                pEpContext->AvgTRBLen = 1024;
                pEpContext->MaxPacketSize = 1024;
            }
            else // All other cases, use HS.
#endif
            if(pXUSBDeviceContext->PortSpeed == XUSB_HIGH_SPEED)
            {
                pEpContext->AvgTRBLen = 512;
                pEpContext->MaxPacketSize = 512;
            }
            else
            {
                pEpContext->AvgTRBLen = 512;
                pEpContext->MaxPacketSize = 64;
            }
            pEpContext->EpType = EP_TYPE_BULK_OUT;
            pEpContext->TRDequeuePtrLo = (pXUSBDeviceContext->BulkOutDequeuePtr)>> 4;
            pEpContext->TRDequeuePtrHi = 0;

            // Setup Link TRB. Last TRB of ring. 
            pLinkTRB = (LinkTRB_T*)&TxRingEp1Out[NUM_TRB_TRANSFER_RING-1];
            pLinkTRB->TC = 1;
            pLinkTRB->RingSegPtrLo = ((NvU32)&TxRingEp1Out[0])>>4;
            pLinkTRB->RingSegPtrHi = 0;
            pLinkTRB->TRBType = LINK_TRB;
        }
        else // EP1_IN
        {
            NvBootUtilMemset((void*)pEpContext, 0, sizeof(EpContext_T));
            pEpContext->EpState = EP_RUNNING;
            // Set error count to 3
            pEpContext->CErr = 3;
            // Set Burst size 0
            pEpContext->MaxBurstSize = 0;
            // Set Packet size as 64 bytes. USB 2.0
            // Set CCS for controller to 1. Cycle bit should be set to 1.
            pEpContext->DCS = 1;
            // Set CCS for controller to 1. Cycle bit should be set to 1.
            pEpContext->DCS = 1;
            // CErr Count
            pEpContext->CEC = 0x3;
            
            // Initialize Producer Cycle State to 1.
            pXUSBDeviceContext->BulkInPCS  = 1;

            // SW copy of Dequeue pointer for control endpoint.
            pXUSBDeviceContext->BulkInDequeuePtr =
            pXUSBDeviceContext->BulkInEnqueuePtr = (NvU32)&TxRingEp1In[0];

            // EP specific Context
            // Set endpoint type to Bulk.
#define EP_TYPE_BULK_IN 6
            pEpContext->EpType = EP_TYPE_BULK_IN;
            pEpContext->TRDequeuePtrLo = (pXUSBDeviceContext->BulkInDequeuePtr)>>4;
            pEpContext->TRDequeuePtrHi = 0;
#if XUSB_DEV_SS_SUPPORTED
            if(pXUSBDeviceContext->PortSpeed == XUSB_SUPER_SPEED)
            {
                pEpContext->AvgTRBLen = 1024;
                pEpContext->MaxPacketSize = 1024;
            }
            else // All other cases, use HS.
#endif
            if(pXUSBDeviceContext->PortSpeed == XUSB_HIGH_SPEED)
            {
                pEpContext->AvgTRBLen = 512;
                pEpContext->MaxPacketSize = 512;
            }
            else
            {
                pEpContext->AvgTRBLen = 512;
                pEpContext->MaxPacketSize = 64;
            }
            // Setup Link TRB. Last TRB of ring. 
            pLinkTRB = (LinkTRB_T*)&TxRingEp1In[NUM_TRB_TRANSFER_RING-1];
            pLinkTRB->TC = 1;
            pLinkTRB->RingSegPtrLo = ((NvU32)&TxRingEp1In[0])>>4;
            pLinkTRB->RingSegPtrHi = 0;
            pLinkTRB->TRBType = LINK_TRB;
        }
    }
    return NvBootError_Success;
}

NvBootError NvBootXusbDeviceQueueTRB(Endpoint_T EpIndex, NormalTRB_T* pTRB, NvU32 RingDoorBell)
{
    LinkTRB_T *pLinkTRB;
    DataTRB_T *pNextTRB;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    NvBootError e =  NvBootError_Success;
    
    // If Control EP
    if(EpIndex == EP0_IN)
    {
        NvBootUtilMemcpy((void*)pXUSBDeviceContext->CntrlEpEnqueuePtr,
               (void*)pTRB,
               sizeof(NormalTRB_T));

        pNextTRB = (DataTRB_T*)pXUSBDeviceContext->CntrlEpEnqueuePtr;
        pNextTRB++;
        // Handle Link TRB
        if(pNextTRB->TRBType == LINK_TRB)
        {
            pLinkTRB = (LinkTRB_T*)pNextTRB;
            // 
            pLinkTRB->C = pXUSBDeviceContext->CntrlPCS;
            pLinkTRB->TC = 1;
            // Toggle cycle bit
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo <<4);
            pXUSBDeviceContext->CntrlPCS ^= 1;
        }
        pXUSBDeviceContext->CntrlEpEnqueuePtr = (NvU32)pNextTRB;

    }
    // Bulk Endpoint
    else if(EpIndex == EP1_OUT)
    {
        NvBootUtilMemcpy((void*)pXUSBDeviceContext->BulkOutEnqueuePtr,
               (void*)pTRB,
               sizeof(NormalTRB_T));
        pNextTRB = (DataTRB_T*)pXUSBDeviceContext->BulkOutEnqueuePtr;
        pNextTRB++;
        // Handle Link TRB
        if(pNextTRB->TRBType == LINK_TRB)
        {
            pLinkTRB = (LinkTRB_T*)pNextTRB;
            pLinkTRB->C = pXUSBDeviceContext->BulkOutPCS;
            pLinkTRB->TC = 1;
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo<<4);
            pXUSBDeviceContext->BulkOutPCS ^= 1;
            
        }
        pXUSBDeviceContext->BulkOutEnqueuePtr = (NvU32)pNextTRB;
    }
        // Bulk Endpoint
    else if(EpIndex == EP1_IN)
    {
        NvBootUtilMemcpy((void*)pXUSBDeviceContext->BulkInEnqueuePtr,
               (void*)pTRB,
               sizeof(NormalTRB_T));
        pNextTRB = (DataTRB_T*)pXUSBDeviceContext->BulkInEnqueuePtr;
        pNextTRB++;
        // Handle Link TRB
        if(pNextTRB->TRBType == LINK_TRB)
        {
            pLinkTRB = (LinkTRB_T*)pNextTRB;
            pLinkTRB->C = pXUSBDeviceContext->BulkInPCS;
            pLinkTRB->TC = 1;
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo<<4);
            pXUSBDeviceContext->BulkInPCS ^= 1;
        }
        pXUSBDeviceContext->BulkInEnqueuePtr = (NvU32)pNextTRB;
    }
    else
        e = NvBootError_IllegalParameter;

    // Ring Doorbell
    if(RingDoorBell)
    {
        // SeqNum is used only for control endpoint
        NvBootXusbDeviceRingDoor(EpIndex, pXUSBDeviceContext->CntrlSeqNum);
    }
    return e;

}
    

NvBootError NvBootXusbDeviceCreateNormalTRB(NormalTRB_T *pNormalTRB, NvU32 Buffer, 
                                          NvU32 Bytes, NvU32 Dir)
{
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    
    pNormalTRB->DataBufPtrLo = Buffer;
    pNormalTRB->DataBufPtrHi = 0;
    pNormalTRB->TRBTxLen = Bytes;
    // Number of packets remaining.
    // Bootrom will always queue only 1 TRB at a time.
    pNormalTRB->TDSize = 0;
    if(Dir == DIR_IN)
        pNormalTRB->C = pXUSBDeviceContext->BulkInPCS;
    else
        pNormalTRB->C = pXUSBDeviceContext->BulkOutPCS;

    pNormalTRB->ENT = 0;
    // Make sure to interrupt on short packet i.e generate event.
    pNormalTRB->ISP = 1;
    // and on Completion.
    pNormalTRB->IOC = 1;

    pNormalTRB->TRBType = NORMAL_TRB;
    return NvBootError_Success;
}

NvBootError NvBootXusbDeviceCreateDataTRB(DataTRB_T *pDataTRB, NvU32 Buffer, 
                                          NvU32 Bytes, NvU32 Dir)
{
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    
    pDataTRB->DataBufPtrLo = Buffer;
    pDataTRB->DataBufPtrHi = 0;
    pDataTRB->TRBTxLen = Bytes;
    // Bootrom will always queue only 1 TRB at a time.
    pDataTRB->TDSize = 0;
    pDataTRB->C = pXUSBDeviceContext->CntrlPCS;
    pDataTRB->ENT = 0;
    // Make sure to interrupt on short packet i.e generate event.
    pDataTRB->ISP = 1;
    // and on Completion.
    pDataTRB->IOC = 1;

    pDataTRB->TRBType = DATA_STAGE_TRB;
    pDataTRB->Dir = Dir;
    return NvBootError_Success;
}

NvBootError NvBootXusbDeviceCreateStatusTRB(StatusTRB_T *pStatusTRB, NvU32 Dir)
{
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    // Event gen on Completion.
    pStatusTRB->C = pXUSBDeviceContext->CntrlPCS;
    pStatusTRB->IOC = 1;
    pStatusTRB->TRBType = STATUS_STAGE_TRB;
    pStatusTRB->Dir =Dir;
    return NvBootError_Success;
}


NvBootError NvBootXusbDeviceInit(void)
{
    NvBootError e;
    NvBootClocksOscFreq OscFreq;
    OscFreq = NvBootClocksGetOscFreq();

    UART_INIT(OscFreq, NV_TRUE);

    // Set pad ownership to XUSB and functionality to device to mode.
    NvBootXusbDeviceSetPadOwnership();

    // Setup  PADs.
    e = NvBootXusbDevicePadSetup();
    if(e != NvBootError_Success)
        return e;

    // BUS_MASTER, MEMORY_SPACE, IO_SPACE ENABLE
    NvBootXusbDeviceBusInit();
    
    // Initialize Event ring
    NvBootXusbDeviceInitializeEventRing((NvU32)EventRing, NUM_TRB_EVENT_RING);
    NvBootXusbDeviceInitializeTransferRing(EP0_IN);

    
    // Initialize EP0
    e = NvBootXusbDeviceInitEndpoint(EP0_IN);
    if( e != NvBootError_Success)
        return e;
    
    NvBootXusbDeviceSetEndpointContext((NvU32)EpContext);
    
    NvBootXusbDeviceInterruptSetup();
    return e;
}

NvBootError NvBootXusbDeviceEnumerate(uint8_t* Buffer)
{
    NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    static NvU32 doOnce = 0;

      // Buffer is unused. Needed earlier because of SNPS driver
      (void)Buffer;

    if(!doOnce)
    {
        // Vbus override and device enable
        NvBootXusbDeviceEnumerateHw();
        pXUSBDeviceContext->DeviceState = DEFAULT;
        pXUSBDeviceContext->WaitForEvent = SETUP_EVENT_TRB;
        doOnce++;
    }
    while(pXUSBDeviceContext->DeviceState != CONFIGURED)
    // while(1)
    {
        e = NvBootXusbDevicePollForEvent(0xFFFFFFFF);
        if(e != NvBootError_Success)
            return e;
    }
    return NvBootError_Success;
}

NvBootError NvBootXusbDevicePadSetup(void)
{
    NvBootError e = NvBootError_Success;
    // Configure the USB phy stabilization delay setting.
    NvBootClocksOscFreq OscFreq;
    OscFreq = NvBootClocksGetOscFreq();

    NvBootXusbDeviceSetPadOwnership();
    NvBootXusbDeviceSetupStaticParamsPad();    
    NvBootXusbDeviceRemovePowerDownPad();
    NvBootXusbDevicePerformTracking(OscFreq);

    // No way to tell how long to wait. Just wait for a bit.
    NvBootUtilWaitUS(30);
    return e;
}

NvBootError NvBootXusbDeviceReceive(uint8_t* Buffer, NvU32 Bytes,  NvU32 *pBytesReceived)
{
    NvBootError e = NvBootError_NotInitialized;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    
    // We can only receive a max of 64K bytes in 1 transaction.
    Bytes = (Bytes>0x10000)?0x10000:Bytes;

    // Sanitize Buffer and Length to fit in IRAM Region
    e = NvBootValidateAddress(IramRange, (uint32_t)Buffer, Bytes);
    if(e!=NvBootError_Success)
        return e;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_OUT;
    
    e = NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;

    pXUSBDeviceContext->TxCount++;
    while(pXUSBDeviceContext->TxCount)
    {
         e = NvBootXusbDevicePollForEvent(0xFFFFFFFF);
         if(e != NvBootError_Success)
            break;
    }
    *pBytesReceived = pXUSBDeviceContext->BytesTxfred;
    return e;
}

NvBootError NvBootXusbDeviceTransmit(uint8_t* Buffer, NvU32 Bytes, NvU32 *pBytesTransmitted)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_IN;

    e = NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;
        pXUSBDeviceContext->TxCount++; 

    while(pXUSBDeviceContext->TxCount)
    {
         e = NvBootXusbDevicePollForEvent(0xFFFFFFFF);
         if(e != NvBootError_Success)
            break;
    }
    *pBytesTransmitted = pXUSBDeviceContext->BytesTxfred;
    return e;
}

NvBootError NvBootXusbDeviceReceiveStart(uint8_t* Buffer, NvU32 Bytes)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_OUT;

    e = NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;
    pXUSBDeviceContext->TxCount++;
    return e;
}
NvBootError NvBootXusbDeviceReceivePoll(NvU32 *pBytesReceived, NvU32 TimeoutUs, uint8_t* OptionalBuffer)
{
    NvBootError e = NvBootError_Success;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // Not needed in XUSB driver.
    (void)OptionalBuffer;

    while(pXUSBDeviceContext->TxCount)
    {
         e = NvBootXusbDevicePollForEvent(TimeoutUs);
         if(e != NvBootError_Success)
            break;
    }
    *pBytesReceived = pXUSBDeviceContext->BytesTxfred;
    return e;
}

NvBootError NvBootXusbDeviceTransmitStart(uint8_t* Buffer, NvU32 Bytes)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_IN;
    // Handle difference in sysram view between host and device.
    e = NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;
        pXUSBDeviceContext->TxCount++; 
    
    return e;
}
NvBootError NvBootXusbDeviceTransmitPoll(NvU32 *pBytesTransferred, NvU32 TimeoutUs, uint8_t* OptionalBuffer)
{
    NvBootError e = NvBootError_Success;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // Use OptionalBuffer to get rid of warning.
    (void)OptionalBuffer;

    while(pXUSBDeviceContext->TxCount)
    {
         e = NvBootXusbDevicePollForEvent(TimeoutUs);
         if(e != NvBootError_Success)
            break;
         // If we get a reset or set config 0, we need to indicate to higher level software so 
         // that transfer can be scheduled again.
         if(pXUSBDeviceContext->DeviceState != CONFIGURED)
         {
            // Being suspended is ok. We should get un-suspended when host initiates a transfer.
            // Removing cable will also create a suspend condition.
            // Ideally this should be a disconnect condition but as we set vbus override, this
            // is not the case. In this case, we will just wait here to be reconnected at which 
            // point we should receive a reset.
            if(pXUSBDeviceContext->DeviceState != SUSPENDED)
            {
                // This could be a reset or addressed pending state. Return same error.
                // In any case, we need to go back into enumerate loop and re-schedule the transfer.
                e = NvBootError_XusbReset;
                break;
            }
         }
    }
    *pBytesTransferred = pXUSBDeviceContext->BytesTxfred;
    return e;
}

void NvBootXusbDeviceGetClockTable(void **XusbClockTable, ClockTableType *Id)
{
    // Only 38.4M OscFreq
    /*
    // First modify tracking clock as per oscillator
    NvBootClocksOscFreq OscFreq;
    OscFreq = NvBootClocksGetOscFreq();

    // Point to correct table as per osc freq. We don't really care about this for fpga.
    switch(OscFreq)
    {
        case NvBootClocksOscFreq_12:
            XusbClockTables[OSC_TABLE] = s_XUSBTrackingClockFrequency_12;
        break;
        default:
            XusbClockTables[OSC_TABLE] = s_XUSBTrackingClockFrequency_38_4;
    }
    */
    // Passing in list of tables.
    // Don't need fpga specific tables anymore. Use for reference for future if needed.
    // if(NvBootIsPlatformFpga())
    // {
        // *XusbClockTable= XusbFpgaClockTables;
    // }
    // else
    // {
        // *XusbClockTable= XusbClockTables;
    // }
    *XusbClockTable= (void*)XusbClockTables;
    // Indicate to clocks engine multi-tables.
    *Id = TYPE_MULTI_TABLE;
}
NvBootError NvBootXusbHandleError(void)
{
    // No specific device level handling for XUSB.
    return NvBootError_Success;
}

