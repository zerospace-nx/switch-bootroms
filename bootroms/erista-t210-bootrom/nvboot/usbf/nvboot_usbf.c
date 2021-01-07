/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_usbf_hw.h"
#include "nvcommon.h"
#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "arusb.h"
#include "arclk_rst.h"
#include "arapbpm.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_usbf_int.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_usbcharging_int.h"
#include "nvboot_wdt_int.h"

//---------------------Global Variables Decleration----------------------------

/**
 * USB data store consists of Queue head and device transfer descriptors.
 * This structure must be aligned to 2048 bytes and must be in IRAM.
 */
NV_ALIGN(2048) static NvBootUsbDescriptorData s_UsbDescriptorBuf;
static NvBootUsbDescriptorData *s_pUsbDescriptorBuf = NULL;

/**
 * Usbf context record and its pointer used locally by the driver
 * Note that the pointer is NULL until NvBootUsbfInit has been called.
 */
extern NvBootUsbfContext s_UsbfContext;
extern NvBootUsbfContext *s_pUsbfCtxt;
extern NvBootInfoTable BootInfoTable;

//---------------------Private Variables Decleration---------------------------

/**
 * USB Device Descriptor: 12 bytes as per the USB2.0 Specification
 * Stores the Device descriptor data must be word aligned
 */
NV_ALIGN(4) static NvU8 s_UsbDeviceDescriptor[USB_DEV_DESCRIPTOR_SIZE] =
{
    USB_DEV_DESCRIPTOR_SIZE,   // bLength - Size of this descriptor in bytes
    0x01,   // bDescriptorType - Device Descriptor Type
    0x00,   // bcd USB (LSB) - USB Spec. Release number
    0x02,   // bcd USB (MSB) - USB Spec. Release number (2.0)
    0x00,   // bDeviceClass - Class is specified in the interface descriptor.
    0x00,   // bDeviceSubClass - SubClass is specified in the interface descriptor.
    0x00,   // bDeviceProtocol - Protocol is specified in the interface descriptor.
    0x40,   // bMaxPacketSize0 - Maximum packet size for EP0
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

/**
 * USB Device Configuration Descriptors: 
 * 32 bytes as per the USB2.0 Specification. This contains 
 * Configuration descriptor, Interface descriptor and endpoint descriptors.
 */
NV_ALIGN(4) static NvU8 s_UsbConfigDescriptor[USB_CONFIG_DESCRIPTOR_SIZE] =
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

    /* Endpoint Descriptor IN EP2 */
    0x07,   // bLength - Size of this descriptor in bytes
    0x05,   // bDescriptorType - ENDPOINT Descriptor Type
    0x81,   // bEndpointAddress - The address of EP on the USB device 
    0x02,   // bmAttributes - Bit 1-0: Transfer Type 10: Bulk, 
    0x40,   // wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    0x00,   // wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00,   // bIntervel - Interval for polling EP, for Interrupt and Isochronous 

    /** Endpoint Descriptor OUT EP1 */
    0x07,   // bLength - Size of this descriptor in bytes
    0x05,   // bDescriptorType - ENDPOINT Descriptor Type
    0x01,   // bEndpointAddress - The address of EP on the USB device 
    0x02,   // bmAttributes - Bit 1-0: Transfer Type 10: Bulk, 
    0x40,   // wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    0x00,   // wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00    // bIntervel - Interval for polling EP, for Interrupt and Isochronous 
};



static NvU8 s_OtherSpeedConfigDesc[32]= {

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
    BULK_IN,   /// bEndpointAddress - The address of EP on the USB device "Bit 7: Direction(0:OUT, 1: IN),Bit 6-4:Res,Bit 3-0:EP no"
    0x02,   /// bmAttributes - Bit 1-0: Transfer Type 00:Control,01:Isochronous,10: Bulk, 11: Interrupt
    0x40,   /// wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    0x00,   /// wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00,   /// bIntervel - Interval for polling EP, applicable for Interrupt and Isochronous data transfer only

    /** Endpoint Descriptor OUT EP1 */
    0x07,   /// bLength - Size of this descriptor in bytes
    0x05,   /// bDescriptorType - ENDPOINT Descriptor Type
    BULK_OUT,   /// bEndpointAddress - The address of EP on the USB device "Bit 7: Direction(0:OUT, 1: IN),Bit 6-4:Res,Bit 3-0:EP no"
    0x02,   /// bmAttributes - Bit 1-0: Transfer Type 00:Control,01:Isochronous,10: Bulk, 11: Interrupt
    0x40,   /// wMaxPacketSize(LSB) - Maximum Packet Size for this EP
    0x00,   /// wMaxPacketSize(MSB) - Maximum Packet Size for this EP
    0x00    /// bIntervel - Interval for polling EP, applicable for Interrupt and Isochronous data transfer only
};


// Stores the Manufactures ID sting descriptor data
NV_ALIGN(4) static NvU8 s_UsbManufacturerID[USB_MANF_STRING_LENGTH] = 
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
NV_ALIGN(4) static NvU8 s_UsbProductID[USB_PRODUCT_STRING_LENGTH] = 
{
    USB_PRODUCT_STRING_LENGTH, // Length of descriptor
    0x03,                      // STRING descriptor type.
    'A', 0x00,
    'P', 0x00,
    'X', 0x00
};

// Stores the Serial Number String descriptor data (Not used for AP15 Bootrom)
NV_ALIGN(4) static NvU8 s_UsbSerialNumber[USB_SERIAL_NUM_LENGTH] =  
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
NV_ALIGN(4) static NvU8 s_UsbLanguageID[USB_LANGUAGE_ID_LENGTH] =
{
    /* Language Id string descriptor */
    USB_LANGUAGE_ID_LENGTH,  // Length of descriptor
    0x03,                    // STRING descriptor type.
    0x09, 0x04               // LANGID Code 0: American English 0x409
};

// Stores the Device Qualifier Desriptor data
NV_ALIGN(4) static NvU8 s_UsbDeviceQualifier[USB_DEV_QUALIFIER_LENGTH] = 
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
NV_ALIGN(4) static NvU8 s_UsbDeviceStatus[USB_DEV_STATUS_LENGTH] = 
{
    USB_DEVICE_SELF_POWERED,
    0,
};

// Mask for ALL end points in the controller
static const NvU32 USB_FLUSH_ALL_EPS = 0xFFFFFFFF;

//---------------------Private Functions Decleration---------------------------

/**
 * Intializes the endpoint in the H/W controller.
 * Configures given endpoint for BULK or CTRL and IN or OUT.
 */
static void 
NvBootUsbfHwInitalizeEndpoints(
    NvBootUsbfContext *pUsbFuncCtxt, 
    NvU32 endPoint);

/*
 * Function to init queue head and endpoints
 */
static NvBootError
NvBootUsbfInitializeQueueHeadAndEp(
    NvBootUsbfContext *pUsbFuncCtxt);

/**
 * Gets the USB controller events generated by the H/W.
 */
static NvU32 NvBootUsbfHwGetEvent(void);

/**
 * Flushes the specified endpoint;
 */
static NvBootError NvBootUsbfHwEndPointFlush(NvU32 endPoint);

/**
 * Gets the Status of specified endpoint.
 */
static NvBootUsbfEpStatus 
NvBootUsbfHwEpGetStatus(
    NvBootUsbfContext *pUsbFuncCtxt, 
    NvU32 endPoint);

/**
 * Initiates the Transferes(In and Out transfers on the BUS) for endpoint 
 */
static NvBootError
NvBootUsbfHwTxfrStart(
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint,
    NvU8 *pDataBuf,
    NvU32 maxTxfrBytes,
    NvBool WaitForTxfrComplete);

/**
 * Clears any pending transfer on an endpoint.
 */
static void 
NvBootUsbfHwTxfrClear (
    NvBootUsbfContext *pUsbFuncCtxt, 
    NvU32 endPoint);


/**
 * Gets the packet size depending on the port speed
 */
static NvU32 
NvBootUsbfHwGetPacketSize(
    NvBootUsbfContext *pUsbFuncCtxt, 
    NvU32 EndPoint);

/**
 * Waits on endpoint till time out for transfer completion.
 */
static NvBootError 
NvBootUsbfHwTxfrWait(
    NvBootUsbfContext *pUsbFuncCtxt, 
    NvU32 EndPoint,
    NvU32 timeoutMs);

/**
 * NvBootUsbfProcessSetupPacket(): Parses the Setup Packet 
 * and does the control  IN transfers to the Host.
 */
static NvBootError NvBootUsbfProcessSetupPacket(NvU8 *pXferBuffer);

/**
 * NvBootUsbfProcessHost2DevDevicePacket(): Process the Host to Device
 * setup packet for device attributes.
 */
static NvBootError 
NvBootUsbfProcessHost2DevDevicePacket(
    NvBool *pIsEndPointStall);

/**
 * NvBootUsbfProcessDeviceDescriptor(): Process the Device to Host
 * setup packet for device Descriptor.
 */
static void 
NvBootUsbfProcessDeviceDescriptor(
    NvBool *pTransmitData, 
    NvU8 **pDataBuffer, 
    NvU32 *pDataSize,
    NvBool *pIsEndPointStall);


/**
 * NvBootConfigureDevDescriptor(): Configures the USB device descriptor based 
 * on the Chip-ID information.
 */
static void NvBootConfigureDevDescriptor(NvU8 *pDevDescriptor);

//---------------------Public Functions Definitions----------------------------
NvBootError NvBootUsbfStartEnumeration(NvU8 *pXferBuffer)
{
    NvBootError BootError = NvBootError_Success;
    NvU32 EpSetupStatus = 0;
    NvU32 UsbfEvents = 0;

    // Validating the parameters 
    NV_ASSERT(pXferBuffer);

    // Address must be 4K bytes aligned
    NV_ASSERT(!(PTR_TO_ADDR(pXferBuffer)&(NVBOOT_USB_BUFFER_ALIGNMENT-1)));

    // Incase of re-enumeration EnumerationDone is TRUE.
    if (!s_pUsbfCtxt->EnumerationDone)
    {
        // Assign the Hardware Q head address
        s_UsbDescriptorBuf.pQueueHead = (NvBootUsbDevQueueHead *)
        (s_pUsbfCtxt->UsbBaseAddr + USB2_QH_USB2D_QH_EP_0_OUT_0);

        // Store the Descriptor Buffer pointer for later use
        s_pUsbDescriptorBuf= &s_UsbDescriptorBuf;

        // Initialize queuehead descriptors and initialize control in/out
        // endpoints
        BootError = NvBootUsbfInitializeQueueHeadAndEp(s_pUsbfCtxt);

        if (BootError != NvBootError_Success)
        {
            return BootError;
        }
    }

    s_pUsbfCtxt->EnumerationDone = NV_FALSE;

    // Start enumeration if cable is connected and HW intialization is success
    while(!s_pUsbfCtxt->EnumerationDone)
    {
        // Get the USB events from the H/W
        UsbfEvents = NvBootUsbfHwGetEvent();

        // Check if there is any BUS reset
        if (UsbfEvents & USB_DRF_DEF(USBSTS, URI, USB_RESET))
        {
            //Stop watchdog once host is connected
            NvBootWdtStop();
            // Clear the HW status registers on reset
            // Clear device address by writing the reset value
            USB_REG_WR(PERIODICLISTBASE, USB_DEF_RESET_VAL(PERIODICLISTBASE));
            // Clear setup token, by reading and wrting back the same value
            USB_REG_WR(ENDPTSETUPSTAT, USB_REG_RD(ENDPTSETUPSTAT));
            // Clear endpoint complete status bits.by reading and writing back
            USB_REG_WR(ENDPTCOMPLETE, USB_REG_RD(ENDPTCOMPLETE));
            // Flush the endpoints
            BootError = NvBootUsbfHwEndPointFlush(USB_FLUSH_ALL_EPS);
        }

        // Look for port change status
        if (UsbfEvents & USB_DRF_DEF(USBSTS, PCI, PORT_CHANGE))
        {
            // If port change detected then get the current speed of operation
            // get the USB port speed for transmiting the packet size
            s_pUsbfCtxt->UsbPortSpeed = (NvBootUsbfPortSpeed)
                                                            USB_REG_READ_VAL(HOSTPC1_DEVLC, PSPD);
        }

        // Get the USB Setup packet status
        EpSetupStatus = USB_REG_RD(ENDPTSETUPSTAT);

        // Check for setup packet arrival. Store it and clear the setup lockout.
        if (EpSetupStatus & USB_DRF_DEF(ENDPTSETUPSTAT, 
                                        ENDPTSETUPSTAT0, SETUP_RCVD))
        {
            // Write back the same value to clear the register
            USB_REG_WR(ENDPTSETUPSTAT,EpSetupStatus);

            // Read the setup packet from the Control OUT endpoint Queue head
            NvBootUtilMemcpy(&s_pUsbfCtxt->setupPkt[0],
                            (void *)&s_pUsbDescriptorBuf->pQueueHead
                            [USB_EP_CTRL_OUT].setupBuffer0,
                            USB_SETUP_PKT_SIZE);

            // Process the setup packet
            BootError = NvBootUsbfProcessSetupPacket(pXferBuffer);
            if (BootError != NvBootError_Success)
            {
                // If there is any problem in the ProcessSetupPacket break here
                break;
            }
        }
    }

    // Check for enumeration complete and restun the Success
    if (s_pUsbfCtxt->EnumerationDone)
    {
        BootError = NvBootError_Success;
    }

    return BootError;
}


NvBootError
NvBootUsbfTransmit(
    NvU8 *pDataBuf,
    NvU32 TxfrSizeBytes,
    NvU32 *pBytesTxfred)
{
    NvBootError BootError = NvBootError_Success;

    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(pDataBuf);
    NV_ASSERT(pBytesTxfred);
    // Address must be 4K bytes aligned
    NV_ASSERT(!(PTR_TO_ADDR(pDataBuf)&(NVBOOT_USB_BUFFER_ALIGNMENT-1)));

    // Check resquest size is more than the USB buffer size
    if (TxfrSizeBytes > NVBOOT_USB_MAX_TXFR_SIZE_BYTES)
    {
        TxfrSizeBytes = NVBOOT_USB_MAX_TXFR_SIZE_BYTES;
    }

    // Transmit Data over the BUS and wait till transmit is complete or error
    BootError = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, 
                                      USB_EP_BULK_IN, 
                                      pDataBuf, 
                                      TxfrSizeBytes,
                                      NV_TRUE);

    if (BootError != NvBootError_Success)
    {
        *pBytesTxfred = 0;
    }
    else
    {
        *pBytesTxfred = TxfrSizeBytes;
    }

    return BootError;
}


NvBootError
NvBootUsbfReceive(
    NvU8 *pDataBuf,
    NvU32 ReceiveSizeBytes,
    NvU32 *pBytesReceived)
{
    NvBootError BootError = NvBootError_Success;

    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(pDataBuf);
    NV_ASSERT(pBytesReceived);
    // Address must be 4K bytes aligned
    NV_ASSERT(!(PTR_TO_ADDR(pDataBuf)&(NVBOOT_USB_BUFFER_ALIGNMENT-1)));

    // Check resquest size is more than the USB buffer size
    if (ReceiveSizeBytes > NVBOOT_USB_MAX_TXFR_SIZE_BYTES)
    {
        ReceiveSizeBytes = NVBOOT_USB_MAX_TXFR_SIZE_BYTES;
    }

    // Receive Data over the BUS and wait till recieve is complete or error
    BootError = NvBootUsbfHwTxfrStart(s_pUsbfCtxt,
                                      USB_EP_BULK_OUT,
                                      pDataBuf,
                                      ReceiveSizeBytes,
                                      NV_TRUE);

    if (BootError != NvBootError_Success)
    {
        *pBytesReceived = 0;
    }
    else
    {
        *pBytesReceived = ReceiveSizeBytes;
    }

    return BootError;
}

NvBootError NvBootUsbfHandlePendingControlTransfers(NvU8 *pDataBuf)
{
    NvU32 EpSetupStatus = 0;
    NvBootError BootError = NvBootError_Success;

    EpSetupStatus = USB_REG_RD(ENDPTSETUPSTAT);

    // Check for setup packet arrival. Store it and clear the setup lockout.
    if (EpSetupStatus & USB_DRF_DEF(ENDPTSETUPSTAT, 
                ENDPTSETUPSTAT0, SETUP_RCVD))
    {
        // Write back the same value to clear the register
        USB_REG_WR(ENDPTSETUPSTAT,EpSetupStatus);

        // Read the setup packet from the Control OUT endpoint Queue head
        NvBootUtilMemcpy(&s_pUsbfCtxt->setupPkt[0],
                    (void *)&s_pUsbDescriptorBuf->pQueueHead
                    [USB_EP_CTRL_OUT].setupBuffer0,
                    USB_SETUP_PKT_SIZE);

        // Process the setup packet
        BootError = NvBootUsbfProcessSetupPacket(pDataBuf);
        NvBootUtilMemset(pDataBuf, 0, NVBOOT_USB_MAX_TXFR_SIZE_BYTES);
    }
    return BootError;
}
NvBootUsbfEpStatus NvBootUsbfQueryEpStatus(NvBootUsbfEndpoint EndPoint)
{
    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(EndPoint < NvBootUsbfEndPoint_Num);

    // Get the endpoint status from the hardware
    return NvBootUsbfHwEpGetStatus(s_pUsbfCtxt,
                                (EndPoint == NvBootUsbfEndPoint_BulkIn) ?
                                USB_EP_BULK_IN : USB_EP_BULK_OUT);

}

void NvBootUsbfTransferCancel(NvBootUsbfEndpoint EndPoint)
{
    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(EndPoint < NvBootUsbfEndPoint_Num);

    // Clear the transfers
    NvBootUsbfHwTxfrClear(s_pUsbfCtxt,
                          (EndPoint == NvBootUsbfEndPoint_BulkIn) ?
                          USB_EP_BULK_IN : USB_EP_BULK_OUT);
}

void NvBootUsbfEpSetStalledState(NvBootUsbfEndpoint Endpoint, NvBool StallState)
{
    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(Endpoint < NvBootUsbfEndPoint_Num);

    // RX/TX Endpoint Stall: Read/Write
    // stallState = 0 = End Point OK
    // stallState = 1 = End Point Stalled
    if (Endpoint == NvBootUsbfEndPoint_BulkIn)
    {
        USB_REG_UPDATE_NUM(ENDPTCTRL1, TXS, StallState);
        if (!StallState)
            USB_REG_UPDATE_DEF(ENDPTCTRL1, TXR, RESET_PID_SEQ);
    }
    else
    {
        USB_REG_UPDATE_NUM(ENDPTCTRL1, RXS, StallState);
        if (!StallState)
            USB_REG_UPDATE_DEF(ENDPTCTRL1, RXR, RESET_PID_SEQ);
    }
}

void NvBootUsbfTransmitStart(NvU8 *pDataBuf, NvU32 TxfrSizeBytes)
{
    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(pDataBuf);
    // Address must be 4K bytes aligned
    NV_ASSERT(!(PTR_TO_ADDR(pDataBuf)&(NVBOOT_USB_BUFFER_ALIGNMENT-1)));

    // Check resquest size is more than the USB buffer size
    if (TxfrSizeBytes > NVBOOT_USB_MAX_TXFR_SIZE_BYTES)
    {
        TxfrSizeBytes = NVBOOT_USB_MAX_TXFR_SIZE_BYTES;
    }

    // Initiate the transfer and come out don't wait for transfer complete.
    NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_BULK_IN, 
                          pDataBuf, TxfrSizeBytes, NV_FALSE);

}


NvU32 NvBootUsbfGetBytesTransmitted(void)
{
    NvBootUsbfEpStatus EpStatus;
    NvBootUsbDevQueueHead *pQueueHead = NULL;

    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);

    // Get once the EP status, to find wether Txfer is success or not
    EpStatus = NvBootUsbfHwEpGetStatus(s_pUsbfCtxt, USB_EP_BULK_IN);

    // Update bytes recieved only if transfer is complete
    if (EpStatus == NvBootUsbfEpStatus_TxfrComplete)
    {
        // Temporary pointer to queue head.
        pQueueHead = &s_pUsbDescriptorBuf->pQueueHead[USB_EP_BULK_IN];
        // Get the actual bytes transfered by USB controller
        return (s_pUsbDescriptorBuf->BytesRequestedForEp[USB_EP_BULK_IN] - 
                USB_DQH_DRF_VAL(TOTAL_BYTES, pQueueHead->DtdToken));
    }

    // return 0 if transfer is not happend
    return 0;
}

void NvBootUsbfReceiveStart(NvU8 *pDataBuf, NvU32 DataSize)
{
    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);
    NV_ASSERT(pDataBuf);
    // Address must be 4K bytes aligned
    NV_ASSERT(!(PTR_TO_ADDR(pDataBuf)&(NVBOOT_USB_BUFFER_ALIGNMENT-1)));

    // Check resquest size is more than the USB buffer size
    if (DataSize > NVBOOT_USB_MAX_TXFR_SIZE_BYTES)
    {
        DataSize = NVBOOT_USB_MAX_TXFR_SIZE_BYTES;
    }

    // Initiate the recieve operation and come out
    NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_BULK_OUT,
                          pDataBuf, DataSize, NV_FALSE);
}

NvU32 NvBootUsbfGetBytesReceived(void)
{
    NvBootUsbfEpStatus EpStatus;
    NvBootUsbDevQueueHead *pQueueHead = NULL;

    // Validating the parameters 
    NV_ASSERT(s_pUsbfCtxt);
    NV_ASSERT(s_pUsbfCtxt->UsbControllerEnabled);

    // Get once the EP status, to find wether Txfer is success or not
    EpStatus = NvBootUsbfHwEpGetStatus(s_pUsbfCtxt, USB_EP_BULK_OUT);

    // Update the buffer and bytes recived only if transfer is completed
    if (EpStatus == NvBootUsbfEpStatus_TxfrComplete)
    {
        // Temporary pointer to queue head.
        pQueueHead = &s_pUsbDescriptorBuf->pQueueHead[USB_EP_BULK_OUT];
        // Get the actual bytes transfered by USB controller
        return (s_pUsbDescriptorBuf->BytesRequestedForEp[USB_EP_BULK_OUT] -
                USB_DQH_DRF_VAL(TOTAL_BYTES, pQueueHead->DtdToken));
    }

    // return 0 if transfer is not happend
    return 0;
}


static NvBootError NvBootUsbfProcessSetupPacket(NvU8 *pXferBuffer)
{
    NvBootUsbfEpStatus EpStatus = NvBootUsbfEpStatus_Stalled;
    NvBootError ErrStatus = NvBootError_Success;
    NvBool transmitData = NV_FALSE;
    NvU8 *pDataBuffer = NULL;
    NvU32 DataSize = 0;
    NvU8 InterfaceStatus[2] = {0, 0}; //GetStatus() Request to an Interface is always 0
    NvU8 EndpointStatus[2] = {0, 0}; //GetStatus() Request to an Interface is always 0
    NvU32 SetupPktLength = 0, EndpointAddr = 0;
    NvBool IsEndPointStall = NV_FALSE; // Set default to Not to stall controll endpoint
    NvBool AckEp = NV_FALSE;


    // This field specifies the length of the data transferred during the
    // second phase of the control transfer. (2 bytes in the setup packet)
    SetupPktLength = (s_pUsbfCtxt->setupPkt[USB_SETUP_LENGTH]) |
                     (s_pUsbfCtxt->setupPkt[USB_SETUP_LENGTH + 1] << 8);

    // switch to the request direction
    switch (s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST_TYPE])
    {
        case HOST2DEV_DEVICE:
            // Process the Host -> device Device descriptor
            ErrStatus = NvBootUsbfProcessHost2DevDevicePacket(&IsEndPointStall);
            break;

        case HOST2DEV_INTERFACE:
            // Start the endpoint for zero packet acknowledgment
            ErrStatus = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_CTRL_IN, 
                                              NULL, 0, NV_TRUE);
            if (ErrStatus == NvBootError_Success)
            {
                // Set the Interface number sent by the host
                s_pUsbfCtxt->UsbInterfaceNo = s_pUsbfCtxt->setupPkt[USB_SETUP_VALUE];
            }
            break;

        case DEV2HOST_DEVICE:
            switch(s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST])
            {
                case GET_STATUS:
                    transmitData = NV_TRUE;
                    pDataBuffer = &s_UsbDeviceStatus[0];
                    DataSize = USB_DEV_STATUS_LENGTH;
                    break;
                case GET_CONFIGURATION:
                    transmitData = NV_TRUE;
                    pDataBuffer = &s_pUsbfCtxt->UsbConfigurationNo;
                    DataSize = SetupPktLength;
                    break;
                case GET_DESCRIPTOR:
                    // Get Descriptor Request
                    NvBootUsbfProcessDeviceDescriptor(&transmitData,
                                                      &pDataBuffer,
                                                      &DataSize,
                                                      &IsEndPointStall);
                    break;
                default:
                    // Stall if any Un supported request comes
                    IsEndPointStall = NV_TRUE;
                    break;
            }
            break;

        case DEV2HOST_INTERFACE:
            switch (s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST])
            {
                case GET_STATUS: // Get Status/Interface Request
                    transmitData = NV_TRUE;
                    pDataBuffer = &InterfaceStatus[0];
                    DataSize = SetupPktLength;
                    break;
                case GET_INTERFACE: // Get Interface Request
                    transmitData = NV_TRUE;
                    pDataBuffer = &s_pUsbfCtxt->UsbInterfaceNo;
                    DataSize = SetupPktLength;
                    break;
                default:
                    // Stall if any Un supported request comes
                    IsEndPointStall = NV_TRUE;
                    break;
            }
            break;

            // Stall here, as we don't support endpoint requests here
        case DEV2HOST_ENDPOINT:
            switch (s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST])
            {
                case GET_STATUS: // Get Status/Interface Request
                    transmitData = NV_TRUE;
                    // Get the Ep number from 'wIndex' field and  get the 
                    // Halt status and update the data buffer. Tempararily 
                    // sending 0 ( not haltted for all ep).
                    // Read the Ep status, If ep is STALLED, return 1 other 
                    // wise 0
                    // Get once the EP status, to find wether Txfer is success or not
                    EndpointAddr = ((s_pUsbfCtxt->setupPkt[USB_SETUP_INDEX]) |
                                     (s_pUsbfCtxt->setupPkt[USB_SETUP_INDEX + 1] << 8));

                    switch(EndpointAddr)
                    {
                        case CTRL_OUT:
                            EpStatus = NvBootUsbfHwEpGetStatus(s_pUsbfCtxt, USB_EP_CTRL_OUT);
                            break;
                        case CTRL_IN:
                            EpStatus = NvBootUsbfHwEpGetStatus(s_pUsbfCtxt, USB_EP_CTRL_IN);
                            break;
                        case BULK_OUT:
                            EpStatus = NvBootUsbfHwEpGetStatus(s_pUsbfCtxt, USB_EP_BULK_OUT);
                            break;
                        case BULK_IN:
                            EpStatus = NvBootUsbfHwEpGetStatus(s_pUsbfCtxt, USB_EP_BULK_IN);
                            break;
                        default:
                            transmitData = NV_FALSE;
                            NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                            break;
                    }
                    if (EpStatus == NvBootUsbfEpStatus_Stalled)
                        EndpointStatus[0] = 1;
                    else
                        EndpointStatus[0] = 0;
                    pDataBuffer = &EndpointStatus[0];
                    DataSize = SetupPktLength;
                    break;
                default:
                    transmitData = NV_FALSE;
                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                    // Stall if any Un supported request comes
                    break;
            }
            break;

        case HOST2DEV_ENDPOINT:
            switch (s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST])
            {
                case SET_FEATURE:
                    switch (s_pUsbfCtxt->setupPkt[USB_SETUP_VALUE])
                    {
                        case ENDPOINT_HALT:
                            AckEp = NV_TRUE;
                            // Get once the EP status, to find wether Txfer is success or not
                            EndpointAddr = (s_pUsbfCtxt->setupPkt[USB_SETUP_INDEX]) |
                                                    (s_pUsbfCtxt->setupPkt[USB_SETUP_INDEX + 1] << 8);

                            switch(EndpointAddr)
                            {
                                case CTRL_OUT:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlOut, NV_TRUE);
                                    break;
                                case CTRL_IN:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                                    break;
                                case BULK_OUT:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_BulkOut, NV_TRUE);
                                    break;
                                case BULK_IN:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_BulkIn, NV_TRUE);
                                    break;
                                default:
                                    AckEp = NV_FALSE;
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                                    break;
                            }

                            if (AckEp)
                            {
                                // ACK the endpoint
                                // Start the endpoint for zero packet acknowledgment
                                ErrStatus = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_CTRL_IN, 
                                                                  NULL, 0, NV_TRUE);
                            }
                            break;
                        default:
                            NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                            break;
                    }
                    break;
                case CLEAR_FEATURE:
                    switch (s_pUsbfCtxt->setupPkt[USB_SETUP_VALUE])
                    {
                        case ENDPOINT_HALT:
                            AckEp = NV_TRUE;
                            // Get once the EP status, to find wether Txfer is success or not
                            EndpointAddr = ((s_pUsbfCtxt->setupPkt[USB_SETUP_INDEX]) |
                                                    (s_pUsbfCtxt->setupPkt[USB_SETUP_INDEX + 1] << 8));

                            switch (EndpointAddr)
                            {
                                case CTRL_OUT:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlOut, NV_FALSE);
                                    break;
                                case CTRL_IN:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_FALSE);
                                    break;
                                case BULK_OUT:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_BulkOut, NV_FALSE);
                                    break;
                                case BULK_IN:
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_BulkIn, NV_FALSE);
                                    break;
                                default:
                                    AckEp = NV_FALSE;
                                    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                                    break;
                            }

                            if (AckEp)
                            {
                                // ACK the endpoint
                                // Start the endpoint for zero packet acknowledgment
                                ErrStatus = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_CTRL_IN, 
                                                                  NULL, 0, NV_TRUE);
                            }
                            break;
                        default:
                            NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_ControlIn, NV_TRUE);
                            break;
                    }
                    break;
                default:
                    // Stall if any Un supported request comes
                   IsEndPointStall = NV_TRUE;
                    break;
            }
            break;
        default:
            // Stall if any Un supported request comes
            IsEndPointStall = NV_TRUE;
            break;
    }

    // Transmit data to the Host if any decriptors need to send to Host
    if (transmitData)
    {
        // Copy Descriptor data to the USB buffer to transmit over USB
        NvBootUtilMemcpy(pXferBuffer, pDataBuffer, DataSize);
        // Transmit the data to the HOST
        ErrStatus = NvBootUsbfHwTxfrStart(
                    s_pUsbfCtxt,
                    USB_EP_CTRL_IN,
                    pXferBuffer,
                    SetupPktLength >= DataSize? DataSize: SetupPktLength,
                    NV_TRUE);
        // Ack on OUT end point after data is sent
        if (ErrStatus == NvBootError_Success)
        {
            // Start the endpoint for zero packet acknowledgment
            ErrStatus = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_CTRL_OUT, 
                                              NULL, 0, NV_TRUE);
        }
    }

    if (IsEndPointStall)
    {
        NvU32 regVal = 0;

        // Control endpoints can only be stalled as a pair. They unstall
        // automatically when another setup packet arrives.
        regVal = USB_DRF_DEF(ENDPTCTRL0, TXT, CTRL) |
                 USB_DRF_DEF(ENDPTCTRL0, TXE, ENABLE) |
                 USB_DRF_DEF(ENDPTCTRL0, TXS, EP_STALL) |
                 USB_DRF_DEF(ENDPTCTRL0, RXT, CTRL) |
                 USB_DRF_DEF(ENDPTCTRL0, RXE, ENABLE) |
                 USB_DRF_DEF(ENDPTCTRL0, RXS, EP_STALL);
        USB_REG_WR(ENDPTCTRL0, regVal);
    }

    return ErrStatus;
}


static NvBootError 
NvBootUsbfProcessHost2DevDevicePacket(
    NvBool *pIsEndPointStall)
{
    NvBootError ErrStatus = NvBootError_Success;

    if (s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST] == SET_ADDRESS)
    {
        // ACK the endpoint
        // Start the endpoint for zero packet acknowledgment
        ErrStatus = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_CTRL_IN, 
                                          NULL, 0, NV_TRUE);

        if (ErrStatus == NvBootError_Success)
        {
            // Set device address in the USB controller
            USB_REG_UPDATE_NUM(PERIODICLISTBASE, USBADR, 
                               s_pUsbfCtxt->setupPkt[USB_SETUP_VALUE]);
        }
    }
    else if (s_pUsbfCtxt->setupPkt[USB_SETUP_REQUEST] == SET_CONFIGURATION)
    {
        // ACK the endpoint
        // Start the endpoint for zero packet acknowledgment
        ErrStatus = NvBootUsbfHwTxfrStart(s_pUsbfCtxt, USB_EP_CTRL_IN, 
                                          NULL, 0, NV_TRUE);

        if (ErrStatus == NvBootError_Success)
        {
            s_pUsbfCtxt->UsbConfigurationNo =
                                s_pUsbfCtxt->setupPkt[USB_SETUP_VALUE];
            // Configuring Bulk Out and IN endpoints
            NvBootUsbfHwInitalizeEndpoints(s_pUsbfCtxt, USB_EP_BULK_OUT);
            NvBootUsbfHwInitalizeEndpoints(s_pUsbfCtxt, USB_EP_BULK_IN);
            // enumeration is done now hand over to Bulk
            s_pUsbfCtxt->EnumerationDone = NV_TRUE;
        }
    }
    else
    {
        // Stall if we don't support any other feature
        *pIsEndPointStall = NV_TRUE;
    }

    return ErrStatus;
}


static void
NvBootUsbfProcessDeviceDescriptor(
    NvBool *pTransmitData,
    NvU8 **pDataBuffer,
    NvU32 *pDataSize,
    NvBool *pIsEndPointStall)
{
    // Get Descriptor Request
    switch(s_pUsbfCtxt->setupPkt[USB_SETUP_DESCRIPTOR])
    {
        case USB_DT_DEVICE:
            *pTransmitData = NV_TRUE;
            NvBootConfigureDevDescriptor(&s_UsbDeviceDescriptor[0]);
            *pDataBuffer = &s_UsbDeviceDescriptor[0];
            *pDataSize = USB_DEV_DESCRIPTOR_SIZE;
            break;

        case USB_DT_CONFIG:
            *pTransmitData = NV_TRUE;
            if (s_pUsbfCtxt->UsbPortSpeed == NvBootUsbfPortSpeed_High)
            {
                s_UsbConfigDescriptor[22] =
                                        (USB_HIGH_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_UsbConfigDescriptor[23] =
                                    ((USB_HIGH_SPEED_PKT_SIZE_BYTES>>8) & 0xFF);
                s_UsbConfigDescriptor[29] =
                                         (USB_HIGH_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_UsbConfigDescriptor[30] =
                                    ((USB_HIGH_SPEED_PKT_SIZE_BYTES>>8) & 0xFF);
            }
            else
            {
                s_UsbConfigDescriptor[22] =
                                        (USB_FULL_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_UsbConfigDescriptor[23] =
                                    ((USB_FULL_SPEED_PKT_SIZE_BYTES>>8) & 0xFF);
                s_UsbConfigDescriptor[29] =
                                        (USB_FULL_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_UsbConfigDescriptor[30] =
                                    ((USB_FULL_SPEED_PKT_SIZE_BYTES>>8) & 0xFF);
            }
            /**
             * a. if usb charger detection code is enabled, low battery not detected, 
             * usb charger detection code is skipped. If rcm follows, then use bMaxPower=32mA.
             * b. if usb charger detection code is enabled, low battery detected, usb 
             * charger detection code executes, use bMaxPower=500mA. If rcm follows, don't re-enumerate
             * if enumeration at bMaxPower=500mA was successful, attempt enumeration at bMaxPower=32mA
             * if enumeration failed during usb charger detection logic.
             * c. if usb charger detection code is disabled, irrespective of low battery, 
             * usb charger detection code is skipped. If rcm follows, then use bMaxPower=32mA.
             */
            if ((CHECK_DEVICE_ENUMERATION_STATUS(BootInfoTable.UsbChargingStatus, 	
                    UsbChargingBITStat_ChargerDetectionEnabled)) &&
                (CHECK_DEVICE_ENUMERATION_STATUS(BootInfoTable.UsbChargingStatus, 	
                    UsbChargingBITStat_IsBatteryLowDetected)) &&
                (!CHECK_DEVICE_ENUMERATION_STATUS(BootInfoTable.UsbChargingStatus, 	
                    UsbChargingBITStat_DeviceEnumerationFailed)))
            {
                /// If usb charger detection code is enabled and low battery is detected
                s_UsbConfigDescriptor[8] = USB_BMAX_POWER_500MA;
            }


            *pDataBuffer = &s_UsbConfigDescriptor[0];
            *pDataSize = USB_CONFIG_DESCRIPTOR_SIZE;
            break;

        case USB_DT_STRING:
            *pTransmitData = NV_TRUE;
            switch (s_pUsbfCtxt->setupPkt[USB_SETUP_VALUE])
            {
                case USB_MANF_ID: //Manufacture ID
                    *pDataSize = (s_UsbManufacturerID[0] <=
                                USB_MANF_STRING_LENGTH) ?
                                s_UsbManufacturerID[0] :
                                USB_MANF_STRING_LENGTH;
                    *pDataBuffer = &s_UsbManufacturerID[0];
                    break;
                case USB_PROD_ID:    // Product ID
                    *pDataSize = (s_UsbProductID[0] <=
                                USB_PRODUCT_STRING_LENGTH) ?
                                s_UsbProductID[0] :
                                USB_PRODUCT_STRING_LENGTH;
                    *pDataBuffer = &s_UsbProductID[0];
                    break;
                case USB_SERIAL_ID:    // Serial Number
                    *pDataBuffer = &s_UsbSerialNumber[0];
                    *pDataSize = (s_UsbSerialNumber[0] <=
                                USB_SERIAL_NUM_LENGTH) ?
                                s_UsbSerialNumber[0] :
                                USB_SERIAL_NUM_LENGTH;
                    break;
                case USB_LANGUAGE_ID:    //Language ID
                //Default case return Language ID
                default: //Language ID
                    *pDataSize = USB_LANGUAGE_ID_LENGTH;
                    *pDataBuffer = &s_UsbLanguageID[0];
                    break;
            }
            break;

        case USB_DT_DEVICE_QUALIFIER:
            *pTransmitData = NV_TRUE;
            *pDataBuffer = &s_UsbDeviceQualifier[0];
            *pDataSize = USB_DEV_QUALIFIER_LENGTH;
            break;
        case USB_DT_OTHER_SPEED_CONFIG:
            *pTransmitData = NV_TRUE;
            if (s_pUsbfCtxt->UsbPortSpeed == NvBootUsbfPortSpeed_High)
            {
                s_OtherSpeedConfigDesc[22] = (USB_FULL_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_OtherSpeedConfigDesc[23] = ((USB_FULL_SPEED_PKT_SIZE_BYTES >>8) & 0xFF);
                s_OtherSpeedConfigDesc[29] = (USB_FULL_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_OtherSpeedConfigDesc[30] = ((USB_FULL_SPEED_PKT_SIZE_BYTES >>8) & 0xFF);
            }
            else
            {
                s_OtherSpeedConfigDesc[22] = (USB_HIGH_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_OtherSpeedConfigDesc[23] = ((USB_HIGH_SPEED_PKT_SIZE_BYTES >>8) & 0xFF);
                s_OtherSpeedConfigDesc[29] = ( USB_HIGH_SPEED_PKT_SIZE_BYTES & 0xFF);
                s_OtherSpeedConfigDesc[30] = ((USB_HIGH_SPEED_PKT_SIZE_BYTES >>8) & 0xFF);
            }
            /**
             * a. if usb charger detection code is enabled, low battery not detected, 
             * usb charger detection code is skipped. If rcm follows, then use bMaxPower=32mA.
             * b. if usb charger detection code is enabled, low battery detected, usb 
             * charger detection code executes, use bMaxPower=500mA. If rcm follows, don't re-enumerate.
             * c. if usb charger detection code is disabled, irrespective of low battery, 
             * usb charger detection code is skipped. If rcm follows, then use bMaxPower=32mA.
             */
            if ((CHECK_DEVICE_ENUMERATION_STATUS(BootInfoTable.UsbChargingStatus, 	
                    UsbChargingBITStat_ChargerDetectionEnabled)) &&
                (CHECK_DEVICE_ENUMERATION_STATUS(BootInfoTable.UsbChargingStatus, 	
                    UsbChargingBITStat_IsBatteryLowDetected)))
            {
                /// If usb charger detection code is enabled and low battery is detected
                s_OtherSpeedConfigDesc[8] = USB_BMAX_POWER_500MA;
            }
            *pDataBuffer = &s_OtherSpeedConfigDesc[0];
            *pDataSize = sizeof(s_OtherSpeedConfigDesc);
            break;
        default:
            // Stall if any Un supported request comes
            *pIsEndPointStall = NV_TRUE;
            break;
    }
}

/*
 * Function to init queue head and endpoints
 * @param None
 *
 * @return HwTimeOut if failed else NvSuccess
 */
static NvBootError
NvBootUsbfInitializeQueueHeadAndEp(NvBootUsbfContext *pUsbFuncCtxt)
{
    NvU32 TimeOut = CONTROLLER_HW_TIMEOUT_US;
    NvU32 regValue = 0;

    // Clear Queuehead descriptors
    NvBootUtilMemset(&s_pUsbDescriptorBuf->pQueueHead[0],
                     0,
                     sizeof(NvBootUsbDevQueueHead) * USBF_MAX_EP_COUNT);

    // Clear Transfer descriptors
    NvBootUtilMemset(&s_pUsbDescriptorBuf->pDataTransDesc[0],
                     0,
                     sizeof(NvBootUsbDevTransDesc) * USBF_MAX_DTDS);

    // Initialize the EndPoint Base Addr with the Queue Head base address
    USB_REG_WR(ASYNCLISTADDR,
               PTR_TO_ADDR(&s_pUsbDescriptorBuf->pQueueHead[0]));

    // Initaialize the control Out endpoint
    NvBootUsbfHwInitalizeEndpoints(pUsbFuncCtxt, USB_EP_CTRL_OUT);
    // Initaialize the control IN endpoint
    NvBootUsbfHwInitalizeEndpoints(pUsbFuncCtxt, USB_EP_CTRL_IN);

    // FIXME:  Disable auto suspend bit (ASUS) Bug#797390
    USB_REG_UPDATE_DEF(HOSTPC1_DEVLC, ASUS, DISABLE);

    USB_REG_UPDATE_DEF(USBCMD, RS, RUN);
    TimeOut = CONTROLLER_HW_TIMEOUT_US;
    do {
        // wait till device starts running.
        regValue = USB_REG_READ_VAL(USBCMD, RS);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (regValue != USB_DRF_DEF_VAL(USBCMD, RS, RUN));
    return NvBootError_Success;
}

static NvU32
NvBootUsbfHwGetPacketSize(
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint)
{
    NvU32 PacketSize = USB_FULL_SPEED_PKT_SIZE_BYTES;
    NvBool HighSpeed = NV_FALSE;

    // Return packet size based on speed and endpoint type.
    switch (EndPoint)
    {
        // If endpoint is control IN or OUT return FULL speed pkt size.
        case USB_EP_CTRL_OUT:
        case USB_EP_CTRL_IN:
            PacketSize = USB_FULL_SPEED_PKT_SIZE_BYTES;
            break;
        // If bulk IN or OUT return speed depending on the port status
        case USB_EP_BULK_OUT:
        case USB_EP_BULK_IN:
            HighSpeed = (pUsbFuncCtxt->UsbPortSpeed == NvBootUsbfPortSpeed_High);
            // Return the speed based on the port speed for bulk endpoints
            // fall through the default for returning the Packet size.
        default:
            // In the default return the packet size as FULL speed packet
            PacketSize = HighSpeed ?
                         USB_HIGH_SPEED_PKT_SIZE_BYTES :
                         USB_FULL_SPEED_PKT_SIZE_BYTES;
            break;
    } 

    return PacketSize;
}



static NvBootError
NvBootUsbfHwTxfrStart(
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint,
    NvU8 *pDataBuf,
    NvU32 maxTxfrBytes,
    NvBool WaitForTxfrComplete)
{
    NvBootError BootError = NvBootError_Success;
    NvBootUsbDevTransDesc *pUsbDevTxfrDesc;
    NvBootUsbDevQueueHead *pUsbDevQueueHead;
    NvBootUsbfEpStatus EpStatus;
    NvU32 regVal;

    // Clear leftover transfer, if any.before starting the transaction
    NvBootUsbfHwTxfrClear(pUsbFuncCtxt, EndPoint);

    // Reference to queue head for configureation.
    pUsbDevQueueHead = &s_pUsbDescriptorBuf->pQueueHead[EndPoint];
    // Clear the Queue Head before configuration
    NvBootUtilMemset(pUsbDevQueueHead, 0, sizeof(NvBootUsbDevQueueHead));

    // Interrupt on setup if it is endpoint 0_OUT.
    if (EndPoint == USB_EP_CTRL_OUT)
    {
        pUsbDevQueueHead->EpCapabilities = USB_DQH_DRF_DEF(IOC, ENABLE);
    }
    // setup the Q head with Max packet length and zero length terminate
    pUsbDevQueueHead->EpCapabilities |= (USB_DQH_DRF_NUM(MAX_PACKET_LENGTH,
                        NvBootUsbfHwGetPacketSize( pUsbFuncCtxt, EndPoint))|
                        USB_DQH_DRF_DEF(ZLT, ZERO_LENGTH_TERM_DISABLED));

    // Don't to terminate the next DTD pointer by writing 0 = Clear
    // we will assign the DTD pointer after configuring DTD
    pUsbDevQueueHead->NextDTDPtr = USB_DQH_DRF_DEF(TERMINATE, CLEAR);
    // Indicate EP is configured
    s_pUsbDescriptorBuf->EpConfigured[EndPoint] = NV_TRUE;
    //Store the Bytes requested by client
    s_pUsbDescriptorBuf->BytesRequestedForEp[EndPoint] = maxTxfrBytes;

    // Reference to DTD for configureation.
    pUsbDevTxfrDesc = &s_pUsbDescriptorBuf->pDataTransDesc[EndPoint];
    // clear the DTD before configuration
    NvBootUtilMemset(pUsbDevTxfrDesc, 0, sizeof(NvBootUsbDevTransDesc));

    // Setup the DTD
    pUsbDevTxfrDesc->NextDtd = USB_DTD_DRF_DEF(TERMINATE, SET);
    // Set number of bytes to transfer in DTD.and set status to active
    pUsbDevTxfrDesc->DtdToken = USB_DTD_DRF_DEF(ACTIVE, SET)|
                                USB_DTD_DRF_NUM(TOTAL_BYTES, maxTxfrBytes);
    // Assign buffer pointer to DTD.
    pUsbDevTxfrDesc->BufPtrs[0] = PTR_TO_ADDR(pDataBuf);

    ///Next DTD address need to program only upper 27 bits
    pUsbDevQueueHead->NextDTDPtr |= USB_DQH_DRF_NUM(NEXT_DTD_PTR,
                                    (PTR_TO_ADDR(pUsbDevTxfrDesc) >> 
                                    USB_DQH_FLD_SHIFT_VAL(NEXT_DTD_PTR)));

    // Start the transaction on the USB Bus by priming the endpoint
    regVal = USB_REG_RD(ENDPTPRIME);
    regVal |= USB_EP_NUM_TO_WORD_MASK(EndPoint);
    USB_REG_WR(ENDPTPRIME, regVal);

    if (WaitForTxfrComplete)
    {
        // wait for transfer complete on the USB bus
        BootError = NvBootUsbfHwTxfrWait(pUsbFuncCtxt,
                                EndPoint, USB_MAX_TXFR_WAIT_TIME_MS);
        if (BootError != NvBootError_Success)
        {
            // there is some error in the transmit operation
            // clear the pending transfer and return the error info.
            NvBootUsbfHwTxfrClear(pUsbFuncCtxt, EndPoint);
            return BootError;
        }
        ///Get once the EP status to find wether Txfer is success or not
        EpStatus = NvBootUsbfHwEpGetStatus(pUsbFuncCtxt, EndPoint);

        if (EpStatus == NvBootUsbfEpStatus_TxfrComplete)
            BootError = NvBootError_Success;
        else
            BootError = NvBootError_TxferFailed;
    }

    return  BootError;
}


static NvBootError
NvBootUsbfHwTxfrWait(
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint,
    NvU32 timeoutMs)
{
    NvBootError retStatus = NvBootError_Success;
    NvBootUsbfEpStatus EpStatus;
    NvU32 time;

    // Get atleaset once the EP status
    EpStatus = NvBootUsbfHwEpGetStatus(pUsbFuncCtxt, EndPoint);

    // If transfer is active wit till the time out or transfer status change
    if (EpStatus == NvBootUsbfEpStatus_TxfrActive)
    {
        // Multiplying by 1000 to make delay wait time in micro seconds
        for (time = 0; time < (timeoutMs*1000); time++)
        {
            NvBootUtilWaitUS(1);
            EpStatus = NvBootUsbfHwEpGetStatus(pUsbFuncCtxt, EndPoint);
            if (EpStatus == NvBootUsbfEpStatus_TxfrActive)
            {
                // continue till time out or transfer is active
                continue;
            }
            else
            {
                // either success or error break here and return the status
                break;
            }
        }
    }
    // Still transfer is active then it means HW is timed out
    if (EpStatus == NvBootUsbfEpStatus_TxfrActive)
    {
        retStatus = NvBootError_HwTimeOut;
    }
    else if (EpStatus == NvBootUsbfEpStatus_NotConfigured)
    {
        // Return EP not configured error
        retStatus = NvBootError_EpNotConfigured;
    }

    return retStatus;
}


static void
NvBootUsbfHwInitalizeEndpoints(
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint)
{
    NvBootUsbDevQueueHead *pQueueHead;

    // Set up Queue Head for endpoint.
    pQueueHead = &s_pUsbDescriptorBuf->pQueueHead[EndPoint];
    NvBootUtilMemset((void*) pQueueHead, 0, sizeof(NvBootUsbDevQueueHead));

    // Interrupt on setup if it is endpoint 0_OUT.
    if (EndPoint == USB_EP_CTRL_OUT)
    {
        pQueueHead->EpCapabilities = USB_DQH_DRF_DEF(IOC, ENABLE);
    }

    // terminate the next DTD pointer by writing 1 = SET
    pQueueHead->NextDTDPtr = USB_DQH_DRF_DEF(TERMINATE, SET);

    pQueueHead->EpCapabilities |= USB_DQH_DRF_NUM(MAX_PACKET_LENGTH,
                           NvBootUsbfHwGetPacketSize( pUsbFuncCtxt, EndPoint));

    if (USB_IS_EP_IN(EndPoint))
    {
        // Configure transmit endpoints.
        // Check endpoint type and configure corresponding register
        // endpoint# 0 and 1 is control OUT and IN
        // and endpoint# 2 and 3 Bulk OUT and IN
        if (EndPoint >> 1)
        {
            // Configuring BULK IN endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL1, TXT, BULK);
            USB_REG_UPDATE_DEF(ENDPTCTRL1, TXS, EP_OK);
            // Whenever a configuration event is received for  this Endpoint,
            // software must write a one to this bit in order to synchronize
            // the data PIDs between the host and device.
            USB_REG_UPDATE_DEF(ENDPTCTRL1, TXR, RESET_PID_SEQ);
            // Enable transmit endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL1, TXE, ENABLE);
        }
        else
        {
            // Configuring CONTROL IN endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL0, TXT, CTRL);
            USB_REG_UPDATE_DEF(ENDPTCTRL0, TXS, EP_OK);
            // Enable transmit endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL0, TXE, ENABLE);
        }
    }
    else
    {
        // Configure Recieve endpoints.
        // Check endpoint type and configure corresponding register
        // endpoint# 0 and 1 is control OUT and IN
        // and endpoint# 2 and 3 Bulk OUT and IN
        if (EndPoint >> 1)
        {
            // Configuring BULK OUT endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL1, RXT, BULK);
            USB_REG_UPDATE_DEF(ENDPTCTRL1, RXS, EP_OK);
            // Whenever a configuration event is received for  this Endpoint,
            // software must write a one to this bit in order to synchronize
            // the data PIDs between the host and device.
            USB_REG_UPDATE_DEF(ENDPTCTRL1, RXR, RESET_PID_SEQ);
            // Enable recieve endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL1, RXE, ENABLE);
        }
        else
        {
            // Configuring CONTROL OUT endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL0, RXT, CTRL);
            USB_REG_UPDATE_DEF(ENDPTCTRL0, RXS, EP_OK);
            // Enable recieve endpoint
            USB_REG_UPDATE_DEF(ENDPTCTRL0, RXE, ENABLE);
        }
    }

}


static NvU32 NvBootUsbfHwGetEvent(void)
{
    NvU32 NewUsbInts = 0;
    NvU32 regVal = 0;

    // Get bitmask of status for new USB ints that we have enabled.
    regVal = USB_REG_RD(USBSTS);
    // prepare the Interupt status mask and get the interrupted bits value
    NewUsbInts = (regVal & (USB_DRF_DEF(USBSTS, SLI, SUSPENDED) |
                                 USB_DRF_DEF(USBSTS, SRI, SOF_RCVD) |
                                 USB_DRF_DEF(USBSTS, URI, USB_RESET) |
                                 USB_DRF_DEF(USBSTS, AAI, ADVANCED) |
                                 USB_DRF_DEF(USBSTS, SEI, ERROR) |
                                 USB_DRF_DEF(USBSTS, FRI, ROLLOVER) |
                                 USB_DRF_DEF(USBSTS, PCI, PORT_CHANGE) |
                                 USB_DRF_DEF(USBSTS, UEI, ERROR) |
                                 USB_DRF_DEF(USBSTS, UI, INT)));
    // Clear the interrupted bits by writing back the staus value to register
    regVal |= NewUsbInts;
    USB_REG_WR(USBSTS,regVal);

    return NewUsbInts;
}


static NvBootError NvBootUsbfHwEndPointFlush(NvU32 EndPoint)
{
    NvU32 TimeOut = CONTROLLER_HW_TIMEOUT_US;
    NvU32 EndpointMask;
    NvU32 RegValue;

    // Get the Endpoint mask depending on the endpoin number
    EndpointMask = (EndPoint == USB_FLUSH_ALL_EPS) ?
                   EndPoint : USB_EP_NUM_TO_WORD_MASK(EndPoint);

    // Flush endpoints
    USB_REG_WR(ENDPTFLUSH, EndpointMask);
    do {
        RegValue = USB_REG_RD(ENDPTFLUSH);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (RegValue & EndpointMask);

    // Wait for status bits to clear as well.
    TimeOut = CONTROLLER_HW_TIMEOUT_US;
    do {
        RegValue = USB_REG_RD(ENDPTSTATUS);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (RegValue & EndpointMask);

    // Wait till all primed endpoints clear.
    TimeOut = CONTROLLER_HW_TIMEOUT_US;
    do {
        RegValue = USB_REG_RD(ENDPTPRIME);
        if (!TimeOut)
        {
            return NvBootError_HwTimeOut;
        }
        NvBootUtilWaitUS(1);
        TimeOut--;
    } while (RegValue & EndpointMask);

    return NvBootError_Success;
}


static void
NvBootUsbfHwTxfrClear (
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint)
{
    NvU32 regVal;

    // flush the endpoint
    NvBootUsbfHwEndPointFlush(EndPoint);

    // clear the Queue head for this endpoint
    NvBootUtilMemset(
                &s_pUsbDescriptorBuf->pDataTransDesc[EndPoint],
                0,
                sizeof(NvBootUsbDevTransDesc));
    // clear the transfer descriptor for this endpoint
    NvBootUtilMemset(&s_pUsbDescriptorBuf->pQueueHead[EndPoint],
                        0, sizeof(NvBootUsbDevQueueHead));

    // Indicate the endpoint is cleared.
    s_pUsbDescriptorBuf->EpConfigured[EndPoint] = NV_FALSE;

    // Clear the endpoint bytes requested
    s_pUsbDescriptorBuf->BytesRequestedForEp[EndPoint] = 0;

    // Clear endpoint complete status bits.
    regVal = USB_REG_RD(ENDPTCOMPLETE);
    regVal |= USB_EP_NUM_TO_WORD_MASK(EndPoint);
    USB_REG_WR(ENDPTCOMPLETE, regVal);
}


static NvBootUsbfEpStatus
NvBootUsbfHwEpGetStatus(
    NvBootUsbfContext *pUsbFuncCtxt,
    NvU32 EndPoint)
{
    NvBootUsbfEpStatus EpStatus = NvBootUsbfEpStatus_NotConfigured;
    NvU32 EpCtrlRegValue = 0;
    NvBootUsbDevQueueHead *pQueueHead;

    // Check endpoint type and read corresponding register
    // endpoint# 0 and 1 is control OUT and IN
    // and endpoint# 2 and 3 Bulk OUT and IN
    if (EndPoint >> 1)
    {
        EpCtrlRegValue = USB_REG_RD(ENDPTCTRL1);
    }
    else
    {
        EpCtrlRegValue = USB_REG_RD(ENDPTCTRL0);
    }

    // If endpoint is stalled, report that
    // TXS, TXE, RXS and RXE are at the same location in ENDPTCTRL0
    // and ENDPTCTRL1 so reading from the same DRF register is OK
    if ((USB_IS_EP_IN(EndPoint)) ?
         USB_DRF_VAL(ENDPTCTRL0, TXS, EpCtrlRegValue) :
         USB_DRF_VAL(ENDPTCTRL0, RXS, EpCtrlRegValue))
    {
        EpStatus = NvBootUsbfEpStatus_Stalled;
        goto Exit;
    }

        // If port is not enabled, return end point is not configured.
    if ((USB_IS_EP_IN(EndPoint)) ?
         USB_DRF_VAL(ENDPTCTRL0, TXE, EpCtrlRegValue) :
         USB_DRF_VAL(ENDPTCTRL0, RXE, EpCtrlRegValue))
    {
        // Temporary pointer to queue head.
        pQueueHead = &s_pUsbDescriptorBuf->pQueueHead[EndPoint];
        // Look for an error by inspecting the DTD fields stored in the DQH.
        if (pQueueHead->DtdToken & ( USB_DQH_DRF_DEF(HALTED,SET)|
                              USB_DQH_DRF_DEF(DATA_BUFFER_ERROR,SET)|
                              USB_DQH_DRF_DEF(TRANSACTION_ERROR,SET)))
        {
            EpStatus = NvBootUsbfEpStatus_TxfrFail;
            goto Exit;
        }
        // If endpoint active, check to see if it has completed an operation.
        if ((USB_REG_RD(ENDPTPRIME) & USB_EP_NUM_TO_WORD_MASK(EndPoint)) ||
            (USB_REG_RD(ENDPTSTATUS) & USB_EP_NUM_TO_WORD_MASK(EndPoint)) )
        {
            EpStatus = NvBootUsbfEpStatus_TxfrActive;
            goto Exit;
        }
        // Check for a complete transaction.
        if (s_pUsbDescriptorBuf->EpConfigured[EndPoint])
        {
            EpStatus = NvBootUsbfEpStatus_TxfrComplete;
            goto Exit;
        }
        // Return saying endpoint is IDLE and ready for Txfer
        EpStatus = NvBootUsbfEpStatus_TxfrIdle;
    }
    else
    {
        // Return as endpoint is not configured
        EpStatus = NvBootUsbfEpStatus_NotConfigured;
    }

Exit:
    return EpStatus;
}

static void NvBootConfigureDevDescriptor(NvU8 *pDevDescriptor)
{
    NvU32 RegValue = 0;
    NvU32 Sku;

    // Read the Chip ID revision register
    RegValue = APB_MISC_REG_RD(GP_HIDREV);

    // Product ID definition (2 bytes as per USB 2.0 Spec)
    // idProduct(LSB - 8bits) = Chip ID
    // idProduct(MSB - 8bits) = Family(4Bits:Bit4-Bit7) and Sku(4Bits:Bit0-Bit3)
    // idProduct(LSB) - Read chip ID from the chip ID register
    pDevDescriptor[10] = NV_DRF_VAL(APB_MISC, GP_HIDREV, CHIPID, RegValue);
    // idProduct(MSB) - Read family and SKu value and define the MSB
    NvBootFuseGetSkuRaw(&Sku);
#define NVBOOT_USBF_DESCRIPTOR_SKU_MASK  0xF  // descriptor gets only 4 low bits
    pDevDescriptor[11] = (NV_DRF_VAL(APB_MISC, GP_HIDREV, HIDFAM, RegValue) << 4) |
        (Sku & NVBOOT_USBF_DESCRIPTOR_SKU_MASK);
    // Device Release number definition (2 bytes as per USB 2.0 Spec)
    // bcd Device (LSB - 8bits) = (Bit4-Bit7 = 0) (Bit0-Bit3 = Minor Rev)
    // bcd Device (MSB - 8bits) = (Bit4-Bit7 = 0) (Bit0-Bit3 = Major Rev)
    // bcd Device (LSB) - Read minor revision
    pDevDescriptor[12] = NV_DRF_VAL(APB_MISC, GP_HIDREV, MINORREV, RegValue);
    // bcd Device (MSB) - Read Major revision
    pDevDescriptor[13] = NV_DRF_VAL(APB_MISC, GP_HIDREV, MAJORREV, RegValue);
}


/* Wrappers to implement rcm ports */
NvBootError SetupUsb(NvU8* pEnumerationBuffer)
{

    for ( ; ; )
    {
        // Start enumeration
        //if (NvBootUsbfStartEnumeration(Buffer[s_State.BufferIndex]) == NvBootError_Success)
        if (NvBootUsbfStartEnumeration(pEnumerationBuffer) == NvBootError_Success)
        {
            // Success fully enumerated
            return NvBootError_Success;

        }
        else
        {
            // Failed to enumerate.
            return NvBootError_DeviceError;
        }
    }
    // It should never come here. Default return is false.
}

NvBootError HandleErrorUSB(void)
{
    NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_BulkOut, NV_TRUE);
    return NvBootError_Success;
}

/* For asynchronous receive */
NvBootError WaitForUsbRecvComplete(NvU32 *RecvSize, NvU32 TimeoutMs, NvU8* pEnumerationBuffer)
{
    NvBootUsbfEpStatus EpStatus;
    NvBootError BootError = NvBootError_Success;

    do
    {
        // Query the endpoint status until the transfer is completed or there
        // is an error detected over the USB or cable disconnect.
        EpStatus = NvBootUsbfQueryEpStatus(NvBootUsbfEndPoint_BulkOut);

        // If EP status is not configured, return back from here
        if (EpStatus == NvBootUsbfEpStatus_NotConfigured)
            break;

        // If BULKEP status is still active, check any control tranfer request from PC
        // Get the USB Setup packet status
        if (EpStatus != NvBootUsbfEpStatus_TxfrComplete)
        {
            // Check if there is any event pending for control transfer and handle it
            //BootError = NvBootUsbfHandlePendingControlTransfers(Buffer[s_State.BufferIndex]);
            BootError = NvBootUsbfHandlePendingControlTransfers(pEnumerationBuffer);
        }
    } while ((EpStatus == NvBootUsbfEpStatus_TxfrActive) || (EpStatus == NvBootUsbfEpStatus_Stalled));

    // Get the bytes recived on the USB
    *RecvSize = NvBootUsbfGetBytesReceived();

    // Check for the endpoint status for transfer complete
    if (EpStatus == NvBootUsbfEpStatus_TxfrComplete)
    {
        // If transfer is successfully completed then return success
        BootError = NvBootError_Success;
    }
    else if (EpStatus == NvBootUsbfEpStatus_NotConfigured)
    {
        // Return EP not configured error
        BootError = NvBootError_EpNotConfigured;
    }
    else
    {
        // If there is any error return the failure
        BootError = NvBootError_TxferFailed;
    }
    return BootError;
}

NvBootError WaitForUsbTxferComplete(NvU32 *TxferSize, NvU32 TimeoutMs, NvU8* pEnumerationBuffer)
{
    NvBootUsbfEpStatus EpStatus;
    NvBootError BootError = NvBootError_Success;

    do
    {
        EpStatus = NvBootUsbfQueryEpStatus(NvBootUsbfEndPoint_BulkIn);

        // If EP status is not configured, return back from here
        if (EpStatus == NvBootUsbfEpStatus_NotConfigured)
            break;
        // If BULKEP status is still active, check any control tranfer request from PC
        // Get the USB Setup packet status
        if (EpStatus != NvBootUsbfEpStatus_TxfrComplete)
        {
            // Check if there is any event pending for control transfer and handle it
            // BootError = NvBootUsbfHandlePendingControlTransfers(Buffer[s_State.BufferIndex ^ 1]);
                BootError = NvBootUsbfHandlePendingControlTransfers(pEnumerationBuffer);
        }
    } while ((EpStatus == NvBootUsbfEpStatus_TxfrActive) || (EpStatus == NvBootUsbfEpStatus_Stalled));

    // Get the bytes recived on the USB
    *TxferSize = NvBootUsbfGetBytesTransmitted();

    // Check for the endpoint status for transfer completion
    if (EpStatus == NvBootUsbfEpStatus_TxfrComplete)
    {
        // If transfer completed successfully then return success
        BootError = NvBootError_Success;
    }
    else if (EpStatus == NvBootUsbfEpStatus_NotConfigured)
    {
        // Return EP not configured error
        BootError = NvBootError_EpNotConfigured;
    }
    else
    {
        /*
         * Stall the endpoint if the EpStatus is TxfrFail so that
         * host should not send any further data.
         */
        NvBootUsbfEpSetStalledState(NvBootUsbfEndPoint_BulkOut, NV_TRUE);

        /* Stop processing. */
        BootError = NvBootError_TxferFailed;
    }
    return BootError;
}
