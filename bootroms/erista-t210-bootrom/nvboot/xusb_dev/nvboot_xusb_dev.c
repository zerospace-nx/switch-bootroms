/** 
 *Insert Nvidia copyright
 */
#include "nvboot_xusb_dev.h"
#include "nvboot_xusb_dev_int.h"
#include "ardev_t_xusb_dev_xhci.h"
#include "ardev_t_fpci_xusb_dev.h"
#include "arxusb_padctl.h"
#include "arxusb_dev.h"
#include "arfuse.h"
#include "arusb_otg.h"
#include "nvrm_drf.h"
#include "nvboot_util_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_wdt_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_arc_int.h"
#include "arapbpm.h"
#include "arapb_misc.h"
#include "nvboot_irom_patch_int.h"

extern const NvU32 s_UsbHsicTrkDiv[];

#define XUSB_DEV_SS_SUPPORTED 0
#define XUSB_FULL_SPEED  1
#define XUSB_HIGH_SPEED  3
#define XUSB_SUPER_SPEED 4
#define SETUP_PACKET_BUFFER_NUM 2
//NvU8 SetupPacketQueue[8*SETUP_PACKET_BUFFER_NUM] __attribute__((aligned(4)));
NvU8 SetupData[8] __attribute__((aligned(4)));
// #define NEXT_SETUP_PKT(a) (a==SETUP_PACKET_BUFFER_NUM-1?0:a+1)
// #define PREV_SETUP_PKT(a) (a==0?SETUP_PACKET_BUFFER_NUM-1:a-1)
#define NV_WRITE32(ADDR, VALUE) \
        *(volatile unsigned int*)(ADDR) = VALUE
#define NV_READ32(ADDR) \
        (*(volatile unsigned int*)(ADDR))
#define SHIFT(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFT(PERIPH##_##REG##_0_##FIELD##_##RANGE)
#define SHIFTMASK(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFTMASK(PERIPH##_##REG##_0_##FIELD##_##RANGE)


#define XUSB_BASE 0x700d0000
#define DIR_OUT 0
#define DIR_IN 1
#define NV_TRUE 1
#define NV_FALSE 0

/** We need 2 ring segments of size 16 each for event ring. 
 *  Use 1 contiguous segment for simplicity.
 */
#define NUM_TRB_EVENT_RING 32
#define EVENT_RING_WRAP_AROUND(a)      \
        (a==&EventRing[NUM_TRB_EVENT_RING-1]?&EventRing[0]:a+1)
static EventTRB_T EventRing[NUM_TRB_EVENT_RING] __attribute__((aligned(16)));

// Transfer Ring for Control Endpoint
#define NUM_TRB_TRANSFER_RING 16
static DataTRB_T TxRingEp0[NUM_TRB_TRANSFER_RING] __attribute__((aligned(16)));
// TODO: tmp using 0x8000_1000
// Transfer Ring for Bulk Out Endpoint
static DataTRB_T TxRingEp1Out[NUM_TRB_TRANSFER_RING]
                        __attribute__((aligned(16)));
// Transfer Ring for Bulk In Endpoint
static DataTRB_T TxRingEp1In[NUM_TRB_TRANSFER_RING]
                        __attribute__((aligned(16)));

// Endpoint descriptor
static EpContext_T EpContext[4] __attribute__((aligned(64)));
//GetStatus() Request to an Interface is always 0
NvU8 InterfaceStatus[2] = {0, 0}; 
//GetStatus() Request to an Interface is always 0
NvU8 EndpointStatus[2] = {0, 0};

// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
///////////////////////////////////////////////////////////////////////////////
//  PLLU configuration information (reference clock is osc/clk_m and PLLU-FOs are fixed at
//  12MHz/60MHz/480MHz).
//
//  reference frequency
//               13.0MHz       19.2MHz      12.0MHz      26.0MHz      16.8MHz....  38.4MHz  48MHz
//  -----------------------------------------------------------------------------------------------
// DIVN      37 (025h)     25 (019h)      40 (028h)    37 (025h)    028 (01Ch)   25 (019h)  20(014h)
// DIVM      1  ( 01h)      1 ( 01h)      1 ( 01h)      2 ( 02h)      1 (01h)      2 (02h)   2 (02h)
// http://nvbugs/1404142 
// PLLU does not have divided reference branches anymore
// Also Post Divider is 5 bits now.
// CLKIN/M must be min 5M and max 20M.
static const UsbPllClockParams s_UsbPllBaseInfo[NvBootClocksOscFreq_MaxVal] = 
{
    //DivN, DivM, DivP
    {0x025, 0x01, 0x1}, // For NvBootClocksOscFreq_13,
    {0x01C,  0x1, 0x1}, // For NvBootClocksOscFreq_16_8
    {    0,    0,   0}, // dummy field
    {    0,    0,   0}, // dummy field
    {0x019, 0x01, 0x1}, // For NvBootClocksOscFreq_19_2
    {0x019, 0x02, 0x1}, // For NvBootClocksOscFreq_38_4,
    {    0,    0,   0}, // dummy field
    {    0,    0,   0}, // dummy field
    {0x028, 0x01, 0x1}, // For NvBootClocksOscFreq_12
    {0x028, 0x04, 0x1}, // For NvBootClocksOscFreq_48,
    {    0,    0,   0}, // dummy field
    {    0,    0,   0}, // dummy field
    {0x025, 0x02, 0x1}  // NvBootClocksOscFreq_26
};

// UTMI PLL needs output = 960 Mhz with osc input
// Bug 1398118
// Note: CLKIN/M ratio should between 12 and 38.4 Mhz
static const UtmiPllClockParams s_UtmiPllBaseInfo[NvBootClocksOscFreq_MaxVal] = 
{
    //DivN, DivM
    {0x04A, 0x01}, // For NvBootClocksOscFreq_13, // Not P0.
    {0x039,  0x1}, // For NvBootClocksOscFreq_16_8 // Not P0.
    {    0,    0}, // dummy field
    {    0,    0}, // dummy field
    {0x032, 0x01}, // For NvBootClocksOscFreq_19_2 // Not P0.
    {0x019, 0x01}, // For NvBootClocksOscFreq_38_4,
    {    0,    0}, // dummy field
    {    0,    0}, // dummy field
    {0x050, 0x01}, // For NvBootClocksOscFreq_12
    {0x028, 0x02}, // For NvBootClocksOscFreq_48, // Not P0.
    {    0,    0}, // dummy field
    {    0,    0}, // dummy field
    {0x04A, 0x02}  // NvBootClocksOscFreq_26 // Not P0.
};

// dummy fields added to maintain NvBootClocksOscFreq enum values in a sequence 
///////////////////////////////////////////////////////////////////////////////
// PLL CONFIGURATION & PARAMETERS: refer to the arapb_misc_utmip.spec file.
///////////////////////////////////////////////////////////////////////////////
// PLL CONFIGURATION & PARAMETERS for different clock generators:
//-----------------------------------------------------------------------------
// Reference frequency            13.0MHz       19.2MHz       12.0MHz      26.0MHz 
// ----------------------------------------------------------------------------
// PLLU_ENABLE_DLY_COUNT   02 (02h)       03 (03h)       02 (02h)     04 (04h)
// PLLU_STABLE_COUNT          51 (33h)       75 (4Bh)       47 (2Fh)     102 (66h)
// PLL_ACTIVE_DLY_COUNT     09 (09h)       12 (0C)        08 (08h)     17 (11h)
// XTAL_FREQ_COUNT             118 (76h)     188 (BCh)     118 (76h)   254 (FEh)
//-----------------------------------------------------------------------------
// Reference frequency            16.8MHz        38.4MHz         48MHz 
// ----------------------------------------------------------------------------
// PLLU_ENABLE_DLY_COUNT   03 (03h)        05 (05h)         06 (06h)
// PLLU_STABLE_COUNT          66 (42h)        150 (96h)       188 (BCh)
// PLL_ACTIVE_DLY_COUNT     11 (0Bh)       24 (18h)         30 (1Eh)
// XTAL_FREQ_COUNT             165 (A5h)      375 (177h)     469 (1D5h)
///////////////////////////////////////////////////////////////////////////////
static const UsbPllDelayParams s_UsbPllDelayParams[NvBootClocksOscFreq_MaxVal] =
{
    //ENABLE_DLY,  STABLE_CNT,  ACTIVE_DLY,  XTAL_FREQ_CNT
    {0x02,         0x33,        0x09,       0x7F}, // For NvBootClocksOscFreq_13,
    {0x03,         0x42,        0x0B,       0xA5}, // For NvBootClocksOscFreq_16_8
    {     0,              0,             0,             0}, // dummy field
    {     0,              0,             0,             0}, // dummy field
    {0x03,         0x4B,       0x0C,        0xBC}, // For NvBootClocksOscFreq_19_2
    {0x05,         0x96,       0x18,      0x177},  //For NvBootClocksOscFreq_38_4
    {     0,              0,             0,             0}, // dummy field
    {     0,             0,              0,             0}, // dummy field
    {0x02,         0x2F,       0x08,        0x76}, // For NvBootClocksOscFreq_12
    {0x06,         0xBC,       0X1F,      0x1D5}, // For NvBootClocksOscFreq_48
    {     0,              0,            0,              0}, // dummy field
    {     0,              0,            0,              0}, // dummy field
    {0x04,         0x66,        0x11,       0xFE}  // For NvBootClocksOscFreq_26
};

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
__attribute__((aligned(4))) static NvU8 s_UsbDeviceDescriptor[USB_DEV_DESCRIPTOR_SIZE] =
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
__attribute__((aligned(4))) static NvU8 s_UsbBOSDescriptor[USB_BOS_DESCRIPTOR_SIZE] =
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
__attribute__((aligned(4))) static NvU8 s_UsbConfigDescriptor[USB_CONFIG_DESCRIPTOR_SIZE] =
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

__attribute__((aligned(4))) static NvU8 s_OtherSpeedConfigDesc[32]= {

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
__attribute__((aligned(4))) static NvU8 s_UsbManufacturerID[USB_MANF_STRING_LENGTH] = 
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
__attribute__((aligned(4))) static NvU8 s_UsbProductID[USB_PRODUCT_STRING_LENGTH] = 
{
    USB_PRODUCT_STRING_LENGTH, // Length of descriptor
    0x03,                      // STRING descriptor type.
    'A', 0x00,
    'P', 0x00,
    'X', 0x00
};

// Stores the Serial Number String descriptor data (Not used for AP15 Bootrom)
__attribute__((aligned(4))) static NvU8 s_UsbSerialNumber[USB_SERIAL_NUM_LENGTH] =  
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
__attribute__((aligned(4))) static NvU8 s_UsbLanguageID[USB_LANGUAGE_ID_LENGTH] =
{
    /* Language Id string descriptor */
    USB_LANGUAGE_ID_LENGTH,  // Length of descriptor
    0x03,                    // STRING descriptor type.
    0x09, 0x04               // LANGID Code 0: American English 0x409
};

// Stores the Device Qualifier Desriptor data
__attribute__((aligned(4))) static NvU8 s_UsbDeviceQualifier[USB_DEV_QUALIFIER_LENGTH] = 
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
    0x00, //Number of Other-speed Configurations
    0x00  // Reserved for future use, must be zero
};


// Stores the Device Status descriptor data
__attribute__((aligned(4))) static NvU8 s_UsbDeviceStatus[USB_DEV_STATUS_LENGTH] = 
{
    USB_DEVICE_SELF_POWERED,
    0,
};

static XUSBDeviceContext_T XUSBDeviceContext;

NvBootError NvBootXusbDeviceInitializeEventRing(void)
{
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    NvU32 RegData;
    /* zero out event ring */
    memset((void*)&EventRing[0], 0, NUM_TRB_EVENT_RING*sizeof(EventTRB_T));
    //memset((void*)0x80000000, 0, NUM_TRB_EVENT_RING*sizeof(EventTRB_T));
    /* Set event ring segment 0 and segment 1 */
    /* Segment 0 */
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_ERST0BALO_0);
    RegData |= (NvU32)&EventRing[0];
    //RegData = 0x80000000;
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST0BALO_0, RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST0BAHI_0, 0);

    /* Segment 1 */
    
     NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST1BALO_0, (NvU32)&EventRing[NUM_TRB_EVENT_RING/2]);
    // RegData = 0x80000100;
    // NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST1BALO_0, RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERST1BAHI_0, 0);

    /* Write segment sizes */
    RegData = NV_DRF_NUM(XUSB_DEV_XHCI, ERSTSZ, ERST0SZ, NUM_TRB_EVENT_RING/2) |
              NV_DRF_NUM(XUSB_DEV_XHCI, ERSTSZ, ERST1SZ, NUM_TRB_EVENT_RING/2);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERSTSZ_0, RegData);
    
    /* Initialize Enqueue and Dequeue pointer of consumer context*/
    pXUSBDeviceContext->EventDequeuePtr =
    pXUSBDeviceContext->EventEnqueuePtr = 
    (NvU32)&EventRing[0];
    //(NvU32)0x80000000;
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
                                 pXUSBDeviceContext->EventEnqueuePtr >> 4,
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
                                 pXUSBDeviceContext->EventDequeuePtr >> 4,
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERDPLO_0, RegData);
    
    // Clear bits 63:32 of Dequeue pointer.
    RegData = 0;
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERDPHI_0, RegData);

    return NvBootError_Success;
}

NvBootError NvBootXusbDeviceInitializeTransferRing(Endpoint_T EpIndex)
{
    /* zero out tx ring */
    memset((void*)&TxRingEp0[0], 0, NUM_TRB_TRANSFER_RING*sizeof(EventTRB_T));
    return NvBootError_Success;
}
static NvBootError NvBootPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout)
{
    NvU32 RegData;
    do {
    RegData = NV_READ32(RegAddr);
    if((RegData & Mask) == ExpectedValue)
        return NvBootError_Success;
    NvBootUtilWaitUS(1);
    Timeout--;
    } while(Timeout);
    return NvBootError_HwTimeOut;
}
NvBootError NvBootXusbDevicePollForEvent(NvU32 Timeout)
{
    NvBootError e = NvBootError_Success;
    NvU32 RegData, ExpectedValue, IntPendingMask;
    volatile EventTRB_T *pEventTRB;
    volatile SetupEventTRB_T *pSetupEventTRB;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext = &XUSBDeviceContext;
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

    pXUSBDeviceContext->EventEnqueuePtr = RegData & SHIFTMASK(XUSB_DEV_XHCI,
                                                              EREPLO,
                                                              ADDRLO);

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

            // Check if we are waiting for setup packet
            
            pSetupEventTRB = (SetupEventTRB_T*)pEventTRB;
                memcpy((void*)&SetupData[0],
                   (void*)&pSetupEventTRB->Data[0],
                    8);
                pXUSBDeviceContext->CntrlSeqNum = pSetupEventTRB->CtrlSeqNum;
                e = NvBootXusbDeviceHandleSetupPacket(&SetupData[0]);
        }
        else if(pEventTRB->TRBType == PORT_STATUS_CHANGE_TRB)
        {
            // Handle all port status changes here.
            e = NvBootXusbDeviceHandlePortStatusChange();
        }
        else if(pEventTRB->TRBType == TRANSFER_EVENT_TRB)
        {  
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
                                 pXUSBDeviceContext->EventDequeuePtr >> 4,
                                 RegData);

    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ERDPLO_0, RegData);
    return e;   
}
NvBootError NvBootXusbDeviceSetConfiguration(NvU8* pSetupData)
{
    NvU16 wValue;
    NvU32 RegData;
    //NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // Last stage of enumeration.
    wValue = pSetupData[USB_SETUP_VALUE] + (pSetupData[USB_SETUP_VALUE+1] << 8);
    if(wValue == 0)
    {
    // TODO set address to 0.
    // RESUME WORK
    }
    else
    {
        NvBootXusbDeviceInitEndpoint(EP1_OUT);
        NvBootXusbDeviceInitEndpoint(EP1_IN);
        // Now set run
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0);
        RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                     CTRL,
                                     RUN,
                                     RUN,   
                                     RegData);
        RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);
        // Also clear Run Change bit just in case to enable Doorbell register.
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_ST_0);
        RegData = NV_FLD_SET_DRF_DEF(XUSB_DEV_XHCI,
                                     ST,
                                     RC,
                                     CLEAR,
                                     RegData);
        RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ST_0, RegData);
        // Send status
        NvBootXusbDeviceIssueStatusTRB(DIR_IN);
        pXUSBDeviceContext->ConfigurationNum = wValue;
        pXUSBDeviceContext->DeviceState = CONFIGURED_STATUS_PENDING;
    }
    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceSetAddress(NvU8* pSetupData)
{
    NvU8 bAddr;
    NvU32 RegData;
    EpContext_T *pEpContext;
    //NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    
    bAddr = pSetupData[USB_SETUP_VALUE];
    pEpContext = &EpContext[EP0_IN];
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     CTRL,
                                     DEVADR,
                                     bAddr,
                                     RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);

    pEpContext->DeviceAddr = bAddr;
    // Send status
    NvBootXusbDeviceIssueStatusTRB(DIR_IN);
    pXUSBDeviceContext->DeviceState = ADDRESSED_STATUS_PENDING;
    return NvBootError_Success;
}

static void NvBootXusbDeviceSetPidRev(NvU8 *pDevDescriptor)
{
    NvU32 RegData = 0;
    NvU32 Sku;

    // Read the Chip ID revision register
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_GP_HIDREV_0);

    // Product ID definition (2 bytes as per USB 2.0 Spec)
    // idProduct(LSB - 8bits) = Chip ID
    // idProduct(MSB - 8bits) = Family(4Bits:Bit4-Bit7) and Sku(4Bits:Bit0-Bit3)
    // idProduct(LSB) - Read chip ID from the chip ID register
    pDevDescriptor[10] = NV_DRF_VAL(APB_MISC, GP_HIDREV, CHIPID, RegData);
    // idProduct(MSB) - Read family and SKu value and define the MSB
    NvBootFuseGetSkuRaw(&Sku);
#define NVBOOT_USBF_DESCRIPTOR_SKU_MASK  0xF  // descriptor gets only 4 low bits
    pDevDescriptor[11] = (NV_DRF_VAL(APB_MISC, GP_HIDREV, HIDFAM, RegData) << 4) |
        (Sku & NVBOOT_USBF_DESCRIPTOR_SKU_MASK);
    // Device Release number definition (2 bytes as per USB 2.0 Spec)
    // bcd Device (LSB - 8bits) = (Bit4-Bit7 = 0) (Bit0-Bit3 = Minor Rev)
    // bcd Device (MSB - 8bits) = (Bit4-Bit7 = 0) (Bit0-Bit3 = Major Rev)
    // bcd Device (LSB) - Read minor revision
    pDevDescriptor[12] = NV_DRF_VAL(APB_MISC, GP_HIDREV, MINORREV, RegData);
    // bcd Device (MSB) - Read Major revision
    pDevDescriptor[13] = NV_DRF_VAL(APB_MISC, GP_HIDREV, MAJORREV, RegData);
}

NvBootError NvBootXusbDeviceGetDescriptor(NvU8* pSetupData)
{
    
    NvU8 bDescType, bDescIndex= 0;
    NvU16 wLength;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    NvBootError e = NvBootError_Success;

    bDescType = pSetupData[USB_SETUP_DESCRIPTOR];
    wLength = *(NvU16*)&(pSetupData[USB_SETUP_LENGTH]);

    switch(bDescType)
    {
        case USB_DT_DEVICE:
#if XUSB_DEV_SS_SUPPORTED
             if(pXUSBDeviceContext->PortSpeed != XUSB_SUPER_SPEED)
             {
                s_UsbDeviceDescriptor[2] = 0; // bcd USB LSB
                s_UsbDeviceDescriptor[3] = 2; // bcd USB MSB
                s_UsbDeviceDescriptor[7] = 64;
             }
#endif
             NvBootXusbDeviceSetPidRev(&s_UsbDeviceDescriptor[0]);
             e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbDeviceDescriptor[0],
                         NV_MIN(wLength, sizeof(s_UsbDeviceDescriptor)),
                         DIR_IN);
            break;

        case USB_DT_CONFIG:
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
            e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbConfigDescriptor[0],
                         NV_MIN(wLength, sizeof(s_UsbConfigDescriptor)),
                         DIR_IN);
            break;

        case USB_DT_STRING:
            bDescIndex = pSetupData[USB_SETUP_VALUE];
            switch (bDescIndex)
            {
                case USB_MANF_ID: //Manufacture ID
                    e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbManufacturerID[0],
                         NV_MIN(wLength, sizeof(s_UsbManufacturerID)),
                         DIR_IN);
                    break;
                case USB_PROD_ID:    // Product ID
                    e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbProductID[0],
                         NV_MIN(wLength, sizeof(s_UsbProductID)),
                         DIR_IN);
                    break;
                case USB_SERIAL_ID:    // Serial Number
                    e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbSerialNumber[0],
                         NV_MIN(wLength, sizeof(s_UsbSerialNumber)),
                         DIR_IN);
                    break;
                case USB_LANGUAGE_ID:    //Language ID
                //Default case return Language ID
                default: //Language ID
                    e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbLanguageID[0],
                         NV_MIN(wLength, sizeof(s_UsbLanguageID)),
                         DIR_IN);
                    break;
            }
            break;

        case USB_DT_DEVICE_QUALIFIER:
            e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbDeviceQualifier[0],
                         NV_MIN(wLength, sizeof(s_UsbDeviceQualifier)),
                         DIR_IN);
            break;
        case USB_DT_OTHER_SPEED_CONFIG:
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
            e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_OtherSpeedConfigDesc[0],
                         NV_MIN(wLength, sizeof(s_OtherSpeedConfigDesc)),
                         DIR_IN);
            break;
#if XUSB_DEV_SS_SUPPORTED
        case USB_DT_BOS:
             e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbBOSDescriptor[0],
                         NV_MIN(wLength, sizeof(s_UsbBOSDescriptor)),
                         DIR_IN);
            break;
#endif
        default:
            // Stall if any Un supported request comes
             NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
            break;
    }
    return e;
}
NvBootError NvBootXusbDeviceEPGetStatus(NvU8* pSetupData)
{
    NvU16 wValue;
    EpContext_T *pEp0Context;
    NvU32 EpStatus;
    NvBootError e;
    // Get the Ep number from 'wIndex' field and  get the 
    // Halt status and update the data buffer. Tempararily 
    // sending 0 ( not haltted for all ep).
    // Read the Ep status, If ep is STALLED, return 1 other 
    // wise 0
    // Get once the EP status, to find wether Txfer is success or not
    wValue = ((pSetupData[USB_SETUP_INDEX]) |
                     (pSetupData[USB_SETUP_INDEX + 1] << 8));
    pEp0Context = 
    (EpContext_T*)(NV_READ32(XUSB_BASE+XUSB_DEV_XHCI_ECPLO_0) & ~0xF);

    pEp0Context = &pEp0Context[(Endpoint_T)wValue];
    EpStatus = pEp0Context->EpState;
    // if Halted
    if (EpStatus == 2)
        EndpointStatus[0] = 1;
    else
        EndpointStatus[0] = 0;

    e = NvBootXusbDeviceIssueDataTRB((NvU32)&EndpointStatus[0], 
                                     sizeof(EndpointStatus),
                                     DIR_IN);
    return e;
}
NvBootError
NvBootXusbDeviceIssueNormalTRB(NvU32 Buffer, NvU32 Bytes, NvU32 Direction)
{
    NormalTRB_T NormalTRB;
    NvBootError e;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;
    Endpoint_T EpIndex;

    memset((void*)&NormalTRB, 0, sizeof(NormalTRB_T));
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

    memset((void*)&DataTRB, 0, sizeof(DataTRB_T));
    e = NvBootXusbDeviceCreateDataTRB(&DataTRB, Buffer, Bytes, Direction);
    if(e != NvBootError_Success)
        return e;
    // Note EP0_IN is bi-directional.
    e = NvBootXusbDeviceQueueTRB(EP0_IN, (NormalTRB_T*)&DataTRB, 1);
    if(e != NvBootError_Success)
        return e;

    // e = NvBootXusbDeviceIssueStatusTRB(DIR_OUT);
    // if(e != NvBootError_Success)
        // return e;
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
    memset((void*)&StatusTRB, 0, sizeof(StatusTRB_T));
    e = NvBootXusbDeviceCreateStatusTRB(&StatusTRB, Direction);
    if(e != NvBootError_Success)
        return e;
    // Note EP0_IN is bi-directional.
    
    e = NvBootXusbDeviceQueueTRB(EP0_IN, (NormalTRB_T*)&StatusTRB, 1);
    
    pXUSBDeviceContext->WaitForEvent = STATUS_STAGE_TRB;
    return e;
}
NvBootError NvBootXusbDeviceStallEndpoint(Endpoint_T EpIndex, NvU32 Stall)
{
    NvU32 RegData;
    if(Stall)
    {
        RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                             EP_HALT,
                             DCI,
                             1 << EpIndex);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);
    }
    else
    {
        RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                             EP_HALT,
                             DCI,
                             0 << EpIndex);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);
    }
    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceHandleSetupPacket(NvU8* pSetupData)
{
    NvU32 RegData, IntPendingMask, ExpectedValue;
    //NvU16 wLength;
    /* Wait for event to be posted */
    NvBootError e;
    Endpoint_T EpIndex;
    //wLength = *(NvU16*)&pSetupData[USB_SETUP_LENGTH];
    /*********************************/
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0);
    RegData &= ~(1<< EP0_IN);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);

    ExpectedValue = 0;
    IntPendingMask = 1;
    // TODO: do we wait or poll till we die. Polling for 1ms.
    // Poll for interrupt pending bit.
    e = NvBootPollField((XUSB_BASE+XUSB_DEV_XHCI_EP_HALT_0),
                        IntPendingMask,
                        ExpectedValue,
                        1000);
    if(e != NvBootError_Success) 
        return e;
    /************************************************/
                    
    switch(pSetupData[USB_SETUP_REQUEST_TYPE])
    {
        case HOST2DEV_DEVICE:
            // Process the Host -> device Device descriptor
            switch(pSetupData[USB_SETUP_REQUEST])
            {
                case SET_CONFIGURATION:
                    e= NvBootXusbDeviceSetConfiguration(pSetupData);
                    if(e!=NvBootError_Success)
                        return e;
                break;
                case SET_ADDRESS:
                    e= NvBootXusbDeviceSetAddress(pSetupData);
                    if(e!=NvBootError_Success)
                        return e;
                break;
            }
            //TODO
            break;

        case HOST2DEV_INTERFACE:
            // Start the endpoint for zero packet acknowledgment
            //TODO
            break;

        case DEV2HOST_DEVICE:
            switch(pSetupData[USB_SETUP_REQUEST])
            {
                case GET_STATUS:
                    e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbDeviceStatus[0],
                         sizeof(s_UsbDeviceStatus),
                         DIR_IN);
                    if(e != NvBootError_Success)
                        return e;
                    break;
                case GET_CONFIGURATION:
                    e = NvBootXusbDeviceIssueDataTRB(
                        (NvU32)&s_UsbConfigDescriptor[0],
                         sizeof(s_UsbConfigDescriptor), 
                         DIR_IN);
                    if(e != NvBootError_Success)
                        return e;
                    break;
                case GET_DESCRIPTOR:
                    // Get Descriptor Request
                    // TODO Enact Stall protocol on invalid requests.
                    e =  NvBootXusbDeviceGetDescriptor(pSetupData);
                    
                    if(e != NvBootError_Success)
                        return e;
                    break;
                default:
                    // Stall if any Un supported request comes
                    //TODO STALL
                    NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;

        case DEV2HOST_INTERFACE:
            switch (pSetupData[USB_SETUP_REQUEST])
            {
                case GET_STATUS: // Get Status/Interface Request
                    // Just sending 0s.
                    e = NvBootXusbDeviceIssueDataTRB((NvU32)&InterfaceStatus[0],
                                     sizeof(InterfaceStatus), DIR_IN);
                    if(e != NvBootError_Success)
                        return e;
                    break;
                case GET_INTERFACE: // Get Interface Request
                    // Just sending 0s.
                    // TODO: NOT USUALLY Supported so..lets decide on best course of action.
                    
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
                    e = NvBootXusbDeviceEPGetStatus(pSetupData);
                    if(e != NvBootError_Success)
                        return e;
                    break;
                default:
                    NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;

        case HOST2DEV_ENDPOINT:
            switch (pSetupData[USB_SETUP_REQUEST])
            {
                case SET_FEATURE:
                    switch (pSetupData[USB_SETUP_VALUE])
                    {
                        case ENDPOINT_HALT:
                            EpIndex = (Endpoint_T)(pSetupData[USB_SETUP_INDEX] |
                                       (pSetupData[USB_SETUP_INDEX + 1] << 8));
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
                    switch (pSetupData[USB_SETUP_VALUE])
                    {
                        case ENDPOINT_HALT:
                                 // Get once the EP status, to find wether Txfer is success or not
                            EpIndex = (Endpoint_T)(pSetupData[USB_SETUP_INDEX] |
                                       pSetupData[USB_SETUP_INDEX + 1] << 8);
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
                    // Stall if any unsupported request comes
                   NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
                    break;
            }
            break;
        default:
            // Stall if any Un supported request comes
            NvBootXusbDeviceStallEndpoint(EP0_IN, NV_TRUE);
            break;
    }
    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceInitEndpoint(Endpoint_T EpIndex)
{
    NvU32 ExpectedValue, Mask, RegData;
    NvBootError e = NvBootError_Success;

    if(EpIndex == EP0_IN || EpIndex == EP0_OUT)
    {
        NvBooXusbDeviceInitEpContext(EP0_IN);
    }
    else if(EpIndex == EP1_IN || EpIndex == EP1_OUT)
    {
        NvBooXusbDeviceInitEpContext(EpIndex);
        // Bit 2 for EP1_OUT , Bit 3 for EP1_IN
        // Force load context
        // Steps from device_mode IAS, 5.1.3.1
        RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                             EP_RELOAD,
                             DCI,
                             1 << EpIndex);
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_RELOAD_0, RegData);
        // TODO Timeout for polling
        Mask = 1 << EpIndex;
        ExpectedValue = 0;
        e = NvBootPollField(XUSB_BASE+XUSB_DEV_XHCI_EP_RELOAD_0,
                            Mask,
                            ExpectedValue,
                            1000);
        if(e!=NvBootError_Success)
            return e;
        // Make sure ep is not paused or halted.
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0);
        RegData &= ~(1 << EpIndex);
        RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0, RegData);
        RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0);
        RegData &= ~(1 << EpIndex);
        RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);
        
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
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo<<4);
        }
        // TODO Add check for full ring
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
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo<<4);
        }
        // TODO Add check for full ring
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
            pNextTRB = (DataTRB_T*)(pLinkTRB->RingSegPtrLo<<4);
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
        // TODO:
        // This could mean 2 things.
        // 1.   The seq number in the data/status stage did not match the 
        //      setup event seq.
        // 2. A new setup packet was received when sending data/status packet.

        // We have a setup packet to process
        // Setuppacketindex always points to next slot. pop out last setup packet.
        // SetupPacketIndex = pXUSBDeviceContext->SetupPacketIndex-1;
        // pSetupData = &SetupPacketQueue[SetupPacketIndex*8];
        // e = NvBootXusbDeviceHandleSetupPacket(pSetupData);
        // TODO DO we need to handle this at Bootrom? If so, use queue concept above
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
    memset((void*)pEpContext, 0, sizeof(EpContext_T));
    // Set Endpoint State to running.
#define EP_RUNNING  1
    pEpContext->EpState = EP_RUNNING;
    // Set error count to 3
    pEpContext->CErr = 3;
    // Set Burst size 0
    pEpContext->MaxBurstSize = 0;
    // Set Packet size as 64 bytes. USB 2.0
    // TODO: Change to 512 if supporting USB 3.0
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

    // Control Endpoint 0.
    if(EpIndex == EP0_IN)
    {
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
        pEpContext->TRDequeuePtrLo = pXUSBDeviceContext->CntrlEpDequeuePtr >> 4;
        pEpContext->TRDequeuePtrHi = 0;

        // Setup Link TRB. Last TRB of ring.
        // FIXME
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
            pEpContext->TRDequeuePtrLo = pXUSBDeviceContext->BulkOutDequeuePtr >> 4;
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
            // Initialize Producer Cycle State to 1.
            pXUSBDeviceContext->BulkInPCS  = 1;

            // SW copy of Dequeue pointer for control endpoint.
            pXUSBDeviceContext->BulkInDequeuePtr =
            pXUSBDeviceContext->BulkInEnqueuePtr = (NvU32)&TxRingEp1In[0];

            // EP specific Context
            // Set endpoint type to Bulk.
#define EP_TYPE_BULK_IN 6
            pEpContext->EpType = EP_TYPE_BULK_IN;
            pEpContext->TRDequeuePtrLo = pXUSBDeviceContext->BulkInDequeuePtr>>4;
            pEpContext->TRDequeuePtrHi = 0;
#if XUSB_DEV_SS_SUPPORTED
            if(pXUSBDeviceContext->PortSpeed == XUSB_SUPER_SPEED)
            {
                pEpContext->AvgTRBLen = 1024;
                pEpContext->MaxPacketSize = 1024;
            }
            else // All other cases, use HS.
#endif
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
    NvU32 RegData;
    NvBootError e =  NvBootError_Success;
    
    // If Control EP
    if(EpIndex == EP0_IN)
    {
        memcpy((void*)pXUSBDeviceContext->CntrlEpEnqueuePtr,
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
        // TODO Add check for full ring
        pXUSBDeviceContext->CntrlEpEnqueuePtr = (NvU32)pNextTRB;

    }
    // Bulk Endpoint
    else if(EpIndex == EP1_OUT)
    {
        memcpy((void*)pXUSBDeviceContext->BulkOutEnqueuePtr,
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
        memcpy((void*)pXUSBDeviceContext->BulkInEnqueuePtr,
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
        // RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_DB_0);
        // RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                     // DB,
                                     // TARGET,
                                     // EpIndex,
                                     // RegData);
        // if(EpIndex ==  EP0_IN)
        // {
            // RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         // DB,
                                         // STREAMID,
                                         // pXUSBDeviceContext->CntrlSeqNum,
                                         // RegData);
        // }
        // NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_DB_0, RegData);
        RegData = NV_DRF_NUM(XUSB_DEV_XHCI,
                                     DB,
                                     TARGET,
                                     EpIndex);
        if(EpIndex ==  EP0_IN)
        {
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         DB,
                                         STREAMID,
                                         pXUSBDeviceContext->CntrlSeqNum,
                                         RegData);
        }
        NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_DB_0, RegData);
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

NvBootError NvBootXusbDeviceHandlePortStatusChange(void)
{
    NvBootError e = NvBootError_Success;
    NvU32 Mask, ExpectedValue, PortSpeed, RegData, RegHalt;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // Let's see why we got here.
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
    RegHalt = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);

    // Connect Status Change and Current Connect Status should be 1
    // to indicate successful connection to downstream port.
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, CSC, RegData)) 
        // || V_DRF_VAL(XUSB_DEV_XHCI, PORTSC, CCS, RegData))
    {
            //RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            PortSpeed = NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PS, RegData);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         CSC,
                                         1,
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, RegData);
            
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
                // TODO Timeout for polling
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
                RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0, RegData);
                RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0);
                RegData &= ~(1 << EP0_IN);
                RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_HALT_0, RegData);
            }
#endif
    }
    // Handle PORT RESET. PR indicates reset event received.
    // PR could get cleared (port reset complete) by the time we read it
    // so check on PRC.
    // See device mode IAS 5.1.4.5
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PR, RegData) ||
            NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PRC, RegData))
    {
            // This is probably a good time to stop the watchdog timer.
            NvBootWdtStop();
            // Wait for Port reset to be complete and Port enable to be set
            Mask = SHIFTMASK(XUSB_DEV_XHCI, PORTSC, PRC);
            ExpectedValue = 1 << SHIFT(XUSB_DEV_XHCI, PORTSC, PRC);
            e = NvBootPollField(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0,
                            Mask,
                            ExpectedValue,
                            50000);// Time for port reset to be complete = 50ms
            if(e != NvBootError_Success)
                return NvBootError_Success;

            // Must clear PRC
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         PRC,
                                         1,
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, RegData);

    }
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, WPR, RegData) ||
            NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, WRC, RegData))
    {
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTHALT,
                                         HALT_LTSSM,
                                         0,   
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0, RegData);            
            // Wait for Warm Port reset to be complete
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            Mask = SHIFTMASK(XUSB_DEV_XHCI, PORTSC, WRC);
            ExpectedValue = 1 << SHIFT(XUSB_DEV_XHCI, PORTSC, WRC);
            e = NvBootPollField(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0,
                            Mask,
                            ExpectedValue,
                            1000);

            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            //*(volatile unsigned int*)0x80000000=RegData;
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         WRC,
                                         1,
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, RegData);

            //e = NvBootError_Success;
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
    // Port Link Status Change
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, PLC, RegData))
    {
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         PLC,
                                         1,
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, RegData);
    }
    // Config Error Change
    if(NV_DRF_VAL(XUSB_DEV_XHCI, PORTSC, CEC, RegData))
    {
            RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0);
            RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                         PORTSC,
                                         CEC,
                                         1,
                                         RegData);
            NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTSC_0, RegData);
            e = NvBootError_XusbPortError;
    }
    
    return e;
}


NvBootError NvBootXusbDeviceInit(void)
{
    NvBootError e;
    NvU32 RegData;
    //NvBootClocksOscFreq OscFreq;
    //OscFreq = NvBootClocksGetOscFreq();

    //NvBootUartInit(OscFreq);

    // Enable clock to XUSB Unit
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_OUT_ENB_W,
                                 CLK_ENB_XUSB,
                                 ENABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0, RegData);

    // Take XUSB Pad Control out of reset.
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_RST_DEVICES_W_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEVICES_W,
                                 SWR_XUSB_PADCTL_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               CLK_RST_CONTROLLER_RST_DEVICES_W_0,RegData);

    NvBootUtilWaitUS(2) ;
    // Setup OTG Pad and BIAS Pad ownership to XUSB
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PAD_MUX_0);
                
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT0, XUSB, RegData);
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_BIAS_PAD, XUSB, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PAD_MUX_0, RegData);

    // Setup PLLU, PLLEs and PADs.
    e = NvBootXusbDevicePadSetup();
    if(e != NvBootError_Success)
        return e;

#if XUSB_DEV_SS_SUPPORTED
    e = NvBootXusbDeviceSSPadSetup();
    if(e != NvBootError_Success)
        return e;
#endif
    // Set port cap to device
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PORT_CAP_0);
                
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PORT_CAP, PORT0_CAP, DEVICE_ONLY, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_PORT_CAP_0, RegData);
    
    // Map ss port to hs port. This step is required even for HS.
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_SS_PORT_MAP_0);
                
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, SS_PORT_MAP, PORT0_MAP, USB2_PORT0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_SS_PORT_MAP_0, RegData);

#if XUSB_DEV_SS_SUPPORTED
    // Initialize Controller Clocks and take XUSB partition out of reset.

    // SS static port params.
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_2_0);
                
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_USB3_PAD0_CTL_2, RX_WANDER, 0xF, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_USB3_PAD0_CTL_2, RX_EQ, 0xF070, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_USB3_PAD0_CTL_2, CDR_CNTL, 0x26, RegData);
      

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_2_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_4_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_USB3_PAD0_CTL_4, DFE_CNTL, 0x002008EE, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_4_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_2_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_MISC_PAD_P0_CTL_2, SPARE_IN, 0x1, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_2_0, RegData);
    
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB3_PAD_MUX_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB3_PAD_MUX, FORCE_PCIE_PAD_IDDQ_DISABLE_MASK0, 0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB3_PAD_MUX, PCIE_PAD_LANE0, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB3_PAD_MUX_0, RegData);

#endif
    // Remove Power Down for VBUS detectors from PMC 
    // registers.
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_PMC_BASE + APBDEV_PMC_USB_AO_0);

    // Remove power down on VBUS WAKEUP PD
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, 
                    USB_AO,
                    VBUS_WAKEUP_PD_P0,
                    0x0,
                    RegData);

    // Remove Power Down ID Wake up for UTMIP P0 to overcome the pad modelling issue
    RegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC, 
                    USB_AO,
                    ID_PD_P0,
                    0x0,
                    RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_PMC_BASE + APBDEV_PMC_USB_AO_0, RegData);

    NvBootUtilWaitUS(1);


    e = NvBootXusbDeviceClkInit();

    // Initialize Event ring
    NvBootXusbDeviceInitializeEventRing();
    NvBootXusbDeviceInitializeTransferRing(EP0_IN);

    
    // Initialize EP0
    e = NvBootXusbDeviceInitEndpoint(EP0_IN);
    if( e != NvBootError_Success)
        return e;

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
    RegData = NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, RegData);

    // Initialize EndPoint Context
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ECPLO_0, (NvU32)&EpContext[0]);

#if XUSB_DEV_SS_SUPPORTED
    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 PORTHALT,
                                 STCHG_INTR_EN,
                                 1,   
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0, RegData);
#endif

    return e;
}

NvBootError NvBootXusbDeviceClkInit(void)
{
    NvU32 RegData;
    
    // Needed for HSIC_480 branch.
    // Register: CLK_RST_CONTROLLER_PLLU_OUTA_0    Field: pllu_out1_rstn
	//@ 0x600060c4
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_PLLU_OUTA_0);
    RegData |= NV_DRF_DEF(CLK_RST_CONTROLLER, PLLU_OUTA, PLLU_OUT1_RSTN, RESET_DISABLE);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_PLLU_OUTA_0, RegData);
    // Wait 2 us before using FO_48M.
    NvBootUtilWaitUS(2);

    // Enable clock to device controller.
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_OUT_ENB_U,
                                 CLK_ENB_XUSB_DEV,
                                 ENABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0,RegData);

    // Set clk source of XUSB dev to PLLP_OUT0 divided down to 102 Mhz. (408/4)



    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_DEV_0);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 // CLK_SOURCE_XUSB_CORE_DEV,
                                 // XUSB_CORE_DEV_CLK_SRC,
                                 // 5,
                                 // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 // CLK_SOURCE_XUSB_CORE_DEV,
                                 // XUSB_CORE_DEV_CLK_DIVISOR,
                                 // 10,
                                 // RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_XUSB_CORE_DEV,
                                 XUSB_CORE_DEV_CLK_SRC,
                                 PLLP_OUT0,
                                 RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_XUSB_CORE_DEV,
                                 XUSB_CORE_DEV_CLK_DIVISOR,
                                 6,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + 
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_DEV_0, RegData);

    NvBootUtilWaitUS(2);

    // Enable clock to FS.

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FS_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_SOURCE_XUSB_FS,
                                 XUSB_FS_CLK_SRC,
                                 FO_48M,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
        CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FS_0,RegData);

    // // Enable Host clock

    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        // CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0);
    // RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 // CLK_OUT_ENB_U,
                                 // CLK_ENB_XUSB_HOST,
                                 // ENABLE,
                                 // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               // CLK_RST_CONTROLLER_CLK_OUT_ENB_U_0,RegData);

    // // Set clk source of XUSB host to PLLP_OUT0 @102 Mhz.
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        // CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_HOST_0);

    // // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 // // CLK_SOURCE_XUSB_CORE_HOST,
                                 // // XUSB_CORE_HOST_CLK_SRC,
                                 // // 5,
                                 // // RegData);
    // // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 // // CLK_SOURCE_XUSB_CORE_HOST,
                                 // // XUSB_CORE_HOST_CLK_DIVISOR,
                                 // // 10,
                                 // // RegData);
    // RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 // CLK_SOURCE_XUSB_CORE_HOST,
                                 // XUSB_CORE_HOST_CLK_SRC,
                                 // PLLP_OUT0,
                                 // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 // CLK_SOURCE_XUSB_CORE_HOST,
                                 // XUSB_CORE_HOST_CLK_DIVISOR,
                                 // 6,
                                 // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + 
               // CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_HOST_0, RegData);

    // NvBootUtilWaitUS(2);

    // Enable clock to XUSB_SS (@120 Mhz)
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 CLK_OUT_ENB_W,
                                 CLK_ENB_XUSB_SS,
                                 ENABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               CLK_RST_CONTROLLER_CLK_OUT_ENB_W_0,RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_SS, XUSB_SS_CLK_DIVISOR, 0x6, RegData);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_SS, XUSB_SS_CLK_SRC, HSIC_480, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS_0, RegData);

    // Take Dev, SS out of reset
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_RST_DEVICES_W_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEVICES_W,
                                 SWR_XUSB_SS_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               CLK_RST_CONTROLLER_RST_DEVICES_W_0, RegData);

    // // Take host controller out of reset
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        // CLK_RST_CONTROLLER_RST_DEVICES_U_0);
    // RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 // RST_DEVICES_U,
                                 // SWR_XUSB_HOST_RST,
                                 // DISABLE,
                                 // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               // CLK_RST_CONTROLLER_RST_DEVICES_U_0, RegData);

    // Take device controller out of reset
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_RST_DEVICES_U_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                                 RST_DEVICES_U,
                                 SWR_XUSB_DEV_RST,
                                 DISABLE,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               CLK_RST_CONTROLLER_RST_DEVICES_U_0, RegData);
   
    NvBootUtilWaitUS(2) ;
    NvBootArcEnable();
    
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

    // Mask Interrupts.
    RegData = NV_READ32(NV_XUSB_DEV_IPFS_REGS +
                        XUSB_DEV_INTR_MASK_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV,
                                 INTR_MASK,
                                 IP_INT_MASK,
                                 1,
                                 RegData);
    NV_WRITE32(NV_XUSB_DEV_IPFS_REGS +
                        XUSB_DEV_INTR_MASK_0, RegData);

    // *(volatile unsigned int*)0x700D9180=0x80068E01;
    // *(volatile unsigned int*)0x700D8004=0x7;
    // *(volatile unsigned int*)0x700D8010=0x700D0000;
    // *(volatile unsigned int*)0x700D9188=0x10000;

    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceEnumerate(NvU8* Buffer)
{
    NvBootError e;
    NvU32 RegData;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    // Set ELPG=0
    RegData = 0;
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_ELPG_PROGRAM_0_0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_ELPG_PROGRAM_1_0, RegData);

    // Set sw override to vbus
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_VBUS_ID_0);
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_VBUS_ID, VBUS_SOURCE_SELECT, VBUS_OVERRIDE, RegData);
    RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_VBUS_ID, ID_SOURCE_SELECT, ID_OVERRIDE, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_VBUS_ID_0, RegData);

    RegData = NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0);
    RegData = NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI,
                                 PORTHALT,
                                 HALT_LTSSM,
                                 0,   
                                 RegData);
    NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_PORTHALT_0, RegData);

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
    // Set sw override to vbus
    // ID sourced through VPIO is 0 which indicates otg_host.
    // Override ID bit alone to 1. Bug 1383185
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_VBUS_ID_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_VBUS_ID, VBUS_OVERRIDE, 1, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_VBUS_ID, ID_OVERRIDE, (1 << 3), RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_VBUS_ID_0, RegData);

    // // FIXME Hacks to make it work for FS on T124
    // NV_WRITE32 (0x700d012C,0x573fd);
	// NV_WRITE32 (0x700d0108,0x927c0);
	// NV_WRITE32 (0x700d010C,0x1);

    // // FPGA HACK. FIXME
// #define NV_PROJ__XUSB_DEV_EMU_CYA 0x700d8200
    // NV_WRITE32(NV_PROJ__XUSB_DEV_EMU_CYA, 0x104000);

    pXUSBDeviceContext->WaitForEvent = SETUP_EVENT_TRB;
    pXUSBDeviceContext->DeviceState = DEFAULT;
    while(pXUSBDeviceContext->DeviceState != CONFIGURED)
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
    NvU32 PlluStableTime = 0, Misc1, Misc2;
    // Configure the USB phy stabilization delay setting.
    NvBootClocksOscFreq OscFreq;
    OscFreq = NvBootClocksGetOscFreq();

    // CPCON and LFCON removed.
    Misc1 = 0;
    Misc2 = 0;

    // Enable PLL U for USB
    NvBootClocksStartPll(NvBootClocksPllId_PllU,
                         s_UsbPllBaseInfo[OscFreq].M,
                         s_UsbPllBaseInfo[OscFreq].N,
                         s_UsbPllBaseInfo[OscFreq].P,
                         Misc1,
                         Misc2,
                         &PlluStableTime);

    NvBootXusbDeviceSetupSWControlUTMIPll(OscFreq);

    NvBootXusbDeviceSetupStaticParamsUTMI();    
    NvBootXusbDeviceRemovePowerDownUTMI();
    NvBootXusbDevicePerformTracking(OscFreq);

    // No way to tell how long to wait. Just wait for a bit.
    NvBootUtilWaitUS(30);
    return e;
}

NvBootError NvBootXusbDevicePerformTracking(NvBootClocksOscFreq OscFreq)
{
    NvU32 RegData;
    /* Perform tracking Bug 1440206 */
    // Enable clock to tracking unit
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
					CLK_OUT_ENB_Y,
					CLK_ENB_USB2_TRK,
					0x1,
					RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0, RegData);

    // The frequency for the tracking circuit should be between 1 to 10 MHz. 
    // osc_clk frequency is between 10 to 20 MHz, the clock divisor should be set to 0x2; 
    // osc_clk frequency is between 20 to 30 MHz, the clock divisor should be set to 0x4; 
    // osc_clk frequency is between 30 to 40 MHz, the clock divisor should be set to 0x6.

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_USB2_HSIC_TRK_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
					CLK_SOURCE_USB2_HSIC_TRK,
					USB2_HSIC_TRK_CLK_DIVISOR,
					s_UsbHsicTrkDiv[OscFreq],
					RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_USB2_HSIC_TRK_0, RegData);

    // Setting TRK_DONE to PD_TRK assertion/TRK_START de-assertion time.
 
    // Uncomment this for si. There is an issue during sims so we are using reset val instead of reading hw reg.
    // RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                // XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0);
    RegData = XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0_RESET_VAL;
    // Setting PD_TRK de-assertion to TRK_START. 30 TRK clock cycles
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, TRK_START_TIMER, 0x1E, RegData);
    // Setting TRK_DONE to PD_TRK assertion. 10 TRK clock cycles
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, TRK_DONE_RESET_TIMER, 0xA, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);

    // Power up tracking circuit.
    // RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                // XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, PD_TRK, 0x0, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);
    

    // // Poll for TRK_DONE to assert. Break if not set by 1ms.
    // PollTrkDoneTime = 1000;
    // while(PollTrkDoneTime)
    // {
        // RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                // XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0);
        // if(NV_DRF_VAL(XUSB_PADCTL,
                      // USB2_BIAS_PAD_CTL_1,
                      // TRK_DONE,
                      // RegData))
            // break;
        // NvBootUtilWaitUS(1);
        // PollTrkDoneTime--;
    // }
    // It was decided to just wait for 100us instead of polling for TRK_DONE
    // as there is a chance COP will miss TRK_DONE assertion.
    NvBootUtilWaitUS(100);

    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, PD_TRK, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);

    // Giving some time for powerdown.(30 trk clk cycles @0.1us)
    NvBootUtilWaitUS(3);
    
    // Bug 200006851. Do another round of tracking.
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, PD_TRK, 0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);
    NvBootUtilWaitUS(100);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_1, PD_TRK, 0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0, RegData);
    // PDTRK should have been asserted by HW State machine after PDTRK_COUNT (10 TRK cycles)
    // But we do it anyway
    
    // Disable clock to tracking unit
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
					CLK_OUT_ENB_Y,
					CLK_ENB_USB2_TRK,
					0x0,
					RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_CLK_OUT_ENB_Y_0, RegData);
    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceRemovePowerDownUTMI(void)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD_ZI, 0, RegData);
    // PD2 deasserted by hw
    // RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD2, 0, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD, 0, RegData);
    // FIXME. Is this field removed?
    //RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, LSBIAS_SEL, 0x0, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0, RegData);

    // PD_DR field in XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0 and 
    // XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0 registers
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, PD_DR, 0, RegData);
    NV_WRITE32( NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                          XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD0_CTL0, PD_CHG, 0, RegData);

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL0_0, RegData);

    //PD fields in XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BIAS_PAD_CTL_0, PD, 0, RegData);    

    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0);
    // This is done by hw now.
    // RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, PD_CHRP_FORCE_POWERUP, 1, RegData);
    // RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, PD_DISC_FORCE_POWERUP, 1, RegData);
    NV_WRITE32( NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                          XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0, RegData);


    return NvBootError_Success;
}
NvBootError NvBootXusbDeviceSetupStaticParamsUTMI(void)
{
    NvU32 RegData, RpdCtrl, HSTermRangeAdj, HSCurrLevel;
   // Read FUSE_USB_CALIB_EXT_0,FUSE_USB_CALIB_0 register to parse RPD_CTRL, 
   // HS_TERM_RANGE_ADJ, and HS_CURR_LEVEL fields and set corresponding fields in 
   // XUSB_PADCTL_USB2_OTG_PAD0_CTL_0/1 registers
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_0);
    HSTermRangeAdj = NV_DRF_VAL(FUSE, USB_CALIB, TERM_RANGE_ADJ, RegData); //HS_TERM_RANGE_ADJ
    HSCurrLevel = NV_DRF_VAL(FUSE, USB_CALIB, HS_CURR_LEVEL, RegData); //HS_CURR_LEVEL
    
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_EXT_0);
    RpdCtrl = NV_DRF_VAL(FUSE, USB_CALIB_EXT, RPD_CTRL, RegData); //RPD_CTRL

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, HS_CURR_LEVEL, HSCurrLevel, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, TERM_RANGE_ADJ, HSTermRangeAdj, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, RPD_CTRL, RpdCtrl, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0, RegData);

    // With 20 nm soc, pads voltage is 1.5v so enable pad protection circuit
    // against 3.3v
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL1_0);
    // VREG_FIX18 = 0 => Internal voltage regulator control,to generate some divided voltage
    // VREG_LEV = 1 => 1.5V
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD0_CTL1, VREG_FIX18, 0, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD0_CTL1, VREG_LEV , 1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL1_0, RegData);

    return NvBootError_Success;
}

/*
 * This function won't be used in Bootrom.
 * HW Control of PLL is for power sequencing.
 * not exploited in Bootrom.
 */
// NvBootError NvBootXusbDeviceSetupHWControlUTMIPll(NvBootClocksOscFreq OscFreq)
// {
    // NvU32 RegData;

    // // The following parameters control the bring up of the plls:
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0);
    
    // /************ Set the Oscillator dependent automatic startup times etc *********/
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, 
                        // UTMIP_PLL_CFG2, 
                        // UTMIP_PLLU_STABLE_COUNT,
                        // s_UsbPllDelayParams[OscFreq].StableCount,
                        // RegData);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                        // UTMIP_PLL_ACTIVE_DLY_COUNT,
                        // s_UsbPllDelayParams[OscFreq].ActiveDelayCount,
                        // RegData);

    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0, RegData);

    // // Set PLL enable delay count and Crystal frequency count ( clock reset domain)
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_PLLU_ENABLE_DLY_COUNT,
                    // s_UsbPllDelayParams[OscFreq].EnableDelayCount,
                    // RegData);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_XTAL_FREQ_COUNT,
                    // s_UsbPllDelayParams[OscFreq].XtalFreqCount,
                    // RegData);
    // /************ End Automatic startup time programming ************/
    
    // /** All force enables should be equal to 0. We are going to setup 
     // ** HW control **************************************************/
    // // power-up for PLLU_ENABLE
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLL_ENABLE_POWERUP,
                    // 0x0,
                    // RegData);

    // // Remove power down for PLLU_ENABLE
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLL_ENABLE_POWERDOWN,
                    // 0x0,
                    // RegData);
    // // Remove power down for PLL_ACTIVE
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLL_ACTIVE_POWERDOWN,
                    // 0x0,
                    // RegData);
    // // Remove power down for PLLU
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLLU_POWERDOWN,
                    // 0x0,
                    // RegData);
    // // Remove power down for PLLU
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLLU_POWERUP,
                    // 0x0,
                    // RegData);
 
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0, RegData);
    // NvBootUtilWaitUS(2);

 // /********* End disabling all force power ups and power downs ********/

// /********** Remove power downs from UTMIP PLL control bits ***********/

    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0);
    // // Remove power down for PLLU_ENABLE
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_A_POWERDOWN,
                    // 0x0,
                    // RegData);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_A_POWERUP,
                    // 0x1,
                    // RegData);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_C_POWERDOWN,
                    // 0x0,
                    // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_C_POWERUP,
                    // 0x1,
                    // RegData);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_B_POWERUP,
                    // 0x1,
                    // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_B_POWERDOWN,
                    // 0x0,
                    // RegData);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_D_POWERUP,
                    // 0x1,
                    // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    // UTMIP_FORCE_PD_SAMP_D_POWERDOWN,
                    // 0x0,
                    // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0, RegData);
    // NvBootUtilWaitUS(2);
    // /********** END Remove power downs from UTMIP PLL control bits ***********/

    // /***** Setting up hw control. Start hw sequencer ********************************/
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIPLL_HW_PWRDN_CFG0, 
                                 // UTMIPLL_USE_LOCKDET,
                                 // 0x1,
                                 // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIPLL_HW_PWRDN_CFG0, 
                                 // UTMIPLL_CLK_ENABLE_SWCTL,
                                 // 0x0,
                                 // RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIPLL_HW_PWRDN_CFG0, 
                                 // UTMIPLL_SEQ_START_STATE,
                                 // 0x1,
                                 // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE +
               // CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0, RegData);
    // /***** End Setting up Hw sequencing **************************************************/
    // /***** Making sure that IDDQ SW override is not enabled **********************/
    // /****** when all non-HSIC USB2.0 ports are assigned to XUSB ******************/
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    // CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);
    // // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // // UTMIPLL_HW_PWRDN_CFG0,
                    // // UTMIPLL_IDDQ_SWCTL,
                    // // 0x1, RegData);
    // // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // // UTMIPLL_HW_PWRDN_CFG0,
                    // // UTMIPLL_IDDQ_OVERRIDE_VALUE,
                    // // 0x0, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // UTMIPLL_HW_PWRDN_CFG0,
                    // UTMIPLL_IDDQ_SWCTL,
                    // 0x0, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // UTMIPLL_HW_PWRDN_CFG0,
                    // UTMIPLL_SEQ_IN_SWCTL,
                    // 0x0, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // UTMIPLL_HW_PWRDN_CFG0,
                    // UTMIPLL_IDDQ_PD_INCLUDE,
                    // 0x0, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            // CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0, RegData);

    // NvBootUtilWaitUS(10);

    // /****** Start music. HW Sequencing enabled ******************/
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    // CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // UTMIPLL_HW_PWRDN_CFG0,
                    // UTMIPLL_SEQ_ENABLE,
                    // 0x1, RegData);

    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            // CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0, RegData);
    // /****** END. HW Sequencing enabled ******************/
    // return NvBootError_Success;
// }

NvBootError NvBootXusbDeviceSetupSWControlUTMIPll(NvBootClocksOscFreq OscFreq)
{
    NvU32 RegData, PlluInputForUtmi=0, LockTime;

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    UTMIPLL_HW_PWRDN_CFG0,
                    UTMIPLL_IDDQ_SWCTL,
                    0x1, RegData);   
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    UTMIPLL_HW_PWRDN_CFG0,
                    UTMIPLL_IDDQ_OVERRIDE_VALUE,
                    0x0, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0, RegData);

    // Set clock source for UTMIP. Default is osc_clk.
    // Based on CYA, choose PLLU as input.
    PlluInputForUtmi = NvBootGetSwCYA() & NVBOOT_SW_CYA_USE_PLLU_SRC_UTMIP;
    if (PlluInputForUtmi)
    {
        RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG3_0);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                        PLL_CFG3, 
                        UTMIP_PLL_REF_SRC_SEL,
                        1, // PLLU
                        RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG3_0, RegData);
        // Configure UTMI PLL dividers.
        // PLLU 480 Mhz VCO output is src for UTMI.
        // Set N=2, M=1 so that UTMI PLL VCO is 960 Mhz.
        RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_NDIV,
                            30,
                            RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_MDIV,
                            15,
                            RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0, RegData);
    }
    else
    {
        // Configure UTMI PLL dividers based on oscillator frequency.
        RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_NDIV,
                            s_UtmiPllBaseInfo[OscFreq].N,
                            RegData);
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP,
                            PLL_CFG0, 
                            UTMIP_PLL_MDIV,
                            s_UtmiPllBaseInfo[OscFreq].M,
                            RegData);
        NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG0_0, RegData);
    }

    // The following parameters control the bring up of the plls:
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0);
    /************ Set the Oscillator dependent automatic startup times etc *********/
    if (PlluInputForUtmi)
    {
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, 
                        PLL_CFG2, 
                        UTMIP_PLLU_STABLE_COUNT,
                        s_UsbPllDelayParams[OscFreq].StableCount,
                        RegData);
    }
    else
    {
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, 
                        PLL_CFG2, 
                        UTMIP_PLLU_STABLE_COUNT,
                        0, // Need not wait for PLLU to be stable if using Osc_Clk as input
                        RegData);
    }

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                        UTMIP_PLL_ACTIVE_DLY_COUNT,
                        s_UsbPllDelayParams[OscFreq].ActiveDelayCount,
                        RegData);
    
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0, RegData);

    // Set PLL enable delay count and Crystal frequency count ( clock reset domain)
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0);

    if(PlluInputForUtmi)
    {
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_PLLU_ENABLE_DLY_COUNT,
                        s_UsbPllDelayParams[OscFreq].EnableDelayCount,
                        RegData);
        // Remove power down for PLLU
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_FORCE_PLLU_POWERDOWN,
                        0x0,
                        RegData);
    }
    else
    {
        RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER_UTMIP, PLL_CFG1, 
                        UTMIP_PLLU_ENABLE_DLY_COUNT,
                        0, // Need not wait for PLLU if input is OSC_CLK
                        RegData);
    }

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    UTMIP_XTAL_FREQ_COUNT,
                    s_UsbPllDelayParams[OscFreq].XtalFreqCount,
                    RegData);
    /************ End Automatic startup time programming ************/

    /********* Disable all force power ups and power downs ********/
    // Power-up for PLLU_ENABLE
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    UTMIP_FORCE_PLL_ENABLE_POWERUP,
                    0x1,
                    RegData);

    // Remove power down for PLLU_ENABLE
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    UTMIP_FORCE_PLL_ENABLE_POWERDOWN,
                    0x0,
                    RegData);
    // Remove power down for PLL_ACTIVE
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    UTMIP_FORCE_PLL_ACTIVE_POWERDOWN,
                    0x0,
                    RegData);
    
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0, RegData);
    //NvBootUtilWaitUS(2);

    /********* End Disabling all force power ups and power downs ********/
    LockTime = 10; // 10 us
    while(LockTime)
    {
        RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_UTMIPLL_HW_PWRDN_CFG0_0);
        if(NV_DRF_VAL(CLK_RST_CONTROLLER,
                      UTMIPLL_HW_PWRDN_CFG0,
                      UTMIPLL_LOCK,
                      RegData))
        break;
        NvBootUtilWaitUS(1);
        LockTime--;
    } 

    /********** Remove power downs from UTMIP PLL Samplers bits ***********/

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0);
    // Remove power down for PLLU_ENABLE
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_A_POWERDOWN,
                    0x0,
                    RegData);

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_A_POWERUP,
                    0x1,
                    RegData);

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_C_POWERDOWN,
                    0x0,
                    RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_C_POWERUP,
                    0x1,
                    RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_B_POWERDOWN,
                    0x0,
                    RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_B_POWERUP,
                    0x1,
                    RegData);

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_D_POWERUP,
                    0x1,
                    RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, 
                    UTMIP_FORCE_PD_SAMP_D_POWERDOWN,
                    0x0,
                    RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG2_0, RegData);
    /********** END Remove power downs from UTMIP PLL control bits ***********/


    NvBootUtilWaitUS(2);

    return NvBootError_Success;
}

/*
 * This function won't be used in Bootrom.
 * HW Control of PLL is for power sequencing.
 * not exploited in Bootrom.
 */
// NvBootError NvBootXusbDeviceSetupHWControlPLLU(void)
// {
    // NvU32 RegData;

    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    // CLK_RST_CONTROLLER_PLLU_HW_PWRDN_CFG0_0);

    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // PLLU_HW_PWRDN_CFG0,
                    // PLLU_USE_LOCKDET,
                    // 0x1, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // PLLU_HW_PWRDN_CFG0,
                    // PLLU_CLK_ENABLE_SWCTL,
                    // 0x0, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // PLLU_HW_PWRDN_CFG0,
                    // PLLU_SEQ_START_STATE,
                    // 0x1, RegData);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // PLLU_HW_PWRDN_CFG0,
                    // PLLU_SEQ_IN_SWCTL,
                    // 0x0, RegData);

    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            // CLK_RST_CONTROLLER_PLLU_HW_PWRDN_CFG0_0, RegData);
    // NvBootUtilWaitUS(10);
    
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    // CLK_RST_CONTROLLER_PLLU_HW_PWRDN_CFG0_0);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    // PLLU_HW_PWRDN_CFG0,
                    // PLLU_SEQ_ENABLE,
                    // 0x1, RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            // CLK_RST_CONTROLLER_PLLU_HW_PWRDN_CFG0_0, RegData);

    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0);
    // // Remove power down for PLLU
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLLU_POWERDOWN,
                    // 0x0,
                    // RegData);
    // // Remove power down for PLLU
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, 
                    // UTMIP_FORCE_PLLU_POWERUP,
                    // 0x0,
                    // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_UTMIP_PLL_CFG1_0, RegData);

    // /* Disable SW override of PLLU */
    // RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLU_BASE_0);
    // RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLU_BASE, 
                    // PLLU_OVERRIDE,
                    // 0x0,
                    // RegData);
    // NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLU_BASE_0, RegData);

    // return NvBootError_Success;
// }
#if XUSB_DEV_SS_SUPPORTED
NvBootError NvBootXusbDeviceSSPadSetup()
{
    NvBootError e = NvBootError_Success;
    NvU32 Mask, ExpectedValue, PortSpeed, RegData;
    /***************** Enable PLLREFE **************************************/
    // Remove from IDDQ state
    
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLREFE_MISC_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLREFE_MISC,
                    PLLREFE_IDDQ,
                    0x0, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLREFE_MISC_0, RegData);
    

    // Disable to program dividers.
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLREFE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLREFE_BASE,
                    PLLREFE_ENABLE,
                    0x0, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLREFE_BASE_0, RegData);

    // Program dividers
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLREFE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLREFE_BASE,
                    PLLREFE_MDIV,
                    0x1, RegData);   
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLREFE_BASE,
                    PLLREFE_NDIV,
                    0x38, RegData);  
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLREFE_BASE_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLREFE_MISC_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLREFE_MISC,
                    PLLREFE_LOCK_ENABLE,
                    0x1, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLREFE_MISC_0, RegData);
    // Enable PLLREFE
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLREFE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLREFE_BASE,
                    PLLREFE_ENABLE,
                    0x1, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLREFE_BASE_0, RegData);
    /**************************** PLLE ******************************************/

    // Remove from IDDQ state
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_MISC_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_MISC,
                    PLLE_IDDQ_OVERRIDE_VALUE,
                    0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_MISC_0, RegData);
    // Disable to program dividers.
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_ENABLE,
                    0x0, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_BASE_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_AUX_0);

    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                        PLLE_AUX,
                        PLLE_REF_SEL_PLLREFE,
                        0x0, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_AUX_0, RegData);


    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_MDIV,
                    0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_NDIV,
                    0xC8, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_PLDIV_CML,
                    0xD, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_BASE_0, RegData);

    // Lock enable
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_MISC_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_MISC,
                    PLLE_LOCK_ENABLE,
                    0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_MISC,
                    PLLE_PTS,
                    0x1, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_MISC_0, RegData);
    
        // Enable PLLE
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_ENABLE,
                    0x1, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_BASE_0, RegData);

    // Poll for lock
    Mask = SHIFTMASK(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK);
    ExpectedValue = 1 << SHIFT(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK);
    e = NvBootPollField(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLE_MISC_0,
                    Mask,
                    ExpectedValue,
                    1000);//
    if(e != NvBootError_Success)
        return e;


    // Disable PLLE
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_ENABLE,
                    0x0, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_BASE_0, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_SS_CNTL_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCBYP,
                    0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_BYPASS_SS,
                    0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_INTERP_RESET,
                    0x1, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);

    // Enable PLLE
    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_BASE_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_BASE,
                    PLLE_ENABLE,
                    0x1, RegData);   
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_BASE_0, RegData);

    // Poll for lock
    Mask = SHIFTMASK(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK);
    ExpectedValue = 1 << SHIFT(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK);
    e = NvBootPollField(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLE_MISC_0,
                    Mask,
                    ExpectedValue,
                    1000);

    if(e != NvBootError_Success)
        return e;

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE+
                    CLK_RST_CONTROLLER_PLLE_SS_CNTL_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCCENTER,
                    0x0, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCINVERT,
                    0x0, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCMAX,
                    0x25, RegData);
    
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCINCINTRV,
                    0x20, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCINC,
                    0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_BYPASS_SS,
                    0x0, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_SSCBYP,
                    0x0, RegData);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                    PLLE_SS_CNTL,
                    PLLE_INTERP_RESET,
                    0x0, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE+
                            CLK_RST_CONTROLLER_PLLE_SS_CNTL_0, RegData);
    // Poll for lock
    Mask = SHIFTMASK(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK);
    ExpectedValue = 1 << SHIFT(CLK_RST_CONTROLLER, PLLE_MISC, PLLE_LOCK);
    e = NvBootPollField(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_PLLE_MISC_0,
                    Mask,
                    ExpectedValue,
                    1000);
    if(e != NvBootError_Success)
        return e;

    // PEX pads used for device mode
    RegData = NV_READ32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_PLL_P0_CTL1, PLL0_MODE,  0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_PLL_P0_CTL1, PLL_PWR_OVRD, 0x1, RegData);
    RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, IOPHY_PLL_P0_CTL1, PLL_RST_, 0x1, RegData);
     NV_WRITE32(NV_ADDRESS_MAP_APB_XUSB_PADCTL_BASE + 
                        XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0, RegData);

    return e;
}
#endif
NvBootError NvBootXusbDeviceReceive(NvU8* Buffer, NvU32 Bytes,  NvU32 *pBytesReceived)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_OUT;
    NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
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

NvBootError NvBootXusbDeviceTransmit(NvU8* Buffer, NvU32 Bytes, NvU32 *pBytesTransmitted)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_IN;
    NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
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

NvBootError NvBootXusbDeviceReceiveStart(NvU8* Buffer, NvU32 Bytes)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_OUT;
    e = NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
    pXUSBDeviceContext->TxCount++;
    return e;
}
NvBootError NvBootXusbDeviceReceivePoll(NvU32 *pBytesReceived, NvU32 TimeoutUs, NvU8* OptionalBuffer)
{
    NvBootError e = NvBootError_Success;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    while(pXUSBDeviceContext->TxCount)
    {
         e = NvBootXusbDevicePollForEvent(TimeoutUs);
         if(e != NvBootError_Success)
            break;
    }
    *pBytesReceived = pXUSBDeviceContext->BytesTxfred;
    return e;
}

NvBootError NvBootXusbDeviceTransmitStart(NvU8* Buffer, NvU32 Bytes)
{
    NvBootError e = NvBootError_Success;
    NvU32 Direction;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    pXUSBDeviceContext->BytesTxfred = Bytes;
    pXUSBDeviceContext->TxCount = 0;
    Direction = DIR_IN;
    NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, Bytes, Direction);
    pXUSBDeviceContext->TxCount++; 
    // Need to transmit zero byte packet if transfer size is equal to packet size
    if((pXUSBDeviceContext->PortSpeed == XUSB_FULL_SPEED && Bytes == 64)
       || (pXUSBDeviceContext->PortSpeed == XUSB_HIGH_SPEED && Bytes == 512)
       || (pXUSBDeviceContext->PortSpeed == XUSB_SUPER_SPEED && Bytes == 1024))
    {
        // Buffer should n't matter. It is ZBP.
        NvBootXusbDeviceIssueNormalTRB((NvU32)Buffer, 0, Direction);
        pXUSBDeviceContext->TxCount++; 
    }
    
    return e;
}
NvBootError NvBootXusbDeviceTransmitPoll(NvU32 *pBytesTransferred, NvU32 TimeoutMs, NvU8* OptionalBuffer)
{
    NvBootError e = NvBootError_Success;
    XUSBDeviceContext_T *pXUSBDeviceContext = &XUSBDeviceContext;

    while(pXUSBDeviceContext->TxCount)
    {
         e = NvBootXusbDevicePollForEvent(0xFFFFFFFF);
         if(e != NvBootError_Success)
            break;
    }
    *pBytesTransferred = pXUSBDeviceContext->BytesTxfred;
    return e;
}
NvBootError NvBootXusbHandleError(void)
{
    // todo
    return NvBootError_Success;
}

