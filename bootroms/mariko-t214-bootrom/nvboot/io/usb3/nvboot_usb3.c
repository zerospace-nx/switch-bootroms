/*
 * Copyright (c) 2011 - 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_usb3_local.h"
#include "nvboot_xusb_msc.h"
#include "nvrm_drf.h"
#include "arclk_rst.h"
#include "armc.h"
#include "arfuse.h"
#include "arxusb_padctl.h"
#include "nvboot_ahb_int.h"
#include "nvboot_bit.h"
#include "nvboot_config.h"
#include "nvboot_clocks_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_codecov_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_usb3_context.h"
#include "nvboot_usb3_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "nvboot_log_int.h"
#include "nvboot_fuse_int.h"
// added from ip release copied here temporarily
// http://nvbugs/870232  to track the release process
#include "ardev_t_xusb_csb.h"
#include "ardev_t_xusb_xhci.h"
#include "ardev_t_fpci_xusb.h"
#include "ardev_t_fpci_xusb_0.h"
#include "nvboot_xusb_dev_int.h"
#include "nvboot_rcm_port_int.h"
#include "nvboot_devmgr_int.h"
#include "nvboot_pads_int.h"

#include "project.h"

/*
 * Xusb host callbacks for Device Manager.
 */
NvBootDevMgrCallbacks Usb3DeviceCallback = 
    {
        /* Callbacks for the Usb3 host  */
        (NvBootDeviceGetParams)NvBootUsb3GetParams,
        (NvBootDeviceValidateParams)NvBootUsb3ValidateParams,
        (NvBootDeviceGetBlockSizes)NvBootUsb3GetBlockSizes,
        (NvBootDeviceInit)NvBootUsb3Init,
        (NvBootDeviceRead)NvBootUsb3Read,
        (NvBootDeviceQueryStatus)NvBootUsb3QueryStatus,
        (NvBootDeviceShutdown)NvBootUsb3Shutdown,
        (NvBootDeviceGetReaderBuffersBase)NvBootUsb3GetReaderBuffersBase,
        (NvBootDevicePinMuxInit)NvBootUsb3PinMuxInit
    };

extern const NvU32 s_UsbHsicTrkDiv[];

/// arxusb_padctl.h ////////
#define NV_XUSB_PADCTL_EXPAND(reg, value) \
    do { \
         value = ((XUSB_PADCTL_##reg##_0)); \
    } while (0)

#define NV_XUSB_PADCTL_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_##reg##_0)); \
    } while (0)

#define NV_XUSB_PADCTL_WRITE(reg, value) \
    do { \
        NV_WRITE32((NV_ADDRESS_MAP_XUSB_PADCTL_BASE + XUSB_PADCTL_##reg##_0), value); \
    } while (0)

#define NV_XUSB_CSB_EXPAND(reg, value) \
    do { \
         value = ((XUSB_CSB_##reg##_0)); \
    } while (0)

//// ardev_t_fpci_xusb.h ///
#define NV_XUSB_CFG_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_XUSB_HOST_APB_DFPCI_CFG + XUSB_CFG_##reg##_0)); \
    } while (0)

#define NV_XUSB_CFG_WRITE(reg, value) \
    do { \
        NV_WRITE32((NV_XUSB_HOST_APB_DFPCI_CFG + XUSB_CFG_##reg##_0), value); \
    } while (0)

#define NV_XUSB_CFG_IND_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_XUSB_HOST_APB_DFPCI_CFG + reg)); \
    } while (0)

#define NV_XUSB_CFG_IND_WRITE(reg, value) \
    do { \
        NV_WRITE32((NV_XUSB_HOST_APB_DFPCI_CFG + reg), value); \
    } while (0)

///ardev_t_xusb_xhci.h ////
#define NV_XUSB_XHCI_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_XUSB_HOST_APB_BAR0_START + XUSB_XHCI_##reg##_0)); \
    } while (0)

#define NV_XUSB_XHCI_WRITE(reg, value) \
    do { \
        NV_WRITE32((NV_XUSB_HOST_APB_BAR0_START+ XUSB_XHCI_##reg##_0), value); \
    } while (0)

///arxusb_host.h ////
#define NV_XUSB_HOST_READ(reg, value) \
    do \
    { \
        value = NV_READ32((NV_XUSB_HOST_IPFS_REGS + XUSB_HOST_##reg##_0)); \
    } while (0)

#define NV_XUSB_HOST_WRITE(reg, value) \
    do { \
        NV_WRITE32((NV_XUSB_HOST_IPFS_REGS + XUSB_HOST_##reg##_0), value); \
    } while (0)

#define INT_TO_BYTE_ARRAY(int_a, byte_b)    \
do \
{ \
 byte_b[0] = int_a&0xff;\
 byte_b[1] = (int_a&0xff00) >> 8; \
 byte_b[2] = (int_a&0xff0000) >> 16; \
 byte_b[3] = (int_a&0xff000000) >> 24; \
}while(0) \

#define SHORT_TO_BYTE_ARRAY(short_a, byte_b)    \
do \
{ \
 byte_b[0] = short_a&0xff;\
 byte_b[1] = (short_a&0xff00) >> 8; \
}while(0) \


// There are 2 Bus Instances (BI) and 3 Port Instances (PI)
// BI 0 services PI 0,1 and BI 1 services PI 2
// BI 1 addresses are at 0x200 offset from BI 0
#if NVBOOT_TARGET_FPGA
#define BI_OFFSET(PortInstance)    (0)
#define EP_ROOTPORTNUM(RootPortNum) (RootPortNum)
#else
#define BI_OFFSET(PortInstance)    (((PortInstance)==0x2)?0x200:0)
#define EP_ROOTPORTNUM(RootPortNum) ((RootPortNum)%0x2)
#endif
#define CEIL_PAGE(LEN, PAGE_SIZE)  (((LEN)+(PAGE_SIZE)-1)/(PAGE_SIZE))

//#define DEBUG_USBHC 1
#if DEBUG_USBHC
#define PRINT_USBH_REG_ACCESS(...)  NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_USBH_MESSAGES(...)    NvOsDebugPrintf(__VA_ARGS__);
#define PRINT_USBH_ERRORS(...)      NvOsDebugPrintf(__VA_ARGS__);
#else
#define PRINT_USBH_REG_ACCESS(...)
#define PRINT_USBH_MESSAGES(...)
#define PRINT_USBH_ERRORS(...)
#endif

// Should it be aligned to 16 or 64?
//NV_ALIGN(16) EP     EpContext = {0};
EP     *EpContext        =  (EP*)EP_CXT_START;
// Should it be aligned to 16 or 64?
TRB    *TRBRing          =  (TRB*)TRB_RING_START; //The TRB structure referenced by this physical memory pointer shall be aligned to a 16-byte boundary.

uint8_t   *BufferXusbCmd    =  (uint8_t*)MSC_BOT_CMD_START; // 32 bytes are good enough for MSC BOT commands
uint8_t   *BufferXusbStatus =  (uint8_t*)MSC_BOT_STATUS_START; // 32 bytes are good enough for MSC BOT status
uint8_t   *BufferXusbData   =  (uint8_t*)MSC_BOT_DATA_START ; // 256 bytes are good enough for MSC BOT command response and also good enough for control data TRB during enumeration

/*
 * This contians default params that will be used pre BCT.Some of the
 * params of this struct, which are specific to Usb3 flash present
 * on board, will be changed based on the fuse settings passed to the
 * API NvBootUsb3GetParams(). See NvBootUsb3GetParams()
 * for more info on what gets changed based on fuse value.
 */
static NvBootUsb3Params s_DefaultUsb3Params;
// Pointer to store the context memory address.
static NvBootUsb3Context *s_Usb3Context = NULL;
// Boot Info table.
extern NvBootInfoTable BootInfoTable;

/* Forward Private Function declarations. */

/* Private Function Definitions. */

/*
 * Functions HwUsbxxx --> does Usbh register programming.
 * Functions NvBootUsbxxx --> Public API's
 */

//_________________________________________________________________________________________________
static void PciRegUpdate(NvBootUsb3Context *Context, NvU32 Register, NvBool Update, NvU32 *value)
{
    NvU32 RegOffet = Register % XUSB_CFG_PAGE_SIZE;
    NvU32 PageNum = Register / XUSB_CFG_PAGE_SIZE;

/*
a.	Find the CSB space address by looking into the ref manual. Here it is 0x110084
        >>> EXPAND
b.	Program the value of 0x110084/0x200 into NV_PROJ__XUSB_CFG_ARU_C11_CSBRANGE
        >>> NV_PROJ__XUSB_CFG_ARU_C11_CSBRANGE
c.	 The corresponding register address in the configuration space (MMIO) is  0x110084%0x200 + 0x800(base address in the configuration space) = 0x884
        >>>        NV_XUSB_CFG_WRITE(CSB_ADDR, (RegOffet + 0x800));
d.	Read/write the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_DWRD0 register by issuing a PCI config read/write request at register address 0x884h.
        >>>> ?
*/
    // update PageNumber in usb3context
    if(Context->CurrentCSBPage != PageNum)
    {
        Context->CurrentCSBPage = PageNum;
        // update the NV_PROJ__XUSB_CFG_ARU_C11_CSBRANGE  with PageNum
        NV_XUSB_CFG_WRITE(ARU_C11_CSBRANGE, PageNum);
    }

    if(Update)
    {
        NV_XUSB_CFG_IND_WRITE((RegOffet + XUSB_CFG_CSB_ADDR_0), *value);
    }
    else
    {
        NV_XUSB_CFG_IND_READ((RegOffet + XUSB_CFG_CSB_ADDR_0), *value);
    }
}

/* Updated Boot Media port to Host controller port mapping
*
In T186, There is a 1:1 mapping between boot port and xusb controller port.
This table is not required unless this 1:1 mapping changes.
*
*/
static NvBool BootPortControllerMapping(uint8_t BootPortNumber, uint8_t* ControllerPortNumber)
{
    uint8_t BootHostControllerMapping[11] = {
                                            T210_USB_HOST_PORT_OTG0,
                                            T210_USB_HOST_PORT_1,
                                            T210_USB_HOST_PORT_2,
                                            T210_USB_HOST_PORT_3,
                                            T210_RESERVED
                                        };
    // set to reserved.
    *ControllerPortNumber = T210_RESERVED;
    *ControllerPortNumber = BootHostControllerMapping[BootPortNumber];
    if(*ControllerPortNumber != T210_RESERVED)
        return NV_TRUE;
    else
        return NV_FALSE;
}

NvBootError HwUsbIsDeviceAttached(NvBootUsb3Context *Context )
{
    NvBootError ErrorCode = NvBootError_Success;
    NvU32 value = 0;
    NvU32 RegOffset = 0;
    uint8_t PortNumOffset = 0;
    uint8_t ControllerPortNumber = 0;

    if(!BootPortControllerMapping((Context->RootPortNum),&ControllerPortNumber))
    {
        return NvBootError_InvalidParameter;// not supported port.
    }
    PortNumOffset = (ControllerPortNumber * PORT_INSTANCE_OFFSET);// using the offset for portnum

    // check the register bits/root port no.,/csc/ccs of the port status for device attachment
    NV_XUSB_CSB_EXPAND(HSPI_PVTPORTSC3, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_PF_VAL(XUSB_CSB_HSPI_PVTPORTSC3_0, CCS , value);

    if(!value )
    {
        ErrorCode = NvBootError_XusbDeviceNotAttached;
    }
    return ErrorCode;
}

#if NVBOOT_TARGET_FPGA
void xusbhostunplug_plug_seq()
{
    volatile NvU32 *HotPlugPtr = (NvU32*)(NV_ADDRESS_MAP_XUSB_HOST_BASE+0x85f4);
    volatile NvU32 HotPlug;

    NvBootUtilWaitUS(TIMEOUT_1S);
    HotPlug = *HotPlugPtr;
    *HotPlugPtr = (NvU32)(HotPlug | 0x7000);
    
    NvBootUtilWaitUS(TIMEOUT_1S);
    
    *HotPlugPtr = (NvU32)(HotPlug);
    
    NvBootUtilWaitUS(TIMEOUT_1S);

}
#endif
//_________________________________________________________________________________________________

NvBootError DevicePortInit( NvBootUsb3Context *Context )
{
    NvBootError ErrorCode = NvBootError_Success;
    NvU32 value = 0;
    NvU32 RegOffset = 0;
    uint8_t PortNumOffset = 0;
    uint8_t ControllerPortNumber = 0;
    NvU32 TimeoutRetries = CONTROLLER_HW_RETRIES_2000; //reset should be asserted for 50ms anywhere between 1ms to 50ms, max timeout 100ms
    NvU32 TimeoutAttachDetachRetries = CONTROLLER_HW_RETRIES_2000;

#if NVBOOT_TARGET_RTL
    //XUSB_CSB_HSPI_PVTPORTSC3_0
    // Need to wait a max timeout value of 120000 clocks of 16.667ns before device attachment is confirmed.
    // as described by bebick during debug.
    // To reduce the wait time wrie a value in XUSB_CFG_HSPX_CORE_CNT9_0, say 1000 instead of 12000

    // hack only for sims to speed up.
    // select page before settign the value to the counter..
    NV_XUSB_CFG_READ(ARU_C11PAGESEL1, RegOffset);
    RegOffset = NV_FLD_SET_DRF_DEF(XUSB_CFG, ARU_C11PAGESEL1, HSP0, SEL, RegOffset);
    NV_XUSB_CFG_WRITE(ARU_C11PAGESEL1, RegOffset);
    // update the counter as mailed by bebick ..
    NV_XUSB_CFG_WRITE(HSPX_CORE_CNT5, 0x61A8);//0x1C350//0xC350
    // changing avlue from 11000 to 6000 as requested in bug
    // changing value from 6000 to 5000 after adjusting the osc-clk to be exactly 13Mhz instead of 13.1,, count got misalligned
    // to 5998,, missing by 2.. hence setting it to 5000 as of now..
    NV_XUSB_CFG_WRITE(HSPX_CORE_CNT2,0x700 );//0x1388//0x1770) //0x2AF8
    // writing (7996) earlier programmed for 1000 to XUSB_CFG_HSPX_CORE_CNT9_0
    NV_XUSB_CFG_WRITE(HSPX_CORE_CNT9, 0x2400);//0x1F3C


    // moving up due to page select..
    NV_XUSB_CFG_WRITE(HSPX_CORE_CNT1, 0x5DC);//0xBB8//0x4B0//0x5DC //0x1F3C
    NV_XUSB_CFG_WRITE(HSPX_CORE_CNT7, 0x3000);//0x1EE0//0x4B0//0x1F3C
    // update for FS hacks
    // set bit20 and clear bit 12 (HS) for page select..
    NV_XUSB_CFG_READ(ARU_C11PAGESEL1, RegOffset);
    RegOffset = NV_FLD_SET_DRF_DEF(XUSB_CFG, ARU_C11PAGESEL1, HSP0, DESEL, RegOffset);
    RegOffset = NV_FLD_SET_DRF_DEF(XUSB_CFG, ARU_C11PAGESEL1, FSP0, SEL, RegOffset);
    NV_XUSB_CFG_WRITE(ARU_C11PAGESEL1, RegOffset);

    // adding fspx core cnt1 pgming to increase connect time
    NV_XUSB_CFG_WRITE(FSPX_CORE_CNT1, 0x4800);

    NV_XUSB_CFG_WRITE(HSPX_CORE_CNT3, 0x9C30);

    /* One write to setup the CSB_RANGE register
    For all NV_PROJ__XUSB_CSB_HSPI _PVTPORTSC* accesses, the port number is identified by USB port number register in FUSE_BOOT_DEVICE_INFO_0 register.
    Check for the device attachment by reading NV_PROJ__XUSB_CSB_HSPI _PVTPORTSC3_CCS field (should be '1')
    One write to clear 'NV_PROJ__XUSB_CSB_HSPI _PVTPORTSC2_CSC' field upon device detection.
    Wait 100ms.
    */
#endif

/*
    if(NvBootGetSwCYA() & NVBOOT_SW_CYA_HOST_DESERIALIZER_ENABLE)
    {
        // Choose page
        NV_XUSB_CFG_WRITE(ARU_C11PAGESEL0, 0);
        NV_XUSB_CFG_WRITE(ARU_C11PAGESEL1, 0x1000);

        // Choose MCP mode.
        NV_XUSB_CFG_READ(HSPX_CORE_NVWRAP_DESER, value);
        value = NV_FLD_SET_DRF_NUM(XUSB_CFG,
                                   HSPX_CORE_NVWRAP_DESER,
                                   MCP_MODE,
                                   0,
                                   value);
        NV_XUSB_CFG_WRITE(HSPX_CORE_NVWRAP_DESER, value); // Chooses t210 mode

        NV_XUSB_CFG_WRITE(ARU_C11PAGESEL1, 0x0);
    }
*/
    // Enable sof count..
    NV_XUSB_CSB_EXPAND(ARU_CTRL, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, ARU_CTRL, MFCOUNT, RUN, value);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_WRITE , &value);

    if(!BootPortControllerMapping((Context->RootPortNum),&ControllerPortNumber))
    {
        return NvBootError_InvalidParameter;// not supported port.
    }
    PortNumOffset = (ControllerPortNumber * PORT_INSTANCE_OFFSET);// using the offset for portnum

    // Step 7

    /*
        Set Port to Disconnected state to check device connection, 
        where N indicates the port the boot media connected to.
        One write to setup the CSB_RANGE register
        For all NV_PROJ__XUSB_CSB_HSPI _PVTPORTSC* accesses, the port number 
        is identified by USB port number register in FUSE_BOOT_DEVICE_INFO_0 register
    */
    
    // Write 'Disconnected' to NV_PROJ__XUSB_CSB_HSPI_PVTPORTSC1N_PLS_CNTRL
    // Write '1' to NV_PROJ__XUSB_CSB_HSPI_PVTPORTSC1_PLS_VALID
    NV_XUSB_CSB_EXPAND(HSPI_PVTPORTSC1, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HSPI_PVTPORTSC1, PLS_CNTRL, DISCONNECTED, value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HSPI_PVTPORTSC1, PLS_VALID, SET, value);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_WRITE , &value);

    // Write 'Disconnected' to NV_PROJ__XUSB_CSB_FSPI_PVTPORTSC1_PLS_CNTRL
    // Write '1' to NV_PROJ__XUSB_CSB_FSPI_PVTPORTSC1_PLS_VALID
    NV_XUSB_CSB_EXPAND(FSPI_PVTPORTSC1, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, FSPI_PVTPORTSC1, PLS_CNTRL, DISCONNECTED, value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, FSPI_PVTPORTSC1, PLS_VALID, SET, value);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_WRITE , &value);

    /* apply port unplug/plug sw sequence reset for fpga
       This is not needed in T18x. Keep in case needed later */
// #if NVBOOT_TARGET_FPGA
    // NvBootUtilWaitUS(TIMEOUT_10MS);
    // xusbhostunplug_plug_seq();
    // NvBootUtilWaitUS(TIMEOUT_10MS);
// #endif
    // Check for the device attachment by reading 
    // NV_PROJ__XUSB_CSB_FSPI _PVTPORTSC3 N _CCS field (should be '1')
    while(TimeoutAttachDetachRetries)
    {
        NV_XUSB_CSB_EXPAND(FSPI_PVTPORTSC3, RegOffset);
        value = 0;
        PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
        value = NV_PF_VAL(XUSB_CSB_FSPI_PVTPORTSC3_0, CCS , value);


        if(value == XUSB_CSB_FSPI_PVTPORTSC3_0_CCS_DEV)
            break;
        TimeoutAttachDetachRetries--;
#if NVBOOT_TARGET_FPGA
        NvBootUtilWaitUS(TIMEOUT_10MS);//(TIMEOUT_1MS); 
#else
        NvBootUtilWaitUS(TIMEOUT_10US);
#endif
    }

    if(!value && !TimeoutAttachDetachRetries)
    {
        NVBOOT_MSG((Q_USB3BOOT_DEV_DETECT_FAILED));
        return NvBootError_XusbDeviceNotAttached;// define in nvboot_error.h
    }

    NVBOOT_MSG((Q_USB3BOOT_DEV_DETECT));

    // One write to clear 'NV_PROJ__XUSB_CSB_FSPI _PVTPORTSC2 N _CSC' field upon device detection.
    // read & clear
    NV_XUSB_CSB_EXPAND(FSPI_PVTPORTSC2, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, FSPI_PVTPORTSC2, CSC, CLEAR, value);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_WRITE , &value);

    //Wait 100ms.
#if NVBOOT_TARGET_RTL
    NvBootUtilWaitUS(TIMEOUT_1US);//(TIMEOUT_1MS); 
#else
    NvBootUtilWaitUS(TIMEOUT_100MS);
#endif

    // Step 8
    // If there is a device attachment detected then Boot rom will reset 
    // the port to change the state to enable and device state to default state, 
    // this reset also change the link speed to HighSpeed.
	// The reset operation is required for USB2.0 devices. 
    // The plan is to support USB2.0 mode only during boot

    //Write 'Reset' to USB3_CSB_FSPI_PVTPORTSC1 N_PLS_CNTRL register to reset the port.
    //Write '1' to NV_PROJ__XUSB_CSB_FSPI_PVTPORTSC1 N_PLS_VALID
    NV_XUSB_CSB_EXPAND(FSPI_PVTPORTSC1, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, FSPI_PVTPORTSC1, PLS_VALID, SET, value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, FSPI_PVTPORTSC1, PLS_CNTRL, RESET, value);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_WRITE , &value);


    // Write 'Reset' to USB3_CSB_HSPI_PVTPORTSC1 N _PLS_CNTRL register to reset the port.
    // Write '1' to NV_PROJ__XUSB_CSB_HSPI_PVTPORTSC1_PLS_VALID
    // The reset operation is required for USB2.0 devices. 
    // The plan is to support USB2.0 mode only during boot.
    // This corresponds to one register write request.
    // Wait 50ms.

    NV_XUSB_CSB_EXPAND(HSPI_PVTPORTSC1, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HSPI_PVTPORTSC1, PLS_VALID, SET, value);
    value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HSPI_PVTPORTSC1, PLS_CNTRL, RESET, value);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_WRITE , &value);

    //Wait 50ms.
#if NVBOOT_TARGET_RTL
    NvBootUtilWaitUS(TIMEOUT_1US);//(TIMEOUT_1MS); 
#else
    NvBootUtilWaitUS(TIMEOUT_50MS);
#endif

    // Step 9
    // Confirm the port reset completion by looking at
    // USB3_CSB_HSPI_PVTPORTSC3_PLS_STATUS == Enabled state (in Port instance)
    // Boot rom needs to poll this field
    NV_XUSB_CSB_EXPAND(HSPI_PVTPORTSC3, RegOffset);
    PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
    value = NV_DRF_VAL(XUSB_CSB, HSPI_PVTPORTSC3, PLS_STATUS, value);

    while(TimeoutRetries)
    {
        NV_XUSB_CSB_EXPAND(HSPI_PVTPORTSC3, RegOffset);
        PciRegUpdate(Context, (RegOffset + PortNumOffset), XUSB_READ, &value);
        value = NV_DRF_VAL(XUSB_CSB, HSPI_PVTPORTSC3, PLS_STATUS, value);

        if((value == XUSB_CSB_HSPI_PVTPORTSC3_0_PLS_STATUS_ENABLED) || \
            (value == XUSB_CSB_HSPI_PVTPORTSC3_0_PLS_STATUS_FS_MODE))
            break;
        TimeoutRetries--;
        // reset should be asserted for 50ms anywhere between 1ms to 50ms, max timeout 100ms
#if NVBOOT_TARGET_RTL
        NvBootUtilWaitUS(TIMEOUT_1US);//(TIMEOUT_1MS); 
#else
        NvBootUtilWaitUS(TIMEOUT_1MS);// polling duration 1 ms
#endif        
    }

    if(!TimeoutRetries)
        return NvBootError_XusbPortResetFailed;

    NVBOOT_MSG((Q_USB3BOOT_DEV_PORT));

    // Wait 10ms for reset recovery.
#if NVBOOT_TARGET_RTL
    NvBootUtilWaitUS(TIMEOUT_1US);
#else
    NvBootUtilWaitUS(TIMEOUT_10MS);
#endif
    return ErrorCode;
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbPrepareControlSetupTRB(NvBootUsb3Context *Context,DeviceRequest *DeviceRequestPtr, NvBootTRTType *TrttTyp)
{
    SetupTRB *SetupTRBPtr = NULL;
    NvBootError ErrorCode = NvBootError_Success;

    Context->TRBRingCtrlEnquePtr    = (TRB *)(&(TRBRing[0]));
    SetupTRBPtr = (SetupTRB *)(Context->TRBRingCtrlEnquePtr);

    NvBootUtilMemset((void *)SetupTRBPtr,0,sizeof(SetupTRB));//To make sure that all reserved bits are set to 0
    switch(DeviceRequestPtr->bRequest)
    {
        //Refer to Section 9.3 of USB 2.0 Specification at http://www.usb.org/developers/docs/
        //bmRequestType (8bits)     : identifies the direction of data transfer in the second phase of the control transfer. The state of the Direction bit is
        //                            ignored if the wLength field is zero, signifying there is no Data stage
        //bRequest      (8bits)     : specifies the particular request
        //wValue        (16 bits)   : It is used to pass a parameter to the device, specific to the request
        //wIndex        (16 bits)   : Is used in requests to specify an endpoint or an interface. bit 0-7 = interface number
        //                            OR bit 0-3 = endpoint number and bit 7 denotes direction of that endpoint
        //wLength       (16 bits)   : This field specifies the length of the data transferred during the second phase of the control transfer. The
        //                            direction of data transfer (host-to-device or device-to-host) is indicated by the Direction bit of the
        //                            bmRequestType field. If wLength is zero, there is no data transfer phase.
        //                            On an input request, a device must never return more data than is indicated by the wLength value; it may
        //                            return less. On an output request, wLength will always indicate the exact amount of data to be sent by the
        //                            host. Device behavior is undefined if the host should send more data than is specified in wLength
        case    GET_DESCRIPTOR      :
        case    GET_CONFIGURATION   :
        case    SET_ADDRESS         :
        case    SET_CONFIGURATION   :
        case    GET_MAX_LUN         :
        case    CLEAR_FEATURE         : break;
        //case    BULK_ONLY_MSC_RST   : break;

        default                     : ErrorCode        = NvBootError_XusbInvalidBmRequest;
                                      break;
    }
    if (ErrorCode == NvBootError_Success)
    {
        SetupTRBPtr->bmRequestType          =   DeviceRequestPtr->bmRequestTypeUnion.bmRequestType;
        SetupTRBPtr->bmRequest               =   DeviceRequestPtr->bRequest;
        SetupTRBPtr->wValue                 =   DeviceRequestPtr->wValue;
        SetupTRBPtr->wIndex                 =   DeviceRequestPtr->wIndex;
        SetupTRBPtr->wLength                =   DeviceRequestPtr->wLength;
        SetupTRBPtr->TRBTfrLen              =   USB_SETUP_PKT_SIZE;   //Tansfer 8 bytes

        /* Since Memory area for this TRB is already initialized to 0, there is no need to program these fields
                SetupTRBPtr->InterrupterTarget      =  0;
                SetupTRBPtr->IOC                    =  0;
            */
        SetupTRBPtr->CycleBit               =  Context->CycleBit;
        SetupTRBPtr->IDT                    =   1;                   //Immediate
        SetupTRBPtr->TRBType                =   NvBootTRB_SetupStage;
        if(DeviceRequestPtr->wLength != 0)
        {
            if(DeviceRequestPtr->bmRequestTypeUnion.stbitfield.dataTransferDirection == DEV2HOST)
            {
                *TrttTyp = NvBootTRT_InDataStage;
            }
            else
            {
                //HOST2DEV
                *TrttTyp = NvBootTRT_OutDataStage;
            }
        }
        else
        {
            *TrttTyp = NvBootTRT_NoDataStage;
        }
        SetupTRBPtr->TRT                    =   *TrttTyp;
    }
    //update TRB pointer
    (Context->TRBRingCtrlEnquePtr)++;
    return ErrorCode;
}
//_________________________________________________________________________________________________

void NvBootXusbPrepareControlDataTRB(NvBootUsb3Context *Context,DeviceRequest *DeviceRequestPtr)
{
    DataStageTRB *DataStageTRBPtr = (DataStageTRB *)(Context->TRBRingCtrlEnquePtr);

    NvBootUtilMemset((void *)DataStageTRBPtr,0,sizeof(DataStageTRB));//To make sure that all reserved bits are set to 0

    DataStageTRBPtr->DataBufferLo       = (NvU32)(BufferXusbData);
    DataStageTRBPtr->TRBTfrLen          = DeviceRequestPtr->wLength;
    /* No need to set to 0 since this memory area is already set to 0
    DataStageTRBPtr->DataBufferHi       = 0;
    DataStageTRBPtr->TDSize             = 0;
    DataStageTRBPtr->InterrupterTarget  = 0;
    DataStageTRBPtr->CycleBit           = 0;
    DataStageTRBPtr->ENT                = 0;
    DataStageTRBPtr->ISP                = 0;
    DataStageTRBPtr->NS                 = 0;
    DataStageTRBPtr->IOC                = 0;
    DataStageTRBPtr->IDT                = 0;
    */
    DataStageTRBPtr->CycleBit           = Context->CycleBit;
    DataStageTRBPtr->TRBType            = NvBootTRB_DataStage;

    /*Logic below is optimized to just one line of code since
    USB_DIR_OUT matches HOST2DEV enum
    USB_DIR_IN  matches DEV2HOST enum
    */
    DataStageTRBPtr->DIR                = DeviceRequestPtr->bmRequestTypeUnion.stbitfield.dataTransferDirection;
    //update TRB pointer
    (Context->TRBRingCtrlEnquePtr)++;
    return;
}
//_________________________________________________________________________________________________

void NvBootXusbPrepareControlStatusTRB(NvBootUsb3Context *Context,DeviceRequest *DeviceRequestPtr)
{
    StatusStageTRB *StatusStageTRBPtr = (StatusStageTRB *)(Context->TRBRingCtrlEnquePtr);

    NvBootUtilMemset((void *)StatusStageTRBPtr,0,sizeof(StatusStageTRB));//To make sure that all reserved bits are set to 0

    StatusStageTRBPtr->TRBType      = NvBootTRB_StatusStage;
    StatusStageTRBPtr->DIR          = !(DeviceRequestPtr->bmRequestTypeUnion.stbitfield.dataTransferDirection);
    StatusStageTRBPtr->CycleBit             = Context->CycleBit;

    /*No need to set to 0 as this memory area is already set to 0
    StatusStageTRBPtr->InterrupterTarget    = 0;
    StatusStageTRBPtr->CycleBit             = 0;
    StatusStageTRBPtr->ENT                  = 0;
    StatusStageTRBPtr->CH                   = 0;
    StatusStageTRBPtr->IOC                  = 0;
    */
    //update TRB pointer
    (Context->TRBRingCtrlEnquePtr)++;
    return;
}

void NvBootXusbPrepareEndTRB(NvBootUsb3Context *Context)
{
    StatusStageTRB *EndStageTRBPtr = (StatusStageTRB *)(Context->TRBRingCtrlEnquePtr);

    NvBootUtilMemset((void *)EndStageTRBPtr,0,sizeof(StatusStageTRB));//To make sure that all reserved bits are set to 0

    EndStageTRBPtr->TRBType      = NvBootTRB_Reserved;
    EndStageTRBPtr->CycleBit             = !(Context->CycleBit);
    //update TRB pointer
    Context->TRBRingCtrlEnquePtr    = (TRB *)(&(TRBRing[0]));
}

//_________________________________________________________________________________________________

NvBootError NvBootXusbProcessEpStallClearRequest(NvBootUsb3Context *Context, USB_Endpoint_Type EpType )
{
    NvBootError ErrorCode = NvBootError_Success;
    NvBootTRTType TrttTyp = NvBootTRT_NoDataStage;
    DeviceRequest DeviceRequestVar;

    // Clear endpoint stall
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = HOST2DEV_ENDPOINT;// 0x2.
    DeviceRequestVar.bRequest                           = CLEAR_FEATURE;
    DeviceRequestVar.wValue                             = 0; // feature selector.. ENDPOINT_HALT (0x00
    if(EpType == USB_CONTROL_BI)
        DeviceRequestVar.wIndex                         = 0;    //wIndex field endpoint no., and diretion
    else if(EpType == USB_BULK_OUT)
        DeviceRequestVar.wIndex                        = (Context->EPNumAndPacketSize[USB_DIR_OUT].EndpointNumber);    //wIndex field endpoint no., and diretion
    else if(EpType == USB_BULK_IN)
        DeviceRequestVar.wIndex                        = ((Context->EPNumAndPacketSize[USB_DIR_IN].EndpointNumber)
                                                                |= ENDPOINT_DESC_ADDRESS_DIR_IN);
    DeviceRequestVar.wLength                            = 0;    //Number of bytes to return

    ErrorCode = NvBootXusbPrepareControlSetupTRB(Context,&DeviceRequestVar,&TrttTyp);
    if(ErrorCode == NvBootError_Success)
    {
        if (TrttTyp != NvBootTRT_NoDataStage)
        {
            //data stage
            NvBootXusbPrepareControlDataTRB(Context,&DeviceRequestVar);
        }
        NvBootXusbPrepareControlStatusTRB(Context,&DeviceRequestVar);
        // Last TRB with Cycle bit modified for End TRB.
        NvBootXusbPrepareEndTRB(Context);
        NvBootXusbUpdateEpContext(Context,USB_CONTROL_BI);
        ErrorCode = NvBootXusbUpdWorkQAndChkCompQ(Context);
        if(ErrorCode == NvBootError_XusbEpStalled)
            ErrorCode = NvBootError_DeviceResponseError;
    }
    return ErrorCode;
}

//_________________________________________________________________________________________________

NvBootError NvBootXusbProcessControlRequest(NvBootUsb3Context *Context, DeviceRequest *DeviceRequestPtr)
{
    NvBootError ErrorCode = NvBootError_Success;
    NvBootTRTType TrttTyp = NvBootTRT_NoDataStage;
    NvU32 RetryCount = USB_MAX_TXFR_RETRIES;

    while(RetryCount)
    {
        ErrorCode = NvBootXusbPrepareControlSetupTRB(Context,DeviceRequestPtr,&TrttTyp);
        if(ErrorCode == NvBootError_Success)
        {
            if (TrttTyp != NvBootTRT_NoDataStage)
            {
                //data stage
                NvBootXusbPrepareControlDataTRB(Context,DeviceRequestPtr);
            }
            NvBootXusbPrepareControlStatusTRB(Context,DeviceRequestPtr);
            // Last TRB with Cycle bit modified for End TRB.
            NvBootXusbPrepareEndTRB(Context);
            NvBootXusbUpdateEpContext(Context,USB_CONTROL_BI);
            ErrorCode = NvBootXusbUpdWorkQAndChkCompQ(Context);
            if(ErrorCode == NvBootError_Success)
                break;
            if(ErrorCode == NvBootError_XusbEpStalled)
            {
                ErrorCode = NvBootXusbProcessEpStallClearRequest(Context, USB_CONTROL_BI);// get ep dir value
                break;
            }
            else
            {
                RetryCount--;
            }
        }
    }
    return ErrorCode;
}
//_________________________________________________________________________________________________

void InitializeEpContext(NvBootUsb3Context *Context)
{
    EP *EpContext = Context->EndPtContext;

    NvBootUtilMemset((void*)EpContext,0x0, sizeof(EP));
    //As Input, this field is initialized to ‘0’ by software.
//    EpContext->EpDw0.EPState = 0;
//    EpContext->EpDw8.ScratchPad = 0;// what shoudl be value ?
//    EpContext->EpDw9.ScratchPad = 0;// what shoudl be value ?
//    EpContext->EpDw10.RO = 0;// what shoudl be value ?
//    EpContext->EpDw14.END = 0;
//    EpContext->EpDw15.EndpointLinkHo = 0; // only the upper 4 bits
//    EpContext->EpDw12.LPU = 0;// endpoint linked list poitner?

    EpContext->EpDw0.Interval       = ONE_MILISEC;          // 1 ms
    EpContext->EpDw1.CErr           = USB_MAX_TXFR_RETRIES;           // max value
    EpContext->EpDw1.MaxBurstSize   = MAX_BURST_SIZE;       //For High-Speed control and bulk endpoints this field shall be cleared to ‘0’.
    EpContext->EpDw6.CErrCnt = EpContext->EpDw1.CErr;       // AS SET IN EpContext->EpDw1.CErr

    //Initialized to Max Burst Size of the Endpoint by firmware
    EpContext->EpDw7.NumP = MAX_BURST_SIZE;
    //This is the scratchpad used by FW to store EP-related information
    EpContext->EpDw10.TLM           = SYSTEM_MEM;           // assuming system mem
    EpContext->EpDw10.DLM           = SYSTEM_MEM;           // assuming system mem

    EpContext->EpDw11.HubAddr       = Context->HubAddress;  //from context..
    EpContext->EpDw11.RootPortNum   = EP_ROOTPORTNUM(Context->RootPortNum); //from context..
    //This is required only for USB3.0 devices.
    // I believe we use only USB2.0 devices for Boot. You can leave this field 0 for this case.
    // as specified by Mani
    EpContext->EpDw12.Speed = HIGH_SPEED;// needs update whate are the valeus?

//    EpContext->EpDw13.InterrupterTarget = 1;

    //MRK(DW14) = 1
    EpContext->EpDw14.MRK = 1;
    EpContext->EpDw14.ELM = SYSTEM_MEM;// IRAM
    // as specified by Mani, For No FW boot, since we will have only one endpoint context active at a time,
    // link pointer should point to itself (The address at which the endpoint context is created).
    EpContext->EpDw14.EndpointLinkLo = (((NvU32)(Context->EndPtContext)) >> 4);// 28 bit lsb of the TR
}
//_________________________________________________________________________________________________

void NvBootXusbUpdateEpContext( NvBootUsb3Context *Context, USB_Endpoint_Type EpType)
{
    EP *EpContext = Context->EndPtContext;

    // Assert on Not supported EpType
    NV_ASSERT((EpType == USB_CONTROL_BI) ||
                        (EpType == USB_BULK_IN) ||
                        (EpType == USB_BULK_OUT));

    EpContext->EpDw0.EPState                = USB_EP_STATE_RUNNING;
    EpContext->EpDw1.EPType                 = EpType;
    EpContext->EpDw2.DCS                    = Context->CycleBit;
    EpContext->EpDw2.TRDequeuePtrLo         = (((NvU32)(Context->TRBRingCtrlEnquePtr)) >> 4);        // 28 bit lsb of the TR
    EpContext->EpDw3.TRDequeuePtrHi         = 0; // only the upper 4 bits
    //FW(DW10) = Set to 1 for SET_ADDRESS request. 0 for all other cases.
    //0 for SET_ADDRESS. Assigned address later on.
    EpContext->EpDw11.DevAddr               = Context->stEnumerationInfo.DevAddr;
    EpContext->EpDw10.FW                    = Context->SetAddress;
    // clear status
    EpContext->EpDw6.Status                 = 0;
    EpContext->EpDw7.DataOffset             = 0;

    //This field represents the endpoint number on the device for which we need to do the transfer.
    //For control endpoint, DCI should be 1. For Bulk IN/OUT endpoints it should
    // have the corresponding endpoint numbers as read from the device descriptor.
    // BY DEFAULT DCI is set for control endpoint

    if (EpType != USB_CONTROL_BI)
    {
        // not control endpoint
       if(EpType == USB_BULK_IN)
        {
            EpContext->EpDw12.DCI           = ((Context->EPNumAndPacketSize[USB_DIR_IN].EndpointNumber) * 2 + 1);
            EpContext->EpDw5.SEQNUM         = Context->BulkSeqNumIn;
        }
        else if(EpType == USB_BULK_OUT)
        {
            EpContext->EpDw12.DCI           = ((Context->EPNumAndPacketSize[USB_DIR_OUT].EndpointNumber) * 2);
            EpContext->EpDw5.SEQNUM         = Context->BulkSeqNumOut;
        }

        // max packet size is byte swapped.. need to check..
        //        EpContext->EpDw1.MaxPacketSize      = Context->EPNumAndPacketSize[tmp].PacketSize;
        EpContext->EpDw1.MaxPacketSize      = USB_TRB_AVERAGE_BULK_LENGTH;
        EpContext->EpDw4.AverageTRBLength   = USB_TRB_AVERAGE_BULK_LENGTH;
    }
    else
    {
        //control endpoint
        EpContext->EpDw12.DCI               = DCI_CTRL; //set it to default value
        EpContext->EpDw1.MaxPacketSize      = Context->stEnumerationInfo.bMaxPacketSize0;
        EpContext->EpDw4.AverageTRBLength   = USB_TRB_AVERAGE_CONTROL_LENGTH;
        EpContext->EpDw5.SEQNUM             = 0;
    }
    return;
}
//_________________________________________________________________________________________________

static void InitializeDataStructures(NvBootUsb3Context *Context, uint8_t RootPortNum)
{
    NvBootUtilMemset((void *)Context,0,sizeof(NvBootUsb3Context));

    // Initialize the Context data structure. Context variable being passed here is actually
    //referring to s_DeviceContext memory area. For more details look at top level bootrom
    //flow
    Context->Usb3BitInfo              = (NvBootUsb3Status*)&BootInfoTable.SecondaryDevStatus[0];
    //Refer to xHCI spec for Endpoint context data structure
    Context->EndPtContext             = EpContext;
    //Always point to start of TRB ring. Since the driver is operating in polling mode
    //there is always one request enqued
    Context->TRBRingEnquePtr          = (TRB *)(&(TRBRing[0]));
    //used purely by driver. Required to prepare control, data and status TRB
    Context->TRBRingCtrlEnquePtr      = (TRB *)(&(TRBRing[0]));

    /* Not required to be set to 0 since this memory area is already set to 0
    //Cycle bit is always 0. There is no Link TRB and also multiple TRBs are not enqueued
    //so it is not required to be changed at any point of code execution
    Context->CycleBit               = 0;
    */

    //Control packet size before get descriptor device is issued.
    Context->stEnumerationInfo.bMaxPacketSize0 = USB_HS_CONTROL_MAX_PACKETSIZE;

    /* Not required to be set to 0 since this memory area is already set to 0
    //DevAddr is 0 before set address is sent
    Context->stEnumerationInfo.DevAddr = 0;     // update during set address
    Context->HubAddress                = 0;     // needs to be updated
    Context->SlotId                    = 0;     // needs to be updated
    */
    Context->RootPortNum               = RootPortNum;
    //Flag to indicate that set address request is not yet sent or successfully executed
    Context->SetAddress                = NV_FALSE;

    InitializeEpContext(Context);
    // Stash the pointer to the context structure.
    s_Usb3Context = Context;
    Context->Usb3BitInfo->Usb3Context = (NvU32)s_Usb3Context;
    return;
}
//_________________________________________________________________________________________________

void NvBootXusbUpdateWorkQ(NvBootUsb3Context *Context)
{
    NvU32 RegOffset =0, BusInstanceOffset = 0;
    NvU32 value = 0;

    // Port0,1 => Bus Instance 0
    // Port 2 => Bus Instance 1
    BusInstanceOffset = BI_OFFSET(Context->RootPortNum);

    // program the PTRHI with 4 bit of MSB EndpointContext
    // 1. #define NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD2_PTRHI
    NV_XUSB_CSB_EXPAND(HS_BI_WORKQ_DWRD2, RegOffset);
    
    PciRegUpdate(Context, RegOffset+BusInstanceOffset, XUSB_WRITE , &value);

    // set the KIND for DWORD 0
    //#define NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD0_KIND_EPTLIST_BULK_INOUT
    NV_XUSB_CSB_EXPAND(HS_BI_WORKQ_DWRD0, RegOffset);
    value = XUSB_CSB_HS_BI_WORKQ_DWRD0_0_KIND_EPTLIST_BULK_INOUT;
    PciRegUpdate(Context, RegOffset+BusInstanceOffset, XUSB_WRITE , &value);

    // lastly program the #define NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD1
    // #define NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD1_ELM
    //#define NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD1_PTRLO
    NV_XUSB_CSB_EXPAND(HS_BI_WORKQ_DWRD1, RegOffset);
    value = (NvU32)(Context->EndPtContext);
    PciRegUpdate(Context, RegOffset+BusInstanceOffset, XUSB_WRITE , &value);

     Context->Usb3BitInfo->XusbDriverStatus  = NvBootXusbStatus_WorkQSubmitted;
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbCheckCompQ(NvBootUsb3Context *Context)
{
    NvBootError e = NvBootError_Success;
    NvU32 RegOffset =0, BusInstanceOffset=0;
    NvU32 value = 0;
    NvU32 timeout = 0;
    NvU32 EPstatus = 0;
    USB_Endpoint_Type EpType = USB_NOT_VALID;
    EP *EndPtContext = Context->EndPtContext;
    NvU32 StallErr = 0;

    // Port0,1 => Bus Instance 0
    // Port 2 => Bus Instance 1
    BusInstanceOffset = BI_OFFSET(Context->RootPortNum);

    Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollStart;

    // check EP context for status and then check complq register for updates.
    // timeout for 50 microseconds
    // while debugging on fpga this code timeout.. hence increasing this to 1 sec.
    timeout = TIMEOUT_1S;
    while( timeout)
    {
        // poll for completion status until NV_PROJ__XUSB_CSB_SS_BI_COMPLQ_DWRD3_STATUS_COMPL_RETIRE or
        EPstatus = (NvU32)(EndPtContext->EpDw6.Status);
        // check for completion status enum are not defined for HS intentionally since the same is defined in SS.
        if((EPstatus != XUSB_CSB_SS_BI_COMPLQ_DWRD3_0_STATUS_NONE) && (EPstatus <  XUSB_CSB_SS_BI_COMPLQ_DWRD3_0_STATUS_RESCH_CSW))
            break;
        NvBootUtilWaitUS(TIMEOUT_1US);
        timeout--;
    }

    // check for completion status with SS enums
    if((EPstatus != XUSB_CSB_SS_BI_COMPLQ_DWRD3_0_STATUS_COMPL_RETIRE) && (!timeout))
    {
        // update another status entry for NvBootXusbStatus_CompQPollError
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollError;

        // popping the entry to clear for next transfer..//confirm the sequence from ASIC team..
        NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
        value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);

        return NvBootError_HwTimeOut;
    }
    if(EPstatus == XUSB_CSB_SS_BI_COMPLQ_DWRD3_0_STATUS_ERR_STALL)
    {
        StallErr = 1;
    }
    // The FW should check the NV_PROJ__XUSB_CSB_HSBI_COMPLQ_CNTRL_VALID bit

   // while debugging on fpga this code timeout.. hence increasing this to 1 sec.
    timeout = CONTROLLER_HW_RETRIES_1SEC;
    while(timeout)
    {
        NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
        value = NV_PF_VAL(XUSB_CSB_HS_BI_COMPLQ_CNTRL_0, VALID , value);
        if(value)
            break;
        timeout--;
        NvBootUtilWaitUS(TIMEOUT_1US);
    }

    if((!value) && (!timeout))
    {
        // update another status entry for NvBootXusbStatus_CompQPollError
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollError;
        return NvBootError_HwTimeOut;
    }

    // check COMPLQ_DWRD0_SUBKIND until SUBKIND_OP_DONE || SUBKIND_EPT_ERROR || SUBKIND_EPT_DONE
    //NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_DWRD0_SUBKIND
    NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_DWRD0, RegOffset);
    PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
    value = NV_PF_VAL(XUSB_CSB_HS_BI_COMPLQ_DWRD0_0, SUBKIND , value);

    //Program the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_CNTRL_POP bit to '1'.

    if(value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_EPT_NRDY)
    {
        e = NvBootError_XusbEpNotReady;
        NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
        value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollEnd;

        // while debugging on fpga this code timeout.. hence increasing this to 1 sec.
        timeout = CONTROLLER_HW_RETRIES_1SEC;
        while(timeout)
        {
            NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_DWRD0, RegOffset);
            PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);

            // check value for no activity!!
            value = NV_PF_VAL(XUSB_CSB_HS_BI_COMPLQ_DWRD0_0, SUBKIND , value);

            if(value != XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_EPT_NRDY)
            {
                //Program the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_CNTRL_POP bit to '1'.
                NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
                PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
                value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
                PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);
                break;
            }
            timeout--;
            NvBootUtilWaitUS(TIMEOUT_10US);
        }
        if(!timeout)
        {
            e = NvBootError_XusbEpNotReady;
        }
    else
    {
           e = NvBootError_Success;
    }
    }
    else if((value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_OP_ERROR) ||
        (value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_STOPREQ))
    {
        e =  NvBootError_DeviceResponseError;// should be controller specific CHECK
    }
    else if (value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_EPT_ERROR)
    {
        // only on XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_EPT_ERROR perform error handling!!
        e = NvBootError_XusbEpError;// perform error handling!!
    }

    // readh NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_DWRD1
    // NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_DWRD1_EPT_PTRLO
    // NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_DWRD1_EPT_PTRHI
    if(value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_EPT_DONE)
    {
        //Program the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_CNTRL_POP bit to '1'.
        NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
        value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollEnd;

        e = NvBootError_Success;

        // while debugging on fpga this code timeout.. hence increasing this to 1 sec.
        timeout = CONTROLLER_HW_RETRIES_1SEC;
        while(timeout)
        {
            NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_DWRD0, RegOffset);
            PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);

            // check value for no activity!!
            value = NV_PF_VAL(XUSB_CSB_HS_BI_COMPLQ_DWRD0_0, SUBKIND , value);

            if(value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_NO_ACTIVITY)
            {
                //Program the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_CNTRL_POP bit to '1'.
                NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
                PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
                value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
                PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);
                break;
            }
            timeout--;
            NvBootUtilWaitUS(TIMEOUT_1US);
        }

        if(!timeout)
        {
            e = NvBootError_HwTimeOut;
        }
        else
        {
            //update sequence no.,
            EpType = (USB_Endpoint_Type)(EndPtContext->EpDw1.EPType);

            if(EpType == USB_BULK_OUT)
            {
                if(StallErr)
                    Context->BulkSeqNumOut = 0;
                else
                    Context->BulkSeqNumOut = (NvU32)(EndPtContext->EpDw5.SEQNUM);
            }
            else if (EpType == USB_BULK_IN)
            {
                if(StallErr)
                    Context->BulkSeqNumIn = 0;
                else
                    Context->BulkSeqNumIn= (NvU32)(EndPtContext->EpDw5.SEQNUM);
            }
        }
    }
    else
    {
        // should not be here..
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollError;
        //Program the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_CNTRL_POP bit to '1'.
        NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
        value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
        PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollEnd;

        // while debugging on fpga this code timeout.. hence increasing this to 1 sec.
        timeout = CONTROLLER_HW_RETRIES_1SEC;
        while(timeout)
        {
            NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_DWRD0, RegOffset);
            PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);

            // check value for no activity!!
            value = NV_PF_VAL(XUSB_CSB_HS_BI_COMPLQ_DWRD0_0, SUBKIND , value);

            if(value == XUSB_CSB_HS_BI_COMPLQ_DWRD0_0_SUBKIND_NO_ACTIVITY)
            {
                //Program the NV_PROJ__XUSB_CSB_HS_BI_COMPLQ_CNTRL_POP bit to '1'.
                NV_XUSB_CSB_EXPAND(HS_BI_COMPLQ_CNTRL, RegOffset);
                PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_READ, &value);
                value = NV_FLD_SET_DRF_DEF(XUSB_CSB, HS_BI_COMPLQ_CNTRL,POP, SET, value);
                PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &value);
                break;
            }
            timeout--;
            NvBootUtilWaitUS(TIMEOUT_1US);
        }

        if(!timeout)
        {
            e = NvBootError_HwTimeOut;
        }
    }
    // delay between ep status done to iram content access
    NvBootUtilWaitUS(TIMEOUT_10US);
    if(StallErr)
            e = NvBootError_XusbEpStalled;
    return e;
}

static NvBootError UsbTransferErrorHandling(NvBootUsb3Context *Context)
{
    NvBootError e = NvBootError_Success;
    NvU32 RegOffset =0;
    NvU32 value = 0;
    NvU32 timeout = 0;
    TRB EventTrb;
    EventTRB EventDataTrb;
    // The BI write a transfer event TRB to the common event queue upon any
    //error condition in a transaction. It is the responsibility of the bootrom
    // code to read this queue upon error detection in the completion status.
    //Possible error completion codes include "Babble Detected Error",
    // "USB Transaction Error", and "Stall Error
    NvBootUtilMemset((void *)&EventTrb,0,sizeof(TRB));
    
    Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_EventQPollStart;

    // poll NV_PROJ__XUSB_CSB_ EVENTQ_CNTRL_VALID to check for a valid entry
    // while debugging on fpga this code timeout.. hence increasing this to 1 sec.
    timeout = CONTROLLER_HW_RETRIES_1SEC;

    while(timeout)
    {
        NV_XUSB_CSB_EXPAND(EVENTQ_CNTRL1, RegOffset);
        PciRegUpdate(Context, RegOffset, XUSB_READ, &value);
        value = NV_PF_VAL(XUSB_CSB_EVENTQ_CNTRL1_0, VALID , value);

        if(value)
            break;
        timeout--;
        NvBootUtilWaitUS(TIMEOUT_1US);
    }

    if((!value) && (!timeout))
    {
        // update another status entry for NvBootXusbStatus_CompQPollError
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_CompQPollError;
        return NvBootError_HwTimeOut;
    }

    // HW event ring! One 1 event ring even with 2 BIs
    // Read NV_PROJ__XUSB_CSB_ EVENTQ_TRBDWRD0-3 to get the transfer event TRB.
    NV_XUSB_CSB_EXPAND(EVENTQ_TRBDWRD0, RegOffset);
    PciRegUpdate(Context, RegOffset, XUSB_READ, &EventTrb.DataBuffer[0]);

    NV_XUSB_CSB_EXPAND(EVENTQ_TRBDWRD1, RegOffset);
    PciRegUpdate(Context, RegOffset, XUSB_READ, &EventTrb.DataBuffer[1]);

    NV_XUSB_CSB_EXPAND(EVENTQ_TRBDWRD2, RegOffset);
    PciRegUpdate(Context, RegOffset, XUSB_READ, &EventTrb.DataBuffer[2]);

    NV_XUSB_CSB_EXPAND(EVENTQ_TRBDWRD3, RegOffset);
    PciRegUpdate(Context, RegOffset, XUSB_READ, &EventTrb.DataBuffer[3]);

    // mapping the data event trb
    NvBootUtilMemcpy((void *)(&EventDataTrb), (void*)(&EventTrb), sizeof(TRB));
    
    // Only 1 event ring so only 1 eventQ even with 2 BIs
    if(EventDataTrb.TRBType != NvBootTRB_EventData)
    {
        // not expected type TRB.
        NV_XUSB_CSB_EXPAND(EVENTQ_CNTRL1, RegOffset);
        PciRegUpdate(Context, RegOffset, XUSB_READ, &value);
        value = NV_FLD_SET_DRF_DEF(XUSB_CSB, EVENTQ_CNTRL1,POP, SET, value);
        PciRegUpdate(Context, RegOffset, XUSB_WRITE , &value);
        Context->Usb3BitInfo->EpStatus = NvBootComplqCode_USB_TRANS_ERR;
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_EventQPollError;
        //error retry..
        e = NvBootError_XusbEpRetry;
    }
    else
    {
        // Set NV_PROJ__XUSB_CSB_ EVENTQ_CNTRL_POP to '1' to remove event TRB from the queue.
        NV_XUSB_CSB_EXPAND(EVENTQ_CNTRL1, RegOffset);
        PciRegUpdate(Context, RegOffset, XUSB_READ, &value);
        value = NV_FLD_SET_DRF_DEF(XUSB_CSB, EVENTQ_CNTRL1,POP, SET, value);
        PciRegUpdate(Context, RegOffset, XUSB_WRITE , &value);

        // check for the event type to NvBootTRB_TransferEvent
        // update the exact error type as described in 6.4.5 xhci spec.
        Context->Usb3BitInfo->XusbDriverStatus = NvBootXusbStatus_EventQPollEnd;
        // update complettion code
        // report complettion code error status
        Context->Usb3BitInfo->EpStatus = EventDataTrb.ComplCode;
    }
    return e;
}

//_________________________________________________________________________________________________
NvBootError NvBootXusbUpdWorkQAndChkCompQ(NvBootUsb3Context *Context)
{
    NvBootError e;

    // update workQ
    NvBootXusbUpdateWorkQ(Context);

    NvBootUtilWaitUS(TIMEOUT_10US);

    // poll for compq
    e = NvBootXusbCheckCompQ(Context);

    if(e == NvBootError_XusbEpError)
    {
        // Error handling code
        e = UsbTransferErrorHandling(Context);
        if((e == NvBootError_Success) &&
            (Context->Usb3BitInfo->XusbDriverStatus == NvBootXusbStatus_EventQPollEnd))
        {
            if(Context->Usb3BitInfo->EpStatus  == NvBootComplqCode_STALL_ERR)
            {
                // clear endpoint stall
                e = NvBootError_XusbEpStalled;
            }
            else if(Context->Usb3BitInfo->EpStatus  == NvBootComplqCode_USB_TRANS_ERR)
            {
                e = NvBootError_XusbEpRetry;
            }
        }
    }
    else if((e == NvBootError_HwTimeOut ) ||
                (e == NvBootError_DeviceResponseError) ||
                (e == NvBootError_XusbEpNotReady))
    {
        // hw timeout/EP not ready/ controller error
        // retry the transfer
        e = NvBootError_XusbEpRetry;
    }
    return e;
}
//_________________________________________________________________________________________________

static NvBootError ParseConfigurationDescriptor(NvBootUsb3Context *Context)
{
    NvBootError ErrorCode = NvBootError_XusbParseConfigDescFail;
    UsbInterfaceDescriptor *UsbInterfaceDescriptorPtr;
    UsbEndpointDescriptor  *UsbEndpointDescriptorPtr;
    NvU32 NumInterfaces, NumEndPoints, i,j,EndpointDir;
    NvBool EndpointInitStatus[2];

    //Initialize to 0
    NumEndPoints = 0;
    // Get ptr to interface descriptor
    UsbInterfaceDescriptorPtr = (UsbInterfaceDescriptor *)( ((uint8_t *)(BufferXusbData)) + UsbConfigDescriptorStructSz );
    //get number of supported interfaces
    NumInterfaces =((UsbConfigDescriptor *)(BufferXusbData))->bNumInterfaces;
    //Get endpoint descriptor for current interface descriptor
    UsbEndpointDescriptorPtr  = (UsbEndpointDescriptor *)(((uint8_t *)UsbInterfaceDescriptorPtr) + UsbInterfaceDescriptorStructSz);


    //Check end point descriptors for a given interface till bulk only IN and OUT type endpoints are found
    //LOOP1
    for (i = 0; ((i < NumInterfaces) && (ErrorCode != NvBootError_Success)) ; i++)
    {
        //get next interface descriptor pointer. Interface Descriptor is followed by array of Endpoint descriptor of size NumEndPoints
        //Value of NumEndPoints used is the value from previous interface Descriptor. In the beginining of loop it is 0
        UsbInterfaceDescriptorPtr = (UsbInterfaceDescriptor *)(((uint8_t *)UsbInterfaceDescriptorPtr) + NumEndPoints * UsbEndpointDescriptorStructSz);
        //Get supported number of endpoints for current interface descriptor
        NumEndPoints              = UsbInterfaceDescriptorPtr->bNumEndpoints;
        //Get endpoint descriptor for current interface descriptor
        UsbEndpointDescriptorPtr  = (UsbEndpointDescriptor *)(((uint8_t *)UsbInterfaceDescriptorPtr) + UsbInterfaceDescriptorStructSz);

        //if current interface class is mass storage and interface protocol is bulk only transport then
        //check endpoint descriptor for valid data else check next interface descriptor if there is any
        if((UsbInterfaceDescriptorPtr->bInterfaceClass == INTERFACE_CLASS_MASS_STORAGE) && \
            (UsbInterfaceDescriptorPtr->bInterfaceProtocol == INTERFACE_PROTOCOL_BULK_ONLY_TRANSPORT))
        {
            //Analyze Endpoint Descriptor
            //See if two end points one of type bulk in and other of type bulk out is found
            EndpointInitStatus[USB_DIR_OUT] = NV_FALSE;
            EndpointInitStatus[USB_DIR_IN]  = NV_FALSE;

            //LOOP2
            for(j = 0; j < NumEndPoints; j++)
            {
                if(UsbEndpointDescriptorPtr->bmAttributes == ENDPOINT_DESC_ATTRIBUTES_BULK_TYPE)
                {
                    //Bulk type end point
                    EndpointDir = (UsbEndpointDescriptorPtr->bEndpointAddress) & ENDPOINT_DESC_ADDRESS_DIR_MASK;
                    if(EndpointDir == ENDPOINT_DESC_ADDRESS_DIR_IN)
                    {
                        EndpointDir = USB_DIR_IN;
                    }
                    else
                    {
                        EndpointDir = USB_DIR_OUT;
                    }
                    Context->EPNumAndPacketSize[EndpointDir].PacketSize =    (UsbEndpointDescriptorPtr->wMaxPacketSize);
                    Context->EPNumAndPacketSize[EndpointDir].EndpointNumber = (UsbEndpointDescriptorPtr->bEndpointAddress) & ENDPOINT_DESC_ADDRESS_ENDPOINT_MASK;
                    EndpointInitStatus[EndpointDir] = NV_TRUE;

                    if((EndpointInitStatus[USB_DIR_OUT]== NV_TRUE) && (EndpointInitStatus[USB_DIR_IN] == NV_TRUE))
                    {
                        //Found IN as well as OUT endpoint
                        //Log interface number
                        Context->stEnumerationInfo.InterfaceIndx = UsbInterfaceDescriptorPtr->bInterfaceNumber;
                        ErrorCode = NvBootError_Success; // This error code is required for calling function as well as for coming out of LOOP1
                        break; // Come out of LOOP2
                    }
                }
                //Check next endpoint descriptor
                UsbEndpointDescriptorPtr = (UsbEndpointDescriptor *)((uint8_t *)UsbEndpointDescriptorPtr + UsbEndpointDescriptorStructSz);
            }
        }
        else
        {
            //Check next interface if there is any
            continue;
        }
    }
    return ErrorCode;
}
//_________________________________________________________________________________________________

// Initialize scsi commands (inquiry, read capacity/capacities, request sense, mode sense & test unit ready)
NvBootError InitializeScsiMedia(NvBootUsb3Context *Context)
{
    NvBootError e = NvBootError_Success;

    // Issue Inquiry Command
    NV_BOOT_CHECK_ERROR(NvBootXusbMscBotProcessRequest(Context, INQUIRY_CMD_OPCODE));
    NVBOOT_MSG((Q_USB3BOOT_SCSI_INQUIRY));

    // test unit ready
    e = NvBootXusbMscBotProcessRequest(Context, TESTUNITREADY_CMD_OPCODE);

    if(e == NvBootError_Success)
    {
        NVBOOT_MSG((Q_USB3BOOT_SCSI_TEST));
        //Read Capacity
        e  = NvBootXusbMscBotProcessRequest(Context, READ_CAPACITY_CMD_OPCODE);

        if (e  == NvBootError_Success)
        {
            NVBOOT_MSG((Q_USB3BOOT_SCSI_READ_CAPACITY));
            // Test unit command is issued for check the interface / device state
            e  = NvBootXusbMscBotProcessRequest(Context, TESTUNITREADY_CMD_OPCODE);
        }
        else
        {
            NVBOOT_MSG((Q_USB3BOOT_SCSI_REQ_SENSE));
            //Issue request sense command to diagnose read capacity failure
            e = NvBootXusbMscBotProcessRequest(Context, REQUESTSENSE_CMD_OPCODE);
        }
    }
    else
    {
        NVBOOT_MSG((Q_USB3BOOT_SCSI_REQ_SENSE));
        //Issue request sense command to diagnose read format capacity failure
        e  = NvBootXusbMscBotProcessRequest(Context, REQUESTSENSE_CMD_OPCODE);
    }
    return e;
}
//_________________________________________________________________________________________________

NvBootError EnumerateDevice(NvBootUsb3Context *Context)
{
    NvBootError e = NvBootError_Success;
    DeviceRequest DeviceRequestVar;


    // Descriptor type in the high byte and the descriptor index
    // in the low byte. The descriptor index is used to select a
    // specific descriptor (only for configuration and string descriptors)
    // wIndex field specifies the Language ID for string descriptors
    // or is reset to zero for other descriptors

    //Get device descriptor @ control endpoint 0
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = DEV2HOST_DEVICE;
    DeviceRequestVar.bRequest                           = GET_DESCRIPTOR;
    DeviceRequestVar.wValue                             = USB_DT_DEVICE << 8;//Descriptor type in the high byte and the descriptor index
                                                                            //in the low byte. The descriptor index is used to select a specific descriptor
                                                                            //(only for configuration and string descriptors)
    DeviceRequestVar.wIndex                             = 0;                //wIndex field specifies the Language ID for string descriptors or is reset to zero for other descriptors
    DeviceRequestVar.wLength                            = USB_DEV_DESCRIPTOR_SIZE; //Number of bytes to return
    NV_BOOT_CHECK_ERROR(NvBootXusbProcessControlRequest(Context, &DeviceRequestVar));

    //update mps.
    Context->stEnumerationInfo.bMaxPacketSize0          = ((UsbDeviceDescriptor *)(BufferXusbData))->bMaxPacketSize0;

    //------------------------------------------------------------
    // set address
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = HOST2DEV_DEVICE;
    DeviceRequestVar.bRequest                           = SET_ADDRESS;
    DeviceRequestVar.wValue                             =  XUSB_DEV_ADDR; //specifies the device address to use for all subsequent accesses.
    DeviceRequestVar.wIndex                             =  0;
    DeviceRequestVar.wLength                            =  0;               //No data stage

    Context->SetAddress = NV_TRUE;
    NV_BOOT_CHECK_ERROR(NvBootXusbProcessControlRequest(Context, &DeviceRequestVar));

    // udpate context with dev Address and clear SetAddress
    Context->stEnumerationInfo.DevAddr                  = XUSB_DEV_ADDR;
    Context->SetAddress                                 = NV_FALSE;
    //---------------------------------------------------------------
    // get device descriptor with addressed control endpoint 0
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = DEV2HOST_DEVICE;
    DeviceRequestVar.bRequest                           = GET_DESCRIPTOR;
    DeviceRequestVar.wValue                             = USB_DT_DEVICE << 8;//Descriptor type in the high byte and the descriptor index
                                                                            //in the low byte. The descriptor index is used to select a specific descriptor
                                                                            //(only for configuration and string descriptors)
    DeviceRequestVar.wIndex                             = 0;                //wIndex field specifies the Language ID for string descriptors or is reset to zero for other descriptors
    DeviceRequestVar.wLength                            = USB_DEV_DESCRIPTOR_SIZE; //Number of bytes to return
    NV_BOOT_CHECK_ERROR(NvBootXusbProcessControlRequest(Context, &DeviceRequestVar));

    //update mps and configuration
    Context->stEnumerationInfo.bMaxPacketSize0          = ((UsbDeviceDescriptor *)(BufferXusbData))->bMaxPacketSize0;
    Context->stEnumerationInfo.bNumConfigurations       = ((UsbDeviceDescriptor *)(BufferXusbData))->bNumConfigurations;
    //-------------------------------------------------------------------------
    // get config descriptor only!!
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = DEV2HOST_DEVICE;
    DeviceRequestVar.bRequest                           = GET_DESCRIPTOR;
    DeviceRequestVar.wValue                             = USB_DT_CONFIG << 8;//Descriptor type in the high byte and the descriptor index
                                                                            //in the low byte. The descriptor index is used to select a specific descriptor
                                                                            //(only for configuration and string descriptors)
    DeviceRequestVar.wIndex                             = 0;                //wIndex field specifies the Language ID for string descriptors or is reset to zero for other descriptors
    DeviceRequestVar.wLength                            = USB_CONFIG_DESCRIPTOR_SIZE; //Number of bytes to return

    NV_BOOT_CHECK_ERROR(NvBootXusbProcessControlRequest(Context, &DeviceRequestVar));
    //update configuration value.
    Context->stEnumerationInfo.bConfigurationValue      = ((UsbConfigDescriptor *)(BufferXusbData))->bConfigurationValue;

    //------------------------------------------------------------------------
    // get complete config descriptor
    //All other fields remain same as above
    DeviceRequestVar.wLength                            = ((UsbConfigDescriptor *)(BufferXusbData))->wTotalLength;; //Number of bytes to return

    NV_BOOT_CHECK_ERROR(NvBootXusbProcessControlRequest(Context, &DeviceRequestVar));

    // parse configuration descriptor for configuration descriptor full length value
    // update bulk endpoints no., in context etc
    // check for MSD support
    // if the MSD device enable configuration else return
     NV_BOOT_CHECK_ERROR(ParseConfigurationDescriptor(Context));

    //---------------------------------------------------------------------------------
    // set configuration !
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = HOST2DEV_DEVICE;
    DeviceRequestVar.bRequest                           = SET_CONFIGURATION;
    DeviceRequestVar.wValue                             = Context->stEnumerationInfo.bConfigurationValue;
    DeviceRequestVar.wIndex                             = 0;
    DeviceRequestVar.wLength                            = 0;

    NV_BOOT_CHECK_ERROR(NvBootXusbProcessControlRequest(Context, &DeviceRequestVar));
    NVBOOT_MSG((Q_USB3BOOT_DEV_CONFIGURED));

    //----------------------------------------------------------------------------------
    // perform class specifc request
    // get max lun
    // Get max lun with addressed endpoint 0
    //The device shall return one byte of data that contains the maximum LUN supported by the device. For example,
    //if the device supports four LUNs then the LUNs would be numbered from 0 to 3 and the return value would be
    // If no LUN is associated with the device, the value returned shall be 0. The host shall not send a command
    //block wrapper (CBW) to a non-existing LUN. Devices that do not support multiple LUNs may STALL this command.

    //TODO : What if device does not support max lun? Is LUN=0 good enough for BOT commands?
    DeviceRequestVar.bmRequestTypeUnion.bmRequestType   = CLASS_SPECIFIC_REQUEST;
    DeviceRequestVar.bRequest                           = GET_MAX_LUN;
    DeviceRequestVar.wValue                             = 0;
    DeviceRequestVar.wIndex                             = Context->stEnumerationInfo.InterfaceIndx;
    DeviceRequestVar.wLength                            = USB_GET_MAX_LUN_LENGTH;
    NvBootXusbProcessControlRequest(Context, &DeviceRequestVar);
    Context->stEnumerationInfo.LUN                      =  *((uint8_t *)(BufferXusbData)); // read one byte
    NVBOOT_MSG((Q_USB3BOOT_DEV_MAXLUN));
    return e;
}
//_________________________________________________________________________________________________

void XusbPadCtrlInit(const NvBootUsb3Params *Params, NvBootUsb3Context *Context)
{
    NvU32 RootPortAttached;
    NvU32 RegData;
    NvU32 HSTermRangeAdj;
    NvU32 HS_Curr_level;
    NvU32 RpdCtrl;
    NvU32 RegVal;

    (void)Context;
    // Configure the USB phy stabilization delay setting.
    NvBootClocksOscFreq OscFreq;

    // read the Boot media register
    NV_XUSB_PADCTL_READ( BOOT_MEDIA, RootPortAttached);
 
    // enable USB boot media bit
    RootPortAttached = NV_FLD_SET_DRF_DEF(XUSB_PADCTL, BOOT_MEDIA, \
                                    BOOT_MEDIA_ENABLE, YES, RootPortAttached);
    // set the root port value from params
    RootPortAttached |= NV_DRF_NUM( XUSB_PADCTL, BOOT_MEDIA, BOOT_PORT,\
                                    Params->RootPortNumber);
    // update the boot media register
    NV_XUSB_PADCTL_WRITE( BOOT_MEDIA, RootPortAttached);

    // read the pad mux register
    NV_XUSB_PADCTL_READ( USB2_PAD_MUX, RegVal);
	
    // @ 7009f000
    //  Clear the following to disable PAD power down, where N is from 0 to 3.
    //  PD_CHG field in XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPADN_CTL0_0
    //  PD_ZI, PD2, and PD fields in XUSB_PADCTL_USB2_OTG_PADN_CTL_0_0
    //  PD_DR field in XUSB_PADCTL_USB2_OTG_PADN_CTL_1_0
   if(Params->RootPortNumber == USB_BOOT_PORT_OTG0)
   {
	   // set the pad ownership from params
	   RegVal = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT0, XUSB, RegVal);

	   RegVal = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_BIAS_PAD, XUSB, RegVal);
	   // update the pad ownership register
	   NV_XUSB_PADCTL_WRITE( USB2_PAD_MUX, RegVal);

	   //USB Pad protection circuit activation bug #1500052 
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD0_CTL1, RegData);
       // FIXME VREGS
       RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD0_CTL1, PD_VREG, 1, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD0_CTL1, RegData);

	   //  Clear the following to disable PAD power down, where N is 0 .
	   //  PD_CHG field in XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD0_CTL0_0
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD0_CTL0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD0_CTL0, PD_CHG, NO, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD0_CTL0, RegData);
	   // @ 7009f080
      
	   // PD_ZI, PD2, and PD fields in XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD0_CTL_0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD_ZI, SW_DEFAULT, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, PD, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD0_CTL_0, RegData);
	   // @ 7009f088
   
	   // PD_DR field in XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0 and
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD0_CTL_1, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, PD_DR, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD0_CTL_1, RegData);
	   // @ 7009f08c

           // PD ownership only for port0.
           //PD field in XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0
           NV_XUSB_PADCTL_READ( USB2_BIAS_PAD_CTL_0, RegData);
           RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_BIAS_PAD_CTL_0, PD, SW_DEFAULT, RegData);
           NV_XUSB_PADCTL_WRITE( USB2_BIAS_PAD_CTL_0, RegData);
           // @ 7009f284
           
           // port 0 tracking code.
           OscFreq = NvBootClocksGetOscFreq();
           NvBootXusbDevicePerformTracking(OscFreq);
   
   }
   else if(Params->RootPortNumber == USB_BOOT_PORT_OTG1)
   {
	   // set the pad ownership from params
	   RegVal = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT1, XUSB, RegVal);
	   // update the pad ownership register
	   NV_XUSB_PADCTL_WRITE( USB2_PAD_MUX, RegVal);

           //USB Pad protection circuit activation bug #1500052 
           NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD1_CTL1, RegData);
           RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD1_CTL1, PD_VREG, 1, RegData);
           NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD1_CTL1, RegData);
    
	   //  Clear the following to disable PAD power down, where N is 1.
	   //  PD_CHG field in XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD1_CTL0_0
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD1_CTL0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD1_CTL0, PD_CHG, NO, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD1_CTL0, RegData);
   
   
	   // PD_ZI, PD2, and PD fields in XUSB_PADCTL_USB2_OTG_PAD1_CTL_0_0
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD1_CTL_0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD1_CTL_0, PD_ZI, SW_DEFAULT, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD1_CTL_0, PD, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD1_CTL_0, RegData);
	   // @ 7009f0c8
   
	   // PD_DR field in XUSB_PADCTL_USB2_OTG_PAD1_CTL_1_0 and
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD1_CTL_1, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD1_CTL_1, PD_DR, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD1_CTL_1, RegData);
		// @ 7009f0cc   
   }
   else if(Params->RootPortNumber == USB_BOOT_PORT_OTG2)
   {
	   // set the pad ownership from params
	   RegVal = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT2, XUSB, RegVal);
	   // update the pad ownership register
	   NV_XUSB_PADCTL_WRITE( USB2_PAD_MUX, RegVal);
	   //USB Pad protection circuit activation bug #1500052 
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD2_CTL1, RegData);
	   RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD2_CTL1, PD_VREG, 1, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD2_CTL1, RegData);

	   //  Clear the following to disable PAD power down, where N is 2.
	   //  PD_CHG field in XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD2_CTL0_0
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD2_CTL0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD2_CTL0, PD_CHG, NO, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD2_CTL0, RegData);
	   // @ 7009f100 
      
	   // PD_ZI, PD2, and PD fields in XUSB_PADCTL_USB2_OTG_PAD2_CTL_0_0
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD2_CTL_0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD2_CTL_0, PD_ZI, SW_DEFAULT, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD2_CTL_0, PD, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD2_CTL_0, RegData);
	   // @ 7009f108 
   
	   // PD_DR field in XUSB_PADCTL_USB2_OTG_PAD2_CTL_1_0 and
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD2_CTL_1, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD2_CTL_1, PD_DR, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD2_CTL_1, RegData);
	   // @ 7009f10c

   }
   else if(Params->RootPortNumber == USB_BOOT_PORT_OTG3)
   {
	   // set the pad ownership from params
	   RegVal = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT3, XUSB, RegVal);
	   // update the pad ownership register
	   NV_XUSB_PADCTL_WRITE( USB2_PAD_MUX, RegVal);

	   //USB Pad protection circuit activation bug #1500052 
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD3_CTL1, RegData);
	   RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD3_CTL1, PD_VREG, 1, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD3_CTL1, RegData);


	   //  Clear the following to disable PAD power down, where N is 3.
	   //  PD_CHG field in XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD3_CTL0_0
	   NV_XUSB_PADCTL_READ( USB2_BATTERY_CHRG_OTGPAD3_CTL0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_BATTERY_CHRG_OTGPAD3_CTL0, PD_CHG, NO, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_BATTERY_CHRG_OTGPAD3_CTL0, RegData);
	   // @ 7009f140 
   
   
	   // PD_ZI, PD2, and PD fields in XUSB_PADCTL_USB2_OTG_PAD3_CTL_0_0
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD3_CTL_0, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD3_CTL_0, PD_ZI, SW_DEFAULT, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD3_CTL_0, PD, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD3_CTL_0, RegData);
	   // @ 7009f148
	   // PD_DR field in XUSB_PADCTL_USB2_OTG_PAD3_CTL_1_0 and
	   NV_XUSB_PADCTL_READ( USB2_OTG_PAD3_CTL_1, RegData);
	   RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, USB2_OTG_PAD3_CTL_1, PD_DR, SW_DEFAULT, RegData);
	   NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD3_CTL_1, RegData);
	   // @ 7009f14c

   }

	
    
    //  Read FUSE_USB_CALIB_0 register to set the following fields, where N is determined by BOOT_PORT.
    //  HS_CURR_LEVEL field in XUSB_PADCTL_USB2_OTG_PADN_CTL_0 register to USB_CALIB[5:0]
    //  TERM_RANGE_ADJ fields in XUSB_PADCTL_USB2_OTG_PADN_CTL_1 register to USB_CALIB[10:7]
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_0);
    RegVal  = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_USB_CALIB_EXT_0);

    RpdCtrl = NV_DRF_VAL(FUSE, USB_CALIB_EXT, RPD_CTRL, RegVal); //RPD_CTRL
    HSTermRangeAdj = NV_DRF_VAL(FUSE, USB_CALIB, TERM_RANGE_ADJ, RegData); //HS_TERM_RANGE_ADJ

    if(Params->RootPortNumber == USB_BOOT_PORT_OTG0)
    {
        HS_Curr_level = NV_DRF_VAL(FUSE, USB_CALIB, HS_CURR_LEVEL_P0, RegData); //HS_CURR_LEVEL
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD0_CTL_0, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_0, HS_CURR_LEVEL, HS_Curr_level, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD0_CTL_0, RegData);
		// @ 7009f088

        // RPD_CTRL and TERM_RANGE_ADJ 
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD0_CTL_1, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, TERM_RANGE_ADJ, HSTermRangeAdj, RegData);
        //XUSB_PADCTL_USB2_OTG_PADX_CTL_1_0[RPD_CTRL] 
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD0_CTL_1, RPD_CTRL, RpdCtrl, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD0_CTL_1, RegData);
		// @ 7009f08c

    }
    else if (Params->RootPortNumber == USB_BOOT_PORT_OTG1)
    {
        HS_Curr_level = NV_DRF_VAL(FUSE, USB_CALIB, HS_CURR_LEVEL_P1, RegData); //HS_CURR_LEVEL
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD1_CTL_0, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD1_CTL_0, HS_CURR_LEVEL, HS_Curr_level, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD1_CTL_0, RegData);
		// @ 7009f0c8

        // RPD_CTRL and TERM_RANGE_ADJ 
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD1_CTL_1, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD1_CTL_1, TERM_RANGE_ADJ, HSTermRangeAdj, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD1_CTL_1, RPD_CTRL, RpdCtrl, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD1_CTL_1, RegData);
		// @ 7009f0cc
    }
    else if (Params->RootPortNumber == USB_BOOT_PORT_OTG2)
    {
        HS_Curr_level = NV_DRF_VAL(FUSE, USB_CALIB, HS_CURR_LEVEL_P2, RegData); //HS_CURR_LEVEL
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD2_CTL_0, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD2_CTL_0, HS_CURR_LEVEL, HS_Curr_level, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD2_CTL_0, RegData);

        // RPD_CTRL and TERM_RANGE_ADJ 
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD2_CTL_1, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD2_CTL_1, TERM_RANGE_ADJ, HSTermRangeAdj, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD2_CTL_1, RPD_CTRL, RpdCtrl, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD2_CTL_1, RegData);
		// @ 7009f10c 
    }
    else if (Params->RootPortNumber == USB_BOOT_PORT_OTG3)
    {
        HS_Curr_level = NV_DRF_VAL(FUSE, USB_CALIB, HS_CURR_LEVEL_P3, RegData); //HS_CURR_LEVEL
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD3_CTL_0, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD3_CTL_0, HS_CURR_LEVEL, HS_Curr_level, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD3_CTL_0, RegData);
		// @ 7009f148

        // RPD_CTRL and TERM_RANGE_ADJ 
        NV_XUSB_PADCTL_READ( USB2_OTG_PAD3_CTL_1, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD3_CTL_1, TERM_RANGE_ADJ, HSTermRangeAdj, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OTG_PAD3_CTL_1, RPD_CTRL, RpdCtrl, RegData);
        NV_XUSB_PADCTL_WRITE( USB2_OTG_PAD3_CTL_1, RegData);
	   // @ 7009f14c
    }

	// Set PORTN_OC_PIN field in XUSB_PADCTL_USB2_OC_MAP_0 register to '0xF', where N is determined by BOOT_PORT
    NV_XUSB_PADCTL_READ( USB2_OC_MAP, RegData);

    //Padctrl: XUSB_PADCTL_USB2_PORT_CAP_0
    //Peatrans.MemWr(0x7009f008,0x00001111);
	
    NV_XUSB_PADCTL_READ( USB2_PORT_CAP, RegVal);

    if(Params->RootPortNumber == USB_BOOT_PORT_OTG0)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT0_OC_PIN, 0xF, RegData);
        RegVal = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_PORT_CAP, PORT0_CAP, 0x1, RegVal);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG1)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT1_OC_PIN, 0xF, RegData);
        RegVal = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_PORT_CAP, PORT1_CAP, 0x1, RegVal);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG2)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT2_OC_PIN, 0xF, RegData);
        RegVal = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_PORT_CAP, PORT2_CAP, 0x1, RegVal);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG3)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT3_OC_PIN, 0xF, RegData);
        RegVal = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_PORT_CAP, PORT3_CAP, 0x1, RegVal);
    }
    NV_XUSB_PADCTL_WRITE( USB2_OC_MAP, RegData);
	// @ 7009f010

    NV_XUSB_PADCTL_WRITE( USB2_PORT_CAP, RegVal);

#if NVBOOT_TARGET_FPGA

    NvBootUtilWaitUS(TIMEOUT_10MS);

	//Padctrl: XUSB_PADCTL_USB2_PORT_CAP_0
	//Peatrans.MemWr(0x7009f008,0x00001111); --- NO -- do it
	RegData = 0x00001111;
    NV_XUSB_PADCTL_WRITE( USB2_PORT_CAP, RegData);

	//Padctrl: PORT_MAP  
	//Peatrans.MemWr(0x7009f014,0x00018820);--- NO
	RegData = 0x00018820;
    NV_XUSB_PADCTL_WRITE( SS_PORT_MAP, RegData);

	//Padctrl: elpg
	//Peatrans.MemWr(0x7009f024,0x00000000);--- NO -- not required.
	RegData = 0x00000000;
    NV_XUSB_PADCTL_WRITE( ELPG_PROGRAM_1, RegData);

	
	//Pad Mux
	//Peatrans.MemWr(0x7009f004,0x00040055);--- NO -- do it
    NV_XUSB_PADCTL_READ( USB2_PAD_MUX, RegData);
	RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_PAD_MUX, USB2_OTG_PAD_PORT1, 1, RegData);
	RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_PAD_MUX, USB2_BIAS_PAD, 1, RegData);
    NV_XUSB_PADCTL_WRITE( USB2_PAD_MUX, RegData);

    NvBootUtilWaitUS(TIMEOUT_10MS);
	
#endif

}

void XusbClkEnb(void)
{
    // Enable the xusb pad ctrl clock.
    NvBootClocksSetEnable(NvBootClocksClockId_XUsbId, NV_TRUE);
    // Enable the xusb Host clock.
    NvBootClocksSetEnable(NvBootClocksClockId_XUsbHostId, NV_TRUE);

#if NVBOOT_TARGET_FPGA
    NvBootClocksSetEnable(NvBootClocksClockId_XUsbDevId, NV_TRUE);
    NvBootClocksSetEnable(NvBootResetDeviceId_XUsbSsId, NV_TRUE);
#endif
}

void XusbClkRstInit(NvBootUsb3Context *Context)
{
    NvU32 RegData = 0;
    NvU32 Value = 0;
    NvU32 RegOffset = 0;
    NvU32 BusInstanceOffset = 0;

    BusInstanceOffset = BI_OFFSET(Context->RootPortNum);

    
    /*
    o   Set  XUSB_CORE_HOST_CLK_SRC of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_HOST_0 to 'PLLP_OUT0'
    o   Set  XUSB_CORE_HOST_CLK_DIVISOR of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_HOST_0 to '0x6'
    o   Set  XUSB_FALCON_CLK_SRC of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FALCON_0 to 'PLLP_OUT0'
    o   Set  XUSB_FALCON_CLK_DIVISOR of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FALCON_0 to '0x2'
    o   Set  XUSB_FS_CLK_SRC of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FS_0 to 'FO_48M'
    o   Set  XUSB_FS_CLK_DIVISOR of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FS_0 to '0x0'
    o   Set  XUSB_SS_CLK_SRC of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS_0 to 'HSIC_480'
    o   Set  XUSB_SS_CLK_DIVISOR of CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS_0 to '0x6'

    */
    RegData = NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_CORE_HOST, XUSB_CORE_HOST_CLK_DIVISOR, 6) |
              NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_CORE_HOST, XUSB_CORE_HOST_CLK_SRC, PLLP_OUT0);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_HOST_0, RegData);

    RegData = NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_FALCON, XUSB_FALCON_CLK_DIVISOR, 2) |
              NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_FALCON, XUSB_FALCON_CLK_SRC, PLLP_OUT0);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FALCON_0, RegData);

    RegData = NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_FS, XUSB_FS_CLK_DIVISOR, 0) |
              NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_FS, XUSB_FS_CLK_SRC, FO_48M);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FS_0, RegData);

    // change as per bebick,, maintain defualt HS_HSICP clock @120Mhz -- 31/1/2014
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS_0);
    RegData |= NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_SS, XUSB_SS_CLK_DIVISOR, 0x6) |
              NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_XUSB_SS, XUSB_SS_CLK_SRC, HSIC_480);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE +
               CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS_0, RegData);

    NvBootUtilWaitUS(2);

    // Clear SWR_XUSB_HOST_RST field in CLK_RST_CONTROLLER_RST_DEV_XUSB_0 
    // register to release the reset to the XUSB host block. The reset to 
    // XUSB host block should only be deasserted after the port and pad 
    // programming in step 1 and the clock programming in 
    // step 2 are both completed.

#if NVBOOT_TARGET_FPGA
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbDevId, NV_FALSE);
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbSsId, NV_FALSE);
#endif
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbHostId, NV_FALSE);

    // Step 4

    // Host controller identification and enumeration
    //Set MEMORY_SPACE and BUS_MASTER bits in NV_PROJ__XUSB_CFG_1 to 0x1

    NV_XUSB_CFG_READ(1, RegData);
    RegData = NV_FLD_SET_DRF_DEF(XUSB_CFG, 1, MEMORY_SPACE, ENABLED, RegData);
    RegData = NV_FLD_SET_DRF_DEF(XUSB_CFG, 1, BUS_MASTER, ENABLED, RegData);
    NV_XUSB_CFG_WRITE(1, RegData);

    // Step 5

    // Need to initialize the following registers by setting them to all 0s:
    // NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD3
    // NV_PROJ__XUSB_CSB_HS_BI_WORKQ_DWRD43
    NV_XUSB_CSB_EXPAND(HS_BI_WORKQ_DWRD3, RegOffset);
    Value = 0;
    PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &Value);

    NV_XUSB_CSB_EXPAND(HS_BI_WORKQ_DWRD4, RegOffset);
    Value = 0;
    PciRegUpdate(Context, RegOffset + BusInstanceOffset, XUSB_WRITE , &Value);
}

void XusbPadVbusEnable(const NvBootUsb3Params *Params)
{
    NvU32 RegData = 0;

    // Step 6
    // Enables platform specific regulators to enable VBUS and pull-up 
    // voltage to the VBUS control PMIC's EN input to the 
    // boot media port via platform specific programming.
	// Setup IO pad(s) to enable control to VBUS and 
	// detection of over current status
    // Enable VBUS power to the device.


	// Read OC pin register in FUSE_BOOT_DEVICE_INFO_0 register to set 
	// PORTN_OC_PIN field in XUSB_PADCTL_USB2_OC_MAP_0 register, 
	// where N is determined by BOOT_PORT. In the case the OC pin is 
	// identified as VBUS_ENABLE pad 0 or 1, the following programming sequence 
	// with separate writes is required, where X matches VBUS pad enumeration.
    // E_IO_HV field in PINMUX_AUX_USB_VBUS_ENX_0 register to ENABLE,
    // E_OD field in PINMUX_AUX_USB_VBUS_ENX_0 register to ENABLE, 
    // E_INPUT field in PINMUX_AUX_USB_VBUS_ENX_0 register to ENABLE, 
    // and PDPU field in PINMUX_AUX_USB_VBUS_ENX_0 register to NONE

	//pinmux programming
    if((Params->VBusEnable == VBUS_ENABLE_0) ||
        (Params->OCPin == XUSB_PADCTL_VBUS_OC_MAP_0_VBUS_ENABLE0_OC_MAP_OC_DETECTED_VBUS_PAD0))
    {
        NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_Usb3, NvBootPinmuxConfig_Usb3_Otg0);
    }
    else if((Params->VBusEnable == VBUS_ENABLE_1) ||
        (Params->OCPin == XUSB_PADCTL_VBUS_OC_MAP_0_VBUS_ENABLE0_OC_MAP_OC_DETECTED_VBUS_PAD1))
    {
        NvBootPadsConfigForBootDevice(NvBootFuseBootDevice_Usb3, NvBootPinmuxConfig_Usb3_Otg1);
    }

    NV_XUSB_PADCTL_READ( USB2_OC_MAP, RegData);

    if(Params->RootPortNumber == USB_BOOT_PORT_OTG0)
    {   
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT0_OC_PIN, Params->OCPin, RegData);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG1)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT1_OC_PIN, Params->OCPin, RegData);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG2)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT2_OC_PIN, Params->OCPin, RegData);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG3)
    {
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, USB2_OC_MAP, PORT3_OC_PIN, Params->OCPin, RegData);
    }
    NV_XUSB_PADCTL_WRITE( USB2_OC_MAP, RegData);

    // Read VBUS number register in FUSE_BOOT_DEVICE_INFO_0 register to 
    // Enable the VBUS power to the port by setting VBUS_ENABLEN of the 
    // XUSB_PADCTL_VBUS_OC_MAP_0 register, where N is determined by VBUS number

	NV_XUSB_PADCTL_READ( VBUS_OC_MAP, RegData);

    if(Params->VBusEnable == VBUS_ENABLE_0)
    {   
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, VBUS_OC_MAP, VBUS_ENABLE0, YES, RegData);
    }
    else if(Params->VBusEnable == VBUS_ENABLE_1)
    {
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, VBUS_OC_MAP, VBUS_ENABLE1, YES, RegData);
    }
    NV_XUSB_PADCTL_WRITE( VBUS_OC_MAP, RegData);

 		//Wait 100ms.
#if NVBOOT_TARGET_RTL
		NvBootUtilWaitUS(TIMEOUT_1US);//(TIMEOUT_1MS);
#else
        NvBootUtilWaitUS(TIMEOUT_100MS);
#endif

    // Clear possible false OC event reporting by writing '1' to 
    // OC_DETECTEDN and OC_DETECTED_VUSB_PADN of the 
    // XUSB_PADCTL_OC_DET_0 register, where N are 0~3

    NV_XUSB_PADCTL_READ( OC_DET, RegData);
    if(Params->RootPortNumber == USB_BOOT_PORT_OTG0)
    {   
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, SET_OC_DETECTED0, YES, RegData);
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, OC_DETECTED_VBUS_PAD0, YES, RegData);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG1)
    {
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, SET_OC_DETECTED1, YES, RegData);
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, OC_DETECTED_VBUS_PAD1, YES, RegData);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG2)
    {
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, SET_OC_DETECTED2, YES, RegData);
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, OC_DETECTED_VBUS_PAD2, YES, RegData);
    }
    else if(Params->RootPortNumber == USB_BOOT_PORT_OTG3)
    {
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, SET_OC_DETECTED3, YES, RegData);
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, OC_DET, OC_DETECTED_VBUS_PAD3, YES, RegData);
    }    
	NV_XUSB_PADCTL_WRITE( OC_DET, RegData);

}

void XusbPadVbusDisable(const NvBootUsb3Params *Params)
{
    NvU32 RegData;
    //vbus Disable
    NV_XUSB_PADCTL_READ( VBUS_OC_MAP, RegData);
    //@ 7009f018
    if(Params->VBusEnable == VBUS_ENABLE_0)
    {
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, VBUS_OC_MAP, VBUS_ENABLE0, NO, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, VBUS_OC_MAP, VBUS_ENABLE0_OC_MAP, Params->OCPin, RegData);
    }
    else if(Params->VBusEnable == VBUS_ENABLE_1)
    {
        RegData = NV_FLD_SET_DRF_DEF( XUSB_PADCTL, VBUS_OC_MAP, VBUS_ENABLE1, NO, RegData);
        RegData = NV_FLD_SET_DRF_NUM( XUSB_PADCTL, VBUS_OC_MAP, VBUS_ENABLE1_OC_MAP, Params->OCPin, RegData);
    }
    NV_XUSB_PADCTL_WRITE( VBUS_OC_MAP, RegData);
    //@ 7009f018
}

static NvBootError XusbResetPort(NvBootUsb3Context *Context, const NvBootUsb3Params *Params)
{
    NvBootError ErrorCode = NvBootError_Success;
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                    CLK_RST_CONTROLLER_RST_DEVICES_W_0);

    //OTG ports disable vbus. Do only if reset was already cleared for padctl.
    if(NV_DRF_VAL(CLK_RST_CONTROLLER, RST_DEVICES_W, SWR_XUSB_PADCTL_RST, RegData)
       == CLK_RST_CONTROLLER_RST_DEVICES_W_0_SWR_XUSB_PADCTL_RST_DISABLE)
    {
        if((Context->RootPortNum == USB_BOOT_PORT_OTG0)  ||
            (Context->RootPortNum == USB_BOOT_PORT_OTG1) ||
            (Context->RootPortNum == USB_BOOT_PORT_OTG2) ||
            (Context->RootPortNum == USB_BOOT_PORT_OTG3)) 
        {
            XusbPadVbusDisable(Params);
        }
    }

    NvBootResetSetEnable(NvBootResetDeviceId_XUsbHostId, NV_TRUE);
  
    // Apply Reset to the xusb pad controller
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbId, NV_TRUE);

    return ErrorCode;
}

NvBootError HwUsbInitController( NvBootUsb3Context *Context, const NvBootUsb3Params *Params)
{
    NvBootError e = NvBootError_Success;

    // Enable clocks for xusb pad ctrl, host (Dev,SS also for fpga).
    XusbClkEnb();

    XusbResetPort(Context, Params);

    // Remove padctl from Reset.
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbId, NV_FALSE);

    NvBootUtilWaitUS(2);

    XusbPadCtrlInit(Params, Context);
    XusbClkRstInit( Context);

    XusbPadVbusEnable(Params);
    // check /reset device connection
    NV_BOOT_CHECK_ERROR(DevicePortInit(Context));

    NvBootArcEnable();

    NVBOOT_MSG((Q_USB3BOOT_INIT));

    return e;
}

/* Public Function Definitions */

NvBootError
NvBootUsb3Init(
    const NvBootUsb3Params *Params,
    NvBootUsb3Context *Context)
{
    NvBootError e = NvBootError_Success;

    NV_ASSERT(Params != NULL);
    NV_ASSERT(Context != NULL);

    NVBOOT_MSG((Q_USB3BOOT_START));

    // Params->RootPortNumber;
    NV_ASSERT( (Params->RootPortNumber == USB_BOOT_PORT_OTG0) ||
        (Params->RootPortNumber == USB_BOOT_PORT_OTG1) ||
        (Params->RootPortNumber == USB_BOOT_PORT_OTG2) ||
        (Params->RootPortNumber == USB_BOOT_PORT_OTG3));

    InitializeDataStructures(Context, Params->RootPortNumber);

    //Usb3 page size of 512 bytes is fixed
    Context->PageSizeLog2  = MSC_MAX_PAGE_SIZE_LOG_2;
    Context->BlockSizeLog2 = NVBOOT_MAX_PAGE_SIZE_LOG2;

#if NVBOOT_TARGET_FPGA
    NvBootUtilWaitUS(TIMEOUT_100MS);
#endif

    // Initialize the Usb Host controller.
    e = HwUsbInitController( Context, Params);

#if NVBOOT_TARGET_FPGA
    NvBootUtilWaitUS(TIMEOUT_100MS);
#endif

    // Check whether device is present
    // this step is already checked as part of device port init,,, need to re-check and remove!!
    if ((e = HwUsbIsDeviceAttached(Context)) != NvBootError_Success)
    {
        NvBootUsb3Shutdown();
    }
    else
    {
        Context->Usb3BitInfo->PortNum = Params->RootPortNumber;
        // only MSD(BOT) is supported as boot media
        // enumerate the attached usb device
        // set configuration
        // Step 9,10 & 11
        NV_BOOT_CHECK_ERROR(EnumerateDevice( Context));

        NVBOOT_MSG((Q_USB3BOOT_DEV_ENUM));
        // Step 12
        // Initialize scsi commands (inquiry, read capacity/capacities, request sense, mode sense & test unit ready)
        NV_BOOT_CHECK_ERROR(InitializeScsiMedia( Context));

        NVBOOT_MSG((Q_USB3BOOT_SCSI_INIT));
    }
    // Log return value in BIT
    Context->Usb3BitInfo->InitReturnVal = (NvU32)e;
    return e;
}
//_________________________________________________________________________________________________

void
NvBootUsb3GetParams(
    const NvU32 ParamIndex,
    NvBootUsb3Params **Params)
{
    uint8_t RootPortNum = 0;
    NV_ASSERT(Params != NULL);

    /*
     * Extract the Root Port selection from Param Index, which comes from fuses.
     * Boot media attached to the Root port attached. The value
     * starting from zero to 3, corresponds to port boot media is attached!!
     */
    RootPortNum = NV_DRF_VAL(USBH_DEVICE, CONFIG,
                                    ROOT_PORT, ParamIndex);

    s_DefaultUsb3Params.RootPortNumber = RootPortNum;

    /* PageSize2kor16k is reserved*/
    /*
    s_DefaultUsb3Params.PageSize2kor16k = NV_DRF_VAL(USBH_DEVICE, CONFIG,
                                    PAGE_SIZE_2KOR16K, ParamIndex);
    */
    /*
     * Extract the OC  from Param Index, which comes from fuses.
     * The value starting from zero to 5, corresponds to OC detected mapping!!
     */
    s_DefaultUsb3Params.OCPin = NV_DRF_VAL(USBH_DEVICE, CONFIG,
                                    OC_PIN, ParamIndex);

    /*
     * Extract the Vbus Enble  from Param Index, which comes from fuses.
     * The value starting from zero to 1, corresponds to pad tri-state values.
     */
    s_DefaultUsb3Params.VBusEnable = NV_DRF_VAL(USBH_DEVICE, CONFIG,
                                    VBUS_ENABLE, ParamIndex);

    /* PageSize2kor16k is reserved*/
    /*
    */
    *Params = (NvBootUsb3Params*)&s_DefaultUsb3Params;

   PRINT_USBH_MESSAGES("\r\n Usb Root port number =0x%x, " \
        s_DefaultUsb3Params.RootPortNumber);
}
//_________________________________________________________________________________________________

NvBool
NvBootUsb3ValidateParams(
    const NvBootUsb3Params *Params)
{
    NV_ASSERT(Params != NULL);
    (void)Params;
    PRINT_USBH_MESSAGES("\r\nValidateParams, RootPortNumber =%d , " \
        , Params->RootPortNumber);

    // Params->RootPortNumber;
    NV_ASSERT( (Params->RootPortNumber == USB_BOOT_PORT_OTG0) ||
        (Params->RootPortNumber == USB_BOOT_PORT_OTG1) ||
        (Params->RootPortNumber == USB_BOOT_PORT_OTG2) ||
        (Params->RootPortNumber == USB_BOOT_PORT_OTG3));

    // Params->VBusEnable;
    NV_ASSERT((Params->VBusEnable == VBUS_ENABLE_0) ||
        (Params->VBusEnable == VBUS_ENABLE_1));

    return NV_TRUE;
}

void
NvBootUsb3GetBlockSizes(
    const NvBootUsb3Params *Params,
    NvU32 *BlockSizeLog2,
    NvU32 *PageSizeLog2)
{
    NV_ASSERT(Params != NULL);
    NV_ASSERT(BlockSizeLog2 != NULL);
    NV_ASSERT(PageSizeLog2 != NULL);
    NV_ASSERT(s_Usb3Context != NULL);

    (void)Params;
    s_Usb3Context->BlockSizeLog2 = NVBOOT_MSC_BLOCK_SIZE_LOG2;
    s_Usb3Context->PageSizeLog2 = MSC_MAX_PAGE_SIZE_LOG_2;

    *BlockSizeLog2 = s_Usb3Context->BlockSizeLog2;
    *PageSizeLog2 = s_Usb3Context->PageSizeLog2;

    PRINT_USBH_MESSAGES("\r\nBlockSize=%d, PageSize=%d, PagesPerBlock=%d",\
        (1 << s_Usb3Context->BlockSizeLog2),(1 << s_Usb3Context->PageSizeLog2),\
          (1 << (s_Usb3Context->BlockSizeLog2 - s_Usb3Context->PageSizeLog2)));
    return;
}

NvBootError
NvBootUsb3Read(
    const NvU32 Block,
    const NvU32 Page,
    const NvU32 Length,
    uint8_t *Dest)
{
    NvBootError ErrorCode = NvBootError_Success;
    NvBootDeviceStatus DeviceStatus = NvBootDeviceStatus_ReadFailure;
    NvBootUsb3Context *Context = s_Usb3Context;
    NvU32 RetryCount = USB_MAX_TXFR_RETRIES;
    NvU32 LogicalBlkAddr;
    uint16_t numPages;

    NV_ASSERT(Page < (1 << (s_Usb3Context->BlockSizeLog2) - (s_Usb3Context->PageSizeLog2)));
    NV_ASSERT(Dest != NULL);
    PRINT_USBH_MESSAGES("\r\nRead Block=%d, Page=%d", Block, Page);

    //**************************READ ONE PAGE*************************

    //Before issuing READ10 command, need to test with TEST_UNIT_READY command for device readyness.
    //This if required can be guarded by CYA bit!
    //ErrorCode = NvBootXusbMscBotProcessRequest(Context, TESTUNITREADY_CMD_OPCODE);

    if (ErrorCode == NvBootError_Success)
    {
        //Issue Read command
        // It is assumed that usb device block size would be multiple of 2 and also
        //page size is more than usb device block size as returned by Read Capacity Command
         // blocksize == 32 pages,, (16384)

    while(RetryCount)
    {
            LogicalBlkAddr = ((Block << (Context->BlockSizeLog2 - (Context->PageSizeLog2))) + Page );
            INT_TO_BYTE_ARRAY(LogicalBlkAddr, Context->LogicalBlkAddr);
            numPages = CEIL_PAGE(Length, (1<<Context->PageSizeLog2));
            SHORT_TO_BYTE_ARRAY(numPages, Context->TransferLen);

            //Make sure BufferData variable is initialized to requested destination address before issuing READ command
            Context->BufferData = Dest;
            //Send Read command
            ErrorCode = NvBootXusbMscBotProcessRequest(Context, READ10_CMD_OPCODE);

            if (ErrorCode == NvBootError_Success)
            {
                NVBOOT_MSG((Q_USB3BOOT_SCSI_READ));
                DeviceStatus = NvBootDeviceStatus_Idle;
                break;
            }
            else
            {
                // Update Device status
                DeviceStatus = NvBootDeviceStatus_ReadFailure;
            }
            RetryCount--;
#if NVBOOT_TARGET_RTL
            NvBootUtilWaitUS(TIMEOUT_1US);//(TIMEOUT_1MS);
#else
            NvBootUtilWaitUS(TIMEOUT_1MS);
#endif
    }
    }
    else
    {
        //either phase error or command failed due to internal problems
        //For failed command try to get sense key and report read failure
        NVBOOT_MSG((Q_USB3BOOT_SCSI_READ_FAIL));
        if(ErrorCode == NvBootError_XusbCswStatusCmdFail)
        {
            //Send Mode Sense command to know the type of problem
            //Sense key is being stored in BIT SecondaryDeviceStatus field
            ErrorCode = NvBootXusbMscBotProcessRequest(Context, REQUESTSENSE_CMD_OPCODE);
        }
    }
    Context->Usb3BitInfo->ReadPageReturnVal = ErrorCode;
    Context->Usb3BitInfo->DeviceStatus = DeviceStatus;
    return ErrorCode;
}

NvBootDeviceStatus NvBootUsb3QueryStatus(void)
{
    return (NvBootDeviceStatus)(s_Usb3Context->Usb3BitInfo->DeviceStatus);
}

void NvBootUsb3Shutdown(void)
{

    // Apply reset to xusb host controller
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbHostId, NV_TRUE);
#if NVBOOT_TARGET_FPGA
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbDevId, NV_TRUE);
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbSsId, NV_TRUE);
#endif

    // Apply Reset to the pad ctrl
    NvBootResetSetEnable(NvBootResetDeviceId_XUsbId, NV_TRUE);

    // Disable the xusb Host clock.
    NvBootClocksSetEnable(NvBootClocksClockId_XUsbHostId, NV_FALSE);

    // Disable the xusb clock.
    NvBootClocksSetEnable(NvBootClocksClockId_XUsbId, NV_FALSE);

    NVBOOT_MSG((Q_USB3BOOT_SHUTDOWN));

    // Disable the bus power.
    XusbPadVbusDisable((NvBootUsb3Params*)&s_DefaultUsb3Params);
    //Clear the Context
    NvBootUtilMemset((void *)s_Usb3Context,0,sizeof(NvBootUsb3Context));
    s_Usb3Context = NULL;
    return;
}

NvBootError NvBootUsb3GetReaderBuffersBase(uint8_t** ReaderBuffersBase ,
                            const NvU32 Alignment, const NvU32 Bytes)
{
    (void)ReaderBuffersBase;
    (void)Alignment;
    (void)Bytes;
    return NvBootError_Unimplemented;
}

/**
    Write routine. Used only in uartmon driver
**/
NvBootError
NvBootUsb3Write(
    const NvU32 Block,
    const NvU32 Page,
    const NvU32 Length,
    uint8_t *Dest)
{
    NvBootError ErrorCode;
    NvBootDeviceStatus DeviceStatus = NvBootDeviceStatus_ReadFailure;
    uint16_t numPages;
    NvBootUsb3Context *Context = s_Usb3Context;
    NvU32 LogicalBlkAddr;

    //Before issuing WRITE command, need to test with TEST_UNIT_READY command for device readyness. 
    ErrorCode = NvBootXusbMscBotProcessRequest(Context, TESTUNITREADY_CMD_OPCODE);

    if (ErrorCode == NvBootError_Success)
    {
        //Issue Read command
        // It is assumed that usb device block size would be multiple of 2 and also
        //page size is more than usb device block size as returned by Read Capacity Command

        LogicalBlkAddr =  ((Block << (Context->BlockSizeLog2 - (Context->PageSizeLog2))) + Page );
        INT_TO_BYTE_ARRAY(LogicalBlkAddr, Context->LogicalBlkAddr);

        numPages = CEIL_PAGE(Length, (1<<Context->PageSizeLog2));
        SHORT_TO_BYTE_ARRAY(numPages, Context->TransferLen);

        //Make sure BufferData variable is initialized to requested destination address before issuing READ command
        Context->BufferData = Dest;  
        //Send Command
        ErrorCode = NvBootXusbMscBotProcessRequest(Context, WRITE10_CMD_OPCODE);
        
        if (ErrorCode == NvBootError_Success)
        {
            DeviceStatus = NvBootDeviceStatus_Idle;
        }

    }
    else
    {
        //either phase error or command failed due to internal problems
        //For failed command try to get sense key and report read failure
        if(ErrorCode == NvBootError_XusbCswStatusCmdFail)
        {
            //Send Mode Sense command to know the type of problem
            //Sense key is being stored in BIT SecondaryDeviceStatus field
            ErrorCode = NvBootXusbMscBotProcessRequest(Context, REQUESTSENSE_CMD_OPCODE); 
        }
    }
    Context->Usb3BitInfo->ReadPageReturnVal = ErrorCode;
    Context->Usb3BitInfo->DeviceStatus = DeviceStatus;

    return ErrorCode;
}

/* gcc accepts __attribute__((unused)) for ununsed variables but this is not
 * portable
 */
NvBootDeviceStatus NvBootUsb3PinMuxInit(const void *Params )
{
    /*
     * Upper layer doesn't check for NvBootError_Unimplemented
     * and considers it an error. Not to break existing functionality,
     * return NvBootError_Success
     */
    (void)Params;
    return NvBootError_Success;
}


