/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#ifndef __XUSB_DEV_HW_H
#define __XUSB_DEV_HW_H
#include "nvtypes.h"
#include "nvboot_error.h"
#include "nvboot_car_int.h"
// extern clock tables here
extern const ClockInst s_XUSBDeviceClkTable[];
extern const ClockTable XusbClockTables[];
// extern const ClockTable XusbFpgaClockTables[];
// Index of secondary table
#define OSC_TABLE 0
extern const ClockInst s_XUSBTrackingClockFrequency_38_4[];
// extern ClockInst s_XUSBTrackingClockFrequency_12[];

/*************** h/w helper routines too small to be functions in their own right ******************/

/**
 *  Set Endpoint Context
 */
 #define NvBootXusbDeviceSetEndpointContext(EpContext) NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_ECPLO_0, (EpContext))
 
 /**
  *  Set address in hw register
  */
 #define NvBootXusbDeviceSetAddressHw(Addr) NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0, \
                                            NV_FLD_SET_DRF_NUM(XUSB_DEV_XHCI, CTRL, DEVADR, (Addr), \
                                            NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_CTRL_0)))

 /**
  * Unpause endpoint
  */
  #define NvBootXusbDeviceUnpauseEp(EpIndex) NV_WRITE32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0, \
                                              NV_READ32(XUSB_BASE + XUSB_DEV_XHCI_EP_PAUSE_0) &  ~(1 << (EpIndex)))

 /**
  *  Legacy pcie init/ Arc enable.
  */
  void NvBootXusbDeviceBusInit();

/* End h/w helper routines */
 
#endif // __XUSB_DEV_HW_H
