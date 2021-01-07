/*
 * Copyright (c) 2007 - 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#include "nvboot_rcm_port_int.h"
#include "nvboot_usbf_int.h"
#include "nvboot_xusb_dev_int.h"
#include "nvboot_error.h"
static NvBootRCMPort_T sRCMPort;

NvBootError NvBootRcmSetupPortHandle(NvBootRCMPortID_T RcmPortId)
{
    if(RcmPortId == RCM_XUSB)
    {
        sRCMPort.Context.PortId      = RCM_XUSB;
        sRCMPort.GetClockTable       = (RCMPortGetClockTable_T)NvBootXusbDeviceGetClockTable;
        sRCMPort.Init                = (RCMPortInit_T)NvBootXusbDeviceInit;
        sRCMPort.Connect             = (RCMPortConnect_T)NvBootXusbDeviceEnumerate;
        sRCMPort.ReceiveStart        = (RCMPortReceiveStart_T)NvBootXusbDeviceReceiveStart;
        sRCMPort.ReceivePoll         = (RCMPortReceivePoll_T)NvBootXusbDeviceReceivePoll;
        sRCMPort.Receive             = (RCMPortReceive_T)NvBootXusbDeviceReceive;
        sRCMPort.TransferStart       = (RCMPortTransferStart_T)NvBootXusbDeviceTransmitStart;
        sRCMPort.TransferPoll        = (RCMPortTransferPoll_T)NvBootXusbDeviceTransmitPoll;
        sRCMPort.Transfer            = (RCMPortTransfer_T)NvBootXusbDeviceTransmit;
        sRCMPort.HandleError         = (RCMPortHandleError_T)NvBootXusbHandleError;
    }
    else
        return NvBootError_InvalidParameter;
    return NvBootError_Success;
}
NvBootRCMPort_T* NvBootRcmGetPortHandle()
{
    return &sRCMPort; 
}
