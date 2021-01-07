/*
 * Copyright (c) 2007 - 2013 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_rcmport_int.h - Definition of the device manager interface.
 */
#ifndef RCM_PORT_INT_H
#define RCM_PORT_INT_H
#include "nvcommon.h"
#include "nvboot_error.h"
typedef enum
{
    RCM_USB_OTG,
    RCM_XUSB
}
NvBootRCMPortID_T;

typedef struct
{
    NvBootRCMPortID_T PortId;
    NvU8 Initialized;
    NvU8 Connected;
}
RCMPortContext_T;

typedef NvBootError (*RCMPortInit_T)(void);
typedef NvBootError (*RCMPortConnect_T)(NvU8* OptionalBuffer);
typedef NvBootError (*RCMPortReceiveStart_T)(NvU8* Buffer, NvU32 Bytes);
typedef NvBootError (*RCMPortReceivePoll_T)(NvU32 *pBytesReceived, NvU32 TimeoutMs, NvU8* OptionalBuffer);
typedef NvBootError (*RCMPortReceive_T)(NvU8* Buffer, NvU32 Bytes, NvU32 *pBytesReceived);
typedef NvBootError (*RCMPortTransferStart_T)(NvU8* Buffer, NvU32 Bytes);
typedef NvBootError (*RCMPortTransferPoll_T)(NvU32 *pBytesTransferred, NvU32 TimeoutMs, NvU8* OptionalBuffer);
typedef NvBootError (*RCMPortTransfer_T)(NvU8* Buffer, NvU32 Bytes, NvU32 *pBytesTransferred);
typedef NvBootError (*RCMPortHandleError_T)(void);

typedef struct
{
    RCMPortContext_T    Context;
    RCMPortInit_T       Init;
    RCMPortConnect_T    Connect;
    RCMPortReceiveStart_T   ReceiveStart; // Non Blocking Read
    RCMPortReceivePoll_T    ReceivePoll; // Poll for Read Complete
    RCMPortReceive_T        Receive; // Blocking Read
    RCMPortTransferStart_T  TransferStart; // Non Blocking Transfer
    RCMPortTransferPoll_T   TransferPoll; // Poll for transfer complete
    RCMPortTransfer_T       Transfer; // Blocking Transfer
    RCMPortHandleError_T    HandleError; // Port specific error handling
}   NvBootRCMPort_T;


NvBootError NvBootRcmSetupPortHandle(NvBootRCMPortID_T RcmPortId);
NvBootRCMPort_T* NvBootRcmGetPortHandle(void);
#endif
