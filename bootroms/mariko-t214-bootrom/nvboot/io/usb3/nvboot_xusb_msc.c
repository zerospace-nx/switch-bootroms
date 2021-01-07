/*
 * Copyright (c) 2011 - 2012 NVIDIA Corporation.  All rights reserved.
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
#include "arfuse.h"
#include "arxusb_padctl.h"
#include "nvboot_bit.h"
#include "nvboot_clocks_int.h"
#include "nvboot_codecov_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_usb3_context.h"
#include "nvboot_usb3_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "project.h"
#include "nvboot_config.h"

// Boot Info table.
extern NvBootInfoTable BootInfoTable;

extern TRB    *TRBRing; //The TRB structure referenced by this physical memory pointer shall be aligned to a 16-byte boundary.

//***************************************Mass Storage Class and Bulk Transport only related functions**********************
//For BOT protocol: http://www.usb.org/developers/devclass_docs/usbmassbulk_10.pdf
//For command related details refer to http://www.usb.org/developers/devclass_docs/usbmass-ufi10.pdf
//Data Transfer Conditions
//The host indicates the expected transfer in the CBW using the Direction bit and the
//dCBWDataTransferLength field. The device then determines the actual direction and data transfer length.
//The device responds by transferring data, STALLing endpoints when specified, and returning the appropriate CSW.

//Command Transport
//The host shall send each CBW, which contains a
//command block, to the device via the Bulk-Out
//endpoint. The CBW shall start on a packet boundary
//and end as a short packet with exactly 31 (1Fh) bytes transferred.
//The device shall indicate a successful transport of a
//CBW by accepting (ACKing) the CBW. If the CBW is
//not valid see 6.6.1 - CBW Not Valid or If the host detects
//a STALL of the Bulk-Out endpoint during command
//transport, the host shall respond with a Reset Recovery

//Data Transport
//All data transport shall begin on a packet boundary.
//The host shall attempt to transfer the exact number of
//bytes to or from the device as specified by the
//dCBWDataTransferLength and the Direction bit.
//To report an error before data transport completes and to maximize data integrity, 
//the device may terminate the command by STALLing the endpoint in use (
//the Bulk-In endpoint during data in, the Bulk-Out endpoint during data out).

//Status Transport
//The device shall send each CSW to the host via the Bulk-In endpoint. The CSW shall start on a packet boundary
//and end as a short packet with exactly 13 (Dh) bytes transferred. 
//The CSW indicates to the host the status of the execution of the command block from the corresponding CBW.
//The dCSWDataResidue field indicates how much of the data transferred is to be considered processed or
//relevant. The host shall ignore any data received beyond that which is relevant
//-The host shall perform a Reset Recovery when Phase Error status is returned in the CSW.
//-For Reset Recovery the host shall issue in the following order: :
//(a) a Bulk-Only Mass Storage Reset
//(b) a Clear Feature HALT to the Bulk-In endpoint
//(c) a Clear Feature HALT to the Bulk-Out endpoint

//Host/Device Data Transfers

//A Bulk-Only Protocol transaction begins with the host sending a CBW to the device and attempting to make the
//appropriate data transfer (In, Out or none). The device receives the CBW, checks and interprets it, attempts to
//satisfy the host's request, and returns status via a CSW

//Valid and Meaningful CBW
//------------------------
//The host communicates its intent to the device through the CBW. The device performs two verifications on
//every CBW received. First, the device verifies that what was received is a valid CBW. Next, the device
//determines if the data within the CBW is meaningful.
//The device shall not use the contents of the dCBWTag in any way other than to copy its value to the dCSWTag of
//the corresponding CSW

//The device shall consider the CBW valid when:
//� The CBW was received after the device had sent a CSW or after a reset,
//� the CBW is 31 (1Fh) bytes in length,
//� and the dCBWSignature is equal to 43425355h.
//The device shall consider the contents of a valid CBW meaningful when:
//� no reserved bits are set,
//� the bCBWLUN contains a valid LUN supported by the device,
//� and both bCBWCBLength and the content of the CBWCB are in accordance with
// bInterfaceSubClass


//Valid and Meaningful CSW
//-------------------------
//The device generally communicates the results of its attempt to satisfy the host뭩 request through the CSW. The
//host performs two verifications on every CSW received. First, the host verifies that what was received is a valid
//CSW Next, the host determines if the data within the CSW is meaningful
//Valid CSW
//The host shall consider the CSW valid when:
//� the CSW is 13 (Dh) bytes in length,
//� and the dCSWSignature is equal to 53425355h,
//� the dCSWTag matches the dCBWTag from the corresponding CBW
//Meaningful CSW
//The host shall consider the contents of the CSW meaningful when:
//either the bCSWStatus value is 00h or 01h and the dCSWDataResidue is less than or equal to dCBWDataTransferLength..
//or the bCSWStatus value is 02h.


//Device Error Handling
//----------------------
//The device may not be able to fully satisfy the host's request. At the point when the device discovers that it
//cannot fully satisfy the request, there may be a Data-In or Data-Out transfer in progress on the bus, and the host
//may have other pending requests. The device may cause the host to terminate such transfers by STALLing the appropriate pipe
//Please note that whether or not a STALL handshake actually appears on the bus depends on whether or not there
//is a transfer in progress at the point in time when the device is ready to STALL the pipe.

//Host Error Handling
//----------------------
//If the host receives a CSW which is not valid, then the host shall perform a Reset Recovery. If the host receives
//a CSW which is not meaningful, then the host may perform a Reset Recovery

//Error Classes
//---------------------
//In every transaction between the host and the device, there are four possible classes of errors. These classes are
//not always independent of each other and may occur at any time during the transaction

//1)CBW Not Valid
//If the CBW is not valid, the device shall STALL the Bulk-In pipe. Also, the device shall either STALL the
//Bulk-Out pipe, or the device shall accept and discard any Bulk-Out data. The device shall maintain this state
//until a Reset Recovery

//2)Internal Device Error
//The device may detect an internal error for which it has no reliable means of recovery other than a reset. The
//device shall respond to such conditions by: either STALLing any data transfer in progress and returning a Phase Error status
//(bCSWStatus = 02h)
//or STALLing all further requests on the Bulk-In and the Bulk-Out pipes until a Reset Recovery.

//3)Host/Device Disagreements
//After recognizing that a CBW is valid and meaningful, and in the absence of internal errors, the device may
//detect a condition where it cannot meet the host뭩 expectation for data transfer, as indicated by the Direction bit
//of the bmCBWFlags field and the dCBWDataTransferLength field of the CBW. In some of these cases, the
//device may require a reset to recover. In these cases, the device shall return Phase Error status (bCSWStatus = 02h)

//4)Command Failure
//After recognizing that a CBW is valid and meaningful, the device may still fail in its attempt to satisfy the
//command. The device shall report this condition by returning a Command Failed status (bCSWStatus = 01h)

/*
Host/Device Data Transfer Matrix (1, 6, 12 cover most of the cases)
              HOST
              Hn            Hi              Ho
DEVICE
Dn          (1) Hn = Dn      (4) Hi > Dn     (9) Ho > Dn
                            
                            
Di          (2) Hn < Di     (5) Hi > Di      (10) Ho <> Di
                            (6) Hi = Di
                            (7) Hi < Di

Do          (3) Hn < Do     (8) Hi <> Do    (11) Ho > Do
                                            (12) Ho = Do
                                            (13) Ho < Do

****************************************************************
Host Expectation
---------------------
Hn Host expects no data transfers
Hi Host expects to receive data from the device
Ho Host expects to send data to the device

Device Intent
---------------------
Dn Device intends to transfer no data
Di Device intends to send data to the host
Do Device intends to receive data from the host
******************************************************************/

#define BYTE_ARRAY_TO_INT(byte_a,int_b) \
do \
{ \
    int_b=0; \
    int_b |= byte_a[0]; \
    int_b |= byte_a[1] << 8; \
    int_b |= byte_a[2] << 16; \
    int_b |= byte_a[3] << 24; \
} \
while(0) \

#define BYTE_ARRAY_TO_SHORT(byte_a,short_b) \
do \
{ \
    short_b=0; \
    short_b |= byte_a[0]; \
    short_b |= byte_a[1]<<8; \
} \
while(0) \

extern void NvBootXusbPrepareEndTRB(NvBootUsb3Context *Context);
extern NvBootError NvBootXusbProcessEpStallClearRequest(NvBootUsb3Context *Context, USB_Endpoint_Type EpType);

//_________________________________________________________________________________________________

NvBootError NvBootXusbMscPrepareCDB(NvBootUsb3Context *Context)
{
    NvBootError ErrorCode = NvBootError_Success;
    CommandBlockWrapper *BufferCBW = ((CommandBlockWrapper *)BufferXusbCmd);
    uint8_t *BufferCDB = BufferCBW->CBWCB;
    uint16_t TransferLen;

    BYTE_ARRAY_TO_SHORT(Context->TransferLen, TransferLen);

    switch(BufferCBW->CBWTag)
    {
        case INQUIRY_CMD_OPCODE     :
                                        BufferCBW->CBWDataTransferLength                = sizeof(InquiryResponse);
                                        BufferCBW->CBWFlags                             = CBWFlags_DATA_DIR_IN;
                                        BufferCBW->CBWCBLength              = INQUIRY_CMD_LEN;
                                        ((InquiryCDB *)BufferCDB)->opcode        = INQUIRY_CMD_OPCODE;
                                        ((InquiryCDB *)BufferCDB)->LogicalUnitNum        = Context->stEnumerationInfo.LUN;
                                        ((InquiryCDB *)BufferCDB)->AllocationLen        = sizeof(InquiryResponse);
                                        break;
        case READ10_CMD_OPCODE      :   
                                        // BufferCBW->CBWDataTransferLength                = 0x200;//*(Context->TransferLen);
                                        BufferCBW->CBWDataTransferLength                = TransferLen << Context->PageSizeLog2;
                                        BufferCBW->CBWFlags                             = CBWFlags_DATA_DIR_IN;
                                        BufferCBW->CBWCBLength              = READ10_CMD_LEN;
                                        ((Read10CDB *)BufferCDB)->LogicalBlkAddrByte4_msb = Context->LogicalBlkAddr[3]; 
                                        ((Read10CDB *)BufferCDB)->LogicalBlkAddrByte3     = Context->LogicalBlkAddr[2];
                                        ((Read10CDB *)BufferCDB)->LogicalBlkAddrByte2     = Context->LogicalBlkAddr[1];
                                        ((Read10CDB *)BufferCDB)->LogicalBlkAddrByte1_lsb = Context->LogicalBlkAddr[0];
                                        ((Read10CDB *)BufferCDB)->TransferLen_msb         = Context->TransferLen[1];
                                        ((Read10CDB *)BufferCDB)->TransferLen_lsb         = Context->TransferLen[0];
                                        break;
        case REQUESTSENSE_CMD_OPCODE :  BufferCBW->CBWDataTransferLength                = sizeof(RequestSenseResponse);
                                        BufferCBW->CBWCBLength              = REQUESTSENSE_CMD_LEN;
                                        BufferCBW->CBWFlags                             = CBWFlags_DATA_DIR_IN;
                                        break;
        case TESTUNITREADY_CMD_OPCODE : BufferCBW->CBWDataTransferLength                = 0; // no data stage. Direction bit is irrelevant for no data stage
                                        BufferCBW->CBWCBLength              = TESTUNITREADY_CMD_LEN;
                                        break;
        case READ_CAPACITY_CMD_OPCODE : BufferCBW->CBWDataTransferLength                = sizeof(ReadCapacityResponse); 
                                        BufferCBW->CBWFlags                             = CBWFlags_DATA_DIR_IN;
                                        BufferCBW->CBWCBLength              = READ_CAPACITY_CMD_LEN;
                                        break;
        case READ_FORMAT_CAPACITY_CMD_OPCODE
                                      : BufferCBW->CBWDataTransferLength                = 252;//sizeof(ReadFmtCapacityResponse); changing the size to 252
                                        BufferCBW->CBWFlags                             = CBWFlags_DATA_DIR_IN;
                                        BufferCBW->CBWCBLength              = READ_FORMAT_CAPACITY_CMD_LEN;
                                        ((ReadFormatCapacityCDB *)BufferCDB)->AllocationLen_msb = (uint8_t)((BufferCBW->CBWDataTransferLength) >> 8);
                                        ((ReadFormatCapacityCDB *)BufferCDB)->AllocationLen_lsb = (uint8_t)((BufferCBW->CBWDataTransferLength) & 0xff);
                                        break;
        // Not used by bootrom but useful to keep it here in order to make Uartmon implementation easier
        case WRITE10_CMD_OPCODE       : 
                                        // BufferCBW->CBWDataTransferLength                    = 0x200;//*(Context->TransferLen);
                                        BufferCBW->CBWDataTransferLength                    = TransferLen << Context->PageSizeLog2;
                                        BufferCBW->CBWFlags                                 = CBWFlags_DATA_DIR_OUT;
                                        ((Write10CDB *)BufferCDB)->LogicalBlkAddrByte4_msb  = Context->LogicalBlkAddr[3]; 
                                        ((Write10CDB *)BufferCDB)->LogicalBlkAddrByte3      = Context->LogicalBlkAddr[2];
                                        ((Write10CDB *)BufferCDB)->LogicalBlkAddrByte2      = Context->LogicalBlkAddr[1];
                                        ((Write10CDB *)BufferCDB)->LogicalBlkAddrByte1_lsb  = Context->LogicalBlkAddr[0];
                                        ((Write10CDB *)BufferCDB)->TransferLen_msb          = Context->TransferLen[1];
                                        ((Write10CDB *)BufferCDB)->TransferLen_lsb          = Context->TransferLen[0];
                                        BufferCBW->CBWCBLength              = WRITE10_CMD_LEN;
                                        break;
        default                     :   ErrorCode = NvBootError_XusbMscInvalidCmd;//invalid or not supported command
                                        break;
                                        
    }
    if (ErrorCode == NvBootError_Success) 
    {
        //offset of opcode and LogicalUnitNum for all command types are same so it's ok to initialize in the end
        ((InquiryCDB *)BufferCDB)->opcode                           = BufferCBW->CBWTag;
        ((InquiryCDB *)BufferCDB)->LogicalUnitNum                   = Context->stEnumerationInfo.LUN;
    }
    return ErrorCode;
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbMscPrepareCBW(NvBootUsb3Context *Context,uint8_t opcode)
{
    CommandBlockWrapper *BufferCBW;
    NvBootUtilMemset(BufferXusbCmd,0,sizeof(CommandBlockWrapper));
    BufferCBW                           = (CommandBlockWrapper *)BufferXusbCmd;
    BufferCBW->CBWSignature             = CBW_SIGNATURE;
    BufferCBW->CBWTag                   = (NvU32)opcode;// update tag with some value instead of opcode..
    BufferCBW->CBWLUN                   = Context->stEnumerationInfo.LUN;
    return(NvBootXusbMscPrepareCDB(Context));
}
//_________________________________________________________________________________________________

void NvBootXusbPrepareNormalTRB(NvBootUsb3Context *Context,uint8_t *DataBufferPtr, NvU32 TRBTfrLen)
{
    NormalTRB *TRBRingEnquePtr = NULL;

    Context->TRBRingCtrlEnquePtr    = (TRB *)(&(TRBRing[0]));
    TRBRingEnquePtr = (NormalTRB *)(Context->TRBRingCtrlEnquePtr);
    NvBootUtilMemset((void *)TRBRingEnquePtr,0,sizeof(NormalTRB));//To make sure that all reserved bits are set to 0
    
    //since bootrom is using just one endpoint context structure for control/bulk-in/bulk-out, 
    //bulk-in and bulk-out transfers cannot be scheduled parallely

    TRBRingEnquePtr->DataBufferLo       = (NvU32)DataBufferPtr;
    TRBRingEnquePtr->TRBTfrLen          = TRBTfrLen;
    TRBRingEnquePtr->CycleBit           = Context->CycleBit;
    TRBRingEnquePtr->TRBType            = NvBootTRB_Normal;

    //TDSize is 0 indicating there is only one TRB for this TD
    //InterrupterTarget is 0. Bootrom is not using interrupt method
    //CycleBit - 0
    //ENT - Evaluate next TRB is 0
    //ISP - Interrupt-on Short Packet is 0
    //NS - No Snoop is 0
    //CH - Chain bit is 0
    //IOC- Interrupt on completion is 0
    //IDT - Immediate Data is 0
    //Rsvd2 is 0
    //BEI - Block Event Interrupt is 0
    //Rsvd3 is 0
    // dummy Last TRB with Cycle bit modified.
    (Context->TRBRingCtrlEnquePtr)++;
    return;
}


//_________________________________________________________________________________________________

void NvBootXusbMscBotAnalyzeReceiveData(NvBootUsb3Context *Context, uint8_t opcode)
{
    NvU32 tmp;
    switch(opcode)
    {
        case INQUIRY_CMD_OPCODE     :   Context->Usb3BitInfo->PeripheralDevTyp = ((InquiryResponse *)BufferXusbData)->PeripheralDevTyp;
                                        break;
        case READ10_CMD_OPCODE      :   //Handled in Read Page function 
                                        break;
        case REQUESTSENSE_CMD_OPCODE :  Context->Usb3BitInfo->SenseKey =  ((RequestSenseResponse *)BufferXusbData)->SenseKey; //log the sense info in BIT
                                        break;
        case READ_CAPACITY_CMD_OPCODE : tmp = ((ReadCapacityResponse *)BufferXusbData)->LastLogicalBlkAddrUnion.stLastLogicalBlkAddrByte.LastLogicalBlkAddrByte1_lsb \
                                               | (((ReadCapacityResponse *)BufferXusbData)->LastLogicalBlkAddrUnion.stLastLogicalBlkAddrByte.LastLogicalBlkAddrByte2 << 8 ) \
                                               | (((ReadCapacityResponse *)BufferXusbData)->LastLogicalBlkAddrUnion.stLastLogicalBlkAddrByte.LastLogicalBlkAddrByte3 << 16 ) \
                                               | (((ReadCapacityResponse *)BufferXusbData)->LastLogicalBlkAddrUnion.stLastLogicalBlkAddrByte.LastLogicalBlkAddrByte4_msb << 24 );
                                        //arrange in little endian format for later use
                                        Context->Usb3BitInfo->LastLogicalBlkAddr = tmp;

                                        Context->LogicalBlkAddr[0] = tmp;
                                        tmp = ((ReadCapacityResponse *)BufferXusbData)->BlockLenInByteUnion.stBlockLenInByte.BlockLenInByte_Byte1_lsb \
                                               | (((ReadCapacityResponse *)BufferXusbData)->BlockLenInByteUnion.stBlockLenInByte.BlockLenInByte_Byte2 << 8 ) \
                                               | (((ReadCapacityResponse *)BufferXusbData)->BlockLenInByteUnion.stBlockLenInByte.BlockLenInByte_Byte3 << 16 ) \
                                               | (((ReadCapacityResponse *)BufferXusbData)->BlockLenInByteUnion.stBlockLenInByte.BlockLenInByte_Byte4_msb << 24 );
                                        
                                        //arrange in little endian format for later use
                                        Context->Usb3BitInfo->BlockLenInByte = tmp;                                      
                                        Context->DeviceNumBlocksPerPage = NV_ICEIL(( 1 << Context->PageSizeLog2),Context->Usb3BitInfo->BlockLenInByte);
                                        Context->TransferLen[0] = tmp;
                                        break;
        case READ_FORMAT_CAPACITY_CMD_OPCODE : // media is assumed to be present and formatted
                                        tmp = ((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockLenInByte_Byte1_lsb \
                                              | (((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockLenInByte_Byte2 << 8) \
                                              | (((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockLenInByte_Byte3_msb << 16);
                                        Context->TransferLen[0] = tmp;
                                        Context->Usb3BitInfo->BlockLenInByte = tmp;                                      

                                        tmp = ((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockNumInByteUnion.stBlockNumInByte.BlockNumInByte_Byte1_lsb \
                                              | (((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockNumInByteUnion.stBlockNumInByte.BlockNumInByte_Byte2 << 8) \
                                              | (((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockNumInByteUnion.stBlockNumInByte.BlockNumInByte_Byte3 << 16) \
                                              | (((ReadFmtCapacityResponse *)BufferXusbData)->CurCapHdr.BlockNumInByteUnion.stBlockNumInByte.BlockNumInByte_Byte4_msb << 24) ;
                                        //arrange in little endian format
                                        Context->Usb3BitInfo->NumBlocks = tmp;
       				     Context->Usb3BitInfo->LastLogicalBlkAddr = tmp;
                                        break;
        /* No data stage
        case WRITE10_CMD_OPCODE       : 
        case TESTUNITREADY_CMD_OPCODE :
        */                                
        default                       : break;                              
    }
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbMscProcessResetRecovery(NvBootUsb3Context *Context, USB_Endpoint_Type EpDir)
{
    //Bulk only Mass Storage Reset
    //Clear feature HALT to the bulk in endpoint
    //Clear feature HALT to bulk out endpoint
    return NvBootXusbProcessEpStallClearRequest(Context, EpDir);
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbMscBotReceiveCSW(NvBootUsb3Context *Context)
{
    NvBootError ErrorCode;

    // Prepare normal TRB for receiving CSW
    //Refer to section 6.4.1 of Data Structures chapter of XHCI spec for more details about structure of normal TRB
    // Compensate buffer for xusb view of sysram (T18x only)
    NvBootXusbPrepareNormalTRB(Context,BufferXusbStatus, sizeof(CommandBlockStatus));

    NvBootXusbPrepareEndTRB(Context);

    // Update endpoint context structure
    // For the endpoint context, refer to section 6.2.3 and please also refer to section 5.4.5.9 of the USB3 Frontend IAS at 
    // https://p4viewer.nvidia.com/get///dev/stdif/usb3/1.0/doc/IAS/USB3_Frontend_IAS.doc
    NvBootXusbUpdateEpContext( Context, USB_BULK_IN);
    
    //Submit the request to workQ and wait for it to be completed by checking compQ
    ErrorCode = NvBootXusbUpdWorkQAndChkCompQ(Context);
    if(ErrorCode == NvBootError_Success)
    {

        //Set the default error code. It will be updated if a good CSW is found   
        ErrorCode = NvBootError_XusbMscResetRecovery;

        //Validate CSW
        //Refer to http://www.usb.org/developers/devclass_docs/usbmassbulk_10.pdf
        if((((CommandBlockStatus *)BufferXusbStatus)->CSWSignature == CSW_SIGNATURE) && \
            (((CommandBlockStatus *)BufferXusbStatus)->CSWTag ==  ((CommandBlockWrapper *)BufferXusbCmd)->CBWTag))
        {
            Context->Usb3BitInfo->CurrCmdCSWStatus          = ((CommandBlockStatus *)BufferXusbStatus)->CSWStatus;
            Context->Usb3BitInfo->CurrCSWTag                = ((CommandBlockStatus *)BufferXusbStatus)->CSWTag;
            if(Context->Usb3BitInfo->CurrCmdCSWStatus != CSW_STATUS_CMD_PHASE_ERR)
            {
                //Log number of bytes remaining to be received/sent
                Context->Usb3BitInfo->CurrEpBytesNotTransferred =((CommandBlockStatus *)BufferXusbStatus)->CSWDataResidue; 
                if(Context->Usb3BitInfo->CurrCmdCSWStatus == CSW_STATUS_CMD_GOOD)
                {
                    //NvBootError_XusbCswStatusCmdGood;
                    ErrorCode = NvBootError_Success; // Upper layer functions require this code.
                }
                else
                {
                    ErrorCode = NvBootError_XusbCswStatusCmdFail;
                }
          
            }
            else
            {
                //phase error, Do reset recovery
                //NvBootError ErrorCode = NvBootError_XusbMscResetRecovery; //Optimized code
            }
            
        }
        else
        {
            //Invalid CSW, Do reset recovery
            //NvBootError ErrorCode = NvBootError_XusbMscResetRecovery;    //Optimized code
        }
    }
    return ErrorCode;
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbMscBotReceiveData(NvBootUsb3Context *Context, uint8_t opcode)
{
    NvBootError ErrorCode = NvBootError_Success;

    if((opcode != READ10_CMD_OPCODE)  && (opcode != WRITE10_CMD_OPCODE))
    {
        //BufferXusbData is 16 byte aligned global memory area of size 128 bytes. 
        //This buffer is good enough for receiving reponses for commands other than READ and WRITE 
        //Read page and write page routines are expected to set the destination address in BufferData 
        //field of structure pointed by Context input variable
        // Adjust Sysram address only for driver initiated scsi commands.
        // Read/Writes coming from out of driver should have adjusted addresses.
        Context->BufferData = (uint8_t*)((NvU32)BufferXusbData);
    }

    //Refer to section 6.4.1 of Data Structures chapter of XHCI spec for more details about structure of normal TRB
    // Compensate for XUSB vs R5 view of Sysram (T18x only)
    //NvBootXusbPrepareNormalTRB(Context,Context->BufferData-SYSRAM_DIFFERENCE,((CommandBlockWrapper *)(BufferXusbCmd))->CBWDataTransferLength);
    NvBootXusbPrepareNormalTRB(Context,Context->BufferData,((CommandBlockWrapper *)(BufferXusbCmd))->CBWDataTransferLength);

    NvBootXusbPrepareEndTRB(Context);

    // Update endpoint context structure
    // For the endpoint context, refer to section 6.2.3 and please also refer to section 5.4.5.9 of the USB3 Frontend IAS at 
    // https://p4viewer.nvidia.com/get///dev/stdif/usb3/1.0/doc/IAS/USB3_Frontend_IAS.doc
    if (opcode == WRITE10_CMD_OPCODE)
        NvBootXusbUpdateEpContext( Context, USB_BULK_OUT);
    else 
        NvBootXusbUpdateEpContext( Context, USB_BULK_IN);

    //Submit the request to workQ and wait for it to be completed by checking compQ
    ErrorCode = NvBootXusbUpdWorkQAndChkCompQ(Context);
    return ErrorCode;
}
//_________________________________________________________________________________________________

NvBootError NvBootXusbMscBotSendCBW(NvBootUsb3Context *Context, uint8_t opcode)
{
    //Prepare Command Block Wrapper  
    NvBootXusbMscPrepareCBW(Context,opcode);

    //Prepare normal TRB. Refer to section 6.4.1 of Data Structures chapter of XHCI spec
    // Compensate for XUSB view of SYSRAM.
    NvBootXusbPrepareNormalTRB(Context,BufferXusbCmd, sizeof(CommandBlockWrapper));

    NvBootXusbPrepareEndTRB(Context);

    //Update endpoint context data structure . For the endpoint context structure refer to 6.2.3 section of XHCI spec. 
    //Also refer to section 5.4.5.9 of USB3 front end IAS
    NvBootXusbUpdateEpContext(Context,USB_BULK_OUT); // enque ptr and packetsize could be retrieved from Context
    
    return NvBootXusbUpdWorkQAndChkCompQ(Context);
}

//_________________________________________________________________________________________________

NvBootError NvBootXusbMscBotProcessRequest(NvBootUsb3Context *Context, uint8_t opcode)
{
    NvBootError ErrorCode, ReceiveDataStatus = NvBootError_Force32;
    CommandBlockWrapper *BufferCBW = ((CommandBlockWrapper *)BufferXusbCmd);
    NvU32 RetryCount = USB_MAX_TXFR_RETRIES;
    USB_Endpoint_Type EpType = USB_BULK_OUT;

    NvBootUtilMemset((void *)&TRBRing[0],0,TRB_RING_SIZE);//To make sure that all reserved bits are set to 0
    while(RetryCount)
    {
	    //Send Command
	    EpType = USB_BULK_OUT;
	    ErrorCode = NvBootXusbMscBotSendCBW(Context,opcode);

	    if(ErrorCode == NvBootError_Success)
	    {   //command sent successfully
	        
			if(BufferCBW->CBWDataTransferLength != 0)
			{   
				//Receive Data if number of bytes expected in data stage is non zero 
				if(BufferCBW->CBWFlags == CBWFlags_DATA_DIR_IN)
				{
					//Receive Data
				        EpType = USB_BULK_IN;
					ErrorCode = NvBootXusbMscBotReceiveData(Context,opcode);
					ReceiveDataStatus = ErrorCode;
				}
				else
				{
					//Send Data 
					//Only command that would fall in this category is WRITE10 command. Bootrom does not support it
					//Nothing to send so its better to reset device state variables. Bootrom is expected not to hit this case
					ErrorCode = NvBootXusbMscBotReceiveData(Context,opcode);
					ReceiveDataStatus = ErrorCode;
					//                ErrorCode = NvBootError_XusbMscResetRecovery;
				}
			}
			//Data (if there is any) received successfully
			if(ErrorCode == NvBootError_Success)
			{
				//Receive Command Status Wrapper
				EpType = USB_BULK_IN;
				ErrorCode = NvBootXusbMscBotReceiveCSW(Context);
			}
		}
            if((ErrorCode == NvBootError_Success) || (ErrorCode == NvBootError_XusbEpStalled) || (ErrorCode == NvBootError_XusbMscResetRecovery))
                break;
            else
            {   
                RetryCount--;
                NvBootUtilWaitUS(TIMEOUT_1MS);
                if(RetryCount)
                    ErrorCode = NvBootError_Success;
            }
	}//retry loop
	if((ErrorCode == NvBootError_Success) && (ReceiveDataStatus == NvBootError_Success))
	{
	    //CSW received successfully and 
	    //Data (if it was supposed to be received) was also received successfully
	    NvBootXusbMscBotAnalyzeReceiveData(Context,opcode);
	}

    if((ErrorCode == NvBootError_XusbEpStalled) || (ErrorCode == NvBootError_XusbMscResetRecovery))
    {
        ErrorCode = NvBootXusbMscProcessResetRecovery(Context,EpType);
    }
    return ErrorCode;
}
//_________________________________________________________________________________________________

