/** 
 *Insert Nvidia copyright
 */
#include "nvboot_clocks_int.h"
#include "nvboot_error.h"

NvBootError NvBootXusbDeviceInitializeEventRing(void);
static NvBootError NvBootPollField(NvU32 RegAddr,
                                   NvU32 Mask,
                                   NvU32 ExpectedValue,
                                   NvU32 Timeout);
NvBootError NvBootXusbDevicePollForEvent(NvU32 Timeout);
NvBootError NvBootXusbDeviceSetConfiguration(NvU8* pSetupData);
NvBootError NvBootXusbDeviceSetAddress(NvU8* pSetupData);
NvBootError NvBootXusbDeviceGetDescriptor(NvU8* pSetupData);
NvBootError NvBootXusbDeviceEPGetStatus(NvU8* pSetupData);
NvBootError
NvBootXusbDeviceIssueDataTRB(NvU32 Buffer, NvU32 Bytes, NvU32 Direction);
NvBootError NvBootXusbDeviceIssueStatusTRB(NvU32 Direction);
NvBootError NvBootXusbDeviceHandleSetupPacket( NvU8* pSetupData);
NvBootError NvBootXusbDeviceInit(void);
NvBootError NvBootXusbDeviceEnumerate(NvU8* Buffer);
NvBootError NvBootXusbDeviceHandlePortStatusChange(void);
NvBootError NvBootXusbDeviceClkInit(void);
NvBootError NvBootXusbDevicePadSetup(void);
NvBootError NvBootXusbDeviceSetupHWControlUTMIPll(NvBootClocksOscFreq);
NvBootError NvBootXusbDeviceSetupHWControlPLLU(void);
NvBootError NvBootXusbDeviceSetupStaticParamsUTMI(void);
NvBootError NvBootXusbDevicePerformTracking(NvBootClocksOscFreq);
NvBootError NvBootXusbDeviceRemovePowerDownUTMI(void);
NvBootError NvBootXusbDeviceSetupSWControlUTMIPll(NvBootClocksOscFreq OscFreq);
NvBootError NvBootXusbDeviceReceive(NvU8* Buffer, NvU32 Bytes,  NvU32 *pBytesReceived);
NvBootError NvBootXusbDeviceTransmit(NvU8* Buffer, NvU32 Bytes, NvU32 *pBytesTransmitted);
NvBootError NvBootXusbDeviceReceiveStart(NvU8* Buffer, NvU32 Bytes);
NvBootError NvBootXusbDeviceReceivePoll(NvU32 *pBytesReceived, NvU32 TimeoutMs, NvU8* OptionalBuffer);
NvBootError NvBootXusbDeviceTransmitStart(NvU8* Buffer, NvU32 Bytes);
NvBootError NvBootXusbDeviceTransmitPoll(NvU32 *pBytesTransferred, NvU32 TimeoutMs, NvU8* OptionalBuffer);
NvBootError NvBootXusbHandleError(void);

