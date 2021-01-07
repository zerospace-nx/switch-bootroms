/** 
 *Insert Nvidia copyright
 */
#include "nvboot_clocks_int.h"
#include "nvboot_car_int.h"
#include "nvboot_error.h"
//extern Endpoint_T;
NvBootError NvBootXusbDeviceInit(void);
NvBootError NvBootXusbDeviceEnumerate(uint8_t* Buffer);
NvBootError NvBootXusbDeviceReceive(uint8_t* Buffer, NvU32 Bytes,  NvU32 *pBytesReceived);
NvBootError NvBootXusbDeviceTransmit(uint8_t* Buffer, NvU32 Bytes, NvU32 *pBytesTransmitted);
NvBootError NvBootXusbDeviceReceiveStart(uint8_t* Buffer, NvU32 Bytes);
NvBootError NvBootXusbDeviceReceivePoll(NvU32 *pBytesReceived, NvU32 TimeoutMs, uint8_t* OptionalBuffer);
NvBootError NvBootXusbDeviceTransmitStart(uint8_t* Buffer, NvU32 Bytes);
NvBootError NvBootXusbDeviceTransmitPoll(NvU32 *pBytesTransferred, NvU32 TimeoutMs, uint8_t* OptionalBuffer);
NvBootError NvBootXusbHandleError(void);
NvBootError NvBootXusbDevicePerformTracking(NvBootClocksOscFreq);
NvBootError NvBootXusbDeviceSetupSWControlUTMIPll(NvBootClocksOscFreq);
void NvBootXusbDeviceGetClockTable(void **XusbClockTable, ClockTableType *Id);




