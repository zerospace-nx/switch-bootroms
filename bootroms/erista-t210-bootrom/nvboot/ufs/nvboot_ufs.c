/** Insert Nvidia header
 */
#include "nvboot_ufs.h"
#include "nvboot_ufs_hci.h"
#include "nvboot_ufs_hc_reg.h"
#include "nvrm_drf.h"
#include "nvboot_error.h"
#include <string.h>

/** NV_DRF offers macro to generate shift mask from range
 * Add macro to create RANGE to pass to NV_DRF SHIFTMASK
 */
#define SHIFTMASK(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFTMASK(PERIPH##_##REG##_0_##FIELD##_##RANGE)
#define SHIFT(PERIPH, REG, FIELD) \
        NV_FIELD_SHIFT(PERIPH##_##REG##_0_##FIELD##_##RANGE)
/* READ/WRITE MACROS */
#define UFS_ADDR(REG) (UFS_##REG)

#define UFS_READ32(REG) \
        NV_READ32(UFS_ADDR(REG));

#define NV_READ32(ADDR) \
        (*(volatile unsigned int*)(ADDR))

#define UFS_WRITE32(REG, VALUE) \
        NV_WRITE32(UFS_ADDR(REG), VALUE);

#define NV_WRITE32(ADDR, VALUE) \
        *(volatile unsigned int*)(ADDR) = VALUE
#define TRUE 1
#define FALSE 0

/** Macros to convert endianness
 */
#define BYTE_SWAP32(a)   \
        ((((a)&0xff) << 24) | (((a)&0xff00)<< 8) | \
        (((a)&0xff0000) >> 8) | (((a)&0xff000000) >> 24))
#define BYTE_SWAP16(a)   \
        ( a&0xff00<< 8 | a&0xff0000 >> 8)     
/* TODO Find HCE SET wait time */
#define HCE_SET_TIMEOUT             1000
#define UTRLRDY_SET_TIMEOUT         1000
#define UTMRLRDY_SET_TIMEOUT        1000
#define NOP_TIMEOUT                 1000
#define QUERY_REQ_DESC_TIMEOUT      1000
#define QUERY_REQ_FLAG_TIMEOUT      1000
#define SCSI_REQ_READ_TIMEOUT       1000
#define QUERY_REQ_ATTRB_TIMEOUT     1000
/* UFS Sector size 512 bytes */
#define UFS_MIN_BLOCK_SIZE_LOG2 9
#define UFS_PAGE_SIZE   1<< UFS_MIN_BLOCK_SIZE_LOG2

extern void NvBootUtilWaitUS(NvU32);

/***** TODO Define context structure 
* 1. NEED Number of TRD/Cmd Descriptors Used. Last TRDIndex, TRDInUse, Response UPIU.
* 2. NEED BOOT LUN or NOT.
* 3. Response UPIU
* 4. TRD Info (start time, timeout)
* 5. BootEnabled?
* 6. Number of Luns.
* 7. LUN block size. (= Bootrom page size)
*/

/** Static structure of TRDs in system memory aligned to 1KB boundary 
 * TODO: Do we need 32 slots? 
 */
#define MAX_TRD_NUM    4

#define NEXT_TRD_IDX(idx) (((idx) == ((MAX_TRD_NUM) - 1)) ? 0 : ((idx) + 1))
static TransferRequestDescriptor_T TxReqDesc[MAX_TRD_NUM] __attribute__((aligned(1024)));
/** Static structure of TMDs in system memory aligned to 1KB boundary
 * TODO: Do we need 8 slots ?
 */
static TaskMgmtRequestDescriptor_T TaskMgmntDesc[8] __attribute__((aligned(1024)));

/** Static structure of Cmd Descriptors and related defines
 * Reserving only 4. BootROM will not need more.
 */
#define MAX_CMD_DESC_NUM    2
#define NEXT_CD_IDX(idx) (((idx) == ((MAX_CMD_DESC_NUM) - 1)) ? 0 : ((idx) + 1))
static CmdDescriptor_T CmdDescriptors[MAX_CMD_DESC_NUM] __attribute__((aligned(128)));
/** End Command Descriptors and related defines */

/* Create UFS context structure
*/
typedef struct
{
    NvU32 TRDTimeOutInUs;
    NvU32 TRDStartTime;
    NvU32 TRDTimeout;
} TRDInfo_T;

typedef struct
{
    NvU32 BootLUNNumBlocks;
    NvU32 BootLUNBlockSize;
    NvU32 BootEnabled;
    NvU32 NumLUN;
    NvU32 NumWLU;
    NvU32 FuseParamLUN;
    NvU32 CmdDescInUse;
    NvU32 LastCmdDescIndex;
    NvU32 TxReqDescInUse;
    NvU32 LastTRDIndex;
    TRDInfo_T   TRDInfo[MAX_TRD_NUM];
    UCDGenericRespUPIU_T UCDGenericRespUPIU;
} UFSContext_T;


static UFSContext_T UFSContext;

NvU32 NvBootUFSGetCmdDescriptor(NvU32 *pCmdDescIndex)
{
    NvU32 NextCmdIndex;
    UFSContext_T *pUFSContext = &UFSContext;

    if(pUFSContext->CmdDescInUse < MAX_CMD_DESC_NUM)
    {
        NextCmdIndex = NEXT_CD_IDX(pUFSContext->LastCmdDescIndex);
        pUFSContext->LastCmdDescIndex = NextCmdIndex;
        pUFSContext->CmdDescInUse++;
        return NvBootError_Success;
    }
    else 
        return NvBootError_UFSResourceMax;
}

NvU32 NvBootPollField(NvU32 RegAddr, NvU32 Mask, NvU32 ExpectedValue, NvU32 Timeout)
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



NvU32 NvBootUFSInit()
{
    NvU32 RegData, Error, ShiftExpectedValue=0;
    /** Enable clocks and bring out of reset
     * MIPI PHY does not require clock as PWM is self clocked.
     * But UFS spec requires reference clock. UFS spec 6.6
     * Supported ref clk frequencies are 19.2, 26, 38.4, 52 Mhz.
     */
    /* Place holder for CAR related code
     * <INSERT CODE HERE>
     */

    /* Take unit out of reset
     * <INSERT CODE HERE>
     */

    /* Write Enable to UFS HCE */
    RegData = UFS_READ32(HCE);
    RegData = NV_FLD_SET_DRF_NUM(UFS,HCE,HCE,1,RegData);
    UFS_WRITE32(HCE, RegData);

    /* Poll for Initialization sequence to be complete */
    ShiftExpectedValue = 1 << SHIFT(UFS,HCE,HCE);
    Error = NvBootPollField(UFS_ADDR(HCE), SHIFTMASK(UFS,HCE,HCE), ShiftExpectedValue, HCE_SET_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;

    /** Send Additional DME Set commands followed by DME_LINKSTARTUP Commands.
     * DME Commands are send using MMIO UIC registers 
     * DP must be set by the end of this routine.
     */
    NvBootUFSDMELinkSetup();

    NvBootUFSSetupTRDTMLists();
    /* Start TM and TR processing engines */
    NvBootUFSStartTMTREngines();

    /* Check if Device is Ready to receive UPIUs*/
    Error = NvBootUFSChkIfDevRdy2RcvDesc();
    if(Error != NvBootError_Success)
        return Error;

    Error = NvBootGetDevInfoPartialInit();
    if((Error != NvBootError_UFSBootNotEnabled) && (Error != NvBootError_Success))
        return Error;

    /* Device is not boot enabled */
    if(Error == NvBootError_UFSBootNotEnabled)
    {
        Error = NvBootUFSCompleteInit();
        if(Error != NvBootError_Success);
            return Error;

        Error = NvBootGetDevInfo();
        if(Error != NvBootError_Success);
            return Error;
    }
    return NvBootError_Success;
}

/** Start Engines for processing Task Management
 * and TRD List. UTMRLRDY and UTRLRDY should be set
 */
NvU32 NvBootUFSStartTMTREngines(void)
{
    NvU32 ShiftExpectedValue, Error, RegData;
    /** Start Processing Engine for TM List and TRD list. 
     * TODO Define UTMRLRDY and UTRLRDY SET TIMEOUT
     */

    RegData = UFS_READ32(UTMRLRSR);
    RegData = NV_FLD_SET_DRF_NUM(UFS,UTMRLRSR,UTMRLRSR,1,RegData);
    UFS_WRITE32(UTMRLRSR, RegData);
    
    RegData = UFS_READ32(UTRLRSR);
    RegData = NV_FLD_SET_DRF_NUM(UFS,UTRLRSR,UTRLRSR,1,RegData);
    UFS_WRITE32(UTRLRSR, RegData);
    
    ShiftExpectedValue = 1 << SHIFT(UFS,HCS,UTMRLRDY);
    Error = NvBootPollField(UFS_ADDR(HCS), SHIFTMASK(UFS,HCS,UTMRLRDY), ShiftExpectedValue, UTMRLRDY_SET_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;
    ShiftExpectedValue = 1 << SHIFT(UFS,HCS,UTRLRDY);
    Error = NvBootPollField(UFS_ADDR(HCS), SHIFTMASK(UFS,HCS,UTRLRDY), ShiftExpectedValue, UTRLRDY_SET_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;
    return NvBootError_Success;
}
void NvBootUFSSetupTRDTMLists()
{
    /* Write Transfer Request List Lower and Upper Base Address */
    UFS_WRITE32(UTRLBA, (NvU32)&TxReqDesc[0]);
    UFS_WRITE32(UTRLBAU, 0);
    
    /* Write Task Management Request List Lower and Upper Base Address */
    UFS_WRITE32(UTMRLBA, (NvU32)&TaskMgmntDesc[0]);
    UFS_WRITE32(UTMRLBAU, 0);
}

/** Get a TRD slot from TRD list if available else
 *  return error.
 */
NvU32 NvBootUFSGetTxReqDescriptor(NvU32 *pTRDIndex)
{
    NvU32 TRDIndex, RegData;
    UFSContext_T *pUFSContext = &UFSContext;
    if(pUFSContext->TxReqDescInUse < MAX_TRD_NUM)
    {
        TRDIndex = NEXT_TRD_IDX(pUFSContext->LastTRDIndex);
        RegData = UFS_READ32(UTRLDBR);
        if(RegData & (1 << TRDIndex))
            return NvBootError_UFSFatalError;
        *pTRDIndex= TRDIndex;
        pUFSContext->LastTRDIndex = TRDIndex;
        pUFSContext->TxReqDescInUse++;
        return NvBootError_Success;
    }
    else
        return NvBootError_UFSResourceMax;
}
/** Creates TRD from given GenericCmdDescriptor (if slot available in TRD List)
 *  and returns success else returns UFSResourceMax Error.
 */
 
NvU32 NvBootUFSCreateTRD(NvU32 TRDIndex, NvU32 CmdDescIndex, NvU32 CmdType)
{
    TransferRequestDescriptor_T *pTxReqDesc;
    NvU32 RegData;
    /* Get TRD */
    
    
    pTxReqDesc = &TxReqDesc[TRDIndex];
    memset((void*)pTxReqDesc, 0, sizeof(TransferRequestDescriptor_T));
    /* Command Type and Data Direction */
    RegData = 0;
    RegData = NV_FLD_SET_DRF_NUM(UFS_TRD, DW0, CT, CmdType, RegData);
    if(CmdType == UFS_TRD_DW0_0_CT_SCSI)
        RegData = NV_FLD_SET_DRF_NUM(UFS_TRD, DW0, DD, UFS_TRD_DW0_0_DD_READ, RegData);
    pTxReqDesc->DW0_CmdTypeDD = RegData;
    /* Fill in Cmd Desc Lower and Upper Address. */
    pTxReqDesc->DW4_CDBaseAddr = (NvU32)&CmdDescriptors[CmdDescIndex];
    pTxReqDesc->DW5_CDBaseAddrU = 0;

    /* Fill in Resp UPIU length and offset */
    RegData =0;
    RegData = NV_FLD_SET_DRF_NUM(UFS_TRD, DW6, RUL, sizeof(UCDGenericRespUPIU_T), RegData);
    RegData = NV_FLD_SET_DRF_NUM(UFS_TRD, DW6, RUO, sizeof(UCDGenericReqUPIU_T), RegData);
    pTxReqDesc->DW6_RspOffsetLen = RegData;
    
    if(CmdType == UFS_TRD_DW0_0_CT_SCSI) {
        RegData =0;
        RegData = NV_FLD_SET_DRF_NUM(UFS_TRD, DW7, PRDTL, sizeof(PRDT_T), RegData);
        RegData = NV_FLD_SET_DRF_NUM(UFS_TRD, DW7, PRDTO,    \
                  sizeof(UCDGenericReqUPIU_T)+ sizeof(UCDGenericRespUPIU_T), RegData);
        pTxReqDesc->DW7_PRDTOffsetLen = RegData;
    }
    return NvBootError_Success;
}

NvU32 NvBootUFSQueueTRD(NvU32 TRDIndex, NvU32 TRDTimeout)
{
    NvU32 RegData;
    UFSContext_T *pUFSContext = &UFSContext;
    /** Confirm that slot *as calculated* is indeed empty
     *  else return error 
     */
    RegData = UFS_READ32(UTRLDBR);
    if(RegData & (1 << TRDIndex))
    {
        NvBootUFSFreeTRDCmdDesc();
        return NvBootError_UFSFatalError;
    }
    /* All slots except the required should be set to zero */
    RegData = 1 << TRDIndex;
    RegData =  NV_FLD_SET_DRF_NUM(UFS, UTRLDBR, UTRLDBR, 1, RegData);
    UFS_WRITE32(UTRLDBR, RegData);
    /* define TRD Info struct */
    memset((void*)&pUFSContext->TRDInfo[TRDIndex], 0, sizeof(TRDInfo_T));
    pUFSContext->TRDInfo[TRDIndex].TRDStartTime = NvBootUtilGetTimeUS();
    pUFSContext->TRDInfo[TRDIndex].TRDTimeout = TRDTimeout;
    return NvBootError_Success;
}

NvU32 NvBootUFSWaitTRDRequestComplete(NvU32 TRDIndex, NvU32 Timeout)
{
    NvU32 RegData, Error, ShiftExpectedValue, Mask;
    NvU32 RspUPIULength = 0, RspUPIUOffset = 0;
    TransferRequestDescriptor_T *pTxReqDesc;
    UFSContext_T *pUFSContext = &UFSContext;
    ShiftExpectedValue = Mask = 1 << TRDIndex;
    /* TODO TIMEOUT for DOORBELL SET */
    if(Timeout)
    {   
        Error = NvBootPollField(UFS_ADDR(UTMRLDBR), Mask, ShiftExpectedValue, Timeout);
        if(Error!= NvBootError_Success)
            return Error;
    }
    else
    {
        RegData = UFS_READ32(UTMRLDBR);
        if(!(RegData & Mask))
        {
            if(NvBootUtilElapsedTimeUS(pUFSContext->TRDInfo[TRDIndex].TRDStartTime) >
                 pUFSContext->TRDInfo[TRDIndex].TRDTimeOutInUs)
                return NvBootError_UFSTRDTimeout;
            else return NvBootError_UFSTRDInProgress;
        }
    }
    RegData = UFS_READ32(IS);
    if(NV_DRF_VAL(UFS,IS,SBFES, RegData) | NV_DRF_VAL(UFS,IS,HCFES, RegData) \
       | NV_DRF_VAL(UFS,IS,UTFES, RegData) | NV_DRF_VAL(UFS,IS,DFES, RegData))
        return NvBootError_UFSFatalError;
    /* TODO Restart controller etc */
    /* Copy Response UPIU into UFS context */
    pTxReqDesc = &TxReqDesc[TRDIndex];
    RegData = pTxReqDesc->DW6_RspOffsetLen;
    RspUPIUOffset = NV_DRF_VAL(UFS_TRD, DW6, RUO, RegData);
    RspUPIULength = NV_DRF_VAL(UFS_TRD, DW6, RUL, RegData);
    memcpy((void*) &pUFSContext->UCDGenericRespUPIU, \
           (void*)(pTxReqDesc->DW4_CDBaseAddr+ RspUPIUOffset),
            RspUPIULength);
        
    return NvBootError_Success;
}

void NvBootUFSFreeTRDCmdDesc(void)
{
    UFSContext_T *pUFSContext = &UFSContext;
    pUFSContext->TxReqDescInUse--;
    pUFSContext->CmdDescInUse--;
    /* Free associated command descriptor */
}

NvU32 NvBootUFSChkIfDevRdy2RcvDesc()
{
    /* Send NOPs OUT till you get NOP in */
    NOP_UPIU_T *pNOP_UPIU;
    CmdDescriptor_T *pCmdDescriptor;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success, NOPReceived =0;

    do
    {
        Error = NvBootUFSGetTxReqDescriptor(&TRDIndex);
        if(Error !=NvBootError_Success)
            return Error;

        Error = NvBootUFSGetCmdDescriptor(&CmdDescIndex);
        if(Error != NvBootError_Success)
            return Error;

        pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
        memset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

        /* Create NOP_OUT UPIU. Only Transaction Code needed */
        pNOP_UPIU = (NOP_UPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
        pNOP_UPIU->BasicHeader.TransType = UPIU_NOP_OUT_TRANSACTION;

        Error = NvBootUFSCreateTRD(TRDIndex, CmdDescIndex, UFS_TRD_DW0_0_CT_DM);
        if(Error!= NvBootError_Success)
            return Error;
        // /* TODO Timeout for NOP */
        Error = NvBootUFSQueueTRD(TRDIndex, NOP_TIMEOUT);
        if(Error != NvBootError_Success)
            return Error;
        
        /* TODO Timeout for NOP */
        Error = NvBootUFSWaitTRDRequestComplete(TRDIndex, NOP_TIMEOUT);
        if(Error!= NvBootError_Success)
            return Error;
        NvBootUFSFreeTRDCmdDesc();
        NOPReceived = NvBootUFSIsRespNOPIN();
    }
    while(!NOPReceived);
    return NvBootError_Success;
    /* TODO HOW MANY TIMES SHOULD WE SEND NOP OUTs */
}

NvU32 NvBootUFSIsRespNOPIN()
{
    UFSContext_T *pUFSContext = &UFSContext;
    if(pUFSContext->UCDGenericRespUPIU.NOP_UPIU.BasicHeader.TransType == \
        UPIU_NOP_IN_TRANSACTION)
        return TRUE;
    else 
        return FALSE;
}
/** Get Descriptor to identify device characteristics
 */
NvU32 NVBootUFSGetDescriptor(NvU8 *pUFSDesc, NvU32 DescIDN, NvU32 DescIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    UFSContext_T *pUFSContext = &UFSContext;
    QueryReqRespUPIU_T *pQueryReqRespUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUFSGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUFSGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    memset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqRespUPIU->BasicHeader.TransType = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqRespUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_TM_FUNC_STD_READ;
    pQueryReqRespUPIU->Opcode = QUERY_REQ_STD_READ_READ_DESC;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = DEVICE_DESC_IDN;
    pQueryReqRespUPIU->TSF.DescFields.DescIDN = DescIDN;
    /* Index */
    pQueryReqRespUPIU->TSF.DescFields.Index = DescIndex;
    /** TODO Find out device descriptor size. It is ok to simply specify max desc size
     * of 255 bytes as device will return actual descriptor size if lesser.
     */
    pQueryReqRespUPIU->TSF.DescFields.LengthBigE = BYTE_SWAP16(255);
    
    /* TODO Task Tags */
    Error = NvBootUFSCreateTRD(TRDIndex, CmdDescIndex, UFS_TRD_DW0_0_CT_DM);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ DESCRIPTOR */
    Error = NvBootUFSQueueTRD(TRDIndex, QUERY_REQ_DESC_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for NOP */
    Error = NvBootUFSWaitTRDRequestComplete(TRDIndex, QUERY_REQ_DESC_TIMEOUT);
    if(Error!= NvBootError_Success)
        return Error;
    NvBootUFSFreeTRDCmdDesc();
    /* TODO FIX THIS . */
    memcpy((void*)pUFSDesc, \
           (void*)&(pUFSContext->UCDGenericRespUPIU.QueryReqRespUPIU.DataSegment[0]), \
            UFS_DESC_MAX_SIZE);
    return NvBootError_Success;
}

/* This routine is called if BootDisable Fuse is not set.
 * It assumes BootLUN is available and enabled (else returns appropriate error)
 * This is done after Partial Initilization. If we find Boot is not enabled,
 * we have to exit and complete initialization.
 */
NvU32 NvBootGetDevInfoPartialInit()
{
    NvU32 Error, BootLUNID, FoundBootLUN = FALSE, LUN;
    NvU8 DescBuffer[UFS_DESC_MAX_SIZE];
    UFSDeviceDesc_T *pDevDesc = 0;
    UFSUnitDesc_T *pUnitDesc = 0;
    UFSContext_T *pUFSContext = &UFSContext;
    

    /** First things first. Get Device descriptor, geometry?
     * Device Descriptor itself may not be accessible during partial initialzation.
     * So keep small timeout. TODO
     * 1. Is device Bootable ?? If yes, go to 2.
     * 2. Find out active Boot LUN (A or B) by Reading Attribute BootLunEn.
     * 3. Have to go through All LUN Unit descriptors to find out Boot Lun.
     * 4. Get Unit Descriptor of identified LUN to find out Block Size,LUN Size
     */
    Error = NvBootUFSGetDescriptor(&DescBuffer[0], DEVICE_DESC_IDN, 0);
    if(Error != NvBootError_Success)
        return Error;
    pDevDesc = (UFSDeviceDesc_T *)&DescBuffer[0];
    
    /* Get what we need from the device descriptor */
    pUFSContext->BootEnabled = pDevDesc->bBootEnable;
    pUFSContext->NumLUN = pDevDesc->bNumberLU;
    /* Do we need this ? */
    pUFSContext->NumWLU = pDevDesc->bNumberWLU;

    if(pUFSContext->BootEnabled)
    {
        /* BIG TODO ReadAttribute to get BootLunEn*/
        /* Find out if Boot LUN A or B is mapped */
        NvBootUFSGetAttribute(&BootLUNID, BOOT_LUN_EN_ATTRB, 0);
        /* Find out who is actually Boot LUN A or B. 
         * This step is only required to support GPT.
         * i.e to find out total size of Boot Partition.
         */
        for(LUN=0; LUN < pUFSContext->NumLUN; LUN++)
        {
            Error = NvBootUFSGetDescriptor(&DescBuffer[0], UNIT_DESC_IDN, LUN);
            if(Error != NvBootError_Success)
                return Error;
            pUnitDesc = (UFSUnitDesc_T *)&DescBuffer[0];
            if(pUnitDesc->bBootLUNID == BootLUNID)
            {   
                FoundBootLUN = TRUE;
                break;
            }
        }
        if(!FoundBootLUN)
            return NvBootError_UFSBootLUNNotFound;
        
        /* Make sure LUN is enabled */
        if(!pUnitDesc->bLUEnable)
            return NvBootError_UFSBootLUNNotEnabled;

        /* Extract useful info from Unit descriptor */
        pUFSContext->BootLUNBlockSize = pUnitDesc->bLogicalBlockSize << UFS_MIN_BLOCK_SIZE_LOG2;
        pUFSContext->BootLUNNumBlocks = pUnitDesc->qLogicalBlockCount;
        return NvBootError_Success;
     }
    else
        return NvBootError_UFSBootNotEnabled;
}
NvU32 NvBootUFSGetAttribute(NvU32 *pUFSAttrb, NvU32 AttrbIDN, NvU32 AttrbIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    QueryReqRespUPIU_T *pQueryReqRespUPIU;
    UFSContext_T *pUFSContext = &UFSContext;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success, AttribBigE;

    Error = NvBootUFSGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUFSGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    memset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqRespUPIU->BasicHeader.TransType = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqRespUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_TM_FUNC_STD_READ;
    pQueryReqRespUPIU->Opcode = QUERY_REQ_STD_READ_READ_ATTRB;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = DEVICE_DESC_IDN;
    pQueryReqRespUPIU->TSF.AttrbFields.AttrbIDN = AttrbIDN;
    /* Index */
    pQueryReqRespUPIU->TSF.AttrbFields.Index = AttrbIndex;
    
    /* TODO Task Tags */
    Error = NvBootUFSCreateTRD(TRDIndex, CmdDescIndex, UFS_TRD_DW0_0_CT_DM);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ ATTRB */
    Error = NvBootUFSQueueTRD(TRDIndex, QUERY_REQ_ATTRB_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for ATTRB
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUFSWaitTRDRequestComplete(TRDIndex, 0);
    if(Error!= NvBootError_Success)
        return Error;
    NvBootUFSFreeTRDCmdDesc();
    AttribBigE = pUFSContext->UCDGenericRespUPIU.QueryReqRespUPIU.TSF.AttrbFields.ValueBigE;
    *pUFSAttrb = BYTE_SWAP32(AttribBigE);

    
    return NvBootError_Success;

}
NvU32 NvBootUFSGetFlag(NvU32 *pUFSFlag, NvU32 FlagIDN, NvU32 FlagIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    QueryReqRespUPIU_T *pQueryReqRespUPIU;
    UFSContext_T *pUFSContext = &UFSContext;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUFSGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUFSGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    memset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqRespUPIU->BasicHeader.TransType = UPIU_QUERY_REQUEST_TRANSACTION;
    pQueryReqRespUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_TM_FUNC_STD_READ;
    pQueryReqRespUPIU->Opcode = QUERY_REQ_STD_READ_READ_FLAG;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = DEVICE_DESC_IDN;
    pQueryReqRespUPIU->TSF.FlagFields.FlagIDN = FlagIDN;
    /* Index */
    pQueryReqRespUPIU->TSF.FlagFields.Index = FlagIndex;
    
    /* TODO Task Tags */
    Error = NvBootUFSCreateTRD(TRDIndex, CmdDescIndex, UFS_TRD_DW0_0_CT_DM);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ FLAG */
    Error = NvBootUFSQueueTRD(TRDIndex, QUERY_REQ_FLAG_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for FLAG
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUFSWaitTRDRequestComplete(TRDIndex, 0);
    if(Error!= NvBootError_Success)
        return Error;
    NvBootUFSFreeTRDCmdDesc();
    *pUFSFlag = pUFSContext->UCDGenericRespUPIU.QueryReqRespUPIU.TSF.FlagFields.FlagValue;
    
    return NvBootError_Success;

}

NvU32 NvBootUFSSetFlag(NvU32 FlagIDN, NvU32 FlagIndex)
{
    CmdDescriptor_T *pCmdDescriptor;
    QueryReqRespUPIU_T *pQueryReqRespUPIU;
    NvU32 TRDIndex = 0, CmdDescIndex = 0, Error = NvBootError_Success;
    NvU8 FlagReadBack;
    UFSContext_T *pUFSContext = &UFSContext;

    Error = NvBootUFSGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUFSGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    memset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));
    pQueryReqRespUPIU = (QueryReqRespUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pQueryReqRespUPIU->BasicHeader.TransType = UPIU_QUERY_REQUEST_TRANSACTION;
    /* TODO: Is SET FLAG a Read or Write Query function */
    pQueryReqRespUPIU->BasicHeader.QueryTMFunction = UPIU_QUERY_TM_FUNC_STD_WRITE;
    pQueryReqRespUPIU->Opcode = QUERY_REQ_STD_WRITE_SET_FLAG;
    /* IDN i.e Device OR Configuration OR Unit descriptor .. */
    // = DEVICE_DESC_IDN;
    pQueryReqRespUPIU->TSF.FlagFields.FlagIDN = FlagIDN;
    /* Index */
    pQueryReqRespUPIU->TSF.FlagFields.Index = FlagIndex;
    
    /* TODO Task Tags */
    Error = NvBootUFSCreateTRD(TRDIndex, CmdDescIndex, UFS_TRD_DW0_0_CT_DM);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for READ FLAG */
    Error = NvBootUFSQueueTRD(TRDIndex, QUERY_REQ_FLAG_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for FLAG
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUFSWaitTRDRequestComplete(TRDIndex, 0);
    if(Error!= NvBootError_Success)
        return Error;
    NvBootUFSFreeTRDCmdDesc();
    /* OSF6 LSB (Big Endian) TODO contains the returned flag*/
    FlagReadBack = pUFSContext->UCDGenericRespUPIU.QueryReqRespUPIU.TSF.FlagFields.FlagValue;
    if(FlagReadBack != 1)
        return NvBootError_UFSFlagSet;

    return NvBootError_Success;
}

NvU32 NvBootUFSCompleteInit(void)
{
    NvU32 Error, FlagDeviceInit;
    /* Set fDeviceInit flag and poll for fDeviceInit flag to be reset
     * by device to 0. The device will move from Boot W-LU Ready to 
     * Device Initialization Complete stage 
     */
    NvBootUFSSetFlag(DEVICE_INIT_IDN, 0);
    /* TODO: Timeout waiting for deviceInit */
    do
    {
        Error = NvBootUFSGetFlag(&FlagDeviceInit, DEVICE_INIT_IDN, 0);
        if(Error != NvBootError_Success)
            return Error;
    }
    while(FlagDeviceInit!=0);
    return NvBootError_Success;
}

/** Retrieves Device Descriptor and Unit descriptor for LUN from fuses.
 * Called only if Boot LUN is not enabled or Descriptor access failed 
 * during partial Init
 */
NvU32 NvBootGetDevInfo(void)
{
    NvU32 Error;
    NvU8 DescBuffer[UFS_DESC_MAX_SIZE];
    UFSDeviceDesc_T *pDevDesc;
    UFSUnitDesc_T *pUnitDesc;
    UFSContext_T *pUFSContext = &UFSContext;

    Error = NvBootUFSGetDescriptor(&DescBuffer[0], DEVICE_DESC_IDN, 0);
    if(Error != NvBootError_Success)
        return Error;
    pDevDesc = (UFSDeviceDesc_T *)&DescBuffer[0];
    
    /* Get what we need from the device descriptor */
    pUFSContext->BootEnabled = pDevDesc->bBootEnable;
    pUFSContext->NumLUN = pDevDesc->bNumberLU;
    /* Do we need this ? */
    pUFSContext->NumWLU = pDevDesc->bNumberWLU;


    Error = NvBootUFSGetDescriptor(&DescBuffer[0], UNIT_DESC_IDN, pUFSContext->FuseParamLUN);
    if(Error != NvBootError_Success)
        return Error;
    pUnitDesc = (UFSUnitDesc_T *)&DescBuffer[0];
    /* Make sure LUN is enabled */
    if(!pUnitDesc->bLUEnable)
        return NvBootError_UFSLUNNotEnabled;

    /* Extract useful info from Unit descriptor */
    pUFSContext->BootLUNBlockSize = pUnitDesc->bLogicalBlockSize << UFS_MIN_BLOCK_SIZE_LOG2;
    pUFSContext->BootLUNNumBlocks = pUnitDesc->qLogicalBlockCount;
    return NvBootError_Success;

}

NvU32 NvBootUFSRead(NvU32 Block, NvU32 Page, NvU32 *pBuffer)
{
    CmdDescriptor_T *pCmdDescriptor;
    CommandUPIU_T *pCommandUPIU;
    UFSContext_T *pUFSContext = &UFSContext;
    NvU32 TRDIndex = 0, RegData, CmdDescIndex = 0, Error = NvBootError_Success;

    Error = NvBootUFSGetTxReqDescriptor(&TRDIndex);
    if(Error !=NvBootError_Success)
        return Error;

    Error = NvBootUFSGetCmdDescriptor(&CmdDescIndex);
    if(Error != NvBootError_Success)
        return Error;

    pCmdDescriptor = &CmdDescriptors[CmdDescIndex];
    memset((void*)pCmdDescriptor, 0, sizeof(CmdDescriptor_T));

    pCommandUPIU = (CommandUPIU_T*)&pCmdDescriptor->UCDGenericReqUPIU;
    pCommandUPIU->BasicHeader.TransType = UPIU_COMMAND_TRANSACTION;
    /* TODO: Is SET FLAG a Read or Write Query function */
    RegData = 0;
    pCommandUPIU->BasicHeader.Flags = 1 << UFS_UPIU_FLAGS_R_SHIFT;
    
    pCommandUPIU->BasicHeader.CmdSetType = UPIU_COMMAND_SET_SCSI;

    /* TODO PAGE SIZE to LOGICAL BLOCK SIZE byte conversion */
    pCommandUPIU->ExpectedDataTxLenBigE = BYTE_SWAP32(UFS_PAGE_SIZE);
    
    /* Construct CDB for READ(6) command */
    
    pCommandUPIU->CDB[0] = 0x08;
    /* Byte 1-3 are used for LBA in Big endian format */
    pCommandUPIU->CDB[3] = (Page & 0xFF);
    pCommandUPIU->CDB[2] = (Page >> 8) & 0xFF;
    pCommandUPIU->CDB[1] = (Page >> 16) & 0x1F;
    /* Fill in transfer length in num of blocks*/
    pCommandUPIU->CDB[4] = 1;
    /* Fill in control = 0x00 */
    pCommandUPIU->CDB[5] = 0;
    /* TODO Fill in PRDT 
     * DW0 Data Base Address Lower bits DWORD aligned.
     * DW1 Data Base Address Upper bits: set to 0. 64 bit Not supported.
     * DW3 Data byte count. 0 based value. Minimum 4 bytes i.e. 3
     */
    pCmdDescriptor->PRDT.DW0 = (NvU32)pBuffer & ~(0x3);
    pCmdDescriptor->PRDT.DW1 = 0;
    pCmdDescriptor->PRDT.DW3 = (UFS_PAGE_SIZE-1);
    
    
    /* TODO Task Tags */
    Error = NvBootUFSCreateTRD(TRDIndex, CmdDescIndex, UFS_TRD_DW0_0_CT_SCSI);
    if(Error!= NvBootError_Success)
        return Error;

    /* TODO Timeout for SCSI READ*/
    Error = NvBootUFSQueueTRD(TRDIndex, SCSI_REQ_READ_TIMEOUT);
    if(Error != NvBootError_Success)
        return Error;
    
    /* TODO Timeout for SCSI READ
     * Timeout is 0 here because we set it part of UFS context earlier. 
     */
    Error = NvBootUFSWaitTRDRequestComplete(TRDIndex, 0);
    if(Error!= NvBootError_Success)
        return Error;
    NvBootUFSFreeTRDCmdDesc();
    
    /* Check Response UPIU */
    if(pUFSContext->UCDGenericRespUPIU.RespUPIU.BasicHeader.Flags & 
        (1 << UFS_UPIU_FLAGS_O_SHIFT | 1 << UFS_UPIU_FLAGS_U_SHIFT | 1<< UFS_UPIU_FLAGS_D_SHIFT))
        return NvBootError_UFSReadError;
    /* TODO Deal with device busy error ? */
    if(pUFSContext->UCDGenericRespUPIU.RespUPIU.BasicHeader.Status != SCSI_STATUS_GOOD)
        return NvBootError_UFSReadError;

    return NvBootError_Success;
}
    
