/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvtypes.h"

#ifndef UFS_HCI_H
#define UFS_HCI_H

/** Create structure for Transfer Request descriptor
 */
typedef struct
{
    /* TODO */
    NvU32 todo1;
}   TaskMgmtUPIU_T;
typedef struct
{
    volatile NvU32 DW0;
    volatile NvU32 DW1;
    volatile NvU32 DW2;
    volatile NvU32 DW3;
    TaskMgmtUPIU_T Request;
    TaskMgmtUPIU_T Response;
    /** 80 bytes till here **/
} TaskMgmtRequestDescriptor_T;

#define DATA_DIR_NIL  0x0
#define DATA_DIR_H2D  0x1
#define DATA_DIR_D2H  0x2

#define OCS_SUCCESS 0x0

/** Create structure for Transfer request descriptor
 * Used for Query Requests, NOP OUT, SCSI command
 */
// typedef struct
// {
    // volatile NvU32 DW0_CmdTypeDD;
    // volatile NvU32 DW1_Rsvd;
    // volatile NvU32 DW2_CmdStatus;
    // volatile NvU32 DW3_Rsvd;
    // volatile NvU32 DW4_CDBaseAddr;
    // volatile NvU32 DW5_CDBaseAddrU;
    // volatile NvU32 DW6_RspOffsetLen;
    // volatile NvU32 DW7_PRDTOffsetLen;
    // /** 32 bytes**/
// } TransferRequestDescriptor_T;
typedef struct
{
    /* DW0 */
    union
    {
        struct
        {
            NvU32 Rsvd0:24;
            NvU32 Interrupt:1;
            NvU32 DD:2;
            NvU32 Rsvd1:1;
            NvU32 CT:4;
        };
        NvU32 DW;
    }   DW0;
    /* DW1 */
    union
    {
        NvU32 Reserved;
        NvU32 DW;
    } DW1;
    /* DW2 */
    union
    {
        struct
        {
            NvU32 OCS:8;
            NvU32 Rsvd0:24;
        };
        NvU32 DW;
    }   DW2;
    /* DW3 */
    union
    {
        NvU32 Reserved;
        NvU32 DW;
    } DW3;
    /* DW4 */
    union
    {
        struct
        {
            NvU32 Rsvd0:7;
            NvU32 CTBA:25;
        };
        NvU32 DW;
    }   DW4;
    /* DW5 */
    union
    {
        NvU32 CTBAU;
        NvU32 DW;
    }   DW5;
    /* DW6 */
    union
    {
        struct
        {
            NvU32 RUL:16;
            NvU32 RUO:16;
        };
        NvU32 DW;
    }   DW6;
    /* DW7 */
    union
    {
        struct
        {
            NvU32 PRDTL:16;
            NvU32 PRDTO:16;
        };
        NvU32 DW;
    }   DW7;
}   TransferRequestDescriptor_T;


/** Transfer Request Descriptor defines for use with NV DRF macros
 */
/** DW0 **/
#define UFS_TRD_DW0_0_CT_RANGE        31:28
#define UFS_TRD_DW0_0_DD_RANGE      26:25
#define UFS_TRD_DW0_0_I_RANGE           24:24
#define UFS_TRD_DW0_0_RESERVED_RANGE            23:0

/** DW1 **/
#define UFS_TRD_DW1_0_RESERVED_RANGE            31:0

/** DW2 **/
#define UFS_TRD_DW2_0_RESERVED_RANGE            31:8
#define UFS_TRD_DW2_0_OCS_RANGE                 7:0

/** DW3 **/
#define UFS_TRD_DW3_0_RESERVED_RANGE            31:0

/** DW4 **/
#define UFS_TRD_DW4_0_UCDBA_RANGE                31:7
#define UFS_TRD_DW4_0_RESERVED_RANGE             6:0

/** DW5 **/
#define UFS_TRD_DW5_0_UCDBAU_RANGE               31:0

/** DW6 **/
#define UFS_TRD_DW6_0_RUO_RANGE                 31:16
#define UFS_TRD_DW6_0_RUL_RANGE                 15:00

/** DW7 **/
#define UFS_TRD_DW7_0_PRDTO_RANGE                 31:16
#define UFS_TRD_DW7_0_PRDTL_RANGE                 15:0

/****************************************************/
#define UFS_TRD_DW0_0_CT_UFS              1

#define UFS_TRD_DW0_0_DD_WRITE                      1
#define UFS_TRD_DW0_0_DD_READ                       2


/** END Transfer Request Descriptor defines */

/** Structure to hold DME commands. This is not directly written into
 * UICCMD registers. 
 */
typedef struct DMECommand
{
    // This is C11 but compiles without warning with std=99
    union
    {
        struct
        {
            NvU32   CmdOp:8;
            NvU32   Rsvd0DW0:24;
        };
        NvU32   DW;
    } UICCmd;
    union
    {
        struct
        {
            /* Begin Arg 1. Reset level is a special case */
            NvU32   GenSelectorIndex:16;
            NvU32   MIBAttribute:16;
        };
        NvU32 DW;
    } UICCmdArg1;
    union
    {
        struct
        {
            /* Begin Arg 2 */
            NvU32   ConfigErrorCode:8;
            NvU32   Rsvd0DW2:8;
            NvU32   AttrSetType:8;
            NvU32   Rsvd1DW2:8;
        };
        NvU32 DW;
    } UICCmdArg2;
    /* Begin Arg 3 */
    NvU32   ReadWriteValue;
} DMECommand_T;
/** End structure to hold DME commands. */

/** UIC CMD Opcodes */
#define DME_GET 0x1
#define DME_SET 0x2
#define DME_PEER_GET 0x3
#define DME_PEER_SET 0x4
#define DME_ENABLE 0x12
#define DME_RESET 0x14
#define DME_LINKSTARTUP 0x16
/** End UIC CMD Opcodes */

/** UIC MIB Attributes */
#define PA_AvailTxDataLanes     0x1520
#define PA_AvailRxDataLanes     0x1540
#define PA_ActiveTxDataLanes    0x1560
#define PA_ConnectedTxDataLanes 0x1561
#define PA_TxGear               0x1568
#define PA_TxTermination        0x1569
#define PA_HSSeries             0x156A
#define PA_ActiveRxDataLanes    0x1580
#define PA_ConnectedRxDataLanes 0x1581
#define PA_RxGear               0x1583
#define PA_RxTermination        0x1584
#define PA_TxHsG1PrepareLength      0x1553
#define PA_TxHsG2PrepareLength      0x1555
#define PA_TxHsG3PrepareLength      0x1557

#define PA_TxHsG1SyncLength         0x1552
#define PA_TxHsG2SyncLength         0x1554
#define PA_TxHsG3SyncLength         0x1556

#define PA_Local_TX_LCC_Enable      0x155e
#define PA_Peer_TX_LCC_Enable       0x155f
#define PA_TxTrailingClocks         0x1564
#define PA_PWRMode                  0x1571
#define PA_SleepNoConfigTime        0x15a2
#define PA_StallNoConfigTime        0x15a3
#define PA_SaveConfigTime           0x15a4

#define PA_Hibern8Time              0x15a7
#define PA_TActivate                0x15a8
#define PA_Granularity              0x15aa

#define PWRModeUserData0            0x15B0
#define PWRModeUserData1            0x15B1
#define PWRModeUserData2            0x15B2

#define T_CPortFlags                0x4025
#define T_ConnectionState           0x4020

#define DME_LayerEnable             0xD000
#define DME_LinkStartup             0xD020
#define VS_TxBurstClosureDelay      0xD084

#define DME_FC0ProtectionTimeOutVal 0xD041
#define DME_TC0ReplayTimeOutVal     0xD042
#define DME_AFC0ReqTimeOutVal       0xD043

/** Unipro powerchange mode.
 * SLOW : PWM
 * SLOW_AUTO : PWM (but does auto burst closure for power saving)
 */
#define PWRMODE_SLOW_MODE    0x2
#define PWRMODE_SLOWAUTO_MODE    0x5


/*****************************************************************************/
/* BIG LIST OF TODOs */

/** Define ranges for UPIU Fields */
#define UFS_UPIU_FLAGS_O_SHIFT        6
#define UFS_UPIU_FLAGS_U_SHIFT        5
#define UFS_UPIU_FLAGS_D_SHIFT        4
#define UFS_UPIU_FLAGS_R_SHIFT        6
#define UFS_UPIU_FLAGS_W_SHIFT        5
#define UFS_UPIU_FLAGS_A_SHIFT        0
#define UFS_UPIU_FLAGS_A_MASK         0x3
/** Define transaction codes etc for different UPIUs */
#define UPIU_NOP_OUT_TRANSACTION             0x0
#define UPIU_NOP_IN_TRANSACTION              0x20
#define UPIU_COMMAND_TRANSACTION             0x1
#define UPIU_QUERY_REQUEST_TRANSACTION       0x16
/* TODO fill in other transactions */

/** Define codes etc for different types of Query Requests */
#define UPIU_QUERY_FUNC_STD_READ         0x1
#define UPIU_QUERY_FUNC_STD_WRITE        0x81
/** Define codes etc for different types of Command Requests */
#define UPIU_COMMAND_READ                   0x1
#define UPIU_COMMAND_WRITE                  0x81

/** Define codes etc for different families of Command*/
#define UPIU_COMMAND_SET_SCSI               0x0

/** All CDB defintions from SCSI block and primary commands */
#define SCSI_READ6_OPCODE                   0x8

#define SCSI_STATUS_GOOD                    0x0
#define SCSI_STATUS_CHECK_CONDITION         0x2
#define SCSI_STATUS_BUSY                    0x8

/** Define codes etc for different types of Query READ/WRITE Requests */
#define TSF_OPCODE_READ_DESC        0x1
#define TSF_OPCODE_WRITE_DESC       0x2
#define TSF_OPCODE_READ_ATTRB       0x3
#define TSF_OPCODE_READ_FLAG        0x5
#define TSF_OPCODE_SET_FLAG        0x6

/* TODO fill in other query reqs */
/** Define codes etc for different Attribute IDNs */
#define QUERY_ATTRB_BOOT_LUN_EN                   0x0
#define QUERY_ATTRB_CURR_POWER_MODE               0x2
#define QUERY_ATTRB_ACTIVE_ICC_LVL                0x3

/** Define codes etc for different Flag IDNs */
#define QUERY_FLAG_DEVICE_INIT_IDN                0x1

/** Define codes etc for different Descriptor IDNs */
#define QUERY_DESC_DEVICE_DESC_IDN                     0x0
#define QUERY_DESC_CONF_DESC_IDN                       0x1
#define QUERY_DESC_UNIT_DESC_IDN                       0x2

/* TODO fill in other descriptor ids */

/** Define offsets etc for different Descriptors  */
#define UFS_DESC_MAX_SIZE 255
/** DEVICE DESCRIPTOR */
#define UFS_DEV_DESC_LENGTH                 0x0
#define UFS_DEV_DESC_DESC_TYPE              0x1
#define UFS_DEV_DESC_DEVICE                 0x2
#define UFS_DEV_DESC_DEVICE_CLASS           0x3
#define UFS_DEV_DESC_DEVICE_SUB_CLASS       0x4
#define UFS_DEV_DESC_PROTOCOL               0x5
#define UFS_DEV_DESC_NUM_LUN                0x6
#define UFS_DEV_DESC_NUM_WLUN               0x7
#define UFS_DEV_DESC_BOOT_ENABLE            0x8
#define UFS_DEV_DESC_DESC_ACCESS_ENABLE     0x9
#define UFS_DEV_DESC_INIT_POWER_MODE        0xA
#define UFS_DEV_DESC_HIGH_PRIORITY_LUN      0xB
#define UFS_DEV_DESC_SECURE_REMOVAL_TYPE    0xC
#define UFS_DEV_DESC_SECURITY_LU            0xD
#define UFS_DEV_DESC_RESERVED               0xE
#define UFS_DEV_DESC_UD0_BASE_OFFSET        0x1A
#define UFS_DEV_DESC_UD_CONFIG_PLENGTH      0x1B

/** UNIT DESCRIPTOR */
#define UFS_UNIT_DESC_LENGTH                0x0
#define UFS_UNIT_DESC_DESC_TYPE             0x1
#define UFS_UNIT_DESC_UNIT_INDEX            0x2
#define UFS_UNIT_DESC_LU_ENABLE             0x3
#define UFS_UNIT_DESC_BOOT_LUN_ID           0x4
#define UFS_UNIT_DESC_LU_WRITE_PROTECT      0x5
#define UFS_UNIT_DESC_LU_QUEUE_DEPTH        0x6
#define UFS_UNIT_DESC_RESERVED              0x7
#define UFS_UNIT_DESC_MEMORY_TYPE           0x8
#define UFS_UNIT_DESC_DATA_RELIABILITY      0x9
#define UFS_UNIT_DESC_LOGICAL_BLOCK_SIZE    0xA
#define UFS_UNIT_DESC_QLOGICAL_BLOCK_COUNT  0xB
#define UFS_UNIT_DESC_ERASE_BLOCK_SIZE      0x13
#define UFS_UNIT_DESC_PROVISIONING_TYPE     0x17
#define UFS_UNIT_DESC_QPHY_MEM_RSRC_COUNT   0x18

typedef struct
{
    uint8_t    bLength;
    uint8_t    bDescriptorType;
    uint8_t    bDevice;
    uint8_t    bDeviceClass;
    uint8_t    bDeviceSubClass;
    uint8_t    bProtocol;
    uint8_t    bNumberLU;
    uint8_t    bNumberWLU;
    uint8_t    bBootEnable;
    uint8_t    bDescAccessEn;
    uint8_t    bInitPowerMode;
    uint8_t    bHighPriorityLUN;
    uint8_t    bSecureRemovalType;
    uint8_t    bSecurityLU;
    uint8_t    Reserved;
    uint8_t    bInitActiveICCLevel;
    uint16_t   wSpecVersion;
    uint16_t   wManufactureDate;
    uint8_t    iManufacturerName;
    uint8_t    iProductName;
    uint8_t    iSerialNumber;
    uint8_t    iOemID;
    uint16_t   wManufacturerID;
    uint8_t    bUD0BaseOffset;
    uint8_t    bUDConfigPLength;   
    uint8_t    bDeviceRTTCap;
    uint16_t   wPeriodicRTCUpdate;
}   __attribute__((packed)) UFSDeviceDesc_T;
typedef struct
{
    uint8_t    bLength;
    uint8_t    bDescType;
    uint8_t    bUnitIndex;
    uint8_t    bLUEnable;
    uint8_t    bBootLUNID;
    uint8_t    bLUWriteProtect;
    uint8_t    bLUQueueDepth;
    uint8_t    bReserved;
    uint8_t    bMemoryType;
    uint8_t    bDataReliability;
    uint8_t    bLogicalBlockSize;
    NvU64   qLogicalBlockCount;
    NvU32   dEraseBlockSize;
    uint8_t    bProvisionType;
    NvU64   qPhyMemResourceCount;
    uint16_t   wContextCapabilities;
    uint8_t    bLargeUnitSizeM1;
}   __attribute__((packed)) UFSUnitDesc_T;


/*****************************************************************************/
/** UPIU Basic Header **/
typedef struct
{
    uint8_t TransCode;// Fields HD, DD, T are all 0. (No header CRC, no Data CRC,
                   // T=0 implies request from host.)
    uint8_t Flags;
    uint8_t LUN;
    uint8_t TaskTag;
    uint8_t CmdSetType;
    uint8_t QueryTMFunction;
    uint8_t Response;
    uint8_t Status;
    uint8_t EHSLength;
    uint8_t DeviceInfo;
    uint16_t DataSegLenBigE;
    /** 12 bytes till here **/
}   __attribute__((packed)) UPIUBasicHeader_T;
/** Command UPIU
 * Used for SCSI Commands.
 * 32 bytes
 */
typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    /** 12 bytes till here */
    NvU32 ExpectedDataTxLenBigE;
    uint8_t CDB[16];
    /** 32 bytes till here **/
    /**** Data Segment begins here. If Data phase collapse is supported, 
    add here. ****/
    //NvU32 E2ECRC;   // Rev1P1 of UFS Spec says E2ECRC not supported.
    //uint8_t Padding[4];    // Response UPIU should be at 64 bit boundary
}   __attribute__((packed)) CommandUPIU_T;

/** Response UPIU 
 */
typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU32 ResidualTxCountBigE;
    uint8_t Reserved[16];
    /** 32 bytes till here **/
    //NvU32 E2ECRC;   // Rev1P1 of UFS Spec says E2ECRC not supported.
    /**** Data Segment begins here ****/
    uint16_t SenseDataLength;
    uint8_t SenseData[18];
}   __attribute__((packed)) ResponseUPIU_T;
// TODO define sense data

/** NOP_OUT UPIU
 */
typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    uint8_t Reserved[20];
    /** 32 bytes till here **/
} __attribute__((packed)) NOP_UPIU_T;


/** Query Request/Response UPIU
 * Used primarily for transferring descriptors between host and device.
 * Max descriptor size is 255 bytes.
 * Keeping data segment size as 256 (255 data + 1 byte padding) so that
 * Response UPIU in TRD will be aligned to 64 bit boundary.
 *                    TRD
 *             | QUERY REQ UPIU  |-> 32 bytes UPIU Header + 255 bytes data + 1 byte padding.
 *             | QUERY RESP UPIU |-> 32 bytes UPIU Header + 255 bytes data + 1 byte padding.
 *             |    NO PRDT      |-> 0 bytes. All data in data segment.
 */

/** Use dozen unions for different types of query requests
*/
typedef struct
{
    uint8_t    FlagIDN;
    uint8_t    Index;
    uint8_t    Selector;
    uint8_t    Reserved1[7];
    uint8_t    FlagValue;
    NvU32   Reserved2;
}   __attribute__((packed)) FlagFields_T;

typedef struct
{
    uint8_t    AttrbIDN;
    uint8_t    Index;
    uint8_t    Selector;
    uint8_t    Reserved1[4];
    NvU32   ValueBigE;
    NvU32   Reserved2;
}   __attribute__((packed)) AttrbFields_T;

typedef struct
{
    uint8_t    DescIDN;
    uint8_t    Index;
    uint8_t    Selector;
    uint8_t    Reserved1[2];
    uint16_t   LengthBigE;
    NvU32   Reserved2[2];
}   __attribute__((packed)) DescFields_T;

typedef struct
{
    uint8_t Opcode;
    union
    {
        DescFields_T DescFields;
        AttrbFields_T AttrbFields;
        FlagFields_T FlagFields;
    };
}   __attribute__((packed)) TSF_T;

typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    TSF_T  TSF;
    NvU32 Reserved;
    /** 32 bytes till here **/
    /** choosing data segment size of 256 as descriptor is maximum 255 bytes.
    and that is probably the largest chunk of data we should receiving.**/
    uint8_t DataSegment[256];
    /** 288 bytes till here **/
}   __attribute__((packed)) QueryReqRespUPIU_T;

typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU32 InputParam1;
    NvU32 InputParam2;
    NvU32 InputParam3;
    NvU32 Reserved1;
    NvU32 Reserved2;
    /** 32 bytes till here **/
}   __attribute__((packed)) TaskMgmtDescriptorUPIU_T;

/** Define PRDT */
typedef struct
{
    NvU32 DW0;
    NvU32 DW1;
    NvU32 DW2;
    NvU32 DW3;
}   __attribute__((packed)) PRDT_T;
/* PRDT_DW0 */
#define PRDT_DW0_0_RESERVED_RANGE   1:0
#define PRDT_DW0_0_DBA_RANGE        31:2
/* PRDT_DW3 */
#define PRDT_DW3_0_DBA_RANGE        17:2
#define PRDT_DW3_0_RESERVED_RANGE   31:18
/** End PRDT defines and structures **/

/* Define union of Command UPIU, QUERY UPIU and NOP UPIU */

typedef union
{
    NOP_UPIU_T NOP_UPIU;
    CommandUPIU_T CmdUPIU;
    QueryReqRespUPIU_T QueryReqRespUPIU;
    /** 288 bytes **/
}   UCDGenericReqUPIU_T;

/* Define generic response */
typedef union
{
    NOP_UPIU_T NOP_UPIU;
    ResponseUPIU_T RespUPIU;
    QueryReqRespUPIU_T QueryReqRespUPIU;
    /** 288 bytes **/
}   UCDGenericRespUPIU_T;

/** UFS Command Descriptor
 *  Structure is as follows:
 *     --------------------
 *    |  UCD Generic UPIU  | NOP/Query/SCI Cmd UPIU, Max 288 bytes (QueryUPIU) 
 *     --------------------
 *    |  Response UPIU     | NOP/Query/SCI RSP Aligned to 64 bit boundary,
 *     --------------------  288 bytes
 *    |  PRDT (if required)| Needed for SCSI Cmd UPIUs
 *     --------------------
 * Structure itself should be aligned to 128 byte aligned
 * so pad accordingly to form array.
 * TODO implement MACRO if possible to complain if alignment not met
 */
#define CMD_DESC_REQ_LENGTH 512
#define CMD_DESC_RESP_LENGTH 512
#define CMD_DESC_PRDT_LENGTH 512
typedef struct
{
    union
    {
    UCDGenericReqUPIU_T UCDGenericReqUPIU; /* 288 bytes */
    uint8_t    CmdDescReq[CMD_DESC_REQ_LENGTH];
    };
    //uint8_t Paddding[RequestPadding];
    /** Response UPIU has to be aligned to 64 bit boundary **/
    union
    {
    UCDGenericRespUPIU_T UCDGenericRespUPIU; /* 288 bytes */
    uint8_t    CmdDescResp[CMD_DESC_RESP_LENGTH];
    };
    union
    {
    PRDT_T PRDT;
    uint8_t    CmdDescPRDT[CMD_DESC_PRDT_LENGTH];
    };
    // NvU32 Padding[48];
    /** 640 bytes with padding **/
}   __attribute__((packed)) CmdDescriptor_T;
/** Data OUT UPIU and DATA IN UPIU handled by Host Controller.
 */

/**
 * Defines the status from UFS
 * Overlay this structure with space reserved for this purpose in BIT
 * SecondaryDevStatus[NVBOOT_SIZE_DEV_STATUS]
 */
typedef struct NvBootUfsStatusRec
{
    ///
    /// Parameters specified by fuses or straps.
    ///

    ///
    /// Parameters discovered during operation
    ///

    ///
    /// Parameters provided by the device
    ///

    ///
    /// Information for driver validation
    ///

} NvBootUfsStatus;


NvU32 NvBootUfsDMELinkSetup(void);
void NvBootUfsSetupTRDTMLists(void);
NvU32 NvBootUfsStartTMTREngines(void);
NvU32 NvBootUfsChkIfDevRdy2RcvDesc(void);
NvU32 NvBootUfsGetDevInfo(void);
NvU32 NvBootUfsGetDevInfoPartialInit(void);
void NvBootUfsFreeTRDCmdDesc(void);
NvU32 NvBootUfsGetTRDSlot(void);
NvU32 NvBootUfsCompleteInit(void);
NvU32 NvBootUfsGetAttribute(NvU32*, NvU32, NvU32);
NvU32 NvBootUfsGetDescriptor(uint8_t*, NvU32, NvU32);
NvU32 NvBootUfsIsRespNOPIN(NvU32 CmdDescIndex);
NvU32 NvBootSetDMECommand(uint8_t CmdOp, uint16_t GenSelIdx, uint16_t MIBAttr, NvU32* Data);
NvBootError NvBootUfsClockEnable();
NvBootError NvBootUfsResetDisable();
NvU32 NvBootUfsCreateTRD(NvU32 TRDIndex, NvU32 CmdDescIndex, NvU32 DataDir);
NvU32 NvBootUfsUphyClkEnableResetDisable();
NvBootError NvBootUfsTestUnitReady(NvU32 LUN);
NvBootError NvBootUfsRequestSense(NvU32 LUN);
NvBootError NvBootUfsSetTimerThreshold(NvU32 Gear, NvU32 ActiveLanes);
NvU32 NvBootUfsLinkUphyPllSetup(NvU32 Pll);
NvU32 NvBootUfsLinkUphyLaneSetup(NvU32 Lane);
NvU32 NvBootUfsLinkUphyPllParamsSetup(NvU32 Pll);
void NvBootUphyLaneIddqClampRelease(NvU32 Lane);
NvBootError NvBootUfsSetActivateTime();
#endif
