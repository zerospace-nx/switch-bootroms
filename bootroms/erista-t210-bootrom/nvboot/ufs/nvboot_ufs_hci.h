/** 
 *Insert Nvidia copyright
 */
#include "nvcommon.h"

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
 
/** Create structure for Transfer request descriptor
 * Used for Query Requests, NOP OUT, SCSI command
 */
typedef struct
{
    volatile NvU32 DW0_CmdTypeDD;
    volatile NvU32 DW1_Rsvd;
    volatile NvU32 DW2_CmdStatus;
    volatile NvU32 DW3_Rsvd;
    volatile NvU32 DW4_CDBaseAddr;
    volatile NvU32 DW5_CDBaseAddrU;
    volatile NvU32 DW6_RspOffsetLen;
    volatile NvU32 DW7_PRDTOffsetLen;
    /** 32 bytes**/
} TransferRequestDescriptor_T;

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
#define UFS_TRD_DW2_0_RESERVED_RANGE            31:16
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

#define UFS_TRD_DW0_0_CT_SCSI             0
#define UFS_TRD_DW0_0_CT_UFS              1
#define UFS_TRD_DW0_0_CT_DM               2

#define UFS_TRD_DW0_0_DD_WRITE                      1
#define UFS_TRD_DW0_0_DD_READ                       2


/** END Transfer Request Descriptor defines */

/** UPIU Basic Header **/
typedef struct
{
    NvU8 TransType;
    NvU8 Flags;
    NvU8 LUN;
    NvU8 TaskTag;
    NvU8 CmdSetType;
    NvU8 QueryTMFunction;
    NvU8 Response;
    NvU8 Status;
    NvU8 EHSLength;
    NvU8 DeviceInfo;
    NvU16 DataSegLenBigE;
    /** 12 bytes till here **/
}   UPIUBasicHeader_T;
/*****************************************************************************/
/* BIG LIST OF TODOs */

/** Define ranges for UPIU Fields */
#define UFS_UPIU_FLAGS_O_SHIFT        6
#define UFS_UPIU_FLAGS_U_SHIFT        5
#define UFS_UPIU_FLAGS_D_SHIFT            4
#define UFS_UPIU_FLAGS_R_SHIFT         6
#define UFS_UPIU_FLAGS_W_SHIFT        5
#define UFS_UPIU_FLAGS_A_SHIFT         0
#define UFS_UPIU_FLAGS_A_MASK          0x3
/** Define transaction codes etc for different UPIUs */
#define UPIU_NOP_OUT_TRANSACTION             0x0
#define UPIU_NOP_IN_TRANSACTION              0x20
#define UPIU_COMMAND_TRANSACTION             0x1
#define UPIU_QUERY_REQUEST_TRANSACTION       0x16
/* TODO fill in other transactions */

/** Define codes etc for different types of Query Requests */
#define UPIU_QUERY_TM_FUNC_STD_READ         0x1
#define UPIU_QUERY_TM_FUNC_STD_WRITE        0x81
/** Define codes etc for different types of Command Requests */
#define UPIU_COMMAND_READ                   0x1
#define UPIU_COMMAND_WRITE                  0x81

/** Define codes etc for different families of Command*/
#define UPIU_COMMAND_SET_SCSI               0x0

/** All CDB defintions from SCSI block and primary commands */
#define SCSI_READ6_OPCODE                   0x8

#define SCSI_STATUS_GOOD                    0x0

/** Define codes etc for different types of Query READ/WRITE Requests */
#define QUERY_REQ_STD_READ_READ_DESC        0x1
#define QUERY_REQ_STD_READ_READ_ATTRB       0x3
#define QUERY_REQ_STD_READ_READ_FLAG        0x5
#define QUERY_REQ_STD_WRITE_SET_FLAG        0x6

/* TODO fill in other query reqs */
/** Define codes etc for different Attribute IDNs */
#define BOOT_LUN_EN_ATTRB                   0x0
#define ACTIVE_ICC_LVL_ATTRB                0x3

/** Define codes etc for different Flag IDNs */
#define DEVICE_INIT_IDN                     0x1

/** Define codes etc for different Descriptor IDNs */
#define DEVICE_DESC_IDN                     0x0
#define CONF_DESC_IDN                       0x1
#define UNIT_DESC_IDN                       0x2

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
    NvU8    bLength;
    NvU8    bDescriptorType;
    NvU8    bDevice;
    NvU8    bDeviceClass;
    NvU8    bDeviceSubClass;
    NvU8    bProtocol;
    NvU8    bNumberLU;
    NvU8    bNumberWLU;
    NvU8    bBootEnable;
    NvU8    bDescAccessEn;
    NvU8    bInitPowerMode;
    NvU8    bHighPriorityLUN;
    NvU8    bSecureRemovalType;
    NvU8    bSecurityLU;
    NvU8    Reserved;
    NvU8    bInitActiveICCLevel;
    NvU16   wSpecVersion;
    NvU16   wManufactureDate;
    NvU8    iManufacturerName;
    NvU8    iProductName;
    NvU8    iSerialNumber;
    NvU8    iOemID;
    NvU16   wManufacturerID;
    NvU8    bUD0BaseOffset;
    NvU8    bUDConfigPLength;   
    NvU8    bDeviceRTTCap;
    NvU16   wPeriodicRTCUpdate;
}   UFSDeviceDesc_T;
typedef struct
{
    NvU8    bLength;
    NvU8    bDescType;
    NvU8    bUnitIndex;
    NvU8    bLUEnable;
    NvU8    bBootLUNID;
    NvU8    bLUWriteProtect;
    NvU8    bLUQueueDepth;
    NvU8    bReserved;
    NvU8    bMemoryType;
    NvU8    bDataReliability;
    NvU8    bLogicalBlockSize;
    NvU64   qLogicalBlockCount;
    NvU32   dEraseBlockSize;
    NvU8    bProvisionType;
    NvU64   qPhyMemResourceCount;
    NvU16   wContextCapabilities;
    NvU8    bLargeUnitSizeM1;
}   UFSUnitDesc_T;


/*****************************************************************************/

/** Command UPIU
 * Used for SCSI Commands.
 * 32 bytes
 */
typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU32 ExpectedDataTxLenBigE;
    NvU8 CDB[16];
    /** 32 bytes till here **/
    //NvU32 E2ECRC;   // Rev1P1 of UFS Spec says E2ECRC not supported.
    //NvU8 Padding[4];    // Response UPIU should be at 64 bit boundary
}   CommandUPIU_T;

/** Response UPIU 
 */
typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU32 ResidualTxCountBigE;
    NvU8 Reserved[16];
    /** 32 bytes till here **/
    //NvU32 E2ECRC;   // Rev1P1 of UFS Spec says E2ECRC not supported.
    NvU8 SenseData[16];
}   ResponseUPIU_T;
// TODO define sense data

/** NOP_OUT UPIU
 */
typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU8 Reserved[20];
    /** 32 bytes till here **/
} NOP_UPIU_T;


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
    NvU8    FlagIDN;
    NvU8    Index;
    NvU8    Selector;
    NvU8    Reserved1[7];
    NvU8    FlagValue;
    NvU32   Reserved2;
}   FlagFields_T;

typedef struct
{
    NvU8    AttrbIDN;
    NvU8    Index;
    NvU8    Selector;
    NvU8    Reserved1[4];
    NvU32   ValueBigE;
    NvU32   Reserved2;
}   AttrbFields_T;

typedef struct
{
    NvU8    DescIDN;
    NvU8    Index;
    NvU8    Selector;
    NvU8    Reserved1[2];
    NvU16   LengthBigE;
    NvU32   Reserved2[2];
}   DescFields_T;

typedef union
{
    DescFields_T DescFields;
    AttrbFields_T AttrbFields;
    FlagFields_T FlagFields;
}   TSF_T;

typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU8 Opcode;
    TSF_T  TSF;
    NvU32 Reserved;
    /** 32 bytes till here **/
    NvU8 DataSegment[256];
    /** 288 bytes till here **/
}   QueryReqRespUPIU_T;

typedef struct
{
    UPIUBasicHeader_T BasicHeader;
    NvU32 InputParam1;
    NvU32 InputParam2;
    NvU32 InputParam3;
    NvU32 Reserved1;
    NvU32 Reserved2;
    /** 32 bytes till here **/
}   TaskMgmtDescriptorUPIU_T;

/** Define PRDT */
typedef struct
{
    NvU32 DW0;
    NvU32 DW1;
    NvU32 DW2;
    NvU32 DW3;
}   PRDT_T;
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
typedef struct
{
    UCDGenericReqUPIU_T UCDGenericReqUPIU;
    /** Response UPIU has to be aligned to 64 bit boundary **/
    UCDGenericRespUPIU_T UCDGenericRespUPIU;
    PRDT_T PRDT;
    NvU32 Padding[48];
    /** 640 bytes with padding **/
}   CmdDescriptor_T;
/** Data OUT UPIU and DATA IN UPIU handled by Host Controller.
 */
#endif
