/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * NvBootInfoTable (BIT) provides information from the Boot ROM (BR)
 * to Bootloaders (BLs).
 *
 * Initially, the BIT is cleared.
 *
 * As the BR works its way through its boot sequence, it records data in
 * the BIT.  This includes information determined as it boots and a log
 * of the boot process.  The BIT also contains a pointer to an in-memory,
 * plaintext copy of the Boot Configuration Table (BCT).
 *
 * The BIT allows BLs to determine how they were loaded, any errors that
 * occured along the way, and which of the set of BLs was finally loaded.
 *
 * The BIT also serves as a tool for diagnosing boot failures.  The cold boot
 * process is necessarily opaque, and the BIT provides a window into what
 * actually happened.  If the device is completely unable to boot, the BR
 * will enter Recovery Mode (RCM), using which one can load an applet that
 * dumps the BIT and BCT contents for analysis on the host.
 */

#ifndef INCLUDED_NVBOOT_BIT_H
#define INCLUDED_NVBOOT_BIT_H

#include "nvcommon.h"
#include "nvboot_bct.h"
#include "nvboot_config.h"
#include "nvboot_osc.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/// Specifies the amount of status data needed in the BIT.
/// One bit of status is used for each of the journal blocks,
/// and 1 additional bit is needed for the second BCT in block 0.
#define NVBOOT_BCT_STATUS_BITS  (NVBOOT_MAX_BCT_SEARCH_BLOCKS + 1)
#define NVBOOT_BCT_STATUS_BYTES ((NVBOOT_BCT_STATUS_BITS + 7) >> 3)

/**
 * Defines the type of boot.
 * Note: There is no BIT for warm boot.
 */
typedef enum
{
    /// Specifies a default (unset) value.
    NvBootType_None = 0,

    /// Specifies a cold boot
    NvBootType_Cold,

    /// Specifies the BR entered RCM
    NvBootType_Recovery,

    /// Specifies UART boot (only available internal to NVIDIA)
    NvBootType_Uart,

    /// Specifies that the BR immediately exited for debugging
    /// purposes.
    /// This can only occur when NOT in ODM production mode,
    /// and when a special BOOT_SELECT value is set.
    NvBootType_ExitRcm,

    NvBootType_Force32 = 0x7fffffff
} NvBootType;

/**
 * Defines the status codes for attempting to load a BL.
 */
typedef enum
{
    /// Specifies a default (unset) value.
    NvBootRdrStatus_None = 0,

    /// Specifies a successful load.
    NvBootRdrStatus_Success,

    /// Specifies validation failure.
    NvBootRdrStatus_ValidationFailure,

    /// Specifies a read error.
    NvBootRdrStatus_DeviceReadError,

    NvBootRdrStatus_Force32 = 0x7fffffff
} NvBootRdrStatus;

/**
 * Defines the information recorded about each BL.
 *
 * BLs that do not become the primary copy to load have status of None.
 * They may still experience Ecc errors if used to recover from ECC
 * errors of another copy of the BL.
 */
typedef struct NvBootBlStateRec
{
    /// Specifies the outcome of attempting to load this BL.
    NvBootRdrStatus Status;

    /// Specifies the first block that experienced an ECC error, if any.
    /// 0 otherwise.
    NvU32           FirstEccBlock;

    /// Specifies the first page that experienced an ECC error, if any.
    /// 0 otherwise.
    NvU32           FirstEccPage;

    /// Specifies the first block that experienced a correctable ECC error,
    /// if any. 0 otherwise. Only correctable errors that push the limits of
    /// the ECC algorithm are recorded (i.e., those very likely to become
    /// uncorrectable errors in the near future).
    NvU32           FirstCorrectedEccBlock;

    /// Specifies the first page that experienced a correctable ECC error,
    /// if any. 0 otherwise. Similar to FirstCorrectedEccBlock.
    NvU32           FirstCorrectedEccPage;

    /// Specifies if the BL experienced any ECC errors.
    NvBool          HadEccError;

    /// Specifies if the BL experienced any CRC errors.
    NvBool          HadCrcError;

    /// Specifies if the BL experienced any corrected ECC errors.
    /// As with FirstCorrectedEcc*, only nearly-uncorrectable errors count.
    NvBool          HadCorrectedEccError;

    /// Specifies if the BL provided data for another BL that experienced an
    /// ECC error.
    NvBool          UsedForEccRecovery;
} NvBootBlState;

/**
 * Defines the status from Usb3
 */
typedef struct NvBootUsb3StatusRec
{
    ///
    /// Parameters specified by fuses or straps.
    ///
    NvU8                PortNum;
    ///
    /// Parameters provided by the device
    ///
    //Sense key as returned by mode sense command
    NvU8                SenseKey;
    //CSW status
    NvU32              CurrCSWTag;
    NvU32              CurrCmdCSWStatus;
    NvU32              CurrEpBytesNotTransferred;
    //Inquiry response
    NvU32               PeripheralDevTyp;
    //Read Fmt capacity
    NvU32               NumBlocks;
    //Read Capacity response
    NvU32               LastLogicalBlkAddr;
    NvU32               BlockLenInByte;
    //Pointer to NvBootUsb3Context
    NvU32               Usb3Context;

    NvU32               InitReturnVal;
    NvU32               ReadPageReturnVal;
    NvU32               XusbDriverStatus;
    // Device status.
    NvU32               DeviceStatus;
    // Endpoint status
    NvU32               EpStatus;

   } NvBootUsb3Status;


/**
 * Defines the status from eMMC and eSD
 */
typedef struct NvBootSdmmcStatusRec
{
    ///
    /// Parameters specified by fuses or straps.
    ///

    /// Specifies the clock source.
    NvU8 ModuleClkSource;

    /// Specifies the clk divisor.
    NvU8 ModuleClkDivisor;

    /// Specifies the clk enable.
    NvU8 ModuleClkEnable;

    /// Specifies the module reset status.
    NvU8 ModuleClkRstStatus;

    /// Specifies the clk internal divisor for interface via fuses or straps.
    NvU8 SdmmcIntrlClkDivisor;

    /// Specifies the data transfer mode.
    NvU8 SdmmcDataMode;

    /// Specifies the data width specified by fuses or straps.
    NvU8 FuseDataWidth;

    /// Specifies the Ddr mode specified by fuses or straps.
    NvBool FuseDdrMode;

    /// Specifies the Config option specified by fuses or straps.
    NvU8 FuseConfig;

    /// Specifies the Read multi/Single page option from config option.
    NvBool FuseReadMode;

    ///
    /// Parameters discovered during operation
    ///

    /// Specifies the discovered card type
    NvU8 DiscoveredCardType;

    /// Specifies the discovered voltage range
    NvU32 DiscoveredVoltageRange;

    /// Specifies the data width chosen to conform to power class constraints
    NvU8 DataWidthUnderUse;

    /// Specifies the power class chosen to conform to power class constraints
    NvU8 PowerClassUnderUse;

    /// Specifies the Auto Cal status
    NvBool AutoCalStatus;

    ///
    /// Parameters provided by the device
    ///

    /// Specifies the card identification data.
    NvU32 Cid[4];

    ///
    /// Information for driver validation
    ///

    /// Specifies the number of pages read from the beginning.
    NvU32 NumPagesRead;

    /// Specifies the # of CRC errors
    NvU32 NumCrcErrors;

    /// Specifies whether the boot was attempted from a Boot Partition.
    NvU8 BootFromBootPartition;

    /// Overall sdmmc hw init and media enumeration time
    NvU32 SdmmcInit;
    
    /// Controller, clock/resets & pad initialization.
    NvU32 SdmmcControllerInit;
    
    /// 	eMMC device Identification/Transfer state transition time
    NvU32 eMMCDeviceEnumeration;
    
    /// Get Operating condition command 1 time, device power up
    NvU32 eMMCCmd1;
    
    /// Retrieve CID from attached device via Cmd2
    NvU32 eMMC_ALL_SEND_CID;
    
    /// Retrieve Card Specific Data via SEND_CSD(CMD9) Command.
    NvU32 eMMC_CSD;
    
    /// Place attached card into transfer mode from identification mode 
    NvU32 eMMC_transfer_mode;
    
    /// Obtain Extended Card specific data
    NvU32 eMMC_ExtCsd;
    
    /// PowerClass settings based on BCT.
    NvU32 eMMC_PowerClass;
    
    /// Select Bus width and partition setup time.
    NvU32 eMMC_WidthPartitionSetup;
    
    ///Data read time for payload (data throughput)
    NvU32 ReadTime;
    ///Associated Data length in bytes for ReadTime
    NvU32 Payload;
    
} NvBootSdmmcStatus;
/**
 * Defines the status from SPI flash devices
 * Overlay this structure with space reserved for this purpose in BIT
 *  SecondaryDevStatus[NVBOOT_SIZE_DEV_STATUS]
 */
typedef struct NvBootSpiFlashStatusRec
{
    /// Clk enable for module
    NvU32 ClkEnable;

    /// for module reset asserted
    NvU32 ClkRstStatus;

    /// transfer mode pio/dma
    NvU32 Mode;

    /// Interface bus width x1/x4
    NvU32 DataWidth;

    /// Specifies the chosen clock source
    NvU32 ClockSource;

    /// Specifies the chosen clock divider
    NvU32 ClockDivider;

    /// Specifies whether fast read was selected
    NvU32 IsFastRead;

    /// Specifies the number of pages read from the beginning.
    NvU32 NumPagesRead;

    /// Specifies the last block read
    NvU32 LastBlockRead;
    /// Specifies the last page read
    NvU32 LastPageRead;

    /// Specifies the boot status
    NvU32 BootStatus;

    /// Specifies the init status
    NvU32 InitStatus;

    /// Specifies the read status
    NvU32 ReadStatus;

    /// Specifies whether parameters successfully validated
    NvU32 ParamsValidated;

    /// Overall Qspi hw controller init, clocks/resets, pads
    NvU32 QspiInit;

    /// Data read time for payload (data throughput)
    NvU32 ReadTime;

    /// Associated Data length in bytes for ReadTime
    NvU32 Payload;
} NvBootSpiFlashStatus;


/*
 * status structures of all the supported secondary boot device types.
 */
typedef union NvBootSecondaryDeviceStatusRec
{
    /// Specifies the status from Usb3
    NvBootUsb3Status Usb3Status;

    /// Specifies the status from eMMC and eSD
    NvBootSdmmcStatus         SdmmcStatus;

    /// Specifies the status from SPI flash
    NvBootSpiFlashStatus      SpiStatus;

} NvBootSecondaryDeviceStatus;

/*
 * Boot time logging
 */

typedef struct NvBootTimeLogRec
{
    // Init timestamp
    NvU32	NvBootTimeLogInit;
    // Exit timestamp
    NvU32	NvBootTimeLogExit;
    // Coldboot Init Time Log
    NvU32 NvBootSetupTickCnt;
    // BCT read tick count
    NvU32	NvBootReadBctTickCnt;
    // BL read tick count
    NvU32	NvBootReadBLTickCnt;
} NvBootTimeLog;

typedef struct NvBootFlowLogRec
{
    // Init timestamp
    NvU32	NvBootFlowLogInit;
    // Exit timestamp
    NvU32	NvBootFlowLogExit;
    // Function Id
    NvU32	NvBootFlowFuncId;	// every function in the dispatcher is provided with id.
    // Fuction Exit Status log
    NvU32	NvBootFlowFuncStatus; // function exit status
} NvBootFlowLog;

 /**
  * Defines the status codes pertaining to BootROM flow.
 *  Each important component state is logged as the BR proceeds  
 */
typedef enum
{
    /// Specifies a default (unset) value.
    NvBootFlowStatus_None = 0,

    /// Specifies IromPatch uncorrected error
    NvBootFlowStatus_IPatchUncorrected_error,

    /// Specifies IromPatch Success.
    NvBootFlowStatus_IPatchSuccess,

    /// Specifies Oscillator setup.
    NvBootFlowStatus_SetupOscClk,

    /// Specifies Low Battery, under charged.
    NvBootFlowStatus_LowBat_NotCharged,

    /// Specifies Low Battery Success.
    NvBootFlowStatus_LowBat_Charged,
    
    /// Specifies PLLP clock enable.
    NvBootFlowStatus_PllpEnabled,

    /// Specifies Sysram/roc MSS reset deassertion
    NvBootFlowStatus_MSSInitialized,
    
    /// Specifies System Fabric initialisation
    NvBootFlowStatus_FabricInitialized,

    /// Specifies a entry to Non Secure Dispatcher routine.
    NvBootFlowStatus_NonSecureDispatcherEntry,

    /// Specifies a exiting from Non Secure Dispatcher routine.
    NvBootFlowStatus_NonSecureDispatcherExit,

    /// Specifies FA mode of operation
    NvBootFlowStatus_FAMode,

    /// Specifies Pre-production UART Mode
    NvBootFlowStatus_PreproductionModeUart,

    /// Specifies NV-production Mode
    NvBootFlowStatus_ProductionMode,

    /// Specifies ODM production Mode
    NvBootFlowStatus_OdmproductionMode,

    /// Specifies Debug RCM Mode
    NvBootFlowStatus_DbgRCMMode,

    /// Specifies Recovery Mode of operation.
    NvBootFlowStatus_RecoveryMode,

    /// Specifies a entry to Secure Dispatcher routine.
    NvBootFlowStatus_SecureDispatcherEntry,

    /// Specifies a Exit from Secure Dispatcher routine.
    NvBootFlowStatus_SecureDispatcherExit,

    /// Specifies a entry to Ramdump routine.
    NvBootFlowStatus_RamDumpInit,

    /// Specifies a entry to Ramdump setup done.
    NvBootFlowStatus_RamDumpExit,

    /// Specifies a entry to Cold Boot routine.
    NvBootFlowStatus_ColdBootEntry,

    /// Specifies a exit from Cold Boot routine.
    NvBootFlowStatus_ColdBootExit,

    /// Specifies Setup Boot device Init success.
    NvBootFlowStatus_CBSetupBootDevice,

    /// Specifies BCT Read success.
    NvBootFlowStatus_CBBctDone,

    /// Specifies Mss Init success.
    NvBootFlowStatus_MssRegionUnInitialized,
    NvBootFlowStatus_MssRegionEnableInitialized,

    /// Specifies Mts Init success. if supported.
    NvBootFlowStatus_CBMtsPrebootInit,

    /// Specifies Reinit success, If supported.
    NvBootFlowStatus_CBReinitSuccess,

    /// Specifies Sdram Init success, If supported.
    NvBootFlowStatus_CBSdramInitSuccess,

    /// Specifies Payload load success.
    NvBootFlowStatus_CBPayloadSuccess,

    /// Specifies a entry to Rcm routine.
    NvBootFlowStatus_RcmEntry,

    /// Specifies exit from RCM routine.
    NvBootFlowStatus_RcmExit,

    /// Specifies a entry to Wb0 routine.
    NvBootFlowStatus_Sc7Entry,

    /// Specifies a error in sc7 mrom acknowledge waypoint.
    NvBootFlowStatus_Sc7AckWaypoint,

    /// Specifies a error in sc7 Doorbell hw timeout.
    NvBootFlowStatus_Sc7DbellError,

    /// Specifies a exit from Wb0 routine.
    NvBootFlowStatus_Sc7Exit,

    /// Specifies a entry to Secure Boot routine.
    NvBootFlowStatus_SecureBootEntry,

    /// Specifies a Exit from Secure Boot routine.
    NvBootFlowStatus_SecureBootExit,

    /// Specifies a Reset Status Based events.
    NvBootFlowStatus_DetectedWDTReset,
    NvBootFlowStatus_DetectedAoTag_SensorReset,
    NvBootFlowStatus_DetectedVfSensorReset,
    NvBootFlowStatus_DetectedSwMainReset,
    NvBootFlowStatus_DetectedHsmReset,
    NvBootFlowStatus_DetectedCsiteDbgReset,
    NvBootFlowStatus_DetectedSc7SpeWdt_0_Reset,
    NvBootFlowStatus_DetectedSc7SpeWdt_1_Reset,
    NvBootFlowStatus_DetectedSysResetN,

    NvBootFlowStatus_CryptoInitEntry,
    NvBootFlowStatus_CryptoInitExit,

    // Start of the secure exit dispatcher.
    NvBootFlowStatus_SecureExitStart,

    NvBootFlowStatus_Force32 = 0x7fffffff
} NvBootFlowStatus;


/**
 * Defines the BIT.
 *
 * Notes:
 * * SecondaryDevice: This is set by cold boot (and soon UART) processing.
 *   Recovery mode does not alter its value.
 * * BctStatus[] is a bit vector representing the cause of BCT read failures.
 *       A 0 bit indicates a validation failure
 *       A 1 bit indicates a device read error
 *       Bit 0 contains the status for the BCT at block 0, slot 0.
 *       Bit 1 contains the status for the BCT at block 0, slot 1.
 *       Bit N contains the status for the BCT at block (N-1), slot 0, which
 *       is a failed attempt to locate the journal block at block N.
 *       (1 <= N < NVBOOT_MAX_BCT_SEARCH_BLOCKS)
 * * BctLastJournalRead contains the cause of the BCT search within the
 *   journal block. Success indicates the search ended with the successful
 *   reading of the BCT in the last slot.  CRC and ECC failures are indicated
 *   appropriately.
 */
typedef struct NvBootInfoTableRec
{
    ///
    /// Version information
    ///

    /// Specifies the version number of the BR code.
    /// !! NOTE !!
    /// This MUST be the first entry in the BIT. BR
    /// uses this field to temporarily store some exit configuration
    /// data. BR will set the correct Boot ROM version number
	/// just before exit.
    ///
    NvU32               BootRomVersion;

    /// Specifies the version number of the BR data structure.
    NvU32               DataVersion;

    /// Specifies the version number of the RCM protocol.
    NvU32               RcmVersion;

    /// Specifies the type of boot.
    NvBootType          BootType;

    /// Specifies the primary boot device.
    NvBootDevType       PrimaryDevice;

    /// Specifies the secondary boot device.
    NvBootDevType       SecondaryDevice;

    /// Specifies the Authentication scheme
    NvU32            AuthenticationScheme;
	
    /// Specifies the Encryption scheme
    NvBool           EncryptionEnabled;

    /// BootROM flow tracker
    NvBootFlowStatus BootROMtracker;
    /// Boot time logging
    NvBootTimeLog	BootTimeLog;

	/// Boot flow logging
    NvBootFlowLog   BootFlowLog[NVBOOT_FLOW_LOG_DEPTH];

    ///
    /// Hardware status information
    ///

    /// Specifies the measured oscillator frequency.
    NvBootClocksOscFreq OscFrequency;

    /// Specifies whether the device was initialized.
    NvBool              DevInitialized;

    /// Specifies whether SDRAM was initialized.
    NvBool              SdramInitialized;

    /// Specifies whether the ForceRecovery AO bit was cleared.
    NvBool              ClearedForceRecovery;

    /// Specifies whether the FailBack AO bit was cleared.
    NvBool              ClearedFailBack;

    /// Specifies whether FailBack was invoked.
    NvBool              InvokedFailBack;

    /// IROM patch status
    // [3:0] - Hamming decode status
    // [6:4] - Reserved '0'
    // [7:7] - IROM patch fuse payload present
    NvU8                IRomPatchStatus;

    ///
    /// BCT information
    ///

    /// Specifies if a Size of BCT was found to be valid.
    NvBool              BctSizeValid;

    /// Specifies the status of attempting to read Size of BCT during the
    /// initial search process.  See the notes above for more details.
    NvU8                BctSizeStatus[NVBOOT_BCT_STATUS_BYTES];

    /// Specifies the status of the last journal block read for BCT size determination.
    NvBootRdrStatus     BctSizeLastJournalRead;

    /// Specifies the block number in which the Size of BCT was validated.
    NvU32               BctSizeBlock;

    /// Specifies the page number of the start of the Size of BCT was validated.
    NvU32               BctSizePage;

    /// Specifies if a valid BCT was found.
    NvBool              BctValid;

    /// Specifies the status of attempting to read BCTs during the
    /// BCT search process.  See the notes above for more details.
    NvU8                BctStatus[NVBOOT_BCT_STATUS_BYTES];

    /// Specifies the status of the last journal block read.
    NvBootRdrStatus     BctLastJournalRead;

    /// Specifies the block number in which the BCT was found.
    NvU32               BctBlock;

    /// Specifies the page number of the start of the BCT that was found.
    NvU32               BctPage;

    /// Specifies the size of the BCT in bytes.  It is 0 until BCT loading
    /// is attempted.
    NvU32               BctSize;  /* 0 until BCT loading is attempted */

    /// Specifies a pointer to the BCT in memory.  It is NULL until BCT
    /// loading is attempted.  The BCT in memory is the last BCT that
    /// the BR tried to load, regardless of whether the operation was
    /// successful.
    NvBootConfigTable  *BctPtr;

    /// Specifies the state of attempting to load each of the BLs.
    NvBootBlState       BlState[NVBOOT_MAX_BOOTLOADERS];

    /// Specifies device-specific status information from the operation
    /// of the secondary boot device. Size define nvboot_config.h
    NvU32 SecondaryDevStatus[NVBOOT_DEV_STATUS_SIZE_BYTES/4];

    /// Specifies the status of usb charger detection
    /// [0:0] = ChargerDetectionEnabled (0 if disabled, 1 if enabled)
    /// [1:1] = IsBatteryLowDetected (1 if low, 0 if high)
    /// [2:2] = IsUsbCableConnected (1 if usb cable is connected, else 0)
    /// [3:3] = RESERVED. Was IsChargingPort (1 if charging port, else 0 if SDP)
    /// [4:4] = RESERVED. Was IsDividerChargerDetected (1 if divider charger detected, else 0)
    /// [5:5] = RESERVED. Was IsACADetected (1 if ACA A/B/C detected, else 0)
    /// [6:6] = RESERVED. Was IsNonCompliantChargerDetected (1 if charger is non-compliant, else 0)
    /// [7:7] = RESERVED. Was IsDcpCdpDetected (1 if DCP/CDP, else 0)
    /// [8:8] = PmicHighCurrentStatusAsserted (1 if driven high, 0 if low)
    /// [9:9] = RESERVED. Was DeviceEnumerationComplete (1 if device enumeration performed)
    /// [10:10] = RESERVED. Was DeviceEnumerationFailed (1 if device enum failed, else 0)
    /// [11:11] = RESERVED. Was ReadPMICUsbChargerControlForHiCurrentFailed (set to 1 if read/write failed, else 0)
    /// [12:12] = RESERVED. Was UsbChargingBITStat_ReadPMICUsbChargerControlForUsbSuspendFailed (set to 1 if
    ///        read/write failed, else 0)
    /// [13:13] = RESERVED. Was UsbChargingBITStat_Timer0Disabled (set to 1 if Timer0 disabled, else 0)
    NvU32 UsbChargingStatus;

    /// NvBootError_Success implies reading the BOOT_SEL
    /// register from the PMIC was successful.
    NvBool PmuBootSelReadError;

    /// Specifies the lowest iRAM address that preserves communicated data.
    /// SafeStartAddr starts out with the address of memory following
    /// the BIT.  When BCT loading starts, it is bumped up to the
    /// memory following the BCT.
    NvU32               SafeStartAddr;

} NvBootInfoTable;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BIT_H */
