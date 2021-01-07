/*
 * Copyright (c) 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef NVBOOT_SATA_AHCI_STRUCT_H
#define NVBOOT_SATA_AHCI_STRUCT_H


// CMD0 Header in Command List
#define NVBOOT_SATA_CMD0_0                 0x0
// This word count will help calculate which offset to read based on the command
// number. Haven't yet put any restriction over the number of commands.
// As per the SATA spec, the total number of commands should be max
// 32 (CM0-CMD31)
#define NVBOOT_SATA_CMD0_0_WORD_COUNT                      0x8

// Note: Read and Write masks could be different for different words based on
// the Reserved bits.
// Command Header 0 DWORD0
#define NVBOOT_SATA_CMD0_0_DWORD0               0x0
#define NVBOOT_SATA_CMD0_0_DWORD0_READ_MASK         0xFFFFF7FF
#define NVBOOT_SATA_CMD0_0_DWORD0_WRITE_MASK                    0xFFFFF7FF

// Physical Region Descriptor Table Length
#define NVBOOT_SATA_CMD0_0_DWORD0_PRDTL_RANGE                   31:16

// Port Multiplier Port
#define NVBOOT_SATA_CMD0_0_DWORD0_PMP_RANGE                     15:12

// Clear Busy Upon R_OK
#define NVBOOT_SATA_CMD0_0_DWORD0_CLEAR_BUSY_RANGE              10:10

// Send BIST FIS
#define NVBOOT_SATA_CMD0_0_DWORD0_BIST_RANGE                    9:9

// Reset
#define NVBOOT_SATA_CMD0_0_DWORD0_RESET_RANGE                   8:8

// Prefetch
#define NVBOOT_SATA_CMD0_0_DWORD0_PREFETCH_RANGE                7:7

// Write
#define NVBOOT_SATA_CMD0_0_DWORD0_WRITE_RANGE                   6:6

// ATAPI
#define NVBOOT_SATA_CMD0_0_DWORD0_ATAPI_RANGE                   5:5

// Command FIS Length
#define NVBOOT_SATA_CMD0_0_DWORD0_CFL_RANGE                     4:0


// Command Header 0 DWORD1
#define NVBOOT_SATA_CMD0_0_DWORD1                       0x20
#define NVBOOT_SATA_CMD0_0_DWORD1_READ_MASK                     0xFFFFFFFF
#define NVBOOT_SATA_CMD0_0_DWORD1_WRITE_MASK                    0xFFFFFFFF

// Physical Region Descriptor Byte Count
#define NVBOOT_SATA_CMD0_0_DWORD1_PRDBC_RANGE                   31:0

// Command Header 0 DWORD2
#define NVBOOT_SATA_CMD0_0_DWORD2                       0x40
#define NVBOOT_SATA_CMD0_0_DWORD2_READ_MASK                     0xFFFFFF80
#define NVBOOT_SATA_CMD0_0_DWORD2_WRITE_MASK                    0xFFFFFF80

// Command Table Descriptor Base Address
#define NVBOOT_SATA_CMD0_0_DWORD2_CTBA_RANGE                    31:7

// Command Header 0 DWORD3
#define NVBOOT_SATA_CMD0_0_DWORD3                       0x60
#define NVBOOT_SATA_CMD0_0_DWORD3_READ_MASK                     0xFFFFFFFF
#define NVBOOT_SATA_CMD0_0_DWORD3_WRITE_MASK                    0xFFFFFFFF

// Command Table Descriptor Base Address Upper 32 bits
#define NVBOOT_SATA_CMD0_0_DWORD3_CTBAU_RANGE                   31:0

// Command Header 0 DWORD4 to DWORD7 are Reserved and should be set to 0's


// Each command header in command list points to a command table
// A command table consists of a CFIS, ACMD and Physical Region Descriptor tables

// For CFIS structure see section 10.3.4 on register H2D FIS of Sata Gold 2.6 spec
#define NVBOOT_SATA_CFIS_0                          0x0
#define NVBOOT_SATA_CFIS_0_WORD_COUNT                       0x5

#define NVBOOT_SATA_CFIS_0_DWORD0                       0x0
#define NVBOOT_SATA_CFIS_0_DWORD0_READ_MASK                     0xFFFF8FFF
#define NVBOOT_SATA_CFIS_0_DWORD0_WRITE_MASK                    0xFFFF8FFF

// Contents of the Features Register of the Shadow register block
#define NVBOOT_SATA_CFIS_0_DWORD0_FEATURES_RANGE                31:24

// Contents of the Command Register of the Shadow register block
#define NVBOOT_SATA_CFIS_0_DWORD0_COMMAND_RANGE                 23:16

/* 
 * This bit is set to one when the register transfer is due to an update of the
 * Command register. The bit is cleared to zero when the register transfer is
 * due to an update of the Device Control register. Setting C bit to one and
 * SRST bit to one in the Device Control Field is invalid and results in 
 * indeterminate behavior.
 */
#define NVBOOT_SATA_CFIS_0_DWORD0_C_RANGE                       15:15

// 16:16 is Reserved and should be always 0 (Reflected in the read mask and write mask
// for Register H2D FIS)

/*
 * When an endpoint device is attached via a Port Multiplier, specifies the
 * device port address that the FIS should be delivered to. This field is set
 * by the host.
 */
#define NVBOOT_SATA_CFIS_0_DWORD0_PMPORT_RANGE                  11:8

/*
 * Set to a value of 27h. Defines the rest of the FIS fields. Defines the length
 * of the FIS as five Dwords.
 */
#define NVBOOT_SATA_CFIS_0_DWORD0_FISTYPE_RANGE                 7:0

#define NVBOOT_SATA_CFIS_0_DWORD1                       0x20
#define NVBOOT_SATA_CFIS_0_DWORD1_READ_MASK                     0xFFFFFFFF
#define NVBOOT_SATA_CFIS_0_DWORD1_WRITE_MASK                    0xFFFFFFFF

// Contents of the Device register of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD1_DEVICE_RANGE                  31:24

// Contents of the LBA High register of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD1_LBA_HIGH_RANGE                23:16

// Contents of the LBA Mid register of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD1_LBA_MID_RANGE                 15:8

// Contents of the LBA Low register of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD1_LBA_LOW_RANGE                 7:0

#define NVBOOT_SATA_CFIS_0_DWORD2                       0x40
#define NVBOOT_SATA_CFIS_0_DWORD2_READ_MASK                     0xFFFFFFFF
#define NVBOOT_SATA_CFIS_0_DWORD2_WRITE_MASK                    0xFFFFFFFF

// Ccontents of the expanded address field of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD2_FEATURES_EXP_RANGE            31:24

#define NVBOOT_SATA_CFIS_0_DWORD2_LBA_HIGH_EXP_RANGE            23:16

#define NVBOOT_SATA_CFIS_0_DWORD2_LBA_MID_EXP_RANGE             15:8

#define NVBOOT_SATA_CFIS_0_DWORD2_LBA_LOW_EXP_RANGE             7:0


#define NVBOOT_SATA_CFIS_0_DWORD3                       0x60
#define NVBOOT_SATA_CFIS_0_DWORD3_READ_MASK                     0xFF00FFFF
#define NVBOOT_SATA_CFIS_0_DWORD3_WRITE_MASK                    0xFF00FFFF

// Contents of the Device Control register of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD3_CONTROL_RANGE                 31:24

// Contents of the Sector Count register of the Shadow Register Block.
#define NVBOOT_SATA_CFIS_0_DWORD3_SECTOR_COUNT_EXP_RANGE        15:8

#define NVBOOT_SATA_CFIS_0_DWORD3_SECTOR_COUNT_RANGE            7:0

// CFIS DWORD4 is Reserved and should be set to 0's
#define NVBOOT_SATA_CFIS_0_DWORD4                       0x80
#define NVBOOT_SATA_CFIS_0_DWORD4_READ_MASK                     0x00000000
#define NVBOOT_SATA_CFIS_0_DWORD4_WRITE_MASK                    0x00000000

// CFIS DWORD5-DWORD15 are Reserved and should be set to 0's
#define NVBOOT_SATA_CFIS_MAX_0_WORD_COUNT                       0x10

#define NVBOOT_SATA_ATAPI_CMD_0                         0x200
#define NVBOOT_SATA_ATAPI_CMD_0_WORD_COUNT                  0x4

// 0x30 bytes are reserved
#define NVBOOT_SATA_RESERVED_0                         0x280
#define NVBOOT_SATA_RESERVED_0_WORD_COUNT                  0xC

// Physical Region Descriptor Table 0 in the command table
#define NVBOOT_SATA_PRDT0_0                             0x400
// This word count will help calculate which offset to read physical region
// descriptors based on the tableindex number. Haven't yet put any restriction
// over the number of number of descriptor tables.
// As per the SATA spec, the total number of tables should be max 65535
#define NVBOOT_SATA_PRDT0_0_WORD_COUNT                          0x4

#define NVBOOT_SATA_PRDT0_0_DWORD0                      0x0
#define NVBOOT_SATA_PRDT0_0_DWORD0_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_PRDT0_0_DWORD0_WRITE_MASK                   0xFFFFFFFF

// Data Base Address
#define NVBOOT_SATA_PRDT0_0_DWORD0_DBA_RANGE                    31:1


#define NVBOOT_SATA_PRDT0_0_DWORD1                      0x20
#define NVBOOT_SATA_PRDT0_0_DWORD1_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_PRDT0_0_DWORD1_WRITE_MASK                   0xFFFFFFFF

// Data Base Address Upper 32 bits
#define NVBOOT_SATA_PRDT0_0_DWORD1_DBAU_RANGE                   31:0

// PRDT DWORD2 is Reserved and should be set to 0's
#define NVBOOT_SATA_PRDT0_0_DWORD2                      0x40

#define NVBOOT_SATA_PRDT0_0_DWORD3                      0x60
#define NVBOOT_SATA_PRDT0_0_DWORD3_READ_MASK                    0x0
#define NVBOOT_SATA_PRDT0_0_DWORD3_WRITE_MASK                   0x0

// Interrupt on Completion
#define NVBOOT_SATA_PRDT0_0_DWORD3_IOC_RANGE                    31:31

// Data Byte Count
#define NVBOOT_SATA_PRDT0_0_DWORD3_DBC_RANGE                    21:0


// All write masks are set to 0x00000000 because FISes are to be read
// and interpreted. They are not expected to be written/modified by SW

/* DMA Setup FIS */
#define NVBOOT_SATA_DSFIS_0                             0x60
#define NVBOOT_SATA_DSFIS_0_WORD_COUNT                      0x7

#define NVBOOT_SATA_DSFIS_0_DWORD0                      0x0
#define NVBOOT_SATA_DSFIS_0_DWORD0_READ_MASK                    0x0000EFFF
#define NVBOOT_SATA_DSFIS_0_DWORD0_WRITE_MASK                   0x0000EFFF

// 31:16 is Reserved and should be set to 0 (reflected in Read mask and Write
// mask for DMA Setup FIS)
// Auto Activate
#define NVBOOT_SATA_DSFIS_0_DWORD0_AUTO_ACTIVATE_RANGE          15:15

// Interrupt
#define NVBOOT_SATA_DSFIS_0_DWORD0_INTERRUPT_RANGE              14:14

// Direction of transfer
#define NVBOOT_SATA_DSFIS_0_DWORD0_DIRECTION_RANGE              13:13

// 12:12 is Reserved and should be set to 0 (reflected in Read mask and Write
// mask for DMA Setup FIS)

// Port Multiplier Port
#define NVBOOT_SATA_DSFIS_0_DWORD0_PMPORT_RANGE                 11:8

// Default value is 0x41 for Register D2H
#define NVBOOT_SATA_DSFIS_0_DWORD0_FISTYPE_RANGE                7:0


#define NVBOOT_SATA_DSFIS_0_DWORD1                      0x20
#define NVBOOT_SATA_DSFIS_0_DWORD1_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_DSFIS_0_DWORD1_WRITE_MASK                   0xFFFFFFFF

// DMA Buffer Identifier Low
#define NVBOOT_SATA_DSFIS_0_DWORD1_DMABUF_IDENT_LO_RANGE        31:0

#define NVBOOT_SATA_DSFIS_0_DWORD2                      0x40
#define NVBOOT_SATA_DSFIS_0_DWORD2_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_DSFIS_0_DWORD2_WRITE_MASK                   0xFFFFFFFF

// DMA Buffer Identifier High
#define NVBOOT_SATA_DSFIS_0_DWORD2_DMABUF_IDENT_HI_RANGE        31:0

// Reserved 
#define NVBOOT_SATA_DSFIS_0_DWORD3                      0x60
#define NVBOOT_SATA_DSFIS_0_DWORD3_READ_MASK                    0x00000000
#define NVBOOT_SATA_DSFIS_0_DWORD3_WRITE_MASK                   0x00000000


#define NVBOOT_SATA_DSFIS_0_DWORD4                      0x80
#define NVBOOT_SATA_DSFIS_0_DWORD4_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_DSFIS_0_DWORD4_WRITE_MASK                   0xFFFFFFFF

// DMA Buffer Identifier Offset
#define NVBOOT_SATA_DSFIS_0_DWORD4_DMABUF_OFFSET_RANGE          31:0

#define NVBOOT_SATA_DSFIS_0_DWORD5                      0xA0
#define NVBOOT_SATA_DSFIS_0_DWORD5_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_DSFIS_0_DWORD5_WRITE_MASK                   0xFFFFFFFF

// DMA Transfer Count
#define NVBOOT_SATA_DSFIS_0_DWORD5_DMA_XFER_CNT_RANGE           31:0

// DSFIS WORD3 and WORD6 are Reserved and should be set to 0's
// Reserved 
#define NVBOOT_SATA_DSFIS_0_DWORD6                      0xC0
#define NVBOOT_SATA_DSFIS_0_DWORD6_READ_MASK                    0x00000000
#define NVBOOT_SATA_DSFIS_0_DWORD6_WRITE_MASK                   0x00000000

/* RESERVED 1 DWORD */
#define NVBOOT_SATA_FIS_RSVD1_0_WORD_COUNT        0x1

/* PIO Setup FIS */
#define NVBOOT_SATA_PSFIS_0                             0x100
#define NVBOOT_SATA_PSFIS_0_WORD_COUNT                      0x5

#define NVBOOT_SATA_PSFIS_0_DWORD0                      0x0
#define NVBOOT_SATA_PSFIS_0_DWORD0_READ_MASK                    0xFFFF6FFF
#define NVBOOT_SATA_PSFIS_0_DWORD0_WRITE_MASK                   0xFFFF6FFF

// Error Register of Command Block
#define NVBOOT_SATA_PSFIS_0_DWORD0_ERROR_RANGE                  31:24

// Status Register of Command Block for initiation of host data transfer
#define NVBOOT_SATA_PSFIS_0_DWORD0_STATUS_RANGE                 23:16

// 15:15 is Reserved and should be set to 0 (reflected in Read mask and Write
// mask for PIO Setup FIS)

// Interrupt
#define NVBOOT_SATA_PSFIS_0_DWORD0_INTERRUPT_RANGE              14:14

// Direction of data transfer
#define NVBOOT_SATA_PSFIS_0_DWORD0_DIRECTION_RANGE              13:13

// 12:12 is Reserved and should be set to 0 (reflected in Read mask and Write
// mask for PIO Setup FIS)

// Port Multiplier Port
#define NVBOOT_SATA_PSFIS_0_DWORD0_PMPORT_RANGE                 11:8

// Default value is 0x5F for Register D2H
#define NVBOOT_SATA_PSFIS_0_DWORD0_FISTYPE_RANGE                7:0

#define NVBOOT_SATA_PSFIS_0_DWORD1                      0x20
#define NVBOOT_SATA_PSFIS_0_DWORD1_READ_MASK                    0xFFFFFFFF
#define NVBOOT_SATA_PSFIS_0_DWORD1_WRITE_MASK                   0xFFFFFFFF

// Device register of Command Block
#define NVBOOT_SATA_PSFIS_0_DWORD1_DEVICE_RANGE                 31:24

// LBA High register of Command Block
#define NVBOOT_SATA_PSFIS_0_DWORD1_LBA_HIGH_RANGE               23:16

// LBA Mid register of Command Block
#define NVBOOT_SATA_PSFIS_0_DWORD1_LBA_MID_RANGE                15:8

// LBA Low register of Command Block
#define NVBOOT_SATA_PSFIS_0_DWORD1_LBA_LOW_RANGE                7:0

#define NVBOOT_SATA_PSFIS_0_DWORD2                      0x40
#define NVBOOT_SATA_PSFIS_0_DWORD2_READ_MASK                    0x00FFFFFF
#define NVBOOT_SATA_PSFIS_0_DWORD2_WRITE_MASK                   0x00FFFFFF

// Expanded address fields of Shadow Block
#define NVBOOT_SATA_PSFIS_0_DWORD2_LBA_HIGH_EXP_RANGE           23:16
#define NVBOOT_SATA_PSFIS_0_DWORD2_LBA_MID_EXP_RANGE            15:8
#define NVBOOT_SATA_PSFIS_0_DWORD2_LBA_LOW_EXP_RANGE            7:0

#define NVBOOT_SATA_PSFIS_0_DWORD3                      0x60
#define NVBOOT_SATA_PSFIS_0_DWORD3_READ_MASK                    0xFF00FFFF
#define NVBOOT_SATA_PSFIS_0_DWORD3_WRITE_MASK                   0xFF00FFFF

// Status Register of Command Block at completion of a host data transfer
#define NVBOOT_SATA_PSFIS_0_DWORD3_ESTATUS_RANGE                31:24

// 23:16 is Reserved and should be 0 (reflected in Read/Write masks for PIO Setup FIS
// Expanded address field of Shadow Block
#define NVBOOT_SATA_PSFIS_0_DWORD3_SECTOR_COUNT_EXP_RANGE       15:8

// Sector Count register of Command Block
#define NVBOOT_SATA_PSFIS_0_DWORD3_SECTOR_COUNT_RANGE           7:0

#define NVBOOT_SATA_PSFIS_0_DWORD4                      0x60
#define NVBOOT_SATA_PSFIS_0_DWORD4_READ_MASK                    0x0000FFFF
#define NVBOOT_SATA_PSFIS_0_DWORD4_WRITE_MASK                   0x0000FFFF

// 31:16 is Reserved

// Number of bytes to transferred in subsequent Data FIS
#define NVBOOT_SATA_PSFIS_0_DWORD4_TRANSFER_CNT_RANGE                15:8

/* RESERVED 3 DWORDs */
#define NVBOOT_SATA_FIS_RSVD2_0_WORD_COUNT        0x3

/* Register Device To Host(Register D2H) */
#define NVBOOT_SATA_RFIS_0                          0x200
#define NVBOOT_SATA_RFIS_0_WORD_COUNT                       0x5

#define NVBOOT_SATA_RFIS_0_DWORD0                       0x0
#define NVBOOT_SATA_RFIS_0_DWORD0_READ_MASK                     0xFFFF4FFF
#define NVBOOT_SATA_RFIS_0_DWORD0_WRITE_MASK                    0x00000000

// Error Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD0_ERROR_RANGE                   31:24

// Status Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD0_STATUS_RANGE                  23:16

// 15:15 is Reserved and should be 0 (reflected in the Read mask
// for Register D2H FIS)

// Interrupt
#define NVBOOT_SATA_RFIS_0_DWORD0_INTERRUPT_RANGE               14:14

// 13:12 is Reserved and should be 0 (reflected in the Read mask
// for Register D2H FIS)

// Port Multiplier Port
#define NVBOOT_SATA_RFIS_0_DWORD0_PMPORT_RANGE                  11:8

// Default value is 0x34 for Register D2H
#define NVBOOT_SATA_RFIS_0_DWORD0_FISTYPE_RANGE                 7:0

#define NVBOOT_SATA_RFIS_0_DWORD1                       0x20
#define NVBOOT_SATA_RFIS_0_DWORD1_READ_MASK                     0xFFFFFFFF
#define NVBOOT_SATA_RFIS_0_DWORD1_WRITE_MASK                    0x00000000

// Device Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD1_DEVICE_RANGE                  31:24

// LBA High Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD1_LBA_HIGH_RANGE                23:16

// LBA Mid Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD1_LBA_MID_RANGE                 15:8

// LBA Low Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD1_LBA_LOW_RANGE                 7:0

#define NVBOOT_SATA_RFIS_0_DWORD2                       0x40
#define NVBOOT_SATA_RFIS_0_DWORD2_READ_MASK                     0x00FFFFFF
#define NVBOOT_SATA_RFIS_0_DWORD2_WRITE_MASK                    0x00000000

// 31:24 is Reserved and should be 0 (reflected in the Read mask
// for Register D2H FIS)

// Expanded Address Fields of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD2_LBA_HIGH_EXP_RANGE            23:16

#define NVBOOT_SATA_RFIS_0_DWORD2_LBA_MID_EXP_RANGE             15:8

#define NVBOOT_SATA_RFIS_0_DWORD2_LBA_LOW_EXP_RANGE             7:0


#define NVBOOT_SATA_RFIS_0_DWORD3                       0x60
#define NVBOOT_SATA_RFIS_0_DWORD3_READ_MASK                     0x0000FFFF
#define NVBOOT_SATA_RFIS_0_DWORD3_WRITE_MASK                    0x00000000

// 31:16 is Reserved and should be 0 (reflected in the Read mask
// for Register D2H FIS)

#define NVBOOT_SATA_RFIS_0_DWORD3_SECTOR_COUNT_EXP_RANGE        15:8

// Sector Count Register of Shadow Register Block
#define NVBOOT_SATA_RFIS_0_DWORD3_SECTOR_COUNT_RANGE            7:0

// Reserved and should be set to 0's
#define NVBOOT_SATA_RFIS_0_DWORD4                       0x80
#define NVBOOT_SATA_RFIS_0_DWORD4_READ_MASK                     0x00000000
#define NVBOOT_SATA_RFIS_0_DWORD4_WRITE_MASK                    0x00000000

/* RESERVED 1 DWORD */
#define NVBOOT_SATA_FIS_RSVD3_0_WORD_COUNT        0x1


/* Set Device Bits FIS */
#define NVBOOT_SATA_SDBFIS_0                            0x2C0
#define NVBOOT_SATA_SDBFIS_0_WORD_COUNT                     0x2

#define NVBOOT_SATA_SDBFIS_0_DWORD0                         0x0
#define NVBOOT_SATA_SDBFIS_0_DWORD0_READ_MASK                   0xFF77CFFF
#define NVBOOT_SATA_SDBFIS_0_DWORD0_WRITE_MASK                  0x00000000

// Error Register of Shadow Register Block
#define NVBOOT_SATA_SDBFIS_0_DWORD0_ERROR_RANGE                 31:24

// 23:23 is Reserved and should be 0 (reflected in the Read mask
// for Set Device Bits FIS)

// Status Register[6:5] of Shadow Register Block
#define NVBOOT_SATA_SDBFIS_0_DWORD0_STATUS_HI_RANGE             22:20

// 19:19 is Reserved and should be 0 (reflected in the Read mask
// for Set Device Bits FIS)

// Status Register[2:0] of Shadow Register Block
#define NVBOOT_SATA_SDBFIS_0_DWORD0_STATUS_LOW_RANGE            18:16

// Notification that device needs host's attention
#define NVBOOT_SATA_SDBFIS_0_DWORD0_NOTIFICATION_RANGE          15:15

// Interrupt
#define NVBOOT_SATA_SDBFIS_0_DWORD0_INTERRUPT_RANGE             14:14

// 13:12 is Reserved and should be 0 (reflected in the Read mask
// for Set Device Bits FIS)

// Port Multiplier Port
#define NVBOOT_SATA_SDBFIS_0_DWORD0_PMPORT_RANGE                11:8

/* Value = 0xA1 for Set Device Bits FIS */
#define NVBOOT_SATA_SDBFIS_0_DWORD0_FISTYPE_RANGE               7:0

// SDBFIS DWORD1 is Reserved and should be set to 0's
#define NVBOOT_SATA_SDBFIS_0_DWORD1                         0x20
#define NVBOOT_SATA_SDBFIS_0_DWORD1_READ_MASK                   0x00000000
#define NVBOOT_SATA_SDBFIS_0_DWORD1_WRITE_MASK                  0x00000000

/* Unknown FIS */
#define NVBOOT_SATA_UFIS_0                          0x300
#define NVBOOT_SATA_UFIS_0_WORD_COUNT                       0x10

/* RESERVED 24 DWORDs */
#define NVBOOT_SATA_FIS_RSVD4_0_WORD_COUNT        0x18 

#endif /* #ifndef NVBOOT_SATA_AHCI_STRUCT_H */
