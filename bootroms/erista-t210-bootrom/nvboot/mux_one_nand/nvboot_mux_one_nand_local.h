/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_mux_one_nand_local.h - Internal definitions for using muxone nand 
 * as a second level boot device.
 */

#ifndef INCLUDED_NVBOOT_MUX_ONE_NAND_LOCAL_H
#define INCLUDED_NVBOOT_MUX_ONE_NAND_LOCAL_H

#if defined(__cplusplus)
extern "C"
{
#endif

// The read timeout in microsecond for the 1 page.
#define SNOR_HW_TIMEOUT_US 100000   // 100ms
#define COMMAND_LOAD_FAILURE (1 <<10)    
#define COMMAND_OP_ONGOING 0x8000

//  Device reset timeout: The maximum time to wait for device to be out of reset
// For FlexMuxOneNand the it is 150 micosecond, 
// for MuxOneNand and OneNand 500 micorsecond
// For worstcase it can 2ms maximum 
// So keeping the timeout to 5 times i.e. 10ms
#define MUXONENAND_RESET_TIMEOUT_US 10000 

// Mux one nand Address map.
#define MUXONENAND_BOOT_RAM0_MAIN                         0x0000
#define MUXONENAND_BOOT_RAM0_SPARE                        0x8000
#define MUXONENAND_DATA_RAM0_MAIN                         0x0200
#define MUXONENAND_DATA_RAM0_SPARE                        0x8010

#define MUXONENAND_MAN_ID                                 0xF000
#define MUXONENAND_DEV_ID                                 0xF001
#define MUXONENAND_VERSION_ID                             0xF002
#define MUXONENAND_DATA_BUFF_SIZE                         0xF003
#define MUXONENAND_BOOT_BUFFER_SIZE                       0xF004
#define MUXONENAND_BUFFER_AMMOUNT                         0xF005
#define MUXONENAND_TECHNOLOGY                             0xF006
#define MUXONENAND_CHIPADD_BLOCK_ADD                      0xF100
#define MUXONENAND_CHIPADD_BUFFER_RAM                     0xF101
#define MUXONENAND_DEST_BLOCK_ADD                         0xF103
#define MUXONENAND_NUMBER_OF_PAGE                         0xF105
#define MUXONENAND_PAGE_SECTOR_ADD                        0xF107
#define MUXONENAND_BUFFER_NUMBER                          0xF200
#define MUXONENAND_MEM_OP_COMMAND                         0xF220
#define MUXONENAND_MEM_CONFIG                             0xF221
#define MUXONENAND_MEM_OP_STATUS                          0xF240
#define MUXONENAND_INTERRUPT_STATUS                       0xF241
#define MUXONENAND_BLOCK_ADD_WRITE_PROTECTION             0xF24C
#define MUXONENAND_WRITE_PROTECTION_STATUS                0xF24E
#define MUXONENAND_ECC_STATUS                             0xFF00
#define MUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_1          0xFF01
#define MUXONENAND_ECC_RESULT_SPARE_AREA_SECTOR_1         0xFF02
#define MUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_2          0xFF03
#define MUXONENAND_ECC_RESULT_SPARE_AREA_SECTOR_2         0xFF04
#define MUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_3          0xFF05
#define MUXONENAND_ECC_RESULT_SPARE_AREA_SECTOR_3         0xFF06
#define MUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_4          0xFF07
#define MUXONENAND_ECC_RESULT_SPARE_AREA_SECTOR_4         0xFF08

#define FLEXMUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_1_2        0xFF00
#define FLEXMUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_3_4        0xFF01
#define FLEXMUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_5_6        0xFF02
#define FLEXMUXONENAND_ECC_RESULT_MAIN_AREA_SECTOR_7_8        0xFF03

#define INTERRUPT_STATUS_DQ_15 (1 << 15)
#define RESET_OPERATION_BUSY 0x8001
#define CONTROLLER_STATUS_ERROR_DQ_10 (1 << 10)
#define BSA_DATA_RAM_SELCECT (1 << 11)

#define MUXONENNAD_COMMAND_DATALOAD 0x0

#define MUXONENAND_SYSCONFIG_0_ECC_RANGE 8:8
#define MUXONENAND_SYSCONFIG_0_ECC_ENABLE  0
#define MUXONENAND_SYSCONFIG_0_ECC_DISABLE 1
    
    
#define MUXONENAND_ECCSTATUS_ERROR_UNCORRECTABLE 0x8888
    
#define FLEXMUXONENAND_ECCSTATUS_0_EVEN_SECTOR_RANGE 4:0
#define FLEXMUXONENAND_ECCSTATUS_0_ODD_SECTOR_RANGE 12:8
    
#define FLEXMUXONENAND_ECC_NO_ERROR 0
#define FLEXMUXONENAND_ECC_1BIT_ERROR_CORRECTABLE 1
#define FLEXMUXONENAND_ECC_2BIT_ERROR_CORRECTABLE 2
#define FLEXMUXONENAND_ECC_3BIT_ERROR_CORRECTABLE 4
#define FLEXMUXONENAND_ECC_4BIT_ERROR_CORRECTABLE 8
#define FLEXMUXONENAND_ECC_MAX_ERROR_CORRECTABLE \
            FLEXMUXONENAND_ECC_4BIT_ERROR_CORRECTABLE
#define FLEXMUXONENAND_ECC_MOREBIT_ERROR_UNCORRECTABLE 16

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MUX_ONE_NAND_LOCAL_H */
