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
 * nvboot_mobile_lba_nand_local.h - Internal definitions for using
 * MobileLbaNand as a second level boot device.
 */

#ifndef INCLUDED_NVBOOT_MOBILE_LBA_NAND_LOCAL_H
#define INCLUDED_NVBOOT_MOBILE_LBA_NAND_LOCAL_H

#if defined(__cplusplus)
extern "C"
{
#endif

/** 
 * These defines are as per nand Flash spec's.
 */
#define ML_NAND_DEVICE_READID_0_MAKER_CODE_RANGE 7:0
#define ML_NAND_DEVICE_READID_0_DEVICE_CODE_RANGE 15:8
#define ML_NAND_DEVICE_READID_0_BUS_WIDTH_RANGE 30:30

#define NVBOOT_NAND_COMMAND_TIMEOUT_IN_US 100000
#define NVBOOT_NAND_READ_TIMEOUT_IN_US 500000

/// Max clock divider supported.
#define NVBOOT_NAND_MAX_CLOCK_DIVIDER_SUPPORTED 127

/// These defines are needed for MobileLbaNand Addressing format.
#define NVBOOT_NAND_ADDR_REG_1_PAGE_OFFSET 16
#define NVBOOT_NAND_ADDR_REG_1_PAGE_MASK 0xFFFF
#define NVBOOT_NAND_ADDR_REG_2_PAGE_OFFSET 16
#define NVBOOT_NAND_BITS_PER_ADDRESS_REGISTER 32

#define NVBOOT_ML_NAND_PAGE_SIZE_LOG2 0x9
#define NVBOOT_ML_NAND_BLOCK_SIZE_LOG2 0xE
#define NVBOOT_ML_NAND_PAGES_PER_BLOCK_LOG2 (NVBOOT_ML_NAND_BLOCK_SIZE_LOG2 - \
    NVBOOT_ML_NAND_PAGE_SIZE_LOG2)

#define NVBOOT_ML_NAND_TRANSFER_PRTOCOL1_CONFIG 0x61
#define NVBOOT_ML_NAND_TRANSFER_PRTOCOL2_CONFIG 0x00
#define NVBOOT_ML_NAND_TRANSFER_PRTOCOL3_CONFIG 0x0F
#define NVBOOT_ML_NAND_MINIMUM_BUSY_TIME 0x1
#define NVBOOT_ML_NAND_SPARE_AREA_ECC_ERR_OFFSET 3
#define NVBOOT_ML_NAND_SPARE_AREA_ECC_ERR_MASK 0xFF000000
#define NVBOOT_ML_NAND_MDA_START_ADDRESS 0x200000
#define NVBOOT_ML_NAND_BOOT_TIME_IN_US 30000

/**
 * These defines hold the  fuse bits information.
 * Nand fuse bits are interpreted in the following way.
 */

/**
 * Bits 0,1 --> Used for Number of address cycles info. Values 00, 01, 10 and 11
 *      indicate address cycles 7, 4, 5 and 6 respectively.
 */
#define ML_NAND_DEVICE_CONFIG_0_ADDRESS_CYCLE_RANGE 1:0
/**
 * Bits 2,3 --> Used for Data width selection. Values 00, 01, 10, and 11 indicate
 *      Discovery, 8-bit, 16-bit and rsvd respectively.
 */
#define ML_NAND_DEVICE_CONFIG_0_DATA_WIDTH_RANGE 3:2
/**
 * Bit 4 --> Indicates the region(Sda/Mda) to read from. Values 0 and 1
 *      indicate Sda and Mda respectively.
 */
#define ML_NAND_DEVICE_CONFIG_0_REGION_SELECT_RANGE 4:4
/**
 * Bit 5 --> Indicates the pimux selection to use. Values 0 and 1
 *      indicate Primary and Secondary pinmuxes respectively.
 */
#define ML_NAND_DEVICE_CONFIG_0_PINMUX_SELECTION_RANGE 5:5
/**
 * Bit 6 --> Used to select pin order.
 *      Values 0, and 1 indicate Primary and Secondary pin ordering respectively.
 */
#define ML_NAND_DEVICE_CONFIG_0_PIN_ORDER_SELECTION_RANGE 6:6

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MOBILE_LBA_NAND_LOCAL_H */
