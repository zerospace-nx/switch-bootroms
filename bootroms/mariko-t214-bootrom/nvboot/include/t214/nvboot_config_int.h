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
 * nvboot_config_int.h - Definition of constants that configure aspects of the
 * BootRom implementation.  The elements in this file are those not needed
 * by bootloaders or any other external utilities.
 */

#ifndef INCLUDED_NVBOOT_CONFIG_INT_H
#define INCLUDED_NVBOOT_CONFIG_INT_H

#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * Selection of engines & key slots for AES operations.
 */
#define NVBOOT_DECRYPT_ENGINE NVBOOT_SBK_ENGINEA
#define NVBOOT_DECRYPT_SLOT   NVBOOT_SBK_DECRYPT_SLOT
#define NVBOOT_HASH_ENGINE    NVBOOT_SBK_ENGINEB
#define NVBOOT_HASH_SLOT      NVBOOT_SBK_DECRYPT_SLOT

/*
 * Selection of engines & key slots for WB0 AES operations.
 */
#define NVBOOT_WB0_HASH_ENGINE    NVBOOT_SBK_ENGINEB
#define NVBOOT_WB0_HASH_SLOT      NVBOOT_SBK_DECRYPT_SLOT


/*
 * Configuration data for the object reader.
 */

/*
 * The number of buffers for the object reader must be at least 2.
 * This provides for one buffer each for device read destination and
 * AES decryption source.
 * The size of a single buffer must be the greater of the max page size and
 * the requirements for USB, both of which are 4096 bytes.
 */
#define NVBOOT_READER_NUM_BUFFERS 2
#define NVBOOT_BUFFER_LENGTH      NVBOOT_MAX_PAGE_SIZE
#define NVBOOT_MAX_BUFFER_SIZE \
  (NVBOOT_BUFFER_LENGTH * NVBOOT_READER_NUM_BUFFERS)

#define NVBOOT_DEFAULT_BOOT_DEVICE NvBootFuseBootDevice_Sdmmc;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CONFIG_INT_H */
