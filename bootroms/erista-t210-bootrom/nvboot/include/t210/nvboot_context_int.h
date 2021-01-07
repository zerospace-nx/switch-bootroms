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
 * nvboot_context_int.h - Definition of the context structure that the boot
 * code uses to keep track of information.  It is expected that there
 * is one global copy of this structure.
 */

#ifndef INCLUDED_NVBOOT_CONTEXT_INT_H
#define INCLUDED_NVBOOT_CONTEXT_INT_H

#include "nvboot_devmgr_int.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootContext: This is the structure used to manage global data used
 * during the boot process.
 */
typedef struct NvBootContextRec
{
    NvU8              *BootLoader; /* Start address of the boot loader. */

    NvBootDevMgr       DevMgr;

    NvBool             DecryptBootloader;
    NvBool             CheckBootloaderHash;
    NvU32              ChunkSize; /* Size of bootloader operations in bytes */

    NvBool             DecryptIsFirstBlock; /* True for processing 1st block */
    NvBool             HashIsFirstBlock; /* True for processing 1st block */

    NvBool             BlFailBack; /* Set from AO bit.  If true, failback to
                                    * older generation(s) of BLs.  If false,
                                    * failure to read the primary BL leads to
                                    * RCM.
                                    */
    NvBool             FactorySecureProvisioningMode;
    NvU8               ValidationKeySize; /* The AES key size to use during
                                             AES deecryption & CMAC hash
                                             calculation */
    NvU8               ProvisioningKeyNum; /* The Secure Provisioning key number to use
                                                to validate the BCT&BL */
    NvU8               DecryptionKeySlotNum; /* SE key slot number to use for
                                                decryption */
    NvU8               CMACHashKeySlotNum; /* SE key slot number to use for 
                                              CMAC hashing */
} NvBootContext;


#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CONTEXT_INT_H */
