/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
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
 * NvBootFlowStatus: Status & data for secure exit context.
 */
typedef struct NvBootFlowStatusContextRec
{
    /* Status Info */
    NvBool	NvBootFlowXusbForceRcm;
    NvBool	NvBootFlowSc7Exit;
    NvBool	NvBootFlowChipStatus;
    NvBool	NvBootPreserveDram;
    NvBool	NvBootRamDumpEnabled;
} NvBootFlowStatusContext;

/*
 * NvBootContext: This is the structure used to manage global data used
 * during the boot process.
 */
typedef struct NvBootContextRec
{
    uint8_t              *BootLoader; /* Start address of the boot loader. */

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
    NvBool		       FactorySecureProvisioningMode;
    uint8_t               ValidationKeySize; /* The AES key size to use during
    					                     AES deecryption & CMAC hash
    					                     calculation */
    uint8_t               ProvisioningKeyNum; /* The Secure Provisioning key number to use
    						to validate the BCT&BL */
    uint8_t               DecryptionKeySlotNum; /* SE key slot number to use for
    						decryption */
    uint8_t		       CMACHashKeySlotNum; /* SE key slot number to use for
    					  CMAC hashing */

    // A flag to tell the exit code whether or not to copy the NV public keys
    // to SYSRAM. For "Download and execute MB1" keys are required. For "download and execute"
    // they are not.
    NvBool             Rcm_CopyKeysToSysram;

    NvBootFlowStatusContext BootFlowStatus;//__attribute__((aligned(4)));

    // SE sticky bits to be restored in context restore stage 3.
    uint32_t SeSecuritySe1;
    uint32_t SeSecuritySe2;
    uint32_t SeTzramSecurity;
    uint32_t Pka1Security;
} NvBootContext;


#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CONTEXT_INT_H */
