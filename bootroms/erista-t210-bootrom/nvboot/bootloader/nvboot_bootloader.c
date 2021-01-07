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
 * nvboot_bootloader.c - Implementation of Boot Loader support.
 */

#include "nvboot_bootloader_int.h"
#include "nvboot_bit.h"
#include "nvboot_config.h"
#include "nvboot_context_int.h"
#include "nvboot_error.h"
#include "nvboot_fuse_int.h"
#include "nvboot_reader_int.h"
#include "nvboot_sdram_int.h"
#include "nvboot_se_int.h"
#include "nvboot_util_int.h"

#include "nvboot_se_defs.h"

#include "arse.h"

/* Global data */
extern NvBootInfoTable   BootInfoTable;
extern NvBootConfigTable BootConfigTable;

/* Storage for the bootloader object descriptions. */
static NvBootReaderObjDesc BlObjDesc[NVBOOT_MAX_BOOTLOADERS];

/* Function prototypes */
static NvBootError
LoadOneBootLoader(
    NvBootContext    *Context,
    NvBootLoaderInfo *Info,
    NvU32             NumCopies,
    NvU32             StartIndex);

static NvBool
IsValidBlDst(NvBootContext *Context, NvBootLoaderInfo *BlInfo);


/*
 * LoadOneBootLoader(): Read a single bootloader.
 *
 * @param Context The current context
 * @param Info    Boot loader information from the BCT
 * @param NumCopies The number of copies of the bootloader, each with its
 * own information structure.
 * 
 * @retval NvBootError_InvalidParameter NumCopies was too large
 * @retval TODO Errors from NvBootReadOneObject()
 *
 * Upon successful completion, the requested bootloader will be loaded into
 * its destination and validated.
 */
static NvBootError
LoadOneBootLoader(
    NvBootContext    *Context,
    NvBootLoaderInfo *Info,
    NvU32             NumCopies,
    NvU32             StartIndex)
{
    NvU32                   i;
    NvBootError             e;
    NvBootFuseOperatingMode Mode;

    NvBootFuseGetOperatingMode(&Mode);

    if (NumCopies > NVBOOT_MAX_BOOTLOADERS)
        return NvBootError_InvalidParameter;

    for (i = 0; i < NumCopies; i++)
    {
        BlObjDesc[i].BlIndex      = StartIndex + i;
        BlObjDesc[i].StartBlock   = Info[i].StartBlock;
        BlObjDesc[i].StartPage    = Info[i].StartPage;
        BlObjDesc[i].Length       = Info[i].Length;        
        /* SignatureOffset is the offset to the start of the signed section 
         * of the object */
        BlObjDesc[i].SignatureOffset = 0;
        BlObjDesc[i].SignatureRef = &(Info[i].Signature);
    }

    /* Read BL and verify it using either RSA-PSS or AES-CMAC */    
    if(Mode == NvBootFuseOperatingMode_OdmProductionSecurePKC)
    {
        /* 
         * Read the whole BL without doing any verification. 
         * Context->CheckBootloaderHash = NV_FALSE;
         * Context->DecryptBootloader = NV_FALSE;
         */
        NV_BOOT_CHECK_ERROR(NvBootReadOneObject(Context,
                                    (NvU8*)(Info[0].LoadAddress),
                                    BlObjDesc,
                                    NumCopies,
                                    &(BootConfigTable.BadBlockTable)));

        
        /* 
         * Public key assumed to already be loaded into slot 1. 
         * Call RSASSA-PSS VERIFY function to verify the RSASSA-PSS
         * signature of the BL.
         */
        NV_BOOT_CHECK_ERROR(NvBootSeRsaPssSignatureVerify(SE_RSA_KEY_PKT_KEY_SLOT_ONE, 
                                    ARSE_RSA_MAX_EXPONENT_SIZE, 
                                    (NvU32 *) Info->LoadAddress,
                                    NULL,
                                    Info->Length,
                                    (NvU32 *) &(Info->Signature.RsaPssSig.Signature[0]),
                                    SE_MODE_PKT_SHAMODE_SHA256, 
                                    ARSE_SHA256_HASH_SIZE / 8));

    }
    else
    {
        NV_BOOT_CHECK_ERROR(NvBootReadOneObject(Context,
                                    (NvU8*)(Info[0].LoadAddress),
                                    BlObjDesc,
                                    NumCopies,
                                    &(BootConfigTable.BadBlockTable)));
    }
    return e;
}

/**
 * IsValidBlDst(): Checks to see if the destination of loading the bootloader,
 * as specified by BlInfo, fits into an appropriate memory region.
 *
 * @param Context The current context
 * @param BlInfo  A description of the bootloader to load
 *
 * @retval NV_TRUE The destination of the BL is valid and the entry point
 * is within the destination.
 * @retval NV_FALSE The destination of the BL is invalid.
 */
static NvBool
IsValidBlDst(NvBootContext *Context, NvBootLoaderInfo *BlInfo)
{
    NvU32 BlStart;
    NvU32 BlEnd;
    NvU32 MemStart; /* Address of the start  of a legal memory range */
    NvU32 MemEnd;   /* Address following end of a legal memory range */

    NV_ASSERT(Context != NULL);
    NV_ASSERT(BlInfo  != NULL);

    BlStart = BlInfo->LoadAddress;
    BlEnd   = BlStart + BlInfo->Length;

    /* Check the destination address range. */

    /*
     * Is it iRAM? The legal range is any where on iRAM pages B, C, and D.
     * iRAM page A is reserved for the boot rom.
     *
     * Note: The +1 on the MemEnd calculation advances the end address to the
     *       byte following the end of the range (for consistency with the
     *       other calculations).
     */
    MemStart = NVBOOT_BL_IRAM_START;
    MemEnd   = NVBOOT_BL_IRAM_END;

    if ((BlStart >= MemStart) && (BlEnd <= MemEnd))
        return NV_TRUE;

    /*
     * Is it SDRAM? Only check if SDRAM was initialized.
     *
     * For the BootROM, the only place that SDRAM can appear is at 
     * EMEM_BASE.
     */
    if (BootInfoTable.SdramInitialized)
    {
        MemStart = NVBOOT_BL_SDRAM_START;
        MemEnd   = MemStart + NvBootSdramQueryTotalSize();

        if ((BlStart >= MemStart) && (BlEnd <= MemEnd))
            return NV_TRUE;
    }

    /*
     * Reaching this point indicates that no legal destination address
     * range was found for the BL.
     */
    return NV_FALSE;
}

/**
 * NvBootLoadBootLoader(): Attempt to load a boot loader.
 *
 * @param[in] Context The current context
 * @param[out] BootRomBranchTarget The address at which to start running the
 * bootloader.  This should be an appropriate entry point.
 *
 * @retval NvBootError_Success A bootloader was successfully loaded.
 * @retval TODO Error codes from LoadOneBootLoader
 * @retval NvBootError_BootLoaderLoadFailure No boot loader could be loaded
 * successfully.
 * @retval NvBootError_InvalidBlDst The bootloader didn't fit in a valid
 * memory space or the entry point was outside the BL.
 *
 * The search algorithm for valid bootloaders:
 *      * Start with the bootloader described in entry 0 of the BCT BL table.
 *      * Try to load this bootloader.  Redundant copies are identified as
 *        bootloaders in adjacent BCT BL table entries with the same version
 *        number.
 *      * Repeat for other bootloaders described in the BCT in consecutive
 *        positions in the BCT BL table.  Higher valued indices denote older
 *        bootloaders.
 * It is assumed that the code that generated the BCT BL table understands
 * this algorithm and provided reasonable data.  Incorrect data can lead
 * to the wrong bootloader being loaded.
 */
NvBootError
NvBootLoadBootLoader(NvBootContext *Context, NvU32 *BootRomBranchTarget)
{
    NvU32       BootLoaderIndex;
    NvU32       NumCopies;
    NvU32       MaxBootLoader;
    NvU32       PrimaryBlVersion;
    NvBootError Error = NvBootError_Success;
    NvBootLoaderInfo *BlInfo = NULL;

    PrimaryBlVersion = BootConfigTable.BootLoader[0].Version;

    // Initially assume FailBack is not used.  Override value later.
    BootInfoTable.InvokedFailBack = NV_FALSE;

    /*
     * Loop over available boot loaders and try to load each one
     * until successful.
     */
    BootLoaderIndex = 0;
    MaxBootLoader = BootConfigTable.BootLoadersUsed;
    while (BootLoaderIndex < MaxBootLoader)
    {
	/* Attempt to load the next boot loader. */
	BlInfo = &(BootConfigTable.BootLoader[BootLoaderIndex]);

        /*
         * Check the version of this bootloader. If it a different generation
         * than the primary BL and the failover bit was not set, break
         * from the loop as BL loading has failed.
         */
        if (BlInfo->Version != PrimaryBlVersion)
        {
            if(!Context->BlFailBack)
            {
                // Stop attempting to load bootloaders.
                break;
            }
            else
            {
                // Record that failback was used.
                BootInfoTable.InvokedFailBack = NV_TRUE;
            }
        }

        /* Validate the BL destination */
        if (!IsValidBlDst(Context, BlInfo))
            return NvBootError_InvalidBlDst;

	/* 
	 * Determine how many copies there are of the boot loader that is
	 * being loaded.
	 */
	NumCopies = 1;
	while (BootLoaderIndex + NumCopies < MaxBootLoader &&
	       BlInfo[NumCopies].Version == BlInfo->Version)
	    NumCopies++;

	Error = LoadOneBootLoader(Context, BlInfo, NumCopies, BootLoaderIndex);
        switch (Error)
        {
            case NvBootError_Success:
                // also handles NvBootError_SE_Signature_Valid
                BootInfoTable.BlState[BootLoaderIndex].Status =
                    NvBootRdrStatus_Success;

                *BootRomBranchTarget = BlInfo[0].EntryPoint;
                return NvBootError_Success;

            case NvBootError_HashMismatch:
            case NvBootError_ValidationFailure:
                // TODO: make sure all possible error codes from RSA-PSS verify handled here
            case NvBootError_SE_RsaPssVerify_Inconsistent:
                BootInfoTable.BlState[BootLoaderIndex].Status =
                    NvBootRdrStatus_ValidationFailure;
                break;

            case NvBootError_DeviceReadError:
            case NvBootError_HwTimeOut:
                BootInfoTable.BlState[BootLoaderIndex].Status =
                    NvBootRdrStatus_DeviceReadError;
                break;

            default:
                NV_ASSERT(0);
        }

	BootLoaderIndex++;
    }
    
    /* This point is reached if no boot loader could be successfully loaded. */
    Error = NvBootError_BootLoaderLoadFailure;

    return Error;
}
