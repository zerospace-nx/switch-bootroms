/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/*
 * nvboot_reader.c - Implementation of object reading, decrypting, and
 *     hash checking.
 */

#include "nvcommon.h"
#include "arse.h"
#include "nvboot_se_aes.h"
#include "nvboot_badblocks_int.h"
#include "nvboot_bit.h"
#include "nvboot_bootloader_int.h"
#include "nvboot_buffers_int.h"
#include "nvboot_config.h"
#include "nvboot_config_int.h"
#include "nvboot_context_int.h"
#include "nvboot_error.h"
#include "nvboot_hash_int.h"
#include "nvboot_reader_int.h"
#include "nvboot_se_int.h"
#include "nvboot_util_int.h"
#include "nvboot_irom_patch_int.h"

// Ensure that no new status codes have been added.
// Anytime a code is added, re-check the logic herein, esp. in
// UpdateDevStatus().
NV_CT_ASSERT(NvBootDeviceStatus_Max == 9);

extern NvBootInfoTable BootInfoTable;

/*
 * NvReaderState: Internal state of the the object reader.
 */
typedef struct NvBootReaderStateRec
{
    NvBootDevMgr        *DevMgr;        /* Pointer to the device manager  */
    NvBootBadBlockTable *BadBlockTable; /* Pointer to the bad block table */

    NvU32  ChunkSize;
    NvU32  ChunkSizeLog2;

    NvU32  ChunksRemainingForDev;
    NvU32  ChunksRemainingForAes;

    /* Count of how many unsigned chunks (page size) to skip before applying
     * offset */
    NvU32  UnsignedChunkCount;

    /* Source information */
    NvBootReaderObjDesc *ObjDesc;
    NvU32                NumCopies;

    /* Current read information */
    NvU32  ActiveCopy; /* Tracks the copy being read from */
    NvU32  DevReadBlock;
    NvU32  DevReadPage;
    NvU32  DevLogicalPage;
    NvU32  RedundantRdBlock;
    NvU32  RedundantRdPage;

    /* Unit idle status */
    NvBool AesIsIdle;
    NvBool DevIsIdle;

    /* Buffer information */
    NvU8   BuffersForDev;
    NvU8   BuffersForAes;
    NvU8   DevDstBuf;
    NvU8   AesSrcBuf;
    NvBootAes128Iv HashK1; /* IV buffer used by the hash code. */
    NvBootAes128Iv HashK2; /* IV buffer used by the hash code. */

    /* Destinations */
    NvU8  *ObjDst;
    NvBootObjectSignature SignatureDst;

    /* Flags to trigger special work w/first chunk. */
    NvBool IsFirstChunkForDevice;
    NvBool IsFirstChunkForDecrypt;
    NvBool IsFirstChunkForHash;

    /* Crypto flags */
    NvBool DecryptObject;
    NvBool HashObject;

    NvU8   AesOpsKeySize; /* Key size is variable between 128, 192, 256 bits */
    NvU8   SEKeySlotForAesCrypto;
    NvU8   SEKeySlotForAesCMAC;
} NvBootReaderState;

/*
 * Declaration of buffer memory.  These must not be static, as they are
 * referenced outside this file.
 * Put into its own section to make sure that the buffer is not zero
 * initialized by the scatter loading from the library
 */
#pragma arm section zidata = "NonZIBuffer"
NV_ALIGN(4096) NvU8
BufferMemory[NVBOOT_MAX_BUFFER_SIZE];
// restoring the section mapping
#pragma arm section

NvU8* Buffer[2] = { &BufferMemory[0], 
                 &BufferMemory[(NVBOOT_MAX_BUFFER_SIZE / NVBOOT_READER_NUM_BUFFERS)] };

/*
 * Statically allocated reader state.  This assumes only one reader is active
 * at a time, which is true for the BootROM.
 */
static NvBootReaderState ReaderState;

NvU8 *g_ReaderBufferMem = NULL;

#define BUFFER_ADDR(index) (g_ReaderBufferMem + ((index) << State->ChunkSizeLog2))

/*
 * Increment a buffer index into a circular buffer.
 */
#define BUFFER_INCR(idx, count) (((idx) == ((count) - 1)) ? 0 : ((idx) + 1))


/*
 * Function prototypes
 */

static NvBool
IsValidPadding(NvU8 *Data, NvU32 Length);

static void
InitReaderState(
    NvBootReaderState   *State,
    NvBootReaderObjDesc *ObjDesc,
    NvU32                NumCopies,
    NvU8                *ReadDst,
    NvBootContext       *Context);

static NvBootError LaunchDevRead       (NvBootReaderState *State);
static NvBootError UpdateDevStatus     (NvBootReaderState *State);
static void        BlockForDevFinish   (NvBootReaderState *State);

static void        LaunchCryptoOps     (NvBootReaderState *State);
static void        UpdateCryptoStatus  (NvBootReaderState *State);
static void        BlockForCryptoFinish(NvBootReaderState *State);

static NvBool      CheckCryptoHash(NvBootHash *Hash1, NvBootHash *Hash2);


/* Verify that any padding holds the pattern 0x80 0x00 0x00 ... */
static NvBool
IsValidPadding(NvU8 *Data, NvU32 Length)
{
    NvU32        AesBlocks;
    NvU32        Remaining;
    NvU8        *p;
    NvU8         RefVal;

    AesBlocks = NV_ICEIL_LOG2(Length, NVBOOT_AES_BLOCK_LENGTH_LOG2);
    Remaining = (AesBlocks << NVBOOT_AES_BLOCK_LENGTH_LOG2) - Length;

    p = Data + Length;
    RefVal = 0x80;

    while (Remaining)
    {
        if (*p != RefVal) return NV_FALSE;
        p++;
        Remaining--;
        RefVal = 0x00;
    }

    return NV_TRUE;
}

/**
 * InitReaderState() Initialize the reader state.
 *
 * @param State BootReaderState
 * @param ObjDesc Pointer to the array of object descriptions
 * @param NumCopies Number of copies of the object to be loaded.
 * @param ReadDst Pointer to where the processed object should be written
 * @param Context Pointer to the current context
 */
static void
InitReaderState(NvBootReaderState   *State,
                NvBootReaderObjDesc *ObjDesc,
                NvU32                NumCopies,
                NvU8                *ReadDst,
                NvBootContext       *Context)
{
    NvBootError bufferAssignError = NvBootError_NotInitialized;

    NV_ASSERT(State   != NULL);
    NV_ASSERT(ObjDesc != NULL);
    NV_ASSERT(ReadDst != NULL);
    NV_ASSERT(Context != NULL);

    State->DevMgr = &(Context->DevMgr);

    /*
     * Initialize the bad block table pointer to NULL.  It will be overridden
     * by NvBootReadOneObject.
     */
    State->BadBlockTable = NULL;

    State->ChunkSizeLog2 = State->DevMgr->PageSizeLog2;
    State->ChunkSize     = (1 << State->ChunkSizeLog2);

    State->ChunksRemainingForDev = NV_ICEIL_LOG2(ObjDesc->Length,
                                                 State->ChunkSizeLog2);
    State->ChunksRemainingForAes = State->ChunksRemainingForDev;

    State->ObjDesc   = ObjDesc;
    State->NumCopies = NumCopies;

    State->UnsignedChunkCount = State->ObjDesc[0].SignatureOffset >>
            State->ChunkSizeLog2;

    /* Setup the reading of the first chunk. */
    State->ActiveCopy      = 0;
    State->DevReadBlock    = ObjDesc->StartBlock;
    State->DevReadPage     = ObjDesc->StartPage;
    State->DevLogicalPage  = 0;

    /* Mark the units idle at the start. */
    State->DevIsIdle = NV_TRUE;
    State->AesIsIdle = NV_TRUE;

    /*
     * Initially assign all but 1 of the buffers to the reading device.
     * The extra buffer is used as the hash destination.
     */
    State->BuffersForDev = NVBOOT_READER_NUM_BUFFERS - 1;
    State->BuffersForAes = 0;

    /*
     * Initialize each component so that when they increment their
     * buffer index w/their first work, they'll use the correct buffer.
     */
    State->DevDstBuf     = 0; /* Will be 1 when incremented. */
    State->AesSrcBuf     = 0; /* Will be 1 when incremented. */

    /* Set the destination pointers. */
    State->ObjDst         = ReadDst;

    /*
     * Set the flags indicating first chunk for all units.
     */
    State->IsFirstChunkForDevice  = NV_TRUE;

    /* Crypto flags */
    /**** TODO: Update the following calculations. ****/
    State->DecryptObject = Context->DecryptBootloader;
    State->HashObject    = Context->CheckBootloaderHash;
#ifdef TARGET_ASIM
    State->DecryptObject = NV_FALSE;
    State->HashObject    = NV_FALSE;
#endif

    State->AesOpsKeySize = Context->ValidationKeySize;

    if(Context->FactorySecureProvisioningMode == NV_TRUE)
    {
        State->SEKeySlotForAesCrypto = NvBootSeAesKeySlot_Secure_Provisioning_Key_Decrypt;
        State->SEKeySlotForAesCMAC = NvBootSeAesKeySlot_Secure_Provisioning_Key_CMAC_Hash;
    }
    else
    {
        State->SEKeySlotForAesCrypto = NvBootSeAesKeySlot_SBK_AES_Decrypt;
        State->SEKeySlotForAesCMAC = NvBootSeAesKeySlot_SBK_AES_CMAC_Hash;
    }

    /*
     * If there is no UnsignedChunkCount to account for, indicate
     * that this is the first chunk for decrypt/hash right away.
     */
    if(State->UnsignedChunkCount == 0)
    {
        State->IsFirstChunkForDecrypt = (State->DecryptObject)? NV_TRUE : NV_FALSE;
        State->IsFirstChunkForHash = (State->HashObject)? NV_TRUE : NV_FALSE;
    }
    else
    {
        State->IsFirstChunkForDecrypt = NV_FALSE;
        State->IsFirstChunkForHash    = NV_FALSE;
    }

    g_ReaderBufferMem = NULL;
    bufferAssignError = State->DevMgr->Callbacks->GetReaderBuffersBase(&g_ReaderBufferMem,
                 4096, NVBOOT_MAX_BUFFER_SIZE);
    if(bufferAssignError != NvBootError_Success)
    {
        g_ReaderBufferMem = &BufferMemory[0];
    }


}

/**
 * MapLogicalPage(): Determine the actual location (block & page) of a
 *     page of an object based on the table of known bad blocks.
 *
 * @param State The reader state structure
 *
 * The logical page represents the page number of an object ignoring block
 * boundaries and bad blocks, starting at 0.
 *
 * MapLogicalPage() uses the logical page, the starting block & page numbers
 * for an object instance, and the bad block table to compute the actual
 * block and page numbers where the logical page is stored for this instance.
 *
 * MapLogicalPage() is used after a read failure to determine where to locate
 * the same page in a redundant copy of the object (which must be a
 * bootloader).
 *
 * PagesRemaining stores the number of pages residing in known good blocks
 * that stand between the current point in the search and the desired
 * page (the number of logical pages between here and the target).
 */
static void
MapLogicalPage(NvBootReaderState *State)
{
    NvU32                PagesPerBlock;
    NvU32                PagesRemaining;
    NvBootReaderObjDesc *Obj;

    NV_ASSERT(State != NULL);

    PagesPerBlock  = 1 << (State->DevMgr->BlockSizeLog2 -
                           State->DevMgr->PageSizeLog2);
    PagesRemaining = State->DevLogicalPage;
    Obj = &(State->ObjDesc[State->ActiveCopy]);

    /* Start the search with the first block. */
    State->RedundantRdBlock = Obj->StartBlock;

    /*
     * First, check to see if the logical page lands in the first
     * block of the redundant copy.
     */
    if (PagesRemaining < (PagesPerBlock - Obj->StartPage))
    {
        /*
         * The logical page landed in the first block.
         * It is safe to assume that the first block is a known good block.
         * Compute the new read location and return.
         * Note that State->RedunantRdBlock is set above.
         */
        State->RedundantRdPage  = Obj->StartPage + PagesRemaining;
        return;
    }

    /*
     * The replacement page did not land in the first block.
     * Skip over the pages in the first block and commence searching
     * through the remaining blocks for the proper replacement.
     */
    PagesRemaining -= (PagesPerBlock - Obj->StartPage);

    /*
     * The final page will land at PagesRemaining % PagesPerBlock.
     * As PagesPerBlock is a power of 2, the following eliminates the
     * divide implied by the remainder operation.
     */
    State->RedundantRdPage = PagesRemaining & (PagesPerBlock-1);

    /*
     * Find the correct block, skipping bad ones, until the number of
     * pages remaining fits within a block.  This indicates that the
     * replacement page lies in the current good block number stored in
     * State->RedundantRdBlock.
     *
     * The page is not in the first block, so the search commences with
     * its successor.
     */
    while (1)
    {
        /* Advance to the next block. */
        State->RedundantRdBlock++;

        if (State->BadBlockTable &&
            NvBootIsBadBlock(State->RedundantRdBlock, State->BadBlockTable))
        {
            /*
             * The block is known to be bad, so don't advance the page count
             * because of this block.
             */
            continue;
        }
        else
        {
            /* The block is good, so advance over the pages in the block. */
            PagesRemaining -= PagesPerBlock;
        }

        /* Check for loop termination, but _only_ after seeing a good block. */
        if (PagesRemaining > PagesPerBlock)
        {
            return;
        }
    }
}

/**
 * LaunchDevRead(): Initiate the read of a page from a device.
 *
 * @param State The reader state structure.
 *
 * @retval TODO Errors from ReadPageCallback()
 *
 * Note that LaunchDevRead() always attempts to read from the primary copy
 * of a bootloader.  Reading from a redundant copy is launched from
 * UpdateDevStatus upon detection of a read failure w/the primary copy.
 */
static NvBootError LaunchDevRead(NvBootReaderState *State)
{
    NvBootError  e;
    NvU8        *Dst;
    NvU32        PagesPerBlock;

    NV_ASSERT(State != NULL);

    State->ActiveCopy = 0; /* Always start from the primary copy */
    State->DevIsIdle = NV_FALSE;
    State->ChunksRemainingForDev--;
    State->BuffersForDev--;
    State->DevDstBuf = BUFFER_INCR(State->DevDstBuf,
                                   NVBOOT_READER_NUM_BUFFERS);

    Dst = BUFFER_ADDR(State->DevDstBuf);
    
    PagesPerBlock = 1 << (State->DevMgr->BlockSizeLog2 -
                          State->DevMgr->PageSizeLog2);

    if (State->IsFirstChunkForDevice == NV_TRUE)
    {
        /*
         * Skip the incrementing of the pages & blocks.
         * Also, the first page should land on a known good
         * block.
         */
        State->IsFirstChunkForDevice = NV_FALSE;
    }
    else
    {
	State->DevLogicalPage++;
	State->DevReadPage++;

	if (State->DevReadPage >= PagesPerBlock)
	{
	    /* Advance to the next known good block. */
	    State->DevReadPage = 0;
            State->DevReadBlock++;
        }
    }

    /* Skip past known bad blocks. */
    if (State->BadBlockTable)
    {
        while (NvBootIsBadBlock(State->DevReadBlock, State->BadBlockTable))
        {
            State->DevReadBlock++;
        }
    }

    /* Initiate the page read. */
    e = State->DevMgr->Callbacks->ReadPage(State->DevReadBlock,
                                           State->DevReadPage,
                                           Dst);

    return e;
}

/**
 * UpdateDevStatus(): Check on the status of the outstanding read request and
 * take any actions needed.
 *
 * @param State The reader state structure.
 *
 * @retval NvBootError_Success No error was encountered.
 * @retval NvBootError_DeviceReadError An unrecoverable read error occured.
 * @retval TODO Error codes from the ReadPage() callbacks.
 */
static NvBootError UpdateDevStatus(NvBootReaderState *State)
{
    NvBootError         e;
    NvBootDeviceStatus  Status;
    NvU8               *Dst;
    NvS32               BlIndex;
    NvU32               Block;
    NvU32               Page;

    NV_ASSERT(State != NULL);

    if (State->IsFirstChunkForDevice)
    {
        /* Skip querying the device and simply return success. */
        return NvBootError_Success;
    }

    Status  = State->DevMgr->Callbacks->QueryStatus();
    BlIndex = State->ObjDesc[State->ActiveCopy].BlIndex;

    /*
     * Identify the block & page. ActiveCopy == 0 implies primary
     * copy, otherwise redundant copy.
     */
    if (State->ActiveCopy == 0)
    {
        Block = State->DevReadBlock;
        Page  = State->DevReadPage;
    }
    else
    {
        Block = State->RedundantRdBlock;
        Page  = State->RedundantRdPage;
    }

    /*
     * Record the first corrected ECC error for this BL.  BlIndex >= 0 implies
     * reading a BL.  BCTs have BlIndex < 0.
     */
    if ((Status == NvBootDeviceStatus_CorrectedEccFailure) &&
        (BlIndex >= 0) && !BootInfoTable.BlState[BlIndex].HadCorrectedEccError)
    {
        // Record the first corrected ECC error.
        BootInfoTable.BlState[BlIndex].HadCorrectedEccError   = NV_TRUE;
        BootInfoTable.BlState[BlIndex].FirstCorrectedEccBlock = Block;
        BootInfoTable.BlState[BlIndex].FirstCorrectedEccPage  = Page;
    }

    if (Status == NvBootDeviceStatus_CrcFailure)
    {
        BootInfoTable.BlState[BlIndex].HadCrcError = NV_TRUE;
    }

    // For read failures, attempt to read from backup copies.
    if ((Status == NvBootDeviceStatus_ReadFailure) ||
        (Status == NvBootDeviceStatus_EccFailure ) ||
        (Status == NvBootDeviceStatus_CrcFailure ) ||
        (Status == NvBootDeviceStatus_DataTimeout))
    {
        /*
         * Record the first read failures for this BL.  BlIndex >= 0 implies
         * reading a BL.  BCTs have BlIndex < 0.
         */
        if ((BlIndex >= 0) && !BootInfoTable.BlState[BlIndex].HadEccError)
        {        
            // Record the first uncorrectable  error.
            BootInfoTable.BlState[BlIndex].HadEccError   = NV_TRUE;
            BootInfoTable.BlState[BlIndex].FirstEccBlock = Block;
            BootInfoTable.BlState[BlIndex].FirstEccPage  = Page;
        }

        /* Attempt to read from the next copy. */
        State->ActiveCopy++;

        if (State->ActiveCopy < State->NumCopies)
        {
            /* Mark this copy as used for Ecc recovery. */
            BlIndex = State->ObjDesc[State->ActiveCopy].BlIndex;

            /* This should only be reached for BLs which have indices >= 0 */
            NV_ASSERT(BlIndex >= 0);

            BootInfoTable.BlState[BlIndex].UsedForEccRecovery = NV_TRUE;

            /*
             * Map the current logical page to a block & page in the redundant
             * copy.
             */
            MapLogicalPage(State);

            /* Start the read. */
            Dst = BUFFER_ADDR(State->DevDstBuf);
    
            NV_BOOT_CHECK_ERROR(State->DevMgr->Callbacks->ReadPage(
                State->RedundantRdBlock,
                State->RedundantRdPage,
                Dst));

            Status = NvBootDeviceStatus_ReadInProgress;
        }
    }

    /* Error status at this point indicates that no recovery is possible. */
    if ((Status == NvBootDeviceStatus_ReadFailure) ||
        (Status == NvBootDeviceStatus_EccFailure ) ||
        (Status == NvBootDeviceStatus_CrcFailure ) ||
        (Status == NvBootDeviceStatus_DataTimeout))
    {
        return NvBootError_DeviceReadError;
    }
   
    /* Check if the device finished reading a chunk. */	
    if (((Status == NvBootDeviceStatus_Idle                ) ||
         (Status == NvBootDeviceStatus_CorrectedEccFailure ) ||
         (Status == NvBootDeviceStatus_CorrectedReadFailure)) &&
        (State->DevIsIdle == NV_FALSE))
    {
        /* The device finished work since last we checked. */
        State->DevIsIdle = NV_TRUE;
        State->BuffersForAes++;
    }

    return NvBootError_Success;
}

/**
 * BlockForDevFinish(): Wait for any pending device read to complete.
 *
 * @param State Cuurent reader state.
 *
 * Used to flush pending work when an error is encountered.
 */
static void BlockForDevFinish(NvBootReaderState *State)
{
    NvBootDeviceStatus Status;

    NV_ASSERT(State != NULL);

    do
    {
        Status = State->DevMgr->Callbacks->QueryStatus();
    } while (Status == NvBootDeviceStatus_ReadInProgress);
}

/**
 * LaunchCryptoOps(): Initiate cryptography operations for a buffer.
 *
 * @param State Current reader state
 */
static void
LaunchCryptoOps(NvBootReaderState *State)
{
    NvU32  NumBlocks;
    NvU8  *Src;
    NvBool First;
    NvBool Last;
    NvU32  Offset; /* Byte offset to usable data. */

    NV_ASSERT(State != NULL);

    /* Update Aes state & buffers */
    State->AesIsIdle = NV_FALSE;
    State->BuffersForAes--;
    State->ChunksRemainingForAes--;
    State->AesSrcBuf = BUFFER_INCR(State->AesSrcBuf,
                                   NVBOOT_READER_NUM_BUFFERS);

    /* TODO: Replace with a single first flag.*/
    First = State->IsFirstChunkForDecrypt || State->IsFirstChunkForHash;

    /*
     * If this is the first block to launch hash/crypto operations on,
     * adjust for any offset (less than AES block size) into the signed
     * section if any.
     *
     * BCT has an unsigned section which needs to be skipped for hashing
     * and decrypt operations. Note, if the SignatureOffset value is
     * larger than the chunk size, we will need to skip over multiple
     * chunks first before applying any final offset.
     *
     * Bootloader does not have any unsigned section to skip.
     *
     */
    if (First)
    {
        // Offset = SignatureOffset % ChunkSize;
        Offset =  State->ObjDesc[0].SignatureOffset -
            ((State->ObjDesc[0].SignatureOffset >> State->ChunkSizeLog2) <<
            State->ChunkSizeLog2);
    }
    else
    {
        Offset = 0;
    }

    if (State->ChunksRemainingForAes == 0)
    {
        NvU32 Remaining;

        /* This is the final chunk, so only read the number of blocks
         * necessary.
         */
        /* Remaining = Length % ChunkSize; */
        Remaining = State->ObjDesc[0].Length -
            ((State->ObjDesc[0].Length >> State->ChunkSizeLog2) <<
             State->ChunkSizeLog2);
        NumBlocks = NV_ICEIL_LOG2(Remaining, NVBOOT_SE_AES_BLOCK_LENGTH_LOG2);

        if (Remaining == 0)
        {
            //number of remaining bytes is multiple of a page size
            NumBlocks = 1 << (State->ChunkSizeLog2 - NVBOOT_SE_AES_BLOCK_LENGTH_LOG2);
        }
    }
    else
    {
        NumBlocks = 1 << (State->ChunkSizeLog2 - NVBOOT_SE_AES_BLOCK_LENGTH_LOG2);
    }

    Src = BUFFER_ADDR(State->AesSrcBuf);

    NumBlocks -= Offset >> NVBOOT_SE_AES_BLOCK_LENGTH_LOG2;

    if (State->UnsignedChunkCount)
    {
        /*
         * Copy the whole skipped chunk to the destination
         */
        memcpy(State->ObjDst, Src, State->ChunkSize);

        State->UnsignedChunkCount--;

        if (State->UnsignedChunkCount == 0)
        {
            State->IsFirstChunkForDecrypt = (State->DecryptObject)? NV_TRUE : NV_FALSE;
            State->IsFirstChunkForHash = (State->HashObject)? NV_TRUE : NV_FALSE;
        }
    }
    else
    {
        /*
         * Copy the skipped Offset to the destination.
         * This happens for BCTs, which is when SignatureOffset > 0.
         */
        memcpy(State->ObjDst, Src, Offset);

        if (State->DecryptObject == NV_TRUE)
        {
            // Decrypt chunk using SE engine.
            NvBootSeAesDecrypt(State->SEKeySlotForAesCrypto,
                              State->AesOpsKeySize,
                              State->IsFirstChunkForDecrypt,
                              NumBlocks,
                              Src + Offset,
                              State->ObjDst + Offset);

            State->IsFirstChunkForDecrypt = NV_FALSE;
        }
        else
        {
            /* Copy the bootloader data to memory. */
            memcpy(State->ObjDst + Offset,
                   Src + Offset,
                   NumBlocks << NVBOOT_SE_AES_BLOCK_LENGTH_LOG2);

            State->IsFirstChunkForDecrypt = NV_FALSE;
        }

        if (State->HashObject == NV_TRUE)
        {
            if (First)
            {
                NvBootSeAesCmacGenerateSubkey(State->SEKeySlotForAesCMAC,
                        State->AesOpsKeySize,
                        (NvU32 *) &(State->HashK1),
                        (NvU32 *) &(State->HashK2));

                State->IsFirstChunkForHash = NV_FALSE;
            }

            Src += Offset;

            Last = (State->ChunksRemainingForAes == 0);

            NvBootSeAesCmacHashBlocks((NvU32 *) &(State->HashK1),
                                      (NvU32 *) &(State->HashK2),
                                      (NvU32 *) Src,
                                      (NvU8 *) &(State->SignatureDst.CryptoHash),
                                      State->SEKeySlotForAesCMAC,
                                      State->AesOpsKeySize,
                                      NumBlocks,
                                      First,
                                      Last);

            /* Spin wait for it to finish. */
            while(NvBootSeIsEngineBusy())
                ;
        }
    }
}

/**
 * UpdateCryptoStatus(): Update the state of cryptographic processing.
 *
 * @param State Current reader state
 */
static void UpdateCryptoStatus(NvBootReaderState *State)
{
    /*
     * When not decrypting, consider the status to be Idle to ensure
     * the correct path through the logic that follows.
     */
    NvBool DecryptIsBusy = NV_FALSE;
    NvBool HashIsBusy    = NV_FALSE;

    NV_ASSERT(State != NULL);

    /* TODO: Revisit this code when using the hash implementation. */
    /* TODO: Revisit this code after the aes update has happened.
     *       It may be safe to unconditionally perform the two busy queries
     *       below.
     */

    if (State->DecryptObject == NV_TRUE)
    {
        DecryptIsBusy = NvBootSeIsEngineBusy();
    }

    if (State->HashObject == NV_TRUE)
    {
        HashIsBusy = NvBootSeIsEngineBusy();
    }

    /*
     * Update buffer state if newly idle.
     */
    if (State->AesIsIdle == NV_FALSE &&
        !DecryptIsBusy &&
        !HashIsBusy)
    {
        State->AesIsIdle = NV_TRUE;
        State->BuffersForDev++;
        State->ObjDst += State->ChunkSize;
    }
}

/**
 * BlockForCryptoFinish(): Wait for any pending cryptographic op to complete.
 *
 * @param State Current reader state.
 *
 * Used to flush pending work when an error is encountered.
 */
static void BlockForCryptoFinish(NvBootReaderState *State)
{
    // Wait until engine becomes IDLE.
    while(NvBootSeIsEngineBusy())
        ;
}

/**
 * CheckCryptoHash(): Compare two hash values.
 *
 * @param Hash1 First hash value.
 * @param Hash2 Second hash value
 *
 * @retval NV_TRUE The two hash values match.
 * @retval NV_FALSE The two hash values differ.
 *
 * Used to compare a computed hash against a reference value.
 */
static NvBool CheckCryptoHash(NvBootHash *Hash1, NvBootHash *Hash2)
{
    int i;

    NV_ASSERT(Hash1 != NULL);
    NV_ASSERT(Hash2 != NULL);

    for (i = 0; i < NVBOOT_AES_BLOCK_LENGTH; i++)
    {
        if (Hash1->hash[i] != Hash2->hash[i]) return NV_FALSE;
    }

    return NV_TRUE;
}

/*
 * TODO: Implement faster path to read unencrypted, unsigned data directly
 *       to the destination?
 */

/**
 * NvBootReadOneObject(): Read one object from a boot device.
 *
 * @param Context The current boot context
 * @param ReadDst Pointer to the destination for the object data
 * @param ObjDesc Array of object descriptors
 * @param NumCopies Number of valid object descriptors
 * @param BadBlockTable Pointer to the bad block table.
 *
 * @retval NvBootError_Success The object was successfully read.
 * @retval NvBootError_HashMismatch The signature check failed.
 * @retval TODO Errors from UpdateDevStatus
 * @retval TODO Errors from LaunchDevRead
 * @retval TODO Errors from UpdateCryptoStatus
 * @retval TODO Errors from LaunchCryptoOps
 *
 * For reading a BCT: 
 *   - There is only one copy of the object, so NumCopies = 1. 
 *   - There is no bad block table, so BadBlockTable = NULL 
 *     and BadBlockTableSize = 0. 
 *   - SignatureOffset (to signed section of BCT) =
 *                      sizeof(NvBootBadBlockTable) + 
 *                      sizeof(NvBootRsaKeyModulus) +
 *                      sizeof(NvBootObjectSignature)
 *   - ReadDst = Context->BCT. 
 *   - SignatureRef = Pointer to Signature member in BCT (not referenced until 
 *     after reading is done)
 *
 * For reading a bootloader 
 *   - There may be multiple copies of the object, so NumCopies >= 1. 
 *   - There is a bad block table, which is provided. 
 *   - SignatureOffset = 0. 
 *   - ReadDst = data from the BCT 
 *   - SignatureRef = Pointer to BootloaderInfoTable Signature
 *
 * All processing is in units of pages.
 * 
 * The code loops over the chunks of the boot loader, reading them to
 * iRAM, decrypting/checksumming them as needed, and storing them out
 * to their destination in iRAM or SDRAM.
 */
NvBootError
NvBootReadOneObject(
    NvBootContext       *Context,
    NvU8                *ReadDst,
    NvBootReaderObjDesc *ObjDesc,
    NvU32                NumCopies,
    NvBootBadBlockTable *BadBlockTable)
{
    NvBootError        e;
    NvBootReaderState *State = &ReaderState;

    NV_ASSERT(Context != NULL);
    NV_ASSERT(ReadDst != NULL);
    NV_ASSERT(ObjDesc != NULL);

    InitReaderState(State, ObjDesc, NumCopies, ReadDst, Context);
    State->BadBlockTable = BadBlockTable;

    /* Continue processing work while there is work to do. */
    while ((State->ChunksRemainingForAes > 0) ||
           (State->DevIsIdle == NV_FALSE)     ||
           (State->AesIsIdle == NV_FALSE))
    {
        /*
         * Handle the device reading in the boot loader.
         */
        /* Check on the device reader progress. */
        NV_BOOT_CHECK_ERROR_CLEANUP(UpdateDevStatus(State));

        /* Start the device reading another chunk if possible. */
        if ((State->DevIsIdle == NV_TRUE) &&
            (State->BuffersForDev > 0)    &&
            (State->ChunksRemainingForDev > 0))
        {
            NV_BOOT_CHECK_ERROR_CLEANUP(LaunchDevRead(State));
        }

        /*
         * Handle the decryption and signing in the AES engine.
         */
        /* Check on the crypto progress. */
        UpdateCryptoStatus(State);

        if ((State->AesIsIdle == NV_TRUE) &&
            (State->BuffersForAes > 0))
        {
            /* Start the decryption & validation of the buffer */
            LaunchCryptoOps(State);
        }
    }

    /*
     * Check the final checksum/crypto hash data, which resides at the
     * end of the hash buffer, with the supplied reference hash.
     */
    if (State->HashObject == NV_TRUE)
    {
        if (!CheckCryptoHash(&ObjDesc[0].SignatureRef->CryptoHash, &(State->SignatureDst.CryptoHash)))
            return NvBootError_HashMismatch;
    }

#ifndef TARGET_ASIM  /* no padding was added b/c AES has been disabled */
    /* Check the padding */
    if (!IsValidPadding(ReadDst, ObjDesc[0].Length))
    {
        return NvBootError_ValidationFailure;
    }
#endif

    return NvBootError_Success;

 fail:
    /* Clean up after an unrecoverable failure. */

    /* Spin wait for the pending work to finish or abort. */
    BlockForDevFinish   (State);
    BlockForCryptoFinish(State);

    return e;
}
