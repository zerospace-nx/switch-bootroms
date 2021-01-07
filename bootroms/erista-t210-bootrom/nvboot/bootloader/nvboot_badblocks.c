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
 * nvboot_badblocks.c - Implementation of Bad Block Table support.
 */

#include "nvboot_badblocks_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_util_int.h"

#if USE_BADBLOCKS
/* Function prototypes */
static NvBool
IsBadVirtualBlock(const NvU32 VirtualBlock, const NvBootBadBlockTable *Table);

/**
 * IsBadVirtualBlock(): Predicate that checks to see if a virtual block 
 * is a known bad block.
 *
 * @param[in] VirtualBlock The virtual block number
 * @param[in] Table The bad block table.
 *
 * @retval NV_TRUE Indicates that the block is believed to be bad
 * @retval NV_FALSE Indicates that the block is believed to be good
 */
static NvBool
IsBadVirtualBlock(const NvU32 VirtualBlock, const NvBootBadBlockTable *Table)
{
    NvU8  TableEntry;

    NV_ASSERT(Table != NULL);

    /* Check to see if the block number is out of range. */
    if (VirtualBlock >= Table->EntriesUsed)
    {
	/* Return NV_FALSE - this table cannot tell that the block is bad. */
	return NV_FALSE;
    }
    else
    {
	/*
	 * Look up the virtual block in the bad block table.
	 * Note that the bit vector is stored in bytes, hence the >> 3 when
	 * finding the byte of the bit vector in the table and the 0x7 when
	 * extracting the bit number for the block check.
	 */
	TableEntry = Table->BadBlocks[VirtualBlock >> 3];
	TableEntry = TableEntry & (1 << (VirtualBlock & 0x7));
	return TableEntry ? NV_TRUE : NV_FALSE;
    }
}
#endif

/**
 * NvBootIsBadBlock(): Predicate that checks to see if Block is a known bad
 * block.
 *
 * @param[in] Block Block number for the query
 * @param[in] Table The bad block table
 *
 * @retval NV_TRUE Indicates that the block is believed to be bad.
 * @retval NV_FALSE Indicates that the blocks is believed to be good.
 *
 * NvBootIsBadBlock() is the public interface to the bad block code.
 * It converts the block number to a virtual block number (an actual entry
 * in the bad block table) and invokes IsBadVirtualBlock().
 */
NvBool
NvBootIsBadBlock(const NvU32 Block, const NvBootBadBlockTable *Table)
{
#if USE_BADBLOCKS
    NvU32 VirtualBlock;

    NV_ASSERT(Table != NULL);

    /* Compute the virtual block number used by this reference. */
    VirtualBlock = Block >> (Table->BlockSizeLog2 -
                             Table->VirtualBlockSizeLog2);

    return IsBadVirtualBlock(VirtualBlock, Table);
#else
    return NV_FALSE;
#endif
}

