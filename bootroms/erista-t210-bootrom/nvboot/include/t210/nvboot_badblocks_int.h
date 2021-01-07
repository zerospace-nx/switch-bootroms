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
 * nvboot_badblocks_int.h - Internal header for the Bad Block Table.
 */

#ifndef INCLUDED_NVBOOT_BADBLOCKS_INT_H
#define INCLUDED_NVBOOT_BADBLOCKS_INT_H

#include "nvboot_badblocks.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * NvBootIsValidBadBlockTable(): Validate the integrity of the bad block
 * table.  Returns NV_TRUE if the table is valid, NV_FALSE if not.
 * Code that uses the bad block table should call this routine first,
 * because other code will not check arguments so diligently.
 */
NvBool NvBootIsValidBadBlockTable(const NvBootBadBlockTable *Table);

/*
 * NvBootIsBadBlock(): Predicate that checks to see if Block is a known bad
 * block.  IsBad is set to NV_TRUE if the block is bad, NV_FALSE otherwise.
 */
NvBool
NvBootIsBadBlock(const NvU32 Block, const NvBootBadBlockTable *Table);

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_BADBLOCKS_INT_H */
