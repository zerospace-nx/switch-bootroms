/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef INCLUDED_NVBOOT_IROM_PATCH_LOCAL_H
#define INCLUDED_NVBOOT_IROM_PATCH_LOCAL_H

/* Fuse allocation for patching */
#define MAX_PAYLOAD (2560 >> 5)
/* Size of exception prologue in DWORD */
#define EXCP_PROLOGUE	(0x48 >> 2)

#define IROM_PATCH_C		0x10		// 16 bit
#define IROM_PATCH_C_MASK	0x000F0000
#define IROM_PATCH_N		0x19		// 25 bit

/* Cam Entries set to 12 */
#define MAX_CAM			0x0C

#ifdef __arm__

extern void IRomPatch_prologue(void);
extern void IRomPatch_end(void);


#pragma arm section zidata = "NonZIBuffer"
NV_ALIGN(4) NvU8 
IRamExcpHandler[sizeof(NvU32) * (EXCP_PROLOGUE + MAX_PAYLOAD)];
#pragma arm section

#endif //__arm__

#endif //INCLUDED_NVBOOT_IROM_PATCH_LOCAL_H
