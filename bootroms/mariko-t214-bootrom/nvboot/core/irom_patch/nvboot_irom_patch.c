/*
 * Copyright (c) 2007 - 2014 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvcommon.h"
#include "nvrm_drf.h"
#include "nvboot_irom_patch_local.h"
#include "arclk_rst.h"
#include "arevp.h"
#include "arfuse.h"
#include "aripatch.h"
#include "nvboot_bit.h"
#include "nvboot_hacks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include "nvboot_bpmp_int.h"
#include "project.h"

#define HAVE_FCODE_H	0
#define FUSE_SIZE_IN_BITS (6 *  1024)	//FIXME: Remove this hardcode.

static const VT_NONSECURE NvU32 IROM_CAM_ADDR_MASK = 0xFFFF0000;

static const VT_NONSECURE NvU32 UINT_BITS = 32 ;
static const VT_NONSECURE NvU32 LOG2_UINT_BITS = 5 ;
static const VT_NONSECURE NvU32 INSIDE_UINT_OFFSET_MASK = 0x1F ;

static const VT_NONSECURE NvU32 H16_BITS = 16 ;
static const VT_NONSECURE NvU32 H16_START_OFFSET = 1 << (16 - 2) ;
static const VT_NONSECURE NvU32 H16_ECC_MASK      = 0xFFF0FFFFU ;
static const VT_NONSECURE NvU32 H16_PARITY_MASK   = 0x00008000U ;
static const VT_NONSECURE NvU32 H16_H_MASK        = 0x00007FFFU ;

static const VT_NONSECURE NvU32 H5_CODEWORD_SIZE = 12 ;
static const VT_NONSECURE NvU32 H5_BIT_OFFSET = 20 ; // the H5 code word is mapped on MSB of a word
static const VT_NONSECURE NvU32 H5_CODEWORD_MASK = 0xFFF00000U ; 
static const VT_NONSECURE NvU32 H5_PARITY_MASK   = 0x01000000U ;

static const VT_NONSECURE NvU32 NO_ERROR = 0 ;
static const VT_NONSECURE NvU32 CORRECTED_ERROR = 1 ;
static const VT_NONSECURE NvU32 UNCORRECTED_ERROR_ODD = 2 ; // calculated syndrome associated with a position outside the buffer itself
static const VT_NONSECURE NvU32 UNCORRECTED_ERROR_EVEN = 3 ;

extern VT_NONSECURE NvU32 SwCYAWord[];

extern NvBootInfoTable  BootInfoTable;

NvU32 ValidCamEntries[MAX_CAM] ={0};

#if USE_FUSES
//Stuffs from nvboot_fuse_program.c - Start
//
// Wait for completion (state machine goes idle).
static void FT_NONSECURE
WaitForFuseIdle(void)
{
    NvU32 RegData;

    do
    {
        RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FUSECTRL_0);
    } while (NV_DRF_VAL(FUSE, FUSECTRL, FUSECTRL_STATE, RegData) != 
             FUSE_FUSECTRL_0_FUSECTRL_STATE_STATE_IDLE);
}

// Expose (Visibility = 1) or hide (Visibility = 0) the fuse registers.
static void FT_NONSECURE SetFuseRegVisibility(NvU32 Visibility)
{
    NvU32 RegData;

    RegData = NV_READ32(NV_ADDRESS_MAP_CLK_RST_BASE +
                        CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    RegData = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER,
                                 MISC_CLK_ENB,
                                 CFG_ALL_VISIBLE,
                                 Visibility,
                                 RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CLK_RST_BASE + CLK_RST_CONTROLLER_MISC_CLK_ENB_0,
               RegData);
}

// Read a word from the fuses.
// Note: The fuses must already have been sensed, and the programming power
//       should be off.
static NvU32 FT_NONSECURE
ReadFuseWord(NvU32 Addr)
{
    NvU32 RegData;

    // Prepare the data 
    NV_WRITE32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FUSEADDR_0, Addr);

    // Trigger the read
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FUSECTRL_0);
    RegData = NV_FLD_SET_DRF_DEF(FUSE, FUSECTRL, FUSECTRL_CMD, READ, RegData);
    NV_WRITE32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FUSECTRL_0, RegData);

    // Wait for completion (state machine goes idle). 
    WaitForFuseIdle();

    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FUSERDATA_0);

    return RegData;
}
//Stuffs from nvboot_fuse_program.c - End
#else
#if HAVE_FCODE_H
#include "fcode.h"
#endif
#endif

// we use a manual table for the Hamming5 code
static const NvU32 VT_NONSECURE H5SyndromeTable[] = { 0x1, 
                                                0x2,
                                                0x4,
                                                0x8,
                                                0x0, // position of the parity bit, not included in standard Hamming 
                                                0x3, // sequential, non zero, non power of two numbers
                                                0x5,
                                                0x6,
                                                0x7,
                                                0x9,
                                                0xA,
                                                0xB } ;

static NvU32 FT_NONSECURE Parity( NvU32 *Data, NvU32 N) {
    NvU32 ParityWord, i ;
    ParityWord = Data[0] ;
    for (i=1; i<N; i++) {
        ParityWord ^= Data[i] ;
    }
    ParityWord = (ParityWord & 0x0000FFFF) ^ (ParityWord >> 16) ;
    ParityWord = (ParityWord & 0x000000FF) ^ (ParityWord >>  8) ;
    ParityWord = (ParityWord & 0x0000000F) ^ (ParityWord >>  4) ;
    ParityWord = (ParityWord & 0x00000003) ^ (ParityWord >>  2) ;
    ParityWord = (ParityWord & 0x00000001) ^ (ParityWord >>  1) ;
    return ParityWord ;
}

static NvU32 FT_NONSECURE Hamming5Syndrome(NvU32 *Data) {
  NvU32 i, Syndrome;
  // Data is assumed to be passed as present in Fuse, i.e. the ECC word starts at bit H5_BIT_OFFSET
  Syndrome = 0 ;
  for (i=0; i<H5_CODEWORD_SIZE; i++) { 
    if (Data[0] & (1 << (H5_BIT_OFFSET + i))) { Syndrome ^= H5SyndromeTable[i] ; }
  }
  // Return the syndrome aligned to the right offset
  return (Syndrome << H5_BIT_OFFSET) ;
}

static NvU32 FT_NONSECURE Hamming5Decode( NvU32 *Data) {
  NvU32 CalculatedParity, Syndrome, i, StoredNonH5 ;
  StoredNonH5 = Data[0] & ~H5_CODEWORD_MASK ; // Required for correct parity
  Data[0] &= H5_CODEWORD_MASK ;
  // parity should be even (0)
  CalculatedParity = Parity(Data, 1) ;
  Syndrome = Hamming5Syndrome(Data) >> H5_BIT_OFFSET ;
  Data[0] |= StoredNonH5 ;
  if (Syndrome != 0) {
    if (!CalculatedParity) { 
      return UNCORRECTED_ERROR_EVEN ;
    }
    // decode using the Syndrome table, if we fall through the position associated with the Syndrome does not exist
    for (i=0; i<H5_CODEWORD_SIZE; i++) {
      if (Syndrome == H5SyndromeTable[i]) {
        Data[0] ^= (1 << (i + H5_BIT_OFFSET)) ;
        return CORRECTED_ERROR ; // correctable error
      }
    }
    return UNCORRECTED_ERROR_ODD ;
  }
  if (CalculatedParity) { 
    Data[0] ^= H5_PARITY_MASK ;
    return CORRECTED_ERROR ;
  }
  return NO_ERROR ;
}
// The H16 routines don't make any assumption on the bits corresponding to the H5 codeword
// This allows for simpler testing and assume some wrapper functions will be used

static NvU32 FT_NONSECURE NvBootIRomPatchHamming16Syndrome( NvU32 *Data, NvU32 N) {
  NvU32 i, j, Syndrome ;
  // Calculate the syndrome
  Syndrome = 0 ;
  for (i=0; i<N; i++) {
    if (Data[i] != 0) {  // for speed, assuming that many words could be zero
      for (j=0; j<UINT_BITS; j++) {
	if ((Data[i] >> j) & 0x1) Syndrome ^= H16_START_OFFSET + (i * UINT_BITS) + j;
      }
    }
  }
  return Syndrome ;
}

static NvU32 FT_NONSECURE NvBootGetIRomPatchSize() {
    NvU32 RegData;
    /// With cumulative irom patch implementation, the length of each record
    /// is a part of the record itself and has a corresponding Hamming code with parity. 
    /// Register FUSE_BOOTROM_PATCH_SIZE_0 has been renamed to FUSE_FIRST_BOOTROM_PATCH_SIZE_0
    /// for historical reasons. (Refer Section 12.3 Fuse Bit Allocation of T177_fuse_IAS.docx,
    /// Arch bug# 1211388, bug for missing definition #1297076 )
    /// Therefore, this API (currently) returns the size of the first record of irom patch.
#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FIRST_BOOTROM_PATCH_SIZE_0);
    RegData = NV_DRF_VAL(FUSE, FIRST_BOOTROM_PATCH_SIZE, FIRST_BOOTROM_PATCH_SIZE, RegData);
#else
    RegData = 0;
#if HAVE_FCODE_H    
    RegData = FIRST_PATCH_SIZE;
#endif    
#endif
    
    return RegData;
}

#ifdef NV_DEBUG
// Stub out this function to save code space in the non secure section on the
// debug build.
static NvU32 FT_NONSECURE NvBootIRomPatchHamming16Decode( NvU32 *Data, NvU32 N) {
    return (NvU32) 0x0;
}
#else // NV_DEBUG
/*
 * Perform Hamming ECC decode.
 *
 * Return value:
 * 	0  - No Error
 * 	1  - Correctable Error
 * 	2  - Uncorrectable Error
 */
static NvU32 FT_NONSECURE NvBootIRomPatchHamming16Decode( NvU32 *Data, NvU32 N) {
   NvU32 CalculatedParity, StoredSyndrome, StoredParity, Syndrome, offset, i, j, offset_bits_set ;
   // parity should be even (0)
   CalculatedParity = Parity(Data, N) ;
   StoredSyndrome = Data[0] & H16_H_MASK ;
   StoredParity = Data[0] & H16_PARITY_MASK ;
   Data[0] &= ~H16_ECC_MASK ;
   Syndrome = NvBootIRomPatchHamming16Syndrome(Data, N) ^ StoredSyndrome ;
   Data[0] ^= StoredParity ;
   Data[0] ^= StoredSyndrome ;
   if (Syndrome != 0) {
	 if (!CalculatedParity) { 
	   return UNCORRECTED_ERROR_EVEN ;
	 }
	 // error is at bit offset Syndrome - H16_START_OFFSET, can be corrected if in range [H16_BITS, N * UINT_BITS[
	 offset = Syndrome - H16_START_OFFSET ;
	 i = (offset >> LOG2_UINT_BITS) ;
	 if ((offset < H16_BITS) || (offset >= UINT_BITS * N)) { // special case of the Hamming bits themselves, detected by Syndrome (or equivalently offset) being a power of two
	   offset_bits_set = 0;
	   for (j=0; j<H16_BITS-1; j++) {
		 if ((Syndrome >> j) & 1) offset_bits_set++;
	   }
	   if (offset_bits_set == 1) {
		 Data[0] ^= Syndrome ;
		 return CORRECTED_ERROR ;
	   } 
	   else {
		 return UNCORRECTED_ERROR_ODD ;
	   }
	 }
	 Data[i] ^= (1 << (offset & INSIDE_UINT_OFFSET_MASK)) ;
	 return CORRECTED_ERROR ; // correctable error
   }
   if (CalculatedParity) { 
	 Data[0] ^= H16_PARITY_MASK ;
	 return CORRECTED_ERROR ;
   }
   return NO_ERROR ;
 }
#endif // NV_DEBUG

/*
 * Replace NvBootUtilMemcpy() with restriction that
 * src, dest must be aligned and size in multiple of 4
 *
 */
static void FT_NONSECURE NvBootIRomPatchCpy4( void *dest, const void *src, size_t size )
{
    uint32_t i;
    uint32_t *p = (uint32_t *)src, *q = (uint32_t *)dest;

    NV_ASSERT( !((uint32_t)src & 0x03) && !((uint32_t)dest & 0x03) && !(size & 0x03) );

    size >>= 0x02;
    for (i=0; i<size; i++){
        *q = *p;
        p++;
        q++;
    }
}

#define CAMEntries(ADDR) \
        ((volatile unsigned int*)(NV_ADDRESS_MAP_IPATCH_BASE + IPATCH_ROM_OVERRIDE_CAM_0 + (ADDR * 4)))

#ifdef NV_DEBUG
// Stub out this function to save code space in the non secure section on the
// debug build.
static void FT_NONSECURE NvBootIromPatchParseUpdateCAM(NvU32 * PatchBuff, NvU32 C)
{
    return;
}
#else // NV_DEBUG
static void FT_NONSECURE NvBootIromPatchParseUpdateCAM(NvU32 * PatchBuff, NvU32 C)
{
    NvU16 i = 0;
    NvU16 j = 0;
    NvU16 CamEntriesPresent = 0;
    NvU32 CamValid = 0;
    NvU32 IPatchValid, CamValidIndex = 0;
    NvU16 ReplaceCamEntry = 0;

    NV_ASSERT(C < MAX_CAM);

    IPatchValid = NV_READ32(NV_ADDRESS_MAP_IPATCH_BASE + IPATCH_ROM_OVERRIDE_VALID_0);

    //check for any chip override entries
    while(i < C)
    {
        if(((PatchBuff[i] & IROM_CAM_ADDR_MASK) >> (H16_BITS - 1)) < NV_ADDRESS_MAP_IROML_SIZE ){
            ValidCamEntries[CamValid] = PatchBuff[i];
            CamValid++;
        }
        i++;
    }

    // FI check
    if(i!=C)
    {
        do_exception();
    }

    //no valid CAM entries
    if(!CamValid)
        return;

    //update PatchBuff with valid cam entries only.
    if(CamValid < C)
    {
        C = CamValid;
    }

    // No Previous Valid Ipatch cam entries detected.
    if(!IPatchValid){
        //Program CAM registers
        CamEntriesPresent = 0;
        CamValid = 0;
        for (i = 0; i < C; i++){
            NV_WRITE32(CAMEntries(CamEntriesPresent), ValidCamEntries[i]);
            CamValid |= (1 << CamEntriesPresent);
            CamEntriesPresent++;
        }
        // FI Check
        if(i!=C)
        {
            do_exception();
        }

    }
    else
    {
        CamValid = IPatchValid;
        while(IPatchValid){
            CamEntriesPresent++;
            IPatchValid >>= 1;
        }
        // FI Check. Ipatch should be 0 by now.
        if(IPatchValid)
        {
            do_exception();
        }
        // cam entries present
        CamValidIndex = CamEntriesPresent;
        
        for (i = 0; i < C; i++){
            ReplaceCamEntry = 0;
            for (j = 0; j < CamEntriesPresent; j++){
                //check for matching cam entry (for subtractive/nullify entry)
                // All Matching entries are replaced.
                IPatchValid = NV_READ32(CAMEntries(j));
                if((IPatchValid & IROM_CAM_ADDR_MASK) == (ValidCamEntries[i] & IROM_CAM_ADDR_MASK)){
                    NV_WRITE32(CAMEntries(j), ValidCamEntries[i]);
                    //donot update CamValid entry since this is replacement only
                    ReplaceCamEntry++;
                }
            }
            // FI Check.
            if(j!=CamEntriesPresent)
            {
                do_exception();
            }
            // update new entry @ the end in the list.
            if(!ReplaceCamEntry){
                // check cam limit 
                if(CamValidIndex >= MAX_CAM)
                    break;
                NV_WRITE32(CAMEntries(CamValidIndex), ValidCamEntries[i]);
                //check valid entry update is proper..
                CamValid |= (1 << CamValidIndex);
                CamValidIndex++;
                // check cam limit 
                if(CamValidIndex >= MAX_CAM)
                    break;
        	}
       	}
        // FI check
        if(i!=C)
        {
            do_exception();
        }
    }
    //Set IPATCH_ROM_OVERRIDE_VALID_0 to enable CAM
    NV_WRITE32(NV_ADDRESS_MAP_IPATCH_BASE + IPATCH_ROM_OVERRIDE_VALID_0, CamValid);
}
#endif //NV_DEBUG

void FT_NONSECURE NvBootApplyIRomPatch(void) {
    NvU32 NumOfPatchWords, PatchBuff[MAX_PAYLOAD], StartPatch, i, I, C,
    patchPL, patchEnd, Status, Excp_prologue_updated = 0;
    NvU8 *IRamPatchPtr = NULL;
    NvU32 IromPatchHeader = 0;
    NvU32 TotalPatchRecordSize = 0;
    
    BootInfoTable.IRomPatchStatus = 0;
    NumOfPatchWords = NvBootGetIRomPatchSize();
    NV_ASSERT(NumOfPatchWords < MAX_PAYLOAD);

    /**
     *  Make sure RAM Buffer is cleared of instructions possibly leftover from previous cycle.
     */
    NvU32 *IRamPatchPtr32 = (NvU32 *)&IRamExcpHandler[0];
    NvU32 RamBufferSize = sizeof(IRamExcpHandler)/sizeof(NvU32);
    for(i=0;i<RamBufferSize;i++)
    {
        IRamPatchPtr32[i]=0xffffffff; // An exception will be triggered if a glitch triggers a jump
                                      // into the buffer.
    }
    
    // FI check
    if(i!=RamBufferSize)
    {
        do_exception();
    }

#if USE_FUSES
    // first patch record, read fuse from end of 6k
    StartPatch = (FUSE_SIZE_IN_BITS - 32) / 32;
#else
#if HAVE_FCODE_H	
	StartPatch = FCODE_HEADER + 1;
#endif
#endif
    while(NumOfPatchWords) {
        TotalPatchRecordSize += NumOfPatchWords;
        if(TotalPatchRecordSize > MAX_PAYLOAD)
        {
            //update the status and return.
            // further reads violates the fuse irompatch size.
            return;
        }
        BootInfoTable.IRomPatchStatus |= 0x80;
#if USE_FUSES
        SetFuseRegVisibility(1);

        //StartPatch -- should be updated properly to fetch subsequent record.
        // reading from the previous patch record end should suffice.
        // AI -- arun, confirm..
		
        // read bootrom patch from fuse into buffer
        for (i = 0; i < NumOfPatchWords; i++)
            PatchBuff[i] = ReadFuseWord(StartPatch--);
        
        // FI protection.
        if(i!=NumOfPatchWords)
            do_exception();
        
        SetFuseRegVisibility(0);
#else
//        StartPatch = NumOfPatchWords;
#if HAVE_FCODE_H	
        for (i=0; i<NumOfPatchWords; i++)
            PatchBuff[i] = fcode[--StartPatch];
#endif	
#endif	

        //save IromPatchrecord  header
        IromPatchHeader = PatchBuff[0];
        
        // Run ECC 
        // HammingDecode() return value:
        // 0  - No Error
        // 1  - Correctable Error
        // 2  - Uncorrectable Errors
        Status = NvBootIRomPatchHamming16Decode(PatchBuff, NumOfPatchWords);

        // Save Status into BIT
        BootInfoTable.IRomPatchStatus |= Status;

        if (Status < UNCORRECTED_ERROR_ODD){
            C = ((PatchBuff[0] & IROM_PATCH_C_MASK) >> IROM_PATCH_C);
            I = NumOfPatchWords - C - 1;// instructions == N[i] - C[i] - 1
            //Program CAM registers
            if(C){
                    // Parse and update repeatative entry..
                    NvBootIromPatchParseUpdateCAM(&PatchBuff[1], C);
            }
            if (I) {
                //Copy SWI excp handler
                if(!Excp_prologue_updated)
                {
                    patchPL = (NvU32)&IRomPatch_prologue;
                    patchEnd = (NvU32)&IRomPatch_end;
                    IRamPatchPtr = (NvU8 *)&IRamExcpHandler;
                    
                    NvBootIRomPatchCpy4(IRamPatchPtr, (NvU8 *)patchPL, (patchEnd - patchPL));
                    IRamPatchPtr += (patchEnd - patchPL);
                    Excp_prologue_updated++;
					
                    NvBootIRomPatchCpy4(IRamPatchPtr, &(PatchBuff[C+1]), (I * sizeof(NvU32)));
                    IRamPatchPtr += (I * sizeof(NvU32));
                   
                    //Update SWI vector to IRAM excp handler
                    NV_WRITE32(NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_SWI_VECTOR_0, (NvU32)IRamExcpHandler);
                }
                else
                {
                    NvBootIRomPatchCpy4(IRamPatchPtr, &(PatchBuff[C+1]), (I * sizeof(NvU32)));
                    IRamPatchPtr += (I * sizeof(NvU32));
                }
            }
            //check for the Next Patch Record.
            PatchBuff[0] = IromPatchHeader;//saved Header ?? check with Marc??
            NumOfPatchWords = 0;
            if((PatchBuff[0] >> IROM_PATCH_N)){
                NumOfPatchWords = PatchBuff[0];
                Status = Hamming5Decode(&IromPatchHeader);
                if(Status >= UNCORRECTED_ERROR_ODD){
                    // Save Status into BIT
                    // failure status
                    BootInfoTable.IRomPatchStatus |= Status;
                    return;
                }else{
                    NumOfPatchWords = (IromPatchHeader >> IROM_PATCH_N); 
            	}
            }
        }
        else{
            // error in decoding.
            // break the loop and exit with status update.
            // NO FI here. This could be a genuine ECC failure in ROM.
            return;
        }
    }
}

void FT_NONSECURE NvBootIRomPatchCleanup(void) {
    int c;
    // Invalidate all CAM
    NV_WRITE32(NV_ADDRESS_MAP_IPATCH_BASE + IPATCH_ROM_OVERRIDE_VALID_0, 
	    NV_RESETVAL(IPATCH, ROM_OVERRIDE_VALID));
    //Restore SWI vector
    NV_WRITE32(NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_SWI_VECTOR_0, 
	    NV_RESETVAL(EVP, COP_SWI_VECTOR));
    //Clean up CAM registers. Don't think we need FI here as this is done before secure exit path.
    // Not much we can do if we detect a glitch.
    for (c = 0; c < MAX_CAM; c++)
	    NV_WRITE32(CAMEntries(c), NV_RESETVAL(IPATCH, ROM_OVERRIDE_CAM));
}

NvU32 FT_NONSECURE NvBootGetSwCYA(void)
{
    return SwCYAWord[0];
}

