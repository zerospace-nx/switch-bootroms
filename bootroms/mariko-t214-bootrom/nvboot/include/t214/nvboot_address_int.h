/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
 
 #ifndef INCLUDE_T214_NVBOOT_ADDRESS_INT_H
 #define INCLUDE_T214_NVBOOT_ADDRESS_INT_H
 
 #include "nvtypes.h"
 #include "project.h"
 #include "address_offsets_sizes.h"
 #include "nvboot_error.h"
 #if defined(__cplusplus)
 extern "C"
 {
 #endif
 
 enum RangeIdentifierRec
 {
     BlRamRange=1,
     IramRange,
     DramRange,
     BtcmRange,
     Sc7FwRamRange,
     Sc7FwTzramRange,
     RcmMsgRange,
     MaxRange = 0x7FFFFFFF
 } RangeIdentifier;
 
 
 typedef struct NvBootAddrRangeRec
 {
     uint32_t RangeId;
     uint32_t Start;
     uint32_t Length;
 } NvBootAddrRange;
 
 /**
  *  Check if an entrypoint fits inside a given payload.
  */
 #define NvBootValidateEntryPoint(ApertureStart, ApertureLen, EntryPoint)    \
    NvBootValidateAddressGeneric((ApertureStart), (ApertureLen), (EntryPoint), sizeof(uint32_t))
 
 /**
  *  @brief Validates a given address range against the static range as determined 
  *         by the Id.
  *  @param RangeId Used to identify the valid address range for a given Id
  *  @param Address Start Address
  *  @param Length Length
  *  @return Valid or not
  */
 NvBootError NvBootValidateAddress(uint32_t RangeId, uint32_t Address, uint32_t Length);
 
  /**
  *  @brief Validates given Address, Len to fit inside a certain known range
  *  @param ValidAddr Known good range
  *  @param ValidLen Known good len
  *  @param TestAddr Addr to be validated
  *  @param TestLen to be validated.
  *  @return 0 Success Or IllegalParameter
  */
 NvBootError NvBootValidateAddressGeneric(uint32_t ApertureStart, uint32_t ApertureLen, NvU32 Addr, NvU32 Len);
 

#if defined(__cplusplus)
 }
 #endif
 
 #endif // INCLUDE_T214_NVBOOT_ADDRESS_INT_H
