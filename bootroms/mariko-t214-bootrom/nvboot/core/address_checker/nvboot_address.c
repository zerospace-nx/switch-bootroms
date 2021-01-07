/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
 
 #include "nvboot_address_int.h"
 #include "nvboot_config.h"
 #include "nvboot_rcm.h"
 #include "arse.h"
 #include <limits.h>
 

 /**
  *  Static table of apertures handled by BootROM. 
  *  It is important to make sure that Aperture+Length will not cause an overflow.
  */
const NvBootAddrRange BootAddressTable[] = 
 {
     /** ID *******Start Address ********* Length ***********/
     {BlRamRange, NVBOOT_BL_IRAM_START, (NVBOOT_BL_IRAM_END-NVBOOT_BL_IRAM_START+1)},
     // IRAM range = 256KB
     {IramRange,  NV_ADDRESS_MAP_IRAM_A_BASE, (NV_ADDRESS_MAP_IRAM_D_LIMIT-NV_ADDRESS_MAP_IRAM_A_BASE+1)},
     // Cap SDRAM range size at 2GB.
     {DramRange,  NVBOOT_BL_SDRAM_START, NV_ADDRESS_MAP_EMEM_LO_SIZE},
     // Sc7FwRamRange is the address range in which the SC7 FW and header can be
     // copied to.
     {Sc7FwRamRange,  NVBOOT_SC7_FW_START, (NVBOOT_SC7_FW_END-NVBOOT_SC7_FW_START+1)},
     // Sc7FwTzramRange is the range in TZRAM where the SC7
     // resume firmware can be stored while in SC7. The other possible location to 
     // store the SC7 firmware is SDRAM.
     {Sc7FwTzramRange, NV_ADDRESS_MAP_TZRAM_BASE, ARSE_TZRAM_BYTE_SIZE},
     // RcmMsgRange is the address range in which the RCM Messages can be
     // copied to. RCM Message consists of a header and a payload. The buffer address is assigned
     // such that payload will always be at NVBOOT_BL_IRAM_START
     {RcmMsgRange,  NVBOOT_RCM_MSG_IRAM_START, (NVBOOT_BL_IRAM_END-NVBOOT_RCM_MSG_IRAM_START+1)},
 };
 
 /**
  *  @brief Validates a given address range against the static range as determined 
  *         by the Id.
  *  @param RangeId Used to identify the valid address range for a given Id
  *  @param Address Start Address
  *  @param Length Length
  *  @return Valid or not
  */
 NvBootError NvBootValidateAddress(uint32_t RangeId, uint32_t Addr, uint32_t Len)
 {
     /// Try to match RangeId with entries in BootAddressTable
     int NumEntries = sizeof(BootAddressTable)/sizeof(NvBootAddrRange);
     
     for(int i=0;i<NumEntries;i++)
     {
         // try to find matching entry
         if(BootAddressTable[i].RangeId == RangeId)
            return NvBootValidateAddressGeneric(BootAddressTable[i].Start, BootAddressTable[i].Length, Addr, Len);
     }
     // no match
     return NvBootError_IllegalParameter;
 }
 /**
  *  @brief Validates given Address, Len to fit inside a certain known range
  *  @param ApertureStart: Start address of aperture
  *  @param ApertureLen:  Length of aperture
  *  @param Addr: Addr to be validated
  *  @param Len to be validated.
  *  @return 0 Success Or IllegalParameter
  */
  NvBootError NvBootValidateAddressGeneric(uint32_t ApertureStart, uint32_t ApertureLen, NvU32 Addr, NvU32 Len)
  {
      /// Minimum 1 byte of length. Checking if 0 bytes fit inside an aperture is invalid.
      if(!Len)
          return NvBootError_IllegalParameter;

      /// Overflow check
      if((Addr+(Len-1)) < Addr)
          return NvBootError_IllegalParameter;

      /**
       *  Valid address range is ApertureStart <= Addr+Len-1 <= ApertureStart+ApertureLen - 1 
       *  Check for above condition
       */
      /// Confirm Addr >= ApertureStart else return fail
      if(Addr < ApertureStart)
          return NvBootError_IllegalParameter;

      ///  Confirm Addr,Len fits in Aperture range
      if((Addr+(Len-1)) > (ApertureStart+(ApertureLen-1)))
          return NvBootError_IllegalParameter;
      
      return NvBootError_Success;
  }
 
 
