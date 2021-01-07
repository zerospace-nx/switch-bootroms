/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvboot_fuse_local.h"
#include "nvrm_drf.h"
#include "arapbpm.h"
#include "arapb_misc.h"
#include "arclk_rst.h"
#include "arfuse.h"
#include "nvboot_crypto_aes_param.h"
#include "nvboot_error.h"
#include "nvboot_fuse.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_pmc_scratch_map.h"
#include "nvboot_crypto_fskp.h"
#include "nvboot_se_hash.h"
#include "nvboot_util_int.h"
#include "project.h"


/*
 * The NV_FIELD64_* macros are helper macros for the public NV_DRF64_* macros.
 */
#define NV_FIELD64_SHIFT(x) ((0?x)%64)
#define NV_FIELD64_MASK(x)  (0xFFFFFFFFFFFFFFFFULL>>(63-((1?x)%64)+((0?x)%64)))

/** NV_DRF64_NUM - define a new 64-bit register value.

    @ingroup nvrm_drf

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param n numeric value for the field
 */
#define NV_DRF64_NUM(d,r,f,n) \
    (((n)& NV_FIELD64_MASK(d##_##r##_0_##f##_RANGE)) << \
        NV_FIELD64_SHIFT(d##_##r##_0_##f##_RANGE))

/**
 * Reports whether the ODM Production Mode fuse is set (burned)
 *
 * Note that this fuse by itself does not determine whether the chip is in
 * ODM Production Mode.
 *
 * If the SECURE_PROVISION_INDEX_0[1] is set (i.e. TEST_PART), we always return
 * NV_FALSE.
 * If the TEST_PART fuse is burned, the chip can never
 * be an ODM production part. Always force Nv_Production mode.
 * We can also use this TEST_PART fuse to simulate Nv Production
 * mode in an unfused chip (for debugging BR with JTAG), as well
 * as debug the Secure Provisioning feature.
 * TEST_PART cannot be burned after SECURITY_MODE has been burned.
 *
 * @param none
 *
 * @return NV_FALSE if TEST_PART fuse is set.
 *         NV_FALSE if ODM production Mode fuse is unset (un-burned).
 *         NV_TRUE if ODM production Mode fuse is set (burned) and TEST_PART fuse unset.
 */
static FT_NONSECURE NvBool
NvBootFuseIsOdmProductionModeFuseSet(void)
{
    NvU32 SecurityModeFuse;
    NvBool IsTestPartFuseSet = NvBootFuseIsSecureProvisionTestPartFuseBurned();

#if USE_FUSES
    SecurityModeFuse = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_SECURITY_MODE_0);
#else
    SecurityModeFuse = NVBOOT_DEFAULT_PRODUCTION_FUSE;
#endif

    if (IsTestPartFuseSet)
    {
        return NV_FALSE;
    }
    else
    {
        if (SecurityModeFuse)
            return NV_TRUE;
        else
            return NV_FALSE;
    }
}

/**
 * Reports whether the NV Production Mode fuse is set (burned)
 *
 * Note that this fuse by itself does not determine whether the chip is in
 * NV Production Mode.
 *
 * If the SECURE_PROVISION_INDEX_0[1] is set (i.e. TEST_PART), we always return
 * NV_TRUE.
 * If the TEST_PART fuse is burned, the chip can never
 * be an ODM production part. Always force Nv_Production mode.
 * We can also use this TEST_PART fuse to simulate Nv Production
 * mode in an unfused chip (for debugging BR with JTAG), as well
 * as debug the Secure Provisioning feature.
 * TEST_PART cannot be burned after SECURITY_MODE has been burned.
 *
 * @param none
 *
 * @return NV_TRUE if TEST_PART fuse is set.
 *         NV_FALSE if Nv production Mode fuse is unset (un-burned).
 *         NV_TRUE if Nv production Mode fuse is set (burned) and TEST_PART fuse unset.
 */
NvBool FT_NONSECURE
NvBootFuseIsNvProductionModeFuseSet(void)
{
    NvU32 ProductionModeFuse;
    NvBool IsTestPartFuseSet = NvBootFuseIsSecureProvisionTestPartFuseBurned();

#if USE_FUSES
    ProductionModeFuse = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRODUCTION_MODE_0);
#else
    ProductionModeFuse = NVBOOT_DEFAULT_PREPRODUCTION_FUSE;
#endif

    if (IsTestPartFuseSet)
    {
        return NV_TRUE;
    }
    else
    {
        if (ProductionModeFuse)
            return NV_TRUE;
        else
            return NV_FALSE;
    }
}

NvU32 FT_NONSECURE
NvBootFuseGetSecureProvisionInfo()
{
    NvU32 RegData;
#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_SECURE_PROVISION_INFO_0);
#else
    RegData = NVBOOT_DEFAULT_SECURE_PROVISION_INFO;
#endif
    return RegData;
}

NvBool FT_NONSECURE
NvBootFuseIsSecureProvisionTestPartFuseBurned()
{
    NvU32 RegData;
    NvU32 TestPartVal;

    RegData = NvBootFuseGetSecureProvisionInfo();
    TestPartVal = NV_DRF_VAL(FUSE, SECURE_PROVISION_INFO, TEST_PART, RegData);

    if(TestPartVal)
        return NV_TRUE;
    else
        return NV_FALSE;
}

/**
 * Reports whether any of the Secure Boot Key fuses are set (burned)
 *
 * @param none
 *
 * @return NV_TRUE if any of the Secure Boot Key fuses are set (burned); else
 *         NV_FALSE
 */
static NvBool FT_NONSECURE
NvBootFuseIsSbkSet(void)
{
    NvU32 AllSbkOred;

#if USE_FUSES
    AllSbkOred  = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY0_0);
    AllSbkOred |= NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY1_0);
    AllSbkOred |= NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY2_0);
    AllSbkOred |= NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY3_0);
#else
    AllSbkOred  = NVBOOT_DEFAULT_SBK0;
    AllSbkOred |= NVBOOT_DEFAULT_SBK1;
    AllSbkOred |= NVBOOT_DEFAULT_SBK2;
    AllSbkOred |= NVBOOT_DEFAULT_SBK3;
#endif

    if (AllSbkOred)
        return NV_TRUE;
    else
        return NV_FALSE;
}

/*
 * externally-visible API's
 */
NvBool FT_NONSECURE
NvBootFuseIsFailureAnalysisMode(void)
{
    volatile NvU32 RegData;

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_FA_0);
#else
    RegData = NVBOOT_DEFAULT_FA_FUSE;
#endif

    if (RegData)
        return NV_TRUE;
    else
        return NV_FALSE;
}

NvBool
NvBootFuseIsOdmProductionMode(void)
{
    if (!NvBootFuseIsFailureAnalysisMode()
         && NvBootFuseIsOdmProductionModeFuseSet())
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }
}

NvBool
NvBootFuseIsOdmProductionModeSecure(void)
{
    if (!NvBootFuseIsOdmProductionMode()) 
        return NV_FALSE;
    
    if (NvBootFuseIsSbkSet())
        return NV_TRUE;
    else
        return NV_FALSE;
} 

NvBool
NvBootFuseIsOdmProductionModeNonsecure(void)
{
    if (!NvBootFuseIsOdmProductionMode())
        return NV_FALSE;
    
    if (NvBootFuseIsSbkSet())
        return NV_FALSE;
    else 
        return NV_TRUE;
}  


NvBool
NvBootFuseIsNvProductionMode(void)
{
    if (!NvBootFuseIsFailureAnalysisMode()     &&
         NvBootFuseIsNvProductionModeFuseSet() &&
        !NvBootFuseIsOdmProductionModeFuseSet())
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }

}

NvBool
NvBootFuseIsPkcBootMode(void)
{
    NvU32 Reg;

#if USE_FUSES
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
#else
    Reg = NVBOOT_DEFAULT_BOOT_SECURITY_AUTH_INFO;
#endif
    Reg &= FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_FIELD;

    //#define FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA  _MK_ENUM_CONST(2)
    if(Reg == FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_PKC_RSA)
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }
}

NvBool FT_NONSECURE
NvBootFuseIsPreproductionMode(void)
{

    if (!NvBootFuseIsFailureAnalysisMode()     &&
        !NvBootFuseIsNvProductionModeFuseSet() &&
        !NvBootFuseIsOdmProductionModeFuseSet())
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }

}

#if USE_FUSES
static void 
NvBootFuseCopyBytes (NvU32 RegAddress, NvU8 *pByte, const NvU32 nByte)
{
    NvU32 RegData;
    NvU32 i;

    NV_ASSERT((pByte != NULL) || (nByte == 0));
    NV_ASSERT (RegAddress != 0);

    for (i = 0, RegData = 0; i < nByte; i++)
    {
        if ((i&3) == 0)
        { 
            RegData = NV_READ32(RegAddress);
            RegAddress += 4;
        }
        pByte[i] = RegData & 0xFF;
        RegData >>= 8;
    }
}
#endif
    
void
NvBootFuseGetSecureBootKey(NvU8 *pKey)
{
#if USE_FUSES
    NV_ASSERT(pKey != NULL);
    NvBootFuseCopyBytes(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY0_0,
                        pKey, NVBOOT_AES_KEY_128_BYTES);
#else
    NvU32 *pKeyWords = (NvU32 *)pKey;
    NV_ASSERT(pKey != NULL);
    pKeyWords[0] = NVBOOT_DEFAULT_SBK0;
    pKeyWords[1] = NVBOOT_DEFAULT_SBK1;
    pKeyWords[2] = NVBOOT_DEFAULT_SBK2;
    pKeyWords[3] = NVBOOT_DEFAULT_SBK3;
#endif
}
    
void
NvBootFuseGetDeviceKey(NvU8 *pKey)
{

#if USE_FUSES
    NV_ASSERT(pKey != NULL);
    NvBootFuseCopyBytes(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATE_KEY4_0,
                        pKey, NVBOOT_DEVICE_KEY_BYTES);
#else
    NvU32 *pDevKey = (NvU32 *)pKey;
    NV_ASSERT(pKey != NULL);

    *pDevKey = NVBOOT_DEFAULT_DK;
#endif
}


void
NvBootFuseHideKeys(void)
{
    NV_WRITE32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PRIVATEKEYDISABLE_0,
               NV_DRF_DEF(FUSE,
                          PRIVATEKEYDISABLE,
                          PRIVATEKEYDISABLE_VAL,
                          KEY_INVISIBLE));
}

void
NvBootFuseGetPcpHash(NvU8 *pKey)
{
#if USE_FUSES
    NV_ASSERT(pKey != NULL);
    NvBootFuseCopyBytes(NV_ADDRESS_MAP_FUSE_BASE + FUSE_PUBLIC_KEY0_0,
                        pKey, NVBOOT_SE_SHA256_LENGTH_BYTES);
#else
    NvU32 *pKeyWords = (NvU32 *)pKey;
    NV_ASSERT(pKey != NULL);
    pKeyWords[0] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY0;
    pKeyWords[1] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY1;
    pKeyWords[2] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY2;
    pKeyWords[3] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY3;
    pKeyWords[4] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY4;
    pKeyWords[5] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY5;
    pKeyWords[6] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY6;
    pKeyWords[7] = NVBOOT_DEFAULT_FUSE_PUBLIC_KEY7;
#endif    
}

void FT_NONSECURE NvBootFuseGetUniqueId(NvBootECID *pId)
{
    NvU32   Rsvd1;
    NvU32   Y;              // Y-coordinate
    NvU32   X;              // X-coordinate
    NvU32   Wafer;          // Wafer
    NvU32   Lot1;            // Lot
    NvU32   Lot0;            // Lot
    NvU32   Fab;            // Fab
    NvU32   Vendor;         // Vendor
    NvU32   Reg, FuseProt;  // Scratch register

    NV_ASSERT(pId != NULL);

    //          Field    Bits   Data
    //          (LSB first)
    //          -------  ----   ----------------------------------------
    //          Reserved   6    
    //          Y          9    Wafer Y-coordinate
    //          X          9    Wafer X-coordinate
    //          WAFER      6    Wafer id
    //          LOT_1      28   Lot code 1
    //          LOT_0      32   Lot code 0
    //          FAB        6    FAB code
    //          VENDOR     4    Vendor code
    //          -------  ----
    //          Total     100
    //
    // Gather up all the bits and pieces.
    //<Vendor:4><Fab:6><Lot0:26><Lot0:6><Lot1:26><Lot1:2><Wafer:6><X:9><Y:9><Reserved:6>
    //


#if USE_FUSES

    // Access to fuse registers is protected, so make all registers visible first
    // Note, this is duplicates SetFuseRegVisibility(1), as no call allowed into secure part
    FuseProt  = NV_READ32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE + CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    FuseProt |= CLK_RST_CONTROLLER_MISC_CLK_ENB_0_CFG_ALL_VISIBLE_FIELD;
    NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE + CLK_RST_CONTROLLER_MISC_CLK_ENB_0, FuseProt);


    // Vendor
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_VENDOR_CODE_0);
    Vendor = NV_DRF_VAL(FUSE, OPT_VENDOR_CODE, OPT_VENDOR_CODE, Reg);

    // Fab
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_FAB_CODE_0);
    Fab = NV_DRF_VAL(FUSE, OPT_FAB_CODE, OPT_FAB_CODE, Reg);

    // Lot code 1
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_LOT_CODE_1_0);
    Lot1 = NV_DRF_VAL(FUSE, OPT_LOT_CODE_1, OPT_LOT_CODE_1, Reg);

    // Lot code 0
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_LOT_CODE_0_0);
    Lot0 = NV_DRF_VAL(FUSE, OPT_LOT_CODE_0, OPT_LOT_CODE_0, Reg);

    // Wafer
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_WAFER_ID_0);
    Wafer = NV_DRF_VAL(FUSE, OPT_WAFER_ID, OPT_WAFER_ID, Reg);

    // X-coordinate
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_X_COORDINATE_0);
    X = NV_DRF_VAL(FUSE, OPT_X_COORDINATE, OPT_X_COORDINATE, Reg);

    // Y-coordinate
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_Y_COORDINATE_0);
    Y = NV_DRF_VAL(FUSE, OPT_Y_COORDINATE, OPT_Y_COORDINATE, Reg);

    // Reserved
    Reg = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_OPT_OPS_RESERVED_0);
    Rsvd1 = NV_DRF_VAL(FUSE, OPT_OPS_RESERVED, OPT_OPS_RESERVED, Reg);

    // Hide back protected registers.  No need to read back, just flip the bit.
    // Note, this is duplicates SetFuseRegVisibility(0), as no call allowed into secure part
    FuseProt &= ~CLK_RST_CONTROLLER_MISC_CLK_ENB_0_CFG_ALL_VISIBLE_FIELD;
    NV_WRITE32(NV_ADDRESS_MAP_PPSB_CLK_RST_BASE + CLK_RST_CONTROLLER_MISC_CLK_ENB_0, FuseProt);

#else  // for fuse hacks

    Rsvd1       = NVBOOT_DEFAULT_RSVD1;
    Y         	= NVBOOT_DEFAULT_Y_COORDINATE;
    X          	= NVBOOT_DEFAULT_X_COORDINATE;
    Wafer    	= NVBOOT_DEFAULT_WAFER_ID;
    Lot1        = NVBOOT_DEFAULT_LOT1_CODE;
    Lot0        = NVBOOT_DEFAULT_LOT0_CODE;
    Fab      	= NVBOOT_DEFAULT_FAB_CODE;
    Vendor 		= NVBOOT_DEFAULT_VENDOR_CODE;

#endif
    //<Vendor:4><Fab:6><Lot0:26><Lot0:6><Lot1:26><Lot1:2><Wafer:6><X:9><Y:9><Reserved:6>

    Reg         = 0;						             //Least significant 32 bits are cleared
    Reg         |= NV_DRF_NUM(ECID,ECID0,RSVD1,Rsvd1);   //<Reserved:6> = 6 bits
    Reg         |= NV_DRF_NUM(ECID,ECID0,Y,Y);           //<Y:9><Reserved:6> = 15 bits
    Reg         |= NV_DRF_NUM(ECID,ECID0,X,X);           //<X:9><Y:9><Reserved:6> = 24 bits
    Reg         |= NV_DRF_NUM(ECID,ECID0,WAFER,Wafer);   //<Wafer:6><X:9><Y:9><Reserved:6> = 30 bits
	Reg         |= NV_DRF_NUM(ECID,ECID0,LOT1,Lot1);     //<Lot1:2><Wafer:6><X:9><Y:9><Reserved:6> = 32 bits   
    pId->ECID_0 = Reg;		

    Lot1        >>= 2;							         //discard 2 bits as it is already copied in ECID_0 field
    Reg         = 0;						             // 32 bits are cleared
    Reg         |= NV_DRF_NUM(ECID,ECID1,LOT1,Lot1);	 //<Lot1:26> = 26 bits
    Reg         |= NV_DRF_NUM(ECID,ECID1,LOT0,Lot0);	 //<Lot0:6><Lot1:26> = 32 bits
    pId->ECID_1 = Reg;	
    Lot0 >>= 6;							                 //discard 6 bits as it is already copied in ECID_1 field

    Reg          = 0;						             // 32 bits are cleared
    Reg         |= NV_DRF_NUM(ECID,ECID2,LOT0,Lot0);	 //<Lot0:26> = 26 bits
    Reg         |= NV_DRF_NUM(ECID,ECID2,FAB,Fab);		 //<Fab:6><Lot0:26> = 32 bits
    pId->ECID_2 = Reg;

    Reg         = 0;						             // 32 bits are cleared
    Reg         |= NV_DRF_NUM(ECID,ECID3,VENDOR,Vendor); //<Vendor:4> = 4 bits
    pId->ECID_3 = Reg;
}

/**
[103:100]
T214: APB_MISC_GP_HIDREV_0_MAJORREV
T194: MISCREG_HIDREV_0_MAJORREV
[111:104]
T214: APB_MISC_GP_HIDREV_0_CHIPID
T194: MISCREG_HIDREV_0_CHIPID
[115:112]
T214: APB_MISC_GP_MINORREV
T194: MISCREG_HIDREV_0_MINORREV
[122:116]
Reserved
[123]
FUSE_RESERVED_PRODUCTION_0[3]
[126:124]
BOOT_SECURITY_INFO[2:0]
[127]
FUSE_PRODUCTION_MODE
*/
void NvBootFuseAddAdditionalEcidInfo(NvBootECID *pEcid)
{
    pEcid->ECID_3 |= NV_DRF_NUM(ECID, ECID3, PRODUCTION_MODE, NvBootFuseIsNvProductionMode());

    uint32_t RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
    uint32_t BootSecurityInfoMask = FUSE_BOOT_SECURITY_INFO_0_AUTHENTICATION_FIELD |
                                     FUSE_BOOT_SECURITY_INFO_0_ENCRYPTION_FIELD;
    RegData &= BootSecurityInfoMask;

    pEcid->ECID_3 |= NV_DRF_NUM(ECID, ECID3, BOOT_SECURITY_INFO, RegData);

    RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE + APB_MISC_GP_HIDREV_0);
    
    // Extract and set Major Revision. T214, MajorRev: 0x2
    pEcid->ECID_3 = NV_FLD_SET_DRF_NUM(ECID, ECID3, MAJORREV, 
                    NV_DRF_VAL(APB_MISC_GP, HIDREV, MAJORREV, RegData),
                    pEcid->ECID_3);
    // Extract and set ChipId. T214, ChipId: 0x21
    pEcid->ECID_3 = NV_FLD_SET_DRF_NUM(ECID, ECID3, CHIPID, 
                    NV_DRF_VAL(APB_MISC_GP, HIDREV, CHIPID, RegData),
                    pEcid->ECID_3);
    // Extract and set Minor Revision.
    pEcid->ECID_3 = NV_FLD_SET_DRF_NUM(ECID, ECID3, MINORREV, 
                    NV_DRF_VAL(APB_MISC_GP, HIDREV, MINORREV, RegData),
                    pEcid->ECID_3);

}

/**
 * Reports whether the the SkipDevSelStraps fuse is set (burned)
 *
 * @param none
 *
 * @return NV_TRUE if the SkipDevSelStrasp fuse is set (burned); else NV_FALSE
 */
NvBool
NvBootFuseSkipDevSelStraps(void)
{
    NvU32 RegData;

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, SKIP_DEV_SEL_STRAPS, RegData);
#else
    RegData = NVBOOT_DEFAULT_SKIP_DEV_SEL_STRAPS_FUSE;
#endif

    if (RegData) 
        return NV_TRUE;
    else
        return NV_FALSE;
}

/*
 * We assume that the enum coding is the same as the bit coding in
 * the fuses.
 * Unlike AP15, these bits are all together in one register.
 */
void
NvBootFuseGetBootDevice(NvBootFuseBootDevice *pDev)
{
    NvU32 RegData;

    NV_ASSERT(pDev != NULL);

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, BOOT_DEVICE_SELECT, RegData);
#else
    RegData = NVBOOT_DEFAULT_DEV_SEL_FUSES;
#endif

    if (RegData >= (int) NvBootFuseBootDevice_Max)
    {
        *pDev = NvBootFuseBootDevice_Sdmmc;
    }
    else
    {
        *pDev = (NvBootFuseBootDevice) RegData;
    }
}

NvBool
NvBootFuseIsUsbChargerDetectionEnabled()
{
    NvU32 RegData;


#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, ENABLE_CHARGER_DETECT, RegData);
#else
    RegData = NVBOOT_DEFAULT_ENABLE_CHARGER_DETECT_FUSE;
#endif

    if (RegData) 
        return NV_TRUE;
    else
        return NV_FALSE;

}

void
NvBootFuseGetBootDeviceConfiguration(NvU32 *pConfigIndex)
{
    NvU32 RegData;

    NV_ASSERT(pConfigIndex != NULL);

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_DEVICE_INFO_0);
    RegData = NV_DRF_VAL(FUSE, BOOT_DEVICE_INFO, BOOT_DEVICE_CONFIG, RegData);
#else
    RegData = NVBOOT_DEFAULT_DEV_CONFIG_FUSES;
#endif

    *pConfigIndex = RegData;
} 

void
NvBootFuseGet2ButtonRcm(NvU32 *p2ButtonRcm)
{
    NvU32 RegData;

    NV_ASSERT(p2ButtonRcm != NULL);

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, 2_BUTTON_RCM, RegData);
#else
    RegData = NVBOOT_DEFAULT_2_BUTTON_RCM_FUSE;
#endif

    *p2ButtonRcm = RegData;
}

void
NvBootFuseGetSku(NvBootSku_SkuId *pSkuId)
{
    NvU32 Sku;

    NV_ASSERT(pSkuId != NULL);
    NvBootFuseGetSkuRaw(&Sku);
    *pSkuId = (NvBootSku_SkuId) (Sku & NVBOOT_SKU_MASK);
}

void
NvBootFuseGetSkuRaw(NvU32 *pSku)
{
    NV_ASSERT(pSku != NULL);

#if USE_FUSES
    *pSku = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_SKU_INFO_0);
#else
    *pSku = NVBOOT_DEFAULT_SKU_FUSES;
#endif
}

void
NvBootFuseSkipDelaySeq(NvU32 *pSkipSequence)
{
    NvU32 RegData;

    NV_ASSERT(pSkipSequence != NULL);

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, SKIP_DELAY_SEQUENCE, RegData);
#else
    RegData = NVBOOT_DEFAULT_SKIP_DELAY_SEQUENCE_FUSE;
#endif

    *pSkipSequence = RegData;
}

void
NvBootFuseGetSataCalib(NvU32 *pSataCalib)
{
    TODO
    *pSataCalib = 0;
#if 0
    NV_ASSERT(pSataCalib != NULL);

#if USE_FUSES
    *pSataCalib = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_SATA_CALIB_0);
    *pSataCalib = NV_DRF_VAL(FUSE, SATA_CALIB, SATA_CALIB, *pSataCalib);
#else
    *pSataCalib = NVBOOT_DEFAULT_SATA_CALIB_FUSE;
#endif
#endif
}

NvBool
NvBootFuseIsSataOobRxCalibDisabled(void)
{
    NvU32 RsvdProdFuse = NVBOOT_DEFAULT_RESERVED_PRODUCTION_FUSE;
#if USE_FUSES
    RsvdProdFuse = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_PRODUCTION_0);
#endif
    return NV_DRF_VAL(FUSE, RESERVED_PRODUCTION, SATA_RX_OOB_BOOT_CALIB_DISABLE, RsvdProdFuse)? 
           NV_TRUE: NV_FALSE;
}

void
NvBootFuseAliasFuses()
{
    /* Fuse aliasing removed.
     * This is done by fuse bypass package.
     */
}

NvBool
NvBootFuseIsWatchdogEnabled()
{
    NvU32 RegData;
#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, ENABLE_WATCHDOG, RegData);
#else
    RegData = 0;
#endif

    if (RegData) 
        return NV_TRUE;
    else
        return NV_FALSE;
}

/*
 * Retrieve port used for RCM.
 */
void
NvBootFuseGetRcmPort(NvBootRCMPortID_T *pRCMPortId)
{
    NvU32 RegData;

    NV_ASSERT(pRCMPortId != NULL);

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_SW_0);
    RegData = NV_DRF_VAL(FUSE, RESERVED_SW, RCM_PORT, RegData);
#else
    RegData = NVBOOT_DEFAULT_RCM_PORT;
#endif

    *pRCMPortId = (NvBootRCMPortID_T) RegData;
}

NvU32
NvBootFuseGetSecureProvisionIndex()
{
    NvU32 RegData;
#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_SECURE_PROVISION_INDEX_0);
#else
    RegData = NVBOOT_DEFAULT_SECURE_PROVISION_INDEX;
#endif
    return RegData;
}

NvBool
NvBootFuseIsSecureProvisionKeyHideFuseBurned()
{
    NvU32 RegData;
    NvU32 KeyHideVal;

    RegData = NvBootFuseGetSecureProvisionInfo();
    KeyHideVal = NV_DRF_VAL(FUSE, SECURE_PROVISION_INFO, KEY_HIDE, RegData);

    if(KeyHideVal)
        return NV_TRUE;
    else
        return NV_FALSE;
}

NvBootError
NvBootFuseGetSecureProvisioningIndexValidity()
{
    NvU32 KeyIndex;
    KeyIndex = NvBootFuseGetSecureProvisionIndex();

    if(KeyIndex == FSKP_DISABLED)
    {
        return NvBootError_SecProvisioningAntiCloningKeyDisabled;
    }
    else if(KeyIndex >= FSKP_ANTI_CLONING_KEY_START &&
       KeyIndex <= FSKP_ANTI_CLONING_KEY_END)
    {
        return NvBootError_Success;
    }
    else
    {
        return NvBootError_SecProvisioningInvalidAntiCloningKey;
    }
}

// See section 10.4 of Secure Boot ISS for truth table of BR handling.
NvBootError
NvBootFuseIsSecureProvisioningMode(NvU32 SecProvisioningKeyNum)
{
    const NvBootError e_AntiCloningStatus = NvBootFuseGetSecureProvisioningIndexValidity();

    // Secure Provisioning is only for NV Production mode.
    if (!NvBootFuseIsNvProductionMode())
        return NvBootError_SecProvisioningDisabled;

    if (e_AntiCloningStatus == NvBootError_SecProvisioningAntiCloningKeyDisabled)
    {
        if (SecProvisioningKeyNum == FSKP_DISABLED)
        {
            return NvBootError_SecProvisioningDisabled;
        }
        else if (SecProvisioningKeyNum >= FSKP_ANTI_CLONING_KEY_START &&
                 SecProvisioningKeyNum <= FSKP_ANTI_CLONING_KEY_END)
        {
            return NvBootError_SecProvisioningInvalidKeyInput;
        }
        else if (SecProvisioningKeyNum >= FSKP_REGULAR_KEY_START &&
                 SecProvisioningKeyNum <= FSKP_REGULAR_KEY_END)
        {
            return NvBootError_SecProvisioningEnabled;
        }
        else // else SecProvisioningKeyNum is out of bounds
        {
            return NvBootError_SecProvisioningInvalidKeyInput;
        }
    }
    else if (e_AntiCloningStatus ==  NvBootError_Success)
    {
        // If a valid anti-cloning fuse is burned, we also don't care about the
        // _Insecure BCT value (SecProvisioningKeyNum parameter).
        return NvBootError_SecProvisioningEnabled;
    }
    else
    {
        return NvBootError_SecProvisioningInvalidKeyInput;
    }
}

void FT_NONSECURE
NvBootFuseGetBootSecurityAuthenticationInfo(NvU32 *pSecureIndex)
{
    NvU32 RegData;

    NV_ASSERT(pSecureIndex != NULL);

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
    RegData = NV_DRF_VAL(FUSE, BOOT_SECURITY_INFO, AUTHENTICATION, RegData);
#else
    RegData = NVBOOT_DEFAULT_BOOT_SECURITY_INFO;
#endif

    *pSecureIndex = RegData;
}

NvBool FT_NONSECURE
NvBootFuseBootSecurityIsEncryptionEnabled()
{
    NvU32 RegData;

#if USE_FUSES
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
#else
    RegData = NVBOOT_DEFAULT_BOOT_SECURITY_ENCRYPT_INFO;
#endif
    return NV_DRF_VAL(FUSE, BOOT_SECURITY_INFO, ENCRYPTION, RegData)?
        NV_TRUE: NV_FALSE;
}

NvBool
NvBootFuseIsOemFuseEncryptionEnabled(void)
{
    NvU32 FuseEncryptionEnabled;

    FuseEncryptionEnabled = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
    FuseEncryptionEnabled = NV_DRF_VAL(FUSE, BOOT_SECURITY_INFO, OEM_FUSE_ENCRYPTION_ENABLE, FuseEncryptionEnabled);

    if( FuseEncryptionEnabled == FUSE_BOOT_SECURITY_INFO_0_OEM_FUSE_ENCRYPTION_ENABLE_ENABLE )
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }
}

NvBool
NvBootFuseIsSeContextAtomicSaveEnabled(void)
{
    NvU32 SeContextAtomicSaveEnabled;

    SeContextAtomicSaveEnabled = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
    SeContextAtomicSaveEnabled = NV_DRF_VAL(FUSE, BOOT_SECURITY_INFO, SE_ATOMIC_CONTEXT_SAVE_ENABLE, SeContextAtomicSaveEnabled);

    if( SeContextAtomicSaveEnabled == FUSE_BOOT_SECURITY_INFO_0_SE_ATOMIC_CONTEXT_SAVE_ENABLE_ENABLE )
    {
        return NV_TRUE;
    }
    else
    {
        return NV_FALSE;
    }
}

NvU32
NvBootFuseGetOemFekBankSelect()
{
    NvU32 RegData;
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_RESERVED_PRODUCTION_0);
    return NV_DRF_VAL(FUSE, RESERVED_PRODUCTION, OEM_FEK_BANK_SELECT, RegData);
}

NvU32 NvBootFuseGetFuseDecryptionKeySelection()
{
    NvU32 RegData;
    RegData = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_BOOT_SECURITY_INFO_0);
    return NV_DRF_VAL(FUSE, BOOT_SECURITY_INFO, OEM_FUSE_ENCRYPTION_SELECT, RegData);
}

/**
 *  Check FUSE_JTAG_SECUREID_VALID_0. This is a dependency for SE RNG.
 */
NvBool FT_NONSECURE __attribute__((optimize("O0")))
NvBootFuseIsJtagSecureIdFuseSet(void)
{
    uint32_t Fuse;
    uint32_t FuseInverse;

    Fuse = NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_JTAG_SECUREID_VALID_0);
    
    FuseInverse = ~(NV_READ32(NV_ADDRESS_MAP_FUSE_BASE + FUSE_JTAG_SECUREID_VALID_0));

    // Double check sensitive conditions. Glitching is bad
    // since an attacker can skip RNG delay.
    if(Fuse == 0)
    {
        if(FuseInverse == 0xFFFFFFFF)
            return NV_FALSE;
    }
    return NV_TRUE;

}
