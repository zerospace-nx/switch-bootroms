/*
 * Copyright (c) 2007 - 2010 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvcommon.h"
#include "arahb_arbc.h"
#include "arapb_misc.h"
#include "arevp.h"
#include "arrtc.h"
#include "arse.h"
#include "artimerus.h"
#include "arsecure_boot.h"
#include "nvboot_bct.h"
#include "nvboot_bit.h"
#include "nvboot_arc_int.h"
#include "nvboot_clocks_int.h"
#include "nvboot_wdt_int.h"
#include "nvboot_codecov_int.h"
#include "nvboot_coldboot_int.h"
#include "nvboot_config.h"
#include "nvboot_config_int.h"
#include "nvboot_error.h"
#include "nvboot_fuse_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_i2c_int.h"
#include "nvboot_limits_int.h"
#include "nvboot_main_int.h"
#include "nvboot_pads_int.h"
#include "nvboot_pmc_int.h"
#include "nvboot_rcm_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_reset_pmu_int.h"
#include "nvboot_se_aes.h"
#include "nvboot_se_int.h"
#include "nvboot_sku_int.h"
#include "nvboot_ssk_int.h"
#include "nvboot_strap_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_util_int.h"
#include "nvboot_log_int.h"
#include "nvboot_usbf_int.h"
#include "nvboot_usbcharging_int.h"
#include "nvboot_version_rom.h"
#include "nvboot_warm_boot_0_int.h"
#include "nvboot_irom_patch_int.h"
#include "nvboot_rcm_port_int.h"
#include "nvboot_lock_pmc_secure.h"
#include "project.h"

/*
 * The following bit flags indicate various actions to be carried out upon exit
 * from the Secure Section of the Boot ROM code.
 */

/* reset the full chip */
#define NVBOOT_SECURE_EXIT_OPTION_FULLRESET (0x1 << 0)
/* zero out the Boot Information Table (BIT) */
#define NVBOOT_SECURE_EXIT_OPTION_CLEAR_BIT (0x1 << 1)
/* zero out the Boot Configuration Table (BCT) */
#define NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT (0x1 << 2)


#ifdef BR_LOCATION_IS_IROM
#define BOOT_CODE_BASE (NV_ADDRESS_MAP_IROM_BASE)
#endif
#ifdef BR_LOCATION_IS_IRAM_PLUS_64K
#define BOOT_CODE_BASE (NV_ADDRESS_MAP_DATAMEM_BASE+64UL*1024UL)
#endif

#ifndef BOOT_CODE_BASE
#error no legal value for BOOT_CODE_BASE provided
#endif

typedef void Ptr(void);

/* Function prototypes */

/**
 * Configure AES engines for Cold Boot operations.
 *
 * If the chip is in ODM Secure Production Mode, then use the SBK as the key;
 * otherwise use a key of all zeroes.
 *
 * * set up the NVBOOT_DECRYPT_SLOT key slot of the NVBOOT_DECRYPT_ENGINE AES
 *   engine for decryption operations
 * * set up the NVBOOT_HASH_SLOT key slot of the NVBOOT_HASH_ENGINE AES engine
 *   for AES-CMAC operations (AES-CMAC employs an underlying AES encryption
 *   operation)
 *
 * Note: the Cold Boot code assumes that decryption and hashing are performed
 *       on different AES engines and therefore that the two operations can
 *       be performed in parallel.  A compile-time assert checks that this
 *       condition is met.
 *
 * @param none
 *
 * @retval none
 */

static void NvBootMainSetupAesEngines(void);

/**
 * prepare AES engines for bootloader and/or Recovery Mode applet; there
 * are two possible scenarios, depending on the boot type --
 *
 * 1. WB0
 *    a. compute SSK key tables and load into NVBOOT_SSK_ENGINE AES engine
 *    b. mark SSK encrypt/decrypt key slots as no-read, no-write
 *    c. set SBK key tables to zeroes in NVBOOT_SBK_ENGINE AES engine
 *    d. mark SBK encrypt/decrypt key slots as write-only
 *    e. load zero key table into all other key slots on both AES engines
 * 2. frozen/cold boot and Recovery Mode
 *    a. compute SSK key tables and load into NVBOOT_SSK_ENGINE AES engine
 *    b. mark SSK encrypt/decrypt key slots as write-only
 *    c. compute SBK key tables and load into NVBOOT_SBK_ENGINE AES engine
 *    d. mark SBK encrypt/decrypt key slots as write-only
 *    e. load zero key table into all other key slots on both AES engines
 *
 * Note: the SSK key tables must be stored in "secure" key slots so 
 *       that they can be marked as no-read, no-write by the bootloader.
 *       The SBK key tables can be stored in either "secure" or "non-secure"
 *       key slots since they only need to be read-protected; write-protection
 *       isn't required because the bootloader is obligated to over-write the
 *       SBK key tables with zeroes.  Thereafter, there is nothing to protect;
 *       hence, no need for write protection.  Access permissions for the 
 *       SSK and SBK key slots are handled in the NvBootAesLockSbkAndSsk()
 *       routine; rigorous checks are performed there.
 *
 *
 * @param IsWarmBoot0 if NV_TRUE, then load an all-zero key table into the SBK
 *        key slots and disable writes to SSK key slots; otherwise compute key
 *        tables based on the SBK fuse value and load the result into the SBK 
 *        key slots, then mark the SSK key slots are write-only
 *
 * @return none
 */

static void NvBootMainConfigureAesEnginesForExit(NvBool IsWarmBoot0);

/**
 * Clean up and exit from the Secure Section of iROM.
 *
 * Steps are
 * 1. flush all sensitive information from the system
 * 2. exit from the Secure Section of iROM
 * 3. begin executing the post-boot-ROM code
 *    a. WB0 restart code for Warm Boot 0
 *    b. boot loader for Cold Boot
 *    c. applet for Recovery Mode
 *    d. full-chip reset for error
 *
 * Steps for flushing sensitive information are
 * 1. disable access to SBK and DK fuses
 * 2. disable read access to SSK's PMC Secure Scratch registers
 * 3. flush key info from remaining AES engines
 * 4. disable read access to SBK key slots in AES engine
 * 5. disable read and (optionally) write access to SSK key slots in AES engine
 *    (write access disabled for WB0, else write access left enabled)
 * 6. over-write the iRAM area containing Boot ROM data, except those areas
 *    containing data explicitly being passed to the bootloader
 * 7. Enable JTAG if desired.
 *
 * exit criteria includes
 * 1. start address for AVP
 * 2. chip reset flag
 *
 * Steps for exiting Secure Section of iROM are
 * 1. disable Secure Section of iROM
 * 2. if an error occurred, reset the full chip
 * 3. else jump to AVP's post-boot-ROM execution start address
 *
 * @return none; this routine should never return
 */

static void
NvBootMainSecureRomExit(
    NvBool IsWarmBoot0,
    NvU32 BootRomExitBranchTargetAvp,
    NvU32 BootRomExitOptionsBitmap,
    NvU32 MainSecureDebugControl);

/*
 * NvBootMainAsmSecureExit() performs the following tasks:
 * - It clears the area of memory defined by Start and Stop
 *   addresses.  This is performed using burst 8 and so we
 *   force alignment on 32 bytes boundary.
 * - It modifies the Secure Boot register, setting it to the
 *   value passed as a parameter, with the following bits handled
 *   o always set to disable the protected IROM access bit
 *   o set the JTAG disable value to a specified value
 *   o always set to disable the Secure Boot flag.  This flag
 *     is sticky so that further changes are not allowed until 
 *     the next reset (and reentering the IROM)
 * - Clears up registers
 * - Passes control to the bootloader, at the address provided
 *   as a parmater.  This uses a BX instruction, so that the 
 *   bootloader can be fully compiled in Thumb code if desired
 *
 * @param BootloaderEntryAddress
 *        Entry address into the boolarder
 * @param StartClearAddress
 *        Start address of the area of memory to be cleared
 *        32 bytes aligned for efficiency
 * @param StopClearAddress
 *        This is the first address that is *not* cleared
 *        32 bytes aligned for efficiency
 * @param SecureRegisterValue
 *        A bitmap where the following fields are handled
 *        SECURE_BOOT_FLAG, always DISABLE (meaning not secure)
 *        PIROM_DISABLE, always DISABLE (meaning no access)
 *        JTAG_DISABLE, normally defined by BCT
 *
 * @retval none, this routine never returns
 */

extern void 
NvBootMainAsmSecureExit(
    NvU32 BootloaderEntryAddress, 
    NvU32 StartClearAddress,
    NvU32 StopClearAddress, 
    NvU32 SecureRegisterValue);

/*
 * NvBootMainNonsecureBootLoader() provides a safe place for the boot
 * process to land when RCM fails.  Depending upon operating mode, JTAG
 * may be enabled at this point, permitting debugging.
 *
 * @retval none, this routine never returns
 */
static void
NvBootMainNonsecureBootLoader(void);

/*
 * NvBootMainRedirectExcp() redirects AVP exception vectors to spiloop
 * in IRAM.
 *
 * @retval none
 */
static void
NvBootMainRedirectExcp(void);

/**
 * Sanitize the prospective value of APBDEV_PMC_DEBUG_AUTHENTICATION_0, which
 * controls the various debug controls in the chip based upon the boot
 * path and features used.
 *
 * Not declared static on purpose. The intention is to eventually expose
 * this function in the nvboot interface, so we can create unit tests that can
 * call this function.
 *
 * The value of SecureDebugControlVal comes mainly from two sources:
 * the Bct field SecureDebugControl and the value passed in via
 * RCM message. The value of SecureDebugControlVal may be ignored
 * in certain chip states - PreProdction/NvProduction/Failure Analysis.
 *
 * @param *SecureDebugControlVal The value to program into APBDEV_PMC_DEBUG_AUTHENTICATION_0
 * sourced from the Bct or Rcm Message.
 * @param *ChipECID The chip's ECID
 * @param *BctECID The ECID passed in via the BCT.
 * @param IsWarmBoot NV_TRUE if this is an LP0 cycle, NV_FALSE otherwise.
 * @param IsRcmMode NV_TRUE if this is an RCM boot, NV_FALSE otherwise.
 * @param IsFactoryProvisioningMode NV_TRUE if this was an a factory secure provisioning boot.
 *
 * @return none
 */
void NvBootMainProcessSecureDebugControl(NvU32 *SecureDebugControlVal,
                                         NvBootECID *ChipECID,
                                         NvBootECID *BctECID,
                                         NvBool IsWarmBoot,
                                         NvBool IsRcmMode,
                                         NvBool IsFactoryProvisioningMode);


/*
 * Global data
 */

// The following variables are forced at known places (bottom of IRAM) 
// by controlling their section and in the order shown
// This allows the bootloader to know where they are and the secure
// ROM exit to not zero them if needed by the bootloader
// TODO, check which ones should be defined as static

#pragma arm section zidata = "MainBIT", rwdata = "MainBIT"
NvBootInfoTable   BootInfoTable; /* Must be in ZI data */

#pragma arm section zidata = "MainBCT", rwdata = "MainBCT"
// to get some free space after the BIT, we align the BCT at a 128 bytes boundary
NvBootConfigTable BootConfigTable __attribute__ ((aligned (128))) ;

#pragma arm section zidata = "ExcpHndlr", rwdata = "ExcpHndlr"
NvU32 ExcpLoop __attribute__ ((aligned (4))); /* AVP exception loop in IRAM */

#pragma arm section zidata = "SecureExitData", rwdata = "SecureExitData"
// This used to be here to make sure these variables were not cleared before
// being used.  Now this is only used to simplify the assembly code that clears memory
// TOOD check if all but one variable could be mapped on the stack, one left as alignment
// They control the behavior of the secure exit code

static NvU32 DummyAlignedStartOfClearArea 
             __attribute__ ((aligned (32))) = 0; 

// returning to normal mapping
#pragma arm section

extern unsigned int Image$$EXEC_IRAM_RW_BIT$$Base ;
extern unsigned int Image$$EXEC_IRAM_RW_BCT$$Base ;
extern unsigned int Image$$EXEC_IRAM_RW_EXCP$$Base ;
extern unsigned int Image$$EXEC_SECURE_EXIT_DATA$$Base ;

#ifdef ENABLE_CODECOV
// instantiate the global code coverage array
NvU32 NvBootCodeCoverageArray[NVBOOT_CODECOV_ARRAY_WORDS];
#endif

void
NvBootMainSetupAesEngines(void)
{
    NvBootAes128Key         Sbk;
    NvBootFuseOperatingMode Mode;
    
    NV_CT_ASSERT(NVBOOT_DECRYPT_ENGINE != NVBOOT_HASH_ENGINE);

    /*
     * Determine the operating mode & key
     */
    NvBootFuseGetOperatingMode(&Mode);
    NvBootFuseGetSecureBootKey((NvU8 *)(Sbk.Key));

    /* Clear the key if not in OdmSecureSBK or OdmSecurePKC mode to force a key of zeroes. */
    if ((Mode != NvBootFuseOperatingMode_OdmProductionSecureSBK)&& \
       (Mode != NvBootFuseOperatingMode_OdmProductionSecurePKC))
    {
        NvBootUtilMemset(&Sbk, 0, sizeof(NvBootAes128Key));
    }

    /*
     * Configure the AES engines
     */
    /*
     * Regardless of whether entering cold boot or forced recovery or warm boot,
     * set up the SE clocks. Assumes PLLP is stable. 
     */
    NvBootSeInitializeSE();

    /*
     * From this point on, there is a copy of the SBK in memory.
     * All exit paths from this function must clear the in-memory copy
     * of the SBK.
     */

    // Write the SBK into SE key slot reserved for SBK
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK, 
                              SE_MODE_PKT_AESMODE_KEY128, 
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3, 
                              &Sbk.Key[0]);

    // Initialize SE SBK key slot OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK, 
                              SE_MODE_PKT_AESMODE_KEY128, 
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS, 
                              0);

    // Initialize SE SBK key slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK, 
                              SE_MODE_PKT_AESMODE_KEY128, 
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS, 
                              0);

    // Write the SBK into a SE key slot for TEMPORARY use by BR to do AES-CMAC
    // hash operations. IMPORTANT: BR *must* clear this key slot at BR exit!
    // This facilitates the interleaving of AES-CMAC hash and AES-decrypt
    // operations when the BCT & bootloaders are read in from secondary
    // storage by dedicating one key slot for AES-CMAC hashing and
    // one key slot for AES decrypt operations. This removes the overhead
    // of having to switch the SE between hash and decrypt operations
    // as well as the saving/restoring of IVs and intermediate hashes.
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK_AES_CMAC_Hash,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                              &Sbk.Key[0]);

    // Initialize SE SBK key slot OriginalIv[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK_AES_CMAC_Hash,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                              0);

    // Initialize SE SBK key slot UpdatedIV[127:0] to zero
    NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK_AES_CMAC_Hash,
                              SE_MODE_PKT_AESMODE_KEY128,
                              SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                              0);

    /* Wipe the memory used by Key. */
    NvBootUtilMemset(&Sbk, 0, sizeof(NvBootAes128Key));
}


void
NvBootMainConfigureAesEnginesForExit( NvBool IsWarmBoot0 )
{
    NvBootSeRsaKeySlot seRsaSlot;
    NvBootSeAesKeySlot seSlot;
    NvBootAes128Key    Sbk;
    NvU32              RegData;

    // Load AES engine with SBK key tables (nonsecure slots)

    if ( IsWarmBoot0 ) {

        // Clear the SBK from the SE key slot reserved for the SBK. Clear all
        // 256-bits just in case, even though SBK is 128-bits.
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY256,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                                  0);

        // Clear SE SBK key slot OriginalIv[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY128,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                                  0);

        // Clear SE SBK key slot UpdatedIV[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY128,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                                  0);
    } else {
        NvBootFuseGetSecureBootKey((NvU8 *)&Sbk.Key);

        // Write the SBK into SE key slot reserved for SBK
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY128,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                                  &Sbk.Key[0]);

        // Initialize SE SBK key slot OriginalIv[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY128,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                                  0);

        // Initialize SE SBK key slot UpdatedIV[127:0] to zero
        NvBootSeKeySlotWriteKeyIV(NvBootSeAesKeySlot_SBK,
                                  SE_MODE_PKT_AESMODE_KEY128,
                                  SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                                  0);

        // Clear local buffer used to hold the SBK
        NvBootUtilMemset(&Sbk, 0, sizeof(NvBootAes128Key));
    }

    if ( ! IsWarmBoot0 )
    {
        // For Cold boot and Recovery Mode, compute SSK and load it into the
        // reserved SE keyslot NvBootSeAesKeySlot_SSK.
        // The reserved SE keyslot isn't locked yet; we'll still need
        // to read the SSK back in order to compute encrypt/decrypt key tables
        // and load them into the relevant AES engine key slots.
        // Generate SSK and load it into the reserved SE keyslot.
        NvBootSskGenerate();

        // Do NOT touch the SE key slots if it is an LP0 cycle. The SE key slots
        // have been restored already by the SE context restore code.
        // Clear the remaining key slots that are not used for SSK or SBK in the SE.
        // Note, this code will also clear the SBK loaded for temporary use by BR in
        // keyslot NvBootSeAesKeySlot_SBK_AES_CMAC_Hash.
        for (seSlot = NvBootSeAesKeySlot_0; seSlot < NvBootSeAesKeySlot_Num; seSlot++)
        {
            if( !((seSlot == NvBootSeAesKeySlot_SBK) || (seSlot == NvBootSeAesKeySlot_SSK)) )
            {
                // Clear the key slots not reserved for the SSK and SBK.
                NvBootSeKeySlotWriteKeyIV(seSlot,
                                          SE_MODE_PKT_AESMODE_KEY256,
                                          SE_CRYPTO_KEYIV_PKT_WORD_QUAD_KEYS_0_3,
                                          0);
            }

            // Clear the OriginalIv[127:0] to zero for all slots.
            NvBootSeKeySlotWriteKeyIV(seSlot,
                                      SE_MODE_PKT_AESMODE_KEY128,
                                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_ORIGINAL_IVS,
                                      0);

            // Clear the UpdatedIV[127:0] to zero for all slots.
            NvBootSeKeySlotWriteKeyIV(seSlot,
                                      SE_MODE_PKT_AESMODE_KEY128,
                                      SE_CRYPTO_KEYIV_PKT_WORD_QUAD_UPDATED_IVS,
                                      0);
        }


        // Clear the SE RSA key slots at coldboot/RCM exit only.
        // WARNING:  The RSA key slot is a blocking function and will poll forever
        // if the SE context restore step fails because the SE is disabled by BR.
        // When the SE has been disabled it will always indicate BUSY.
        // If the SE context restore fails, skip the clearing of the RSA key slots.
        for (seRsaSlot = NvBootSeRsaKeySlot_1; seRsaSlot < NvBootSeRsaKeySlot_Num; seRsaSlot++)
        {
            NvBootSeRsaClearKeySlot(seRsaSlot);
        }
    }

    // Disable read access to SBK key slots. Configure SSK key slots depending
    // on the boot type --
    // * if WB0, disable both reads and writes
    // * else, disable only reads
    // SSK key permissions are handled by calling NvBootSskLockSsk in
    // NvBootMainSecureRomExit(...).

    // Disable read access to SE SBK key slot. SSK key permissions for SE handled
    // in NvBootSskLockSsk.
    // Note: The SBK key slot key permission should have been restored already
    // by the SE context restore code. Doing this step in the LP0 path is
    // redundant but there is no harm in doing so.
    NvBootSeLockSbk();

    // Handle SE engine disables.  For WarmBoot, restore the disable state
    // recored in the PMC register.  For all others, clear the PMC disable
    // bit.
    if (IsWarmBoot0 && NvBootPmcGetAesDisable())
    {
        // Propagate crypto disable to SE.
        NvBootSeDisableEngine();
    }
    else
    {
        NvBootPmcSetAesDisable(NV_FALSE);
    }

    // Clear SE TZRAM for all BR paths except LP0
    if ( ! IsWarmBoot0 )
    {
        NvBootSeClearTzram();
    }

    // Clear all informational status bits in SE_INT_STATUS_0 except ERR_STAT.
    // SE_OP_DONE, OUT_DONE, OUT_LL_BUF_WR, IN_DONE, IN_LL_BUF_RD
    // are cleared before BR exit since these status bits are only for
    // SE transaction-to-transaction status information.
    // We are purposely leaving ERR_STAT not cleared since this bit tells us
    // an error as occurred and it should be preserved for the next SW in the
    // boot chain to know about it.
    RegData = 0;
    RegData = NV_FLD_SET_DRF_DEF(SE, INT_STATUS, SE_OP_DONE, SW_CLEAR, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, INT_STATUS, OUT_DONE, SW_CLEAR, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, INT_STATUS, OUT_LL_BUF_WR, SW_CLEAR, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, INT_STATUS, IN_DONE, SW_CLEAR, RegData);
    RegData = NV_FLD_SET_DRF_DEF(SE, INT_STATUS, IN_LL_BUF_RD, SW_CLEAR, RegData);
    NvBootSetSeReg(SE_INT_STATUS_0, RegData);

    if (IsWarmBoot0)
    {
        // Lock PMC Secure Scratch registers.
        // Note: Call to NvBootWriteLockPmcSecure should be after all accesses 
        // to secure scratch by Bootrom are over.
        NvBootWriteLockPmcSecure();
    }

}

void NvBootMainProcessSecureDebugControl(NvU32 *SecureDebugControlVal,
                                         NvBootECID *ChipECID,
                                         NvBootECID *BctECID,
                                         NvBool IsWarmBoot,
                                         NvBool IsRcmMode,
                                         NvBool IsFactoryProvisioningMode)
{
    // Mask out any undefined bits passed in via BCT or RCM.
    *SecureDebugControlVal &= APBDEV_PMC_DEBUG_AUTHENTICATION_0_RESET_MASK;

    // If this is an LP0 cycle, we don't need to change the value of
    // APBDEV_PMC_DEBUG_AUTHENICATION_0.
    // The value in this register is preserved in the always-on PMC block.
    // However, APBDEV_PMC_DEBUG_AUTHENTICATION_0 is always programmed unconditionally at
    // NvBootMainAsmSecureExit since it is boot type unaware.
    // Thus we need to make sure that upon exit, that NvBootAsmSecureExit is setting
    // APBDEV_PMC_DEBUG_AUTHENTICATION_0 to the same value as before the LP0 cycle.
    if(IsWarmBoot)
    {
        // Read back the current value of APBDEV_PMC_DEBUG_AUTHENTICATION_0,
        // and set SecureDebugControlVal to that value.
        *SecureDebugControlVal = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_DEBUG_AUTHENTICATION_0);
        return;
    }

    if(NvBootFuseIsPreproductionMode() || (NvBootFuseIsNvProductionMode() && IsFactoryProvisioningMode == NV_FALSE))
    {
        // If we are in NvProduction mode with secure provisioning disabled,
        // we just enable JTAG regardless. The values of DBGEN, NIDEN, SPIDEN,
        // SPNIDEN, DEVICEEN comes from //hw/ar/doc/t210/dfd/DFD_TBSA_adherence.xlsx
        // for Nv Production mode.
        // This is true for RCM cases too. Hence the RCM mode check happens below.
        *SecureDebugControlVal = NV_DRF_DEF(APBDEV_PMC, DEBUG_AUTHENTICATION, DBGEN, DEFAULT) |
                                 NV_DRF_DEF(APBDEV_PMC, DEBUG_AUTHENTICATION, NIDEN, DEFAULT) |
                                 NV_DRF_DEF(APBDEV_PMC, DEBUG_AUTHENTICATION, SPIDEN, DEFAULT) |
                                 NV_DRF_DEF(APBDEV_PMC, DEBUG_AUTHENTICATION, SPNIDEN, DEFAULT) |
                                 NV_DRF_DEF(APBDEV_PMC, DEBUG_AUTHENTICATION, DEVICEEN, DEFAULT) |
                                 NV_DRF_DEF(APBDEV_PMC, DEBUG_AUTHENTICATION, JTAG_ENABLE, DEFAULT);
        return;
    }

    // If we are in RCM mode, no more processing is necessaary since it is done in
    // the NvBootRcm function.
    if(IsRcmMode)
        return;

    // If not LP0, and not RCM, then it must be a coldboot. UART and FA modes don't
    // reach this code.
    // If we are in ODM production mode or in Factory Secure Provisioning Mode,
    // enforce the ECID check.
    if(NvBootFuseIsOdmProductionMode() ||
      (NvBootFuseIsNvProductionMode() && IsFactoryProvisioningMode == NV_TRUE)  )
    {

        if( (ChipECID->ECID_0 == BctECID->ECID_0) &&
            (ChipECID->ECID_1 == BctECID->ECID_1) &&
            (ChipECID->ECID_2 == BctECID->ECID_2) &&
            (ChipECID->ECID_3 == BctECID->ECID_3) )

        {
            return;
        }

    }

    // If the above ECID check fails, disregard SecureDebugControlVal, and set
    // all debug controls to disable.
    *SecureDebugControlVal = NV_DRF_NUM(APBDEV_PMC, DEBUG_AUTHENTICATION, DBGEN, 0) |
                             NV_DRF_NUM(APBDEV_PMC, DEBUG_AUTHENTICATION, NIDEN, 0) |
                             NV_DRF_NUM(APBDEV_PMC, DEBUG_AUTHENTICATION, SPIDEN, 0) |
                             NV_DRF_NUM(APBDEV_PMC, DEBUG_AUTHENTICATION, SPNIDEN, 0) |
                             NV_DRF_NUM(APBDEV_PMC, DEBUG_AUTHENTICATION, DEVICEEN, 0) |
                             NV_DRF_NUM(APBDEV_PMC, DEBUG_AUTHENTICATION, JTAG_ENABLE, 0);
}

void
NvBootMainSecureRomExit(NvBool IsWarmBoot0,
                        NvU32 BootRomExitBranchTargetAvp,
                        NvU32 BootRomExitOptionsBitmap,
                        NvU32 MainSecureDebugControl)
{
    NvBootSku_SkuId Sku;
    NvU32 SecureRegData;
    NvU32 RegData;
    NvU32 PmcRegData;
    NvU32 PmcObsOverrideEn;
    NvU32 PmcApb2JtagOverrideEn;
#ifdef NV_DEBUG
#if NVBOOT_TARGET_FPGA
    NvU32 StackAddr;
#endif
#endif

    PROFILE();

    //Store the address of the coverage array into the MAGIC_ADDR
#ifdef ENABLE_CODECOV
    {
        NvU32 *magicPtr = (NvU32*)MAGIC_ADDRESS;
        *magicPtr = (NvU32)NvBootCodeCoverageArray;
    }
#endif

    // Check if we encountered an unrecoverable problem.  If so, then reset
    // the chip and start over.

    if (BootRomExitOptionsBitmap & NVBOOT_SECURE_EXIT_OPTION_FULLRESET) {
        NvBootResetFullChip();
    }

#ifndef TARGET_ASIM
    // prepare AES engines for bootloader and/or Recovery Mode applet; there
    // are two possible scenarios, depending on the boot type --
    //
    // 1. WB0
    //    a. compute SSK key tables and load into NVBOOT_SSK_ENGINE AES engine
    //    b. mark SSK encrypt/decrypt key slots as no-read, no-write
    //    c. set SBK key tables to zeroes in NVBOOT_SBK_ENGINE AES engine
    //    d. mark SBK encrypt/decrypt key slots as write-only
    //    e. load zero key table into all other key slots on both AES engines
    // 2. frozen/cold boot and Recovery Mode
    //    a. compute SSK key tables and load into NVBOOT_SSK_ENGINE AES engine
    //    b. mark SSK encrypt/decrypt key slots as write-only
    //    c. compute SBK key tables and load into NVBOOT_SBK_ENGINE AES engine
    //    d. mark SBK encrypt/decrypt key slots as write-only
    //    e. load zero key table into all other key slots on both AES engines
    //    f. Zero out SE TZRAM

    NvBootMainConfigureAesEnginesForExit(IsWarmBoot0);
#endif


    // disable access to SBK and DK fuses, depending on operating mode
    //
    // Operating Mode        SBK/DK access
    // ------------------    -------------
    // Preproduction Mode*       read
    // Failure Analysis*         none
    // Nv Production Mode        read
    // ODM Non-secure            none
    // ODM Secure                none
    //
    // * Processing for these operating modes is not performed here.  The FA
    //   Mode functionality is handled by hardware directly (the FA Fuse 
    //   permanently disables access to SBK and DK).  The Preproduction Mode
    //   functionality is performed in the non-secure section of the Boot ROM
    //   code.
    //
    // Note: Read access to the SBK and DK fuses is left enabled in NV 
    //       Production Mode so that, as a risk mitigation, we can burn and
    //       verify these fuses from a bootloader or Recovery Mode applet (i.e.,
    //       from outside the Boot ROM) so long as the chip is in NV Production
    //       Mode.  In reality, the ability the burn fuses in this mode has
    //       always existed; it's the ability to read back the fuse values and
    //       check whether the desired settings have been achieved that is
    //       enabled here.

    if ( ! NvBootFuseIsNvProductionMode() ) {
        NvBootFuseHideKeys();
    }

    // If we are in NvProductionMode/Secure provisioning mode, hide the SBK
    // if the KEY_HIDE fuse has already been burned.
    // This is to protect against an attacker from disconnecting the device
    // after the SBK has been burned but before the SECURITY_MODE fuse
    // has been burned. See the T210_Tegra_Security_GFD.docx for more details.
    if ( NvBootFuseIsNvProductionMode() && NvBootFuseIsSecureProvisionKeyHideFuseBurned() ) {
        NvBootFuseHideKeys();
    }

    // Read the SKU fuses that are understood by Boot ROM and configure the chip
    // accordingly
    NvBootFuseGetSku(&Sku);
    NvBootSkuEnforceSku(Sku);

    /**
     * Set access permissions for the SSK depending on boot type
     * and method used to save/restore SSK. 
     * For WB0, both read- and write-access are disabled.
     * For other boot types (i.e., frozen/cold boot and Recovery Mode),
     * only read access is disabled.
     */
    NvBootSskLockSsk(IsWarmBoot0);    

    // Optionally zero out the BIT for WB0 boot types
    if (BootRomExitOptionsBitmap & NVBOOT_SECURE_EXIT_OPTION_CLEAR_BIT) {
        NvBootUtilMemset( (void*) &BootInfoTable, 0, sizeof(BootInfoTable) ) ;
    }

    // Always zero out the unused bytes between the end of BIT and start of BCT
    NvBootUtilMemset( (void*) ( (NvU32) &BootInfoTable + sizeof(BootInfoTable)), 0,
                      (NvU32) &BootConfigTable - (NvU32) &BootInfoTable -
                      sizeof(BootInfoTable) ) ;
    
    // Optionally zero out the BCT for WB0 and forced Recovery boot types,
    if (BootRomExitOptionsBitmap & NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT) {
        NvBootUtilMemset( (void*) &BootConfigTable, 0, sizeof(BootConfigTable) ) ; 
    }

    // Always zero out the unused bytes between the end of ExcpLoop and the dummy
    // alignment variable
    NvBootUtilMemset( (void*) ( (NvU32) &ExcpLoop + sizeof(ExcpLoop)), 0,
                      (NvU32) &DummyAlignedStartOfClearArea - (NvU32) &ExcpLoop -
                      sizeof(ExcpLoop) ) ;

    // always clean the area of memory starting with the EXEC_IRAM_RW_BULK
    // all the way to the end of the stack section ARM_LIB_STACK
    // the final part of the code is mostly in assembly to have explicit 
    // control of register allocations so to avoid problems with zeroing
    // information that would still be needed.  Also this allows for a very
    // efficient coding (8 registers at a time)
    
    // the wipe out code requires 32 bytes alignment of the start address, 
    // this is enforced by suitable alignment attributes

    SecureRegData = (SB_CSR_0_PIROM_DISABLE_DISABLE << SB_CSR_0_PIROM_DISABLE_SHIFT)        | 
                    (SB_CSR_0_SECURE_BOOT_FLAG_DISABLE << SB_CSR_0_SECURE_BOOT_FLAG_SHIFT)  ;

#ifdef NV_DEBUG
#if NVBOOT_TARGET_FPGA
    // before wiping out memory, we scan it to detect the maximum size of the stack
    // this is only performed for debug FPGA.  The value found is written in the 
    // MATCH_VALUE register of the RTC (RTC clock is automatically started at reset)
    for (StackAddr = NVBOOT_MAIN_TOP_OF_STACK - NVBOOT_MAIN_STACK_SIZE;
         NV_READ32(StackAddr) == 0; 
         StackAddr += 4) 
    NV_WRITE32(NV_ADDRESS_MAP_RTC_BASE + APBDEV_RTC_SECONDS_ALARM0_0, StackAddr) ;
#endif
#endif

    // Unconditionally clear AHB_AHB_SPARE_REG_0_APB2JTAG_OVERRIDE_EN and
    // AHB_AHB_SPARE_REG_0_OBS_OVERRIDE_EN at every BR exit except for LP0.
    // Unconditionally clear the associated bits in PMC scratch
    // (i.e. APBDEV_PMC_SCRATCH49_0[1:0]) at every BR exit except for LP0.
    // If LP0, read the values to be set for APB2JTAG_OVERRIDE_EN and
    // OBS_OVERRIDE_EN from PMC scratch.
    // Use a Read-Modify-Write.
    RegData = NV_READ32(NV_ADDRESS_MAP_AHB_ARBC_BASE +
                        AHB_AHB_SPARE_REG_0);
    PmcRegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH49_0);

    if(IsWarmBoot0)
    {
        PmcObsOverrideEn =      NV_DRF_VAL(APBDEV_PMC,
                                           SCRATCH49,
                                           AHB_SPARE_REG_0_OBS_OVERRIDE_EN,
                                           PmcRegData);
        PmcApb2JtagOverrideEn = NV_DRF_VAL(APBDEV_PMC,
                                           SCRATCH49,
                                           AHB_SPARE_REG_0_APB2JTAG_OVERRIDE_EN,
                                           PmcRegData);

        RegData = NV_FLD_SET_DRF_NUM(AHB,
                                     AHB_SPARE_REG,
                                     OBS_OVERRIDE_EN,
                                     PmcObsOverrideEn,
                                     RegData);
        RegData = NV_FLD_SET_DRF_NUM(AHB,
                                     AHB_SPARE_REG,
                                     APB2JTAG_OVERRIDE_EN,
                                     PmcApb2JtagOverrideEn,
                                     RegData);
    }
    else
    {
        RegData = NV_FLD_SET_DRF_NUM(AHB,
                             AHB_SPARE_REG,
                             APB2JTAG_OVERRIDE_EN,
                             0,
                             RegData);
        RegData = NV_FLD_SET_DRF_NUM(AHB,
                             AHB_SPARE_REG,
                             OBS_OVERRIDE_EN,
                             0,
                             RegData);

        // Clear scratch registers for these two OBS bits when not in LP0 path
        // as they may have undefined value at power on reset.
        // See http://nvbugs/1324739/27.
        PmcRegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                        SCRATCH49,
                                        AHB_SPARE_REG_0_OBS_OVERRIDE_EN,
                                        0,
                                        PmcRegData);
        PmcRegData = NV_FLD_SET_DRF_NUM(APBDEV_PMC,
                                        SCRATCH49,
                                        AHB_SPARE_REG_0_APB2JTAG_OVERRIDE_EN,
                                        0,
                                        PmcRegData);
        NV_WRITE32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH49_0, PmcRegData);
    }
    NV_WRITE32(NV_ADDRESS_MAP_AHB_ARBC_BASE +
                        AHB_AHB_SPARE_REG_0, RegData);

    // Arc Disable used to be here. Bug 1448250 requires us to leave ARC open
    // during RCM and Coldboot for TSEC use.

    //BootROM patch clean-up
    NvBootIRomPatchCleanup();
    //Redirect AVP exception vectors to IRAM
    NvBootMainRedirectExcp();

    BootInfoTable.BootTimeLog.NvBootTimeLogExit = SecureRegData;
    // Stash the value of MainSecureDebugControl in BootInfoTable.BootRomVersion.
    // The exit code in boot0.ss will write the correct Boot ROM version before BR
    // exit.
    BootInfoTable.BootRomVersion = MainSecureDebugControl;

#ifndef TARGET_ASIM // WDT timer is not supported by ASIM yet
    //Reload watchdog counter before handing off to bootloader
    NvBootWdtReload(WDT_TIMEOUT_VAL_NONRCM);
#endif

    // NOTE: this routine should never return
    NvBootMainAsmSecureExit(BootRomExitBranchTargetAvp,
                            (NvU32) &DummyAlignedStartOfClearArea,
                            NV_ADDRESS_MAP_IRAM_A_BASE + NVBOOT_MAIN_IRAM_SIZE,
                            (NvU32) &(BootInfoTable.BootTimeLog.NvBootTimeLogExit)) ;

    // if we got here, something went terribly wrong ... reset the chip
    NvBootResetFullChip();
}


void
NvBootMainSecureRomEnter(void)
{
    NvBootError status = NvBootError_None;
    NvU32 BootRomExitBranchTargetAvp;
    NvU32 BootRomExitOptionsBitmap = 0;
    NvBool IsWarmBoot0 = NV_FALSE;
    NvBool IsForceRcmByStrap = NV_FALSE;
    NvBool IsForceRcmByPmc = NV_FALSE;
    NvBool IsForceRcm = NV_FALSE;
    NvBool IsRcmMode = NV_FALSE;
    NvBool HaltAtWb0 = NV_FALSE;
    NvBool IsFactorySecureProvisioningMode = NV_FALSE;
    NvBool IsFactorySecureProvisioningModeRcm = NV_FALSE;
    // MainSecureDebugControl should default to all debug controls disabled:
    NvU32 MainSecureDebugControl = APBDEV_PMC_DEBUG_AUTHENTICATION_0_SW_DEFAULT_VAL;
    volatile NvU32 HaltValue = 0xBEEFBEEF;
    NvBootRCMPort_T *pRcmPort;
    NvBootRCMPortID_T RcmPortId;
    NvBootECID ChipECID;
    NVBOOT_MSG((Q_MAIN_SECURE_ROM_ENTER));

    if((NvBootFuseIsOdmProductionMode()) && (NvBootFuseIsWatchdogEnabled()))
    {
        //odm production mode and watchdog enable fuse are set
        NvBootWdtInit();
        NvBootWdtStart();
    }
    // first, check to see if a Warm Boot 0 is in progress
    // save the warm boot flag in a global variable
    IsWarmBoot0 = NvBootPmcQueryFlag(NvBootPmcFlagId_Wb0);

    // Move boot init timestamp into its rightful place holder
    BootInfoTable.BootTimeLog.NvBootTimeLogInit = NV_READ32(NV_ADDRESS_MAP_IRAM_A_BASE + 0x100);

    PROFILE();

#ifndef TARGET_ASIM
    /*
     * Regardless of whether entering cold boot or forced recovery or warm boot,
     * set up the AES engines appropriately.
     */
    NvBootMainSetupAesEngines();
#endif


    if ( IsWarmBoot0 ) {

        // process Warm Boot
        // return status code and boot rom exit criteria
        // if status code shows an LP0 context restore failure, exiting to
        // LP0 recovery code is allowed and possible (assuming LP0 recovery code
        // is validated correctly). Make sure that BR does not call SE API
        // functions which require SW polling if SE context restore fails.
        // SW polling functions will fail since reads to SE_STATUS will always
        // return BUSY when the SE is disabled by BR if SE context restoration
        // failed.
        // For T124, the check for SE context restore failure at RSA key slot
        // clearing has been removed since we should never clear any of the SE
        // contents at LP0 because it should have been restored at by BR during
        // SE context restore. We keep the check here for SE context restore
        // failure since we want to allow LP0 resume even if the SE context
        // restore failed.
        //
        // if status code shows an error, reset chip; else exit Secure
        // Section of iROM cleanly
        //

        // Check if the HaltAtWb0 flag is set. This flag is only effective
        // in Preproduction mode.
        HaltAtWb0 = NvBootPmcQueryFlag(NvBootPmcFlagId_HaltAtWb0);

        // If HaltAtWb0 flag in SCRATCH0 is set, and we are in
        // Preproduction mode, halt to allow JTAG attachment.
        if (HaltAtWb0 && NvBootFuseIsPreproductionMode())
        {
            // Clear the HaltAtWb0 flag.
            NvBootPmcSetFlag(NvBootPmcFlagId_HaltAtWb0, NV_FALSE);

            while(HaltValue)
                ;
        }

        status = NvBootWb0WarmBoot(&BootRomExitBranchTargetAvp);

        // Set status to NvBootError_Success to exit the BR normally if
        // SE context restore failed.
        // All other errors, force chip reset.
        if (status == NvBootError_SE_Context_Restore_Failure)
        {
            status = NvBootError_Success;
        }

        if (status != NvBootError_Success) {
            NvBootResetFullChip();
        }
        // no need for an else, chip is dead

        // For ARC lock-down
        BootInfoTable.SdramInitialized = NV_TRUE;

        // LP0 exit destination can be IRAM or SDRAM
        // Disable and lock down ARC unconditionally
        NvBootArcDisable(NV_ADDRESS_MAP_EMEM_BASE);
    }
    else
    {

        /* Set the version and boot information in the BIT */
        BootInfoTable.BootRomVersion = NVBOOT_BOOTROM_VERSION;
        BootInfoTable.DataVersion    = NVBOOT_BOOTDATA_VERSION;
        BootInfoTable.RcmVersion     = NVBOOT_RCM_VERSION;
        BootInfoTable.PrimaryDevice  = NvBootDevType_Irom;
        BootInfoTable.OscFrequency   = NvBootClocksGetOscFreq();
        BootInfoTable.SafeStartAddr  = PTR_TO_ADDR(&BootInfoTable) +
        sizeof(NvBootInfoTable);
 
        /*
         * must be a cold boot or forced recovery
         */
        /*
         * Check if Recovery Mode is being forced because
         * * flag in PMC Scratch0 register is set
         * OR
         * * Boot Source strap is pulled high
         */
        
        IsForceRcmByStrap = NvBootStrapIsForceRecoveryMode();

        IsForceRcmByPmc = NvBootPmcQueryFlag(NvBootPmcFlagId_ForceRecovery);

        // Initialize USB HW controller. This will be required for usb charging
        // and/or usb2 and/or usb3.
#if !NVBOOT_TARGET_QT
#ifndef TARGET_ASIM /* USB is not supported by ASIM yet */

        // Read RCM Port fuse. Default is Synopsys USB Port
        NvBootFuseGetRcmPort(&RcmPortId);

        // Register functions with RCM Port handler
        status = NvBootRcmSetupPortHandle(RcmPortId);
        if(status != NvBootError_Success)
        {
            // Set the BL to the dummy BL that spins.
            BootRomExitBranchTargetAvp =
                PTR_TO_ADDR(NvBootMainNonsecureBootLoader);

            // Mark the boot as a success to ensure a proper exit.
            status = NvBootError_Success;
        }

        pRcmPort = NvBootRcmGetPortHandle();

        if(pRcmPort->Init()== NvBootError_Success)
        {
            pRcmPort->Context.Initialized = NV_TRUE;
            // Enable USB Charger detection
            NvBootUsbChargingInit();
        }
        
#endif // TARGET_ASIM
#endif // NVBOOT_TARGET_QT
        if ( IsForceRcmByStrap || IsForceRcmByPmc )
        {
            // enter Recovery Mode
            // return status code and boot rom exit criteria
            //
            // Upon return, exit Secure Section of iROM cleanly

            // Force Recovery state machine calls for the PMC flag to be 
            // cleared on the first attempt at Recovery Mode.  

            IsForceRcm = NV_TRUE;

            if ( IsForceRcmByPmc )
            {
                NvBootPmcSetFlag(NvBootPmcFlagId_ForceRecovery, NV_FALSE);
                BootInfoTable.ClearedForceRecovery = NV_TRUE;
            }

            // If this is a Forced Recovery, AND
            // Debug RCM is forced, AND
            // ODM production mode is not set,
            // do NOT enter RCM and exit BR immediately.
            //
            // Implicitly, in a chip mode that's NOT ODM Production mode, the
            // Boot ROM will automatically enable JTAG upon exit.
            if ( ( IsForceRcmByStrap ) &&
                 ( NvBootStrapIsDebugRecoveryMode()) &&
                 (!NvBootFuseIsOdmProductionMode() )
               )
            {
                // Set the BL to the dummy BL that spins.
                BootRomExitBranchTargetAvp =
                    PTR_TO_ADDR(NvBootMainNonsecureBootLoader);

                // Set the reason why Boot ROM is exiting in BIT.
                BootInfoTable.BootType = NvBootType_ExitRcm;

                // Mark the boot as a success to ensure a proper exit.
                status = NvBootError_Success;
            }
            else
            {
                status = NvBootRcm(NV_TRUE, &IsFactorySecureProvisioningModeRcm, &MainSecureDebugControl, &BootRomExitBranchTargetAvp);
            }

            if (status != NvBootError_Success)
            {
                // Set the BL to the dummy BL that spins.
                BootRomExitBranchTargetAvp = 
                    PTR_TO_ADDR(NvBootMainNonsecureBootLoader);

                // Mark the boot as a success to ensure a proper exit.
                status = NvBootError_Success;
            }
        }
        else
        {
            // must be a frozen/cold boot
            //
            // attempt to load Boot Loader
            // return status code and boot rom exit criteria
            //
            // if status code shows an error, enter Recovery Mode;
            // else exit Secure Section of iROM cleanly

            status = NvBootColdBoot(&BootRomExitBranchTargetAvp);

            if ( ( status != NvBootError_Success ) )
            {

                // enter Recovery Mode
                // return status code and boot rom exit criteria
                //
                // Upon return, exit Secure Section of iROM cleanly

                status = NvBootRcm(NV_FALSE, &IsFactorySecureProvisioningModeRcm, &MainSecureDebugControl, &BootRomExitBranchTargetAvp);

                if (status != NvBootError_Success)
                {
                    // Set the BL to the dummy BL that spins.
                    BootRomExitBranchTargetAvp =
                        PTR_TO_ADDR(NvBootMainNonsecureBootLoader);

                    // Mark the boot as a success to ensure a proper exit.
                    status = NvBootError_Success;
                }
            }
        }
    }

    // All paths that reach here ensure that status is Success.
    NV_ASSERT(status == NvBootError_Success);

    // All paths that reach this point have status == NvBootError_Success.
    // The code that follows has been left in place as a precaution against
    // future modifications that break the assumption and are not caught when
    // testing debug builds.

    // if any kind of error occurred and we weren't able to fix it before we
    // got here, then there's no way to recover; reset the chip and try again
    if ( status != NvBootError_Success ) {
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_FULLRESET;
    }

    // Set a flag if the path was RCM.
    IsRcmMode = (BootInfoTable.BootType == NvBootType_Recovery) ? NV_TRUE : NV_FALSE;

    // The Boot Configuration Table (BCT) and Boot Information Table (BIT)
    // data is left undisturbed in memory after the Boot ROM exits, under
    // some circumstances as defined below --
    //
    // Boot Type                   BIT             BCT
    // ----------------------      ------          ------
    // Pre-producion*              valid           zeroed
    // Failure Analysis*           valid           zeroed
    // Warm Boot 0                 zeroed          zeroed
    // Cold Boot                   valid           valid
    // Forced Recovery Mode        valid           zeroed
    // Unforced Recovery Mode      valid           present, valid or invalid
    //
    // * Processing for these boot types is not performed here; it's performed
    //   in the non-secure section of the Boot ROM code
    //
    // Note: an Unforced Recovery occurs when an error prevents a Cold Boot
    //       from succeeding.  In this case, it is not possible to guarantee
    //       that the BCT is valid, however it is left in memory intact in
    //       case it will be useful in identifying why the Cold Boot failed.

    if ( IsForceRcm )
    {
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT;
    }
    
    if ( IsWarmBoot0 )
    {
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_CLEAR_BIT;
        BootRomExitOptionsBitmap |= NVBOOT_SECURE_EXIT_OPTION_CLEAR_BCT;
    }

    // If the BCT was validated successfully (this only happens in a successful coldboot),
    // conditionally accept the BCT value of SecureDebugControl.
    // The value of MainSecureDebugControl will be processed in
    // NvBootMainProcessSecureDebugControl.
    // If this is an RCM boot, the MainSecureDebugControl values will be changed
    // in NvBootRcm().
    // If the BCT is valid, check if we are in factory secure provisioning mode via
    // the BCT secure provisioning key number.
    // If we don't have the BCT, check if the anti-cloning key has been burned.
    // Only take the BCT value of SecureDebugControl at coldboot. Don't use the BCT
    // value in the case where BCT is valid, BL is not valid, BR goes to RCM and returns
    // via a Download_Execute command.
    if ( BootInfoTable.BctValid && (BootInfoTable.BootType == NvBootType_Cold))
    {
        MainSecureDebugControl = BootConfigTable.SecureDebugControl;

        if(NvBootFuseIsSecureProvisioningMode(BootConfigTable.SecProvisioningKeyNum_Secure) == NvBootError_SecProvisioningEnabled)
        {
            IsFactorySecureProvisioningMode = NV_TRUE;
        }
    }

    // If the Bct isn't valid or this isn't a coldboot, then the only other
    // possiblity that we are in secure provisioning mode is via RCM.
    IsFactorySecureProvisioningMode |= IsFactorySecureProvisioningModeRcm;

    NvBootFuseGetUniqueId(&ChipECID);

    // Process MainSecureDebugControl value.
    NvBootMainProcessSecureDebugControl(&MainSecureDebugControl,
                                        &ChipECID,
                                        &BootConfigTable.UniqueChipId,
                                        IsWarmBoot0,
                                        IsRcmMode,
                                        IsFactorySecureProvisioningMode);

    // flush all secure information in preparation for exiting Secure Section
    // of boot ROM --
    // 1. disable access to SBK and DK fuses
    // 2. disable read access to SSK always-on register
    // 3. flush SBK from SE key slot used for AES CMAC hashing
    // 4. over-write the iRAM area containing Boot ROM variables and code
    // 5. Enable/Disable JTAG
    //
    // exit Secure Section of iROM, disable Secure Section of iROM, then set
    // up chip according to desired exit criteria
    // NOTE: this routine should never return
    NvBootMainSecureRomExit(IsWarmBoot0,
                            BootRomExitBranchTargetAvp,
                            BootRomExitOptionsBitmap,
                            MainSecureDebugControl);

    // if we got here, something went terribly wrong ... reset the chip
    NvBootResetFullChip();
}

void
NvBootMainSecureRomEnterBeforeScatter()
{
    NvBootClocksOscFreq OscFreq;
    NvU32 RegData, pllPStableTime;

    // Assert PLL*_OUT*_RSTN field to PLL*_OUT* divider. This step
    // must be done regardless if starting up PLLP manually or via auto-restart.
    // This step must be done before startup of the PLL. The reset disable
    // will be done in NvBootClocksIsPllStable.
    // Enable OUTx divider reset (Bug 954159)
    // 0 - Reset Enable, 1 - Reset Disable
    NvBootClocksPllDivRstCtrl(NvBootClocksPllId_PllP, 0);

    // Only start PLLP if it did not happen automatically on a warm boot.
    // Note that GetOscFreq() will always return a reasonable value.
    if (!NvBootPmcIsPllpOverrideEnabled())
    {
        OscFreq = NvBootClocksGetOscFreq();
        NvBootClocksStartPllpBeforeScatterLoad(OscFreq);
    }
    // If the above NvBootClocksStartPllpBeforeScatterLoad function was not
    // called, it is assumed that PLLP has been auto-started by
    // programming the appropriate registers in PMC. In this case, the
    // hardware will sequence the PLL except for enabling the lock detection
    // circuitry.

    // Make sure the lock detection circuitry is enabled. This step is for
    // safety only, as the lock detection circuitry should already be enabled
    // at POR for the PLLs that can be auto-restarted (i.e. PLLP/U/M; i.e. POR reset
    // value for PLLP_EN_LCKDET is ENABLE).
    RegData = NV_READ32(NV_ADDRESS_MAP_CAR_BASE +
                        CLK_RST_CONTROLLER_PLLP_MISC_0);
    RegData = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER,
                         PLLP_MISC,
                         PLLP_EN_LCKDET,
                         ENABLE,
                         RegData);
    NV_WRITE32(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_MISC_0,
               RegData);

    pllPStableTime = NV_READ32(NV_ADDRESS_MAP_TMRUS_BASE +
                               TIMERUS_CNTR_1US_0) + NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY;
    // Poll for PLLP lock bit and timeout on polling loop
    while (!NvBootClocksIsPllStable(NvBootClocksPllId_PllP, pllPStableTime));

    NvBootClocksSetAvpClockBeforeScatterLoad();
}


#pragma arm section rodata = "MainNonsecure", code = "MainNonsecure"

// Initial clock setup needed by all paths in the Boot ROM.
static NvBootClocksOscFreq
NvBootMainNonsecureConfigureClocks(void)
{
    NvBootClocksOscFreq OscFreq;

    // Enable the pmc clock to acess the pmc scratch register
    NvBootClocksSetEnable(NvBootClocksClockId_PmcId, NV_TRUE);

    if (NvBootPmcIsPllpOverrideEnabled())
    {
        // The override is enabled, so only copy the OscFreq to CAR.
        OscFreq = NvBootPmcGetPllpOverrideOscFreq();
    }
    else
    {
        // The override is not enabled, so measure the oscillator,
        // store it in CAR, store it in the Pllp Override fields.
        // PLLP will actually be started later if needed.
        OscFreq = NvBootClocksMeasureOscFreq();

        // If the frequency is unknown, we assume 38.4 MHz, the primary
        // oscillator frequency expected on T210 platforms.
        if (OscFreq == NvBootClocksOscFreq_Unknown)
        {
            OscFreq = NvBootClocksOscFreq_Default_If_Unknown;
        }

        // Set up the BootROM portion of the PllpOverride.
        NvBootPmcSetPllpOverride(NV_TRUE, OscFreq);
    }

    // Store the oscillator frequency in CAR.
    NvBootClocksSetOscFreq(OscFreq);

    // Configure the microsecond timer.
    NvBootClocksConfigureUsecTimer(OscFreq);

    /**
     * Set up PLLU auto-restart on WB0. Note Boot ROM is only
     * responsible for enabling the PLL auto-restart via PLLx_ENABLE. 
     * PLLx_OVERRIDE_ENABLE is the domain of the BL, OS, or LP0 entry code. 
     * Assumes OscFreq has the correct value
     * If PllU override is enabled, then do nothing. 
     */
    if (!NvBootPmcIsPlluOverrideEnabled()) 
    {
        NvBootPmcSetPlluOverride(NV_TRUE, OscFreq);
    }


    return OscFreq;
}

void
NvBootMainNonsecureRomEnter(void)
{
    NvBootClocksOscFreq OscFreq;
    NvU32 RegData;
    NvBool IsPreproductionUartBoot = NV_FALSE;
    NvBool IsWarmBoot0 = NV_FALSE;

    OscFreq = NvBootMainNonsecureConfigureClocks();
    
    RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH0_0);
    IsWarmBoot0 = (RegData & NvBootPmcFlagId_Wb0) ? NV_TRUE: NV_FALSE;

    // Bring the pads out of Deep Power Down.  By doing this here,
    // instead of the warmboot code, the Boot ROM can use any of
    // the peripherals with impunity.
    NvBootPadsExitDeepPowerDown(IsWarmBoot0);

    // check for Failure Analysis (FA) Mode
    //
    // FA Mode is the highest priority; it overrides all other modes
    // In FA and Preproduction, the only possible path is to the UART
    // bootloader
#if NVBOOT_TARGET_QT
    // Bit 29:26 are BOOT_SELECT. Bit 9 is BOOT_FAST_UART.
    // Use UART boot and select slow UART
    NV_WRITE32(NV_ADDRESS_MAP_APB_MISC_BASE +
                    APB_MISC_PP_STRAPPING_OPT_A_0, 0x0); 
#endif
    if ( NvBootFuseIsFailureAnalysisMode() || NvBootFuseIsPreproductionMode() )
    {
        if (NvBootFuseIsPreproductionMode())
        {
            // Ignore strap on LP0 exit
            RegData = NV_READ32(NV_ADDRESS_MAP_PMC_BASE + APBDEV_PMC_SCRATCH0_0);
            if (RegData & NvBootPmcFlagId_Wb0)
                return;
            // Or Enter UART Boot if BOOT_SELECT = NvBootStrapDevSel_UartBoot_Preproduction
            RegData = NV_READ32(NV_ADDRESS_MAP_APB_MISC_BASE +
                    APB_MISC_PP_STRAPPING_OPT_A_0);
            if (NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A,
                           BOOT_SELECT, RegData) == NvBootStrapDevSel_UartBoot_PreProduction)
                IsPreproductionUartBoot = NV_TRUE;

            if (IsPreproductionUartBoot == NV_FALSE)
                return;

        }

        // By design in RTL, the Protected PIROM cannot be read when the FA
        // fuse is blown. However, we must ensure that the SB_PIROM_START_0
        // register is also write protected to ensure that the Protected
        // ROM start address is not programmed to some non-expected value
        // by a mallicious entity such that the Protected IROM can be read even
        // though we are in FA mode and the PRIOM_DISABLE bit is set.
        //
        // The SB_PIROM_START_0 register is only programmable while
        // SECURE_BOOT_FLAG is ENABLE. Thus, in FA mode, before downloading
        // and executing the UART payload, set SECURE_BOOT_FLAG to DISABLE
        // and PIROM_DISABLE to DISABLE. Setting SECURE_BOOT_FLAG to DISABLE
        // write protects SB_PIROM_START_0, making it impossible to change the
        // start address to some non-expected value such that the PIROM is
        // able to be read. In addition, PIROM_DISABLE is set to DISABLE to
        // disable the reading of the PIROM region.
        //
        // Any entity should not be able to read the protected IROM except for
        // the Boot ROM starting in T132. This is to protect the MTS Decryption
        // Key which will be stored in IROM for T132.
        // See http://nvbugs/1185573 and http://nvbugs/1034867.
        //
        // Notes: Setting PIROM_DISABLE is just a guarantee mechanism. If the
        // FA fuse is burnt, PIROM shouldn't be readable regardless.
        if (NvBootFuseIsFailureAnalysisMode())
        {
            RegData = NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0);
            RegData = NV_FLD_SET_DRF_DEF(SB,
                                         CSR,
                                         PIROM_DISABLE,
                                         DISABLE,
                                         RegData);
            RegData = NV_FLD_SET_DRF_DEF(SB,
                                         CSR,
                                         SECURE_BOOT_FLAG,
                                         DISABLE,
                                         RegData);
            NV_WRITE32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_CSR_0, RegData);
        }

        // boot from UART
        // NOTE: this routine should never return
        NvBootUartDownload(OscFreq);

        // if we got here, something went terribly wrong ... reset the chip
        NvBootResetFullChip();
    }
}

/*
 * NvBootMainNonsecureBootLoader() provides a safe place for the boot
 * process to land when RCM fails.  Depending upon operating mode, JTAG
 * may be enabled at this point, permitting debugging.
 *
 * @retval none, this routine never returns
 */
static void
NvBootMainNonsecureBootLoader(void)
{
    NV_BOOT_SPIN_WAIT();
}

// for cleanliness restore default sections
#pragma arm section

static const NvU32 EvpTable[] = {
//    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_RESET_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_UNDEF_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_SWI_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_PREFETCH_ABORT_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_DATA_ABORT_VECTOR_0,
//    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_RSVD_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_IRQ_VECTOR_0,
    NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_FIQ_VECTOR_0
};

/*
 * NvBootMainRedirectExcp() redirects AVP exception vectors to spiloop
 * in IRAM.
 *
 * @retval none
 */
static void
NvBootMainRedirectExcp(void)
{
    NvU32 i;
    /* ARM instruction for B . */
    ExcpLoop = 0xEAFFFFFE;
    i = 0;
    do {
	NV_WRITE32(EvpTable[i], (NvU32)&ExcpLoop);
    }while (EvpTable[i++] != (NV_ADDRESS_MAP_VECTOR_BASE + EVP_COP_FIQ_VECTOR_0));
}

