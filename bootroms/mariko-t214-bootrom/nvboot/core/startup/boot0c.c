#include <nvboot_bit.h>
#include <nvboot_section_defs.h>
#include <nvboot_dispatcher_int.h>
#include <nvboot_context_int.h>
#include "nvboot_pmc_int.h"
#include "nvboot_strap_int.h"
#include <nvboot_error.h>
//#include "arpmc_impl.h"
//#include "armiscreg.h"
//#include "arscratch.h"
//#include "arbpmp_atcmcfg.h"
#include "arsecure_boot.h"
#include "project.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_fuse_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_reset_int.h"
#include "nvboot_util_int.h"
#include "nvboot_bpmp_int.h"
#include "nvboot_rng_int.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_fidet_int.h"
#include "nvboot_version_rom.h"

NvBootInfoTable VT_MAINBIT BootInfoTable;
// NvBootConfigTableBuffer is the buffer allocated in on-chip RAM (IRAM, SYSRAM, etc.)
// for storing the local copy of the BCT.
// It needs to be
// (sizeof(NvBootConfigTable)+max_page_size-1 / max_page_size) * max_page_size,
// in case the BCT size is not max_page_size aligned. i.e. if the BCT is 3KB,
// and the max page size is 2KB, the minimum read is 2 pages and 4KB total.
// Therefore we need a buffer size of 4KB.
NvBootConfigTableBuffer VT_MAINBCT BootConfigTable;
NvBootContext  VT_ZIRW Context;
NvBootConfigTable *pBootConfigTable;

// global counter for FI mitigation.
int32_t FI_counter1;
/**
 *  Global counter to increase code distance between error detection and response.
 *  FI mitigation technique.
 */
int32_t FI_IncrDist;

extern void NvBootMainNonSecureBootLoader();
NvBool IsRcmMode()
{
    NvBool IsForceRcmByStrap = NV_FALSE;
    NvBool IsForceRcmByPmc = NV_FALSE;
    
    /*
    * Check if Recovery Mode is being forced because
    * * flag in PMC Scratch0 register is set
    * OR
    * * RCM strap is pulled low
    */
    
    IsForceRcmByStrap = NvBootStrapIsForceRecoveryMode();
    
    IsForceRcmByPmc = NvBootPmcQueryFlag(NvBootPmcFlagId_ForceRecovery);

    if(IsForceRcmByStrap || IsForceRcmByPmc)
    {

        // set boot flow status flag.
        Context.BootFlowStatus.NvBootFlowXusbForceRcm = NV_TRUE;
        
        if ( IsForceRcmByPmc )
        {
        	NvBootPmcSetFlag(NvBootPmcFlagId_ForceRecovery, NV_FALSE);
        	BootInfoTable.ClearedForceRecovery = NV_TRUE;
        }
        BootInfoTable.BootROMtracker = NvBootFlowStatus_RecoveryMode;
        return NV_TRUE;
    }
    return NV_FALSE;
}

/**
 *  This is chip specific code really. In Parker type chips, we need pBootConfigTable pointing to
 *  a fixed location closer to end of sysram. In t210 style chips, we need it to be in IRAM as per linker allocated 
 *  space for global BCT instance.
 */
void SetBctPointers()
{
    pBootConfigTable = &(BootConfigTable.Bct);
}

/**
 * do_exception(void)
 *
 */
void trap_exception(void);
__attribute__((noreturn)) void FT_NONSECURE do_exception(void)
{
    NvBootMinimalAssetLockDownExit();
    NvBootFaultInjectDetect();
    trap_exception();
    for(;;);
}

extern void *__secure_region__;
void FT_NONSECURE NvBootSetupPiromRegion(void)
{
    uint32_t piromStart = (uint32_t)&__secure_region__;

    piromStart -= NV_ADDRESS_MAP_IROM_BASE;
    NV_WRITE32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_PIROM_START_0, piromStart);
    piromStart = NV_READ32(NV_ADDRESS_MAP_SECURE_BOOT_BASE + SB_PIROM_START_0);
}

/**
 *  Setup/Clear structures/variables not setup by runtime init.
 *  Start RNG.
 *  Everything that is needed before invoking dispatcher tables goes here.
 */
void NvBootPreDispatcherSetup()
{
    // Authentication scheme 
    NvBootFuseGetBootSecurityAuthenticationInfo(&(BootInfoTable.AuthenticationScheme));
    
    //Encryption scheme
    BootInfoTable.EncryptionEnabled = NvBootFuseBootSecurityIsEncryptionEnabled();
    
    // Set global BCT pointer to point to global BCT instance.
    SetBctPointers();
    
    NvBootMainSecureInit();
    
    // Bring up SE clocks and initialize RNG.
    NvBootCryptoMgrHwEngineInit();
}

// Main in T210 is also in boot0c.c.
// See also NvBootMainSecureRomEnter() in T210.
// WDT has been initialized in non-secure dispatcher if the conditions
// have been satisfied.
int main(void)
{
    NvBootError Error = NvBootError_Idle;
    NvBool IsRcmMode_Status = NV_FALSE;
    NvBool IsWarmBoot_Status = NV_FALSE;

    /* Check RMA status if ODM enabled this */
    NvBootCheckRMAStatus();

    /** 
     *  Everything that is needed before invoking dispatcher tables goes here.
     *  This can't be put in dispatcher because it sets up RNG which is used by dispatcher.
     */
    NvBootPreDispatcherSetup();

    /**
     *  Run Crypto Init dispatcher. We need these tasks to run regardless of earlier Error.
     *  So OR with Error of this dispatcher run. Will be handler later.
     *  Coldboot - Force RCM
     *  Warmboot - Reset.
     */

    BootInfoTable.BootROMtracker =  NvBootFlowStatus_CryptoInitEntry;
    Error = NvBootSecureDispatcher(NvBootTaskListId_CryptoInit);
    BootInfoTable.BootROMtracker = NvBootFlowStatus_CryptoInitExit;

    // Check if Warmboot
    IsWarmBoot_Status = NvBootQueryRstStatusWarmBootFlag();
    if(!IsWarmBoot_Status)
    {
        // coldboot or rcm
        /* Set the version and boot information in the BIT */
        BootInfoTable.BootRomVersion = NVBOOT_BOOTROM_VERSION;
        BootInfoTable.DataVersion	 = NVBOOT_BOOTDATA_VERSION;
        BootInfoTable.RcmVersion	 = NVBOOT_RCM_VERSION;
        BootInfoTable.PrimaryDevice  = NvBootDevType_Irom;
        BootInfoTable.OscFrequency	 = NvBootClocksGetOscFreq();
        BootInfoTable.SafeStartAddr  = PTR_TO_ADDR(&BootInfoTable) +
        sizeof(NvBootInfoTable);

        // Subsystem init for t214 is only mem clock enable. This
        // should only be called in non-sc7 paths. SC7 paths will handle
        // mem clock init itself.
        Error = NvBootBpmpSubSystemInit();

        // check for Forced RCM mode
        IsRcmMode_Status = IsRcmMode();
        
        // Cold boot function dispatcher.
        if((Error == NvBootError_Success) && !IsRcmMode_Status)
        {
            BootInfoTable.BootROMtracker = NvBootFlowStatus_ColdBootEntry;
            Error = NvBootSecureDispatcher(NvBootTaskListId_ColdBoot);
            BootInfoTable.BootROMtracker = NvBootFlowStatus_ColdBootExit;
        }

        if(((Error != NvBootError_Success) || IsRcmMode_Status))
        {
            BootInfoTable.BootROMtracker = NvBootFlowStatus_RcmEntry;
            Error = NvBootSecureDispatcher(NvBootTaskListId_Rcm);
            BootInfoTable.BootROMtracker = NvBootFlowStatus_RcmExit;
        }

        // Exit status for RCM to spin
        if (Error != NvBootError_Success)
        {
            // Set the BL to the dummy BL that spins.
            Context.BootLoader = (uint8_t*)do_exception;
            
            // Mark the boot as a success to ensure a proper exit.
            Error = NvBootError_Success;
        }

    }
    else
    {
        Context.BootFlowStatus.NvBootPreserveDram = NV_TRUE;
        Context.BootFlowStatus.NvBootRamDumpEnabled = NV_FALSE;
        BootInfoTable.BootROMtracker = NvBootFlowStatus_Sc7Entry;
        BootInfoTable.BootType = NvBootType_Sc7;

        // We might have encountered an error during the Predispatcher init or CryptoMgr Init routines.
        // Don't enter warmboot dispatcher in such cases.
        if(Error == NvBootError_Success) 
        {
        Error = NvBootSecureDispatcher(NvBootTaskListId_WarmBoot);
        }
        // Context Restore failure is ok.
        if(Error == NvBootError_SE_Context_Restore_Failure)
            Error = NvBootError_Success;

        BootInfoTable.BootROMtracker = NvBootFlowStatus_Sc7Exit;
        Context.BootFlowStatus.NvBootFlowSc7Exit = 1;

        // Reset full chip UNLESS SCRATCH_DBG_NORST_SPIN is set.
        if(Error != NvBootError_Success)
        {
            if(NvBootQueryWarmbootErrorDebugSpin())
            {
                // Infinite loop
                while(1); // Do not call into another routine that does a while(1).
                          // If this is glitched, we fall into Reset or even better Secure Exit.
            }
            NvBootResetFullChip();
        }
    }
	
	   // All paths that reach here ensure that status is Success.
    NV_ASSERT(Error == NvBootError_Success);

    if(Error != NvBootError_Success)
        Context.BootFlowStatus.NvBootFlowChipStatus = NV_TRUE;
	
    BootInfoTable.BootROMtracker = NvBootFlowStatus_SecureExitStart;
    NvBootSecureDispatcher(NvBootTaskListId_SecureExit);

    return 0;
}

