/*
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvtypes.h"
#include "nvrm_drf.h"
#include "arpka1.h"
#include "nv_ref.h"
#include "nvboot_config.h"
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_crypto_param.h"
#include "nvboot_crypto_mgr_int.h"
#include "nvboot_pka_int.h"
#include "nvboot_se_int.h"
#include "nvboot_util_int.h"
#include "nvboot_fuse_int.h"


static volatile unsigned RADDR = 0;

void my_assert(int cond) {
    cond |= cond; // to get rid of compiler warning.
}

//void send_write_req(unsigned addr, uint32_t value) {
void send_write_req(unsigned addr, NvU32 value) {
    NV_WRITE32(NV_ADDRESS_MAP_PKA1_BASE + addr, value);
}

void send_read_req(unsigned addr) {
    RADDR = NV_READ32(NV_ADDRESS_MAP_PKA1_BASE + addr);
}

unsigned wait_read_result() {
    //return RTRegRd32(RADDR);
    return RADDR;
}

unsigned requireSE_Reg(unsigned addr) {
    send_read_req(addr);
    return wait_read_result();
}

int writeSE_Reg_verify(unsigned addr, unsigned value) {
    send_write_req(addr, value);
    my_assert(requireSE_Reg(addr) == value);
    return 0;
}

int writeSE_Reg(unsigned addr, unsigned value) {
    //return writeSE_Reg_verify(addr, value);
    //return send_write_req(addr, value);
    send_write_req(addr, value);
    return 0;
}

void requireMutex() {
    do {
    //} while ((requireSE_Reg(NV_SSE_SE_CTRL_PKA_MUTEX) & 0x1) == 0);
    } while ((requireSE_Reg(PKA1_CTRL_PKA_MUTEX) & 0x1) == 0);
}

void releaseMutex() {
    // Per SE Arch, use the SE_PKA1_CTRL_PKA_MUTEX_RELEASE register (offset 0x8118)
    // to release the mutex instead of writing "1" to 
    // SE_PKA1_CTRL_PKA_MUTEX (offseth 0x8114).
    // Using offset (0x8118 does not trigger a HW scrubbing sequence).
    writeSE_Reg(PKA1_CTRL_PKA_MUTEX_RELEASE, PKA1_CTRL_PKA_MUTEX_RELEASE_SE_LOCK_RELEASE);
}

NvU32
NvBootPkaGetPka0Reg(NvU32 Reg)
{
    return NV_READ32(NV_ADDRESS_MAP_PKA1_BASE + Reg);
}

void
NvBootPkaSetPka0Reg(NvU32 Reg, NvU32 Data)
{
    NV_WRITE32(NV_ADDRESS_MAP_PKA1_BASE + Reg, Data);
}

void
NvBootPkaGetMutex()
{
    requireMutex();

    // Note when debugging: Don't single step this, the time between
    // you requesting the mutex and to this disable may be longer
    // than the timeout value and the PKA may release the mutex
    // before you complete this write.
    // Set PKA to not timeout on mutex hold.
    writeSE_Reg(PKA1_CTRL_PKA_MUTEX_WDTMR, PKA1_CTRL_PKA_MUTEX_WDTMR_VAL_DISABLE);
}

void NvBootPkaReleaseMutex()
{
    releaseMutex();
}

void
NvBootPkaHwInit()
{
    /**
     * BR doesn't need to use PKA1 MUTEX for each operation. However,
     * we need to do a "mutex acquire and release" sequence at the beginning
     * of BR before any PKA1 operation. This is because "PKA1 after reset will
     * clear some internal data for security, MUTEX status will be a
     * guard for SW to ensure let HW finishing those operation before starting a HW task."
     * So, we Read PKA1_PKA_MUTEX (0x8114) register and wait for MUTEX acquisition.
     * Once MUTEX is gained, release it immediately via PKA1_CTRL_PKA_MUTEX_RELEASE (0x8118).
     *
     * Now, we can use PKA1 without MUTEX anymore.
     */
    NvBootPkaGetMutex();
    NvBootPkaReleaseMutex();

    // Per NVSE IAS 6.3.4.3 PKA1 in Cold Boot
    // [BR] Write PKA1_TRNG_CTRL twice to ensure TRNG is working in secure mode,
    // self-seeding mode.
    writeSE_Reg(PKA1_TRNG_SMODE, 0x104 | REF_NUM(PKA1_TRNG_SMODE_MAX_REJECTS, PKA1_TRNG_SMODE_MAX_REJECTS_10));
    writeSE_Reg(PKA1_TRNG_SMODE, 0x104 | REF_NUM(PKA1_TRNG_SMODE_MAX_REJECTS, PKA1_TRNG_SMODE_MAX_REJECTS_10));

}

void
NvBootPkaDisablePka()
{
    NvU32 RegData;
    RegData = NvBootPkaGetPka0Reg(PKA1_PKA1_SECURITY);
    RegData |= (PKA1_PKA1_SECURITY_SE_ENG_DIS_TRUE << 1);

    NvBootPkaSetPka0Reg(PKA1_PKA1_SECURITY, RegData);
}

void
NvBootPkaReadKeySlot(NvU32 *Data, NvU32 KeySlotNum)
{
    NvU32 i,j;
    NvU32 RegAddr;

    RegAddr = 0;

    for(i = PKA1_CTRL_KSLT_ADDR_FIELD_RSA_EXPONENT;
        i <= PKA1_CTRL_KSLT_ADDR_FIELD_RSA_R_SQUARE;
        i++)
    {
        RegAddr = NV_FLD_SET_DRF_NUM(PKA1, CTRL_KSLT_ADDR, FIELD, i, RegAddr);

        for(j = 0; j < NVBOOT_PKA1_KEYSLOT_MAX_WORD_SIZE; j++)
        {
            RegAddr = NV_FLD_SET_DRF_NUM(PKA1, CTRL_KSLT_ADDR, WORD_ADDR, j, RegAddr);
            NvBootPkaSetPka0Reg(PKA1_CTRL_KSLT_ADDR(KeySlotNum), RegAddr);
            *Data++ = NvBootPkaGetPka0Reg(PKA1_CTRL_KSLT_DATA(KeySlotNum));
        }
    }
}

void
NvBootPkaWriteKeySlot(NvU32 *Data, NvU32 KeySlotNum)
{
    NvU32 i,j;
    NvU32 RegAddr;

    RegAddr = 0;

    for(i = PKA1_CTRL_KSLT_ADDR_FIELD_RSA_EXPONENT;
        i <= PKA1_CTRL_KSLT_ADDR_FIELD_RSA_R_SQUARE;
        i++)
    {
        RegAddr = NV_FLD_SET_DRF_NUM(PKA1, CTRL_KSLT_ADDR, FIELD, i, RegAddr);

        for(j = 0; j < NVBOOT_PKA1_KEYSLOT_MAX_WORD_SIZE; j++)
        {
            RegAddr = NV_FLD_SET_DRF_NUM(PKA1, CTRL_KSLT_ADDR, WORD_ADDR, j, RegAddr);
            NvBootPkaSetPka0Reg(PKA1_CTRL_KSLT_ADDR(KeySlotNum), RegAddr);
            // If Data ptr is zero, always write zero to clear out the key slot.
            NvBootPkaSetPka0Reg(PKA1_CTRL_KSLT_DATA(KeySlotNum), Data == 0 ? 0 : *Data++);
        }
    }
}


void
NvBootPkaKeySlotSetAccess(NvU32 AccessVal, NvU32 KeySlotNum)
{
    NvBootPkaSetPka0Reg(PKA1_PKA1_KEYTABLE_ACCESS(KeySlotNum), AccessVal);
}

void
NvBootPkaClearAllKeySlots()
{
    NvU32 Slot;

    for (Slot = 0; Slot < ARSE_PKA1_NUM_KEY_SLOTS; Slot++)
    {
        NvBootPkaWriteKeySlot(0, Slot);
    }
}

void
NvBootPkaDisableAllKeySlotReadWrite()
{
    NvU32 i;
    for(i = 0; i < ARSE_PKA1_NUM_KEY_SLOTS; i++)
    {
        NvBootPkaSetPka0Reg(PKA1_PKA1_KEYTABLE_ACCESS(i), 0);

        // Per Chuang, Tzsecure is sufficient.
        NvBootPkaSetPka0Reg(PKA1_PKA1_SECURITY_PERKEY(i),PKA1_PKA1_SECURITY_PERKEY_OWNER_TZSECURE);
    }
}
