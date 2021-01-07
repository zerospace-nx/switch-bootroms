/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * \file nvboot_pka_int.h
 *
 * \brief NVIDIA PKA Engine NvBoot API
 *
 * This file defines all of the necessary API calls for Boot ROM to
 * utilize the PKA for boot. Note, PKA in this case refers to "PKA1", the
 * Elliptic PKA IP, not the "PKA0" or legacy RSA engine in the SE.
 *
 */
#ifndef INCLUDED_NVBOOT_PKA1_INT_H
#define INCLUDED_NVBOOT_PKA1_INT_H

#include "nvboot_se_pka.h"
/** 
 *  Alias XX_RANGE to XX as defined in pka manual 
 */
#define PKA1_CTRL_KSLT_ADDR_0_WORD_ADDR_RANGE       PKA1_CTRL_KSLT_ADDR_WORD_ADDR
#define PKA1_CTRL_KSLT_ADDR_0_FIELD_RANGE           PKA1_CTRL_KSLT_ADDR_FIELD
#define PKA1_CTRL_KSLT_ADDR_0_AINC_RANGE            PKA1_CTRL_KSLT_ADDR_AINC
#define PKA1_CTRL_KSLT_DATA_0_DATA_RANGE            PKA1_CTRL_KSLT_DATA_DATA
#define PKA1_CTRL_PKA_MUTEX_RR_TMOUT_0_LOCK_RANGE     PKA1_CTRL_PKA_MUTEX_RR_TMOUT_LOCK
#define PKA1_CTRL_PKA_MUTEX_RR_TMOUT_0_VAL_RANGE      PKA1_CTRL_PKA_MUTEX_RR_TMOUT_VAL



//easy way out
#define PKA_KEYSLOT_READ_ONLY_VAL 0x6


/**
 * \brief Read PKA register.
 *
 */
NvU32
NvBootPkaGetPka0Reg(NvU32 Reg);

/**
 * \brief Write PKA register.
 *
 */
void
NvBootPkaSetPka0Reg(NvU32 Reg, NvU32 Data);

/**
 * \brief Get PKA mutex.
 *
 * \note This function wait and poll until mutex is acquired.
 *
 * \note This function will disable the mutex timeout once mutex is acquired.
 *       Thus, the mutex will be held indefinitely until the acquirer releases
 *       it.
 *
 * \note If PKA Mutex is released successfully via writing PKA Mutex Register,
 *       PKA operand memory will be scrubbed automatically to make sure no confidential
 *       information left.
 *
 */
void
NvBootPkaGetMutex();

/**
 * \brief Release PKA mutex.
 *
 * \note If PKA Mutex is released successfully via writing PKA Mutex Register,
 *       PKA operand memory will be scrubbed automatically to make sure no confidential
 *       information left.
 *
 */
void
NvBootPkaReleaseMutex();

/**
 * \brief Initialize the PKA hardware with any required programming.
 *
 * \note Currently we only need to do a "mutex acquire and release".
 * \note Clocks to the SE should be enabled first.
 *
 */
void
NvBootPkaHwInit();

/**
 * \brief Disable PKA engine until the next full system reset.
 *
 */
void
NvBootPkaDisablePka();


/**
 * \brief Read out a whole PKA1 key slot.
 *
 * \param Data Must point to a buffer of size 16384 bits.
 *
 * \param KeySlotNum Must be a value from 0 to 3.
 *
 * \note  Each key slot can contain ECC or RSA key data.
 *        This function unconditionally reads out all data
 *        from the key slot. See section 6.2 of the SE_PKA_IAS.doc
 *        for the RSA and ECC key formatting.
 */
void
NvBootPkaReadKeySlot(NvU32 *Data, NvU32 KeySlotNum);

/**
 * \brief Write out a whole PKA1 key slot.
 *
 * \param Data Must point to a buffer of size 16384 bits OR
 *             be a NULL pointer. If a NULL pointer then
 *             this function will zero out the key slot.
 *
 * \param KeySlotNum Must be a value from 0 to 3.
 *
 * \note  Each key slot can contain ECC or RSA key data.
 *        This function unconditionally writes out all data
 *        to the key slot. It is up to the caller to format
 *        the subfields of Data correctly. See section 6.2
 *        of the SE_PKA_IAS.doc for the RSA and ECC
 *        key formatting.
 */
void
NvBootPkaWriteKeySlot(NvU32 *Data, NvU32 KeySlotNum);

/**
 * \brief Set keyslot access of PKA keyslot via SE_PKA1_PKA1_KEYTABLE_ACCESS(i)
 *
 * \note The code does not do a read/modify/write since the access permissions are
 *       stick to disabled.
 *
 */
void
NvBootPkaKeySlotSetAccess(NvU32 AccessVal, NvU32 KeySlotNum);

/**
 * \brief Disable read/write/use access to every key slot.
 *
 * \note Expected to only be used if SE context restore
 *       known pattern check fails.
 */
void
NvBootPkaDisableAllKeySlotReadWrite(void);

/**
 * \brief Clear all keyslot data to 0.
 */
void
NvBootPkaClearAllKeySlots();

/** The following function prototypes are originally from
 * "directChannal.h", which are exposed here so that
 * the se_pka unit test in the tests folder can use these
 * functions.
 */
void my_assert(int cond);

void send_write_req(unsigned addr, NvU32 value);

void send_read_req(unsigned addr);

unsigned wait_read_result();

unsigned requireSE_Reg(unsigned addr);

int writeSE_Reg_verify(unsigned addr, unsigned value);

int writeSE_Reg(unsigned addr, unsigned value);

void requireMutex();

void releaseMutex();

#endif
