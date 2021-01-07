/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#include "nvboot_car_int.h"
#include "nvboot_clocks_int.h"
#include "project.h"
#include "arclk_rst.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include "nvboot_error.h"
#include "nvboot_osc.h"
#include "nvrm_drf.h"

/**
 *  Checks if given address is within clock domain
 */
static uint32_t NvBootClocksIsValidAddr(uint32_t Addr)
{
    if(Addr < NV_ADDRESS_MAP_CAR_BASE || Addr > NV_ADDRESS_MAP_CAR_LIMIT)
    {
        return  NV_FALSE;
    }
    return NV_TRUE;
}

// Cmds engine
NvBootError NvBootClocksProcessInst(ClockInst table[])
{
    uint32_t Opcode, Addr, Reg, i=0;
    Set *SetInst;
    FieldSet *FSInst;
    Poll *PollInst;
    Wait *WaitInst;
    
    if(table==NULL)
        return NvBootError_InvalidParameter;

    // Tables are Null instruction terminated.
    // Null instruction is all 0s.
    while((Opcode = table[i][0]))
    {
        // Clock Instructions
        if(!(Opcode & PLL_INST_MASK))
        {
            switch(Opcode)
            {
                case Clk_W:
                    SetInst = (Set*)&table[i][0];
                    
                    Addr = NV_ADDRESS_MAP_CAR_BASE+SetInst->Offset;
                    // Ensure Addr is within clock domain
                    if(!NvBootClocksIsValidAddr(Addr))
                    {
                        return  NvBootError_IllegalParameter;
                    }

                    NV_WRITE32(Addr, SetInst->Value);
                    NvBootUtilWaitUS(2);
                break;
                case Clk_Rmw:
                    FSInst = (FieldSet*)&table[i][0];
                    
                    Addr = NV_ADDRESS_MAP_CAR_BASE+FSInst->Offset;
                    // Ensure Addr is within clock domain
                    if(!NvBootClocksIsValidAddr(Addr))
                    {
                        return  NvBootError_IllegalParameter;
                    }

                    Reg = NV_READ32(Addr);
                    Reg &= ~(FSInst->Mask);
                    Reg |= FSInst->Value;
                    NV_WRITE32(Addr, Reg);
                    NvBootUtilWaitUS(2);
                break;
                case Poll_Fld:
                    PollInst = (Poll*)&table[i][0];
                    Addr = NV_ADDRESS_MAP_CAR_BASE+PollInst->Offset;
                    // Ensure Addr is within clock domain
                    if(!NvBootClocksIsValidAddr(Addr))
                    {
                        return  NvBootError_IllegalParameter;
                    }
                    NvBootError e = NvBootPollField(Addr, PollInst->Mask, PollInst->Value, PollInst->Time);
                    // This will stop executing all instructions and return.
                    // Exercise caution in choosing timeout value.
                    if(e!=NvBootError_Success)
                    {
                        return e;
                    }
                break;
                case Wait_us:
                    WaitInst = (Wait*)&table[i][0];
                    NvBootUtilWaitUS(WaitInst->Time);
                break;
                default:
                    return NvBootError_InvalidParameter;
            }
        }
        else // PLL Instructions
        {
            if(Opcode  == Pll_Start)
            {
                PllOps *PllInst = (PllOps*)(&table[i][0]);
                // Stable time is not used. Used to simply satisfy the wrapper
                NvU32 StableTime;
                NvBootClocksStartPll(PllInst->Id,
                     PllInst->M,
                     PllInst->N,
                     PllInst->P,
                     PllInst->Misc1,
                     PllInst->Misc2,
                     &StableTime);
            }
        }
        i++; // Next Instruction
    }
    return NvBootError_Success;
}

NvBootError NvBootClocksEngine(void *table, ClockTableType TableOrList)
{
    NvBootError e = NvBootError_Success;
    ClockTable *Tables;
    uint32_t i=0;

    if(!table)
        return NvBootError_InvalidParameter;
    
    switch(TableOrList)
    {
        case TYPE_SINGLE_TABLE:
            // Single table
            e = NvBootClocksProcessInst((ClockInst*)table);
        break;
        case TYPE_MULTI_TABLE:
            // List of pointer to Tables (Null Terminated)
            
            Tables = (ClockTable*)table;
            while(Tables[i])
            {
                e = NvBootClocksProcessInst(Tables[i]);
                if(e != NvBootError_Success)
                    break;
                i++; //Next table
            }
        break;
        default:
            return NvBootError_InvalidParameter;
    }
    return e;
}
