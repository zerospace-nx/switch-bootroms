/*
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */
#ifndef __CAR_H
#define __CAR_H
#include <stdint.h>
#include "nvboot_error.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_util_int.h"
#include "arclk_rst.h"

/**
 *  Macros to make CAR register names, Defines and Nums used for NV_DRF macros
 */
 
/**
 *  Make complete CAR register name
 */
#define ClkReg(Reg)  (CLK_RST_CONTROLLER_##Reg##_0)

/** Generate Mask for a field.
 *  This is also used to set bits for 1 bit length fields
 */
#define ClkRegFld(Reg, Fld) (CLK_RST_CONTROLLER_##Reg##_0_##Fld##_FIELD)
#define ClkRegMask(Reg, Fld) (CLK_RST_CONTROLLER_##Reg##_0_##Fld##_FIELD)

/** Generate names defs used in NV_DRF_DEF macro */
#define ClkRegDef(Reg, Fld, Def) (CLK_RST_CONTROLLER_##Reg##_0_##Fld##_##Def)

/** Helper Macros for OR-ing upto 3 Fields (either Num or Def)
 *  
 */
#define OR_FLD_N(_1_1, _1_2, _2_1, _2_2, _3_1, _3_2, OR,...) OR
#define OR_FLD_1(Reg, MK_MACRO, Fld1,Def1) ClkRegMask(Reg,Fld1), NV_DRF_##MK_MACRO(CLK_RST_CONTROLLER, Reg, Fld1, Def1)
#define OR_FLD_2(Reg, MK_MACRO, Fld1,Def1,Fld2,Def2) (ClkRegMask(Reg,Fld1)|ClkRegMask(Reg,Fld2)),\
                                           (NV_DRF_##MK_MACRO(CLK_RST_CONTROLLER, Reg, Fld1, Def1)|\
                                            NV_DRF_##MK_MACRO(CLK_RST_CONTROLLER, Reg, Fld2, Def2))
                                            
#define OR_FLD_3(Reg, MK_MACRO, Fld1,Def1,Fld2,Def2,Fld3,Def3) (ClkRegMask(Reg,Fld1)|ClkRegMask(Reg,Fld2)|ClkRegMask(Reg,Fld3)),\
                                                     (NV_DRF_##MK_MACRO(CLK_RST_CONTROLLER, Reg, Fld1, Def1)|\
                                                      NV_DRF_##MK_MACRO(CLK_RST_CONTROLLER, Reg, Fld2, Def2)|\
                                                      NV_DRF_##MK_MACRO(CLK_RST_CONTROLLER, Reg, Fld3, Def3))
                                           

/** Helper Macros for OR-ing upto 3 1-Bit Set Fields
 *  
 */
#define OR_N(_1,_2,_3,_4,OR,...) OR
#define OR_1(Reg, a)    (ClkRegFld(Reg, a))
#define OR_2(Reg, a,b) (ClkRegFld(Reg, a) | ClkRegFld(Reg, b))
#define OR_3(Reg, a,b,c) (ClkRegFld(Reg,a)|ClkRegFld(Reg,b)|ClkRegFld(Reg,c))
#define OR_4(Reg, a,b,c,d) (ClkRegFld(Reg,a)|ClkRegFld(Reg,b)|ClkRegFld(Reg,c)|ClkRegFld(Reg,d))


/** Make clock instructions for different opcodes
 *
 */
#define Mk_Clk_Rmw_Def(Reg, ...)    ClkReg(Reg), OR_FLD_N(__VA_ARGS__, OR_FLD_3, OR_FLD_3, OR_FLD_2,OR_FLD_2, OR_FLD_1,OR_FLD_1)(Reg, DEF, __VA_ARGS__)
#define Mk_Clk_Rmw_Num(Reg, ...)    ClkReg(Reg), OR_FLD_N(__VA_ARGS__, OR_FLD_3, OR_FLD_3, OR_FLD_2,OR_FLD_2, OR_FLD_1,OR_FLD_1)(Reg, NUM, __VA_ARGS__)
#define Mk_Clk_Rmw(Reg, Value)    ClkReg(Reg), Value
#define Mk_Clk_Src(Reg, SrcFld, SrcDef, DivFld, DivNum) Mk_Clk_Rmw_Num(Reg, SrcFld, ClkRegDef(Reg, SrcFld, SrcDef), DivFld, DivNum)
#define Mk_Poll_Def(Time,...)    Time, Mk_Clk_Rmw_Def(__VA_ARGS__)
#define Mk_Poll_Num(Time, ...)   Time, Mk_Clk_Rmw_Num(__VA_ARGS__)
#define Mk_Wait_us(Time)   Time
#define Mk_Clk_W1b(Reg, ...)    ClkReg(Reg), OR_N(__VA_ARGS__, OR_4, OR_3, OR_2, OR_1)(Reg, __VA_ARGS__)
#define Mk_Clk_W(Reg, Value)    ClkReg(Reg), Value
/**
 *  Make pll instructions
 */
// Nothing special here.
#define Mk_Pll_Start(...)   __VA_ARGS__
 
/** Add braces and make partial/complete clock instruction */
#define Inst(a, ...)    {a, __VA_ARGS__}
#define Instc(a, ...)   {a, Mk_##a(__VA_ARGS__)}

/** Helper macros for PLLs */

/**
 *  If a PLL uses KVCO, KCP and Setup, do not create a custom macro. 
 *  For all others, feel free to do so.
 */
#define MISC1_PLL_0_KVCO_RANGE            0: 0
#define MISC1_PLL_0_KCP_RANGE             2: 1
#define MISC2_PLL_0_SETUP_RANGE          23: 0

#define Misc1(Kvco, Kcp)  (NV_DRF_NUM(MISC1, PLL, KVCO, Kvco)| NV_DRF_NUM(MISC1, PLL, KCP, Kcp))
#define Misc2(Setup)      (NV_DRF_NUM(MISC2, PLL, SETUP, Setup))

/**
 *  Custom Misc1 and Misc2 MACROS for UTMI PllDiv
 */
#define Misc1_Utmi(Xtal, EnbDelay)    (NV_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, UTMIP_XTAL_FREQ_COUNT, Xtal) \
                                       | NV_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG1, UTMIP_PLLU_ENABLE_DLY_COUNT, EnbDelay))
#define Misc2_Utmi(StableCount, ActDelay)    (NV_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, UTMIP_PLL_ACTIVE_DLY_COUNT, ActDelay) \
                                              | NV_DRF_NUM(CLK_RST_CONTROLLER, UTMIP_PLL_CFG2, UTMIP_PLLU_STABLE_COUNT, StableCount))
 
 
/** Raw clock instructions 
 *  
 */

/** Set instruction. 
 *  Simply writes value to CAR offset. Does not RMW.
 */
typedef struct Set 
{
    uint32_t Opcode;
    uint32_t Offset;
    uint32_t Value;
} Set;

/** Field Set instruction. 
 *  RMW. Read, Mask and Set field.
 */
typedef struct FieldSet
{
    uint32_t Opcode;
    uint32_t Offset;
    uint32_t Mask;
    uint32_t Value;
} FieldSet;

typedef struct Poll
{
    uint32_t Opcode;
    uint32_t Time;
    uint32_t Offset;
    uint32_t Mask;
    uint32_t Value;
} Poll;

typedef struct Wait
{
    uint32_t Opcode;
    uint32_t Time;
} Wait;

typedef struct PllOps
{
    uint32_t Opcode;
    uint32_t Id;
    uint32_t M;
    uint32_t N;
    uint32_t P;
    uint32_t Misc1; // Use for packing KVCO, KCP (if applicable. Can be used in any other way)
    uint32_t Misc2; // Use for packing Setup
} PllOps;

typedef union AllInsts {
    PllOps PllOpsInst;
    FieldSet FieldInst;
    Set SetInst;
} AllInsts;

/**
 *  Defines a single Clock Instruction. It is basically an Array of ints.
 */
typedef const uint32_t ClockInst[sizeof(AllInsts)/sizeof(uint32_t)];

/**
 *   Defines a clock table.
 */
typedef ClockInst* ClockTable;

/**
 *  Attempt to disambiguate:
 *  ClockInst [] =  Single Clock Table = ClockTable
 *  ClockTable [] = List of Tables     = ClockTable*
 */
 
/**
 *  Engine needs to know what was passed in.
 *  ClockInst[] or ClockTable[]
 */
typedef enum 
{
    TYPE_SINGLE_TABLE=0,//default,
    TYPE_MULTI_TABLE
} ClockTableType;

typedef enum 
{
    UTMI_PLL
} PllIds;

#define PLL_INST_MASK   (0x80)
typedef enum ClockOps
{
    // 0 opcode is end of table
    End,
    Clk_W=1,
    // Clk_W1b is an alias for Clk_W
    Clk_W1b=Clk_W,
    Clk_Rmw,
    // Below are aliases for Clk_Rmw. Use only with helper macros.
    Clk_Rmw_Def=Clk_Rmw,
    Clk_Rmw_Num=Clk_Rmw,
    Clk_Src=Clk_Rmw,
    Wait_us,
    Poll_Fld,
    // Below are aliases. Use only with helper macros.
    Poll_Def=Poll_Fld,
    Poll_Num=Poll_Fld,
    Pll_Start=0x80,
    ClockOps_Max=0xFF
} ClockOps;

/**
 *  Processes a single table of ClockInst i.e. ClockInst []
 */
NvBootError NvBootClocksProcessInst(ClockInst*);
/**
 *  Processes a list of Tables i.e. ClockTable []
 */
NvBootError NvBootClocksEngine(void *, ClockTableType);

#endif // End #ifndef __CAR_H