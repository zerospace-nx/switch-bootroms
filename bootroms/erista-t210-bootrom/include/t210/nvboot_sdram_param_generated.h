typedef struct NvBootSdramParamsRec

{

    /// Specifies the type of memory device
    NvBootMemoryType MemoryType;
    /// Specifies the M value for PllM
    NvU32 PllMInputDivider;
    /// Specifies the N value for PllM
    NvU32 PllMFeedbackDivider;
    /// Specifies the time to wait for PLLM to lock (in microseconds)
    NvU32 PllMStableTime;
    /// Specifies misc. control bits
    NvU32 PllMSetupControl;
    /// Specifies the P value for PLLM
    NvU32 PllMPostDivider;
    /// Specifies value for Charge Pump Gain Control
    NvU32 PllMKCP;
    /// Specirfic VCO gain
    NvU32 PllMKVCO;
    /// Spare BCT param
    NvU32 EmcBctSpare0;
    /// Spare BCT param
    NvU32 EmcBctSpare1;
    /// Spare BCT param
    NvU32 EmcBctSpare2;
    /// Spare BCT param
    NvU32 EmcBctSpare3;
    /// Spare BCT param
    NvU32 EmcBctSpare4;
    /// Spare BCT param
    NvU32 EmcBctSpare5;
    /// Spare BCT param
    NvU32 EmcBctSpare6;
    /// Spare BCT param
    NvU32 EmcBctSpare7;
    /// Spare BCT param
    NvU32 EmcBctSpare8;
    /// Spare BCT param
    NvU32 EmcBctSpare9;
    /// Spare BCT param
    NvU32 EmcBctSpare10;
    /// Spare BCT param
    NvU32 EmcBctSpare11;
    /// Spare BCT param
    NvU32 EmcBctSpare12;
    /// Spare BCT param
    NvU32 EmcBctSpare13;
    /// Defines EMC_2X_CLK_SRC, EMC_2X_CLK_DIVISOR, EMC_INVERT_DCD
    NvU32 EmcClockSource;
    /// Defines EMC_2X_CLK_SRC, EMC_2X_CLK_DIVISOR, EMC_INVERT_DCD
    NvU32 EmcClockSourceDll;
    /// Defines possible override for PLLLM_MISC2
    NvU32 ClkRstControllerPllmMisc2Override;
    /// enables override for PLLLM_MISC2
    NvU32 ClkRstControllerPllmMisc2OverrideEnable;
    /// defines CLK_ENB_MC1 in register clk_rst_controller_clk_enb_w_clr
    NvU32 ClearClk2Mc1;

    /// Auto-calibration of EMC pads
    /// 
    /// Specifies the value for EMC_AUTO_CAL_INTERVAL
    NvU32 EmcAutoCalInterval;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG
    /// Note: Trigger bits are set by the SDRAM code.
    NvU32 EmcAutoCalConfig;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG2
    NvU32 EmcAutoCalConfig2;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG3
    NvU32 EmcAutoCalConfig3;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG4
    NvU32 EmcAutoCalConfig4;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG5
    NvU32 EmcAutoCalConfig5;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG6
    NvU32 EmcAutoCalConfig6;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG7
    NvU32 EmcAutoCalConfig7;
    /// Specifies the value for EMC_AUTO_CAL_CONFIG8
    NvU32 EmcAutoCalConfig8;
    /// Specifies the value for EMC_AUTO_CAL_VREF_SEL_0
    NvU32 EmcAutoCalVrefSel0;
    /// Specifies the value for EMC_AUTO_CAL_VREF_SEL_1
    NvU32 EmcAutoCalVrefSel1;
    /// Specifies the value for EMC_AUTO_CAL_CHANNEL
    NvU32 EmcAutoCalChannel;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_0
    NvU32 EmcPmacroAutocalCfg0;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_1
    NvU32 EmcPmacroAutocalCfg1;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_2
    NvU32 EmcPmacroAutocalCfg2;
    /// Specifies the value for EMC_PMACRO_RX_TERM
    NvU32 EmcPmacroRxTerm;
    /// Specifies the value for EMC_PMACRO_DQ_TX_DRV
    NvU32 EmcPmacroDqTxDrv;
    /// Specifies the value for EMC_PMACRO_CA_TX_DRV
    NvU32 EmcPmacroCaTxDrv;
    /// Specifies the value for EMC_PMACRO_CMD_TX_DRV
    NvU32 EmcPmacroCmdTxDrv;
    /// Specifies the value for EMC_PMACRO_AUTOCAL_CFG_COMMON
    NvU32 EmcPmacroAutocalCfgCommon;
    /// Specifies the value for EMC_PMACRO_ZCTRL
    NvU32 EmcPmacroZctrl;
    /// Specifies the time for the calibration to stabilize (in microseconds)
    NvU32 EmcAutoCalWait;
    /// Specifies the value for EMC_XM2COMPPADCTRL
    NvU32 EmcXm2CompPadCtrl;
    /// Specifies the value for EMC_XM2COMPPADCTRL2
    NvU32 EmcXm2CompPadCtrl2;
    /// Specifies the value for EMC_XM2COMPPADCTRL3
    NvU32 EmcXm2CompPadCtrl3;

    /// DRAM size information
    /// Specifies the value for EMC_ADR_CFG
    NvU32 EmcAdrCfg;

    /// Specifies the time to wait after asserting pin CKE (in microseconds)
    NvU32 EmcPinProgramWait;
    /// Specifies the extra delay before/after pin RESET/CKE command
    NvU32 EmcPinExtraWait;
    /// Specifies the value for GPIO_EN in EMC_PIN
    NvU32 EmcPinGpioEn;
    /// Specifies the value for GPIO in  EMC_PIN
    NvU32 EmcPinGpio;
    /// Specifies the extra delay after the first writing of EMC_TIMING_CONTROL
    NvU32 EmcTimingControlWait;

    /// Timing parameters required for the SDRAM
    /// 
    /// Specifies the value for EMC_RC
    NvU32 EmcRc;
    /// Specifies the value for EMC_RFC
    NvU32 EmcRfc;
    /// Specifies the value for EMC_RFCPB
    NvU32 EmcRfcPb;
    /// Specifies the value for EMC_REFCTRL2
    NvU32 EmcRefctrl2;
    /// Specifies the value for EMC_RFC_SLR
    NvU32 EmcRfcSlr;
    /// Specifies the value for EMC_RAS
    NvU32 EmcRas;
    /// Specifies the value for EMC_RP
    NvU32 EmcRp;
    /// Specifies the value for EMC_R2R
    NvU32 EmcR2r;
    /// Specifies the value for EMC_W2W
    NvU32 EmcW2w;
    /// Specifies the value for EMC_R2W
    NvU32 EmcR2w;
    /// Specifies the value for EMC_W2R
    NvU32 EmcW2r;
    /// Specifies the value for EMC_R2P
    NvU32 EmcR2p;
    /// Specifies the value for EMC_W2P
    NvU32 EmcW2p;
    /// Specifies the value for EMC_TPPD
    NvU32 EmcTppd;
    /// Specifies the value for EMC_CCDMW
    NvU32 EmcCcdmw;
    /// Specifies the value for EMC_RD_RCD
    NvU32 EmcRdRcd;
    /// Specifies the value for EMC_WR_RCD
    NvU32 EmcWrRcd;
    /// Specifies the value for EMC_RRD
    NvU32 EmcRrd;
    /// Specifies the value for EMC_REXT
    NvU32 EmcRext;
    /// Specifies the value for EMC_WEXT
    NvU32 EmcWext;
    /// Specifies the value for EMC_WDV
    NvU32 EmcWdv;
    /// Specifies the value for EMC_WDV_CHK
    NvU32 EmcWdvChk;
    /// Specifies the value for EMC_WSV
    NvU32 EmcWsv;
    /// Specifies the value for EMC_WSV
    NvU32 EmcWev;
    /// Specifies the value for EMC_WDV_MASK
    NvU32 EmcWdvMask;
    /// Specifies the value for EMC_WS_DURATION
    NvU32 EmcWsDuration;
    /// Specifies the value for EMC_WS_DURATION
    NvU32 EmcWeDuration;
    /// Specifies the value for EMC_QUSE
    NvU32 EmcQUse;
    /// Specifies the value for EMC_QUSE_WIDTH
    NvU32 EmcQuseWidth;
    /// Specifies the value for EMC_IBDLY
    NvU32 EmcIbdly;
    /// Specifies the value for EMC_OBDLY
    NvU32 EmcObdly;
    /// Specifies the value for EMC_EINPUT
    NvU32 EmcEInput;
    /// Specifies the value for EMC_EINPUT_DURATION
    NvU32 EmcEInputDuration;
    /// Specifies the value for EMC_PUTERM_EXTRA
    NvU32 EmcPutermExtra;
    /// Specifies the value for EMC_PUTERM_WIDTH
    NvU32 EmcPutermWidth;
    /// Specifies the value for EMC_QRST
    NvU32 EmcQRst;
    /// Specifies the value for EMC_QSAFE
    NvU32 EmcQSafe;
    /// Specifies the value for EMC_RDV
    NvU32 EmcRdv;
    /// Specifies the value for EMC_RDV_MASK
    NvU32 EmcRdvMask;
    /// Specifies the value for EMC_RDV_EARLY
    NvU32 EmcRdvEarly;
    /// Specifies the value for EMC_RDV_EARLY_MASK
    NvU32 EmcRdvEarlyMask;
    /// Specifies the value for EMC_QPOP
    NvU32 EmcQpop;
    /// Specifies the value for EMC_REFRESH
    NvU32 EmcRefresh;
    /// Specifies the value for EMC_BURST_REFRESH_NUM
    NvU32 EmcBurstRefreshNum;
    /// Specifies the value for EMC_PRE_REFRESH_REQ_CNT
    NvU32 EmcPreRefreshReqCnt;
    /// Specifies the value for EMC_PDEX2WR
    NvU32 EmcPdEx2Wr;
    /// Specifies the value for EMC_PDEX2RD
    NvU32 EmcPdEx2Rd;
    /// Specifies the value for EMC_PCHG2PDEN
    NvU32 EmcPChg2Pden;
    /// Specifies the value for EMC_ACT2PDEN
    NvU32 EmcAct2Pden;
    /// Specifies the value for EMC_AR2PDEN
    NvU32 EmcAr2Pden;
    /// Specifies the value for EMC_RW2PDEN
    NvU32 EmcRw2Pden;
    /// Specifies the value for EMC_CKE2PDEN
    NvU32 EmcCke2Pden;
    /// Specifies the value for EMC_PDEX2CKE
    NvU32 EmcPdex2Cke;
    /// Specifies the value for EMC_PDEX2MRR
    NvU32 EmcPdex2Mrr;
    /// Specifies the value for EMC_TXSR
    NvU32 EmcTxsr;
    /// Specifies the value for EMC_TXSRDLL
    NvU32 EmcTxsrDll;
    /// Specifies the value for EMC_TCKE
    NvU32 EmcTcke;
    /// Specifies the value for EMC_TCKESR
    NvU32 EmcTckesr;
    /// Specifies the value for EMC_TPD
    NvU32 EmcTpd;
    /// Specifies the value for EMC_TFAW
    NvU32 EmcTfaw;
    /// Specifies the value for EMC_TRPAB
    NvU32 EmcTrpab;
    /// Specifies the value for EMC_TCLKSTABLE
    NvU32 EmcTClkStable;
    /// Specifies the value for EMC_TCLKSTOP
    NvU32 EmcTClkStop;
    /// Specifies the value for EMC_TREFBW
    NvU32 EmcTRefBw;

    /// FBIO configuration values
    /// 
    /// Specifies the value for EMC_FBIO_CFG5
    NvU32 EmcFbioCfg5;
    /// Specifies the value for EMC_FBIO_CFG7
    NvU32 EmcFbioCfg7;

    /// FBIO configuration values
    /// 
    /// Specifies the value for EMC_FBIO_CFG8
    NvU32 EmcFbioCfg8;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_0;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_1;
    /// Command mapping for CMD brick 0
    NvU32 EmcCmdMappingCmd0_2;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_0;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_1;
    /// Command mapping for CMD brick 1
    NvU32 EmcCmdMappingCmd1_2;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_0;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_1;
    /// Command mapping for CMD brick 2
    NvU32 EmcCmdMappingCmd2_2;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_0;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_1;
    /// Command mapping for CMD brick 3
    NvU32 EmcCmdMappingCmd3_2;
    /// Command mapping for DATA bricks
    NvU32 EmcCmdMappingByte;
    /// Specifies the value for EMC_FBIO_SPARE
    NvU32 EmcFbioSpare;

    /// Specifies the value for EMC_CFG_RSV
    NvU32 EmcCfgRsv;

    /// MRS command values
    /// 
    /// Specifies the value for EMC_MRS
    NvU32 EmcMrs;
    /// Specifies the MP0 command to initialize mode registers
    NvU32 EmcEmrs;
    /// Specifies the MR2 command to initialize mode registers
    NvU32 EmcEmrs2;
    /// Specifies the MR3 command to initialize mode registers
    NvU32 EmcEmrs3;
    /// Specifies the programming to LPDDR2 Mode Register 1 at cold boot
    NvU32 EmcMrw1;
    /// Specifies the programming to LPDDR2 Mode Register 2 at cold boot
    NvU32 EmcMrw2;
    /// Specifies the programming to LPDDR2/4 Mode Register 3/13 at cold boot
    NvU32 EmcMrw3;
    /// Specifies the programming to LPDDR2 Mode Register 11 at cold boot
    NvU32 EmcMrw4;
    /// Specifies the programming to LPDDR4 Mode Register 3 at cold boot
    NvU32 EmcMrw6;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot
    NvU32 EmcMrw8;
    /// Specifies the programming to LPDDR4 Mode Register 11 at cold boot
    NvU32 EmcMrw9;
    /// Specifies the programming to LPDDR4 Mode Register 12 at cold boot
    NvU32 EmcMrw10;
    /// Specifies the programming to LPDDR4 Mode Register 14 at cold boot
    NvU32 EmcMrw12;
    /// Specifies the programming to LPDDR4 Mode Register 14 at cold boot
    NvU32 EmcMrw13;
    /// Specifies the programming to LPDDR4 Mode Register 22 at cold boot
    NvU32 EmcMrw14;
    /// Specifies the programming to extra LPDDR2 Mode Register at cold boot
    NvU32 EmcMrwExtra;
    /// Specifies the programming to extra LPDDR2 Mode Register at warm boot
    NvU32 EmcWarmBootMrwExtra;
    /// Specify the enable of extra Mode Register programming at warm boot
    NvU32 EmcWarmBootExtraModeRegWriteEnable;
    /// Specify the enable of extra Mode Register programming at cold boot
    NvU32 EmcExtraModeRegWriteEnable;

    /// Specifies the EMC_MRW reset command value
    NvU32 EmcMrwResetCommand;
    /// Specifies the EMC Reset wait time (in microseconds)
    NvU32 EmcMrwResetNInitWait;
    /// Specifies the value for EMC_MRS_WAIT_CNT
    NvU32 EmcMrsWaitCnt;
    /// Specifies the value for EMC_MRS_WAIT_CNT2
    NvU32 EmcMrsWaitCnt2;

    /// EMC miscellaneous configurations
    /// 
    /// Specifies the value for EMC_CFG
    NvU32 EmcCfg;
    /// Specifies the value for EMC_CFG_2
    NvU32 EmcCfg2;
    /// Specifies the pipe bypass controls
    NvU32 EmcCfgPipe;
    /// Specifies the Clock Enable Override for Pipe/Barrelshifters
    NvU32 EmcCfgPipeClk;
    /// Specifies the value for EMC_FDPD_CTRL_CMD_NO_RAMP
    NvU32 EmcFdpdCtrlCmdNoRamp;
    /// specify the value for EMC_CFG_UPDATE
    NvU32 EmcCfgUpdate;
    /// Specifies the value for EMC_DBG
    NvU32 EmcDbg;
    /// Specifies the value for EMC_DBG at initialization
    NvU32 EmcDbgWriteMux;
    /// Specifies the value for EMC_CMDQ
    NvU32 EmcCmdQ;
    /// Specifies the value for EMC_MC2EMCQ
    NvU32 EmcMc2EmcQ;
    /// Specifies the value for EMC_DYN_SELF_REF_CONTROL
    NvU32 EmcDynSelfRefControl;

    /// Specifies the value for MEM_INIT_DONE
    NvU32 AhbArbitrationXbarCtrlMemInitDone;

    /// Specifies the value for EMC_CFG_DIG_DLL
    NvU32 EmcCfgDigDll;

    /// Specifies the value for EMC_CFG_DIG_DLL_1
    NvU32 EmcCfgDigDll_1;
    /// Specifies the value for EMC_CFG_DIG_DLL_PERIOD
    NvU32 EmcCfgDigDllPeriod;
    /// Specifies the vlaue of *DEV_SELECTN of various EMC registers
    NvU32 EmcDevSelect;

    /// Specifies the value for EMC_SEL_DPD_CTRL
    NvU32 EmcSelDpdCtrl;
    /// Specifies the value for fdpd ctrl delays on dq
    NvU32 EmcFdpdCtrlDq;
    /// Specifies the value for fdpd ctrl delays on cmd
    NvU32 EmcFdpdCtrlCmd;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0
    NvU32 EmcPmacroIbVrefDq_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1
    NvU32 EmcPmacroIbVrefDq_1;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_0
    NvU32 EmcPmacroIbVrefDqs_0;
    /// Specifies the value for EMC_PMACRO_IB_VREF_DQ_1
    NvU32 EmcPmacroIbVrefDqs_1;
    /// Specifies the value for EMC_PMACRO_IB_RXRT
    NvU32 EmcPmacroIbRxrt;
    /// Specifies the value for EMC_CFG_PIPE_1
    NvU32 EmcCfgPipe1;
    /// Specifies the value for EMC_CFG_PIPE_2
    NvU32 EmcCfgPipe2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_0
    NvU32 EmcPmacroQuseDdllRank0_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_1
    NvU32 EmcPmacroQuseDdllRank0_1;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_2
    NvU32 EmcPmacroQuseDdllRank0_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_3
    NvU32 EmcPmacroQuseDdllRank0_3;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_4
    NvU32 EmcPmacroQuseDdllRank0_4;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK0_5
    NvU32 EmcPmacroQuseDdllRank0_5;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_0
    NvU32 EmcPmacroQuseDdllRank1_0;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_1
    NvU32 EmcPmacroQuseDdllRank1_1;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_2
    NvU32 EmcPmacroQuseDdllRank1_2;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_3
    NvU32 EmcPmacroQuseDdllRank1_3;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_4
    NvU32 EmcPmacroQuseDdllRank1_4;
    /// Specifies the value for EMC_PMACRO_QUSE_DDLL_RANK1_5
    NvU32 EmcPmacroQuseDdllRank1_5;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_0
    NvU32 EmcPmacroObDdllLongDqRank0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_1
    NvU32 EmcPmacroObDdllLongDqRank0_1;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_2
    NvU32 EmcPmacroObDdllLongDqRank0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_3
    NvU32 EmcPmacroObDdllLongDqRank0_3;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_4
    NvU32 EmcPmacroObDdllLongDqRank0_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK0_5
    NvU32 EmcPmacroObDdllLongDqRank0_5;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_0
    NvU32 EmcPmacroObDdllLongDqRank1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_1
    NvU32 EmcPmacroObDdllLongDqRank1_1;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_2
    NvU32 EmcPmacroObDdllLongDqRank1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_3
    NvU32 EmcPmacroObDdllLongDqRank1_3;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_4
    NvU32 EmcPmacroObDdllLongDqRank1_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQ_RANK1_5
    NvU32 EmcPmacroObDdllLongDqRank1_5;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_0
    NvU32 EmcPmacroObDdllLongDqsRank0_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_1
    NvU32 EmcPmacroObDdllLongDqsRank0_1;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_2
    NvU32 EmcPmacroObDdllLongDqsRank0_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_3
    NvU32 EmcPmacroObDdllLongDqsRank0_3;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_4
    NvU32 EmcPmacroObDdllLongDqsRank0_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK0_5
    NvU32 EmcPmacroObDdllLongDqsRank0_5;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_0
    NvU32 EmcPmacroObDdllLongDqsRank1_0;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_1
    NvU32 EmcPmacroObDdllLongDqsRank1_1;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_2
    NvU32 EmcPmacroObDdllLongDqsRank1_2;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_3
    NvU32 EmcPmacroObDdllLongDqsRank1_3;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_4
    NvU32 EmcPmacroObDdllLongDqsRank1_4;
    /// Specifies the value for EMC_PMACRO_OB_DDLL_LONG_DQS_RANK1_5
    NvU32 EmcPmacroObDdllLongDqsRank1_5;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_0
    NvU32 EmcPmacroIbDdllLongDqsRank0_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_1
    NvU32 EmcPmacroIbDdllLongDqsRank0_1;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_2
    NvU32 EmcPmacroIbDdllLongDqsRank0_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK0_3
    NvU32 EmcPmacroIbDdllLongDqsRank0_3;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_0
    NvU32 EmcPmacroIbDdllLongDqsRank1_0;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_1
    NvU32 EmcPmacroIbDdllLongDqsRank1_1;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_2
    NvU32 EmcPmacroIbDdllLongDqsRank1_2;
    /// Specifies the value for EMC_PMACRO_IB_DDLL_LONG_DQS_RANK1_3
    NvU32 EmcPmacroIbDdllLongDqsRank1_3;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_0
    NvU32 EmcPmacroDdllLongCmd_0;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_1
    NvU32 EmcPmacroDdllLongCmd_1;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_2
    NvU32 EmcPmacroDdllLongCmd_2;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_3
    NvU32 EmcPmacroDdllLongCmd_3;
    /// Specifies the value for EMC_PMACRO_DDLL_LONG_CMD_4
    NvU32 EmcPmacroDdllLongCmd_4;
    /// Specifies the value for EMC_PMACRO_DDLL_SHORT_CMD_0
    NvU32 EmcPmacroDdllShortCmd_0;
    /// Specifies the value for EMC_PMACRO_DDLL_SHORT_CMD_1
    NvU32 EmcPmacroDdllShortCmd_1;
    /// Specifies the value for EMC_PMACRO_DDLL_SHORT_CMD_2
    NvU32 EmcPmacroDdllShortCmd_2;

    /// Specifies the delay after asserting CKE pin during a WarmBoot0
    /// sequence (in microseconds)
    NvU32 WarmBootWait;

    /// Specifies the value for EMC_ODT_WRITE
    NvU32 EmcOdtWrite;

    /// Periodic ZQ calibration
    /// 
    /// Specifies the value for EMC_ZCAL_INTERVAL
    /// Value 0 disables ZQ calibration
    NvU32 EmcZcalInterval;
    /// Specifies the value for EMC_ZCAL_WAIT_CNT
    NvU32 EmcZcalWaitCnt;
    /// Specifies the value for EMC_ZCAL_MRW_CMD
    NvU32 EmcZcalMrwCmd;

    /// DRAM initialization sequence flow control
    /// 
    /// Specifies the MRS command value for resetting DLL
    NvU32 EmcMrsResetDll;
    /// Specifies the command for ZQ initialization of device 0
    NvU32 EmcZcalInitDev0;
    /// Specifies the command for ZQ initialization of device 1
    NvU32 EmcZcalInitDev1;
    /// Specifies the wait time after programming a ZQ initialization command
    /// (in microseconds)
    NvU32 EmcZcalInitWait;
    /// Specifies the enable for ZQ calibration at cold boot [bit 0] and warm boot [bit 1]
    NvU32 EmcZcalWarmColdBootEnables;
    /// Specifies the MRW command to LPDDR2 for ZQ calibration on warmboot
    /// Is issued to both devices separately
    NvU32 EmcMrwLpddr2ZcalWarmBoot;
    /// Specifies the ZQ command to DDR3 for ZQ calibration on warmboot
    /// Is issued to both devices separately
    NvU32 EmcZqCalDdr3WarmBoot;
    /// Specifies the ZQ command to LPDDR4 for ZQ calibration on warmboot
    /// Is issued to both devices separately
    NvU32 EmcZqCalLpDdr4WarmBoot;
    /// Specifies the wait time for ZQ calibration on warmboot
    /// (in microseconds)
    NvU32 EmcZcalWarmBootWait;
    /// Specifies the enable for DRAM Mode Register programming at warm boot
    NvU32 EmcMrsWarmBootEnable;
    /// Specifies the wait time after sending an MRS DLL reset command
    /// in microseconds)
    NvU32 EmcMrsResetDllWait;
    /// Specifies the extra MRS command to initialize mode registers
    NvU32 EmcMrsExtra;
    /// Specifies the extra MRS command at warm boot
    NvU32 EmcWarmBootMrsExtra;
    /// Specifies the EMRS command to enable the DDR2 DLL
    NvU32 EmcEmrsDdr2DllEnable;
    /// Specifies the MRS command to reset the DDR2 DLL
    NvU32 EmcMrsDdr2DllReset;
    /// Specifies the EMRS command to set OCD calibration
    NvU32 EmcEmrsDdr2OcdCalib;
    /// Specifies the wait between initializing DDR and setting OCD
    /// calibration (in microseconds)
    NvU32 EmcDdr2Wait;
    /// Specifies the value for EMC_CLKEN_OVERRIDE
    NvU32 EmcClkenOverride;
    /// Specifies LOG2 of the extra refresh numbers after booting
    /// Program 0 to disable
    NvU32 EmcExtraRefreshNum;
    /// Specifies the master override for all EMC clocks
    NvU32 EmcClkenOverrideAllWarmBoot;
    /// Specifies the master override for all MC clocks
    NvU32 McClkenOverrideAllWarmBoot;
    /// Specifies digital dll period, choosing between 4 to 64 ms
    NvU32 EmcCfgDigDllPeriodWarmBoot;

    /// Pad controls
    /// 
    /// Specifies the value for PMC_VDDP_SEL
    NvU32 PmcVddpSel;
    /// Specifies the wait time after programming PMC_VDDP_SEL
    NvU32 PmcVddpSelWait;
    /// Specifies the value for PMC_DDR_PWR
    NvU32 PmcDdrPwr;
    /// Specifies the value for PMC_DDR_CFG
    NvU32 PmcDdrCfg;
    /// Specifies the value for PMC_IO_DPD3_REQ
    NvU32 PmcIoDpd3Req;
    /// Specifies the wait time after programming PMC_IO_DPD3_REQ
    NvU32 PmcIoDpd3ReqWait;
    /// Specifies the wait time after programming PMC_IO_DPD4_REQ
    NvU32 PmcIoDpd4ReqWait;
    /// Specifies the value for PMC_REG_SHORT
    NvU32 PmcRegShort;
    /// Specifies the value for PMC_NO_IOPOWER
    NvU32 PmcNoIoPower;
    /// Specifies the wait time after programing PMC_DDR_CNTRL
    NvU32 PmcDdrCntrlWait;
    /// Specifies the value for PMC_DDR_CNTRL
    NvU32 PmcDdrCntrl;
    /// Specifies the value for EMC_ACPD_CONTROL
    NvU32 EmcAcpdControl;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE0
    NvU32 EmcSwizzleRank0Byte0;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE1
    NvU32 EmcSwizzleRank0Byte1;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE2
    NvU32 EmcSwizzleRank0Byte2;
    /// Specifies the value for EMC_SWIZZLE_RANK0_BYTE3
    NvU32 EmcSwizzleRank0Byte3;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE0
    NvU32 EmcSwizzleRank1Byte0;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE1
    NvU32 EmcSwizzleRank1Byte1;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE2
    NvU32 EmcSwizzleRank1Byte2;
    /// Specifies the value for EMC_SWIZZLE_RANK1_BYTE3
    NvU32 EmcSwizzleRank1Byte3;

    /// Specifies the value for EMC_TXDSRVTTGEN
    NvU32 EmcTxdsrvttgen;
    /// Specifies the value for EMC_DATA_BRLSHFT_0
    NvU32 EmcDataBrlshft0;
    /// Specifies the value for EMC_DATA_BRLSHFT_1
    NvU32 EmcDataBrlshft1;
    /// Specifies the value for EMC_DQS_BRLSHFT_0
    NvU32 EmcDqsBrlshft0;
    /// Specifies the value for EMC_DQS_BRLSHFT_1
    NvU32 EmcDqsBrlshft1;
    /// Specifies the value for EMC_CMD_BRLSHFT_0
    NvU32 EmcCmdBrlshft0;
    /// Specifies the value for EMC_CMD_BRLSHFT_1
    NvU32 EmcCmdBrlshft1;
    /// Specifies the value for EMC_CMD_BRLSHFT_2
    NvU32 EmcCmdBrlshft2;
    /// Specifies the value for EMC_CMD_BRLSHFT_3
    NvU32 EmcCmdBrlshft3;
    /// Specifies the value for EMC_QUSE_BRLSHFT_0
    NvU32 EmcQuseBrlshft0;
    /// Specifies the value for EMC_QUSE_BRLSHFT_1
    NvU32 EmcQuseBrlshft1;
    /// Specifies the value for EMC_QUSE_BRLSHFT_2
    NvU32 EmcQuseBrlshft2;
    /// Specifies the value for EMC_QUSE_BRLSHFT_3
    NvU32 EmcQuseBrlshft3;
    /// Specifies the value for EMC_DLL_CFG_0
    NvU32 EmcDllCfg0;
    /// Specifies the value for EMC_DLL_CFG_1
    NvU32 EmcDllCfg1;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch1;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch2;
    /// Specifiy scratch values for PMC setup at warmboot
    NvU32 EmcPmcScratch3;
    /// Specifies the value for EMC_PMACRO_PAD_CFG_CTRL
    NvU32 EmcPmacroPadCfgCtrl;
    /// Specifies the value for EMC_PMACRO_VTTGEN_CTRL_0
    NvU32 EmcPmacroVttgenCtrl0;
    /// Specifies the value for EMC_PMACRO_VTTGEN_CTRL_1
    NvU32 EmcPmacroVttgenCtrl1;
    /// Specifies the value for EMC_PMACRO_VTTGEN_CTRL_2
    NvU32 EmcPmacroVttgenCtrl2;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL
    NvU32 EmcPmacroBrickCtrlRfu1;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL_FDPD
    NvU32 EmcPmacroCmdBrickCtrlFdpd;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL
    NvU32 EmcPmacroBrickCtrlRfu2;
    /// Specifies the value for EMC_PMACRO_BRICK_CTRL_FDPD
    NvU32 EmcPmacroDataBrickCtrlFdpd;
    /// Specifies the value for EMC_PMACRO_BG_BIAS_CTRL_0
    NvU32 EmcPmacroBgBiasCtrl0;
    /// Specifies the value for EMC_PMACRO_DATA_PAD_RX_CTRL
    NvU32 EmcPmacroDataPadRxCtrl;
    /// Specifies the value for EMC_PMACRO_CMD_PAD_RX_CTRL
    NvU32 EmcPmacroCmdPadRxCtrl;
    /// Specifies the value for EMC_PMACRO_DATA_RX_TERM_MODE
    NvU32 EmcPmacroDataRxTermMode;
    /// Specifies the value for EMC_PMACRO_CMD_RX_TERM_MODE
    NvU32 EmcPmacroCmdRxTermMode;
    /// Specifies the value for EMC_PMACRO_DATA_PAD_TX_CTRL
    NvU32 EmcPmacroDataPadTxCtrl;
    /// Specifies the value for EMC_PMACRO_COMMON_PAD_TX_CTRL
    NvU32 EmcPmacroCommonPadTxCtrl;
    /// Specifies the value for EMC_PMACRO_CMD_PAD_TX_CTRL
    NvU32 EmcPmacroCmdPadTxCtrl;
    /// Specifies the value for EMC_CFG_3
    NvU32 EmcCfg3;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_0
    NvU32 EmcPmacroTxPwrd0;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_1
    NvU32 EmcPmacroTxPwrd1;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_2
    NvU32 EmcPmacroTxPwrd2;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_3
    NvU32 EmcPmacroTxPwrd3;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_4
    NvU32 EmcPmacroTxPwrd4;
    /// Specifies the value for EMC_PMACRO_TX_PWRD_5
    NvU32 EmcPmacroTxPwrd5;
    /// Specifies the value for EMC_CONFIG_SAMPLE_DELAY
    NvU32 EmcConfigSampleDelay;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_0
    NvU32 EmcPmacroBrickMapping0;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_1
    NvU32 EmcPmacroBrickMapping1;
    /// Specifies the value for EMC_PMACRO_BRICK_MAPPING_2
    NvU32 EmcPmacroBrickMapping2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_0
    NvU32 EmcPmacroTxSelClkSrc0;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_1
    NvU32 EmcPmacroTxSelClkSrc1;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_2
    NvU32 EmcPmacroTxSelClkSrc2;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_3
    NvU32 EmcPmacroTxSelClkSrc3;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_4
    NvU32 EmcPmacroTxSelClkSrc4;
    /// Specifies the value for EMC_PMACRO_TX_SEL_CLK_SRC_5
    NvU32 EmcPmacroTxSelClkSrc5;
    /// Specifies the value for EMC_PMACRO_DDLL_BYPASS
    NvU32 EmcPmacroDdllBypass;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_0
    NvU32 EmcPmacroDdllPwrd0;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_1
    NvU32 EmcPmacroDdllPwrd1;
    /// Specifies the value for EMC_PMACRO_DDLL_PWRD_2
    NvU32 EmcPmacroDdllPwrd2;
    /// Specifies the value for EMC_PMACRO_CMD_CTRL_0
    NvU32 EmcPmacroCmdCtrl0;
    /// Specifies the value for EMC_PMACRO_CMD_CTRL_1
    NvU32 EmcPmacroCmdCtrl1;
    /// Specifies the value for EMC_PMACRO_CMD_CTRL_2
    NvU32 EmcPmacroCmdCtrl2;

    /// DRAM size information
    /// 
    /// Specifies the value for MC_EMEM_ADR_CFG
    NvU32 McEmemAdrCfg;
    /// Specifies the value for MC_EMEM_ADR_CFG_DEV0
    NvU32 McEmemAdrCfgDev0;
    /// Specifies the value for MC_EMEM_ADR_CFG_DEV1
    NvU32 McEmemAdrCfgDev1;
    /// Specifies the value for MC_EMEM_ADR_CFG_CHANNEL_MASK
    NvU32 McEmemAdrCfgChannelMask;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_0
    NvU32 McEmemAdrCfgBankMask0;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_1
    NvU32 McEmemAdrCfgBankMask1;
    /// Specifies the value for MC_EMEM_ADR_CFG_BANK_MASK_2
    NvU32 McEmemAdrCfgBankMask2;

    /// Specifies the value for MC_EMEM_CFG which holds the external memory
    /// size (in KBytes)
    NvU32 McEmemCfg;

    /// MC arbitration configuration
    /// 
    /// Specifies the value for MC_EMEM_ARB_CFG
    NvU32 McEmemArbCfg;
    /// Specifies the value for MC_EMEM_ARB_OUTSTANDING_REQ
    NvU32 McEmemArbOutstandingReq;
    /// Specifies the value for MC_EMEM_ARB_REFPB_HP_CTRL
    NvU32 McEmemArbRefpbHpCtrl;
    /// Specifies the value for MC_EMEM_ARB_REFPB_BANK_CTRL
    NvU32 McEmemArbRefpbBankCtrl;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RCD
    NvU32 McEmemArbTimingRcd;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RP
    NvU32 McEmemArbTimingRp;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RC
    NvU32 McEmemArbTimingRc;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RAS
    NvU32 McEmemArbTimingRas;
    /// Specifies the value for MC_EMEM_ARB_TIMING_FAW
    NvU32 McEmemArbTimingFaw;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RRD
    NvU32 McEmemArbTimingRrd;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RAP2PRE
    NvU32 McEmemArbTimingRap2Pre;
    /// Specifies the value for MC_EMEM_ARB_TIMING_WAP2PRE
    NvU32 McEmemArbTimingWap2Pre;
    /// Specifies the value for MC_EMEM_ARB_TIMING_R2R
    NvU32 McEmemArbTimingR2R;
    /// Specifies the value for MC_EMEM_ARB_TIMING_W2W
    NvU32 McEmemArbTimingW2W;
    /// Specifies the value for MC_EMEM_ARB_TIMING_R2W
    NvU32 McEmemArbTimingR2W;
    /// Specifies the value for MC_EMEM_ARB_TIMING_W2R
    NvU32 McEmemArbTimingW2R;
    /// Specifies the value for MC_EMEM_ARB_TIMING_RFCPB
    NvU32 McEmemArbTimingRFCPB;
    /// Specifies the value for MC_EMEM_ARB_DA_TURNS
    NvU32 McEmemArbDaTurns;
    /// Specifies the value for MC_EMEM_ARB_DA_COVERS
    NvU32 McEmemArbDaCovers;
    /// Specifies the value for MC_EMEM_ARB_MISC0
    NvU32 McEmemArbMisc0;
    /// Specifies the value for MC_EMEM_ARB_MISC1
    NvU32 McEmemArbMisc1;
    /// Specifies the value for MC_EMEM_ARB_MISC2
    NvU32 McEmemArbMisc2;
    /// Specifies the value for MC_EMEM_ARB_RING1_THROTTLE
    NvU32 McEmemArbRing1Throttle;
    /// Specifies the value for MC_EMEM_ARB_OVERRIDE
    NvU32 McEmemArbOverride;
    /// Specifies the value for MC_EMEM_ARB_OVERRIDE_1
    NvU32 McEmemArbOverride1;
    /// Specifies the value for MC_EMEM_ARB_RSV
    NvU32 McEmemArbRsv;
    /// Specifies the value for MC_DA_CONFIG0
    NvU32 McDaCfg0;
    /// specifies the DRAM CAS to CAS delay timing for masked writes
    NvU32 McEmemArbTimingCcdmw;

    /// Specifies the value for MC_CLKEN_OVERRIDE
    NvU32 McClkenOverride;

    /// Specifies the value for MC_STAT_CONTROL
    NvU32 McStatControl;
    /// Specifies the value for MC_VIDEO_PROTECT_BOM
    NvU32 McVideoProtectBom;
    /// Specifies the value for MC_VIDEO_PROTECT_BOM_ADR_HI
    NvU32 McVideoProtectBomAdrHi;
    /// Specifies the value for MC_VIDEO_PROTECT_SIZE_MB
    NvU32 McVideoProtectSizeMb;
    /// Specifies the value for MC_VIDEO_PROTECT_VPR_OVERRIDE
    NvU32 McVideoProtectVprOverride;
    /// Specifies the value for MC_VIDEO_PROTECT_VPR_OVERRIDE1
    NvU32 McVideoProtectVprOverride1;
    /// Specifies the value for MC_VIDEO_PROTECT_GPU_OVERRIDE_0
    NvU32 McVideoProtectGpuOverride0;
    /// Specifies the value for MC_VIDEO_PROTECT_GPU_OVERRIDE_1
    NvU32 McVideoProtectGpuOverride1;
    /// Specifies the value for MC_SEC_CARVEOUT_BOM
    NvU32 McSecCarveoutBom;
    /// Specifies the value for MC_SEC_CARVEOUT_ADR_HI
    NvU32 McSecCarveoutAdrHi;
    /// Specifies the value for MC_SEC_CARVEOUT_SIZE_MB
    NvU32 McSecCarveoutSizeMb;
    /// Specifies the value for MC_VIDEO_PROTECT_REG_CTRL.VIDEO_PROTECT_WRITE_ACCESS
    NvU32 McVideoProtectWriteAccess;
    /// Specifies the value for MC_SEC_CARVEOUT_REG_CTRL.SEC_CARVEOUT_WRITE_ACCESS
    NvU32 McSecCarveoutProtectWriteAccess;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_BOM
    NvU32 McGeneralizedCarveout1Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_BOM_HI
    NvU32 McGeneralizedCarveout1BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_SIZE_128KB
    NvU32 McGeneralizedCarveout1Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout1Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout1Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout1Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout1Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout1Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout1ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout1ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout1ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout1ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout1ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT1_CFG0
    NvU32 McGeneralizedCarveout1Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_BOM
    NvU32 McGeneralizedCarveout2Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_BOM_HI
    NvU32 McGeneralizedCarveout2BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_SIZE_128KB
    NvU32 McGeneralizedCarveout2Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout2Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout2Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout2Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout2Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout2Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout2ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout2ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout2ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout2ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout2ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT2_CFG0
    NvU32 McGeneralizedCarveout2Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_BOM
    NvU32 McGeneralizedCarveout3Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_BOM_HI
    NvU32 McGeneralizedCarveout3BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_SIZE_128KB
    NvU32 McGeneralizedCarveout3Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout3Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout3Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout3Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout3Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout3Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout3ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout3ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout3ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout3ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout3ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT3_CFG0
    NvU32 McGeneralizedCarveout3Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_BOM
    NvU32 McGeneralizedCarveout4Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_BOM_HI
    NvU32 McGeneralizedCarveout4BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_SIZE_128KB
    NvU32 McGeneralizedCarveout4Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout4Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout4Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout4Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout4Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout4Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout4ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout4ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout4ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout4ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout4ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT4_CFG0
    NvU32 McGeneralizedCarveout4Cfg0;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_BOM
    NvU32 McGeneralizedCarveout5Bom;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_BOM_HI
    NvU32 McGeneralizedCarveout5BomHi;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_SIZE_128KB
    NvU32 McGeneralizedCarveout5Size128kb;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS0
    NvU32 McGeneralizedCarveout5Access0;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS1
    NvU32 McGeneralizedCarveout5Access1;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS2
    NvU32 McGeneralizedCarveout5Access2;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS3
    NvU32 McGeneralizedCarveout5Access3;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_ACCESS4
    NvU32 McGeneralizedCarveout5Access4;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS0
    NvU32 McGeneralizedCarveout5ForceInternalAccess0;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS1
    NvU32 McGeneralizedCarveout5ForceInternalAccess1;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS2
    NvU32 McGeneralizedCarveout5ForceInternalAccess2;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS3
    NvU32 McGeneralizedCarveout5ForceInternalAccess3;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CLIENT_FORCE_INTERNAL_ACCESS4
    NvU32 McGeneralizedCarveout5ForceInternalAccess4;
    /// Specifies the value for MC_SECURITY_CARVEOUT5_CFG0
    NvU32 McGeneralizedCarveout5Cfg0;

    /// Specifies enable for CA training
    NvU32 EmcCaTrainingEnable;
    /// Set if bit 6 select is greater than bit 7 select; uses aremc.spec packet SWIZZLE_BIT6_GT_BIT7
    NvU32 SwizzleRankByteEncode;
    /// Specifies enable and offset for patched boot rom write
    NvU32 BootRomPatchControl;
    /// Specifies data for patched boot rom write
    NvU32 BootRomPatchData;
    /// Specifies the value for MC_MTS_CARVEOUT_BOM
    NvU32 McMtsCarveoutBom;
    /// Specifies the value for MC_MTS_CARVEOUT_ADR_HI
    NvU32 McMtsCarveoutAdrHi;
    /// Specifies the value for MC_MTS_CARVEOUT_SIZE_MB
    NvU32 McMtsCarveoutSizeMb;
    /// Specifies the value for MC_MTS_CARVEOUT_REG_CTRL
    NvU32 McMtsCarveoutRegCtrl;

} NvBootSdramParams
;

    // End of generated code by warmboot_code_gen
