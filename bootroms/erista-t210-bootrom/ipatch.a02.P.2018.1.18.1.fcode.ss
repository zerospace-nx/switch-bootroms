    AREA    |.text|, CODE
    CODE16
;===========================================================================
; Total blocks
    DCD 0x1    ; total blocks
    DCD     b_inst_cnt
    DCD     g_ins_cnt
    DCD g0_code_start
b_inst_start
;===========================================================================
; Tag Block
    DCB "$CP$"
    DCD b0_ins_cnt    ; patch svc/instruction count
    DCD b0_cam_cnt    ; patch cam entries
    DCD b0_cam_start    ; Pointer to cam entries block
    DCD b0_code_start    ; Pointer to svc/instruction block

;===========================================================================
; CAM register entries. Up to 12 entries
;  IROM patch tracking bug 2047771
b0_cam_start    
    DCD 0x0b57df00  ; SVC #00 (H/W SLCG bug fix)
    DCD 0x1820df22  ; SVC #02 PMC scratch overlap (between MC and BootROM PMIC usage) during SC7 exit. 
    DCD 0x3797df26  ; SVC #03 USB HS Squelch (Electrical Issue)
    DCD 0x3B4D2100  ; Fix USB setup packet security issue
    DCD 0x042bdf2C  ; SVC #07 (Use straps saved in secure scratch during non-L0 boot)
    DCD 0x37AAdf42  ; SVC #08 USB Electrical Issue fix.
    DCD 0x0972df4B  ; SVC #09 (Fuse based JTAG disable h/w bug fix)
    DCD 0x2293df54  ; SVC #0A; SC7 Fix Destination in IRAM B, Sanitize Length, Override Entrypoint
    DCD 0x21FAdf5D  ; SVC #0B; SC7 override Entrypoint for RSA-PSS
    DCD 0xBBA2AC57  ; IROM region patch 1; FSKP key 58
    DCD 0xBBAC3D19  ; IROM region patch 2; FSKP key 58
    DCD 0x1E952001  ; SE context restore sequence update. (set SE_TZRAM_SECURITY[0])
b0_cam_cnt EQU ((. - b0_cam_start)/4)

b0_code_start
    CODE16
;SVC #00
;Enable clocks & disable SLCG in CAR
    mov        r2, #0
    mvn        r2, r2
    ldr        r1,=0x60006410    
    str        r2, [r1,#0x30]        ;CLK_RST_CONTROLLER_CLK_ENB_V_SET_0 (440)
    str        r2, [r1,#0x38]        ;CLK_RST_CONTROLLER_CLK_ENB_W_SET_0    (448)
    ldr        r1,=0x600060f8    
    str        r2, [r1,#0]        ;CAR CLK OVR A (f8)
    str        r2, [r1,#4]        ;CAR CLK OVR B (fc)
    ldr        r1,=0x60006284    
    str        r2, [r1,#0]        ;CLK_RST_CONTROLLER_CLK_ENB_X_SET_0 (284)
    str        r2, [r1,#0x18]        ;CLK_RST_CONTROLLER_CLK_ENB_Y_SET_0    (29c)

    add        r1, #0x80
    add        r1, #0x1c        ;0x60006320
    str        r2, [r1,#0x0]        ;CLK_RST_CONTROLLER_CLK_ENB_L_SET_0 (320)
    str        r2, [r1,#0x8]        ;CLK_RST_CONTROLLER_CLK_ENB_H_SET_0    (328)
    str        r2, [r1,#0x10]        ;CLK_RST_CONTROLLER_CLK_ENB_U_SET_0 (330)
    add        r1, #0x80
    str        r2, [r1,#0]        ;CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRC_0 (3a0)
    str        r2, [r1,#0x4]        ;CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD_0 (3a4)    
    ldr        r1,=0x60006554    
    str        r2, [r1,#0]        ;CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRE_0 (554)

    mov        r2, #0xa0
    lsl        r2, #0x18        ;0xa0000000
    ldr        r1,=0x60006148    
    str        r2, [r1,#0]        ;CLK_RST_CONTROLLER_CLK_SOURCE_VI_0(148)
    add        r1, #0x38
    str        r2, [r1,#0x0]        ;CLK_RST_CONTROLLER_CLK_SOURCE_HOST1X_0 (180)

    mov        r2, #0xe0
    lsl        r2, #0x18        ;0xe0000000
    ldr        r1,=0x600066a0    
    str        r2, [r1,#0]        ;CLK_RST_CONTROLLER_CLK_SOURCE_NVENC_0(6a0)
    mov        r1, #0x0        ; restore       
    mov        r0, #0xe              ; instruction replaced @0x001016ae
    b       g_code_end
;SVC #02
    ldr        r0,[r1,#0x18]         ;r1 already has the base addr we want in it
    mov        r2, #1
    orr        r0, r2            ;we want to just set the lowest bit rather than wiping this scratch out. When we return this will be STR'd
    b       g_code_end
;SVC #03
    ldr        r2, [r4,#0x50]         ; Read UTMIP_BIAS_CFG2_0
    adds    r2, r2, #2             ; Set HSSQUELCH_LEVEL_NEW=0x2
    str        r2, [r4,#0x50]          ; Write UTMIP_BIAS_CFG2_0
    subs    r1, r1, #0x80         ; Clear Bit 7 of UTMIP_SPARE_CFG0_0
    str        r1, [r4,#0x34]        ;restore instruction @106f2E
    b        g_code_end
;SVC #07
    mov    r0, #0x70
    lsl    r0, #0x18        ;0x70000000
    ldr    r6,=0x7000EF14    ;APBDEV_PMC_SECURE_SCRATCH111_0 
    ldr    r2,=0x7000E5b4    ;APBDEV_PMC_RST_STATUS_0
    ldr    r2, [r2,#0x0]         ;read APBDEV_PMC_RST_STATUS_0
    cmp    r2, #0            ;RST_SOURCE: 0 = POR
    beq    restore_por
    ldr    r2, [r6,#0x0]         ;read APBDEV_PMC_SECURE_SCRATCH111_0 
    str    r2, [r0,#0x8]        ;restore strap_opt_a from secure scratch
    b        b3_sec_lock
restore_por
    ldr    r2, [r0,#0x8]         ;read APB_MISC_PP_STRAPPING_OPT_A_0 
    lsr    r0, #0x12        ; mask for bit 12:10
    orr    r2, r0            ; orr 
    str    r2, [r6,#0x0]        ;copy strap_opt_a to secure scratch
b3_sec_lock
    ldr    r6,=0x7000E9c0    ;APBDEV_PMC_SEC_DISABLE8_0 
    ldr    r0, [r6,#0x0]         ;read APBDEV_PMC_SEC_DISABLE8_0 
    mov    r2, #0x1
    lsl    r2, #14
    orr    r2, r0            ;read modify
    str    r2, [r6,#0x0]        ;write APBDEV_PMC_SEC_DISABLE8_0 bit 14
rst_status_por    
    ldr    r0, [r5,#0x10]        ; restore instuction @ 0x00100856
    b       g_code_end
;SVC #08
    mov    r2, #0xf 
    lsl    r2, r2,#24
    bic    r1, r1,r2            ; Clear bits 27-24 UTMIP_HSRX_CFG0_0[UTMIP_PCOUNT_UPDN_DIV] = 0
    str    r1, [r4,#0x10]        ; replacing displaced instruction
    ldr    r1, [r4,#0x50]         ; Read UTMIP_BIAS_CFG2_0
    mov    r2, #0x7 ; 
    bic    r1, r1,r2;  Set UTMIP_BIAS_CFG2_0[HSSQUELCH_LEVEL_NEW]=0x0
    str    r1, [r4,#0x50]      ; Write UTMIP_BIAS_CFG2_0
    b        g_code_end
;SVC #09
    ldr    r2,=0x7000FA9C    ;NV_ADDRESS_MAP_FUSE_BASE + FUSE_ODM_INFO_0
    ldr    r2, [r2,#0]        ; read odm_info
    lsr r2, #08                ; data @ [12:8] to bit 0
    lsl r2, #01                ; shift back and put a 0 in bit 0 
    ldr r1, [r4,#0]            ; read scr
    bic r1, r2                 ; clear bit marked as disable in fuse
    str r1, [r4,#0]            ; update secureDebug control register
    cmp    r0, #0x0            ; restore instruction @0x001012e4
    b       g_code_end
;SVC #0A
    ; Get LengthInsecure.
    ldr r0,=0x400049F0
    ldr r2, [r0, #0]
    ; Store LengthInSecure for memcpy from dram to fixed destination in IRAM.
    str r2, [sp, #0]
    ; Force the destination of the memcpy to 0x40010000.
    ldr r0,=0x40010000 
    ; gs_Wb0Header.LengthInsecure <= 128KB-1?
    lsrs r2, r2, #17
    ; No? All good else reset.
    beq g_code_end
    ; Reset here
    ; Trash any registers you want here.
    ldr r0,=0x7000e400
    str r4, [r0, #0] ; r4 is 0x40004BF0, reuse this to force reset to save space. MAIN_RST is bit 4.
    b   . ; Need branch to self after triggering reset, else bpmp goes into the weeds and causes
          ; weird behavior.
;SVC #0B
    ; Modify the "Src" parameter in the RsaPssSignatureVerify call to
    ;   point to the fixed address where the signed section starts at 0x40010220.
    ;   r2 gets popped off the stack at g_code_end, so we need to modify the
    ;   stack directly.
    ldr r2,=0x40010220
    str r2, [sp, #0]
    ; r2 is address of signed section start
    ; r2+0x18 is the entry point
    ldr r2, [r2,#0x18]
    ; r0 contains the address of RSA PSS signature=0x40004b10. We need to get to entry point = 0x40004c28
    ; r0 is trashed later so we can modify it in steps to get to 0x40004c28
    add r0, #0xfc
    ; store r2= entry point in r0=0x40004c28
    str r2, [r0, #0x1c]
    b   g_code_end
   LTORG       ;Literals within the same block
b0_ins_cnt EQU ((. - b0_code_start)/4)

g0_code_start
g_code_end
    CODE16
    pop    {r2}
    mov    pc, lr
g_ins_cnt EQU ((. - g0_code_start)/4)

b1_code_start
    CODE16
    LTORG       ;Literals within the same block
b1_ins_cnt EQU ((. - b1_code_start)/4)

b2_code_start
    CODE16
    LTORG       ;Literals within the same block
b2_ins_cnt EQU ((. - b2_code_start)/4)

b3_code_start
    CODE16
b3_ins_cnt EQU ((. - b3_code_start)/4)

;===========================================================================
; End of Blocks
b_inst_cnt EQU ((. - b_inst_start)/4)
;===========================================================================
    END


