/* Insert Nvidia Headers here
*/

#define UFS_HC_BASE             0xABCD0000
#define UFS_CAP                 UFS_HC_BASE + 0x00
#define UFS_RESERVED1           UFS_HC_BASE + 0x04
#define UFS_VER                 UFS_HC_BASE + 0x08
#define UFS_HCDDID              UFS_HC_BASE + 0x0C
#define UFS_HCPMID              UFS_HC_BASE + 0x0F
#define UFS_RESERVED2           UFS_HC_BASE + 0x10
#define UFS_IS                  UFS_HC_BASE + 0x20
#define UFS_HCS                 UFS_HC_BASE + 0x30
#define UFS_HCE                 UFS_HC_BASE + 0x34
#define UFS_UTRLBA              UFS_HC_BASE + 0x50
#define UFS_UTRLBAU             UFS_HC_BASE + 0x54
#define UFS_UTRLDBR             UFS_HC_BASE + 0x58
#define UFS_UTRLCLR             UFS_HC_BASE + 0x5C
#define UFS_UTRLRSR             UFS_HC_BASE + 0x60

#define UFS_UTMRLBA             UFS_HC_BASE + 0x70
#define UFS_UTMRLBAU            UFS_HC_BASE + 0x74
#define UFS_UTMRLDBR            UFS_HC_BASE + 0x78
#define UFS_UTMRLCLR            UFS_HC_BASE + 0x7C
#define UFS_UTMRLRSR            UFS_HC_BASE + 0x80
/*
*
*
*
*
*/

/* Define bitfields for use with NV DRF macros */
/* UFS_HCS */
#define UFS_HCS_0_UCRDY_RANGE                         2:2
#define UFS_HCS_0_UTMRLRDY_RANGE                      2:2
#define UFS_HCS_0_UTRLRDY_RANGE                       1:1
#define UFS_HCS_0_DP_RANGE                            0:0
/* UFS_HCE */
#define UFS_HCE_0_HCE_RANGE                     0:0
/* UFS_UTRLDBR */
#define UFS_UTRLDBR_0_UTRLDBR_RANGE           31:0
/* UFS_UTMRLDBR */
#define UFS_UTMRLDBR_0_UTMRLDBR_RANGE           31:0
/* UFS_IS */
#define UFS_IS_0_SBFES_RANGE                    17:17
#define UFS_IS_0_HCFES_RANGE                    16:16
#define UFS_IS_0_UTFES_RANGE                    12:12
#define UFS_IS_0_DFES_RANGE                     11:11

/* UFS_UTRLCLR */
#define UFS_UTRLRSR_0_UTRLRSR_RANGE             1:0
/* UFS_UTMRLCLR */
#define UFS_UTMRLRSR_0_UTMRLRSR_RANGE           1:0



