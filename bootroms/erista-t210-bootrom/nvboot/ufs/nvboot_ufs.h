/** Insert Nvidia header
 */
#ifndef _NVBOOT_UFS
#define _NVBOOT_UFS

#include "nvcommon.h"


NvU32 NvBootUFSDMELinkSetup(void);
void NvBootUFSSetupTRDTMLists(void);
NvU32 NvBootUFSStartTMTREngines(void);
NvU32 NvBootUFSChkIfDevRdy2RcvDesc(void);
NvU32 NvBootGetDevInfo(void);
NvU32 NvBootGetDevInfoPartialInit(void);
void NvBootUFSFreeTRDCmdDesc(void);
NvU32 NvBootUFSGetTRDSlot(void);
NvU32 NvBootUFSCompleteInit(void);
NvU32 NvBootUFSGetAttribute(NvU32*, NvU32, NvU32);
NvU32 NvBootUFSIsRespNOPIN(void);
NvU32 NVBootUFSGetDescriptor(NvU8*, NvU32, NvU32);


#endif
