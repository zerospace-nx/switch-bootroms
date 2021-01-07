/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
 * 
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include "nvcommon.h"
#include "nvboot_config.h"
#include "nvboot_context_int.h"
#include "nvboot_hacks_int.h"
#include "nvboot_log_int.h"
#include "nvboot_main_int.h"
#include "nvboot_util_int.h"

NvBootContext Context;

/*
 * main(): The main bootRom code.
 */
int main(void)
{
#if NVBOOT_SPIN_WAIT_AT_START
  /*    NV_BOOT_SPIN_WAIT() */
#endif

    // Init serial logging if msg payload is present at IRAM_D
    NvBootLogInit();
    NvBootMainSecureRomEnter();  // after C runtime initialization

#if NVBOOT_SPIN_WAIT_AT_END
    {
        for (;;)
            ;
    }
#endif

    return 0;
}
