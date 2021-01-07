/*
 * Copyright (c) 2007 - 2016 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>

#include "nvrm_drf.h"
#include "arapb_misc.h"
#include "arfuse.h"
#include "aruart.h"
//#include "armiscreg.h"
#include "nvboot_hardware_access_int.h"
#include "nvboot_uart_int.h"
#include "nvboot_util_int.h"
#include "nvboot_version_rom.h"
#include "nvboot_printf_int.h"
#include "nvboot_error.h"

#define ERROR_PRINT 1
#if ERROR_PRINT
#define X(a) #a,
static const char* NvBootErrorStrings[] = {
    NVBOOT_ERRORS
};
#undef X
#endif

static int32_t
NvBoot_strlen(const char * str)
{
    int32_t len=0;
    while(str[len] != 0)
    {
        len++;
    }
    return len;
}

static int32_t
NvBoot_IntegerToString (uint32_t n, int32_t sign, int32_t pad, uint8_t* s, size_t size, uint32_t radix)
{
    static const char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    int32_t len = 0;
    uint32_t total;
    uint32_t x = n;

    // calculate length
    do
    {
        x /= radix;
        len++;
    } while (x > 0);

    if (sign < 0)
        len++;

    if (len > pad)
    {
        total = len;
        pad = 0;
    }
    else
    {
        total = pad;
        pad -= len;
    }

    // doesn't fit, just bail out
    if (total > size) return -1;

    // write sign
    if (sign < 0)
    {
        *(s++) = '-';
        len--;
    }

    // pad
    while (pad--)
        *(s++) = '0';

    // write integer
    x = n;
    while (len--)
    {
        s[len] = digits[x % radix];
        x /= radix;
    }

    return total;
}

#if ERROR_PRINT
static int32_t NvBoot_ErrorToString (uint32_t e, uint8_t *s, uint32_t remaining)
{
    const char *eStr = NvBootErrorStrings[e];
    const char *prefix = "NvBootError_";
    const uint32_t prefixLength = 12;
    const uint32_t Length = NvBoot_strlen(eStr);

    if(Length + prefixLength <= remaining) {
        NvBootUtilMemcpy(s, prefix, prefixLength);
        NvBootUtilMemcpy(&s[prefixLength], eStr, Length);
        return Length + prefixLength;
    } else {
        return -1;
    }
}
#endif // ERROR_PRINT

uint32_t NvBoot_printf(
    uint8_t* buffer,
    size_t size,
    const char* format,
    va_list ap)
{
    const char * f = format;
    uint8_t* out = buffer;
    uint32_t remaining = size - 1;

    while (remaining > 0)
    {
        int wrote = 0;
        char cur = *(f++);

        // end of format

        if (cur == '\0')
            break;

        // formatted argument

        if (cur == '%')
        {
            const char * arg = f;
            int pad = 0;

            // support for %0N zero-padding

            if (*arg == '0')
            {
                arg++;

                while ((*arg >= '0') && (*arg <= '9'))
                {
                    pad *= 10;
                    pad += (*(arg++)) - '0';
                }
            }

            switch (*(arg++))
            {
                case 'd':
                case 'D':
                case 'i':
                    {
                        int val = va_arg(ap, int);
                        int sign = 1;

                        if (val < 0)
                        {
                            sign = -1;
                            val = -val;
                        }

                        wrote = NvBoot_IntegerToString((uint32_t)val, sign, pad, out, remaining, 10);
                    }
                    break;

                case 'u':
                case 'U':
                    {
                        unsigned val = va_arg(ap, unsigned);
                        wrote = NvBoot_IntegerToString(val, 1, pad, out, remaining, 10);
                    }
                    break;
                case 'p':
                case 'x':
                case 'X':
                    {
                        unsigned val = va_arg(ap, unsigned);
                        wrote = NvBoot_IntegerToString(val, 1, pad, out, remaining, 16);
                    }
                    break;
                case 's':
                case 'S':
                    {
                        char * val = va_arg(ap, char *);
                        //int len = 0;
                        size_t len = 0;

                        if (val != NULL)
                        {
                            len =NvBoot_strlen(val);

                            if(val[0] == '\0'){
                                wrote=-2;
                                break;
                            }
                        }
                        else
                        {
                            wrote=-2;
                            break;
                        }
                        if (len > remaining)
                        {
                            wrote = -1;
                        }
                        else
                        {
                            if(val) {
                                NvBootUtilMemcpy((char *)out, val, len);
                                wrote = len;
                            }
                        }
                    }
                    break;
                case 'c':
                case 'C':
                    {
                        char val = (char)va_arg(ap, unsigned);

                        *out = val;
                        wrote = 1;
                    }
                    break;
                case 'e':
                case 'E':
                    {
#if ERROR_PRINT
                        uint32_t val = va_arg(ap, unsigned);
                        wrote = NvBoot_ErrorToString(val, out, remaining);
#else // else just output as %x
                        unsigned val = va_arg(ap, unsigned);
                        wrote = NvBoot_IntegerToString(val, 1, pad, out, remaining, 16);
#endif // ERROR_PRINT
                    }
                    break;
                default:
                    // unsupported -- print as is
                    (void)va_arg(ap, unsigned);
                    break;
            }

            if (wrote > 0 || wrote == -2)
                f = arg;
        }

        if (wrote == -1)
        {
            break;
        }

        if (wrote == 0)
        {
            *out = cur;
            wrote = 1;
        }

        if(wrote > 0) {
            remaining -= wrote;
            out += wrote;
        }
    }

    *out = '\0';
    return (size - 1) - remaining;
}

uint32_t
NvBootPrintf(const char *fmt, ...)
{
    uint8_t buff[MAX_CONSOLE_BUFFER_LEN];
    va_list ap;
    uint32_t ret;

    va_start(ap, fmt);
    ret = NvBoot_printf(buff, sizeof(buff), fmt, ap);
    va_end(ap);

    size_t bytes;
    NvBootUartPollingWrite(buff, ret, &bytes);

    return bytes;
}
