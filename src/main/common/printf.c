/*
 * Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "printf.h"

#ifdef REQUIRE_PRINTF_LONG_SUPPORT
#include "typeconversion.h"
#endif


#ifdef REQUIRE_CC_ARM_PRINTF_SUPPORT

putcf stdout_putf;
void *stdout_putp;

// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void *putp, putcf putf, int n, char z, char *bf)
{
    int written = 0;
    char fc = z ? '0' : ' ';
    char ch;
    char *p = bf;
    while (*p++ && n > 0)
        n--;
    while (n-- > 0) {
        putf(putp, fc); written++;
    }
    while ((ch = *bf++)) {
        putf(putp, ch); written++;
    }
    return written;
}

// function to convert float to string
static void tfp_ftoa(double f, char *buf, int precision) {
    int i = 0;

    // Handle negative numbers
    if (f < 0) {
        buf[i++] = '-';
        f = -f;  // Make f positive
    }

    // Extract integer part
    int ipart = (int)f;
    double fpart = f - (double)ipart;

    // Convert integer part
    if (ipart == 0) {
        buf[i++] = '0';
    } else {
        char temp[10];
        int j = 0;
        while (ipart) {
            temp[j++] = (ipart % 10) + '0'; // + '0' ASCII 48
            ipart /= 10;
        }
        while (j > 0) {
            buf[i++] = temp[--j];
        }
    }

    buf[i++] = '.';

    // Convert fractional part
    for (int j = 0; j < precision; j++) {
        fpart *= 10;
        int digit = (int)fpart;
        buf[i++] = digit + '0';
        fpart -= digit;
    }

    // Round last digit in case of precision overflow
    if ((int)(fpart * 10) >= 5) {
        buf[i - 1] += 1;  // Round up the last digit
    }

    buf[i] = '\0';
}

// return number of bytes written
int tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
    char bf[32];
    int written = 0;
    char ch;
    const int buffer_size = sizeof(bf) - 1; // Reserve one byte for '\0'

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch); written++;
        } else {
            char lz = 0;
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            int precision = 6; // Default float precision
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
            if (ch == '.') {
                ch = *(fmt++);
                if (ch >= '0' && ch <= '9') {
                    ch = a2i(ch, &fmt, 10, &precision);
                }
            }
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
            case 0:
                goto abort;
            case 'u':{
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                    else
#endif
                        ui2a(va_arg(va, unsigned int), 10, 0, bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                }
            case 'd':{
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        li2a(va_arg(va, unsigned long int), bf);
                    else
#endif
                        i2a(va_arg(va, int), bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                }
            case 'x':
            case 'X':
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                else
#endif
                    ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                written += putchw(putp, putf, w, lz, bf);
                break;
            case 'c':
                putf(putp, (char) (va_arg(va, int))); written++;
                break;
            case 's':
                written += putchw(putp, putf, w, 0, va_arg(va, char *));
                break;
            case '%':
                putf(putp, ch); written++;
                break;
            case 'n':
                *va_arg(va, int*) = written;
                break;
            case 'f': {
                double f = va_arg(va, double);
                tfp_ftoa(f, bf, precision);
                written = putchw(putp, putf, w, lz, bf);
                if (written > buffer_size) written = buffer_size; // buffer overflow
                break;
            }
            default:
                break;
            }
        }
    }
abort:
    return written;
}

void init_printf(void *putp, void (*putf) (void *, char))
{
    stdout_putf = putf;
    stdout_putp = putp;
}

static void putcp(void *p, char c)
{
    *(*((char **) p))++ = c;
}

int tfp_sprintf(char *s, const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    int written = tfp_format(&s, putcp, fmt, va);
    putcp(&s, 0);
    va_end(va);
    return written;
}

#endif // REQUIRE_CC_ARM_PRINTF_SUPPORT

