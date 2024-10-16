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

#define DEFAULT_FLOAT_PRECISION 6

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
static void tfp_ftoa(double f, char *buf, int buf_size, int precision) {
    int i = 0;
    const double scales[] = { 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6 };

    // Handle negative numbers
    if (f < 0) {
        buf[i++] = '-';
        f = -f;  // Make f positive
    }

    // Extract integer part
    long int_part = (long)f;

    // Extract fractional part
    double frac_part = f - (double)int_part;

    // Scale the fractional part for rounding
    double scaled_frac_part = frac_part * scales[precision];

    // Get the rounded fractional part
    long rounded_frac = (long)(scaled_frac_part);
    double last_digit = scaled_frac_part - rounded_frac; // Get last digit for rounding

    // Handle rounding based on the last digit
    if (last_digit >= (double)0.5) {
        rounded_frac++;  // Round up if the last digit is 5 or more
    }

    // Check for overflow in the fractional part
    if (rounded_frac >= scales[precision]) {
        rounded_frac = 0;
        int_part++;  // Increment the integer part if necessary
    }

    // Convert the integer part to string
    if (int_part == 0) {
        buf[i++] = '0';
    } else {
        long num = int_part;
        int digits = 0;

        // Calculate the number of digits in the integer part
        long temp = num;
        while (temp > 0) {
            temp /= 10;
            digits++;
        }

        // Write digits starting from the highest digit
        for (int j = digits - 1; j >= 0; j--) {
            if (i < buf_size - 1) {
                buf[i + j] = (num % 10) + '0';
                num /= 10;
            }
        }
        i += digits;
    }

    // Add decimal point and fractional part if precision > 0
    if (precision > 0 && i < buf_size - 1) {
        buf[i++] = '.';

        // Convert the fractional part to string
        for (int j = 0; j < precision && i < buf_size - 1; j++) {
            rounded_frac *= 10;  // Shift left
            buf[i++] = (char)(rounded_frac / scales[precision]) + '0';
            rounded_frac %= (long)scales[precision];
        }
    }

    if (i < buf_size) {
        buf[i] = '\0';
    } else {
        buf[buf_size - 1] = '\0';
    }
}

// return number of bytes written
int tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
    char bf[32];
    int written = 0;
    char ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch); written++;
        } else {
            char lz = 0;
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            int precision = DEFAULT_FLOAT_PRECISION; // Default float precision
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
                if (precision > DEFAULT_FLOAT_PRECISION) precision = DEFAULT_FLOAT_PRECISION;
                tfp_ftoa(f, bf, sizeof(bf) - 1, precision);
                written = putchw(putp, putf, w, lz, bf);
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

