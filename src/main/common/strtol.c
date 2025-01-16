/*  Copyright (C) 2002     Manuel Novoa III
 *  From my (incomplete) stdlib library for linux and (soon) elks.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, see
 *  <http://www.gnu.org/licenses/>.

 * Adapted for Betaflight by Petr Ledvina, 2018

 */

#include <ctype.h>
#include <limits.h>

#include "platform.h"

#include "common/utils.h"
#include "common/strtol.h"

#define _STRTO_ENDPTR 1

unsigned long _strto_l(const char * str, char ** endptr, int base, int sflag)
{
    unsigned long number, cutoff;
#if _STRTO_ENDPTR
    const char *fail_char;
#define SET_FAIL(X) fail_char = (X)
#else
#define SET_FAIL(X) ((void)(X)) /* Keep side effects. */
#endif
    unsigned char negative, digit, cutoff_digit;

    SET_FAIL(str);

    while (isspace(*str)) { /* Skip leading whitespace. */
        ++str;
    }

    /* Handle optional sign. */
    negative = 0;
    switch (*str) {
        case '-':
            negative = 1;	/* Fall through to increment str. */
            FALLTHROUGH;
        case '+':
            ++str;
    }

    if (!base || base == 16 || base == 2) {		/* Either dynamic (base = 0) or base with 0[xb] prefix. */
        if (*str == '0') {
            SET_FAIL(++str);
            if ((!base || base == 16) && tolower(*str) == 'x') {
                ++str;
                base = 16;
            } else if ((!base || base == 2) && tolower(*str) == 'b') {
                ++str;
                base = 2;
            } else if(!base) {
                base = 8;
            }
        }
    }

    number = 0;

    if (((unsigned)(base - 2)) < 35) { /* Legal base. */
        cutoff_digit = ULONG_MAX % base;
        cutoff = ULONG_MAX / base;
        do {
            digit = (        (*str - '0') <= 9)
                ? /* 0..9 */ (*str - '0')
                : /* else */ (((0x20 | *str) >= 'a') /* WARNING: assumes ascii. */
                              ? /* >= A/a */ ((0x20 | *str) - ('a' - 10))
                              : /* else   */ 40 /* bad value */);

            if (digit >= base) {
                break;
            }

            SET_FAIL(++str);

            if ((number > cutoff)
                || ((number == cutoff) && (digit > cutoff_digit))) {
                number = ULONG_MAX;
                negative &= sflag;
            } else {
                number = number * base + digit;
            }
        } while (1);
    }

#if _STRTO_ENDPTR
    if (endptr) {
        *endptr = (char *) fail_char;
    }
#endif

    {
        unsigned long tmp = (negative
                             ? ((unsigned long)(-(1+LONG_MIN)))+1
                             : LONG_MAX);
        if (sflag && (number > tmp)) {
            number = tmp;
        }
    }

    return negative ? (unsigned long)(-((long)number)) : number;
}

long strtol(const char * str, char ** endptr, int base)
{
    return _strto_l(str, endptr, base, 1);
}

unsigned long strtoul(const char * str, char ** endptr, int base)
{
    return _strto_l(str, endptr, base, 0);
}

int atoi(const char *str)
{
    return strtol(str, NULL, 10);
}
