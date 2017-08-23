/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <limits.h>

#include "string_light.h"
#include "typeconversion.h"

int sl_isalnum(int c)
{
    return sl_isdigit(c) || sl_isupper(c) || sl_islower(c);
}

int sl_isdigit(int c)
{
    return (c >= '0' && c <= '9');
}

int sl_isupper(int c)
{
    return (c >= 'A' && c <= 'Z');
}

int sl_islower(int c)
{
    return (c >= 'a' && c <= 'z');
}

int sl_tolower(int c)
{
    return sl_isupper(c) ? (c) - 'A' + 'a' : c;
}

int sl_toupper(int c)
{
    return sl_islower(c) ? (c) - 'a' + 'A' : c;
}

int sl_strcasecmp(const char * s1, const char * s2)
{
    return sl_strncasecmp(s1, s2, INT_MAX);
}

int sl_strncasecmp(const char * s1, const char * s2, int n)
{
    const unsigned char * ucs1 = (const unsigned char *) s1;
    const unsigned char * ucs2 = (const unsigned char *) s2;

    int d = 0;

    for ( ; n != 0; n--) {
        const int c1 = sl_tolower(*ucs1++);
        const int c2 = sl_tolower(*ucs2++);
        if (((d = c1 - c2) != 0) || (c2 == '\0')) {
            break;
        }
    }

    return d;
}
