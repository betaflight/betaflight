/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <limits.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#include "typeconversion.h"

int isalnum(int c)
{
    return isdigit(c) || isupper(c) || islower(c);
}

int isdigit(int c)
{
    return (c >= '0' && c <= '9');
}

int isupper(int c)
{
    return (c >= 'A' && c <= 'Z');
}

int islower(int c)
{
    return (c >= 'a' && c <= 'z');
}

int isspace(int c)
{
    return (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v');
}

int tolower(int c)
{
    return isupper(c) ? (c) - 'A' + 'a' : c;
}

int toupper(int c)
{
    return islower(c) ? (c) - 'a' + 'A' : c;
}

int strcasecmp(const char * s1, const char * s2)
{
    return strncasecmp(s1, s2, (size_t)INT_MAX);
}

int strncasecmp(const char * s1, const char * s2, size_t n)
{
    const unsigned char * ucs1 = (const unsigned char *) s1;
    const unsigned char * ucs2 = (const unsigned char *) s2;

    int d = 0;

    for ( ; n != 0; n--) {
        const int c1 = tolower(*ucs1++);
        const int c2 = tolower(*ucs2++);
        if (((d = c1 - c2) != 0) || (c2 == '\0')) {
            break;
        }
    }

    return d;
}

char *strcasestr(const char *haystack, const char *needle)
{
    int nLen = strlen(needle);
    do {
        if (!strncasecmp(haystack, needle, nLen)) {
            return (char *)haystack;
        }
        haystack++;
    } while (*haystack);
    return NULL;
}
