/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "parse.h"

#include <errno.h>
#include <stdlib.h>

#include "platform.h"

intParseResult_t intParseResultOk(long int val)
{
    intParseResult_t res = {
        .status = INT_PARSE_STATUS_OK,
        .result = { .value = val },
    };
    return res;
}

intParseResult_t intParseResultErr(enum intParseError_e err)
{
    intParseResult_t res = {
        .status = INT_PARSE_STATUS_ERR,
        .result = { .err = err },
    };
    return res;
}

intParseResult_t parseIntArg(const char * *const cmdline)
{
    // Find start of an argument to be able to check end of a string
    // before calling strtol
    const char * argStart = *cmdline;
    while (argStart && *argStart == ' ') {
        argStart++;
    }
    if (*argStart == '\0') {
        return intParseResultErr(INT_PARSE_ERROR_END_OF_LINE);
    }

    const char * *const argEnd = cmdline;
    errno = 0;
    long int n = strtol(argStart, (char **)argEnd, 10);
    if (argStart == *argEnd) {
        return intParseResultErr(INT_PARSE_ERROR_NOT_A_NUMBER);
    }
    if (errno != 0) {
        return intParseResultErr(INT_PARSE_ERROR_NOT_IN_RANGE);
    }

    *cmdline = *argEnd;
    return intParseResultOk(n);
}

intParseResult_t parseIntArgInRange(
    const char * *const cmdline,
    long int fromVal,
    long int toVal
)
{
    intParseResult_t res = parseIntArg(cmdline);
    if (res.status == INT_PARSE_STATUS_ERR) {
        return res;
    }

    long int n = res.result.value;
    if (n < fromVal || n >= toVal) {
        return intParseResultErr(INT_PARSE_ERROR_NOT_IN_RANGE);
    }

    // *cmdline = *argEnd;
    return res;
}
