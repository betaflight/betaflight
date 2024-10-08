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

#pragma once

enum intParseStatus_e {
    INT_PARSE_STATUS_OK,
    INT_PARSE_STATUS_ERR
};

enum intParseError_e {
    INT_PARSE_ERROR_NOT_IN_RANGE,
    INT_PARSE_ERROR_NOT_A_NUMBER,
    INT_PARSE_ERROR_END_OF_LINE,
};

typedef struct intParseResult_s {
    enum intParseStatus_e status;
    union {
        long int value;
        enum intParseError_e err;
    } result;
} intParseResult_t;

intParseResult_t intParseResultOk(long int val);
intParseResult_t intParseResultErr(enum intParseError_e err);
intParseResult_t parseIntArg(const char * *const cmdline);
intParseResult_t parseIntArgInRange(
    const char * *const cmdline,
    long int fromVal,
    long int toVal
);
