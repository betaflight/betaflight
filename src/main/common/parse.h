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
    INT_PARSE_STATUS_ERR,
};

enum intParseError_e {
    INT_PARSE_ERROR_NOT_IN_RANGE,
    INT_PARSE_ERROR_NOT_A_NUMBER,
    INT_PARSE_ERROR_END_OF_LINE,
};

/**
 * intParseResult_s struct represent a tagged union:
 * - when 'status' field is INT_PARSE_STATUS_ERR, 'err' field contains the error
 * - when 'status' field is INT_PARSE_STATUS_OK, 'value' field stores parsed
 *   integer and 'next' field points to the next symbol
 */
typedef struct intParseResult_s {
    enum intParseStatus_e status;
    union {
        struct {
            long int value;
            const char *next;
        };
        enum intParseError_e err;
    };
} intParseResult_t;

/**
 * @brief Parses integer argument from a command line string. Skips whitespaces
 *        until the first non-whitespace character.
 * @param cmdline is a pointer to a command line string to parse
 * @retval Parsing result
 */
intParseResult_t parseIntArg(const char *cmdline);
/**
 * @brief Parses integer argument from a command line string and validates
 *        that it lies inside a range. Skips whitespaces until the first
 *        non-whitespace character.
 * @param cmdline is a pointer to a command line string to parse
 * @param fromVal start of the range (including)
 * @param toVal end of the range (excluding)
 * @retval Parsing result
 */
intParseResult_t parseIntArgInRange(
    const char *cmdline,
    long int fromVal,
    long int toVal
);
