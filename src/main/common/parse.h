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


#include "common/utils.h"
#include "utils.h"

enum argType_e {
    ARG_TYPE_SEP,
    ARG_TYPE_STRING,
    ARG_TYPE_STRING_VARIANT,
    ARG_TYPE_INT,
    ARG_TYPE_INT_IN_RANGE,
    ARG_TYPE_INT_CONVERT,
};

typedef struct convertResult_s {
    bool isOk;
    union {
        long int value;
        const char *errMsg;
    };
} convertResult_t;

typedef convertResult_t convertFn(long int value);

enum storeType_e {
    ARG_STORE_NULL,
    ARG_STORE_UINT8,
    ARG_STORE_UINT16,
    ARG_STORE_UINT32,
};

typedef struct parseArg_s {
    const char *name;
    enum argType_e type;
    bool isTerminator;
    // Payload
    union {
        // ARG_TYPE_SEP
        struct {
            char sep;
        };
        // ARG_TYPE_STRING
        struct {
            const char *variants;
        };
        // ARG_TYPE_INT_IN_RANGE
        struct {
            long int start;
            long int end;
        } range;
        // ARG_TYPE_INT_CONVERT
        struct {
            convertFn *convert;
        };
    };
    enum storeType_e storeType;
    union {
        uint8_t *uint8ValuePtr;
        uint16_t *uint16ValuePtr;
        uint32_t *uint32ValuePtr;
    };
} parseArg_t;

parseArg_t argSep(char sep);
parseArg_t argString(bool isTerminator, const char *name);
parseArg_t argStringVariant_storeUint8(bool isTerminator, const char *name, const char *variants, uint8_t *valuePtr);
parseArg_t argInt(bool isTerminator, const char *name);
parseArg_t argIntInRange(bool isTerminator, const char *name, long int start, long int end);
parseArg_t argIntConvert(bool isTerminator, const char *name, convertFn *convert);

parseArg_t argInt_storeUint8(bool isTerminator, const char *name, uint8_t *valuePtr);
parseArg_t argInt_storeUint16(bool isTerminator, const char *name, uint16_t *valuePtr);
parseArg_t argIntInRange_storeUint8(bool isTerminator, const char *name, long int start, long int end, uint8_t *valuePtr);
parseArg_t argIntInRange_storeUint16(bool isTerminator, const char *name, long int start, long int end, uint16_t *valuePtr);
parseArg_t argIntConvert_storeUint8(bool isTerminator, const char *name, convertFn *convert, uint8_t *valuePtr);

enum parseStatus_e {
    PARSE_STATUS_OK,
    PARSE_STATUS_EMPTY_ARGS,
    PARSE_STATUS_END_OF_LINE,
    PARSE_STATUS_TOO_MANY_ARGS,
    PARSE_STATUS_INVALID_VARIANT,
    PARSE_STATUS_NOT_A_NUMBER,
    PARSE_STATUS_INT_NOT_IN_RANGE,
    PARSE_STATUS_INT_CONVERT_ERR,
    PARSE_STATUS_MISSING_SEP,
    PARSE_STATUS_CUSTOM_ERR,
    PARSE_STATUS_INTERNAL_ERR,
};

typedef struct parseArgsResult_s {
    enum parseStatus_e status;
    union {
        struct {
            unsigned numArgs;
            const char *rest;
        };
        struct {
            const char *errArgName;
            const char *errMsg;
            long int rangeStart;
            long int rangeEnd;
        };
    };
} parseArgsResult_t;

parseArgsResult_t parseArgsResultOk(unsigned numArgs, const char *rest);
parseArgsResult_t parseArgsResultErr(enum parseStatus_e err, const char *argName);
parseArgsResult_t parseArgsResultErrNotInRange(const char *argName, long int rangeStart, long int rangeEnd);
parseArgsResult_t parseArgsResultErrCustomMsg(const char *argName, const char *errMsg);

typedef struct parsedArg_s {
    bool isParsed;
    union {
        struct {
            const char *strStart;
            unsigned strLen;
        };
        struct {
            long int intValue;
        };
    };
} parsedArg_t;

void initParsedArgWithString(parsedArg_t *arg, const char *strStart, unsigned strLen);

void initParsedArgWithInt(parsedArg_t *arg, long int value);

parseArgsResult_t parseArgs(
    const char *cmdline,
    const parseArg_t *args,
    unsigned argsSize,
    parsedArg_t *parsedArgs,
    bool exactNumArgs
);
