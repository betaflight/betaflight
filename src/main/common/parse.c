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

#include "platform.h"

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "parse.h"

inline parseArg_t argSep(char sep)
{
    return (parseArg_t) {
        .name = "<SEP>",
        .type = ARG_TYPE_SEP,
        .sep = sep,
        .isTerminator = false,
    };
}

inline parseArg_t argString(bool isTerminator, const char *name)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_STRING,
        .isTerminator = isTerminator,
    };
}

parseArg_t argStringVariant_storeUint8(bool isTerminator, const char *name, const char *variants, uint8_t *valuePtr)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_STRING_VARIANT,
        .isTerminator = isTerminator,
        .variants = variants,
        .storeType = ARG_STORE_UINT8,
        .uint8ValuePtr = valuePtr,
    };
}

inline parseArg_t argInt(bool isTerminator, const char *name)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT,
        .isTerminator = isTerminator,
    };
}

inline parseArg_t argInt_storeUint8(bool isTerminator, const char *name, uint8_t *valuePtr)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT,
        .isTerminator = isTerminator,
        .storeType = ARG_STORE_UINT8,
        .uint8ValuePtr = valuePtr,
    };
}

inline parseArg_t argInt_storeUint16(bool isTerminator, const char *name, uint16_t *valuePtr)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT,
        .isTerminator = isTerminator,
        .storeType = ARG_STORE_UINT16,
        .uint16ValuePtr = valuePtr,
    };
}

inline parseArg_t argIntInRange(bool isTerminator, const char *name, long int start, long int end)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT_IN_RANGE,
        .range = { .start = start, .end = end },
        .isTerminator = isTerminator,
    };
}

inline parseArg_t argIntInRange_storeUint8(bool isTerminator, const char *name, long int start, long int end, uint8_t *valuePtr)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT_IN_RANGE,
        .range = { .start = start, .end = end },
        .isTerminator = isTerminator,
        .storeType = ARG_STORE_UINT8,
        .uint8ValuePtr = valuePtr,
    };
}

inline parseArg_t argIntInRange_storeUint16(bool isTerminator, const char *name, long int start, long int end, uint16_t *valuePtr)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT_IN_RANGE,
        .range = { .start = start, .end = end },
        .isTerminator = isTerminator,
        .storeType = ARG_STORE_UINT16,
        .uint16ValuePtr = valuePtr,
    };
}

inline parseArg_t argIntConvert(bool isTerminator, const char *name, convertFn *convert)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT_CONVERT,
        .convert = convert,
        .isTerminator = isTerminator,
    };
}

inline parseArg_t argIntConvert_storeUint8(bool isTerminator, const char *name, convertFn *convert, uint8_t *valuePtr)
{
    return (parseArg_t) {
        .name = name,
        .type = ARG_TYPE_INT_CONVERT,
        .convert = convert,
        .isTerminator = isTerminator,
        .storeType = ARG_STORE_UINT8,
        .uint8ValuePtr = valuePtr,
    };
}

inline parseArgsResult_t parseArgsResultOk(unsigned numArgs, const char *rest)
{
    return (parseArgsResult_t) {
        .status = PARSE_STATUS_OK,
        .numArgs = numArgs,
        .rest = rest,
    };
}

inline parseArgsResult_t parseArgsResultErr(enum parseStatus_e err, const char *argName)
{
    return (parseArgsResult_t) {
        .status = err ? err : PARSE_STATUS_INTERNAL_ERR,
        .errArgName = argName,
    };
}

inline parseArgsResult_t parseArgsResultErrNotInRange(const char *argName, long int rangeStart, long int rangeEnd)
{
    return (parseArgsResult_t) {
        .status = PARSE_STATUS_INT_NOT_IN_RANGE,
        .errArgName = argName,
        .rangeStart = rangeStart,
        .rangeEnd = rangeEnd,
    };
}

inline parseArgsResult_t parseArgsResultErrCustomMsg(const char *argName, const char *errMsg)
{
    return (parseArgsResult_t) {
        .status = PARSE_STATUS_CUSTOM_ERR,
        .errArgName = argName,
        .errMsg = errMsg,
    };
}

void initParsedArgWithString(parsedArg_t *arg, const char *strStart, unsigned strLen)
{
    arg->isParsed = true;
    arg->strStart = strStart;
    arg->strLen = strLen;
}

void initParsedArgWithInt(parsedArg_t *arg, long int value)
{
    arg->isParsed = true;
    arg->intValue = value;
}

struct parseIntArgResult_s {
    enum parseStatus_e status;
    union {
        long int value;
        struct {
            const char *errArgName;
            long int rangeStart;
            long int rangeEnd;
        };
    };
};

static struct parseArgsResult_s parseInt(
    const char *str, const char *argName, long int *value, char sep
)
{
    const char *argEnd = NULL;
    errno = 0;
    *value = strtol(str, (char **) &argEnd, 10);
    if (*argEnd != '\0' && *argEnd != ' ' && *argEnd != sep) {
        return parseArgsResultErr(PARSE_STATUS_NOT_A_NUMBER, argName);
    }
    if (errno) {
        return parseArgsResultErrNotInRange(argName, 0, 1000000);
    }
    return parseArgsResultOk(0, argEnd);
}

parseArgsResult_t parseArgs(
    const char *cmdline,
    const parseArg_t *args,
    unsigned argsSize,
    parsedArg_t *parsedArgs,
    bool exactNumArgs
) {
    const char *argStart = cmdline;
    const char *argEnd = NULL;
    const parseArg_t *prevArg = NULL;

    unsigned i = 0;
    for (; i < argsSize; i++) {
        const parseArg_t *arg = &args[i];

        // Skip spaces at the beginning of an argument
        while (argStart && *argStart == ' ') {
            argStart++;
        }

        if (*argStart == '\0') {
            if (!prevArg) {
                return parseArgsResultErr(PARSE_STATUS_EMPTY_ARGS, NULL);
            }
            if (!prevArg->isTerminator) {
                return parseArgsResultErr(PARSE_STATUS_END_OF_LINE, NULL);
            }
            i--;
            goto ok;
        }

        char sep = '\0';
        if (i + 1 < argsSize) {
            const parseArg_t *nextArg = &args[i + 1];
            if (nextArg->type == ARG_TYPE_SEP) {
                sep = nextArg->sep;
            }
        }
        parsedArg_t *parsedArg = &parsedArgs[i];
        switch (arg->type) {
        case ARG_TYPE_SEP:
        {
            if (*argStart != arg->sep) {
                const char *prevArgName = NULL;
                if (prevArg) {
                    prevArgName = arg->name;
                }
                return parseArgsResultErr(PARSE_STATUS_MISSING_SEP, prevArgName);
            }
            argEnd = argStart + 1;
        }
            break;
        case ARG_TYPE_STRING:
        {
            argEnd = strchr(argStart, ' ');
            if (argEnd) {
                initParsedArgWithString(parsedArg, argStart, argEnd - argStart);
            } else {
                unsigned len = strlen(argStart);
                initParsedArgWithString(parsedArg, argStart, len);
                argEnd = argStart + len;
            }
        }
            break;
        case ARG_TYPE_STRING_VARIANT:
        {
            argEnd = strchr(argStart, ' ');
            unsigned argLen = 0;
            if (argEnd) {
                argLen = argEnd - argStart;
            } else {
                argLen = strlen(argStart);
            }
            if (arg->variants) {
                const char *varStart = arg->variants;
                unsigned varIx = 0;
                while (true) {
                    const char *varEnd = strchr(varStart, '|');
                    unsigned varLen = 0;
                    if (varEnd) {
                        varLen = varEnd - varStart;
                    } else {
                        varLen = strlen(varStart);
                    }
                    if (argLen == varLen && strncasecmp(varStart, argStart, varLen) == 0) {
                        initParsedArgWithInt(parsedArg, varIx);
                        break;
                    }
                    if (!varEnd) {
                        break;
                    }
                    varStart = varEnd + 1;
                    varIx++;
                }
                if (!parsedArg->isParsed) {
                    return parseArgsResultErr(PARSE_STATUS_INVALID_VARIANT, arg->name);
                }
            }
        }
            break;
        case ARG_TYPE_INT:
        {
            long int value;
            parseArgsResult_t intRes = parseInt(argStart, arg->name, &value, sep);
            if (intRes.status != PARSE_STATUS_OK) {
                return intRes;
            }
            initParsedArgWithInt(parsedArg, value);
            argEnd = intRes.rest;
        }
            break;
        case ARG_TYPE_INT_IN_RANGE:
        {
            long int value;
            parseArgsResult_t intRes = parseInt(argStart, arg->name, &value, sep);
            if (intRes.status != PARSE_STATUS_OK) {
                return intRes;
            }
            if (value < arg->range.start || value >= arg->range.end) {
                return parseArgsResultErrNotInRange(arg->name, arg->range.start, arg->range.end);
            }
            initParsedArgWithInt(parsedArg, value);
            argEnd = intRes.rest;
        }
            break;
        case ARG_TYPE_INT_CONVERT:
        {
            long int value;
            parseArgsResult_t intRes = parseInt(argStart, arg->name, &value, sep);
            if (intRes.status != PARSE_STATUS_OK) {
                return intRes;
            }
            initParsedArgWithInt(parsedArg, value);
            argEnd = intRes.rest;
            convertResult_t convertRes = (*arg->convert)(value);
            if (!convertRes.isOk) {
                if (convertRes.errMsg) {
                    return parseArgsResultErrCustomMsg(arg->name, convertRes.errMsg);
                }
                return parseArgsResultErr(PARSE_STATUS_INT_CONVERT_ERR, arg->name);
            }
            initParsedArgWithInt(parsedArg, convertRes.value);
        }
            break;
        }

        argStart = argEnd;
        prevArg = arg;
    }

    ok:
    while (argStart && *argStart == ' ') {
        argStart++;
    }
    if (exactNumArgs && *argStart != '\0') {
        return parseArgsResultErr(PARSE_STATUS_TOO_MANY_ARGS, NULL);
    }

    for (unsigned j = 0; j < argsSize; j++) {
        const parseArg_t *arg = &args[j];
        parsedArg_t *parsedArg = &parsedArgs[j];
        switch (arg->storeType) {
            case ARG_STORE_NULL:
                break;
            case ARG_STORE_UINT8:
                *arg->uint8ValuePtr = j <= i ? (uint8_t) parsedArg->intValue : 0;
                break;
            case ARG_STORE_UINT16:
                *arg->uint16ValuePtr = j <= i ? (uint16_t) parsedArg->intValue : 0;
                break;
            case ARG_STORE_UINT32:
                *arg->uint32ValuePtr = j <= i ? (uint32_t) parsedArg->intValue : 0;
                break;
        }
    }

    return parseArgsResultOk(i + 1, argStart);
}
