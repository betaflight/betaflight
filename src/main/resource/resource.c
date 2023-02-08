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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

#include "resource/resource_table.h"


#if defined(USE_RESOURCE_MGMT)

bool resource_strToPin(char *ptr, ioTag_t *tag)
{
    if (strcasecmp(ptr, "NONE") == 0) {
        *tag = IO_TAG_NONE;

        return true;
    } else {
        const unsigned port = (*ptr >= 'a') ? *ptr - 'a' : *ptr - 'A';
        if (port < 8) {
            ptr++;

            char *end;
            const long pin = strtol(ptr, &end, 10);
            if (end != ptr && pin >= 0 && pin < 16) {
                *tag = DEFIO_TAG_MAKE(port, pin);

                return true;
            }
        }
    }

    return false;
}

ioTag_t *resource_getIoTag(const resourceValue_t value, uint8_t index)
{
    const pgRegistry_t* rec = pgFind(value.pgn);
    return CONST_CAST(ioTag_t *, rec->address + value.stride * index + value.offset);
}

void resource_assign(uint8_t resourceIndex, uint8_t index, ioTag_t newTag)
{
    if (!newTag) {
        return;
    }

    for (int r = 0; r < (int)resource_resourceTableLength(); r++) {
        for (int i = 0; i < RESOURCE_VALUE_MAX_INDEX(resourceTable[r].maxIndex); i++) {
            ioTag_t *tag = resource_getIoTag(resourceTable[r], i);
            if (*tag == newTag) {
                // resource already assigned

                if (r == resourceIndex) {
                    if (i == index) {
                        continue;
                    }
                    // cleared
                    *tag = IO_TAG_NONE;
                }
            }
        }
    }
}


#define MAX_RESOURCE_COMMAND_LENGTH 50

// Successful if result is CLEARED, or ASSIGNED
// Failure otherwise
resourceApplyResult_t resource_apply(const char *args)
{
    char cmdline[MAX_RESOURCE_COMMAND_LENGTH + 1] = {0};
    strncpy(cmdline, args, MAX_RESOURCE_COMMAND_LENGTH);

    char *pch = NULL;
    char *saveptr;

    pch = strtok_r(cmdline, " ", &saveptr);

    unsigned resourceIndex = 0;
    for (; ; resourceIndex++) {
        if (resourceIndex >= resource_resourceTableLength()) {
            return INVALID_RESOURSE_NAME;
        }

        const char *resourceName = ownerNames[resourceTable[resourceIndex].owner];
        if (strncasecmp(pch, resourceName, strlen(resourceName)) == 0) {
            break;
        }
    }

    pch = strtok_r(NULL, " ", &saveptr);
    int index = atoi(pch);

    if (resourceTable[resourceIndex].maxIndex > 0 || index > 0) {
        if (index <= 0 || index > RESOURCE_VALUE_MAX_INDEX(resourceTable[resourceIndex].maxIndex)) {
            return RESOURCE_INDEX_OUT_OF_RANGE;
        }
        index -= 1;

        pch = strtok_r(NULL, " ", &saveptr);
    }

    ioTag_t *tag = resource_getIoTag(resourceTable[resourceIndex], index);

    if (strlen(pch) > 0) {
        if (resource_strToPin(pch, tag)) {
            if (*tag == IO_TAG_NONE) {
                return CLEARED;
            } else {
                ioRec_t *rec = IO_Rec(IOGetByTag(*tag));
                if (rec) {
                    resource_assign(resourceIndex, index, *tag);
                } else {
                    return PARSE_ERROR;
                }
                return ASSIGNED;
            }
        }
    }

    return PARSE_ERROR;
}

// Call once, immediately after pgReset
void resource_applyDefaults(void)
{
#ifdef RESOURCE_0
    resource_apply(RESOURCE_0);
#endif
#ifdef RESOURCE_1
    resource_apply(RESOURCE_1);
#endif
#ifdef RESOURCE_2
    resource_apply(RESOURCE_2);
#endif
#ifdef RESOURCE_3
    resource_apply(RESOURCE_3);
#endif
#ifdef RESOURCE_4
    resource_apply(RESOURCE_4);
#endif
#ifdef RESOURCE_5
    resource_apply(RESOURCE_5);
#endif
#ifdef RESOURCE_6
    resource_apply(RESOURCE_6);
#endif
#ifdef RESOURCE_7
    resource_apply(RESOURCE_7);
#endif
#ifdef RESOURCE_8
    resource_apply(RESOURCE_8);
#endif
#ifdef RESOURCE_9
    resource_apply(RESOURCE_9);
#endif
#ifdef RESOURCE_10
    resource_apply(RESOURCE_10);
#endif
#ifdef RESOURCE_11
    resource_apply(RESOURCE_11);
#endif
#ifdef RESOURCE_12
    resource_apply(RESOURCE_12);
#endif
#ifdef RESOURCE_13
    resource_apply(RESOURCE_13);
#endif
#ifdef RESOURCE_14
    resource_apply(RESOURCE_14);
#endif
#ifdef RESOURCE_15
    resource_apply(RESOURCE_15);
#endif
#ifdef RESOURCE_16
    resource_apply(RESOURCE_16);
#endif
#ifdef RESOURCE_17
    resource_apply(RESOURCE_17);
#endif
#ifdef RESOURCE_18
    resource_apply(RESOURCE_18);
#endif
#ifdef RESOURCE_19
    resource_apply(RESOURCE_19);
#endif
#ifdef RESOURCE_20
    resource_apply(RESOURCE_20);
#endif
#ifdef RESOURCE_21
    resource_apply(RESOURCE_21);
#endif
#ifdef RESOURCE_22
    resource_apply(RESOURCE_22);
#endif
#ifdef RESOURCE_23
    resource_apply(RESOURCE_23);
#endif
#ifdef RESOURCE_24
    resource_apply(RESOURCE_24);
#endif
#ifdef RESOURCE_25
    resource_apply(RESOURCE_25);
#endif
#ifdef RESOURCE_26
    resource_apply(RESOURCE_26);
#endif
#ifdef RESOURCE_27
    resource_apply(RESOURCE_27);
#endif
#ifdef RESOURCE_28
    resource_apply(RESOURCE_28);
#endif
#ifdef RESOURCE_29
    resource_apply(RESOURCE_29);
#endif
#ifdef RESOURCE_30
    resource_apply(RESOURCE_30);
#endif
#ifdef RESOURCE_31
    resource_apply(RESOURCE_31);
#endif
#ifdef RESOURCE_32
    resource_apply(RESOURCE_32);
#endif
#ifdef RESOURCE_33
    resource_apply(RESOURCE_33);
#endif
#ifdef RESOURCE_34
    resource_apply(RESOURCE_34);
#endif
#ifdef RESOURCE_35
    resource_apply(RESOURCE_35);
#endif
#ifdef RESOURCE_36
    resource_apply(RESOURCE_36);
#endif
#ifdef RESOURCE_37
    resource_apply(RESOURCE_37);
#endif
#ifdef RESOURCE_38
    resource_apply(RESOURCE_38);
#endif
#ifdef RESOURCE_39
    resource_apply(RESOURCE_39);
#endif
}

#endif // USE_RESOURCE_MGMT
