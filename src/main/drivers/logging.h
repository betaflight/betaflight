/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "logging_codes.h"

typedef struct bootLogEntry_s {
    uint32_t    timestamp;
    uint16_t    eventCode;
    uint16_t    eventFlags;
    union {
        uint16_t    u16[4];
        uint32_t    u32[2];
    } params;
} bootLogEntry_t;

void initBootlog(void);
int getBootlogEventCount(void);
bootLogEntry_t * getBootlogEvent(int index);
const char * getBootlogEventDescription(bootLogEventCode_e eventCode);
void addBootlogEvent2(bootLogEventCode_e eventCode, bootLogFlags_e eventFlags);
void addBootlogEvent4(bootLogEventCode_e eventCode, bootLogFlags_e eventFlags, uint32_t param1, uint32_t param2);
void addBootlogEvent6(bootLogEventCode_e eventCode, bootLogFlags_e eventFlags, uint16_t param1, uint16_t param2, uint16_t param3, uint16_t param4);
