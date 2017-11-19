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

#include "common/time.h"
#include "pg/pg.h"

typedef struct transponderConfig_s {
    transponderProvider_e provider;
    uint8_t reserved;
    uint8_t data[9];
    ioTag_t ioTag;
} transponderConfig_t;

typedef struct transponderRequirement_s {
    uint8_t provider; // See transponderProvider_e
    uint8_t dataLength;
    uint16_t transmitDelay;
    uint16_t transmitJitter;
} transponderRequirement_t;

PG_DECLARE(transponderConfig_t, transponderConfig);

extern const transponderRequirement_t transponderRequirements[TRANSPONDER_PROVIDER_COUNT];

void transponderInit(void);

void transponderUpdate(timeUs_t currentTimeUs);
void transponderUpdateData(void);
void transponderTransmitOnce(void);
void transponderStartRepeating(void);
void transponderStopRepeating(void);
