/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#ifdef USE_ALTHOLD_MODE

#include "common/time.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

typedef struct altholdConfig_s {
    uint16_t velPidP;
    uint16_t velPidD;
    uint16_t velPidI;

    uint16_t altPidP;
    uint16_t altPidD;
    uint16_t altPidI;

    uint16_t minThrottle;
    uint16_t maxThrottle;
} altholdConfig_t;

PG_DECLARE(altholdConfig_t, altholdConfig);


void initAltHoldState(void);
void updateAltHoldState(timeUs_t currentTimeUs);
float getAltHoldThrottle(void);


#endif
