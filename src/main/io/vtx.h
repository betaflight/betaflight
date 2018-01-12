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

#include <stdint.h>

#include "platform.h"
#include "common/time.h"
#include "pg/pg.h"

typedef struct vtxSettingsConfig_s {
    uint8_t band;           // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t channel;        // 1-8
    uint8_t power;          // 0 = lowest
    uint16_t freq;          // sets freq in MHz if band=0
    uint16_t pitModeFreq;   // sets out-of-range pitmode frequency
    uint8_t lowPowerDisarm; // min power while disarmed
} vtxSettingsConfig_t;

PG_DECLARE(vtxSettingsConfig_t, vtxSettingsConfig);

void vtxInit(void);
void vtxUpdate(timeUs_t currentTimeUs);
