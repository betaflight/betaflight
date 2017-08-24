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

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"

typedef struct vtxRTC6705Config_s {
    uint8_t band;       // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t channel;    // 1-8
    uint8_t power;      // 0 = lowest
} vtxRTC6705Config_t;

PG_DECLARE(vtxRTC6705Config_t, vtxRTC6705Config);

#ifdef RTC6705_POWER_PIN
#define RTC6705_POWER_COUNT 3
#define VTX_RTC6705_DEFAULT_POWER 1
#else
#define RTC6705_POWER_COUNT 2
#define VTX_RTC6705_DEFAULT_POWER 0
#endif

extern const char * const rtc6705PowerNames[RTC6705_POWER_COUNT];

void vtxRTC6705Configure(void);
bool vtxRTC6705Init();
