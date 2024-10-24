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

#pragma once

#include <stdint.h>

#include "platform.h"
#include "common/time.h"
#include "pg/pg.h"

typedef enum {
    VTX_LOW_POWER_DISARM_OFF = 0,
    VTX_LOW_POWER_DISARM_ALWAYS,
    VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM, // Set low power until arming for the first time
} vtxLowerPowerDisarm_e;

typedef struct vtxSettingsConfig_s {
    uint8_t band;           // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t channel;        // 1-8
    uint8_t power;          // 0 = lowest
    uint16_t freq;          // sets freq in MHz if band=0
    uint16_t pitModeFreq;   // sets out-of-range pitmode frequency
    uint8_t lowPowerDisarm; // min power while disarmed, from vtxLowerPowerDisarm_e
    uint8_t softserialAlt;  // prepend 0xff before sending frame even with SOFTSERIAL
} vtxSettingsConfig_t;

PG_DECLARE(vtxSettingsConfig_t, vtxSettingsConfig);

void vtxInit(void);
void vtxUpdate(timeUs_t currentTimeUs);
