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

#ifndef MAX_CURRENT_METERS
#define MAX_CURRENT_METERS 2 // physical & virtual
#endif

typedef struct currentMeterConfig_s {
    int16_t currentMeterScale;             // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    uint16_t currentMeterOffset;            // offset of the current sensor in millivolt steps
} currentMeterConfig_t;

PG_DECLARE_ARR(currentMeterConfig_t, MAX_CURRENT_METERS, currentMeterConfig);

typedef struct currentMeter_s {
    int32_t amperage;               // amperage read by current sensor in centiampere (1/100th A)
    int32_t mAhDrawn;               // milliampere hours drawn from the battery since start
} currentMeter_t;

typedef enum {
    CURRENT_METER_ADC = 0,
    CURRENT_METER_VIRTUAL
} currentMeterIndex_e;

extern currentMeter_t currentMeters[MAX_CURRENT_METERS];

void currentMeterInit(void);
void currentUpdateMeter(int32_t lastUpdateAt);
void currentUpdateVirtualMeter(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset);

int32_t currentSensorToCentiamps(const uint16_t src, currentMeterConfig_t *config);

currentMeter_t *getCurrentMeter(currentMeterIndex_e index);

void currentMeterInit(void);
