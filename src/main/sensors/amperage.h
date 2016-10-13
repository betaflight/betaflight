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

#ifndef MAX_AMPERAGE_METERS
#define MAX_AMPERAGE_METERS 2 // physical & virtual
#endif

typedef struct amperageMeterConfig_s {
    int16_t scale;             // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    uint16_t offset;            // offset of the current sensor in millivolt steps
} amperageMeterConfig_t;

PG_DECLARE_ARR(amperageMeterConfig_t, MAX_AMPERAGE_METERS, amperageMeterConfig);

typedef struct amperageMeter_s {
    int32_t amperage;               // amperage read by current sensor in centiampere (1/100th A)
    int32_t mAhDrawn;               // milliampere hours drawn from the battery since start
} amperageMeter_t;

typedef enum {
    AMPERAGE_METER_VIRTUAL = 0,
    AMPERAGE_METER_ADC,
} amperageMeter_e;

extern amperageMeter_t amperageMeters[MAX_AMPERAGE_METERS];

void amperageMeterInit(void);
void amperageUpdateMeter(int32_t lastUpdateAt);
void amperageUpdateVirtualMeter(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset);

int32_t amperageSensorToCentiamps(const uint16_t src, amperageMeterConfig_t *config);

amperageMeter_t *getAmperageMeter(amperageMeter_e index);

void amperageMeterInit(void);
