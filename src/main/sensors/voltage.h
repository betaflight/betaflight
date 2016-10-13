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

#define VBAT_SCALE_DEFAULT 110
#define VBAT_RESDIVVAL_DEFAULT 10
#define VBAT_RESDIVMULTIPLIER_DEFAULT 1
#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 255

#define VBATT_LPF_FREQ  1.0f

#ifndef MAX_VOLTAGE_METERS
#define MAX_VOLTAGE_METERS 1 // VBAT - some boards have external, 12V and 5V meters.
#endif

typedef struct voltageMeterConfig_s {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatresdivval;                  // resistor divider R2 (default NAZE 10(K))
    uint8_t vbatresdivmultiplier;           // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
} voltageMeterConfig_t;

PG_DECLARE_ARR(voltageMeterConfig_t, MAX_VOLTAGE_METERS, voltageMeterConfig);

typedef struct voltageMeterState_s {
    uint16_t vbat;                // battery voltage in 0.1V steps (filtered)
    uint16_t vbatRaw;
    uint16_t vbatLatestADC;       // most recent unsmoothed raw reading from vbat ADC
    biquadFilter_t vbatFilterState;
} voltageMeterState_t;

extern voltageMeterState_t voltageMeterStates[MAX_VOLTAGE_METERS];


void voltageMeterInit(void);
void voltageMeterUpdate(void);

voltageMeterState_t *getVoltageMeter(uint8_t index);
uint16_t getVoltageForADCChannel(uint8_t channel);
uint16_t getLatestVoltageForADCChannel(uint8_t channel);

voltageMeterConfig_t *getVoltageMeterConfig(const uint8_t channel);
