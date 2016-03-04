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

#include "rx/rx.h"
#include "common/maths.h"

#define VBAT_SCALE_DEFAULT 110
#define VBAT_RESDIVVAL_DEFAULT 10
#define VBAT_RESDIVMULTIPLIER_DEFAULT 1
#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 255

typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_VIRTUAL,
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_VIRTUAL
} currentSensor_e;

typedef struct batteryConfig_s {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatresdivval;                  // resistor divider R2 (default NAZE 10(K))
    uint8_t vbatresdivmultiplier;           // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
    uint8_t vbatwarningcellvoltage;         // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
    uint8_t vbatPidCompensation;            // Scale PIDsum to battery voltage

    int16_t currentMeterScale;             // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    uint16_t currentMeterOffset;            // offset of the current sensor in millivolt steps
    currentSensor_e  currentMeterType;      // type of current meter used, either ADC or virtual

    // FIXME this doesn't belong in here since it's a concern of MSP, not of the battery code.
    uint8_t multiwiiCurrentMeterOutput;     // if set to 1 output the amperage in milliamp steps instead of 0.01A steps via msp
    uint16_t batteryCapacity;               // mAh
} batteryConfig_t;

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT
} batteryState_e;

extern uint16_t vbat;
extern uint16_t vbatRaw;
extern uint16_t vbatLatestADC;
extern uint8_t batteryCellCount;
extern uint16_t batteryWarningVoltage;
extern uint16_t amperageLatestADC;
extern int32_t amperage;
extern int32_t mAhDrawn;

uint16_t batteryAdcToVoltage(uint16_t src);
batteryState_e getBatteryState(void);
const  char * getBatteryStateString(void);
void updateBattery(void);
void batteryInit(batteryConfig_t *initialBatteryConfig);
batteryConfig_t *batteryConfig;

void updateCurrentMeter(int32_t lastUpdateAt, rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
int32_t currentMeterToCentiamps(uint16_t src);

fix12_t calculateVbatPidCompensation(void);
uint8_t calculateBatteryPercentage(void);
uint8_t calculateBatteryCapacityRemainingPercentage(void);
