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


typedef enum {
    CURRENT_METER_NONE = 0,
    CURRENT_METER_ADC,
    CURRENT_METER_VIRTUAL,
    CURRENT_METER_ESC,
    CURRENT_METER_MAX = CURRENT_METER_ESC
} currentMeterSource_e;

//
// fixed ids, current can be measured at many different places, these identifiers are the ones we support or would consider supporting.
//

typedef enum {
    CURRENT_METER_ID_NONE = 0,

    CURRENT_METER_ID_VBAT_1 = 10,       // 10-19 for battery meters
    CURRENT_METER_ID_VBAT_2,
    //..
    CURRENT_METER_ID_VBAT_10 = 19,

    CURRENT_METER_ID_5V_1 = 20,         // 20-29 for 5V meters
    CURRENT_METER_ID_5V_2,
    //..
    CURRENT_METER_ID_5V_10 = 29,

    CURRENT_METER_ID_9V_1 = 30,         // 30-39 for 9V meters
    CURRENT_METER_ID_9V_2,
    //..
    CURRENT_METER_ID_9V_10 = 39,

    CURRENT_METER_ID_12V_1 = 40,        // 40-49 for 12V meters
    CURRENT_METER_ID_12V_2,
    //..
    CURRENT_METER_ID_12V_10 = 49,

    CURRENT_METER_ID_ESC_COMBINED_1 = 50, // 50-59 for ESC combined (it's doubtful an FC would ever expose 51-59 however)
    // ...
    CURRENT_METER_ID_ESC_COMBINED_10 = 59,

    CURRENT_METER_ID_ESC_MOTOR_1 = 60,  // 60-79 for ESC motors (20 motors)
    CURRENT_METER_ID_ESC_MOTOR_2,
    CURRENT_METER_ID_ESC_MOTOR_3,
    CURRENT_METER_ID_ESC_MOTOR_4,
    CURRENT_METER_ID_ESC_MOTOR_5,
    CURRENT_METER_ID_ESC_MOTOR_6,
    CURRENT_METER_ID_ESC_MOTOR_7,
    CURRENT_METER_ID_ESC_MOTOR_8,
    //...
    CURRENT_METER_ID_ESC_MOTOR_20 = 79,
} currentMeterId_e;

typedef struct currentMeter_s {
    int32_t amperage;           // current read by current sensor in centiampere (1/100th A)
    int32_t amperageLatest;     // current read by current sensor in centiampere (1/100th A) (unfiltered)
    int32_t mAhDrawn;           // milliampere hours drawn from the battery since start
    float mAhDrawnF;
} currentMeter_t;

// NOTE: currentMeterConfig is only used by physical and virtual current meters, not ESC based current meters.
#define MAX_ADC_OR_VIRTUAL_CURRENT_METERS 2 // 1x ADC, 1x Virtual

typedef enum {
    CURRENT_SENSOR_VIRTUAL = 0, // virtual is FIRST because it should be possible to build without ADC current sensors.
    CURRENT_SENSOR_ADC,
} currentSensor_e;

// WARNING - do not mix usage of CURRENT_SENSOR_* and CURRENT_METER_*, they are separate concerns.

typedef struct currentMeterADCOrVirtualConfig_s {
    int16_t scale;              // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    uint16_t offset;            // offset of the current sensor in millivolt steps
} currentMeterADCOrVirtualConfig_t;

PG_DECLARE_ARRAY(currentMeterADCOrVirtualConfig_t, MAX_ADC_OR_VIRTUAL_CURRENT_METERS, currentMeterADCOrVirtualConfig);

void currentMeterADCInit(void);

void currentUpdateADCMeter(currentMeter_t *state, int32_t lastUpdateAt);
void currentUpdateESCMeter(currentMeter_t *state, int32_t lastUpdateAt);
void currentUpdateVirtualMeter(currentMeter_t *state, int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset);

void resetCurrentMeterState(currentMeter_t *state);
