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

//
// meters
//

typedef enum {
    VOLTAGE_METER_NONE = 0,
    VOLTAGE_METER_ADC,
    VOLTAGE_METER_ESC
} voltageMeterSource_e;

typedef struct voltageMeter_s {
    uint16_t filtered;                      // voltage in 0.1V steps
    uint16_t unfiltered;                    // voltage in 0.1V steps
} voltageMeter_t;

//
// fixed ids, voltage can be measured at many different places, these identifiers are the ones we support or would consider supporting.
//

typedef enum {
    VOLTAGE_METER_ID_NONE = 0,

    VOLTAGE_METER_ID_VBAT_1 = 10,       // 10-19 for battery meters
    VOLTAGE_METER_ID_VBAT_2,
    //..
    VOLTAGE_METER_ID_VBAT_10 = 19,

    VOLTAGE_METER_ID_5V_1 = 20,         // 20-29 for 5V meters
    VOLTAGE_METER_ID_5V_2,
    //..
    VOLTAGE_METER_ID_5V_10 = 29,

    VOLTAGE_METER_ID_9V_1 = 30,         // 30-39 for 9V meters
    VOLTAGE_METER_ID_9V_2,
    //..
    VOLTAGE_METER_ID_9V_10 = 39,

    VOLTAGE_METER_ID_12V_1 = 40,        // 40-49 for 12V meters
    VOLTAGE_METER_ID_12V_2,
    //..
    VOLTAGE_METER_ID_12V_10 = 49,

    VOLTAGE_METER_ID_ESC_COMBINED_1 = 50, // 50-59 for ESC combined (it's doubtful an FC would ever expose 51-59 however)
    // ...
    VOLTAGE_METER_ID_ESC_COMBINED_10 = 59,

    VOLTAGE_METER_ID_ESC_MOTOR_1 = 60,  // 60-79 for ESC motors (20 motors)
    VOLTAGE_METER_ID_ESC_MOTOR_2,
    VOLTAGE_METER_ID_ESC_MOTOR_3,
    VOLTAGE_METER_ID_ESC_MOTOR_4,
    VOLTAGE_METER_ID_ESC_MOTOR_5,
    VOLTAGE_METER_ID_ESC_MOTOR_6,
    VOLTAGE_METER_ID_ESC_MOTOR_7,
    VOLTAGE_METER_ID_ESC_MOTOR_8,
    //...
    VOLTAGE_METER_ID_ESC_MOTOR_20 = 79,

    VOLTAGE_METER_ID_CELL_1 = 80,       // 80-119 for cell meters (40 cells)
    VOLTAGE_METER_ID_CELL_2,
    //...
    VOLTAGE_METER_ID_CELL_40 = 119,

} voltageMeterId_e;


//
// sensors
//

typedef enum {
    VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER = 0,
    VOLTAGE_SENSOR_TYPE_ESC
} voltageSensorType_e;


//
// adc sensors
//

#ifndef MAX_VOLTAGE_SENSOR_ADC
#define MAX_VOLTAGE_SENSOR_ADC 1 // VBAT - some boards have external, 12V and 5V meters.
#endif

typedef enum {
    VOLTAGE_SENSOR_ADC_VBAT = 0,
    VOLTAGE_SENSOR_ADC_5V = 1,
    VOLTAGE_SENSOR_ADC_12V = 2
} voltageSensorADC_e;

// WARNING - do not mix usage of VOLTAGE_METER_* and VOLTAGE_SENSOR_*, they are separate concerns.

#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 255

#define VBATT_LPF_FREQ  1.0f

typedef struct voltageSensorADCConfig_s {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatresdivval;                  // resistor divider R2 (default NAZE 10(K))
    uint8_t vbatresdivmultiplier;           // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
} voltageSensorADCConfig_t;

PG_DECLARE_ARRAY(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);

void voltageMeterADCInit(void);
void voltageMeterADCRefresh(void);
void voltageMeterADCRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter);

void voltageMeterESCInit(void);
void voltageMeterESCRefresh(void);
void voltageMeterESCReadCombined(voltageMeter_t *voltageMeter);
void voltageMeterESCReadMotor(uint8_t motor, voltageMeter_t *voltageMeter);

void voltageMeterReset(voltageMeter_t *voltageMeter);

void voltageReadMeter(voltageMeterId_e id, voltageMeter_t *voltageMeter);
