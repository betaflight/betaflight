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
// fixed ids, voltage can be measured at many different places, these identifiers are the ones we support or would consider supporting.
//

typedef enum {
    VOLTAGE_METER_ID_NONE = 0,

    VOLTAGE_METER_ID_BATTERY_1 = 10,       // 10-19 for battery meters
    VOLTAGE_METER_ID_BATTERY_2,
    //..
    VOLTAGE_METER_ID_BATTERY_10 = 19,

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
    VOLTAGE_METER_ID_ESC_MOTOR_9,
    VOLTAGE_METER_ID_ESC_MOTOR_10,
    VOLTAGE_METER_ID_ESC_MOTOR_11,
    VOLTAGE_METER_ID_ESC_MOTOR_12,
    //...
    VOLTAGE_METER_ID_ESC_MOTOR_20 = 79,

    VOLTAGE_METER_ID_CELL_1 = 80,       // 80-119 for cell meters (40 cells)
    VOLTAGE_METER_ID_CELL_2,
    //...
    VOLTAGE_METER_ID_CELL_40 = 119,

} voltageMeterId_e;
