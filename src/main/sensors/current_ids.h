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
// fixed ids, current can be measured at many different places, these identifiers are the ones we support or would consider supporting.
//

typedef enum {
    CURRENT_METER_ID_NONE = 0,

    CURRENT_METER_ID_BATTERY_1 = 10,       // 10-19 for battery meters
    CURRENT_METER_ID_BATTERY_2,
    //..
    CURRENT_METER_ID_BATTERY_10 = 19,

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
    CURRENT_METER_ID_ESC_MOTOR_9,
    CURRENT_METER_ID_ESC_MOTOR_10,
    CURRENT_METER_ID_ESC_MOTOR_11,
    CURRENT_METER_ID_ESC_MOTOR_12,
    //...
    CURRENT_METER_ID_ESC_MOTOR_20 = 79,

    CURRENT_METER_ID_VIRTUAL_1 = 80,       // 80-89 for virtual meters
    CURRENT_METER_ID_VIRTUAL_2,

    CURRENT_METER_ID_MSP_1 = 90,       // 90-99 for MSP meters
    CURRENT_METER_ID_MSP_2,

} currentMeterId_e;
