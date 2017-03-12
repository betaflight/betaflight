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
    BOOT_EVENT_FLAGS_NONE           = 0,
    BOOT_EVENT_FLAGS_WARNING        = 1 << 0,
    BOOT_EVENT_FLAGS_ERROR          = 1 << 1,

    BOOT_EVENT_FLAGS_PARAM16        = 1 << 14,
    BOOT_EVENT_FLAGS_PARAM32        = 1 << 15
} bootLogFlags_e;

typedef enum {
    BOOT_EVENT_CONFIG_LOADED            = 0,
    BOOT_EVENT_SYSTEM_INIT_DONE         = 1,
    BOOT_EVENT_PWM_INIT_DONE            = 2,
    BOOT_EVENT_EXTRA_BOOT_DELAY         = 3,
    BOOT_EVENT_SENSOR_INIT_DONE         = 4,
    BOOT_EVENT_GPS_INIT_DONE            = 5,
    BOOT_EVENT_LEDSTRIP_INIT_DONE       = 6,
    BOOT_EVENT_TELEMETRY_INIT_DONE      = 7,
    BOOT_EVENT_SYSTEM_READY             = 8,
    BOOT_EVENT_GYRO_DETECTION           = 9,
    BOOT_EVENT_ACC_DETECTION            = 10,
    BOOT_EVENT_BARO_DETECTION           = 11,
    BOOT_EVENT_MAG_DETECTION            = 12,
    BOOT_EVENT_RANGEFINDER_DETECTION    = 13,
    BOOT_EVENT_MAG_INIT_FAILED          = 14,
    BOOT_EVENT_HMC5883L_READ_OK_COUNT   = 15,
    BOOT_EVENT_HMC5883L_READ_FAILED     = 16,
    BOOT_EVENT_HMC5883L_SATURATION      = 17,
    BOOT_EVENT_TIMER_CH_SKIPPED         = 18,   // 1 - MAX_MOTORS exceeded, 2 - MAX_SERVOS exceeded, 3 - feature clash
    BOOT_EVENT_TIMER_CH_MAPPED          = 19,   // 0 - PPM, 1 - PWM, 2 - MOTOR, 3 - SERVO
    BOOT_EVENT_PITOT_DETECTION          = 20,
    BOOT_EVENT_HARDWARE_IO_CONFLICT     = 21,   // Hardware IO resource conflict, parameters: #1 - current owner, #2 - requested owner

    BOOT_EVENT_CODE_COUNT
} bootLogEventCode_e;

