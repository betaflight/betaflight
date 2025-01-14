/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"

#define ALL_MOTORS 255
#define MOTOR_OUTPUT_LIMIT_PERCENT_MIN 1
#define MOTOR_OUTPUT_LIMIT_PERCENT_MAX 100

typedef enum {
    MOTOR_PROTOCOL_STANDARD = 0,
    MOTOR_PROTOCOL_ONESHOT125,
    MOTOR_PROTOCOL_ONESHOT42,
    MOTOR_PROTOCOL_MULTISHOT,
    MOTOR_PROTOCOL_BRUSHED,
    MOTOR_PROTOCOL_DSHOT150,
    MOTOR_PROTOCOL_DSHOT300,
    MOTOR_PROTOCOL_DSHOT600,
/*  MOTOR_PROTOCOL_DSHOT1200, removed */
    MOTOR_PROTOCOL_PROSHOT1000,
    MOTOR_PROTOCOL_DISABLED,
    MOTOR_PROTOCOL_MAX
} motorProtocolTypes_e;

typedef struct motorVTable_s {
    // Common
    void (*postInit)(void);
    float (*convertExternalToMotor)(uint16_t externalValue);
    uint16_t (*convertMotorToExternal)(float motorValue);
    bool (*enable)(void);
    void (*disable)(void);
    bool (*isMotorEnabled)(uint8_t index);
    bool (*telemetryWait)(void);
    bool (*decodeTelemetry)(void);
    void (*updateInit)(void);
    void (*write)(uint8_t index, float value);
    void (*writeInt)(uint8_t index, uint16_t value);
    void (*updateComplete)(void);
    void (*shutdown)(void);
    bool (*isMotorIdle)(uint8_t index);

    // Digital commands
    void (*requestTelemetry)(uint8_t index);
} motorVTable_t;

typedef struct motorDevice_s {
    motorVTable_t vTable;
    uint8_t       count;
    bool          initialized;
    bool          enabled;
    timeMs_t      motorEnableTimeMs;
} motorDevice_t;
