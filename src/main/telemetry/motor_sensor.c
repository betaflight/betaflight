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

#include "platform.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "build/debug.h"
#include "config/feature.h"
#include "drivers/dshot.h"
#include "sensors/esc_sensor.h"
#include "telemetry/motor_sensor.h"
#include "drivers/motor.h"
#include "flight/mixer.h"

#include "pg/motor.h"

#include "common/maths.h"

#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)

const escSensorData_t *getMotorSensorData(int motorIndex, motorSensorSource_e source)
{
    // Special case: ESC_SENSOR_COMBINED doesn't require a valid motor index
    if (source == MOTOR_SENSOR_SOURCE_ESC_SENSOR && motorIndex == ESC_SENSOR_COMBINED) {
#ifdef USE_ESC_SENSOR
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            return escSensorGetCombinedData();
        }
#endif
        return NULL;
    }

    // Validate motor index for all other cases
    if (motorIndex < 0 || motorIndex >= getMotorCount()) {
        return NULL;
    }

    switch (source) {
    case MOTOR_SENSOR_SOURCE_DSHOT:
#ifdef USE_DSHOT_TELEMETRY
        if (isDshotMotorTelemetryActive(motorIndex)) {
            static escSensorData_t dshotSensorData[MAX_SUPPORTED_MOTORS];
            const dshotTelemetryMotorState_t *motorState = &dshotTelemetryState.motorState[motorIndex];
            const bool edt = (motorState->telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK) != 0;

            dshotSensorData[motorIndex].rpm = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];
            dshotSensorData[motorIndex].temperature = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) ?
                motorState->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] : 0;
            dshotSensorData[motorIndex].current = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT)) ?
                motorState->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] : 0;
            return &dshotSensorData[motorIndex];
        }
#endif
        break;
    case MOTOR_SENSOR_SOURCE_ESC_SENSOR:
#ifdef USE_ESC_SENSOR
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            // Single snapshot buffer: the next call overwrites it
            static escSensorData_t snapshot;
            escSensorCopyData(motorIndex, &snapshot);
            return &snapshot;
        }
#endif
        break;
    default:
        break;
    }
    return NULL;
}

#endif // USE_DSHOT_TELEMETRY || USE_ESC_SENSOR
