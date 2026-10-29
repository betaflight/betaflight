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
#include "sensors/motor_sensor.h"
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
            escSensorData_t *data = &dshotSensorData[motorIndex];

            data->dataAge = 0; // Telemetry is active, so the data is current
            data->rpm = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];
            data->temperature = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) ?
                motorState->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] : 0;
            // Voltage: EDT step is 0.25V, convert to 0.01V
            data->voltage = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_VOLTAGE)) ?
                motorState->telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] * 25 : 0;
            // Current: EDT step is 1A, convert to 0.01A
            data->current = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT)) ?
                motorState->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] * 100 : 0;
            data->consumption = 0; // EDT does not report consumption
            return data;
        }
#endif
        break;
    case MOTOR_SENSOR_SOURCE_ESC_SENSOR:
#ifdef USE_ESC_SENSOR
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            return escSensorGetData(motorIndex);
        }
#endif
        break;
    default:
        break;
    }
    return NULL;
}

#endif // USE_DSHOT_TELEMETRY || USE_ESC_SENSOR
