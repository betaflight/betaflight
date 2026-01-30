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

escSensorData_t *getMotorSensorData(int motorIndex, motorSensorSource_e source)
{
    if (motorIndex < 0 || motorIndex >= getMotorCount()) {
        return NULL;
    }

    switch (source) {
        case MOTOR_SENSOR_SOURCE_DSHOT:
#ifdef USE_DSHOT_TELEMETRY
            static escSensorData_t dshotSensorData[MAX_SUPPORTED_MOTORS];
            if (isDshotMotorTelemetryActive(motorIndex)) {
                const dshotTelemetryMotorState_t *motorState = &dshotTelemetryState.motorState[motorIndex];
                dshotSensorData[motorIndex].rpm = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];
                const bool edt = (motorState->telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK) != 0;
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
                static escSensorData_t combinedEscSensorData;
                if (motorIndex < getMotorCount()) {
                    static escSensorData_t snapshot;
                    escSensorCopyData(motorIndex, &snapshot);
                    return &snapshot;
                } else if (motorIndex == ESC_SENSOR_COMBINED) {
                    if (escSensorIsCombinedDataDirty()) {
                        const escSensorData_t *dataArray = escSensorGetDataArray();
                        combinedEscSensorData.dataAge = 0;
                        combinedEscSensorData.temperature = 0;
                        combinedEscSensorData.voltage = 0;
                        combinedEscSensorData.current = 0;
                        combinedEscSensorData.consumption = 0;
                        combinedEscSensorData.rpm = 0;
                        for (int i = 0; i < getMotorCount(); i++) {
                            combinedEscSensorData.dataAge = MAX(combinedEscSensorData.dataAge, dataArray[i].dataAge);
                            combinedEscSensorData.temperature = MAX(combinedEscSensorData.temperature, dataArray[i].temperature);
                            combinedEscSensorData.voltage += dataArray[i].voltage;
                            combinedEscSensorData.current += dataArray[i].current;
                            combinedEscSensorData.consumption += dataArray[i].consumption;
                            combinedEscSensorData.rpm += dataArray[i].rpm;
                        }
                        combinedEscSensorData.voltage = combinedEscSensorData.voltage / getMotorCount();
                        combinedEscSensorData.rpm = combinedEscSensorData.rpm / getMotorCount();
                        escSensorClearCombinedDataDirty();
                        DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_SENSOR_TMP, combinedEscSensorData.dataAge);
                    }
                    return &combinedEscSensorData;
                }
            }
#endif
            break;
        default:
            break;
    }
    return NULL;
}

#endif // USE_DSHOT_TELEMETRY || USE_ESC_SENSOR
