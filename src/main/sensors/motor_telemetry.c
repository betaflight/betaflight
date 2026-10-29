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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/time.h"

#include "flight/mixer.h"

#include "sensors/esc_sensor.h"

#include "motor_telemetry.h"

STATIC_ASSERT(MAX_SUPPORTED_MOTORS < MOTOR_TELEMETRY_COMBINED, motor_telemetry_combined_index_is_a_motor_index);

// DShot extended telemetry has no fixed interval. The specification uses
// 1000ms as an example for temperature and voltage. Expire a value that does
// not update for 5 intervals.
#define DSHOT_EDT_TIMEOUT_MS 5000

static motorTelemetry_t motorTelemetry[MAX_SUPPORTED_MOTORS];
static motorTelemetry_t motorTelemetryCombined;

static bool escSensorIsEnabled(void)
{
#ifdef USE_ESC_SENSOR
    return featureIsEnabled(FEATURE_ESC_SENSOR);
#else
    return false;
#endif
}

static bool dshotTelemetryIsEnabled(void)
{
#ifdef USE_DSHOT_TELEMETRY
    return useDshotTelemetry;
#else
    return false;
#endif
}

bool motorTelemetryIsEnabled(void)
{
    return escSensorIsEnabled() || dshotTelemetryIsEnabled();
}

#ifdef USE_DSHOT_TELEMETRY
static timeMs_t dshotEdtUpdateMs[MAX_SUPPORTED_MOTORS][DSHOT_TELEMETRY_TYPE_COUNT];

// True if the extended telemetry value is available and updated recently
static bool isDshotEdtFresh(dshotTelemetryMotorState_t *motorState, unsigned motorIndex, dshotTelemetryType_t type, timeMs_t nowMs)
{
    if (motorState->telemetryUpdated & (1 << type)) {
        motorState->telemetryUpdated &= ~(1 << type);
        dshotEdtUpdateMs[motorIndex][type] = nowMs;
    }

    return (motorState->telemetryTypes & (1 << type))
        && cmpTimeMs(nowMs, dshotEdtUpdateMs[motorIndex][type]) < DSHOT_EDT_TIMEOUT_MS;
}

static void mergeDshotTelemetry(motorTelemetry_t *telemetry, unsigned motorIndex)
{
    if (!useDshotTelemetry || !isDshotMotorTelemetryActive(motorIndex)) {
        return;
    }

    dshotTelemetryMotorState_t *motorState = &dshotTelemetryState.motorState[motorIndex];
    const uint32_t types = motorState->telemetryTypes;
    const timeMs_t nowMs = millis();

    telemetry->sources |= MOTOR_TELEMETRY_SOURCE_DSHOT;
    telemetry->dataAge = 0;

    telemetry->rpm = lrintf(getDshotRpm(motorIndex));
    telemetry->valid |= MOTOR_TELEMETRY_RPM;

    if ((types & DSHOT_EXTENDED_TELEMETRY_MASK) == 0) {
        return;
    }

    // Temperature: step 1 degree Celsius
    if (isDshotEdtFresh(motorState, motorIndex, DSHOT_TELEMETRY_TYPE_TEMPERATURE, nowMs)) {
        telemetry->temperature = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE];
        telemetry->valid |= MOTOR_TELEMETRY_TEMPERATURE;
    }
    // Voltage: step 0.25V, convert to 0.01V
    if (isDshotEdtFresh(motorState, motorIndex, DSHOT_TELEMETRY_TYPE_VOLTAGE, nowMs)) {
        telemetry->voltage = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] * 25;
        telemetry->valid |= MOTOR_TELEMETRY_VOLTAGE;
    }
    // Current: step 1A, convert to 0.01A
    if (isDshotEdtFresh(motorState, motorIndex, DSHOT_TELEMETRY_TYPE_CURRENT, nowMs)) {
        telemetry->current = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] * 100;
        telemetry->valid |= MOTOR_TELEMETRY_CURRENT;
    }
}
#endif

#ifdef USE_ESC_SENSOR
static void mergeEscSensorTelemetry(motorTelemetry_t *telemetry, unsigned motorIndex)
{
    if (!featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return;
    }

    // Do not use stale data. Then DShot telemetry can supply the value, and
    // all merged values are fresh.
    const escSensorData_t *escData = escSensorGetData(motorIndex);
    if (!escData || escData->dataAge > ESC_BATTERY_AGE_MAX) {
        return;
    }

    telemetry->sources |= MOTOR_TELEMETRY_SOURCE_ESC_SENSOR;
    // The battery meters use the ESC sensor data, so report its age
    telemetry->dataAge = escData->dataAge;

    // DShot RPM has a higher update rate: keep it if it is available
    if (!(telemetry->valid & MOTOR_TELEMETRY_RPM)) {
        telemetry->rpm = lrintf(erpmToRpm(escData->rpm));
        telemetry->valid |= MOTOR_TELEMETRY_RPM;
    }

    // The ESC sensor has a better resolution than DShot extended telemetry
    telemetry->temperature = escData->temperature;
    telemetry->voltage = escData->voltage;
    telemetry->current = escData->current;
    telemetry->consumption = escData->consumption;
    telemetry->valid |= MOTOR_TELEMETRY_TEMPERATURE | MOTOR_TELEMETRY_VOLTAGE | MOTOR_TELEMETRY_CURRENT | MOTOR_TELEMETRY_CONSUMPTION;
}
#endif

static void updateMotorTelemetry(unsigned motorIndex)
{
    motorTelemetry_t *telemetry = &motorTelemetry[motorIndex];

    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->dataAge = ESC_DATA_INVALID;

#ifdef USE_DSHOT_TELEMETRY
    mergeDshotTelemetry(telemetry, motorIndex);
#endif
#ifdef USE_ESC_SENSOR
    mergeEscSensorTelemetry(telemetry, motorIndex);
#endif
}

static const motorTelemetry_t *updateCombinedTelemetry(void)
{
    // A sum is valid only if all motors have the value. An average or a
    // maximum is valid if one or more motors have the value.
    const uint8_t sumFields = MOTOR_TELEMETRY_CURRENT | MOTOR_TELEMETRY_CONSUMPTION;
    const unsigned motorCount = getMotorCount();

    motorTelemetry_t *combined = &motorTelemetryCombined;
    uint32_t voltageSum = 0;
    unsigned rpmCount = 0;
    unsigned voltageCount = 0;
    uint8_t anyValid = 0;
    uint8_t allValid = motorCount ? sumFields : 0;
    bool hasData = false;

    memset(combined, 0, sizeof(*combined));

    for (unsigned i = 0; i < motorCount; i++) {
        updateMotorTelemetry(i);
        const motorTelemetry_t *telemetry = &motorTelemetry[i];

        anyValid |= telemetry->valid;
        allValid &= telemetry->valid;
        combined->sources |= telemetry->sources;

        // Age of the motors that have data. A motor without data does not
        // make the data of the other motors stale.
        if (telemetry->valid) {
            combined->dataAge = MAX(combined->dataAge, telemetry->dataAge);
            hasData = true;
        }

        if (telemetry->valid & MOTOR_TELEMETRY_RPM) {
            combined->rpm += telemetry->rpm;
            rpmCount++;
        }
        if (telemetry->valid & MOTOR_TELEMETRY_TEMPERATURE) {
            combined->temperature = MAX(combined->temperature, telemetry->temperature);
        }
        if (telemetry->valid & MOTOR_TELEMETRY_VOLTAGE) {
            voltageSum += telemetry->voltage;
            voltageCount++;
        }
        if (telemetry->valid & MOTOR_TELEMETRY_CURRENT) {
            combined->current += telemetry->current;
        }
        if (telemetry->valid & MOTOR_TELEMETRY_CONSUMPTION) {
            combined->consumption += telemetry->consumption;
        }
    }

    combined->valid = (anyValid & ~sumFields) | allValid;
    if (!hasData) {
        combined->dataAge = ESC_DATA_INVALID;
    }

    if (rpmCount) {
        combined->rpm /= rpmCount;
    }
    if (voltageCount) {
        combined->voltage = voltageSum / voltageCount;
    }

    return combined;
}

const motorTelemetry_t *motorTelemetryGet(unsigned motorIndex)
{
    if (!motorTelemetryIsEnabled()) {
        return NULL;
    }

    if (motorIndex == MOTOR_TELEMETRY_COMBINED) {
        return updateCombinedTelemetry();
    }

    if (motorIndex >= (unsigned)getMotorCount()) {
        return NULL;
    }

    updateMotorTelemetry(motorIndex);

    return &motorTelemetry[motorIndex];
}

#endif // USE_DSHOT_TELEMETRY || USE_ESC_SENSOR
