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

// A value is valid only if it is fresh:
// - DShot eRPM arrives with each motor update. Expire it after 500ms.
// - DShot extended telemetry has no fixed interval. The specification uses
//   1000ms as an example for temperature and voltage. Expire a value after
//   5 intervals.
// - ESC sensor data is valid if its age is not more than ESC_BATTERY_AGE_MAX.
#define DSHOT_ERPM_TIMEOUT_MS 500
#define DSHOT_EDT_TIMEOUT_MS  5000

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
static bool isDshotValueFresh(const dshotTelemetryMotorState_t *motorState, dshotTelemetryType_t type, timeMs_t nowMs, timeDelta_t timeoutMs)
{
    return (motorState->telemetryTypes & (1 << type))
        && cmpTimeMs(nowMs, motorState->updateMs[type]) < timeoutMs;
}

// Fresh DShot telemetry values, in motorTelemetry_t units
static void getDshotTelemetry(motorTelemetry_t *dshot, unsigned motorIndex)
{
    if (!useDshotTelemetry) {
        return;
    }

    const dshotTelemetryMotorState_t *motorState = &dshotTelemetryState.motorState[motorIndex];
    const timeMs_t nowMs = millis();

    if (isDshotValueFresh(motorState, DSHOT_TELEMETRY_TYPE_eRPM, nowMs, DSHOT_ERPM_TIMEOUT_MS)) {
        dshot->rpm = lrintf(getDshotRpm(motorIndex));
        dshot->valid |= MOTOR_TELEMETRY_RPM;
    }
    // Temperature: step 1 degree Celsius
    if (isDshotValueFresh(motorState, DSHOT_TELEMETRY_TYPE_TEMPERATURE, nowMs, DSHOT_EDT_TIMEOUT_MS)) {
        dshot->temperature = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE];
        dshot->valid |= MOTOR_TELEMETRY_TEMPERATURE;
    }
    // Voltage: step 0.25V, convert to 0.01V
    if (isDshotValueFresh(motorState, DSHOT_TELEMETRY_TYPE_VOLTAGE, nowMs, DSHOT_EDT_TIMEOUT_MS)) {
        dshot->voltage = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] * 25;
        dshot->valid |= MOTOR_TELEMETRY_VOLTAGE;
    }
    // Current: step 1A, convert to 0.01A
    if (isDshotValueFresh(motorState, DSHOT_TELEMETRY_TYPE_CURRENT, nowMs, DSHOT_EDT_TIMEOUT_MS)) {
        dshot->current = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] * 100;
        dshot->valid |= MOTOR_TELEMETRY_CURRENT;
    }
}
#endif

#ifdef USE_ESC_SENSOR
// Fresh ESC sensor values, in motorTelemetry_t units
static void getEscSensorTelemetry(motorTelemetry_t *esc, unsigned motorIndex)
{
    if (!featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return;
    }

    const escSensorData_t *escData = escSensorGetData(motorIndex);
    if (!escData || escData->dataAge > ESC_BATTERY_AGE_MAX) {
        return;
    }

    esc->rpm = lrintf(erpmToRpm(escData->rpm));
    esc->temperature = escData->temperature;
    esc->voltage = escData->voltage;
    esc->current = escData->current;
    esc->consumption = escData->consumption;
    esc->valid = MOTOR_TELEMETRY_RPM | MOTOR_TELEMETRY_TEMPERATURE | MOTOR_TELEMETRY_VOLTAGE | MOTOR_TELEMETRY_CURRENT | MOTOR_TELEMETRY_CONSUMPTION;
}
#endif

// Copy the fields in mask from src to dst, if src has them and dst does not
static void mergeFields(motorTelemetry_t *dst, const motorTelemetry_t *src, uint8_t mask)
{
    const uint8_t fields = src->valid & ~dst->valid & mask;

    if (fields & MOTOR_TELEMETRY_RPM) {
        dst->rpm = src->rpm;
    }
    if (fields & MOTOR_TELEMETRY_TEMPERATURE) {
        dst->temperature = src->temperature;
    }
    if (fields & MOTOR_TELEMETRY_VOLTAGE) {
        dst->voltage = src->voltage;
    }
    if (fields & MOTOR_TELEMETRY_CURRENT) {
        dst->current = src->current;
    }
    if (fields & MOTOR_TELEMETRY_CONSUMPTION) {
        dst->consumption = src->consumption;
    }
    dst->valid |= fields;
}

static void updateMotorTelemetry(unsigned motorIndex)
{
    motorTelemetry_t dshot = { 0 };
    motorTelemetry_t esc = { 0 };

#ifdef USE_DSHOT_TELEMETRY
    getDshotTelemetry(&dshot, motorIndex);
#endif
#ifdef USE_ESC_SENSOR
    getEscSensorTelemetry(&esc, motorIndex);
#endif

    // RPM: DShot first, it has a higher update rate.
    // Other fields: ESC sensor first, it has a better resolution.
    motorTelemetry_t *telemetry = &motorTelemetry[motorIndex];
    memset(telemetry, 0, sizeof(*telemetry));
    mergeFields(telemetry, &dshot, MOTOR_TELEMETRY_RPM);
    mergeFields(telemetry, &esc, 0xFF);
    mergeFields(telemetry, &dshot, 0xFF);
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

    memset(combined, 0, sizeof(*combined));

    for (unsigned i = 0; i < motorCount; i++) {
        updateMotorTelemetry(i);
        const motorTelemetry_t *telemetry = &motorTelemetry[i];

        anyValid |= telemetry->valid;
        allValid &= telemetry->valid;

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
