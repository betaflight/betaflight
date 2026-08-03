/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_PITOT

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/pitot/pitot_ms4525.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/pitot.h"
#include "sensors/sensors.h"

#if ENABLE_DRONECAN
#include "io/dronecan/dronecan_airspeed.h"
#endif

#ifndef PITOT_I2C_INSTANCE
#define PITOT_I2C_INSTANCE I2C_DEVICE
#endif

#define PITOT_CALIBRATION_SAMPLES TASK_PITOT_RATE_HZ  // ~1 s of at-rest samples
#define AIR_DENSITY_SEA_LEVEL     1.225f    // kg/m^3
#define PITOT_LPF_HZ              5.0f      // differential-pressure smoothing
#define PITOT_DRONECAN_STALE_US   500000    // drop the DroneCAN source if no frame for 0.5 s
#define PITOT_SIGNAL_TIMEOUT_US   1000000   // no valid sample from any source -> sensor lost

pitot_t pitot;

static uint16_t calibrationCount = 0;
static float calibrationAccum = 0.0f;
static float sourceZero[PITOT_HARDWARE_COUNT];     // at-rest offset captured per source
static bool sourceCalibrated[PITOT_HARDWARE_COUNT];
static pt1Filter_t diffPressureLpf;
static bool lpfInitialised = false;
static bool i2cReady = false;
static pitotSensor_e activeSource = PITOT_NONE;
static timeUs_t lastValidSampleUs = 0;

PG_REGISTER_WITH_RESET_FN(pitotConfig_t, pitotConfig, PG_PITOT_CONFIG, 0);

void pgResetFn_pitotConfig(pitotConfig_t *config)
{
    config->pitot_hardware = PITOT_NONE;
    config->pitot_busType = BUS_TYPE_I2C;
    config->pitot_i2c_device = I2C_DEV_TO_CFG(PITOT_I2C_INSTANCE);
    config->pitot_i2c_address = 0;  // 0 = driver default (MS4525: 0x28)
}

bool pitotIsConfigured(void)
{
    return pitotConfig()->pitot_hardware != PITOT_NONE;
}

static bool detectI2C(void)
{
#ifdef USE_PITOT_MS4525
    extDevice_t *extDev = &pitot.dev.dev;
    if (pitotConfig()->pitot_busType != BUS_TYPE_I2C) {
        return false;
    }
    i2cBusSetInstance(extDev, pitotConfig()->pitot_i2c_device);
    extDev->busType_u.i2c.address = pitotConfig()->pitot_i2c_address;
    return ms4525Detect(&pitot.dev);
#else
    return false;
#endif
}

void pitotInit(void)
{
    if (!pitotIsConfigured()) {
        return;
    }
    // The I2C part is probed synchronously here (registers the bus device); the
    // DroneCAN source is confirmed at runtime on its first frame. SENSOR_PITOT
    // is only set once a source actually delivers a sample (see pitotUpdate).
    const pitotSensor_e hardware = pitotConfig()->pitot_hardware;
    if (hardware == PITOT_MS4525 || hardware == PITOT_DEFAULT) {
        i2cReady = detectI2C();
    }
    // Each source is zeroed on its first at-rest sample (see pitotUpdate).
}

static bool readI2C(float *diffPressurePa, float *temperatureK)
{
    return i2cReady && pitot.dev.read && pitot.dev.read(&pitot.dev, diffPressurePa, temperatureK);
}

static bool readDronecan(float *diffPressurePa, float *temperatureK)
{
#if ENABLE_DRONECAN
    float staticPa;
    if (!dronecanAirspeedGetLatest(diffPressurePa, &staticPa, temperatureK)) {
        return false;
    }
    // cmpTimeUs is signed: reject negative (overflowed) and stale ages so a very
    // old frame is not mistaken for fresh.
    const timeDelta_t ageUs = cmpTimeUs(micros(), dronecanAirspeedLastUpdateUs());
    return ageUs >= 0 && ageUs < PITOT_DRONECAN_STALE_US;
#else
    UNUSED(diffPressurePa);
    UNUSED(temperatureK);
    return false;
#endif
}

// Reads the configured source. AUTO tries I2C first and falls back to DroneCAN;
// an explicit selection reads only that backend, so a source pitotInit never
// validated can never supply data. Returns the source that produced the sample.
static pitotSensor_e readActiveSample(float *diffPressurePa, float *temperatureK)
{
    switch (pitotConfig()->pitot_hardware) {
    case PITOT_MS4525:
        return readI2C(diffPressurePa, temperatureK) ? PITOT_MS4525 : PITOT_NONE;
    case PITOT_DRONECAN:
        return readDronecan(diffPressurePa, temperatureK) ? PITOT_DRONECAN : PITOT_NONE;
    case PITOT_DEFAULT:
        if (readI2C(diffPressurePa, temperatureK)) {
            return PITOT_MS4525;
        }
        if (readDronecan(diffPressurePa, temperatureK)) {
            return PITOT_DRONECAN;
        }
        FALLTHROUGH;
    default:
        return PITOT_NONE;
    }
}

void pitotStartCalibration(void)
{
    calibrationCount = PITOT_CALIBRATION_SAMPLES;
    calibrationAccum = 0.0f;
    if (activeSource != PITOT_NONE) {
        sourceCalibrated[activeSource] = false;
    }
}

bool pitotIsCalibrated(void)
{
    return activeSource != PITOT_NONE && sourceCalibrated[activeSource];
}

static float airspeedFromPressure(float diffPressurePa)
{
    // Indicated airspeed, cm/s: v = sqrt(2*q/rho). Sign carries flow direction.
    const float q = fabsf(diffPressurePa);
    const float v = sqrtf(2.0f * q / AIR_DENSITY_SEA_LEVEL);
    return (diffPressurePa < 0.0f ? -v : v) * 100.0f;
}

uint32_t pitotUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    float diffPressurePa;
    float temperatureK;
    const pitotSensor_e source = readActiveSample(&diffPressurePa, &temperatureK);
    if (source == PITOT_NONE) {
        // No source has data. After a bounded gap, declare the sensor lost so a
        // caller does not keep reading a frozen airspeed.
        if (activeSource != PITOT_NONE
                && cmpTimeUs(currentTimeUs, lastValidSampleUs) > PITOT_SIGNAL_TIMEOUT_US) {
            activeSource = PITOT_NONE;
            sensorsClear(SENSOR_PITOT);
            pitot.diffPressure = 0.0f;
            pitot.airspeed = 0.0f;
        }
        return TASK_PERIOD_HZ(TASK_PITOT_RATE_HZ);
    }
    lastValidSampleUs = currentTimeUs;

    if (source != activeSource) {
        // A backend delivered its first sample, or an AUTO fallback took over.
        // Each source keeps its own zero and filter state; a source is only
        // zeroed once, and only while disarmed, so a mid-flight switch reuses the
        // stored offset instead of capturing dynamic pressure as the zero.
        activeSource = source;
        detectedSensors[SENSOR_INDEX_PITOT] = source;
        sensorsSet(SENSOR_PITOT);
        lpfInitialised = false;
        if (!sourceCalibrated[source] && !ARMING_FLAG(ARMED)) {
            pitotStartCalibration();
        }
    }

    if (!lpfInitialised) {
        pt1FilterInit(&diffPressureLpf, pt1FilterGain(PITOT_LPF_HZ, 1.0f / TASK_PITOT_RATE_HZ));
        lpfInitialised = true;
    }
    diffPressurePa = pt1FilterApply(&diffPressureLpf, diffPressurePa);

    // Zeroing only runs while disarmed (at rest); arming abandons a partial pass.
    if (calibrationCount > 0) {
        if (ARMING_FLAG(ARMED)) {
            calibrationCount = 0;
        } else {
            calibrationAccum += diffPressurePa;
            if (--calibrationCount == 0) {
                sourceZero[source] = calibrationAccum / PITOT_CALIBRATION_SAMPLES;
                sourceCalibrated[source] = true;
            }
        }
    }

    pitot.pressureZero = sourceZero[source];
    pitot.temperature = temperatureK;
    pitot.diffPressure = diffPressurePa - sourceZero[source];
    pitot.airspeed = airspeedFromPressure(pitot.diffPressure);

    DEBUG_SET(DEBUG_PITOT, 0, lrintf(pitot.airspeed));
    DEBUG_SET(DEBUG_PITOT, 1, lrintf(pitot.diffPressure));
    DEBUG_SET(DEBUG_PITOT, 2, lrintf(diffPressurePa));
    DEBUG_SET(DEBUG_PITOT, 3, lrintf(pitot.temperature - 273.15f));

    return TASK_PERIOD_HZ(TASK_PITOT_RATE_HZ);
}

float pitotGetAirspeed(void)
{
    return pitot.airspeed;
}

#endif // USE_PITOT
