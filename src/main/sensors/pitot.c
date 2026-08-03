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

#define PITOT_CALIBRATION_SAMPLES 100
#define AIR_DENSITY_SEA_LEVEL     1.225f   // kg/m^3
#define PITOT_LPF_HZ              5.0f     // differential-pressure smoothing
#define PITOT_DRONECAN_STALE_US   500000   // drop the DroneCAN source if no frame for 0.5 s

pitot_t pitot;

static uint16_t calibrationCount = 0;
static float calibrationAccum = 0.0f;
static pt1Filter_t diffPressureLpf;
static bool lpfInitialised = false;
static bool i2cReady = false;

PG_REGISTER_WITH_RESET_FN(pitotConfig_t, pitotConfig, PG_PITOT_CONFIG, 0);

void pgResetFn_pitotConfig(pitotConfig_t *config)
{
    config->pitot_hardware = PITOT_NONE;
    config->pitot_busType = BUS_TYPE_I2C;
    config->pitot_i2c_device = I2C_DEV_TO_CFG(PITOT_I2C_INSTANCE);
    config->pitot_i2c_address = 0;  // 0 = driver default (MS4525: 0x28)
}

static bool dronecanReady(void)
{
#if ENABLE_DRONECAN
    return true;    // subscriber is installed; per-read freshness is checked separately
#else
    return false;
#endif
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

// Probe both backends the configured source might use, then latch which are
// present. DroneCAN is "present" whenever the transport is compiled in; its
// per-frame freshness gates the read.
static bool pitotDetect(pitotSensor_e hardwareToUse)
{
    i2cReady = false;

    if (hardwareToUse == PITOT_MS4525 || hardwareToUse == PITOT_DEFAULT) {
        i2cReady = detectI2C();
    }
    const bool dronecanPresent =
        (hardwareToUse == PITOT_DRONECAN || hardwareToUse == PITOT_DEFAULT) && dronecanReady();

    if (!i2cReady && !dronecanPresent) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_PITOT] = i2cReady ? PITOT_MS4525 : PITOT_DRONECAN;
    sensorsSet(SENSOR_PITOT);
    return true;
}

void pitotInit(void)
{
    if (pitotConfig()->pitot_hardware == PITOT_NONE) {
        return;
    }
    if (pitotDetect(pitotConfig()->pitot_hardware)) {
        pitotStartCalibration();    // zero the at-rest offset over the first samples
    }
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
    return cmpTimeUs(micros(), dronecanAirspeedLastUpdateUs()) < PITOT_DRONECAN_STALE_US;
#else
    UNUSED(diffPressurePa);
    UNUSED(temperatureK);
    return false;
#endif
}

static bool readSource(pitotSensor_e source, float *diffPressurePa, float *temperatureK)
{
    switch (source) {
    case PITOT_MS4525:
        return readI2C(diffPressurePa, temperatureK);
    case PITOT_DRONECAN:
        return readDronecan(diffPressurePa, temperatureK);
    default:
        return false;
    }
}

void pitotStartCalibration(void)
{
    calibrationCount = PITOT_CALIBRATION_SAMPLES;
    calibrationAccum = 0.0f;
}

bool pitotIsCalibrated(void)
{
    return calibrationCount == 0;
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

    // Configured source is primary; the other backend is the staleness fallback.
    pitotSensor_e primary = pitotConfig()->pitot_hardware;
    if (primary == PITOT_DEFAULT) {
        primary = i2cReady ? PITOT_MS4525 : PITOT_DRONECAN;
    }
    const pitotSensor_e secondary = (primary == PITOT_DRONECAN) ? PITOT_MS4525 : PITOT_DRONECAN;

    float diffPressurePa;
    float temperatureK;
    if (!readSource(primary, &diffPressurePa, &temperatureK)
            && !readSource(secondary, &diffPressurePa, &temperatureK)) {
        return TASK_PERIOD_HZ(TASK_PITOT_RATE_HZ);
    }

    if (!lpfInitialised) {
        pt1FilterInit(&diffPressureLpf, pt1FilterGain(PITOT_LPF_HZ, 1.0f / TASK_PITOT_RATE_HZ));
        lpfInitialised = true;
    }
    diffPressurePa = pt1FilterApply(&diffPressureLpf, diffPressurePa);

    if (calibrationCount > 0) {
        calibrationAccum += diffPressurePa;
        if (--calibrationCount == 0) {
            pitot.pressureZero = calibrationAccum / PITOT_CALIBRATION_SAMPLES;
        }
    }

    pitot.temperature = temperatureK;
    pitot.diffPressure = diffPressurePa - pitot.pressureZero;
    pitot.airspeed = airspeedFromPressure(pitot.diffPressure);

    DEBUG_SET(DEBUG_PITOT, 0, lrintf(pitot.airspeed));
    DEBUG_SET(DEBUG_PITOT, 1, lrintf(pitot.diffPressure));
    DEBUG_SET(DEBUG_PITOT, 2, lrintf(diffPressurePa));
    DEBUG_SET(DEBUG_PITOT, 3, lrintf(pitot.temperature - 273.15f));

    return TASK_PERIOD_HZ(TASK_PITOT_RATE_HZ);
}

float getAirspeed(void)
{
    return pitot.airspeed;
}

#endif // USE_PITOT
