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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/logging.h"
#include "drivers/pitotmeter.h"
#include "drivers/pitotmeter_ms4525.h"
#include "drivers/pitotmeter_adc.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/pitotmeter.h"
#include "sensors/sensors.h"

pitot_t pitot;

#ifdef PITOT

static timeMs_t pitotCalibrationTimeout = 0;
static bool pitotCalibrationFinished = false;
static float pitotPressureZero = 0;
static float pitotPressure = 0;
static float pitotTemperature = 0;
static float indicatedAirspeed = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig, PG_PITOTMETER_CONFIG, 0);

#ifdef PITOT
#define PITOT_HARDWARE_DEFAULT    PITOT_AUTODETECT
#else
#define PITOT_HARDWARE_DEFAULT    PITOT_NONE
#endif
PG_RESET_TEMPLATE(pitotmeterConfig_t, pitotmeterConfig,
    .pitot_hardware = PITOT_HARDWARE_DEFAULT,
    .use_median_filtering = 1,
    .pitot_noise_lpf = 0.6f,
    .pitot_scale = 1.00f
);

bool pitotDetect(pitotDev_t *dev, uint8_t pitotHardwareToUse)
{
    pitotSensor_e pitotHardware = PITOT_NONE;
    requestedSensors[SENSOR_INDEX_PITOT] = pitotHardwareToUse;

    switch (pitotHardwareToUse) {
        case PITOT_AUTODETECT:
        case PITOT_MS4525:
#ifdef USE_PITOT_MS4525
            if (ms4525Detect(dev)) {
                pitotHardware = PITOT_MS4525;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }

        case PITOT_ADC:
#if defined(USE_PITOT_ADC)
            if (adcPitotDetect(dev)) {
                pitotHardware = PITOT_ADC;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }

        case PITOT_VIRTUAL:
#if defined(USE_PITOT_VIRTUAL)
            /*
            if (adcPitotDetect(&pitot)) {
                pitotHardware = PITOT_ADC;
                break;
            }
            */
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }

        case PITOT_FAKE:
#ifdef USE_PITOT_FAKE
            if (fakePitotDetect(dev)) {
                pitotHardware = PITOT_FAKE;
                break;
            }
#endif
            /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
            if (pitotHardwareToUse != PITOT_AUTODETECT) {
                break;
            }

        case PITOT_NONE:
            pitotHardware = PITOT_NONE;
            break;
    }

    addBootlogEvent6(BOOT_EVENT_PITOT_DETECTION, BOOT_EVENT_FLAGS_NONE, pitotHardware, 0, 0, 0);

    if (pitotHardware == PITOT_NONE) {
        sensorsClear(SENSOR_PITOT);
        return false;
    }

    detectedSensors[SENSOR_INDEX_PITOT] = pitotHardware;
    sensorsSet(SENSOR_PITOT);
    return true;
}

bool pitotInit(void)
{
    if (!pitotDetect(&pitot.dev, pitotmeterConfig()->pitot_hardware)) {
        return false;
    }
    return true;
}

bool pitotIsCalibrationComplete(void)
{
    return pitotCalibrationFinished;
}

void pitotStartCalibration(void)
{
    pitotCalibrationTimeout = millis();
    pitotCalibrationFinished = false;
}

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyPitotmeterMedianFilter(int32_t newPressureReading)
{
    static int32_t pitotFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    pitotFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(pitotFilterSamples);
    else
        return newPressureReading;
}

typedef enum {
    PITOTMETER_NEEDS_SAMPLES = 0,
    PITOTMETER_NEEDS_CALCULATION,
} pitotmeterState_e;

uint32_t pitotUpdate(void)
{
    static pitotmeterState_e state = PITOTMETER_NEEDS_SAMPLES;

    switch (state) {
        default:
        case PITOTMETER_NEEDS_SAMPLES:
            pitot.dev.start();
            state = PITOTMETER_NEEDS_CALCULATION;
            return pitot.dev.delay;
        break;

        case PITOTMETER_NEEDS_CALCULATION:
            pitot.dev.get();
            pitot.dev.calculate(&pitotPressure, &pitotTemperature);
            DEBUG_SET(DEBUG_PITOT, 0, pitotPressure);
            if (pitotmeterConfig()->use_median_filtering) {
                pitotPressure = applyPitotmeterMedianFilter(pitotPressure);
            }
            DEBUG_SET(DEBUG_PITOT, 1, pitotPressure);
            state = PITOTMETER_NEEDS_SAMPLES;
           return pitot.dev.delay;
        break;
    }
}

#define AIR_DENSITY_SEA_LEVEL_15C   1.225f      // Air density at sea level and 15 degrees Celsius
#define AIR_GAS_CONST               287.1f      //  J / (kg * K)
#define P0                          101325.0f   // standard pressure

static void performPitotCalibrationCycle(void)
{
    const float pitotPressureZeroError = pitotPressure - pitotPressureZero;
    pitotPressureZero += pitotPressureZeroError * 0.15f;

    if (ABS(pitotPressureZeroError) < (P0 * 0.00001f)) {    // 0.001% calibration error
        if ((millis() - pitotCalibrationTimeout) > 250) {
            pitotCalibrationFinished = true;
        }
    }
    else {
        pitotCalibrationTimeout = millis();
    }
}

int32_t pitotCalculateAirSpeed(void)
{
    if (pitotIsCalibrationComplete()) {
        // https://en.wikipedia.org/wiki/Indicated_airspeed
        // Indicated airspeed (IAS) is the airspeed read directly from the airspeed indicator on an aircraft, driven by the pitot-static system.
        // The IAS is an important value for the pilot because it is the indicated speeds which are specified in the aircraft flight manual for
        // such important performance values as the stall speed. A typical aircraft will always stall at the same indicated airspeed (for the current configuration)
        // regardless of density, altitude or true airspeed.
        //
        // Therefore we shouldn't care about CAS/TAS and only calculate IAS since it's more indicative to the pilot and more useful in calculations
        // It also allows us to use pitot_scale to calibrate the dynamic pressure sensor scale
        const float indicatedAirspeed_tmp = pitotmeterConfig()->pitot_scale * sqrtf(2.0f * fabsf(pitotPressure - pitotPressureZero) / AIR_DENSITY_SEA_LEVEL_15C);
        indicatedAirspeed += pitotmeterConfig()->pitot_noise_lpf * (indicatedAirspeed_tmp - indicatedAirspeed);

        DEBUG_SET(DEBUG_PITOT, 2, indicatedAirspeed_tmp);
        DEBUG_SET(DEBUG_PITOT, 3, indicatedAirspeed);

        pitot.airSpeed = indicatedAirspeed * 100;
    } else {
        performPitotCalibrationCycle();
        pitot.airSpeed = 0;
    }
    return pitot.airSpeed;
}

bool pitotIsHealthy(void)
{
    return true;
}

#endif /* PITOT */
