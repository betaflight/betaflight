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

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp085.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/barometer/barometer_ms56xx.h"
#include "drivers/logging.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "flight/hil.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

baro_t baro;                        // barometer access functions

PG_REGISTER_WITH_RESET_TEMPLATE(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);

#ifdef BARO
#define BARO_HARDWARE_DEFAULT    BARO_AUTODETECT
#else
#define BARO_HARDWARE_DEFAULT    BARO_NONE
#endif
PG_RESET_TEMPLATE(barometerConfig_t, barometerConfig,
    .baro_hardware = BARO_HARDWARE_DEFAULT,
    .use_median_filtering = 1
);

#ifdef BARO

static timeMs_t baroCalibrationTimeout = 0;
static bool baroCalibrationFinished = false;
static float baroGroundAltitude = 0;
static float baroGroundPressure = 101325.0f; // 101325 pascal, 1 standard atmosphere

bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse)
{
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = BARO_NONE;
    requestedSensors[SENSOR_INDEX_BARO] = baroHardwareToUse;

#ifdef USE_BARO_BMP085
    const bmp085Config_t *bmp085Config = NULL;

#if defined(BARO_XCLR_GPIO) && defined(BARO_EOC_GPIO)
    static const bmp085Config_t defaultBMP085Config = {
        .xclrIO = IO_TAG(BARO_XCLR_PIN),
        .eocIO = IO_TAG(BARO_EOC_PIN),
    };
    bmp085Config = &defaultBMP085Config;
#endif

#ifdef NAZE
    if (hardwareRevision == NAZE32) {
        bmp085Disable(bmp085Config);
    }
#endif

#endif

    switch (baroHardwareToUse) {
    case BARO_AUTODETECT:
    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        if (bmp085Detect(bmp085Config, dev)) {
            baroHardware = BARO_BMP085;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }

    case BARO_MS5607:
#ifdef USE_BARO_MS5607
        if (ms56xxDetect(dev, BARO_MS5607)) {
            baroHardware = BARO_MS5607;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }

    case BARO_MS5611:
#ifdef USE_BARO_MS5611
        if (ms56xxDetect(dev, BARO_MS5611)) {
            baroHardware = BARO_MS5611;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }

    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
        if (bmp280Detect(dev)) {
            baroHardware = BARO_BMP280;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }

    case BARO_FAKE:
#ifdef USE_FAKE_BARO
        if (fakeBaroDetect(dev)) {
            baroHardware = BARO_FAKE;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (baroHardwareToUse != BARO_AUTODETECT) {
            break;
        }

    case BARO_NONE:
        baroHardware = BARO_NONE;
        break;
    }

    addBootlogEvent6(BOOT_EVENT_BARO_DETECTION, BOOT_EVENT_FLAGS_NONE, baroHardware, 0, 0, 0);

    if (baroHardware == BARO_NONE) {
        sensorsClear(SENSOR_BARO);
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
}

bool baroInit(void)
{
    if (!baroDetect(&baro.dev, barometerConfig()->baro_hardware)) {
        return false;
    }
    return true;
}

bool baroIsCalibrationComplete(void)
{
    return baroCalibrationFinished;
}

void baroStartCalibration(void)
{
    baroCalibrationTimeout = millis();
    baroCalibrationFinished = false;
}

#define PRESSURE_SAMPLES_MEDIAN 3

/*
altitude pressure
      0m   101325Pa
    100m   100129Pa delta = 1196
   1000m    89875Pa
   1100m    88790Pa delta = 1085
At sea level an altitude change of 100m results in a pressure change of 1196Pa, at 1000m pressure change is 1085Pa
So set glitch threshold at 1000 - this represents an altitude change of approximately 100m.
*/
#define PRESSURE_DELTA_GLITCH_THRESHOLD 1000
static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;

    int nextSampleIndex = currentFilterSampleIndex + 1;
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }
    int previousSampleIndex = currentFilterSampleIndex - 1;
    if (previousSampleIndex < 0) {
        previousSampleIndex = PRESSURE_SAMPLES_MEDIAN - 1;
    }
    const int previousPressureReading = barometerFilterSamples[previousSampleIndex];

    if (medianFilterReady) {
        if (ABS(previousPressureReading - newPressureReading) < PRESSURE_DELTA_GLITCH_THRESHOLD) {
            barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
            currentFilterSampleIndex = nextSampleIndex;
            return quickMedianFilter3(barometerFilterSamples);
        } else {
            // glitch threshold exceeded, so just return previous reading and don't add the glitched reading to the filter array
            return barometerFilterSamples[previousSampleIndex];
        }
    } else {
        barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
        currentFilterSampleIndex = nextSampleIndex;
        return newPressureReading;
    }
}

typedef enum {
    BAROMETER_NEEDS_SAMPLES = 0,
    BAROMETER_NEEDS_CALCULATION
} barometerState_e;

uint32_t baroUpdate(void)
{
    static barometerState_e state = BAROMETER_NEEDS_SAMPLES;

    switch (state) {
        default:
        case BAROMETER_NEEDS_SAMPLES:
            baro.dev.get_ut();
            baro.dev.start_up();
            state = BAROMETER_NEEDS_CALCULATION;
            return baro.dev.up_delay;
        break;

        case BAROMETER_NEEDS_CALCULATION:
            baro.dev.get_up();
            baro.dev.start_ut();
            baro.dev.calculate(&baro.baroPressure, &baro.baroTemperature);
            if (barometerConfig()->use_median_filtering) {
                baro.baroPressure = applyBarometerMedianFilter(baro.baroPressure);
            }
            state = BAROMETER_NEEDS_SAMPLES;
            return baro.dev.ut_delay;
        break;
    }
}

static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

static void performBaroCalibrationCycle(void)
{
    const float baroGroundPressureError = baro.baroPressure - baroGroundPressure;
    baroGroundPressure += baroGroundPressureError * 0.15f;

    if (ABS(baroGroundPressureError) < (baroGroundPressure * 0.00005f)) {    // 0.005% calibration error (should give c. 10cm calibration error)
        if ((millis() - baroCalibrationTimeout) > 250) {
            baroGroundAltitude = pressureToAltitude(baroGroundPressure);
            baroCalibrationFinished = true;
        }
    }
    else {
        baroCalibrationTimeout = millis();
    }
}

int32_t baroCalculateAltitude(void)
{
    if (!baroIsCalibrationComplete()) {
        performBaroCalibrationCycle();
        baro.BaroAlt = 0;
    }
    else {
#ifdef HIL
        if (hilActive) {
            baro.BaroAlt = hilToFC.baroAlt;
            return baro.BaroAlt;
        }
#endif
        // calculates height from ground via baro readings
        baro.BaroAlt = pressureToAltitude(baro.baroPressure) - baroGroundAltitude;
    }

    return baro.BaroAlt;
}

int32_t baroGetLatestAltitude(void)
{
    return baro.BaroAlt;
}

bool baroIsHealthy(void)
{
    return true;
}

#endif /* BARO */
