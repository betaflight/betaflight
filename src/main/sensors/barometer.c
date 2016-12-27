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

#include "common/maths.h"

#include "drivers/barometer.h"
#include "drivers/barometer_bmp085.h"
#include "drivers/barometer_bmp280.h"
#include "drivers/barometer_fake.h"
#include "drivers/barometer_ms5611.h"
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

baro_t baro;                        // barometer access functions

#ifdef BARO

static uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
static int32_t baroPressure = 0;
static int32_t baroTemperature = 0;

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 0;
static uint32_t baroPressureSum = 0;

static const barometerConfig_t *barometerConfig;

bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse)
{
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;

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

    switch (baroHardware) {
    case BARO_DEFAULT:
        ; // fallthough
    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        if (bmp085Detect(bmp085Config, dev)) {
            baroHardware = BARO_BMP085;
            break;
        }
#endif
        ; // fallthough
    case BARO_MS5611:
#ifdef USE_BARO_MS5611
        if (ms5611Detect(dev)) {
            baroHardware = BARO_MS5611;
            break;
        }
#endif
        ; // fallthough
    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
        if (bmp280Detect(dev)) {
            baroHardware = BARO_BMP280;
            break;
        }
#endif
        ; // fallthough
    case BARO_NONE:
        baroHardware = BARO_NONE;
        break;
    }

    if (baroHardware == BARO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
}

void useBarometerConfig(const barometerConfig_t *barometerConfigToUse)
{
    barometerConfig = barometerConfigToUse;
}

bool isBaroCalibrationComplete(void)
{
    return calibratingB == 0;
}

void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingB = calibrationCyclesRequired;
}

static bool baroReady = false;

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

#define PRESSURE_SAMPLE_COUNT (barometerConfig->baro_sample_count - 1)

static uint32_t recalculateBarometerTotal(uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == baroSampleCount) {
        nextSampleIndex = 0;
        baroReady = true;
    }
    barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

    // recalculate pressure total
    // Note, the pressure total is made up of baroSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

typedef enum {
    BAROMETER_NEEDS_SAMPLES = 0,
    BAROMETER_NEEDS_CALCULATION
} barometerState_e;


bool isBaroReady(void) {
    return baroReady;
}

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
            baro.dev.calculate(&baroPressure, &baroTemperature);
            baroPressureSum = recalculateBarometerTotal(barometerConfig->baro_sample_count, baroPressureSum, baroPressure);
            state = BAROMETER_NEEDS_SAMPLES;
            return baro.dev.ut_delay;
        break;
    }
}

int32_t baroCalculateAltitude(void)
{
    int32_t BaroAlt_tmp;

    // calculates height from ground via baro readings
    // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / PRESSURE_SAMPLE_COUNT) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
    BaroAlt_tmp -= baroGroundAltitude;
    baro.BaroAlt = lrintf((float)baro.BaroAlt * barometerConfig->baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - barometerConfig->baro_noise_lpf)); // additional LPF to reduce baro noise

    return baro.BaroAlt;
}

void performBaroCalibrationCycle(void)
{
    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressureSum / PRESSURE_SAMPLE_COUNT;
    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

    calibratingB--;
}

#endif /* BARO */
