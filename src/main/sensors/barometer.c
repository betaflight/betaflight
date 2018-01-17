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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp085.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/barometer/barometer_ms5611.h"
#include "drivers/barometer/barometer_lps.h"

#include "fc/runtime_config.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

baro_t baro;                        // barometer access functions

PG_REGISTER_WITH_RESET_FN(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);

void pgResetFn_barometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 600;
    barometerConfig->baro_cf_vel = 985;
    barometerConfig->baro_cf_alt = 965;
    barometerConfig->baro_hardware = BARO_DEFAULT;

    // For backward compatibility; ceate a valid default value for bus parameters
    //
    // 1. If DEFAULT_BARO_xxx is defined, use it.
    // 2. Determine default based on USE_BARO_xxx
    //   a. Precedence is in the order of popularity; BMP280, MS5611 then BMP085, then
    //   b. If SPI variant is specified, it is likely onboard, so take it.

#if !(defined(DEFAULT_BARO_SPI_BMP280) || defined(DEFAULT_BARO_BMP280) || defined(DEFAULT_BARO_SPI_MS5611) || defined(DEFAULT_BARO_MS5611) || defined(DEFAULT_BARO_BMP085) || defined(DEFAULT_BARO_SPI_LPS))
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
#if defined(USE_BARO_SPI_BMP280)
#define DEFAULT_BARO_SPI_BMP280
#else
#define DEFAULT_BARO_BMP280
#endif
#elif defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611)
#if defined(USE_BARO_SPI_MS5611)
#define DEFAULT_BARO_SPI_MS5611
#else
#define DEFAULT_BARO_MS5611
#endif
#elif defined(USE_BARO_SPI_LPS)
#define DEFAULT_BARO_SPI_LPS
#elif defined(DEFAULT_BARO_BMP085)
#define DEFAULT_BARO_BMP085
#endif
#endif

#if defined(DEFAULT_BARO_SPI_BMP280)
    barometerConfig->baro_bustype = BUSTYPE_SPI;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(BMP280_SPI_INSTANCE));
    barometerConfig->baro_spi_csn = IO_TAG(BMP280_CS_PIN);
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
#elif defined(DEFAULT_BARO_SPI_MS5611)
    barometerConfig->baro_bustype = BUSTYPE_SPI;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(MS5611_SPI_INSTANCE));
    barometerConfig->baro_spi_csn = IO_TAG(MS5611_CS_PIN);
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
#elif defined(DEFAULT_BARO_SPI_LPS)
    barometerConfig->baro_bustype = BUSTYPE_SPI;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(spiDeviceByInstance(LPS_SPI_INSTANCE));
    barometerConfig->baro_spi_csn = IO_TAG(LPS_CS_PIN);
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
#elif defined(DEFAULT_BARO_MS5611) || defined(DEFAULT_BARO_BMP280) || defined(DEFAULT_BARO_BMP085)
    // All I2C devices shares a default config with address = 0 (per device default)
    barometerConfig->baro_bustype = BUSTYPE_I2C;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(BARO_I2C_INSTANCE);
    barometerConfig->baro_i2c_address = 0;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
#else
    barometerConfig->baro_hardware = BARO_NONE;
    barometerConfig->baro_bustype = BUSTYPE_NONE;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(I2CINVALID);
    barometerConfig->baro_i2c_address = 0;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
#endif
}

#ifdef USE_BARO

static uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
static int32_t baroPressure = 0;
static int32_t baroTemperature = 0;

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 8*101325;
static uint32_t baroPressureSum = 0;

bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse)
{
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;

#if !defined(USE_BARO_BMP085) && !defined(USE_BARO_MS5611) && !defined(USE_BARO_SPI_MS5611) && !defined(USE_BARO_BMP280) && !defined(USE_BARO_SPI_BMP280)
    UNUSED(dev);
#endif

    switch (barometerConfig()->baro_bustype) {
    case BUSTYPE_I2C:
#ifdef USE_I2C
        dev->busdev.bustype = BUSTYPE_I2C;
        dev->busdev.busdev_u.i2c.device = I2C_CFG_TO_DEV(barometerConfig()->baro_i2c_device);
        dev->busdev.busdev_u.i2c.address = barometerConfig()->baro_i2c_address;
#endif
        break;

    case BUSTYPE_SPI:
#ifdef USE_SPI
        dev->busdev.bustype = BUSTYPE_SPI;
        spiBusSetInstance(&dev->busdev, spiInstanceByDevice(SPI_CFG_TO_DEV(barometerConfig()->baro_spi_device)));
        dev->busdev.busdev_u.spi.csnPin = IOGetByTag(barometerConfig()->baro_spi_csn);
#endif
        break;

    default:
        return false;
    }

    switch (baroHardware) {
    case BARO_DEFAULT:
        FALLTHROUGH;

    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        {
            const bmp085Config_t *bmp085Config = NULL;

#if defined(BARO_XCLR_GPIO) && defined(BARO_EOC_GPIO)
            static const bmp085Config_t defaultBMP085Config = {
                .xclrIO = IO_TAG(BARO_XCLR_PIN),
                .eocIO = IO_TAG(BARO_EOC_PIN),
            };
            bmp085Config = &defaultBMP085Config;
#endif

            if (bmp085Detect(bmp085Config, dev)) {
                baroHardware = BARO_BMP085;
                break;
            }
        }
#endif
        FALLTHROUGH;

    case BARO_MS5611:
#if defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611)
        if (ms5611Detect(dev)) {
            baroHardware = BARO_MS5611;
            break;
        }
#endif
        FALLTHROUGH;

    case BARO_LPS:
#if defined(USE_BARO_SPI_LPS)
        if (lpsDetect(dev)) {
            baroHardware = BARO_LPS;
            break;
        }
#endif
        FALLTHROUGH;

    case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
        if (bmp280Detect(dev)) {
            baroHardware = BARO_BMP280;
            break;
        }
#endif
        FALLTHROUGH;

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

#define PRESSURE_SAMPLE_COUNT (barometerConfig()->baro_sample_count - 1)

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
            baro.dev.get_ut(&baro.dev);
            baro.dev.start_up(&baro.dev);
            state = BAROMETER_NEEDS_CALCULATION;
            return baro.dev.up_delay;
        break;

        case BAROMETER_NEEDS_CALCULATION:
            baro.dev.get_up(&baro.dev);
            baro.dev.start_ut(&baro.dev);
            baro.dev.calculate(&baroPressure, &baroTemperature);
            baro.baroPressure = baroPressure;
            baro.baroTemperature = baroTemperature;
            baroPressureSum = recalculateBarometerTotal(barometerConfig()->baro_sample_count, baroPressureSum, baroPressure);
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
    if (isBaroCalibrationComplete()) {
        BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / PRESSURE_SAMPLE_COUNT) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
        BaroAlt_tmp -= baroGroundAltitude;
        baro.BaroAlt = lrintf((float)baro.BaroAlt * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_noise_lpf) + (float)BaroAlt_tmp * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_noise_lpf))); // additional LPF to reduce baro noise
    }
    else {
        baro.BaroAlt = 0;
    }
    return baro.BaroAlt;
}

void performBaroCalibrationCycle(void)
{
    static int32_t savedGroundPressure = 0;

    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressureSum / PRESSURE_SAMPLE_COUNT;
    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

    if (baroGroundPressure == savedGroundPressure)
      calibratingB = 0;
    else {
      calibratingB--;
      savedGroundPressure=baroGroundPressure;
    }
}

#endif /* BARO */
