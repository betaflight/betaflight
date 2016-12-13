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
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "config/config.h"

#include "drivers/logging.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/rangefinder.h"
#include "sensors/initialisation.h"

uint8_t requestedSensors[SENSOR_INDEX_COUNT] = { GYRO_AUTODETECT, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE, PITOT_NONE };
uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE, PITOT_NONE };


bool sensorsAutodetect(const gyroConfig_t *gyroConfig,
                const accelerometerConfig_t *accConfig,
                const compassConfig_t *compassConfig,
                const barometerConfig_t *baroConfig,
                const pitotmeterConfig_t *pitotConfig)
{
    if (!gyroInit(gyroConfig)) {
        return false;
    }

#ifdef ASYNC_GYRO_PROCESSING
     // ACC will be updated at its own rate
    accInit(accConfig, getAccUpdateRate());
#else
    // acc updated at same frequency in taskMainPidLoop in mw.c
    accInit(accConfig, gyro.targetLooptime);
#endif

#ifdef BARO
<<<<<<< HEAD
    baroDetect(&baro.dev, baroConfig->baro_hardware);
=======
static bool detectBaro(baroSensor_e baroHardwareToUse)
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
            if (bmp085Detect(bmp085Config, &baro)) {
                baroHardware = BARO_BMP085;
                break;
            }
#endif
        ; // fallthough
        case BARO_MS5611:
#ifdef USE_BARO_MS5611
            if (ms5611Detect(&baro)) {
                baroHardware = BARO_MS5611;
                break;
            }
#endif
            ; // fallthough
        case BARO_BMP280:
#if defined(USE_BARO_BMP280) || defined(USE_BARO_SPI_BMP280)
            if (bmp280Detect(&baro)) {
                baroHardware = BARO_BMP280;
                break;
            }
#endif
            ; // fallthrough
        case BARO_FAKE:
#ifdef USE_FAKE_BARO
            if (fakeBaroDetect(&baro)) {
                baroHardware = BARO_FAKE;
                break;
            }
#endif
            ; // fallthrough
        case BARO_NONE:
            baroHardware = BARO_NONE;
            break;
    }

    addBootlogEvent6(BOOT_EVENT_BARO_DETECTION, BOOT_EVENT_FLAGS_NONE, baroHardware, 0, 0, 0);

    if (baroHardware == BARO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
}
#endif // BARO

#ifdef PITOT
static bool detectPitot(uint8_t pitotHardwareToUse)
{
    pitotSensor_e pitotHardware = pitotHardwareToUse;

    switch (pitotHardware) {
        case PITOT_DEFAULT:
            ; // Fallthrough

        case PITOT_MS4525:
#ifdef USE_PITOT_MS4525
            if (ms4525Detect(&pitot)) {
                pitotHardware = PITOT_MS4525;
                break;
            }
#endif
            ; // Fallthrough

        case PITOT_FAKE:
#ifdef USE_PITOT_FAKE
            if (fakePitotDetect(&pitot)) {
                pitotHardware = PITOT_FAKE;
                break;
            }
#endif
            ; // Fallthrough

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
#endif

#ifdef MAG
static bool detectMag(magSensor_e magHardwareToUse)
{
    magSensor_e magHardware = MAG_NONE;

#ifdef USE_MAG_HMC5883
    const hmc5883Config_t *hmc5883Config = 0;

#ifdef NAZE // TODO remove this target specific define
    static const hmc5883Config_t nazeHmc5883Config_v1_v4 = {
            .intTag = IO_TAG(PB12) /* perhaps disabled? */
    };
    static const hmc5883Config_t nazeHmc5883Config_v5 = {
            .intTag = IO_TAG(MAG_INT_EXTI)
    };
    if (hardwareRevision < NAZE32_REV5) {
        hmc5883Config = &nazeHmc5883Config_v1_v4;
    } else {
        hmc5883Config = &nazeHmc5883Config_v5;
    }
#endif

#ifdef MAG_INT_EXTI
    static const hmc5883Config_t extiHmc5883Config = {
        .intTag = IO_TAG(MAG_INT_EXTI)
    };

    hmc5883Config = &extiHmc5883Config;
#endif

#endif

    magAlign = ALIGN_DEFAULT;

    switch(magHardwareToUse) {
        case MAG_DEFAULT:
            ; // fallthrough

        case MAG_HMC5883:
#ifdef USE_MAG_HMC5883
            if (hmc5883lDetect(&mag, hmc5883Config)) {
#ifdef MAG_HMC5883_ALIGN
                magAlign = MAG_HMC5883_ALIGN;
#endif
                magHardware = MAG_HMC5883;
                break;
            }
#endif
            ; // fallthrough

        case MAG_AK8975:
#ifdef USE_MAG_AK8975
            if (ak8975Detect(&mag)) {
#ifdef MAG_AK8975_ALIGN
                magAlign = MAG_AK8975_ALIGN;
#endif
                magHardware = MAG_AK8975;
                break;
            }
#endif
            ; // fallthrough

        case MAG_AK8963:
#ifdef USE_MAG_AK8963
            if (ak8963Detect(&mag)) {
#ifdef MAG_AK8963_ALIGN
                magAlign = MAG_AK8963_ALIGN;
#endif
                magHardware = MAG_AK8963;
                break;
            }
#endif
            ; // fallthrough

        case MAG_GPS:
#ifdef GPS
            if (gpsMagDetect(&mag)) {
#ifdef MAG_GPS_ALIGN
                magAlign = MAG_GPS_ALIGN;
#endif
                magHardware = MAG_GPS;
                break;
            }
#endif
            ; // fallthrough

        case MAG_MAG3110:
#ifdef USE_MAG_MAG3110
            if (mag3110detect(&mag)) {
#ifdef MAG_MAG3110_ALIGN
                magAlign = MAG_MAG3110_ALIGN;
#endif
                magHardware = MAG_MAG3110;
                break;
            }
#endif
            ; // fallthrough

        case MAG_IST8310:
#ifdef USE_MAG_IST8310
        	if (ist8310Detect(&mag)) {
#ifdef MAG_IST8310_ALIGN
        		magAlign = MAG_IST8310_ALIGN;
#endif
        		magHardware = MAG_IST8310;
        		break;
        	}
#endif
        	; // fallthrough

        case MAG_FAKE:
#ifdef USE_FAKE_MAG
            if (fakeMagDetect(&mag)) {
                magHardware = MAG_FAKE;
                break;
            }
#endif
            ; // fallthrough

        case MAG_NONE:
            magHardware = MAG_NONE;
            break;
    }

    addBootlogEvent6(BOOT_EVENT_MAG_DETECTION, BOOT_EVENT_FLAGS_NONE, magHardware, 0, 0, 0);

    // If not in autodetect mode and detected the wrong chip - disregard the compass even if detected
    if ((magHardwareToUse != MAG_DEFAULT && magHardware != magHardwareToUse) || (magHardware == MAG_NONE)) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);
    return true;
}
#endif // MAG

#ifdef SONAR
/*
 * Detect which rangefinder is present
 */
static rangefinderType_e detectRangefinder(void)
{
    rangefinderType_e rangefinderType = RANGEFINDER_NONE;
    if (feature(FEATURE_SONAR)) {
        // the user has set the sonar feature, so assume they have an HC-SR04 plugged in,
        // since there is no way to detect it
        rangefinderType = RANGEFINDER_HCSR04;
    }
#ifdef USE_SONAR_SRF10
    if (srf10_detect()) {
        // if an SFR10 sonar rangefinder is detected then use it in preference to the assumed HC-SR04
        rangefinderType = RANGEFINDER_SRF10;
    }
#endif

    addBootlogEvent6(BOOT_EVENT_RANGEFINDER_DETECTION, BOOT_EVENT_FLAGS_NONE, rangefinderType, 0, 0, 0);

    detectedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderType;

    if (rangefinderType != RANGEFINDER_NONE) {
        sensorsSet(SENSOR_SONAR);
    }

    return rangefinderType;
}
#endif

static void reconfigureAlignment(const sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    if (sensorAlignmentConfig->gyro_align != ALIGN_DEFAULT) {
        gyroAlign = sensorAlignmentConfig->gyro_align;
    }
    if (sensorAlignmentConfig->acc_align != ALIGN_DEFAULT) {
        accAlign = sensorAlignmentConfig->acc_align;
    }
    if (sensorAlignmentConfig->mag_align != ALIGN_DEFAULT) {
        magAlign = sensorAlignmentConfig->mag_align;
    }
}

bool sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig,
        uint8_t accHardwareToUse,
        uint8_t magHardwareToUse,
        uint8_t baroHardwareToUse,
        uint8_t pitotHardwareToUse,
        int16_t magDeclinationFromConfig,
        uint32_t looptime,
        uint8_t gyroLpf,
        uint8_t gyroSync,
        uint8_t gyroSyncDenominator)
{
    memset(&acc, 0, sizeof(acc));
    memset(&gyro, 0, sizeof(gyro));

#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_MPU6050) || defined(USE_GYRO_SPI_MPU9250)
    const extiConfig_t *extiConfig = selectMPUIntExtiConfig();
    detectMpu(extiConfig);
#endif

    if (!detectGyro()) {
        return false;
    }

    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.targetLooptime = gyroSetSampleRate(looptime, gyroLpf, gyroSync, gyroSyncDenominator);    // Set gyro sample rate before initialisation
    gyro.init(gyroLpf); // driver initialisation
    gyroInit(); // sensor initialisation

    if (detectAcc(accHardwareToUse)) {
        acc.acc_1G = 256; // set default
        acc.init(&acc);
    #ifdef ASYNC_GYRO_PROCESSING
        /*
         * ACC will be updated at its own rate
         */
        accInit(getAccUpdateRate());
    #else
        /*
         * acc updated at same frequency in taskMainPidLoop in mw.c
         */
        accInit(gyro.targetLooptime);
    #endif
    }

#ifdef BARO
    detectBaro(baroHardwareToUse);
>>>>>>> 46da00f... fix ist8310 init wrong and init target file.
#else
    UNUSED(baroConfig);
#endif

#ifdef PITOT
    pitotDetect(&pitot.dev, pitotConfig->pitot_hardware);
#else
    UNUSED(pitotConfig);
#endif

    // FIXME extract to a method to reduce dependencies, maybe move to sensors_compass.c
    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
#ifdef MAG
    if (compassDetect(&mag.dev, compassConfig->mag_hardware)) {
        // calculate magnetic declination
        if (!compassInit(compassConfig)) {
            addBootlogEvent2(BOOT_EVENT_MAG_INIT_FAILED, BOOT_EVENT_FLAGS_ERROR);
            sensorsClear(SENSOR_MAG);
        }
    }
#else
    UNUSED(compassConfig);
#endif

#ifdef SONAR
    const rangefinderType_e rangefinderType = rangefinderDetect();
    rangefinderInit(rangefinderType);
#endif
    if (gyroConfig->gyro_align != ALIGN_DEFAULT) {
        gyro.dev.gyroAlign = gyroConfig->gyro_align;
    }
    if (accConfig->acc_align != ALIGN_DEFAULT) {
        acc.dev.accAlign = accConfig->acc_align;
    }
    if (compassConfig->mag_align != ALIGN_DEFAULT) {
        mag.dev.magAlign = compassConfig->mag_align;
    }

    return true;
}
