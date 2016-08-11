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

#include <platform.h>

#include "build/build_config.h"

#include "config/parameter_group.h"

#include "common/axis.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/exti.h"

#include "drivers/sensor.h"

#include "drivers/accgyro.h"
#include "drivers/accgyro_adxl345.h"
#include "drivers/accgyro_bma280.h"
#include "drivers/accgyro_l3g4200d.h"
#include "drivers/accgyro_mma845x.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_mpu6500.h"
#include "drivers/accgyro_l3gd20.h"
#include "drivers/accgyro_lsm303dlhc.h"

#include "drivers/bus_spi.h"
#include "drivers/accgyro_spi_mpu6000.h"
#include "drivers/accgyro_spi_mpu6500.h"
#include "drivers/gyro_sync.h"

#include "drivers/barometer.h"
#include "drivers/barometer_bmp085.h"
#include "drivers/barometer_bmp280.h"
#include "drivers/barometer_ms5611.h"

#include "drivers/compass.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/compass_ak8975.h"
#include "drivers/compass_ak8963.h"

#include "drivers/sonar_hcsr04.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/sonar.h"
#include "sensors/initialisation.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

extern gyro_t gyro;
extern baro_t baro;
extern acc_t acc;

uint8_t detectedSensors[MAX_SENSORS_TO_DETECT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };


const extiConfig_t *selectMPUIntExtiConfig(void)
{
#ifdef NAZE
    // MPU_INT output on rev4 PB13
    static const extiConfig_t nazeRev4MPUIntExtiConfig = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOB,
            .gpioPin = Pin_13,
            .gpioPort = GPIOB,
            .io = IO_TAG(PB13),
    };
    // MPU_INT output on rev5 hardware PC13
    static const extiConfig_t nazeRev5MPUIntExtiConfig = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
            .gpioPin = Pin_13,
            .gpioPort = GPIOC,
            .io = IO_TAG(PC13),
    };

    if (hardwareRevision < NAZE32_REV5) {
        return &nazeRev4MPUIntExtiConfig;
    } else {
        return &nazeRev5MPUIntExtiConfig;
    }
#endif

#if defined(SPRACINGF3) || defined(SPRACINGF3MINI) || defined(SPRACINGF3EVO)
    static const extiConfig_t spRacingF3MPUIntExtiConfig = {
            .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
            .gpioPort = GPIOC,
            .gpioPin = Pin_13,
            .io = IO_TAG(PC13),
    };
    return &spRacingF3MPUIntExtiConfig;
#endif

#if defined(CC3D)
    static const extiConfig_t cc3dMPUIntExtiConfig = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOA,
            .gpioPort = GPIOA,
            .gpioPin = Pin_3,
            .io = IO_TAG(PA3),
    };
    return &cc3dMPUIntExtiConfig;
#endif

#if defined(MOTOLAB) || defined(RCEXPLORERF3)
    static const extiConfig_t MotolabF3MPUIntExtiConfig = {
            .gpioAHBPeripherals = RCC_AHBPeriph_GPIOA,
            .gpioPort = GPIOA,
            .gpioPin = Pin_15,
            .io = IO_TAG(PA15),
    };
    return &MotolabF3MPUIntExtiConfig;
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
    static const extiConfig_t colibriRaceMPUIntExtiConfig = {
         .gpioAHBPeripherals = RCC_AHBPeriph_GPIOA,
         .gpioPort = GPIOA,
         .gpioPin = Pin_5,
         .io = IO_TAG(PA5),
    };
    return &colibriRaceMPUIntExtiConfig;
#endif

#ifdef ALIENFLIGHTF3
    // MPU_INT output on V1 PA15
    static const extiConfig_t alienFlightF3V1MPUIntExtiConfig = {
            .gpioAHBPeripherals = RCC_AHBPeriph_GPIOA,
            .gpioPort = GPIOA,
            .gpioPin = Pin_15,
            .io = IO_TAG(PA15),
    };
    // MPU_INT output on V2 PB13
    static const extiConfig_t alienFlightF3V2MPUIntExtiConfig = {
            .gpioAHBPeripherals = RCC_AHBPeriph_GPIOB,
            .gpioPort = GPIOB,
            .gpioPin = Pin_13,
            .io = IO_TAG(PB13),
    };
    if (hardwareRevision == AFF3_REV_1) {
        return &alienFlightF3V1MPUIntExtiConfig;
    } else {
        return &alienFlightF3V2MPUIntExtiConfig;
    }
#endif

    return NULL;
}

#ifdef USE_FAKE_GYRO
static void fakeGyroInit(uint8_t lpf)
{
    UNUSED(lpf);
}

static bool fakeGyroRead(int16_t *gyroADC)
{
    memset(gyroADC, 0, sizeof(int16_t[XYZ_AXIS_COUNT]));
    return true;
}

static bool fakeGyroReadTemp(int16_t *tempData)
{
    UNUSED(tempData);
    return true;
}

bool fakeGyroDetect(gyro_t *gyro)
{
    gyro->init = fakeGyroInit;
    gyro->read = fakeGyroRead;
    gyro->temperature = fakeGyroReadTemp;
    return true;
}
#endif

#ifdef USE_FAKE_ACC
static void fakeAccInit(struct acc_s *acc)
{
    UNUSED(acc);
}

static bool fakeAccRead(int16_t *accData)
{
    memset(accData, 0, sizeof(int16_t[XYZ_AXIS_COUNT]));
    return true;
}

bool fakeAccDetect(acc_t *acc)
{
    acc->init = fakeAccInit;
    acc->read = fakeAccRead;
    acc->revisionCode = 0;
    return true;
}
#endif

bool detectGyro(void)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;
    gyroAlign = ALIGN_DEFAULT;


    switch(gyroHardware) {
        case GYRO_DEFAULT:
            ; // fallthrough
        case GYRO_MPU6050:
#ifdef USE_GYRO_MPU6050
            if (mpu6050GyroDetect(&gyro)) {
#ifdef GYRO_MPU6050_ALIGN
                gyroHardware = GYRO_MPU6050;
                gyroAlign = GYRO_MPU6050_ALIGN;
#endif
                break;
            }
#endif
            ; // fallthrough
        case GYRO_L3G4200D:
#ifdef USE_GYRO_L3G4200D
            if (l3g4200dDetect(&gyro)) {
#ifdef GYRO_L3G4200D_ALIGN
                gyroHardware = GYRO_L3G4200D;
                gyroAlign = GYRO_L3G4200D_ALIGN;
#endif
                break;
            }
#endif
            ; // fallthrough

        case GYRO_MPU3050:
#ifdef USE_GYRO_MPU3050
            if (mpu3050Detect(&gyro)) {
#ifdef GYRO_MPU3050_ALIGN
                gyroHardware = GYRO_MPU3050;
                gyroAlign = GYRO_MPU3050_ALIGN;
#endif
                break;
            }
#endif
            ; // fallthrough

        case GYRO_L3GD20:
#ifdef USE_GYRO_L3GD20
            if (l3gd20Detect(&gyro)) {
#ifdef GYRO_L3GD20_ALIGN
                gyroHardware = GYRO_L3GD20;
                gyroAlign = GYRO_L3GD20_ALIGN;
#endif
                break;
            }
#endif
            ; // fallthrough

        case GYRO_MPU6000:
#ifdef USE_GYRO_SPI_MPU6000
            if (mpu6000SpiGyroDetect(&gyro)) {
#ifdef GYRO_MPU6000_ALIGN
                gyroHardware = GYRO_MPU6000;
                gyroAlign = GYRO_MPU6000_ALIGN;
#endif
                break;
            }
#endif
            ; // fallthrough

        case GYRO_MPU6500:
#ifdef USE_GYRO_MPU6500
            if (mpu6500GyroDetect(&gyro)) {
                gyroHardware = GYRO_MPU6500;
#ifdef GYRO_MPU6500_ALIGN
                gyroAlign = GYRO_MPU6500_ALIGN;
#endif

                break;
            }
#endif

#ifdef USE_GYRO_SPI_MPU6500
            if (mpu6500SpiGyroDetect(&gyro)) {
                gyroHardware = GYRO_MPU6500;
#ifdef GYRO_MPU6500_ALIGN
                gyroAlign = GYRO_MPU6500_ALIGN;
#endif
                break;
            }
#endif
            ; // fallthrough

        case GYRO_FAKE:
#ifdef USE_FAKE_GYRO
            if (fakeGyroDetect(&gyro)) {
                gyroHardware = GYRO_FAKE;
                break;
            }
#endif
            ; // fallthrough
        case GYRO_NONE:
            gyroHardware = GYRO_NONE;
    }

    if (gyroHardware == GYRO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
    sensorsSet(SENSOR_GYRO);

    return true;
}

static void detectAcc(accelerationSensor_e accHardwareToUse)
{
    bool sensorDetected;
    UNUSED(sensorDetected); // avoid unused-variable warning on some targets.

    accelerationSensor_e accHardware;

#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif

retry:
    accAlign = ALIGN_DEFAULT;

    switch (accHardwareToUse) {
        case ACC_DEFAULT:
            ; // fallthrough
        case ACC_ADXL345: // ADXL345
#ifdef USE_ACC_ADXL345
            acc_params.useFifo = false;
            acc_params.dataRate = 800; // unused currently
#ifdef NAZE
            sensorDetected = (hardwareRevision < NAZE32_REV5) && adxl345Detect(&acc_params, &acc);
#else
            sensorDetected = adxl345Detect(&acc_params, &acc);
#endif
            if (sensorDetected) {
#ifdef ACC_ADXL345_ALIGN
                accAlign = ACC_ADXL345_ALIGN;
#endif
                accHardware = ACC_ADXL345;
                break;
            }
#endif
            ; // fallthrough
        case ACC_LSM303DLHC:
#ifdef USE_ACC_LSM303DLHC
            if (lsm303dlhcAccDetect(&acc)) {
#ifdef ACC_LSM303DLHC_ALIGN
                accAlign = ACC_LSM303DLHC_ALIGN;
#endif
                accHardware = ACC_LSM303DLHC;
                break;
            }
#endif
            ; // fallthrough
        case ACC_MPU6050: // MPU6050
#ifdef USE_ACC_MPU6050
            if (mpu6050AccDetect(&acc)) {
#ifdef ACC_MPU6050_ALIGN
                accAlign = ACC_MPU6050_ALIGN;
#endif
                accHardware = ACC_MPU6050;
                break;
            }
#endif
            ; // fallthrough
        case ACC_MMA8452: // MMA8452
#ifdef USE_ACC_MMA8452
#ifdef NAZE
            sensorDetected = (hardwareRevision < NAZE32_REV5) && mma8452Detect(&acc);
#else
            sensorDetected = mma8452Detect(&acc);
#endif
            if (sensorDetected) {
#ifdef ACC_MMA8452_ALIGN
                accAlign = ACC_MMA8452_ALIGN;
#endif
                accHardware = ACC_MMA8452;
                break;
            }
#endif
            ; // fallthrough
        case ACC_BMA280: // BMA280
#ifdef USE_ACC_BMA280
            if (bma280Detect(&acc)) {
#ifdef ACC_BMA280_ALIGN
                accAlign = ACC_BMA280_ALIGN;
#endif
                accHardware = ACC_BMA280;
                break;
            }
#endif
            ; // fallthrough
        case ACC_MPU6000:
#ifdef USE_ACC_SPI_MPU6000
            if (mpu6000SpiAccDetect(&acc)) {
#ifdef ACC_MPU6000_ALIGN
                accAlign = ACC_MPU6000_ALIGN;
#endif
                accHardware = ACC_MPU6000;
                break;
            }
#endif
            ; // fallthrough
        case ACC_MPU6500:
#ifdef USE_ACC_MPU6500
            if (mpu6500AccDetect(&acc)) {
#ifdef ACC_MPU6500_ALIGN
                accAlign = ACC_MPU6500_ALIGN;
#endif
                accHardware = ACC_MPU6500;
                break;
            }
#endif

#ifdef USE_ACC_SPI_MPU6500
            if (mpu6500SpiAccDetect(&acc)) {
#ifdef ACC_MPU6500_ALIGN
                accAlign = ACC_MPU6500_ALIGN;
#endif
                accHardware = ACC_MPU6500;
                break;
            }
#endif
            ; // fallthrough
        case ACC_FAKE:
#ifdef USE_FAKE_ACC
            if (fakeAccDetect(&acc)) {
                accHardware = ACC_FAKE;
                break;
            }
#endif
            ; // fallthrough
        case ACC_NONE: // disable ACC
            accHardware = ACC_NONE;
            break;

    }

    // Found anything? Check if error or ACC is really missing.
    if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        accHardwareToUse = ACC_DEFAULT;
        goto retry;
    }


    if (accHardware == ACC_NONE) {
        return;
    }

    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
}

static void detectBaro(baroSensor_e baroHardwareToUse)
{
#ifndef BARO
    UNUSED(baroHardwareToUse);
#else
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;

#ifdef USE_BARO_BMP085

    const bmp085Config_t *bmp085Config = NULL;

#if defined(BARO_XCLR_GPIO) && defined(BARO_EOC_GPIO)
    static const bmp085Config_t defaultBMP085Config = {
            .gpioAPB2Peripherals = BARO_APB2_PERIPHERALS,
            .xclrGpioPin = BARO_XCLR_PIN,
            .xclrGpioPort = BARO_XCLR_GPIO,
            .eocGpioPin = BARO_EOC_PIN,
            .eocGpioPort = BARO_EOC_GPIO
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
            ; // fallthrough

        case BARO_BMP085: // Always test before MS5611 as some BMP180's can pass MS5611 CRC test
#ifdef USE_BARO_BMP085
            if (bmp085Detect(bmp085Config, &baro)) {
                baroHardware = BARO_BMP085;
                break;
            }
#endif
            ; // fallthrough

        case BARO_MS5611:
#ifdef USE_BARO_MS5611
            if (ms5611Detect(&baro)) {
                baroHardware = BARO_MS5611;
                break;
            }
#endif
            ; // fallthrough

        case BARO_BMP280:
#ifdef USE_BARO_BMP280
            if (bmp280Detect(&baro)) {
                baroHardware = BARO_BMP280;
                break;
            }
#endif
            ; // fallthrough

        case BARO_NONE:
            baroHardware = BARO_NONE;
            break;
    }

    if (baroHardware == BARO_NONE) {
        return;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
#endif
}

#ifdef MAG
static void detectMag(magSensor_e magHardwareToUse)
{
    magSensor_e magHardware;

#ifdef USE_MAG_HMC5883
    const hmc5883Config_t *hmc5883Config = 0;

#ifdef NAZE
    static const hmc5883Config_t nazeHmc5883Config_v1_v4 = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOB,
            .gpioPin = Pin_12,
            .gpioPort = GPIOB,

            /* Disabled for v4 needs more work.
            .intIO = IO_TAG(PB12),
            */
    };
    static const hmc5883Config_t nazeHmc5883Config_v5 = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
            .gpioPin = Pin_14,
            .gpioPort = GPIOC,
            .intIO = IO_TAG(PC14),
    };
    if (hardwareRevision < NAZE32_REV5) {
        hmc5883Config = &nazeHmc5883Config_v1_v4;
    } else {
        hmc5883Config = &nazeHmc5883Config_v5;
    }
#endif

#ifdef SPRACINGF3
    static const hmc5883Config_t spRacingF3Hmc5883Config = {
        .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
        .gpioPin = Pin_14,
        .gpioPort = GPIOC,
        .intIO = IO_TAG(PC14),
    };

    hmc5883Config = &spRacingF3Hmc5883Config;
#endif

#endif

retry:

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

        case MAG_NONE:
            magHardware = MAG_NONE;
            break;
    }

    if (magHardware == MAG_NONE && magHardwareToUse != MAG_DEFAULT && magHardwareToUse != MAG_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        magHardwareToUse = MAG_DEFAULT;
        goto retry;
    }

    if (magHardware == MAG_NONE) {
        return;
    }

    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);
}
#endif

void reconfigureAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    if (sensorAlignmentConfig->gyro_align != ALIGN_DEFAULT) {
        gyroAlign = sensorAlignmentConfig->gyro_align;
    }
    if (sensorAlignmentConfig->acc_align != ALIGN_DEFAULT) {
        accAlign = sensorAlignmentConfig->acc_align;
    }
#ifdef MAG
    if (sensorAlignmentConfig->mag_align != ALIGN_DEFAULT) {
        magAlign = sensorAlignmentConfig->mag_align;
    }
#endif
}

bool sensorsAutodetect(void)
{
    memset(&acc, 0, sizeof(acc));
    memset(&gyro, 0, sizeof(gyro));

#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_MPU6050)

    const extiConfig_t *extiConfig = selectMPUIntExtiConfig();

    mpuDetectionResult_t *mpuDetectionResult = detectMpu(extiConfig);
    UNUSED(mpuDetectionResult);
#endif

    if (!detectGyro()) {
        return false;
    }
    detectAcc(sensorSelectionConfig()->acc_hardware);
    detectBaro(sensorSelectionConfig()->baro_hardware);


    // Now time to init things, acc first
    if (sensors(SENSOR_ACC)) {
        acc.acc_1G = 256; // set default
        acc.init(&acc);
    }
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.init(gyroConfig()->gyro_lpf);

#ifdef MAG
    detectMag(sensorSelectionConfig()->mag_hardware);
#endif

    reconfigureAlignment(sensorAlignmentConfig());

    return true;
}

