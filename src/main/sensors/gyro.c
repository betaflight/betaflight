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
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_adxl345.h"
#include "drivers/accgyro/accgyro_bma280.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/accgyro/accgyro_l3g4200d.h"
#include "drivers/accgyro/accgyro_l3gd20.h"
#include "drivers/accgyro/accgyro_lsm303dlhc.h"
#include "drivers/accgyro/accgyro_mma845x.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyroanalyse.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#if ((FLASH_SIZE > 128) && (defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20689) || defined(USE_GYRO_SPI_MPU6500)))
#define USE_GYRO_SLEW_LIMITER
#endif

FAST_RAM gyro_t gyro;
static FAST_RAM uint8_t gyroDebugMode;

static uint8_t gyroToUse = 0;

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_RAM uint8_t overflowAxisMask;
#endif
static FAST_RAM float accumulatedMeasurements[XYZ_AXIS_COUNT];
static FAST_RAM float gyroPrevious[XYZ_AXIS_COUNT];
static FAST_RAM timeUs_t accumulatedMeasurementTimeUs;
static FAST_RAM timeUs_t accumulationLastTimeSampledUs;

typedef struct gyroCalibration_s {
    int32_t sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    uint16_t calibratingG;
} gyroCalibration_t;

bool firstArmingCalibrationWasStarted = false;

typedef union gyroSoftFilter_u {
    biquadFilter_t gyroFilterLpfState[XYZ_AXIS_COUNT];
    pt1Filter_t gyroFilterPt1State[XYZ_AXIS_COUNT];
    firFilterDenoise_t gyroDenoiseState[XYZ_AXIS_COUNT];
} gyroSoftLpfFilter_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;
    // gyro soft filter
    filterApplyFnPtr softLpfFilterApplyFn;
    gyroSoftLpfFilter_t softLpfFilter;
    filter_t *softLpfFilterPtr[XYZ_AXIS_COUNT];
    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];
    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];
    filterApplyFnPtr notchFilterDynApplyFn;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT];
    timeUs_t overflowTimeUs;
    bool overflowDetected;
#if defined(USE_GYRO_FAST_KALMAN)
    // gyro kalman filter
    filterApplyFnPtr fastKalmanApplyFn;
    fastKalman_t fastKalman[XYZ_AXIS_COUNT];
#elif defined(USE_GYRO_BIQUAD_RC_FIR2)
    // gyro biquad RC FIR2 filter
    filterApplyFnPtr biquadRCFIR2ApplyFn;
    biquadFilter_t biquadRCFIR2[XYZ_AXIS_COUNT];
#endif
} gyroSensor_t;

STATIC_UNIT_TESTED FAST_RAM gyroSensor_t gyroSensor1;
#ifdef USE_DUAL_GYRO
STATIC_UNIT_TESTED FAST_RAM gyroSensor_t gyroSensor2;
#endif

#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyroSensor1.gyroDev;
#endif

#if defined(USE_GYRO_FAST_KALMAN)
static void gyroInitFilterKalman(gyroSensor_t *gyroSensor, uint16_t gyro_filter_q, uint16_t gyro_filter_r, uint16_t gyro_filter_p);
#elif defined (USE_GYRO_BIQUAD_RC_FIR2)
static void gyroInitFilterBiquadRCFIR2(gyroSensor_t *gyroSensor, uint16_t lpfHz);
#endif
static void gyroInitSensorFilters(gyroSensor_t *gyroSensor);

#define DEBUG_GYRO_CALIBRATION 3

#ifdef STM32F10X
#define GYRO_SYNC_DENOM_DEFAULT 8
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) \
   || defined(USE_GYRO_SPI_ICM20689)
#define GYRO_SYNC_DENOM_DEFAULT 1
#else
#define GYRO_SYNC_DENOM_DEFAULT 4
#endif

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 1);

#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#ifdef USE_DUAL_GYRO
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH
#else
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1
#endif
#endif

PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_align = ALIGN_DEFAULT,
    .gyroMovementCalibrationThreshold = 48,
    .gyro_sync_denom = GYRO_SYNC_DENOM_DEFAULT,
    .gyro_lpf = GYRO_LPF_256HZ,
    .gyro_soft_lpf_type = FILTER_PT1,
    .gyro_soft_lpf_hz = 90,
    .gyro_high_fsr = false,
    .gyro_use_32khz = false,
    .gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT,
    .gyro_soft_notch_hz_1 = 400,
    .gyro_soft_notch_cutoff_1 = 300,
    .gyro_soft_notch_hz_2 = 200,
    .gyro_soft_notch_cutoff_2 = 100,
    .checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES,
    .gyro_soft_lpf_hz_2 = 0,
    .gyro_filter_q = 0,
    .gyro_filter_r = 0,
    .gyro_filter_p = 0,
    .gyro_offset_yaw = 0,
);


const busDevice_t *gyroSensorBus(void)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.bus;
    } else {
        return &gyroSensor1.gyroDev.bus;
    }
#else
    return &gyroSensor1.gyroDev.bus;
#endif
}

const mpuConfiguration_t *gyroMpuConfiguration(void)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.mpuConfiguration;
    } else {
        return &gyroSensor1.gyroDev.mpuConfiguration;
    }
#else
    return &gyroSensor1.gyroDev.mpuConfiguration;
#endif
}

const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return &gyroSensor2.gyroDev.mpuDetectionResult;
    } else {
        return &gyroSensor1.gyroDev.mpuDetectionResult;
    }
#else
    return &gyroSensor1.gyroDev.mpuDetectionResult;
#endif
}

STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    switch (gyroHardware) {
    case GYRO_DEFAULT:
        FALLTHROUGH;

#ifdef USE_GYRO_MPU6050
    case GYRO_MPU6050:
        if (mpu6050GyroDetect(dev)) {
            gyroHardware = GYRO_MPU6050;
#ifdef GYRO_MPU6050_ALIGN
            dev->gyroAlign = GYRO_MPU6050_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3G4200D
    case GYRO_L3G4200D:
        if (l3g4200dDetect(dev)) {
            gyroHardware = GYRO_L3G4200D;
#ifdef GYRO_L3G4200D_ALIGN
            dev->gyroAlign = GYRO_L3G4200D_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_MPU3050
    case GYRO_MPU3050:
        if (mpu3050Detect(dev)) {
            gyroHardware = GYRO_MPU3050;
#ifdef GYRO_MPU3050_ALIGN
            dev->gyroAlign = GYRO_MPU3050_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3GD20
    case GYRO_L3GD20:
        if (l3gd20Detect(dev)) {
            gyroHardware = GYRO_L3GD20;
#ifdef GYRO_L3GD20_ALIGN
            dev->gyroAlign = GYRO_L3GD20_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU6000
    case GYRO_MPU6000:
        if (mpu6000SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU6000;
#ifdef GYRO_MPU6000_ALIGN
            dev->gyroAlign = GYRO_MPU6000_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#if defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500)
    case GYRO_MPU6500:
    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
#ifdef USE_GYRO_SPI_MPU6500
        if (mpu6500GyroDetect(dev) || mpu6500SpiGyroDetect(dev)) {
#else
        if (mpu6500GyroDetect(dev)) {
#endif
            switch (dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                gyroHardware = GYRO_MPU9250;
                break;
            case ICM_20601_SPI:
                gyroHardware = GYRO_ICM20601;
                break;
            case ICM_20602_SPI:
                gyroHardware = GYRO_ICM20602;
                break;
            case ICM_20608_SPI:
                gyroHardware = GYRO_ICM20608G;
                break;
            default:
                gyroHardware = GYRO_MPU6500;
            }
#ifdef GYRO_MPU6500_ALIGN
            dev->gyroAlign = GYRO_MPU6500_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU9250
    case GYRO_MPU9250:
        if (mpu9250SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU9250;
#ifdef GYRO_MPU9250_ALIGN
            dev->gyroAlign = GYRO_MPU9250_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20649
    case GYRO_ICM20649:
        if (icm20649SpiGyroDetect(dev)) {
            gyroHardware = GYRO_ICM20649;
#ifdef GYRO_ICM20649_ALIGN
            dev->gyroAlign = GYRO_ICM20649_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20689
    case GYRO_ICM20689:
        if (icm20689SpiGyroDetect(dev)) {
            gyroHardware = GYRO_ICM20689;
#ifdef GYRO_ICM20689_ALIGN
            dev->gyroAlign = GYRO_ICM20689_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
    case GYRO_BMI160:
        if (bmi160SpiGyroDetect(dev)) {
            gyroHardware = GYRO_BMI160;
#ifdef GYRO_BMI160_ALIGN
            dev->gyroAlign = GYRO_BMI160_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_FAKE_GYRO
    case GYRO_FAKE:
        if (fakeGyroDetect(dev)) {
            gyroHardware = GYRO_FAKE;
            break;
        }
        FALLTHROUGH;
#endif

    default:
        gyroHardware = GYRO_NONE;
    }

    if (gyroHardware != GYRO_NONE) {
        detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
        sensorsSet(SENSOR_GYRO);
    }


    return gyroHardware;
}

static bool gyroInitSensor(gyroSensor_t *gyroSensor)
{
    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;

#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) \
 || defined(USE_ACC_MPU6050) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) || defined(USE_GYRO_SPI_ICM20689)

    mpuDetect(&gyroSensor->gyroDev);
    mpuResetFn = gyroSensor->gyroDev.mpuConfiguration.resetFn; // must be set after mpuDetect
#endif

    const gyroSensor_e gyroHardware = gyroDetect(&gyroSensor->gyroDev);
    if (gyroHardware == GYRO_NONE) {
        return false;
    }

    switch (gyroHardware) {
    case GYRO_MPU6500:
    case GYRO_MPU9250:
    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
    case GYRO_ICM20689:
        // do nothing, as gyro supports 32kHz
        break;
    default:
        // gyro does not support 32kHz
        gyroConfigMutable()->gyro_use_32khz = false;
        break;
    }

    // Must set gyro targetLooptime before gyroDev.init and initialisation of filters
    gyro.targetLooptime = gyroSetSampleRate(&gyroSensor->gyroDev, gyroConfig()->gyro_lpf, gyroConfig()->gyro_sync_denom, gyroConfig()->gyro_use_32khz);
    gyroSensor->gyroDev.lpf = gyroConfig()->gyro_lpf;
    gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);
    if (gyroConfig()->gyro_align != ALIGN_DEFAULT) {
        gyroSensor->gyroDev.gyroAlign = gyroConfig()->gyro_align;
    }

    gyroInitSensorFilters(gyroSensor);

#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyseInit(gyro.targetLooptime);
#endif
    return true;
}

bool gyroInit(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_YAW) {
        overflowAxisMask = GYRO_OVERFLOW_Z;
    } else if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_ALL_AXES) {
        overflowAxisMask = GYRO_OVERFLOW_X | GYRO_OVERFLOW_Y | GYRO_OVERFLOW_Z;
    } else {
        overflowAxisMask = 0;
    }
#endif

    switch (debugMode) {
    case DEBUG_FFT:
    case DEBUG_GYRO_NOTCH:
    case DEBUG_GYRO:
    case DEBUG_GYRO_RAW:
        gyroDebugMode = debugMode;
        break;
    default:
        // debugMode is not gyro-related
        gyroDebugMode = DEBUG_NONE;
        break;
    }
    firstArmingCalibrationWasStarted = false;

    bool ret = false;
    memset(&gyro, 0, sizeof(gyro));
    gyroToUse = gyroConfig()->gyro_to_use;

#if defined(USE_DUAL_GYRO) && defined(GYRO_1_CS_PIN)
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(GYRO_1_CS_PIN));
        IOInit(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
        IOHi(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.
        IOConfigGPIO(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    }
#endif

#if defined(USE_DUAL_GYRO) && defined(GYRO_2_CS_PIN)
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(GYRO_2_CS_PIN));
        IOInit(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin, OWNER_MPU_CS, 1);
        IOHi(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.
        IOConfigGPIO(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    }
#endif

    gyroSensor1.gyroDev.gyroAlign = ALIGN_DEFAULT;

#if defined(GYRO_1_EXTI_PIN)
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG(GYRO_1_EXTI_PIN);
#elif defined(MPU_INT_EXTI)
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG(MPU_INT_EXTI);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor1.gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // GYRO_1_EXTI_PIN
#ifdef USE_DUAL_GYRO
#ifdef GYRO_1_ALIGN
    gyroSensor1.gyroDev.gyroAlign = GYRO_1_ALIGN;
#endif
    gyroSensor1.gyroDev.bus.bustype = BUSTYPE_SPI;
    spiBusSetInstance(&gyroSensor1.gyroDev.bus, GYRO_1_SPI_INSTANCE);
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        ret = gyroInitSensor(&gyroSensor1);
        if (!ret) {
            return false; // TODO handle failure of first gyro detection better. - Perhaps update the config to use second gyro then indicate a new failure mode and reboot.
        }
    }
#else
    ret = gyroInitSensor(&gyroSensor1);
#endif

#ifdef USE_DUAL_GYRO

    gyroSensor2.gyroDev.gyroAlign = ALIGN_DEFAULT;

#if defined(GYRO_2_EXTI_PIN)
    gyroSensor2.gyroDev.mpuIntExtiTag =  IO_TAG(GYRO_2_EXTI_PIN);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor2.gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor2.gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // GYRO_2_EXTI_PIN
#ifdef GYRO_2_ALIGN
    gyroSensor2.gyroDev.gyroAlign = GYRO_2_ALIGN;
#endif
    gyroSensor2.gyroDev.bus.bustype = BUSTYPE_SPI;
    spiBusSetInstance(&gyroSensor2.gyroDev.bus, GYRO_2_SPI_INSTANCE);
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        ret = gyroInitSensor(&gyroSensor2);
        if (!ret) {
            return false; // TODO handle failure of second gyro detection better. - Perhaps update the config to use first gyro then indicate a new failure mode and reboot.
        }
    }
#endif
    return ret;
}

void gyroInitFilterLpf(gyroSensor_t *gyroSensor, uint8_t lpfHz)
{
    gyroSensor->softLpfFilterApplyFn = nullFilterApply;
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;

    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {  // Initialisation needs to happen once samplingrate is known
        switch (gyroConfig()->gyro_soft_lpf_type) {
        case FILTER_BIQUAD:
            gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                gyroSensor->softLpfFilterPtr[axis] = (filter_t *)&gyroSensor->softLpfFilter.gyroFilterLpfState[axis];
                biquadFilterInitLPF(&gyroSensor->softLpfFilter.gyroFilterLpfState[axis], lpfHz, gyro.targetLooptime);
            }
            break;
        case FILTER_PT1:
            gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
            const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                gyroSensor->softLpfFilterPtr[axis] = (filter_t *)&gyroSensor->softLpfFilter.gyroFilterPt1State[axis];
                pt1FilterInit(&gyroSensor->softLpfFilter.gyroFilterPt1State[axis], lpfHz, gyroDt);
            }
            break;
        default:
            gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                gyroSensor->softLpfFilterPtr[axis] = (filter_t *)&gyroSensor->softLpfFilter.gyroDenoiseState[axis];
                firFilterDenoiseInit(&gyroSensor->softLpfFilter.gyroDenoiseState[axis], lpfHz, gyro.targetLooptime);
            }
            break;
        }
    }
}

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}

#if defined(USE_GYRO_SLEW_LIMITER)
void gyroInitSlewLimiter(gyroSensor_t *gyroSensor) {

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = 0;
    }
}
#endif

static void gyroInitFilterNotch1(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

static void gyroInitFilterNotch2(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

#ifdef USE_GYRO_DATA_ANALYSE
static bool isDynamicFilterActive(void)
{
    return feature(FEATURE_DYNAMIC_FILTER);
}

static void gyroInitFilterDynamicNotch(gyroSensor_t *gyroSensor)
{
    gyroSensor->notchFilterDynApplyFn = nullFilterApply;

    if (isDynamicFilterActive()) {
        gyroSensor->notchFilterDynApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1; // must be this function, not DF2
        const float notchQ = filterGetNotchQ(400, 390); //just any init value
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilterDyn[axis], 400, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}
#endif

#if defined(USE_GYRO_FAST_KALMAN)
static void gyroInitFilterKalman(gyroSensor_t *gyroSensor, uint16_t gyro_filter_q, uint16_t gyro_filter_r, uint16_t gyro_filter_p)
{
    gyroSensor->fastKalmanApplyFn = nullFilterApply;

    // If Kalman Filter noise covariances for Process and Measurement are non-zero, we treat as enabled
    if (gyro_filter_q != 0 && gyro_filter_r != 0) {
        gyroSensor->fastKalmanApplyFn = (filterApplyFnPtr)fastKalmanUpdate;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            fastKalmanInit(&gyroSensor->fastKalman[axis], gyro_filter_q, gyro_filter_r, gyro_filter_p);
        }
    }
}
#elif defined(USE_GYRO_BIQUAD_RC_FIR2)
static void gyroInitFilterBiquadRCFIR2(gyroSensor_t *gyroSensor, uint16_t lpfHz)
{
    gyroSensor->biquadRCFIR2ApplyFn = nullFilterApply;
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {  // Initialisation needs to happen once samplingrate is known
        gyroSensor->biquadRCFIR2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadRCFIR2FilterInit(&gyroSensor->biquadRCFIR2[axis], lpfHz, gyroDt);
        }
    }
}
#endif

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
#if defined(USE_GYRO_SLEW_LIMITER)
    gyroInitSlewLimiter(gyroSensor);
#endif
#if defined(USE_GYRO_FAST_KALMAN)
    gyroInitFilterKalman(gyroSensor, gyroConfig()->gyro_filter_q, gyroConfig()->gyro_filter_r, gyroConfig()->gyro_filter_p);
#elif defined(USE_GYRO_BIQUAD_RC_FIR2)
    gyroInitFilterBiquadRCFIR2(gyroSensor, gyroConfig()->gyro_soft_lpf_hz_2);
#endif
    gyroInitFilterLpf(gyroSensor, gyroConfig()->gyro_soft_lpf_hz);
    gyroInitFilterNotch1(gyroSensor, gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroSensor, gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
#ifdef USE_GYRO_DATA_ANALYSE
    gyroInitFilterDynamicNotch(gyroSensor);
#endif
}

void gyroInitFilters(void)
{
    gyroInitSensorFilters(&gyroSensor1);
#ifdef USE_DUAL_GYRO
    gyroInitSensorFilters(&gyroSensor2);
#endif
}

FAST_CODE bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.calibratingG == 0;
}

FAST_CODE bool isGyroCalibrationComplete(void)
{
#ifdef USE_DUAL_GYRO
    switch (gyroToUse) {
        default:
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyroSensor1);
        }
        case GYRO_CONFIG_USE_GYRO_2: {
            return isGyroSensorCalibrationComplete(&gyroSensor2);
        }
        case GYRO_CONFIG_USE_GYRO_BOTH: {
            return isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2);
        }
    }
#else
    return isGyroSensorCalibrationComplete(&gyroSensor1);
#endif
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->calibratingG == 1;
}

static uint16_t gyroCalculateCalibratingCycles(void)
{
    return (CALIBRATING_GYRO_TIME_US / gyro.targetLooptime);
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->calibratingG == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
    gyroSensor->calibration.calibratingG = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (!(isFirstArmingCalibration && firstArmingCalibrationWasStarted)) {
        gyroSetCalibrationCycles(&gyroSensor1);
#ifdef USE_DUAL_GYRO
        gyroSetCalibrationCycles(&gyroSensor2);
#endif

        if (isFirstArmingCalibration) {
            firstArmingCalibrationWasStarted = true;
        }
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !isGyroCalibrationComplete();
}

STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);

            DEBUG_SET(DEBUG_GYRO, DEBUG_GYRO_CALIBRATION, lrintf(stddev));

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }
    --gyroSensor->calibration.calibratingG;

}

#if defined(USE_GYRO_SLEW_LIMITER)
FAST_CODE int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis)
{
    int32_t ret = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
    if (gyroConfig()->checkOverflow) {
        // don't use the slew limiter if overflow checking is on
        return ret;
    }
    if (abs(ret - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1<<14)) {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = ret;
    }
    return ret;
}
#endif

static void checkForOverflow(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (gyroSensor->overflowDetected) {
        const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * gyroSensor->gyroDev.scale;
        if ((abs(gyro.gyroADCf[X]) < gyroOverflowResetRate)
              && (abs(gyro.gyroADCf[Y]) < gyroOverflowResetRate)
              && (abs(gyro.gyroADCf[Z]) < gyroOverflowResetRate)) {
            // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
            // reset requires good OK values on all axes
            if (cmpTimeUs(currentTimeUs, gyroSensor->overflowTimeUs) > 50000) {
                gyroSensor->overflowDetected = false;
            }
        } else {
            // not a consecutive OK value, so reset the overflow time
            gyroSensor->overflowTimeUs = currentTimeUs;
        }
    } else {
#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * gyroSensor->gyroDev.scale;
        if (abs(gyro.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (abs(gyro.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (abs(gyro.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & overflowAxisMask) {
            gyroSensor->overflowDetected = true;
            gyroSensor->overflowTimeUs = currentTimeUs;
        }
#endif // SIMULATOR_BUILD
    }
#else
    UNUSED(gyroSensor);
    UNUSED(currentTimeUs);
#endif // USE_GYRO_OVERFLOW_CHECK
}

static FAST_CODE void gyroUpdateSensor(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[X] = gyroSlewLimiter(gyroSensor, X) - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSlewLimiter(gyroSensor, Y) - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        alignSensors(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
        // still calibrating, so no need to further process gyro data
        return;
    }

#ifdef USE_GYRO_DATA_ANALYSE
    if (isDynamicFilterActive()) {
        gyroDataAnalyse(&gyroSensor->gyroDev, gyroSensor->notchFilterDyn);
    }
#endif

    const timeDelta_t sampleDeltaUs = currentTimeUs - accumulationLastTimeSampledUs;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs += sampleDeltaUs;

    if (gyroConfig()->checkOverflow) {
        checkForOverflow(gyroSensor, currentTimeUs);
    }
    if (gyroDebugMode == DEBUG_NONE) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // NOTE: this branch optimized for when there is no gyro debugging, ensure it is kept in step with non-optimized branch
            float gyroADCf = gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;
#if defined(USE_GYRO_FAST_KALMAN)
            gyroADCf = gyroSensor->fastKalmanApplyFn((filter_t *)&gyroSensor->fastKalman[axis], gyroADCf);
#elif defined(USE_GYRO_BIQUAD_RC_FIR2)
            gyroADCf = gyroSensor->biquadRCFIR2ApplyFn((filter_t *)&gyroSensor->biquadRCFIR2[axis], gyroADCf);
#endif
#ifdef USE_GYRO_DATA_ANALYSE
            gyroADCf = gyroSensor->notchFilterDynApplyFn((filter_t *)&gyroSensor->notchFilterDyn[axis], gyroADCf);
#endif
            gyroADCf = gyroSensor->notchFilter1ApplyFn((filter_t *)&gyroSensor->notchFilter1[axis], gyroADCf);
            gyroADCf = gyroSensor->notchFilter2ApplyFn((filter_t *)&gyroSensor->notchFilter2[axis], gyroADCf);
            gyroADCf = gyroSensor->softLpfFilterApplyFn(gyroSensor->softLpfFilterPtr[axis], gyroADCf);
            gyroSensor->gyroDev.gyroADCf[axis] = gyroADCf;
            if (!gyroSensor->overflowDetected) {
                // integrate using trapezium rule to avoid bias
                accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyroADCf) * sampleDeltaUs;
                gyroPrevious[axis] = gyroADCf;
            }
        }
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            DEBUG_SET(DEBUG_GYRO_RAW, axis, gyroSensor->gyroDev.gyroADCRaw[axis]);
            // scale gyro output to degrees per second
            float gyroADCf = gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;
            // DEBUG_GYRO_NOTCH records the unfiltered gyro output
            DEBUG_SET(DEBUG_GYRO_NOTCH, axis, lrintf(gyroADCf));

#if defined(USE_GYRO_FAST_KALMAN)
            // apply fast kalman
            gyroADCf = gyroSensor->fastKalmanApplyFn((filter_t *)&gyroSensor->fastKalman[axis], gyroADCf);
#elif defined(USE_GYRO_BIQUAD_RC_FIR2)
            // apply biquad RC+FIR2
            gyroADCf = gyroSensor->biquadRCFIR2ApplyFn((filter_t *)&gyroSensor->biquadRCFIR2[axis], gyroADCf);
#endif

#ifdef USE_GYRO_DATA_ANALYSE
            // apply dynamic notch filter
            if (isDynamicFilterActive()) {
                if (axis == 0) {
                    DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf)); // store raw data
                }
                gyroADCf = gyroSensor->notchFilterDynApplyFn((filter_t *)&gyroSensor->notchFilterDyn[axis], gyroADCf);
                if (axis == 0) {
                    DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf)); // store data after dynamic notch
                }
            }
#endif

            // apply static notch filters
            gyroADCf = gyroSensor->notchFilter1ApplyFn((filter_t *)&gyroSensor->notchFilter1[axis], gyroADCf);
            gyroADCf = gyroSensor->notchFilter2ApplyFn((filter_t *)&gyroSensor->notchFilter2[axis], gyroADCf);

            // apply LPF
            DEBUG_SET(DEBUG_GYRO, axis, lrintf(gyroADCf));
            gyroADCf = gyroSensor->softLpfFilterApplyFn(gyroSensor->softLpfFilterPtr[axis], gyroADCf);

            gyroSensor->gyroDev.gyroADCf[axis] = gyroADCf;
            if (!gyroSensor->overflowDetected) {
                // integrate using trapezium rule to avoid bias
                accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyroADCf) * sampleDeltaUs;
                gyroPrevious[axis] = gyroADCf;
            }
        }
    }
}

FAST_CODE void gyroUpdate(timeUs_t currentTimeUs)
{
#ifdef USE_DUAL_GYRO
    switch (gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1)) {
            gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 0, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[Y]));
        break;
    case GYRO_CONFIG_USE_GYRO_2:
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = gyroSensor2.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor2.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor2.gyroDev.gyroADCf[Z];
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 3, lrintf(gyro.gyroADCf[Y]));
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = (gyroSensor1.gyroDev.gyroADCf[X] + gyroSensor2.gyroDev.gyroADCf[X]) / 2.0f;
            gyro.gyroADCf[Y] = (gyroSensor1.gyroDev.gyroADCf[Y] + gyroSensor2.gyroDev.gyroADCf[Y]) / 2.0f;
            gyro.gyroADCf[Z] = (gyroSensor1.gyroDev.gyroADCf[Z] + gyroSensor2.gyroDev.gyroADCf[Z]) / 2.0f;
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X] - gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y] - gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf(gyroSensor1.gyroDev.gyroADCf[Z] - gyroSensor2.gyroDev.gyroADCf[Z]));
        break;
    }
#else
    gyroUpdateSensor(&gyroSensor1, currentTimeUs);
    gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
    gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
    gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];
#endif
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementTimeUs > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementTimeUs = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

void gyroReadTemperature(void)
{
    if (gyroSensor1.gyroDev.temperatureFn) {
        gyroSensor1.gyroDev.temperatureFn(&gyroSensor1.gyroDev, &gyroSensor1.gyroDev.temperature);
    }
}

int16_t gyroGetTemperature(void)
{
    return gyroSensor1.gyroDev.temperature;
}

int16_t gyroRateDps(int axis)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return lrintf(gyro.gyroADCf[axis] / gyroSensor2.gyroDev.scale);
    } else {
        return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
    }
#else
    return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
#endif
}

bool gyroOverflowDetected(void)
{
    return gyroSensor1.overflowDetected;
}

uint16_t gyroAbsRateDps(int axis)
{
    return fabsf(gyro.gyroADCf[axis]);
}
