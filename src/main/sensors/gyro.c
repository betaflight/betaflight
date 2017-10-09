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

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

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
#include "drivers/bus_spi.h"
#include "drivers/gyro_sync.h"
#include "drivers/io.h"

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

#if ((FLASH_SIZE > 128) && (defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20689)))
#define USE_GYRO_SLEW_LIMITER
#endif

gyro_t gyro;
static uint8_t gyroDebugMode;


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
    void *softLpfFilterPtr[XYZ_AXIS_COUNT];
    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];
    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];
    filterApplyFnPtr notchFilterDynApplyFn;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT];
} gyroSensor_t;

static gyroSensor_t gyroSensor1;

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

PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 1);

PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_align = ALIGN_DEFAULT,
    .gyroMovementCalibrationThreshold = 48,
    .gyro_sync_denom = GYRO_SYNC_DENOM_DEFAULT,
    .gyro_lpf = GYRO_LPF_256HZ,
    .gyro_soft_lpf_type = FILTER_PT1,
    .gyro_soft_lpf_hz = 90,
    .gyro_high_fsr = false,
    .gyro_use_32khz = false,
    .gyro_to_use = 0,
    .gyro_soft_notch_hz_1 = 400,
    .gyro_soft_notch_cutoff_1 = 300,
    .gyro_soft_notch_hz_2 = 200,
    .gyro_soft_notch_cutoff_2 = 100
);


const busDevice_t *gyroSensorBus(void)
{
    return &gyroSensor1.gyroDev.bus;
}

const mpuConfiguration_t *gyroMpuConfiguration(void)
{
    return &gyroSensor1.gyroDev.mpuConfiguration;
}

const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
    return &gyroSensor1.gyroDev.mpuDetectionResult;
}

STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    dev->gyroAlign = ALIGN_DEFAULT;

    switch (gyroHardware) {
    case GYRO_DEFAULT:
#ifdef USE_GYRO_MPU6050
    case GYRO_MPU6050:
        if (mpu6050GyroDetect(dev)) {
            gyroHardware = GYRO_MPU6050;
#ifdef GYRO_MPU6050_ALIGN
            dev->gyroAlign = GYRO_MPU6050_ALIGN;
#endif
            break;
        }
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
#endif

#ifdef USE_GYRO_SPI_MPU9250
    case GYRO_MPU9250:

        if (mpu9250SpiGyroDetect(dev))
        {
            gyroHardware = GYRO_MPU9250;
#ifdef GYRO_MPU9250_ALIGN
            dev->gyroAlign = GYRO_MPU9250_ALIGN;
#endif
        break;
    }
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
#endif

#ifdef USE_FAKE_GYRO
    case GYRO_FAKE:
        if (fakeGyroDetect(dev)) {
            gyroHardware = GYRO_FAKE;
            break;
        }
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
#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) \
 || defined(USE_ACC_MPU6050) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) || defined(USE_GYRO_SPI_ICM20689)

#if defined(MPU_INT_EXTI)
    gyroSensor->gyroDev.mpuIntExtiTag =  IO_TAG(MPU_INT_EXTI);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor->gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor->gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // MPU_INT_EXTI

#ifdef USE_DUAL_GYRO
    // set cnsPin using GYRO_n_CS_PIN defined in target.h
    gyroSensor->gyroDev.bus.busdev_u.spi.csnPin = gyroConfig()->gyro_to_use == 0 ? IOGetByTag(IO_TAG(GYRO_0_CS_PIN)) : IOGetByTag(IO_TAG(GYRO_1_CS_PIN));
#else
    gyroSensor->gyroDev.bus.busdev_u.spi.csnPin = IO_NONE; // set cnsPin to IO_NONE so mpuDetect will set it according to value defined in target.h
#endif // USE_DUAL_GYRO
    mpuDetect(&gyroSensor->gyroDev);
    mpuResetFn = gyroSensor->gyroDev.mpuConfiguration.resetFn; // must be set after mpuDetect
#endif
    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;

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
    memset(&gyro, 0, sizeof(gyro));
    return gyroInitSensor(&gyroSensor1);
}

void gyroInitFilterLpf(gyroSensor_t *gyroSensor, uint8_t lpfHz)
{
    gyroSensor->softLpfFilterApplyFn = nullFilterApply;
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;

    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {  // Initialisation needs to happen once samplingrate is known
        switch (gyroConfig()->gyro_soft_lpf_type) {
        case FILTER_BIQUAD:
            gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = 0; axis < 3; axis++) {
                gyroSensor->softLpfFilterPtr[axis] = &gyroSensor->softLpfFilter.gyroFilterLpfState[axis];
                biquadFilterInitLPF(&gyroSensor->softLpfFilter.gyroFilterLpfState[axis], lpfHz, gyro.targetLooptime);
            }
            break;
        case FILTER_PT1:
            gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
            const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
            for (int axis = 0; axis < 3; axis++) {
                gyroSensor->softLpfFilterPtr[axis] = &gyroSensor->softLpfFilter.gyroFilterPt1State[axis];
                pt1FilterInit(&gyroSensor->softLpfFilter.gyroFilterPt1State[axis], lpfHz, gyroDt);
            }
            break;
        default:
            gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = 0; axis < 3; axis++) {
                gyroSensor->softLpfFilterPtr[axis] = &gyroSensor->softLpfFilter.gyroDenoiseState[axis];
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
        for (int axis = 0; axis < 3; axis++) {
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
        for (int axis = 0; axis < 3; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

#ifdef USE_GYRO_DATA_ANALYSE
static void gyroInitFilterDynamicNotch(gyroSensor_t *gyroSensor)
{
    gyroSensor->notchFilterDynApplyFn = nullFilterApply;

    if (isDynamicFilterActive()) {
        gyroSensor->notchFilterDynApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1; // must be this function, not DF2
        const float notchQ = filterGetNotchQ(400, 390); //just any init value
        for (int axis = 0; axis < 3; axis++) {
            biquadFilterInit(&gyroSensor->notchFilterDyn[axis], 400, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}
#endif

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
#if defined(USE_GYRO_SLEW_LIMITER)
    gyroInitSlewLimiter(gyroSensor);
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
}

bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.calibratingG == 0;
}

bool isGyroCalibrationComplete(void)
{
    return isGyroSensorCalibrationComplete(&gyroSensor1);
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->calibratingG == 1;
}

static uint16_t gyroCalculateCalibratingCycles(void)
{
    return (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime) * CALIBRATING_GYRO_CYCLES;
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

        // Sum up CALIBRATING_GYRO_CYCLES readings
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
            gyroSensor->gyroDev.gyroZero[axis] = (gyroSensor->calibration.sum[axis] + (gyroCalculateCalibratingCycles() / 2)) / gyroCalculateCalibratingCycles();
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
int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis)
{
    int32_t newRawGyro = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
    if (abs(newRawGyro - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1<<14)) {
        newRawGyro = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = newRawGyro;
    }
    return newRawGyro;
}
#endif

void gyroUpdateSensor(gyroSensor_t *gyroSensor)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        alignSensors(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
        // Reset gyro values to zero to prevent other code from using uncalibrated data
        gyro.gyroADCf[X] = 0.0f;
        gyro.gyroADCf[Y] = 0.0f;
        gyro.gyroADCf[Z] = 0.0f;
        // still calibrating, so no need to further process gyro data
        return;
    }

#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyse(&gyroSensor->gyroDev, gyroSensor->notchFilterDyn);
#endif

    if (gyroDebugMode == DEBUG_NONE) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // NOTE: this branch optimized for when there is no gyro debugging, ensure it is kept in step with non-optimized branch
            float gyroADCf = (float)gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;
#ifdef USE_GYRO_DATA_ANALYSE
            gyroADCf = gyroSensor->notchFilterDynApplyFn(&gyroSensor->notchFilterDyn[axis], gyroADCf);
#endif
            gyroADCf = gyroSensor->notchFilter1ApplyFn(&gyroSensor->notchFilter1[axis], gyroADCf);
            gyroADCf = gyroSensor->notchFilter2ApplyFn(&gyroSensor->notchFilter2[axis], gyroADCf);
            gyroADCf = gyroSensor->softLpfFilterApplyFn(gyroSensor->softLpfFilterPtr[axis], gyroADCf);
            gyro.gyroADCf[axis] = gyroADCf;
        }
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            DEBUG_SET(DEBUG_GYRO_RAW, axis, gyroSensor->gyroDev.gyroADCRaw[axis]);
            // scale gyro output to degrees per second
            float gyroADCf = (float)gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;
            // DEBUG_GYRO_NOTCH records the unfiltered gyro output
            DEBUG_SET(DEBUG_GYRO_NOTCH, axis, lrintf(gyroADCf));

#ifdef USE_GYRO_DATA_ANALYSE
            // Apply Dynamic Notch filtering
            if (isDynamicFilterActive()) {
                if (axis == 0) {
                    DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf)); // store raw data
                }
                gyroADCf = gyroSensor->notchFilterDynApplyFn(&gyroSensor->notchFilterDyn[axis], gyroADCf);
                if (axis == 0) {
                    DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf)); // store data after dynamic notch
                }
            }
#endif

            // Apply Static Notch filtering
            gyroADCf = gyroSensor->notchFilter1ApplyFn(&gyroSensor->notchFilter1[axis], gyroADCf);
            gyroADCf = gyroSensor->notchFilter2ApplyFn(&gyroSensor->notchFilter2[axis], gyroADCf);

            // Apply LPF
            DEBUG_SET(DEBUG_GYRO, axis, lrintf(gyroADCf));
            gyroADCf = gyroSensor->softLpfFilterApplyFn(gyroSensor->softLpfFilterPtr[axis], gyroADCf);

            gyro.gyroADCf[axis] = gyroADCf;
        }
    }
}

void gyroUpdate(void)
{
    gyroUpdateSensor(&gyroSensor1);
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
    return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
}
