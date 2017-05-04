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
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/bus_spi.h"
#include "drivers/gyro_sync.h"
#include "drivers/io.h"
#include "drivers/system.h"

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

gyro_t gyro;

STATIC_UNIT_TESTED gyroDev_t gyroDev0;
static int16_t gyroTemperature0;

typedef struct gyroCalibration_s {
    int32_t g[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    uint16_t calibratingG;
} gyroCalibration_t;

STATIC_UNIT_TESTED gyroCalibration_t gyroCalibration;

static filterApplyFnPtr softLpfFilterApplyFn;
static void *softLpfFilter[3];
static filterApplyFnPtr notchFilter1ApplyFn;
static void *notchFilter1[3];
static filterApplyFnPtr notchFilter2ApplyFn;
static void *notchFilter2[3];

#define DEBUG_GYRO_CALIBRATION 3

#ifdef STM32F10X
#define GYRO_SYNC_DENOM_DEFAULT 8
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_ICM20601)  || defined(USE_GYRO_SPI_ICM20689)
#define GYRO_SYNC_DENOM_DEFAULT 1
#else
#define GYRO_SYNC_DENOM_DEFAULT 4
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_align = ALIGN_DEFAULT,
    .gyroMovementCalibrationThreshold = 48,
    .gyro_sync_denom = GYRO_SYNC_DENOM_DEFAULT,
    .gyro_lpf = GYRO_LPF_256HZ,
    .gyro_soft_lpf_type = FILTER_PT1,
    .gyro_soft_lpf_hz = 90,
    .gyro_isr_update = false,
    .gyro_use_32khz = false,
    .gyro_to_use = 0,
    .gyro_soft_notch_hz_1 = 400,
    .gyro_soft_notch_cutoff_1 = 300,
    .gyro_soft_notch_hz_2 = 200,
    .gyro_soft_notch_cutoff_2 = 100
);


const busDevice_t *gyroSensorBus(void)
{
    return &gyroDev0.bus;
}

const mpuConfiguration_t *gyroMpuConfiguration(void)
{
    return &gyroDev0.mpuConfiguration;
}

const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
    return &gyroDev0.mpuDetectionResult;
}

STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    dev->gyroAlign = ALIGN_DEFAULT;

    switch(gyroHardware) {
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
            switch(dev->mpuDetectionResult.sensor) {
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

bool gyroInit(void)
{
    memset(&gyro, 0, sizeof(gyro));
#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_MPU6050) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20689)

#if defined(MPU_INT_EXTI)
    gyroDev0.mpuIntExtiTag =  IO_TAG(MPU_INT_EXTI);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroDev0.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroDev0.mpuIntExtiTag =  IO_TAG_NONE;
#endif

#ifdef USE_DUAL_GYRO
    // set cnsPin using GYRO_n_CS_PIN defined in target.h
    gyroDev0.bus.spi.csnPin = gyroConfig()->gyro_to_use == 0 ? IOGetByTag(IO_TAG(GYRO_0_CS_PIN)) : IOGetByTag(IO_TAG(GYRO_1_CS_PIN));
#else
    gyroDev0.bus.spi.csnPin = IO_NONE; // set cnsPin to IO_NONE so mpuDetect will set it according to value defined in target.h
#endif // USE_DUAL_GYRO
    mpuDetect(&gyroDev0);
    mpuResetFn = gyroDev0.mpuConfiguration.resetFn; // must be set after mpuDetect
#endif
    const gyroSensor_e gyroHardware = gyroDetect(&gyroDev0);
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
    gyro.targetLooptime = gyroSetSampleRate(&gyroDev0, gyroConfig()->gyro_lpf, gyroConfig()->gyro_sync_denom, gyroConfig()->gyro_use_32khz);
    gyroDev0.lpf = gyroConfig()->gyro_lpf;
    gyroDev0.init(&gyroDev0);
    if (gyroConfig()->gyro_align != ALIGN_DEFAULT) {
        gyroDev0.gyroAlign = gyroConfig()->gyro_align;
    }
    gyroInitFilters();
#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyseInit(gyro.targetLooptime);
#endif
    return true;
}

void gyroInitFilterLpf(uint8_t lpfHz)
{
    static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];
    static pt1Filter_t gyroFilterPt1[XYZ_AXIS_COUNT];
    static firFilterDenoise_t gyroDenoiseState[XYZ_AXIS_COUNT];

    softLpfFilterApplyFn = nullFilterApply;
    const uint32_t gyroFrequencyNyquist = (1.0f / (gyro.targetLooptime * 0.000001f)) / 2; // No rounding needed

    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {  // Initialisation needs to happen once samplingrate is known
        switch (gyroConfig()->gyro_soft_lpf_type) {
        case FILTER_BIQUAD:
            softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = 0; axis < 3; axis++) {
                softLpfFilter[axis] = &gyroFilterLPF[axis];
                biquadFilterInitLPF(softLpfFilter[axis], lpfHz, gyro.targetLooptime);
            }
            break;
        case FILTER_PT1:
            softLpfFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
            const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
            for (int axis = 0; axis < 3; axis++) {
                softLpfFilter[axis] = &gyroFilterPt1[axis];
                pt1FilterInit(softLpfFilter[axis], lpfHz, gyroDt);
            }
            break;
        default:
            softLpfFilterApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = 0; axis < 3; axis++) {
                softLpfFilter[axis] = &gyroDenoiseState[axis];
                firFilterDenoiseInit(softLpfFilter[axis], lpfHz, gyro.targetLooptime);
            }
            break;
        }
    }
}

void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz)
{
    static biquadFilter_t gyroFilterNotch[XYZ_AXIS_COUNT];

    notchFilter1ApplyFn = nullFilterApply;
    const uint32_t gyroFrequencyNyquist = (1.0f / (gyro.targetLooptime * 0.000001f)) / 2; // No rounding needed
    if (notchHz && notchHz <= gyroFrequencyNyquist) {
        notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < 3; axis++) {
            notchFilter1[axis] = &gyroFilterNotch[axis];
            biquadFilterInit(notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz)
{
    static biquadFilter_t gyroFilterNotch[XYZ_AXIS_COUNT];

    notchFilter2ApplyFn = nullFilterApply;
    const uint32_t gyroFrequencyNyquist = (1.0f / (gyro.targetLooptime * 0.000001f)) / 2; // No rounding needed
    if (notchHz && notchHz <= gyroFrequencyNyquist) {
        notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < 3; axis++) {
            notchFilter2[axis] = &gyroFilterNotch[axis];
            biquadFilterInit(notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

void gyroInitFilters(void)
{
    gyroInitFilterLpf(gyroConfig()->gyro_soft_lpf_hz);
    gyroInitFilterNotch1(gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
}

bool isGyroCalibrationComplete(void)
{
    return gyroCalibration.calibratingG == 0;
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

void gyroSetCalibrationCycles(void)
{
    gyroCalibration.calibratingG = gyroCalculateCalibratingCycles();
}

STATIC_UNIT_TESTED void performGyroCalibration(gyroDev_t *gyroDev, gyroCalibration_t *gyroCalibration, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < 3; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(gyroCalibration)) {
            gyroCalibration->g[axis] = 0;
            devClear(&gyroCalibration->var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroDev->gyroZero[axis] = 0;
        }

        // Sum up CALIBRATING_GYRO_CYCLES readings
        gyroCalibration->g[axis] += gyroDev->gyroADCRaw[axis];
        devPush(&gyroCalibration->var[axis], gyroDev->gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(gyroCalibration)) {
            const float stddev = devStandardDeviation(&gyroCalibration->var[axis]);

            DEBUG_SET(DEBUG_GYRO, DEBUG_GYRO_CALIBRATION, lrintf(stddev));

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles();
                return;
            }
            gyroDev->gyroZero[axis] = (gyroCalibration->g[axis] + (gyroCalculateCalibratingCycles() / 2)) / gyroCalculateCalibratingCycles();
        }
    }

    if (isOnFinalGyroCalibrationCycle(gyroCalibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        beeper(BEEPER_GYRO_CALIBRATED);
    }
    gyroCalibration->calibratingG--;

}

#if defined(GYRO_USES_SPI) && defined(USE_MPU_DATA_READY_SIGNAL)
static bool gyroUpdateISR(gyroDev_t* gyroDev)
{
    if (!gyroDev->dataReady || !gyroDev->read(gyroDev)) {
        return false;
    }
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    debug[2] = (uint16_t)(micros() & 0xffff);
#endif
    gyroDev->dataReady = false;
    // move gyro data into 32-bit variables to avoid overflows in calculations
    gyroDev->gyroADC[X] = (int32_t)gyroDev->gyroADCRaw[X] - (int32_t)gyroDev->gyroZero[X];
    gyroDev->gyroADC[Y] = (int32_t)gyroDev->gyroADCRaw[Y] - (int32_t)gyroDev->gyroZero[Y];
    gyroDev->gyroADC[Z] = (int32_t)gyroDev->gyroADCRaw[Z] - (int32_t)gyroDev->gyroZero[Z];

    alignSensors(gyroDev->gyroADC, gyroDev->gyroAlign);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // scale gyro output to degrees per second
        float gyroADCf = (float)gyroDev->gyroADC[axis] * gyroDev->scale;
        gyroADCf = softLpfFilterApplyFn(softLpfFilter[axis], gyroADCf);
        gyroADCf = notchFilter1ApplyFn(notchFilter1[axis], gyroADCf);
        gyroADCf = notchFilter2ApplyFn(notchFilter2[axis], gyroADCf);
        gyro.gyroADCf[axis] = gyroADCf;
    }
#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyse(gyroDev, &gyro);
#endif
    return true;
}
#endif

void gyroUpdate(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    if (gyroDev0.update) {
        // if the gyro update function is set then return, since the gyro is read in gyroUpdateISR
        return;
    }
    if (!gyroDev0.read(&gyroDev0)) {
        return;
    }
    gyroDev0.dataReady = false;

    if (isGyroCalibrationComplete()) {
#if defined(GYRO_USES_SPI) && defined(USE_MPU_DATA_READY_SIGNAL)
        // SPI-based gyro so can read and update in ISR
        if (gyroConfig()->gyro_isr_update) {
            mpuGyroSetIsrUpdate(&gyroDev0, gyroUpdateISR);
            return;
        }
#endif
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
        debug[3] = (uint16_t)(micros() & 0xffff);
#endif
        // move gyro data into 32-bit variables to avoid overflows in calculations
        gyroDev0.gyroADC[X] = (int32_t)gyroDev0.gyroADCRaw[X] - (int32_t)gyroDev0.gyroZero[X];
        gyroDev0.gyroADC[Y] = (int32_t)gyroDev0.gyroADCRaw[Y] - (int32_t)gyroDev0.gyroZero[Y];
        gyroDev0.gyroADC[Z] = (int32_t)gyroDev0.gyroADCRaw[Z] - (int32_t)gyroDev0.gyroZero[Z];

        alignSensors(gyroDev0.gyroADC, gyroDev0.gyroAlign);
    } else {
        performGyroCalibration(&gyroDev0, &gyroCalibration, gyroConfig()->gyroMovementCalibrationThreshold);
        // Reset gyro values to zero to prevent other code from using uncalibrated data
        gyro.gyroADCf[X] = 0.0f;
        gyro.gyroADCf[Y] = 0.0f;
        gyro.gyroADCf[Z] = 0.0f;
        // still calibrating, so no need to further process gyro data
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // scale gyro output to degrees per second
        float gyroADCf = (float)gyroDev0.gyroADC[axis] * gyroDev0.scale;

        // Apply LPF
        DEBUG_SET(DEBUG_GYRO, axis, lrintf(gyroADCf));
        gyroADCf = softLpfFilterApplyFn(softLpfFilter[axis], gyroADCf);

        // Apply Notch filtering
        DEBUG_SET(DEBUG_NOTCH, axis, lrintf(gyroADCf));
        gyroADCf = notchFilter1ApplyFn(notchFilter1[axis], gyroADCf);
        gyroADCf = notchFilter2ApplyFn(notchFilter2[axis], gyroADCf);
        gyro.gyroADCf[axis] = gyroADCf;
    }

#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyse(&gyroDev0, &gyro);
#endif
}

void gyroReadTemperature(void)
{
    if (gyroDev0.temperature) {
        gyroDev0.temperature(&gyroDev0, &gyroTemperature0);
    }
}

int16_t gyroGetTemperature(void)
{
    return gyroTemperature0;
}

int16_t gyroRateDps(int axis)
{
    return lrintf(gyro.gyroADCf[axis] / gyroDev0.scale);
}
