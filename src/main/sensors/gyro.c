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

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/accgyro.h"
#include "drivers/accgyro_adxl345.h"
#include "drivers/accgyro_bma280.h"
#include "drivers/accgyro_fake.h"
#include "drivers/accgyro_l3g4200d.h"
#include "drivers/accgyro_mma845x.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_mpu6500.h"
#include "drivers/accgyro_l3gd20.h"
#include "drivers/accgyro_lsm303dlhc.h"
#include "drivers/bus_spi.h"
#include "drivers/accgyro_spi_icm20689.h"
#include "drivers/accgyro_spi_mpu6000.h"
#include "drivers/accgyro_spi_mpu6500.h"
#include "drivers/accgyro_spi_mpu9250.h"
#include "drivers/gyro_sync.h"
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

gyro_t gyro;                      // gyro access functions

static int32_t gyroADC[XYZ_AXIS_COUNT];

static int32_t gyroZero[XYZ_AXIS_COUNT] = { 0, 0, 0 };
static const gyroConfig_t *gyroConfig;
static uint16_t calibratingG = 0;

static filterApplyFnPtr softLpfFilterApplyFn;
static void *softLpfFilter[3];
static filterApplyFnPtr notchFilter1ApplyFn;
static void *notchFilter1[3];
static filterApplyFnPtr notchFilter2ApplyFn;
static void *notchFilter2[3];

bool gyroDetect(gyroDev_t *dev)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    dev->gyroAlign = ALIGN_DEFAULT;

    switch(gyroHardware) {
    case GYRO_DEFAULT:
        ; // fallthrough
    case GYRO_MPU6050:
#ifdef USE_GYRO_MPU6050
        if (mpu6050GyroDetect(dev)) {
            gyroHardware = GYRO_MPU6050;
#ifdef GYRO_MPU6050_ALIGN
            dev->gyroAlign = GYRO_MPU6050_ALIGN;
#endif
            break;
        }
#endif
        ; // fallthrough
    case GYRO_L3G4200D:
#ifdef USE_GYRO_L3G4200D
        if (l3g4200dDetect(dev)) {
            gyroHardware = GYRO_L3G4200D;
#ifdef GYRO_L3G4200D_ALIGN
            dev->gyroAlign = GYRO_L3G4200D_ALIGN;
#endif
            break;
        }
#endif
        ; // fallthrough

    case GYRO_MPU3050:
#ifdef USE_GYRO_MPU3050
        if (mpu3050Detect(dev)) {
            gyroHardware = GYRO_MPU3050;
#ifdef GYRO_MPU3050_ALIGN
            dev->gyroAlign = GYRO_MPU3050_ALIGN;
#endif
            break;
        }
#endif
        ; // fallthrough

    case GYRO_L3GD20:
#ifdef USE_GYRO_L3GD20
        if (l3gd20Detect(dev)) {
            gyroHardware = GYRO_L3GD20;
#ifdef GYRO_L3GD20_ALIGN
            dev->gyroAlign = GYRO_L3GD20_ALIGN;
#endif
            break;
        }
#endif
        ; // fallthrough

    case GYRO_MPU6000:
#ifdef USE_GYRO_SPI_MPU6000
        if (mpu6000SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU6000;
#ifdef GYRO_MPU6000_ALIGN
            dev->gyroAlign = GYRO_MPU6000_ALIGN;
#endif
            break;
        }
#endif
        ; // fallthrough

    case GYRO_MPU6500:
#if defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500)
#ifdef USE_GYRO_SPI_MPU6500
        if (mpu6500GyroDetect(dev) || mpu6500SpiGyroDetect(dev))
#else
        if (mpu6500GyroDetect(dev))
#endif
        {
            gyroHardware = GYRO_MPU6500;
#ifdef GYRO_MPU6500_ALIGN
            dev->gyroAlign = GYRO_MPU6500_ALIGN;
#endif

            break;
        }
#endif
        ; // fallthrough

case GYRO_MPU9250:
#ifdef USE_GYRO_SPI_MPU9250

    if (mpu9250SpiGyroDetect(dev))
    {
        gyroHardware = GYRO_MPU9250;
#ifdef GYRO_MPU9250_ALIGN
        dev->gyroAlign = GYRO_MPU9250_ALIGN;
#endif

        break;
    }
#endif
    ; // fallthrough

    case GYRO_ICM20689:
#ifdef USE_GYRO_SPI_ICM20689
        if (icm20689SpiGyroDetect(dev))
        {
            gyroHardware = GYRO_ICM20689;
#ifdef GYRO_ICM20689_ALIGN
            dev->gyroAlign = GYRO_ICM20689_ALIGN;
#endif

            break;
        }
#endif
        ; // fallthrough

    case GYRO_FAKE:
#ifdef USE_FAKE_GYRO
        if (fakeGyroDetect(dev)) {
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

void gyroInit(const gyroConfig_t *gyroConfigToUse)
{
    static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];
    static pt1Filter_t gyroFilterPt1[XYZ_AXIS_COUNT];
    static firFilterDenoise_t gyroDenoiseState[XYZ_AXIS_COUNT];
    static biquadFilter_t gyroFilterNotch_1[XYZ_AXIS_COUNT];
    static biquadFilter_t gyroFilterNotch_2[XYZ_AXIS_COUNT];

    gyroConfig = gyroConfigToUse;
    gyro.targetLooptime = gyroSetSampleRate(gyroConfig->gyro_lpf, gyroConfig->gyro_sync_denom);    // Set gyro sample rate before initialisation
    gyro.dev.lpf = gyroConfig->gyro_lpf;
    gyro.dev.init(&gyro.dev);

    softLpfFilterApplyFn = nullFilterApply;
    notchFilter1ApplyFn = nullFilterApply;
    notchFilter2ApplyFn = nullFilterApply;

    if (gyroConfig->gyro_soft_lpf_hz) {  // Initialisation needs to happen once samplingrate is known
        if (gyroConfig->gyro_soft_lpf_type == FILTER_BIQUAD) {
            softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = 0; axis < 3; axis++) {
                softLpfFilter[axis] = &gyroFilterLPF[axis];
                biquadFilterInitLPF(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, gyro.targetLooptime);
            }
        } else if (gyroConfig->gyro_soft_lpf_type == FILTER_PT1) {
            softLpfFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
            const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
            for (int axis = 0; axis < 3; axis++) {
                softLpfFilter[axis] = &gyroFilterPt1[axis];
                pt1FilterInit(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, gyroDt);
            }
        } else {
            softLpfFilterApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = 0; axis < 3; axis++) {
                softLpfFilter[axis] = &gyroDenoiseState[axis];
                firFilterDenoiseInit(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, gyro.targetLooptime);
            }
        }
    }

    if (gyroConfig->gyro_soft_notch_hz_1) {
        notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float gyroSoftNotchQ1 = filterGetNotchQ(gyroConfig->gyro_soft_notch_hz_1, gyroConfig->gyro_soft_notch_cutoff_1);
        for (int axis = 0; axis < 3; axis++) {
            notchFilter1[axis] = &gyroFilterNotch_1[axis];
            biquadFilterInit(notchFilter1[axis], gyroConfig->gyro_soft_notch_hz_1, gyro.targetLooptime, gyroSoftNotchQ1, FILTER_NOTCH);
        }
    }
    if (gyroConfig->gyro_soft_notch_hz_2) {
        notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float gyroSoftNotchQ2 = filterGetNotchQ(gyroConfig->gyro_soft_notch_hz_2, gyroConfig->gyro_soft_notch_cutoff_2);
        for (int axis = 0; axis < 3; axis++) {
            notchFilter2[axis] = &gyroFilterNotch_2[axis];
            biquadFilterInit(notchFilter2[axis], gyroConfig->gyro_soft_notch_hz_2, gyro.targetLooptime, gyroSoftNotchQ2, FILTER_NOTCH);
        }
    }
}

bool isGyroCalibrationComplete(void)
{
    return calibratingG == 0;
}

static bool isOnFinalGyroCalibrationCycle(void)
{
    return calibratingG == 1;
}

static uint16_t gyroCalculateCalibratingCycles(void)
{
    return (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime) * CALIBRATING_GYRO_CYCLES;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
    return calibratingG == gyroCalculateCalibratingCycles();
}

void gyroSetCalibrationCycles(void)
{
    calibratingG = gyroCalculateCalibratingCycles();
}

static void performGyroCalibration(uint8_t gyroMovementCalibrationThreshold)
{
    static int32_t g[3];
    static stdev_t var[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle()) {
            g[axis] = 0;
            devClear(&var[axis]);
        }

        // Sum up CALIBRATING_GYRO_CYCLES readings
        g[axis] += gyroADC[axis];
        devPush(&var[axis], gyroADC[axis]);

        // Reset global variables to prevent other code from using un-calibrated data
        gyroADC[axis] = 0;
        gyroZero[axis] = 0;

        if (isOnFinalGyroCalibrationCycle()) {
            float dev = devStandardDeviation(&var[axis]);
            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && dev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles();
                return;
            }
            gyroZero[axis] = (g[axis] + (gyroCalculateCalibratingCycles() / 2)) / gyroCalculateCalibratingCycles();
        }
    }

    if (isOnFinalGyroCalibrationCycle()) {
        beeper(BEEPER_GYRO_CALIBRATED);
    }
    calibratingG--;

}

void gyroUpdate(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    if (!gyro.dev.read(&gyro.dev)) {
        return;
    }
    gyro.dev.dataReady = false;
    gyroADC[X] = gyro.dev.gyroADCRaw[X];
    gyroADC[Y] = gyro.dev.gyroADCRaw[Y];
    gyroADC[Z] = gyro.dev.gyroADCRaw[Z];

    alignSensors(gyroADC, gyro.dev.gyroAlign);

    const bool calibrationComplete = isGyroCalibrationComplete();
    if (!calibrationComplete) {
        performGyroCalibration(gyroConfig->gyroMovementCalibrationThreshold);
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        // scale gyro output to degrees per second
        gyro.gyroADCf[axis] = (float)gyroADC[axis] * gyro.dev.scale;

        DEBUG_SET(DEBUG_GYRO, axis, lrintf(gyro.gyroADCf[axis]));

        gyro.gyroADCf[axis] = softLpfFilterApplyFn(softLpfFilter[axis], gyro.gyroADCf[axis]);

        DEBUG_SET(DEBUG_NOTCH, axis, lrintf(gyro.gyroADCf[axis]));

        gyro.gyroADCf[axis] = notchFilter1ApplyFn(notchFilter1[axis], gyro.gyroADCf[axis]);

        gyro.gyroADCf[axis] = notchFilter2ApplyFn(notchFilter2[axis], gyro.gyroADCf[axis]);

        if (!calibrationComplete) {
            gyroADC[axis] = lrintf(gyro.gyroADCf[axis] / gyro.dev.scale);
        }
    }
}
