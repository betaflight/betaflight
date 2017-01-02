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
#include "drivers/accgyro_spi_mpu6000.h"
#include "drivers/accgyro_spi_mpu6500.h"
#include "drivers/gyro_sync.h"
#include "drivers/io.h"
#include "drivers/logging.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

gyro_t gyro;                      // gyro access functions

static int32_t gyroZero[XYZ_AXIS_COUNT] = { 0, 0, 0 };
static const gyroConfig_t *gyroConfig;

static uint16_t calibratingG = 0;

static filterApplyFnPtr softLpfFilterApplyFn;
static void *softLpfFilter[XYZ_AXIS_COUNT];

#ifdef USE_GYRO_NOTCH_1
static filterApplyFnPtr notchFilter1ApplyFn;
static void *notchFilter1[XYZ_AXIS_COUNT];
#endif
#ifdef USE_GYRO_NOTCH_2
static filterApplyFnPtr notchFilter2ApplyFn;
static void *notchFilter2[XYZ_AXIS_COUNT];
#endif

static const extiConfig_t *selectMPUIntExtiConfig(void)
{
#if defined(MPU_INT_EXTI)
    static const extiConfig_t mpuIntExtiConfig = { .tag = IO_TAG(MPU_INT_EXTI) };
    return &mpuIntExtiConfig;
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    return selectMPUIntExtiConfigByHardwareRevision();
#else
    return NULL;
#endif
}

static bool gyroDetect(gyroDev_t *dev, const extiConfig_t *extiConfig)
{
    dev->mpuIntExtiConfig =  extiConfig;

    gyroSensor_e gyroHardware = GYRO_AUTODETECT;

    dev->gyroAlign = ALIGN_DEFAULT;

    switch(gyroHardware) {
    case GYRO_AUTODETECT:
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
        if (mpu6500GyroDetect(dev) || mpu6500SpiGyroDetect(dev)) {
#else
        if (mpu6500GyroDetect(dev)) {
#endif
            gyroHardware = GYRO_MPU6500;
#ifdef GYRO_MPU6500_ALIGN
            dev->gyroAlign = GYRO_MPU6500_ALIGN;
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

    addBootlogEvent6(BOOT_EVENT_GYRO_DETECTION, BOOT_EVENT_FLAGS_NONE, gyroHardware, 0, 0, 0);

    if (gyroHardware == GYRO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
    sensorsSet(SENSOR_GYRO);

    return true;
}

bool gyroInit(const gyroConfig_t *gyroConfigToUse)
{
    gyroConfig = gyroConfigToUse;
    memset(&gyro, 0, sizeof(gyro));
#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_MPU6050)
    const extiConfig_t *extiConfig = selectMPUIntExtiConfig();
    mpuDetect(&gyro.dev);
    mpuReset = gyro.dev.mpuConfiguration.reset;
#endif

    if (!gyroDetect(&gyro.dev, extiConfig)) {
        return false;
    }
    // After refactoring this function is always called after gyro sampling rate is known, so
    // no additional condition is required
    // Set gyro sample rate before driver initialisation
    gyro.dev.lpf = gyroConfig->gyro_lpf;
    gyro.targetLooptime = gyroSetSampleRate(gyroConfig->looptime, gyroConfig->gyro_lpf, gyroConfig->gyroSync, gyroConfig->gyroSyncDenominator);
    // driver initialisation
    gyro.dev.init(&gyro.dev);
	
    static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];
    softLpfFilterApplyFn = nullFilterApply;
#ifdef USE_GYRO_NOTCH_1
    static biquadFilter_t gyroFilterNotch_1[XYZ_AXIS_COUNT];
    notchFilter1ApplyFn = nullFilterApply;
#endif
#ifdef USE_GYRO_NOTCH_2
    static biquadFilter_t gyroFilterNotch_2[XYZ_AXIS_COUNT];
    notchFilter2ApplyFn = nullFilterApply;
#endif
	
    if (gyroConfig->gyro_soft_lpf_hz) {
        softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; axis++) {
            softLpfFilter[axis] = &gyroFilterLPF[axis];
        #ifdef ASYNC_GYRO_PROCESSING
            biquadFilterInitLPF(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, getGyroUpdateRate());
        #else
            biquadFilterInitLPF(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, gyro.targetLooptime);
        #endif
        }
    }

#ifdef USE_GYRO_NOTCH_1
    if (gyroConfig->gyro_soft_notch_hz_1) {
        notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; axis++) {
            notchFilter1[axis] = &gyroFilterNotch_1[axis];
        #ifdef ASYNC_GYRO_PROCESSING
            biquadFilterInitNotch(notchFilter1[axis], getGyroUpdateRate(), gyroConfig->gyro_soft_notch_hz_1, gyroConfig->gyro_soft_notch_cutoff_1);
        #else
            biquadFilterInitNotch(notchFilter1[axis], gyro.targetLooptime, gyroConfig->gyro_soft_notch_hz_1, gyroConfig->gyro_soft_notch_cutoff_1);
        #endif
        }
    }
#endif

#ifdef USE_GYRO_NOTCH_2
    if (gyroConfig->gyro_soft_notch_hz_2) {
        notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; axis++) {
            notchFilter2[axis] = &gyroFilterNotch_2[axis];
        #ifdef ASYNC_GYRO_PROCESSING
            biquadFilterInitNotch(notchFilter2[axis], getGyroUpdateRate(), gyroConfig->gyro_soft_notch_hz_2, gyroConfig->gyro_soft_notch_cutoff_2);
        #else
            biquadFilterInitNotch(notchFilter2[axis], gyro.targetLooptime, gyroConfig->gyro_soft_notch_hz_2, gyroConfig->gyro_soft_notch_cutoff_2);
        #endif
        }
    }
#endif
    return true;

}

void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingG = calibrationCyclesRequired;
}

bool isGyroCalibrationComplete(void)
{
    return calibratingG == 0;
}

static bool isOnFinalGyroCalibrationCycle(void)
{
    return calibratingG == 1;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
    return calibratingG == CALIBRATING_GYRO_CYCLES;
}

static void performAcclerationCalibration(uint8_t gyroMovementCalibrationThreshold)
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
        g[axis] += gyro.gyroADC[axis];
        devPush(&var[axis], gyro.gyroADC[axis]);

        // Reset global variables to prevent other code from using un-calibrated data
        gyro.gyroADC[axis] = 0;
        gyroZero[axis] = 0;

        if (isOnFinalGyroCalibrationCycle()) {
            float dev = devStandardDeviation(&var[axis]);
            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && dev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                return;
            }
            gyroZero[axis] = (g[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
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
    float gyroADCf[XYZ_AXIS_COUNT];

    // Prepare a copy of int32_t gyroADC for mangling to prevent overflow
    gyro.gyroADC[X] = gyro.dev.gyroADCRaw[X];
    gyro.gyroADC[Y] = gyro.dev.gyroADCRaw[Y];
    gyro.gyroADC[Z] = gyro.dev.gyroADCRaw[Z];

    if (!isGyroCalibrationComplete()) {
        performAcclerationCalibration(gyroConfig->gyroMovementCalibrationThreshold);
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyro.gyroADC[axis] -= gyroZero[axis];
        
        gyroADCf[axis] = (float)gyro.gyroADC[axis];
        
        gyroADCf[axis] = softLpfFilterApplyFn(softLpfFilter[axis], gyroADCf[axis]);
#ifdef USE_GYRO_NOTCH_1
        gyroADCf[axis] = notchFilter1ApplyFn(notchFilter1[axis], gyroADCf[axis]);
#endif

#ifdef USE_GYRO_NOTCH_2
        gyroADCf[axis] = notchFilter2ApplyFn(notchFilter2[axis], gyroADCf[axis]);
#endif
        gyro.gyroADC[axis] = lrintf(gyroADCf[axis]);
    }    

    alignSensors(gyro.gyroADC, gyro.dev.gyroAlign);
}
