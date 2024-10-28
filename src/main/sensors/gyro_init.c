/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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

#include "config/config.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/accgyro/accgyro_spi_lsm6dso.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsv16x.h"

#ifdef USE_GYRO_L3GD20
#include "drivers/accgyro/accgyro_spi_l3gd20.h"
#endif

#ifdef USE_GYRO_L3G4200D
#include "drivers/accgyro/legacy/accgyro_l3g4200d.h"
#endif

#include "drivers/accgyro/gyro_sync.h"

#include "fc/runtime_config.h"

#ifdef USE_DYN_NOTCH_FILTER
#include "flight/dyn_notch_filter.h"
#endif

#include "pg/gyrodev.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#if !defined(USE_GYRO_L3G4200D) && !defined(USE_GYRO_L3GD20) \
    && !defined(USE_GYRO_MPU3050) && !defined(USE_GYRO_MPU6050) && !defined(USE_GYRO_MPU6500) \
    && !defined(USE_GYRO_SPI_MPU6000) && !defined(USE_GYRO_SPI_MPU6500) && !defined(USE_GYRO_SPI_MPU9250) \
    && !defined(USE_GYRO_SPI_ICM20602) && !defined(USE_GYRO_SPI_ICM20649) && !defined(USE_GYRO_SPI_ICM20689) \
    && !defined(USE_ACCGYRO_BMI160) && !defined(USE_ACCGYRO_BMI270) \
    && !defined(USE_GYRO_SPI_ICM42605) && !defined(USE_GYRO_SPI_ICM42688P) \
    && !defined(USE_ACCGYRO_LSM6DSO) && !defined(USE_ACCGYRO_LSM6DSV16X) \
    && !defined(USE_VIRTUAL_GYRO)
#error At least one USE_GYRO device definition required
#endif

#ifdef USE_MULTI_GYRO
#define ACTIVE_GYRO ((gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_2) ? &gyro.gyroSensor2 : &gyro.gyroSensor1)
#else
#define ACTIVE_GYRO (&gyro.gyroSensor1)
#endif

// The gyro buffer is split 50/50, the first half for the transmit buffer, the second half for the receive buffer
// This buffer is large enough for the gyros currently supported in accgyro_mpu.c but should be reviewed id other
// gyro types are supported with SPI DMA.
#define GYRO_BUF_SIZE 32

static gyroDetectionFlags_t gyroDetectionFlags = GYRO_NONE_MASK;

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

static void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyro.notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyro.notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyro.notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyro.notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyro.notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyro.notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LPF1:
        lowpassFilterApplyFn = &gyro.lowpassFilterApplyFn;
        lowpassFilter = gyro.lowpassFilter;
        break;

    case FILTER_LPF2:
        lowpassFilterApplyFn = &gyro.lowpass2FilterApplyFn;
        lowpassFilter = gyro.lowpass2Filter;
        break;

    default:
        return false;
    }

    bool ret = false;

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / looptime;
    const float gyroDt = looptime * 1e-6f;

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified
    if (lpfHz) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&lowpassFilter[axis].pt1FilterState, pt1FilterGain(lpfHz, gyroDt));
            }
            ret = true;
            break;
        case FILTER_BIQUAD:
            if (lpfHz <= gyroFrequencyNyquist) {
#ifdef USE_DYN_LPF
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApplyDF1;
#else
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
#endif
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz, looptime);
                }
                ret = true;
            }
            break;
        case FILTER_PT2:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt2FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterInit(&lowpassFilter[axis].pt2FilterState, pt2FilterGain(lpfHz, gyroDt));
            }
            ret = true;
            break;
        case FILTER_PT3:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt3FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterInit(&lowpassFilter[axis].pt3FilterState, pt3FilterGain(lpfHz, gyroDt));
            }
            ret = true;
            break;
        }
    }
    return ret;
}

#ifdef USE_DYN_LPF
static void dynLpfFilterInit(void)
{
    if (gyroConfig()->gyro_lpf1_dyn_min_hz > 0) {
        switch (gyroConfig()->gyro_lpf1_type) {
        case FILTER_PT1:
            gyro.dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            gyro.dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        case FILTER_PT2:
            gyro.dynLpfFilter = DYN_LPF_PT2;
            break;
        case FILTER_PT3:
            gyro.dynLpfFilter = DYN_LPF_PT3;
            break;
        default:
            gyro.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        gyro.dynLpfFilter = DYN_LPF_NONE;
    }
    gyro.dynLpfMin = gyroConfig()->gyro_lpf1_dyn_min_hz;
    gyro.dynLpfMax = gyroConfig()->gyro_lpf1_dyn_max_hz;
    gyro.dynLpfCurveExpo = gyroConfig()->gyro_lpf1_dyn_expo;
}
#endif

void gyroInitFilters(void)
{
    uint16_t gyro_lpf1_init_hz = gyroConfig()->gyro_lpf1_static_hz;

#ifdef USE_DYN_LPF
    if (gyroConfig()->gyro_lpf1_dyn_min_hz > 0) {
        gyro_lpf1_init_hz = gyroConfig()->gyro_lpf1_dyn_min_hz;
    }
#endif

    gyroInitLowpassFilterLpf(
      FILTER_LPF1,
      gyroConfig()->gyro_lpf1_type,
      gyro_lpf1_init_hz,
      gyro.targetLooptime
    );

    gyro.downsampleFilterEnabled = gyroInitLowpassFilterLpf(
      FILTER_LPF2,
      gyroConfig()->gyro_lpf2_type,
      gyroConfig()->gyro_lpf2_static_hz,
      gyro.sampleLooptime
    );

    gyroInitFilterNotch1(gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
#ifdef USE_DYN_LPF
    dynLpfFilterInit();
#endif
#ifdef USE_DYN_NOTCH_FILTER
    dynNotchInit(dynNotchConfig(), gyro.targetLooptime);
#endif

    const float k = pt1FilterGain(GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ, gyro.targetLooptime * 1e-6f);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pt1FilterInit(&gyro.imuGyroFilter[axis], k);
    }
}

#if defined(USE_GYRO_SLEW_LIMITER)
void gyroInitSlewLimiter(gyroSensor_t *gyroSensor)
{

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = 0;
    }
}
#endif

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
#if defined(USE_GYRO_SLEW_LIMITER)
    gyroInitSlewLimiter(gyroSensor);
#else
    UNUSED(gyroSensor);
#endif
}

void gyroInitSensor(gyroSensor_t *gyroSensor, const gyroDeviceConfig_t *config)
{
    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;
    gyroSensor->gyroDev.gyroAlign = config->alignment;
    buildRotationMatrixFromAngles(&gyroSensor->gyroDev.rotationMatrix, &config->customAlignment);
    gyroSensor->gyroDev.mpuIntExtiTag = config->extiTag;
    gyroSensor->gyroDev.hardware_lpf = gyroConfig()->gyro_hardware_lpf;

    // The targetLooptime gets set later based on the active sensor's gyroSampleRateHz and pid_process_denom
    gyroSensor->gyroDev.gyroSampleRateHz = gyroSetSampleRate(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);

    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.
    switch (gyroSensor->gyroDev.gyroHardware) {
    case GYRO_NONE:    // Won't ever actually get here, but included to account for all gyro types
    case GYRO_DEFAULT:
    case GYRO_VIRTUAL:
    case GYRO_MPU6050:
    case GYRO_L3G4200D:
    case GYRO_MPU3050:
    case GYRO_L3GD20:
    case GYRO_BMI160:
    case GYRO_BMI270:
    case GYRO_MPU6000:
    case GYRO_MPU6500:
    case GYRO_MPU9250:
    case GYRO_LSM6DSO:
    case GYRO_LSM6DSV16X:
    case GYRO_ICM42688P:
    case GYRO_ICM42605:
        gyroSensor->gyroDev.gyroHasOverflowProtection = true;
        break;

    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
    case GYRO_ICM20649:  // we don't actually know if this is affected, but as there are currently no flight controllers using it we err on the side of caution
    case GYRO_ICM20689:
        gyroSensor->gyroDev.gyroHasOverflowProtection = false;
        break;

    default:
        gyroSensor->gyroDev.gyroHasOverflowProtection = false;  // default catch for newly added gyros until proven to be unaffected
        break;
    }

    gyroInitSensorFilters(gyroSensor);
}

STATIC_UNIT_TESTED gyroHardware_e gyroDetect(gyroDev_t *dev)
{
    gyroHardware_e gyroHardware = GYRO_DEFAULT;

    switch (gyroHardware) {
    case GYRO_DEFAULT:
        FALLTHROUGH;

#ifdef USE_GYRO_MPU6050
    case GYRO_MPU6050:
        if (mpu6050GyroDetect(dev)) {
            gyroHardware = GYRO_MPU6050;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3G4200D
    case GYRO_L3G4200D:
        if (l3g4200dDetect(dev)) {
            gyroHardware = GYRO_L3G4200D;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_MPU3050
    case GYRO_MPU3050:
        if (mpu3050Detect(dev)) {
            gyroHardware = GYRO_MPU3050;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3GD20
    case GYRO_L3GD20:
        if (l3gd20GyroDetect(dev)) {
            gyroHardware = GYRO_L3GD20;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU6000
    case GYRO_MPU6000:
        if (mpu6000SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU6000;
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
        if (mpu6500SpiGyroDetect(dev)) {
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
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU9250
    case GYRO_MPU9250:
        if (mpu9250SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU9250;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20649
    case GYRO_ICM20649:
        if (icm20649SpiGyroDetect(dev)) {
            gyroHardware = GYRO_ICM20649;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20689
    case GYRO_ICM20689:
        if (icm20689SpiGyroDetect(dev)) {
            gyroHardware = GYRO_ICM20689;
            break;
        }
        FALLTHROUGH;
#endif

#if defined(USE_GYRO_SPI_ICM42605) || defined(USE_GYRO_SPI_ICM42688P)
    case GYRO_ICM42605:
    case GYRO_ICM42688P:
        if (icm426xxSpiGyroDetect(dev)) {
            switch (dev->mpuDetectionResult.sensor) {
            case ICM_42605_SPI:
                gyroHardware = GYRO_ICM42605;
                break;
            case ICM_42688P_SPI:
                gyroHardware = GYRO_ICM42688P;
                break;
            default:
                gyroHardware = GYRO_NONE;
                break;
            }
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
    case GYRO_BMI160:
        if (bmi160SpiGyroDetect(dev)) {
            gyroHardware = GYRO_BMI160;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI270
    case GYRO_BMI270:
        if (bmi270SpiGyroDetect(dev)) {
            gyroHardware = GYRO_BMI270;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_LSM6DSO
    case GYRO_LSM6DSO:
        if (lsm6dsoSpiGyroDetect(dev)) {
            gyroHardware = GYRO_LSM6DSO;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_LSM6DSV16X
    case GYRO_LSM6DSV16X:
        if (lsm6dsv16xSpiGyroDetect(dev)) {
            gyroHardware = GYRO_LSM6DSV16X;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_VIRTUAL_GYRO
    case GYRO_VIRTUAL:
        if (virtualGyroDetect(dev)) {
            gyroHardware = GYRO_VIRTUAL;
            break;
        }
        FALLTHROUGH;
#endif

    default:
        gyroHardware = GYRO_NONE;
    }

    if (gyroHardware != GYRO_NONE) {
        sensorsSet(SENSOR_GYRO);
    }


    return gyroHardware;
}

static bool gyroDetectSensor(gyroSensor_t *gyroSensor, const gyroDeviceConfig_t *config)
{
#ifdef USE_VIRTUAL_GYRO
    UNUSED(config);
#else
    bool gyroFound = mpuDetect(&gyroSensor->gyroDev, config);
    if (!gyroFound) {
        return false;
    }
#endif

    const gyroHardware_e gyroHardware = gyroDetect(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.gyroHardware = gyroHardware;

    return gyroHardware != GYRO_NONE;
}

static void gyroPreInitSensor(const gyroDeviceConfig_t *config)
{
#ifdef USE_VIRTUAL_GYRO
    UNUSED(config);
#else
    mpuPreInit(config);
#endif
}

void gyroPreInit(void)
{
    gyroPreInitSensor(gyroDeviceConfig(0));
#ifdef USE_MULTI_GYRO
    gyroPreInitSensor(gyroDeviceConfig(1));
#endif
}

bool gyroInit(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_YAW) {
        gyro.overflowAxisMask = GYRO_OVERFLOW_Z;
    } else if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_ALL_AXES) {
        gyro.overflowAxisMask = GYRO_OVERFLOW_X | GYRO_OVERFLOW_Y | GYRO_OVERFLOW_Z;
    } else {
        gyro.overflowAxisMask = 0;
    }
#endif

    gyro.gyroDebugMode = DEBUG_NONE;
    gyro.useDualGyroDebugging = false;
    gyro.gyroHasOverflowProtection = true;

    switch (debugMode) {
    case DEBUG_FFT:
    case DEBUG_FFT_FREQ:
    case DEBUG_GYRO_RAW:
    case DEBUG_GYRO_FILTERED:
    case DEBUG_DYN_LPF:
    case DEBUG_GYRO_SAMPLE:
        gyro.gyroDebugMode = debugMode;
        break;
    case DEBUG_DUAL_GYRO_DIFF:
    case DEBUG_DUAL_GYRO_RAW:
    case DEBUG_DUAL_GYRO_SCALED:
        gyro.useDualGyroDebugging = true;
        break;
    }

    gyroDetectionFlags = GYRO_NONE_MASK;
    uint8_t gyrosToScan = gyroConfig()->gyrosDetected;

    gyro.gyroToUse = gyroConfig()->gyro_to_use;
    gyro.gyroDebugAxis = gyroConfig()->gyro_filter_debug_axis;

    if ((!gyrosToScan || (gyrosToScan & GYRO_1_MASK)) && gyroDetectSensor(&gyro.gyroSensor1, gyroDeviceConfig(0))) {
        gyroDetectionFlags |= GYRO_1_MASK;
    }

#if defined(USE_MULTI_GYRO)
    if ((!gyrosToScan || (gyrosToScan & GYRO_2_MASK)) && gyroDetectSensor(&gyro.gyroSensor2, gyroDeviceConfig(1))) {
        gyroDetectionFlags |= GYRO_2_MASK;
    }
#endif

    if (gyroDetectionFlags == GYRO_NONE_MASK) {
        return false;
    }

    bool eepromWriteRequired = false;
    if (!gyrosToScan) {
        gyroConfigMutable()->gyrosDetected = gyroDetectionFlags;
        eepromWriteRequired = true;
    }

#if defined(USE_MULTI_GYRO)
    if ((gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH && !((gyroDetectionFlags & GYRO_ALL_MASK) == GYRO_ALL_MASK))
        || (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_1 && !(gyroDetectionFlags & GYRO_1_MASK))
        || (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_2 && !(gyroDetectionFlags & GYRO_2_MASK))) {
        if (gyroDetectionFlags & GYRO_1_MASK) {
            gyro.gyroToUse = GYRO_CONFIG_USE_GYRO_1;
        } else {
            gyro.gyroToUse = GYRO_CONFIG_USE_GYRO_2;
        }

        gyroConfigMutable()->gyro_to_use = gyro.gyroToUse;
        eepromWriteRequired = true;
    }

    // Only allow using both gyros simultaneously if they are the same hardware type.
    if (((gyroDetectionFlags & GYRO_ALL_MASK) == GYRO_ALL_MASK) && gyro.gyroSensor1.gyroDev.gyroHardware == gyro.gyroSensor2.gyroDev.gyroHardware) {
        gyroDetectionFlags |= GYRO_IDENTICAL_MASK;
    } else if (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        // If the user selected "BOTH" and they are not the same type, then reset to using only the first gyro.
        gyro.gyroToUse = GYRO_CONFIG_USE_GYRO_1;
        gyroConfigMutable()->gyro_to_use = gyro.gyroToUse;
        eepromWriteRequired = true;
    }

    if (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        static DMA_DATA uint8_t gyroBuf2[GYRO_BUF_SIZE];
        // SPI DMA buffer required per device
        gyro.gyroSensor2.gyroDev.dev.txBuf = gyroBuf2;
        gyro.gyroSensor2.gyroDev.dev.rxBuf = &gyroBuf2[GYRO_BUF_SIZE / 2];

        gyroInitSensor(&gyro.gyroSensor2, gyroDeviceConfig(1));
        gyro.gyroHasOverflowProtection = gyro.gyroHasOverflowProtection && gyro.gyroSensor2.gyroDev.gyroHasOverflowProtection;
        detectedSensors[SENSOR_INDEX_GYRO] = gyro.gyroSensor2.gyroDev.gyroHardware;
    }
#endif

    if (eepromWriteRequired) {
        writeEEPROM();
    }

    if (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        static DMA_DATA uint8_t gyroBuf1[GYRO_BUF_SIZE];
        // SPI DMA buffer required per device
        gyro.gyroSensor1.gyroDev.dev.txBuf = gyroBuf1;
        gyro.gyroSensor1.gyroDev.dev.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];
        gyroInitSensor(&gyro.gyroSensor1, gyroDeviceConfig(0));
        gyro.gyroHasOverflowProtection =  gyro.gyroHasOverflowProtection && gyro.gyroSensor1.gyroDev.gyroHasOverflowProtection;
        detectedSensors[SENSOR_INDEX_GYRO] = gyro.gyroSensor1.gyroDev.gyroHardware;
    }

    // Copy the sensor's scale to the high-level gyro object. If running in "BOTH" mode
    // then logic above requires both sensors to be the same so we'll use sensor1's scale.
    // This will need to be revised if we ever allow different sensor types to be used simultaneously.
    // Likewise determine the appropriate raw data for use in DEBUG_GYRO_RAW
    gyro.scale = gyro.gyroSensor1.gyroDev.scale;
    gyro.rawSensorDev = &gyro.gyroSensor1.gyroDev;
#if defined(USE_MULTI_GYRO)
    if (gyro.gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        gyro.scale = gyro.gyroSensor2.gyroDev.scale;
        gyro.rawSensorDev = &gyro.gyroSensor2.gyroDev;
    }
#endif

    if (gyro.rawSensorDev) {
        gyro.sampleRateHz = gyro.rawSensorDev->gyroSampleRateHz;
        gyro.accSampleRateHz = gyro.rawSensorDev->accSampleRateHz;
    } else {
        gyro.sampleRateHz = 0;
        gyro.accSampleRateHz = 0;
    }

    return true;
}

gyroDetectionFlags_t getGyroDetectionFlags(void)
{
    return gyroDetectionFlags;
}

void gyroSetTargetLooptime(uint8_t pidDenom)
{
    activePidLoopDenom = pidDenom;
    if (gyro.sampleRateHz) {
        gyro.sampleLooptime = 1e6f / gyro.sampleRateHz;
        gyro.targetLooptime = activePidLoopDenom * 1e6f / gyro.sampleRateHz;
    } else {
        gyro.sampleLooptime = 0;
        gyro.targetLooptime = 0;
    }
}


gyroDev_t *gyroActiveDev(void)
{
    return &ACTIVE_GYRO->gyroDev;
}

const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
    return &ACTIVE_GYRO->gyroDev.mpuDetectionResult;
}

int16_t gyroRateDps(int axis)
{
    return lrintf(gyro.gyroADCf[axis] / ACTIVE_GYRO->gyroDev.scale);
}

#ifdef USE_GYRO_REGISTER_DUMP
static extDevice_t *gyroSensorDevByInstance(uint8_t whichSensor)
{
#ifdef USE_MULTI_GYRO
    if (whichSensor == GYRO_CONFIG_USE_GYRO_2) {
        return &gyro.gyroSensor2.gyroDev.dev;
    }
#else
    UNUSED(whichSensor);
#endif
    return &gyro.gyroSensor1.gyroDev.dev;
}

uint8_t gyroReadRegister(uint8_t whichSensor, uint8_t reg)
{
    return mpuGyroReadRegister(gyroSensorDevByInstance(whichSensor), reg);
}
#endif // USE_GYRO_REGISTER_DUMP
