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
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_reset.h"
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
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/logging.h"
#include "drivers/sensor.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif


acc_t acc;                       // acc access functions

static uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

static biquadFilter_t accFilter[XYZ_AXIS_COUNT];

#ifdef USE_ACC_NOTCH
static filterApplyFnPtr accNotchFilterApplyFn;
static void *accNotchFilter[XYZ_AXIS_COUNT];
#endif

PG_REGISTER_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 1);

void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance)
{
    RESET_CONFIG_2(accelerometerConfig_t, instance,
        .acc_align = ALIGN_DEFAULT,
        .acc_hardware = ACC_AUTODETECT,
        .acc_lpf_hz = 15,
        .acc_notch_hz = 0,
        .acc_notch_cutoff = 1
    );
    RESET_CONFIG_2(flightDynamicsTrims_t, &instance->accZero,
        .raw[X] = 0,
        .raw[Y] = 0,
        .raw[Z] = 0
    );
    RESET_CONFIG_2(flightDynamicsTrims_t, &instance->accGain,
         .raw[X] = 4096,
         .raw[Y] = 4096,
         .raw[Z] = 4096
    );
}

static bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware = ACC_NONE;

#ifdef USE_ACC_ADXL345
#endif

    dev->accAlign = ALIGN_DEFAULT;

    requestedSensors[SENSOR_INDEX_ACC] = accHardwareToUse;

    switch (accHardwareToUse) {
    case ACC_AUTODETECT:
        ; // fallthrough
#ifdef USE_ACC_ADXL345
    case ACC_ADXL345: {
        drv_adxl345_config_t acc_params;
        acc_params.useFifo = false;
        acc_params.dataRate = 800; // unused currently
#ifdef NAZE
        if (hardwareRevision < NAZE32_REV5 && adxl345Detect(dev, &acc_params)) {
#else
        if (adxl345Detect(dev, &acc_params)) {
#endif
#ifdef ACC_ADXL345_ALIGN
            dev->accAlign = ACC_ADXL345_ALIGN;
#endif
            accHardware = ACC_ADXL345;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
    }
#endif

#ifdef USE_ACC_LSM303DLHC
    case ACC_LSM303DLHC:
        if (lsm303dlhcAccDetect(dev)) {
#ifdef ACC_LSM303DLHC_ALIGN
            dev->accAlign = ACC_LSM303DLHC_ALIGN;
#endif
            accHardware = ACC_LSM303DLHC;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#ifdef USE_ACC_MPU6050
    case ACC_MPU6050: // MPU6050
        if (mpu6050AccDetect(dev)) {
#ifdef ACC_MPU6050_ALIGN
            dev->accAlign = ACC_MPU6050_ALIGN;
#endif
            accHardware = ACC_MPU6050;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#ifdef USE_ACC_MMA8452
    case ACC_MMA8452: // MMA8452
#ifdef NAZE
        // Not supported with this frequency
        if (hardwareRevision < NAZE32_REV5 && mma8452Detect(dev)) {
#else
        if (mma8452Detect(dev)) {
#endif
#ifdef ACC_MMA8452_ALIGN
            dev->accAlign = ACC_MMA8452_ALIGN;
#endif
            accHardware = ACC_MMA8452;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#ifdef USE_ACC_BMA280
    case ACC_BMA280: // BMA280
        if (bma280Detect(dev)) {
#ifdef ACC_BMA280_ALIGN
            dev->accAlign = ACC_BMA280_ALIGN;
#endif
            accHardware = ACC_BMA280;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#ifdef USE_ACC_SPI_MPU6000
    case ACC_MPU6000:
        if (mpu6000SpiAccDetect(dev)) {
#ifdef ACC_MPU6000_ALIGN
            dev->accAlign = ACC_MPU6000_ALIGN;
#endif
            accHardware = ACC_MPU6000;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
    case ACC_MPU6500:
#ifdef USE_ACC_SPI_MPU6500
        if (mpu6500AccDetect(dev) || mpu6500SpiAccDetect(dev)) {
#else
        if (mpu6500AccDetect(dev)) {
#endif
#ifdef ACC_MPU6500_ALIGN
            dev->accAlign = ACC_MPU6500_ALIGN;
#endif
            accHardware = ACC_MPU6500;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#if defined(USE_ACC_SPI_MPU9250)
    case ACC_MPU9250:
        if (mpu9250SpiAccDetect(dev)) {
#ifdef ACC_MPU9250_ALIGN
            dev->accAlign = ACC_MPU9250_ALIGN;
#endif
            accHardware = ACC_MPU9250;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

#ifdef USE_FAKE_ACC
    case ACC_FAKE:
        if (fakeAccDetect(dev)) {
            accHardware = ACC_FAKE;
            break;
        }
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }
#endif

    default:
    case ACC_NONE: // disable ACC
        accHardware = ACC_NONE;
        break;
    }

    addBootlogEvent6(BOOT_EVENT_ACC_DETECTION, BOOT_EVENT_FLAGS_NONE, accHardware, 0, 0, 0);

    if (accHardware == ACC_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
    return true;
}

bool accInit(uint32_t targetLooptime)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.bus = *gyroSensorBus();
    acc.dev.mpuConfiguration = *gyroMpuConfiguration();
    acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    if (!accDetect(&acc.dev, accelerometerConfig()->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.initFn(&acc.dev);
    acc.accTargetLooptime = targetLooptime;
    accInitFilters();
    if (accelerometerConfig()->acc_align != ALIGN_DEFAULT) {
        acc.dev.accAlign = accelerometerConfig()->acc_align;
    }
    return true;
}

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

bool accIsCalibrationComplete(void)
{
    return calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static sensorCalibrationState_t calState;
static bool calibratedAxis[6];
static int32_t accSamples[6][3];
static int  calibratedAxisCount = 0;

bool accGetCalibrationAxisStatus(int axis)
{
    if (accIsCalibrationComplete()) {
        if (STATE(ACCELEROMETER_CALIBRATED)) {
            return true;    // if calibration is valid - all axis are calibrated
        }
        else {
            return calibratedAxis[axis];
        }
    }
    else {
        return calibratedAxis[axis];
    }
}

uint8_t accGetCalibrationAxisFlags(void)
{
    uint8_t flags = 0;
    for (int i = 0; i < 6; i++) {
        if (accGetCalibrationAxisStatus(0)) {
            flags |= (1 << i);
        }
    }

    return flags;
}

int getPrimaryAxisIndex(int32_t sample[3])
{
    // Tolerate up to atan(1 / 1.5) = 33 deg tilt (in worst case 66 deg separation between points)
    if ((ABS(sample[Z]) / 1.5f) > ABS(sample[X]) && (ABS(sample[Z]) / 1.5f) > ABS(sample[Y])) {
        //Z-axis
        return (sample[Z] > 0) ? 0 : 1;
    }
    else if ((ABS(sample[X]) / 1.5f) > ABS(sample[Y]) && (ABS(sample[X]) / 1.5f) > ABS(sample[Z])) {
        //X-axis
        return (sample[X] > 0) ? 2 : 3;
    }
    else if ((ABS(sample[Y]) / 1.5f) > ABS(sample[X]) && (ABS(sample[Y]) / 1.5f) > ABS(sample[Z])) {
        //Y-axis
        return (sample[Y] > 0) ? 4 : 5;
    }
    else
        return -1;
}

static void performAcclerationCalibration(void)
{
    int axisIndex = getPrimaryAxisIndex(acc.accADC);

    // Check if sample is usable
    if (axisIndex < 0) {
        return;
    }

    // Top-up and first calibration cycle, reset everything
    if (axisIndex == 0 && isOnFirstAccelerationCalibrationCycle()) {
        for (int axis = 0; axis < 6; axis++) {
            calibratedAxis[axis] = false;
            accSamples[axis][X] = 0;
            accSamples[axis][Y] = 0;
            accSamples[axis][Z] = 0;
        }

        calibratedAxisCount = 0;
        sensorCalibrationResetState(&calState);
        DISABLE_STATE(ACCELEROMETER_CALIBRATED);
    }

    if (!calibratedAxis[axisIndex]) {
        sensorCalibrationPushSampleForOffsetCalculation(&calState, acc.accADC);
        accSamples[axisIndex][X] += acc.accADC[X];
        accSamples[axisIndex][Y] += acc.accADC[Y];
        accSamples[axisIndex][Z] += acc.accADC[Z];

        if (isOnFinalAccelerationCalibrationCycle()) {
            calibratedAxis[axisIndex] = true;
            calibratedAxisCount++;

            beeperConfirmationBeeps(2);
        }
    }

    if (calibratedAxisCount == 6) {
        float accTmp[3];
        int32_t accSample[3];

        /* Calculate offset */
        sensorCalibrationSolveForOffset(&calState, accTmp);

        for (int axis = 0; axis < 3; axis++) {
            accelerometerConfigMutable()->accZero.raw[axis] = lrintf(accTmp[axis]);
        }

        /* Not we can offset our accumulated averages samples and calculate scale factors and calculate gains */
        sensorCalibrationResetState(&calState);

        for (int axis = 0; axis < 6; axis++) {
            accSample[X] = accSamples[axis][X] / CALIBRATING_ACC_CYCLES - accelerometerConfig()->accZero.raw[X];
            accSample[Y] = accSamples[axis][Y] / CALIBRATING_ACC_CYCLES - accelerometerConfig()->accZero.raw[Y];
            accSample[Z] = accSamples[axis][Z] / CALIBRATING_ACC_CYCLES - accelerometerConfig()->accZero.raw[Z];

            sensorCalibrationPushSampleForScaleCalculation(&calState, axis / 2, accSample, acc.dev.acc_1G);
        }

        sensorCalibrationSolveForScale(&calState, accTmp);

        for (int axis = 0; axis < 3; axis++) {
            accelerometerConfigMutable()->accGain.raw[axis] = lrintf(accTmp[axis] * 4096);
        }

        saveConfigAndNotify();
    }

    calibratingA--;
}

static void applyAccelerationZero(const flightDynamicsTrims_t * accZero, const flightDynamicsTrims_t * accGain)
{
    acc.accADC[X] = (acc.accADC[X] - accZero->raw[X]) * accGain->raw[X] / 4096;
    acc.accADC[Y] = (acc.accADC[Y] - accZero->raw[Y]) * accGain->raw[Y] / 4096;
    acc.accADC[Z] = (acc.accADC[Z] - accZero->raw[Z]) * accGain->raw[Z] / 4096;
}

void accUpdate(void)
{
    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

    if (accelerometerConfig()->acc_lpf_hz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = lrintf(biquadFilterApply(&accFilter[axis], (float)acc.accADC[axis]));
        }
    }

#ifdef USE_ACC_NOTCH
    if (accelerometerConfig()->acc_notch_hz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = lrintf(accNotchFilterApplyFn(accNotchFilter[axis], (float)acc.accADC[axis]));
        }
    }
#endif

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration();
    }

    applyAccelerationZero(&accelerometerConfig()->accZero, &accelerometerConfig()->accGain);

    alignSensors(acc.accADC, acc.dev.accAlign);
}

void accSetCalibrationValues(void)
{
    if ((accelerometerConfig()->accZero.raw[X] == 0) && (accelerometerConfig()->accZero.raw[Y] == 0) && (accelerometerConfig()->accZero.raw[Z] == 0) &&
        (accelerometerConfig()->accGain.raw[X] == 4096) && (accelerometerConfig()->accGain.raw[Y] == 4096) &&(accelerometerConfig()->accGain.raw[Z] == 4096)) {
        DISABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
    else {
        ENABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
}

void accInitFilters(void)
{
    if (acc.accTargetLooptime && accelerometerConfig()->acc_lpf_hz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accelerometerConfig()->acc_lpf_hz, acc.accTargetLooptime);
        }    
    }

#ifdef USE_ACC_NOTCH
    static biquadFilter_t accFilterNotch[XYZ_AXIS_COUNT];
    accNotchFilterApplyFn = nullFilterApply;

    if (acc.accTargetLooptime && accelerometerConfig()->acc_notch_hz) {
        accNotchFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accNotchFilter[axis] = &accFilterNotch[axis];
            biquadFilterInitNotch(accNotchFilter[axis], acc.accTargetLooptime, accelerometerConfig()->acc_notch_hz, accelerometerConfig()->acc_notch_cutoff);
        }
    }
#endif

}

bool accIsHealthy(void)
{
    return true;
}
