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

#include "config/config.h"

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
#include "drivers/logging.h"
#include "drivers/sensor.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "fc/runtime_config.h"

#include "io/beeper.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

acc_t acc;                       // acc access functions

static uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

static flightDynamicsTrims_t * accZero;
static flightDynamicsTrims_t * accGain;

static uint8_t accLpfCutHz = 0;
static biquadFilter_t accFilter[XYZ_AXIS_COUNT];

static bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware = ACC_NONE;

#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif

    dev->accAlign = ALIGN_DEFAULT;

    requestedSensors[SENSOR_INDEX_ACC] = accHardwareToUse;

    switch (accHardwareToUse) {
    case ACC_AUTODETECT:
        ; // fallthrough
    case ACC_ADXL345: // ADXL345
#ifdef USE_ACC_ADXL345
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
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_LSM303DLHC:
#ifdef USE_ACC_LSM303DLHC
        if (lsm303dlhcAccDetect(dev)) {
#ifdef ACC_LSM303DLHC_ALIGN
            dev->accAlign = ACC_LSM303DLHC_ALIGN;
#endif
            accHardware = ACC_LSM303DLHC;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_MPU6050: // MPU6050
#ifdef USE_ACC_MPU6050
        if (mpu6050AccDetect(dev)) {
#ifdef ACC_MPU6050_ALIGN
            dev->accAlign = ACC_MPU6050_ALIGN;
#endif
            accHardware = ACC_MPU6050;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_MMA8452: // MMA8452
#ifdef USE_ACC_MMA8452
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
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_BMA280: // BMA280
#ifdef USE_ACC_BMA280
        if (bma280Detect(dev)) {
#ifdef ACC_BMA280_ALIGN
            dev->accAlign = ACC_BMA280_ALIGN;
#endif
            accHardware = ACC_BMA280;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_MPU6000:
#ifdef USE_ACC_SPI_MPU6000
        if (mpu6000SpiAccDetect(dev)) {
#ifdef ACC_MPU6000_ALIGN
            dev->accAlign = ACC_MPU6000_ALIGN;
#endif
            accHardware = ACC_MPU6000;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_MPU6500:
#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
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
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

    case ACC_FAKE:
#ifdef USE_FAKE_ACC
        if (fakeAccDetect(dev)) {
            accHardware = ACC_FAKE;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (accHardwareToUse != ACC_AUTODETECT) {
            break;
        }

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

bool accInit(const accelerometerConfig_t *accConfig, uint32_t targetLooptime)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.mpuConfiguration = gyro.dev.mpuConfiguration;
    acc.dev.mpuDetectionResult = gyro.dev.mpuDetectionResult;
    if (!accDetect(&acc.dev, accConfig->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.init(&acc.dev);
    acc.accTargetLooptime = targetLooptime;
    if (accLpfCutHz) {
        for (int axis = 0; axis < 3; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accTargetLooptime);
        }
    }
    return true;
}

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

bool isAccelerationCalibrationComplete(void)
{
    return calibratingA == 0;
}

bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static sensorCalibrationState_t calState;
static bool calibratedAxis[6];
static int32_t accSamples[6][3];
static int  calibratedAxisCount = 0;

int getPrimaryAxisIndex(int32_t sample[3])
{
    if (ABS(sample[Z]) > ABS(sample[X]) && ABS(sample[Z]) > ABS(sample[Y])) {
        //Z-axis
        return (sample[Z] > 0) ? 0 : 1;
    }
    else if (ABS(sample[X]) > ABS(sample[Y]) && ABS(sample[X]) > ABS(sample[Z])) {
        //X-axis
        return (sample[X] > 0) ? 2 : 3;
    }
    else if (ABS(sample[Y]) > ABS(sample[X]) && ABS(sample[Y]) > ABS(sample[Z])) {
        //Y-axis
        return (sample[Y] > 0) ? 4 : 5;
    }
    else
        return -1;
}

void performAcclerationCalibration(void)
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
            accZero->raw[axis] = lrintf(accTmp[axis]);
        }

        /* Not we can offset our accumulated averages samples and calculate scale factors and calculate gains */
        sensorCalibrationResetState(&calState);

        for (int axis = 0; axis < 6; axis++) {
            accSample[X] = accSamples[axis][X] / CALIBRATING_ACC_CYCLES - accZero->raw[X];
            accSample[Y] = accSamples[axis][Y] / CALIBRATING_ACC_CYCLES - accZero->raw[Y];
            accSample[Z] = accSamples[axis][Z] / CALIBRATING_ACC_CYCLES - accZero->raw[Z];

            sensorCalibrationPushSampleForScaleCalculation(&calState, axis / 2, accSample, acc.dev.acc_1G);
        }

        sensorCalibrationSolveForScale(&calState, accTmp);

        for (int axis = 0; axis < 3; axis++) {
            accGain->raw[axis] = lrintf(accTmp[axis] * 4096);
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

void updateAccelerationReadings(void)
{
    if (!acc.dev.read(&acc.dev)) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = lrintf(biquadFilterApply(&accFilter[axis], (float)acc.accADC[axis]));
        }
    }

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration();
    }

    applyAccelerationZero(accZero, accGain);

    alignSensors(acc.accADC, acc.dev.accAlign);
}

void setAccelerationCalibrationValues(flightDynamicsTrims_t * accZeroToUse, flightDynamicsTrims_t * accGainToUse)
{
    accZero = accZeroToUse;
    accGain = accGainToUse;

    if ((accZero->raw[X] == 0) && (accZero->raw[Y] == 0) && (accZero->raw[Z] == 0) &&
        (accGain->raw[X] == 4096) && (accGain->raw[Y] == 4096) &&(accGain->raw[Z] == 4096)) {
        DISABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
    else {
        ENABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
}

void setAccelerationFilter(uint8_t initialAccLpfCutHz)
{
    accLpfCutHz = initialAccLpfCutHz;
    if (acc.accTargetLooptime) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accTargetLooptime);
        }
    }
}

bool isAccelerometerHealthy(void)
{
    return true;
}
