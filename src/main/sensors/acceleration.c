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
#include "drivers/system.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif


acc_t acc;                       // acc access functions

static uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

static flightDynamicsTrims_t *accelerationTrims;

static uint16_t accLpfCutHz = 0;
static biquadFilter_t accFilter[XYZ_AXIS_COUNT];

bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware;

#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif

retry:
    dev->accAlign = ALIGN_DEFAULT;

    switch (accHardwareToUse) {
    case ACC_DEFAULT:
        ; // fallthrough
    case ACC_ADXL345: // ADXL345
#ifdef USE_ACC_ADXL345
        acc_params.useFifo = false;
        acc_params.dataRate = 800; // unused currently
#ifdef NAZE
        if (hardwareRevision < NAZE32_REV5 && adxl345Detect(&acc_params, dev)) {
#else
        if (adxl345Detect(&acc_params, dev)) {
#endif
#ifdef ACC_ADXL345_ALIGN
            dev->accAlign = ACC_ADXL345_ALIGN;
#endif
            accHardware = ACC_ADXL345;
            break;
        }
#endif
        ; // fallthrough
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
        ; // fallthrough
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
        ; // fallthrough
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
        ; // fallthrough
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
        ; // fallthrough
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
        ; // fallthrough
    case ACC_MPU6500:
    case ACC_ICM20608G:
    case ACC_ICM20602:
    case ACC_MPU9250:
#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
#ifdef USE_ACC_SPI_MPU6500
        if (mpu6500AccDetect(dev) || mpu6500SpiAccDetect(dev))
#else
        if (mpu6500AccDetect(dev))
#endif
        {
#ifdef ACC_MPU6500_ALIGN
            dev->accAlign = ACC_MPU6500_ALIGN;
#endif
            switch(dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                accHardware = ACC_MPU9250;
                break;
            case ICM_20608_SPI:
                accHardware = ACC_ICM20608G;
                break;
            case ICM_20602_SPI:
                accHardware = ACC_ICM20602;
                break;
            default:        
                accHardware = ACC_MPU6500;
            }            
            break;
        }
#endif
        ; // fallthrough
    case ACC_ICM20689:
#ifdef USE_ACC_SPI_ICM20689

        if (icm20689SpiAccDetect(dev))
        {
#ifdef ACC_ICM20689_ALIGN
            dev->accAlign = ACC_ICM20689_ALIGN;
#endif
            accHardware = ACC_ICM20689;
            break;
        }
#endif
        ; // fallthrough
    case ACC_FAKE:
#ifdef USE_FAKE_ACC
        if (fakeAccDetect(dev)) {
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
        return false;
    }

    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
    return true;
}

bool accInit(const accelerometerConfig_t *accelerometerConfig, uint32_t gyroSamplingInverval)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.mpuConfiguration = gyro.dev.mpuConfiguration;
    acc.dev.mpuDetectionResult = gyro.dev.mpuDetectionResult;
    if (!accDetect(&acc.dev, accelerometerConfig->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.init(&acc.dev); // driver initialisation
    // set the acc sampling interval according to the gyro sampling interval
    switch (gyroSamplingInverval) {  // Switch statement kept in place to change acc sampling interval in the future
    case 500:
    case 375:
    case 250:
    case 125:
        acc.accSamplingInterval = 1000;
        break;
    case 1000:
    default:
#ifdef STM32F10X
        acc.accSamplingInterval = 1000;
#else
        acc.accSamplingInterval = 1000;
#endif
    }
    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
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

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->values.roll = 0;
    rollAndPitchTrims->values.pitch = 0;
}

static void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle())
            a[axis] = 0;

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accSmooth[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accSmooth[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveConfigAndNotify();
    }

    calibratingA--;
}

static void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    uint8_t axis;
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static rollAndPitchTrims_t angleTrim_saved = { { 0, 0 } };

    // Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[X] = accelerationTrims->raw[X];
        accZero_saved[Y] = accelerationTrims->raw[Y];
        accZero_saved[Z] = accelerationTrims->raw[Z];
        angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
        angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += acc.accSmooth[axis];
            // Clear global variables for next reading
            acc.accSmooth[axis] = 0;
            accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
            // recover saved values to maintain current flight behaviour until new values are transferred
            accelerationTrims->raw[X] = accZero_saved[X];
            accelerationTrims->raw[Y] = accZero_saved[Y];
            accelerationTrims->raw[Z] = accZero_saved[Z];
            rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
            rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationTrims->raw[X] = b[X] / 50;
        accelerationTrims->raw[Y] = b[Y] / 50;
        accelerationTrims->raw[Z] = b[Z] / 50 - acc.dev.acc_1G;    // for nunchuck 200=1G

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveConfigAndNotify();
    }
}

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accSmooth[X] -= accelerationTrims->raw[X];
    acc.accSmooth[Y] -= accelerationTrims->raw[Y];
    acc.accSmooth[Z] -= accelerationTrims->raw[Z];
}

void accUpdate(rollAndPitchTrims_t *rollAndPitchTrims)
{
    if (!acc.dev.read(&acc.dev)) {
        return;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        DEBUG_SET(DEBUG_ACCELEROMETER, axis, acc.dev.ADCRaw[axis]);
        acc.accSmooth[axis] = acc.dev.ADCRaw[axis];
    }

    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accSmooth[axis] = lrintf(biquadFilterApply(&accFilter[axis], (float)acc.accSmooth[axis]));
        }
    }

    alignSensors(acc.accSmooth, acc.dev.accAlign);

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration(rollAndPitchTrims);
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }

    applyAccelerationTrims(accelerationTrims);
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationTrims = accelerationTrimsToUse;
}

void setAccelerationFilter(uint16_t initialAccLpfCutHz)
{
    accLpfCutHz = initialAccLpfCutHz;
    if (acc.accSamplingInterval) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
        }
    }
}
