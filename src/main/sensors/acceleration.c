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

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_fake.h"
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

#ifdef USE_ACC_ADXL345
#include "drivers/accgyro_legacy/accgyro_adxl345.h"
#endif

#ifdef USE_ACC_BMA280
#include "drivers/accgyro_legacy/accgyro_bma280.h"
#endif

#ifdef USE_ACC_LSM303DLHC
#include "drivers/accgyro_legacy/accgyro_lsm303dlhc.h"
#endif

#ifdef USE_ACC_MMA8452
#include "drivers/accgyro_legacy/accgyro_mma845x.h"
#endif

#include "drivers/bus_spi.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "pg/gyrodev.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "acceleration.h"

FAST_RAM_ZERO_INIT acc_t acc;                       // acc access functions

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    RESET_CONFIG_2(rollAndPitchTrims_t, rollAndPitchTrims,
        .values.roll = 0,
        .values.pitch = 0,
    );
}

void accResetRollAndPitchTrims(void)
{
    resetRollAndPitchTrims(&accelerometerConfigMutable()->accelerometerTrims);
}

static void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero)
{
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
}

void accResetFlightDynamicsTrims(void)
{
    resetFlightDynamicsTrims(&accelerometerConfigMutable()->accZero);
}

void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance)
{
    RESET_CONFIG_2(accelerometerConfig_t, instance,
        .acc_lpf_hz = 10,
        .acc_hardware = ACC_DEFAULT,
        .acc_high_fsr = false,
    );
    resetRollAndPitchTrims(&instance->accelerometerTrims);
    resetFlightDynamicsTrims(&instance->accZero);
}

PG_REGISTER_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 1);

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

static float accumulatedMeasurements[XYZ_AXIS_COUNT];
static int accumulatedMeasurementCount;

static uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

static flightDynamicsTrims_t *accelerationTrims;

static uint16_t accLpfCutHz = 0;
static biquadFilter_t accFilter[XYZ_AXIS_COUNT];

bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware = ACC_NONE;

#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif

retry:

    switch (accHardwareToUse) {
    case ACC_DEFAULT:
        FALLTHROUGH;

#ifdef USE_ACC_ADXL345
    case ACC_ADXL345: // ADXL345
        acc_params.useFifo = false;
        acc_params.dataRate = 800; // unused currently
        if (adxl345Detect(&acc_params, dev)) {
            accHardware = ACC_ADXL345;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_LSM303DLHC
    case ACC_LSM303DLHC:
        if (lsm303dlhcAccDetect(dev)) {
            accHardware = ACC_LSM303DLHC;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_MPU6050
    case ACC_MPU6050: // MPU6050
        if (mpu6050AccDetect(dev)) {
            accHardware = ACC_MPU6050;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_MMA8452
    case ACC_MMA8452: // MMA8452
        if (mma8452Detect(dev)) {
            accHardware = ACC_MMA8452;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_BMA280
    case ACC_BMA280: // BMA280
        if (bma280Detect(dev)) {
            accHardware = ACC_BMA280;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_MPU6000
    case ACC_MPU6000:
        if (mpu6000SpiAccDetect(dev)) {
            accHardware = ACC_MPU6000;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_MPU9250
    case ACC_MPU9250:
        if (mpu9250SpiAccDetect(dev)) {
            accHardware = ACC_MPU9250;
            break;
        }
        FALLTHROUGH;
#endif

    case ACC_MPU6500:
    case ACC_ICM20601:
    case ACC_ICM20602:
    case ACC_ICM20608G:
#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
#ifdef USE_ACC_SPI_MPU6500
        if (mpu6500AccDetect(dev) || mpu6500SpiAccDetect(dev)) {
#else
        if (mpu6500AccDetect(dev)) {
#endif
            switch (dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                accHardware = ACC_MPU9250;
                break;
            case ICM_20601_SPI:
                accHardware = ACC_ICM20601;
                break;
            case ICM_20602_SPI:
                accHardware = ACC_ICM20602;
                break;
            case ICM_20608_SPI:
                accHardware = ACC_ICM20608G;
                break;
            default:
                accHardware = ACC_MPU6500;
            }
            break;
        }
#endif
        FALLTHROUGH;

#ifdef USE_ACC_SPI_ICM20649
    case ACC_ICM20649:
        if (icm20649SpiAccDetect(dev)) {
            accHardware = ACC_ICM20649;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_ICM20689
    case ACC_ICM20689:
        if (icm20689SpiAccDetect(dev)) {
            accHardware = ACC_ICM20689;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
    case ACC_BMI160:
        if (bmi160SpiAccDetect(dev)) {
            accHardware = ACC_BMI160;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_FAKE_ACC
    case ACC_FAKE:
        if (fakeAccDetect(dev)) {
            accHardware = ACC_FAKE;
            break;
        }
        FALLTHROUGH;
#endif

    default:
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

bool accInit(uint32_t gyroSamplingInverval)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.bus = *gyroSensorBus();
    acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    acc.dev.acc_high_fsr = accelerometerConfig()->acc_high_fsr;

    // Copy alignment from active gyro, as all production boards use acc-gyro-combi chip.
    // Exceptions are STM32F3DISCOVERY and STM32F411DISCOVERY, and (may be) handled in future enhancement.

    acc.dev.accAlign = gyroDeviceConfig(0)->align;
#ifdef USE_MULTI_GYRO
    if (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_2) {
        acc.dev.accAlign = gyroDeviceConfig(1)->align;
    }
#endif

    if (!accDetect(&acc.dev, accelerometerConfig()->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.initFn(&acc.dev); // driver initialisation
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;
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
        acc.accSamplingInterval = 1000;
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

static void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis] = 0;
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
        for (int axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += acc.accADC[axis];
            // Clear global variables for next reading
            acc.accADC[axis] = 0;
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
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}

void accUpdate(timeUs_t currentTimeUs, rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        DEBUG_SET(DEBUG_ACCELEROMETER, axis, acc.dev.ADCRaw[axis]);
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

    if (accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = biquadFilterApply(&accFilter[axis], acc.accADC[axis]);
        }
    }

    alignSensors(acc.accADC, acc.dev.accAlign);

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration(rollAndPitchTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }

    applyAccelerationTrims(accelerationTrims);

    ++accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accumulatedMeasurements[axis] += acc.accADC[axis];
    }
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementCount;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationTrims = accelerationTrimsToUse;
}

void accInitFilters(void)
{
    accLpfCutHz = accelerometerConfig()->acc_lpf_hz;
    if (acc.accSamplingInterval) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
        }
    }
}

void applyAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfigMutable()->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfigMutable()->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;
}
#endif
