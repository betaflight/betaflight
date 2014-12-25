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

#pragma once

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_LSM303DLHC = 5,
    ACC_SPI_MPU6000 = 6,
    ACC_SPI_MPU6500 = 7,
    ACC_FAKE = 8,
    ACC_NONE = 9
} accelSensor_e;

extern uint8_t accHardware;
extern sensor_align_e accAlign;
extern acc_t acc;
extern uint16_t acc_1G;

typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;

bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims);
void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse);
