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
typedef enum {
    ACC_DEFAULT = 0,
    ACC_NONE = 1,
    ACC_ADXL345 = 2,
    ACC_MPU6050 = 3,
    ACC_MMA8452 = 4,
    ACC_BMA280 = 5,
    ACC_LSM303DLHC = 6,
    ACC_MPU6000 = 7,
    ACC_MPU6500 = 8,
    ACC_MPU9250 = 9,
    ACC_FAKE = 10,
    ACC_MAX = ACC_FAKE
} accelerationSensor_e;

extern sensor_align_e accAlign;
extern acc_t acc;

extern int32_t accADC[XYZ_AXIS_COUNT];

void accInit(uint32_t accTargetLooptime);
bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void updateAccelerationReadings(void);
union flightDynamicsTrims_u;
void setAccelerationZero(union flightDynamicsTrims_u * accZeroToUse);
void setAccelerationGain(union flightDynamicsTrims_u * accGainToUse);
void setAccelerationFilter(uint8_t initialAccLpfCutHz);
