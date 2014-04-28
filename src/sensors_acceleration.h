#pragma once

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_LSM303DLHC = 5,
    ACC_FAKE = 6,
    ACC_NONE = 7
} AccelSensors;

extern uint8_t accHardware;
extern sensor_align_e accAlign;
extern acc_t acc;
extern uint16_t acc_1G;

bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims);
void setAccelerationTrims(int16_flightDynamicsTrims_t *accelerationTrimsToUse);
