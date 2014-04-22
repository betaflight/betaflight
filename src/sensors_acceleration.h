#pragma once

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_NONE = 5
} AccelSensors;

extern uint8_t accHardware;
extern sensor_align_e accAlign;
extern acc_t acc;
extern uint16_t calibratingA;

void ACC_SetCalibrationCycles(uint16_t calibrationCyclesRequired);
void ACC_Common(void);
void ACC_getADC(void);

