#pragma once

extern gyro_t gyro;
extern sensor_align_e gyroAlign;

typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
} gyroConfig_t;

void useGyroConfig(gyroConfig_t *gyroConfigToUse);
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void gyroGetADC(void);
bool isGyroCalibrationComplete(void);

