#pragma once

extern uint16_t acc_1G;
extern gyro_t gyro;
extern sensor_align_e gyroAlign;

void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void gyroGetADC(uint8_t gyroMovementCalibrationThreshold);

