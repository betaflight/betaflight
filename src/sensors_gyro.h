#pragma once

extern uint16_t acc_1G;
extern gyro_t gyro;
extern sensor_align_e gyroAlign;

void GYRO_SetCalibrationCycles(uint16_t calibrationCyclesRequired);
void GYRO_Common(void);
void Gyro_getADC(void);

