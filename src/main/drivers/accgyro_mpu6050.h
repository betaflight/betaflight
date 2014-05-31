#pragma once

bool mpu6050AccDetect(acc_t *acc);
bool mpu6050GyroDetect(gyro_t *gyro, uint16_t lpf);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
