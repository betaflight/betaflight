#pragma once

bool mpu6050Detect(acc_t * acc, gyro_t * gyro, uint16_t lpf);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
