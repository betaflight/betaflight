#pragma once

bool mpu6050Detect(sensor_t *acc, sensor_t *gyro);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
