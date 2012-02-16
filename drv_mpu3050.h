#pragma once

void mpu3050Init(void);
void mpu3050Read(int16_t *gyroData);
int16_t mpu3050ReadTemp(void);
