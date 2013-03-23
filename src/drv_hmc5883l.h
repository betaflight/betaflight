#pragma once

bool hmc5883lDetect(void);
void hmc5883lInit(float *calibrationGain);
void hmc5883lRead(int16_t *magData);
