#pragma once

bool hmc5883lDetect(int8_t *align);
void hmc5883lInit(float *calibrationGain);
void hmc5883lRead(int16_t *magData);
