#pragma once

bool hmc5883lDetect(void);
void hmc5883lInit(void);
void hmc5883lFinishCal(void);
void hmc5883lRead(int16_t *magData);
