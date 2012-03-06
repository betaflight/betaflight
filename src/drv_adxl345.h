#pragma once

bool adxl345Detect(void);
void adxl345Init(void);
void adxl345Read(int16_t *accelData);
