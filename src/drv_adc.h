#pragma once

void adcInit(void);
uint16_t adcGetBattery(void);
#ifdef FY90Q
void adcSensorInit(sensor_t *acc, sensor_t *gyro);
#endif
