#pragma once

typedef struct drv_adxl345_config_t {
    bool useFifo;
    uint16_t dataRate;
} drv_adxl345_config_t;

bool adxl345Detect(drv_adxl345_config_t *init, sensor_t *acc);
