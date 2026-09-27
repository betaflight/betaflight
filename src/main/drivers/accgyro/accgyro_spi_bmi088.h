/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 */

#pragma once

#include "drivers/accgyro/accgyro.h"

#define BMI088_ACC_CHIP_ID 0x1E
#define BMI088_GYRO_CHIP_ID 0x0F

uint8_t bmi088GyroDetect(const extDevice_t *dev);
bool bmi088SpiGyroDetect(gyroDev_t *gyro);
bool bmi088SpiAccDetect(accDev_t *acc);
