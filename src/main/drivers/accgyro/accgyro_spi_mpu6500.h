/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/sensor.h"

uint8_t mpu6500SpiDetect(const busDevice_t *bus);

bool mpu6500SpiAccDetect(accDev_t *acc);
bool mpu6500SpiGyroDetect(gyroDev_t *gyro);

bool mpu6500SpiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool mpu6500SpiReadRegister(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t *data);

void mpu6500SpiGyroInit(gyroDev_t *gyro);
void mpu6500SpiAccInit(accDev_t *acc);
