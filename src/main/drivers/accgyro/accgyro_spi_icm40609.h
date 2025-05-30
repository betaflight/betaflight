/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/bus.h"
 
void icm40609AccInit(accDev_t *acc);
void icm40609GyroInit(gyroDev_t *gyro);

uint8_t icm40609SpiDetect(const extDevice_t *dev);

bool icm40609SpiAccDetect(accDev_t *acc);
bool icm40609SpiGyroDetect(gyroDev_t *gyro);
