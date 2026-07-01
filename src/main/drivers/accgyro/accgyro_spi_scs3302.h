
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

#include <stdbool.h>
//#include <stdint.h>

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"

// Discovery functions
uint8_t scs3302SpiDetect(const extDevice_t *dev);
bool scs3302SpiAccDetect(accDev_t *acc);
bool scs3302SpiGyroDetect(gyroDev_t *gyro);
