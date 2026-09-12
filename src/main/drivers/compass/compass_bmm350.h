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
 *
 * BMM350 magnetometer driver (I2C only).
 *
 * Magnetic reset (BR/FGR) requires a 2.2 uF capacitor on CRST. Without it,
 * heading accuracy is degraded. Addresses 0x14 (ADSEL=GND) and 0x15 (ADSEL=VDDIO).
 */

#pragma once

#include "drivers/compass/compass.h"

/**
 * @brief Detect a BMM350 on I2C and install init/read callbacks.
 *
 * @param magDev Magnetometer device (address 0 probes 0x14 then 0x15).
 * @return true if CHIP_ID 0x33 was read.
 */
bool bmm350Detect(magDev_t *magDev);
