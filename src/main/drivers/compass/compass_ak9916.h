/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later
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
 * AKM AK09916 magnetometer driver (I2C only).
 *
 * Note: the part number is AK09916; the driver, symbols and settings follow the
 * Betaflight convention of dropping the leading zero (AK09916 -> AK9916).
 *
 * In a Here2/Here3 GPS the AK09916 is the magnetometer of an ICM-20948 and is
 * exposed by the module's own MCU on its I2C bus at address 0x0C, which is what
 * this driver probes. Register map and conversion follow
 * https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass_AK09916.cpp
 */

#pragma once

#include "drivers/io_types.h"

/**
 * @brief Detect an AK09916 on I2C and install init/read callbacks.
 *
 * @param mag Magnetometer device (address 0 probes the default 0x0C).
 * @return true if WIA2 (register 0x01) reported the AK09916 device id 0x09.
 */
bool ak9916Detect(magDev_t *mag);
