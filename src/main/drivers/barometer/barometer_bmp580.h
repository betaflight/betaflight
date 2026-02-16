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
 * BMP580 Driver
 *
 * References:
 * BMP580 datasheet - https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp580/
 * BMP5-Sensor-API - https://github.com/boschsensortec/BMP5-Sensor-API
 */

#pragma once

typedef struct bmp580Config_s {
    ioTag_t eocTag;
} bmp580Config_t;

bool bmp580Detect(const bmp580Config_t *config, baroDev_t *baro);
