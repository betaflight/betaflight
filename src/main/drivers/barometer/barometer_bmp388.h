/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * BMP388 Driver author: Dominic Clifton
 *
 * References:
 * BMP388 datasheet - https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP388-DS001.pdf
 * BMP3-Sensor-API - https://github.com/BoschSensortec/BMP3-Sensor-API
 * BMP280 Cleanflight driver
 */

#pragma once

typedef struct bmp388Config_s {
    ioTag_t eocTag;
} bmp388Config_t;

bool bmp388Detect(const bmp388Config_t *config, baroDev_t *baro);
