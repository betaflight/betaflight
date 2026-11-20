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
 * BMP5xx Driver (supports BMP580, BMP581)
 *
 * References:
 * BMP580 datasheet - https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp580/
 * BMP5-Sensor-API - https://github.com/boschsensortec/BMP5-Sensor-API
 */

#pragma once

typedef struct bmp5xxConfig_s {
    ioTag_t eocTag;
} bmp5xxConfig_t;

/**
 * @brief Detect and initialize BMP580/BMP581 barometer
 * @param config Pointer to BMP5xx configuration (EOC pin tag)
 * @param baro Pointer to barometer device structure to initialize
 * @param detectedChip Pointer to store the raw family ASIC ID (0x50 or 0x51), or NULL if not needed
 * @return true if BMP580/BMP581 detected and initialized, false otherwise
 */
bool bmp5xxDetect(const bmp5xxConfig_t *config, baroDev_t *baro, uint8_t *detectedChip);

typedef enum {
    BMP5XX_READ_ERROR = -1,
    BMP5XX_READ_PENDING = 0,
    BMP5XX_READ_READY = 1,
} bmp5xxReadResult_e;

// Read a fresh compensated sample from an initialized device. Output values
// are unchanged when data is pending or a register transaction fails.
bmp5xxReadResult_e bmp5xxReadSample(baroDev_t *baro, int32_t *pressure, int32_t *temperature);
