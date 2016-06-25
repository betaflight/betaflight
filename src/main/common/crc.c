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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "common/crc.h"

uint16_t crc16_CCITT(uint16_t crc, uint8_t value)
{
    uint8_t i;

    crc = crc ^ (int16_t)value << 8;

    for (i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = crc << 1 ^ 0x1021;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}
