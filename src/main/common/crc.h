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

#include <stdint.h>

struct sbuf_s;

uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length);
void crc16_ccitt_sbuf_append(struct sbuf_s *dst, uint8_t *start);

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);
void crc8_dvb_s2_sbuf_append(struct sbuf_s *dst, uint8_t *start);
