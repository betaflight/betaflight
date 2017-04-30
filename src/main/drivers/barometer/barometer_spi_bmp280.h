/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

void bmp280SpiInit(void);
bool bmp280ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
bool bmp280WriteRegister(uint8_t reg, uint8_t data);
void bmp280_spi_start_up(void);
void bmp280_spi_get_up(void);
