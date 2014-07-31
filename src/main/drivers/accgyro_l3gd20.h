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

#define L3GD20_SPI          SPI1

#define L3GD20_CS_GPIO      GPIOE
#define L3GD20_CS_PIN       GPIO_Pin_3
#define L3GD20_CS_GPIO_CLK  RCC_AHBPeriph_GPIOE

bool l3gd20Detect(gyro_t *gyro, uint16_t lpf);
