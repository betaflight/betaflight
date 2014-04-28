/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#define L3GD20_SPI          SPI1

#define L3GD20_CS_GPIO      GPIOE
#define L3GD20_CS_PIN       GPIO_Pin_3
#define L3GD20_CS_GPIO_CLK  RCC_AHBPeriph_GPIOE

#define L3GD20_GYRO_SCALE_FACTOR  0.00030543f  // (17.5e-3) * pi/180  (17.5 mdps/bit)

bool l3gd20Detect(gyro_t *gyro, uint16_t lpf);
