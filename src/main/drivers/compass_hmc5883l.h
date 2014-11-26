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

typedef struct hmc5883Config_s {
    uint32_t gpioAPB2Peripherals;
    uint16_t gpioPin;
    GPIO_TypeDef *gpioPort;
} hmc5883Config_t;

bool hmc5883lDetect(void);
void hmc5883lInit(hmc5883Config_t *hmc5883Config);
void hmc5883lRead(int16_t *magData);
