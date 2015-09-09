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

typedef struct bmp085Config_s {
    uint32_t gpioAPB2Peripherals;
    uint16_t xclrGpioPin;
    GPIO_TypeDef *xclrGpioPort;
    uint16_t eocGpioPin;
    GPIO_TypeDef *eocGpioPort;
} bmp085Config_t;

bool bmp085Detect(const bmp085Config_t *config, baro_t *baro);
void bmp085Disable(const bmp085Config_t *config);

#if defined(BARO_EOC_GPIO)
bool bmp085TestEOCConnected(const bmp085Config_t *config);
#endif

#ifdef UNIT_TEST
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
#endif
