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

typedef struct extiConfig_s {
#ifdef STM32F303
    uint32_t gpioAHBPeripherals;
#endif
#ifdef STM32F10X
    uint32_t gpioAPB2Peripherals;
#endif
    uint16_t gpioPin;
    GPIO_TypeDef *gpioPort;

    uint8_t exti_port_source;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
} extiConfig_t;
