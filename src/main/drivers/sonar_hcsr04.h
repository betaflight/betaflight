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

typedef struct sonarHardware_s {
    uint16_t trigger_pin;
    uint16_t echo_pin;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
} sonarHardware_t;

#define SONAR_GPIO GPIOB

void hcsr04_init(const sonarHardware_t *sonarHardware);

void hcsr04_start_reading(void);
int32_t hcsr04_get_distance(void);
