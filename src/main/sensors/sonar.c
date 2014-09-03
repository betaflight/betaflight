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

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sonar_hcsr04.h"
#include "drivers/gpio.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "flight/flight.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"

int32_t sonarAlt = -1;	// in cm , -1 indicate sonar is not in range

#ifdef SONAR

void Sonar_init(void)
{
#if defined(NAZE) || defined(EUSTM32F103RC)
    static const sonarHardware_t const sonarPWM56 = {
        .trigger_pin = Pin_8,   // PWM5 (PB8) - 5v tolerant
        .echo_pin = Pin_9,      // PWM6 (PB9) - 5v tolerant
        .exti_line = EXTI_Line9,
        .exti_pin_source = GPIO_PinSource9,
        .exti_irqn = EXTI9_5_IRQn
    };
    static const sonarHardware_t const sonarRC78 = {
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    // If we are using parallel PWM for our receiver, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_RX_PARALLEL_PWM)) {
        hcsr04_init(&sonarPWM56);
    } else {
        hcsr04_init(&sonarRC78);
    }
#elif defined(OLIMEXINO)
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    hcsr04_init(&sonarHardware);
#else
#error Sonar not defined for target
#endif

    sensorsSet(SENSOR_SONAR);
    sonarAlt = 0;
}

void Sonar_update(void)
{
    hcsr04_get_distance(&sonarAlt);
}

int32_t sonarCalculateAltitude(int32_t sonarAlt, int16_t tiltAngle)
{
    // calculate sonar altitude only if the sonar is facing downwards(<25deg)
    if (tiltAngle > 250)
        return -1;

    return sonarAlt * (900.0f - tiltAngle) / 900.0f;
}

#endif
