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
#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sonar_hcsr04.h"
#include "drivers/gpio.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"

// in cm , -1 indicate sonar is not in range - inclination adjusted by imu

#ifdef SONAR

static int32_t calculatedAltitude;

const sonarHardware_t *sonarGetHardwareConfiguration(batteryConfig_t *batteryConfig) 
{
#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R)
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
    // If we are using parallel PWM for our receiver or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_RX_PARALLEL_PWM ) || (feature(FEATURE_CURRENT_METER) && batteryConfig->currentMeterType == CURRENT_SENSOR_ADC) ) {
        return &sonarPWM56;
    } else {
        return &sonarRC78;
    }
#elif defined(OLIMEXINO)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#elif defined(CC3D)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_5,   // (PB5)
        .echo_pin = Pin_0,      // (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line0,
        .exti_pin_source = GPIO_PinSource0,
        .exti_irqn = EXTI0_IRQn
    };
    return &sonarHardware;
#elif defined(SPRACINGF3)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_0,   // RC_CH7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RC_CH8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = EXTI_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#else
#error Sonar not defined for target
#endif
}

void sonarInit(const sonarHardware_t *sonarHardware)
{
    hcsr04_init(sonarHardware);
    sensorsSet(SENSOR_SONAR);
    calculatedAltitude = -1;
}

void sonarUpdate(void)
{
    hcsr04_start_reading();
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, -1 is returned instead.
 */
int32_t sonarRead(void)
{
    return hcsr04_get_distance();
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too strong, -1 is returned instead.
 */
int32_t sonarCalculateAltitude(int32_t sonarAlt, int16_t tiltAngle)
{
    // calculate sonar altitude only if the sonar is facing downwards(<25deg)
    if (tiltAngle > 250)
        calculatedAltitude = -1;
    else
        calculatedAltitude = sonarAlt * (900.0f - tiltAngle) / 900.0f;

    return calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or -1 if sonarCalculateAltitude
 * has never been called.
 */
int32_t sonarGetLatestAltitude(void)
{
    return calculatedAltitude;
}

#endif
