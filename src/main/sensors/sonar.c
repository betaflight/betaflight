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
#include <math.h>

#include <platform.h>
#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sonar_hcsr04.h"
#include "drivers/gpio.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"

// Sonar measurements are in cm, a value of SONAR_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu
    float baro_cf_vel;                      // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    float baro_cf_alt;                      // apply CF to use ACC for height estimation

#ifdef SONAR
int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
float sonarMaxTiltCos;

static int32_t calculatedAltitude;

const sonarHardware_t *sonarGetHardwareConfiguration(batteryConfig_t *batteryConfig)
{
#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R) || defined(PORT103V)
    static const sonarHardware_t const sonarPWM56 = {
        .trigger_pin = Pin_8,   // PWM5 (PB8) - 5v tolerant
        .trigger_gpio = GPIOB,
        .echo_pin = Pin_9,      // PWM6 (PB9) - 5v tolerant
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line9,
        .exti_pin_source = GPIO_PinSource9,
        .exti_irqn = EXTI9_5_IRQn
    };
    static const sonarHardware_t sonarRC78 = {
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .trigger_gpio = GPIOB,
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && batteryConfig->currentMeterType == CURRENT_SENSOR_ADC)) {
        return &sonarPWM56;
    } else {
        return &sonarRC78;
    }
#elif defined(OLIMEXINO)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .trigger_gpio = GPIOB,
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#elif defined(CC3D)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_5,   // (PB5)
        .trigger_gpio = GPIOB,
        .echo_pin = Pin_0,      // (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line0,
        .exti_pin_source = GPIO_PinSource0,
        .exti_irqn = EXTI0_IRQn
    };
    return &sonarHardware;
#elif defined(SPRACINGF3) || defined(SPRACINGF3MINI)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_0,   // RC_CH7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .trigger_gpio = GPIOB,
        .echo_pin = Pin_1,      // RC_CH8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line1,
        .exti_pin_source = EXTI_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#elif defined(SPARKY)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_2,   // PWM6 (PA2) - only 3.3v ( add a 1K Ohms resistor )
        .trigger_gpio = GPIOA,
        .echo_pin = Pin_1,      // PWM7 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line1,
        .exti_pin_source = EXTI_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#elif defined(UNIT_TEST)
    UNUSED(batteryConfig);
    return 0;
#else
#error Sonar not defined for target
#endif
}

void sonarInit(const sonarHardware_t *sonarHardware)
{
    sonarRange_t sonarRange;

    hcsr04_init(sonarHardware, &sonarRange);
    sensorsSet(SENSOR_SONAR);
    sonarMaxRangeCm = sonarRange.maxRangeCm;
    sonarCfAltCm = sonarMaxRangeCm / 2;
    sonarMaxTiltDeciDegrees =  sonarRange.detectionConeExtendedDeciDegrees / 2;
    sonarMaxTiltCos = cos_approx(sonarMaxTiltDeciDegrees / 10.0f * RAD);
    sonarMaxAltWithTiltCm = sonarMaxRangeCm * sonarMaxTiltCos;
    calculatedAltitude = SONAR_OUT_OF_RANGE;
}

#define DISTANCE_SAMPLES_MEDIAN 5

static int32_t applySonarMedianFilter(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    if (newSonarReading > SONAR_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(sonarFilterSamples);
    else
        return newSonarReading;
}

void sonarUpdate(void)
{
    hcsr04_start_reading();
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarRead(void)
{
    int32_t distance = hcsr04_get_distance();
    if (distance > HCSR04_MAX_RANGE_CM)
        distance = SONAR_OUT_OF_RANGE;

    return applySonarMedianFilter(distance);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarCalculateAltitude(int32_t sonarDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the sonar cone
    if (cosTiltAngle <= sonarMaxTiltCos)
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
        calculatedAltitude = sonarDistance * cosTiltAngle;
    return calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or SONAR_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t sonarGetLatestAltitude(void)
{
    return calculatedAltitude;
}

#endif
