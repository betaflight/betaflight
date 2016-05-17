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

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/gpio.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sonar_srf10.h"
#include "drivers/sonar.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/battery.h"

// Sonar measurements are in cm, a value of SONAR_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu

#ifdef SONAR
int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
static sonarFunctionPointers_t sonarFunctionPointers;
float sonarMaxTiltCos;

static int32_t calculatedAltitude;

static const sonarHardware_t *sonarGetHardwareConfigurationForHCSR04(currentSensor_e currentSensor)
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
            || (feature(FEATURE_CURRENT_METER) && currentSensor == CURRENT_SENSOR_ADC) ) {
        return &sonarPWM56;
    } else {
        return &sonarRC78;
    }
#elif defined(OLIMEXINO)
    UNUSED(currentSensor);
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
    UNUSED(currentSensor);
    static const sonarHardware_t const sonarHardware = {
        .trigger_pin = Pin_5,   // RX4 (PB5)
        .trigger_gpio = GPIOB,
        .echo_pin = Pin_0,      // RX5 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_gpio = GPIOB,
        .exti_line = EXTI_Line0,
        .exti_pin_source = GPIO_PinSource0,
        .exti_irqn = EXTI0_IRQn
    };
    return &sonarHardware;
#elif defined(SPRACINGF3) || defined(SPRACINGF3MINI)
    UNUSED(currentSensor);
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
    UNUSED(currentSensor);
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
   UNUSED(currentSensor);
   return 0;
#else
#error Sonar not defined for target
#endif
}

STATIC_UNIT_TESTED void sonarSetFunctionPointers(sonarHardwareType_e sonarHardwareType)
{

    switch (sonarHardwareType) {
    case SONAR_NONE:
        break;
    case SONAR_HCSR04:
        sonarFunctionPointers.init = hcsr04_init;
        sonarFunctionPointers.update = hcsr04_start_reading;
        sonarFunctionPointers.read = hcsr04_get_distance;
        break;
    case SONAR_SRF10:
        sonarFunctionPointers.init = srf10_init;
        sonarFunctionPointers.update = srf10_start_reading;
        sonarFunctionPointers.read = srf10_get_distance;
        break;
    }
}

/*
 * Get the HCSR04 sonar hardware configuration.
 * NOTE: sonarInit() must be subsequently called before using any of the sonar functions.
 */
const sonarHardware_t *sonarGetHardwareConfiguration(currentSensor_e currentSensor)
{
    // Return the configuration for the HC-SR04 hardware.
    // Unfortunately the I2C bus is not initialised at this point
    // so cannot detect if another sonar device is present
    return sonarGetHardwareConfigurationForHCSR04(currentSensor);
}

/*
 * Detect what sonar hardware is present and set the function pointers accordingly
 */
static sonarHardwareType_e sonarDetect(void)
{
    sonarHardwareType_e sonarHardwareType;
    if (srf10_detect()) {
        sonarHardwareType = SONAR_SRF10;
    } else {
        // the user has set the sonar feature, so assume they have an HC-SR04 plugged in,
        // since there is no way to detect it
        sonarHardwareType = SONAR_HCSR04;
    }
    sensorsSet(SENSOR_SONAR);
#ifndef UNIT_TEST
    sonarSetFunctionPointers(sonarHardwareType);
#endif
    return sonarHardwareType;
}

void sonarInit(const sonarHardware_t *sonarHardware)
{
    calculatedAltitude = SONAR_OUT_OF_RANGE;
    const sonarHardwareType_e sonarHardwareType = sonarDetect();
    if (sonarHardwareType == SONAR_HCSR04) {
        hcsr04_set_sonar_hardware(sonarHardware);
    }

    sonarRange_t sonarRange;
    sonarFunctionPointers.init(&sonarRange);
    sonarMaxRangeCm = sonarRange.maxRangeCm;
    sonarCfAltCm = sonarMaxRangeCm / 2;
    sonarMaxTiltDeciDegrees =  sonarRange.detectionConeExtendedDeciDegrees / 2;
    sonarMaxTiltCos = cos_approx(sonarMaxTiltDeciDegrees / 10.0f * RAD);
    sonarMaxAltWithTiltCm = sonarMaxRangeCm * sonarMaxTiltCos;
}


static int32_t applySonarMedianFilter(int32_t newSonarReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 5
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newSonarReading > SONAR_OUT_OF_RANGE) {// only accept samples that are in range
        sonarFilterSamples[filterSampleIndex] = newSonarReading;
        ++filterSampleIndex;
        if (filterSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            filterSampleIndex = 0;
            medianFilterReady = true;
        }
    }
    return medianFilterReady ? quickMedianFilter5(sonarFilterSamples) : newSonarReading;
}

/*
 * This is called periodically by the scheduler
 */
void sonarUpdate(void)
{
    if (sonarFunctionPointers.update) {
        sonarFunctionPointers.update();
    }
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarRead(void)
{
    if (sonarFunctionPointers.read) {
        const int32_t distance = sonarFunctionPointers.read();
        return applySonarMedianFilter(distance);
    }
    return 0;
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
    if (cosTiltAngle < sonarMaxTiltCos)
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    else if (sonarDistance == SONAR_OUT_OF_RANGE)
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    else
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

