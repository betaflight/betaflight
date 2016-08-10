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
#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/feature.h"

#include "drivers/sonar_hcsr04.h"
#include "drivers/gpio.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/config.h"

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

const sonarHardware_t *sonarGetHardwareConfiguration(currentSensor_e  currentMeterType)
{
#if defined(SONAR_PWM_TRIGGER_PIN)
    static const sonarHardware_t const sonarPWM = {
        .trigger_pin = SONAR_PWM_TRIGGER_PIN,
        .trigger_gpio = SONAR_PWM_TRIGGER_GPIO,
        .echo_pin = SONAR_PWM_ECHO_PIN,
        .echo_gpio = SONAR_PWM_ECHO_GPIO,
        .triggerIO = IO_TAG(SONAR_PWM_TRIGGER_IO),
        .echoIO = IO_TAG(SONAR_PWM_ECHO_IO),
    };
#endif
#if !defined(UNIT_TEST)
    static const sonarHardware_t sonarRC = {
        .trigger_pin = SONAR_TRIGGER_PIN,
        .trigger_gpio = SONAR_TRIGGER_GPIO,
        .echo_pin = SONAR_ECHO_PIN,
        .echo_gpio = SONAR_ECHO_GPIO,
        .triggerIO = IO_TAG(SONAR_TRIGGER_IO),
        .echoIO = IO_TAG(SONAR_ECHO_IO),
    };
#endif
#if defined(SONAR_PWM_TRIGGER_PIN)
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use RC pins 7 and 8
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && currentMeterType == CURRENT_SENSOR_ADC)) {
        return &sonarPWM;
    } else {
        return &sonarRC;
    }
#elif defined(SONAR_TRIGGER_PIN)
    UNUSED(currentMeterType);
    return &sonarRC;
#elif defined(UNIT_TEST)
    UNUSED(currentMeterType);
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
