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

#include "platform.h"

#ifdef USE_ALTIMETER

#include "build/build_config.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/altimeter.h"

// Sonar measurements are in cm, a value of ALTIMETER_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu

int16_t altimeterMaxRangeCm;
int16_t altimeterMaxAltWithTiltCm;
int16_t altimeterCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t altimeterMaxTiltDeciDegrees;
float altimeterMaxTiltCos;

static int32_t calculatedAltitude;

PG_REGISTER_WITH_RESET_TEMPLATE(sonarConfig_t, sonarConfig, PG_SONAR_CONFIG, 0);

#ifndef SONAR_TRIGGER_PIN
#define SONAR_TRIGGER_PIN NONE
#endif
#ifndef SONAR_ECHO_PIN
#define SONAR_ECHO_PIN    NONE
#endif

PG_RESET_TEMPLATE(sonarConfig_t, sonarConfig,
    .triggerTag = IO_TAG(SONAR_TRIGGER_PIN),
    .echoTag = IO_TAG(SONAR_ECHO_PIN)
);

const altimeterDevice_t *altimeter = NULL;

#ifdef USE_SONAR
static bool sonarDetect(void)
{
    if (feature(FEATURE_SONAR)) {
        // the user has set the sonar feature, so assume they have an HC-SR04 plugged in,
        // since there is no way to detect it
        sensorsSet(SENSOR_ALTIMETER);
        return true;
    }
    return false;
}
#endif

void altimeterInit(void)
{
#ifdef USE_SONAR
    if (sonarDetect()) {
        altimeter = hcsr04_init(sonarConfig());
    }
#endif

    if (!altimeter) {
        return;
    }

    altimeterMaxRangeCm = altimeter->range->maxRangeCm;
    altimeterCfAltCm = altimeterMaxRangeCm / 2;
    altimeterMaxTiltDeciDegrees =  altimeter->range->detectionConeExtendedDeciDegrees / 2;
    altimeterMaxTiltCos = cos_approx(altimeterMaxTiltDeciDegrees / 10.0f * RAD);
    altimeterMaxAltWithTiltCm = altimeterMaxRangeCm * altimeterMaxTiltCos;
    calculatedAltitude = ALTIMETER_OUT_OF_RANGE;
}

#define DISTANCE_SAMPLES_MEDIAN 5

static int32_t applySonarMedianFilter(int32_t newAltimeterReading)
{
    static int32_t altimeterFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    if (newAltimeterReading > ALTIMETER_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        altimeterFilterSamples[currentFilterSampleIndex] = newAltimeterReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(altimeterFilterSamples);
    else
        return newAltimeterReading;
}

void altimeterUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (altimeter) {
        altimeter->vTable->startReading();
    }
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, ALTIMETER_OUT_OF_RANGE is returned.
 */
int32_t altimeterRead(void)
{
    int32_t distance = ALTIMETER_OUT_OF_RANGE;

    if (altimeter) {
        distance = altimeter->vTable->getDistance();
    }

    return applySonarMedianFilter(distance);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, ALTIMETER_OUT_OF_RANGE is returned.
 */
int32_t altimeterCalculateAltitude(int32_t altimeterDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the sonar cone
    if (cosTiltAngle <= altimeterMaxTiltCos)
        calculatedAltitude = ALTIMETER_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
        calculatedAltitude = altimeterDistance * cosTiltAngle;
    return calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or ALTIMETER_OUT_OF_RANGE if altimeterCalculateAltitude
 * has never been called.
 */
int32_t altimeterGetLatestAltitude(void)
{
    return calculatedAltitude;
}

#endif
