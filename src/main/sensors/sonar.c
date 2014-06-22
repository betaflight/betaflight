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
#include "config/runtime_config.h"

#include "flight/flight.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"

int32_t sonarAlt = -1;	// in cm , -1 indicate sonar is not in range

#ifdef SONAR

void Sonar_init(void)
{
    hcsr04_init(sonar_rc78);
    sensorsSet(SENSOR_SONAR);
    sonarAlt = 0;
}

void Sonar_update(void)
{
    hcsr04_get_distance(&sonarAlt);
}

int16_t sonarCalculateAltitude(int32_t sonarAlt, rollAndPitchInclination_t *angle)
{
	// calculate sonar altitude
	int16_t sonarAnglecorrection = max(abs(angle->values.rollDeciDegrees), abs(angle->values.pitchDeciDegrees));
	if (sonarAnglecorrection > 250)
		return -1;

	return sonarAlt * (900.0f - sonarAnglecorrection) / 900.0f;
}


#endif
