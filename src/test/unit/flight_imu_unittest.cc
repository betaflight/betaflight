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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#define BARO

#include "common/axis.h"
#include "flight/flight.h"

#include "sensors/sensors.h"
#include "drivers/accgyro.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"

#include "config/runtime_config.h"

#include "flight/mixer.h"
#include "flight/imu.h"

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DOWNWARDS_THRUST true
#define UPWARDS_THRUST false


bool isThrustFacingDownwards(rollAndPitchInclination_t *inclinations);

typedef struct inclinationExpectation_s {
    rollAndPitchInclination_t inclination;
    bool expectDownwardsThrust;
} inclinationExpectation_t;

TEST(FlightImuTest, IsThrustFacingDownwards)
{
    // given

    inclinationExpectation_t inclinationExpectations[] = {
            { { 0, 0 }, DOWNWARDS_THRUST },
            { { 799, 799 }, DOWNWARDS_THRUST },
            { { 800, 799 }, UPWARDS_THRUST },
            { { 799, 800 }, UPWARDS_THRUST },
            { { 800, 800 }, UPWARDS_THRUST },
            { { 801, 801 }, UPWARDS_THRUST },
            { { -799, -799 }, DOWNWARDS_THRUST },
            { { -800, -799 }, UPWARDS_THRUST },
            { { -799, -800 }, UPWARDS_THRUST },
            { { -800, -800 }, UPWARDS_THRUST },
            { { -801, -801 }, UPWARDS_THRUST }
    };
    uint8_t testIterationCount = sizeof(inclinationExpectations) / sizeof(inclinationExpectation_t);

    // expect

    for (uint8_t index = 0; index < testIterationCount; index ++) {
        inclinationExpectation_t *angleInclinationExpectation = &inclinationExpectations[index];
        printf("iteration: %d\n", index);
        bool result = isThrustFacingDownwards(&angleInclinationExpectation->inclination);
        EXPECT_EQ(angleInclinationExpectation->expectDownwardsThrust, result);
    }
}

// STUBS

uint16_t acc_1G;
int16_t heading;
flags_t f;
gyro_t gyro;
int16_t magADC[XYZ_AXIS_COUNT];
int32_t BaroAlt;
int16_t debug[4];


void gyroGetADC(void) {};
bool sensors(uint32_t mask)
{
    UNUSED(mask);
    return false;
};
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(rollAndPitchTrims);
}

uint32_t micros(void) { return 0; }
bool isBaroCalibrationComplete(void) { return true; }
void performBaroCalibrationCycle(void) {}
int32_t baroCalculateAltitude(void) { return 0; }
int constrain(int amt, int low, int high)
{
    UNUSED(amt);
    UNUSED(low);
    UNUSED(high);
    return 0;
}
