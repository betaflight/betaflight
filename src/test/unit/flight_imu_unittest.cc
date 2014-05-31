#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#define BARO


// FIXME this giant list of includes (below) and stubs (bottom) indicates there is too much going on in flight_imu.c and that it needs decoupling and breaking up.

#include "common/axis.h"
#include "flight/flight.h"

#include "sensors/sensors.h"
#include "drivers/accgyro.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"

#include "io/gps.h"

#include "io/gimbal.h"
#include "flight/mixer.h"

// FIXME remove dependency on config.h
#include "sensors/boardalignment.h"
#include "io/battery.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "rx/rx.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "telemetry/telemetry.h"
#include "flight/failsafe.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"


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
profile_t currentProfile;
master_t masterConfig;
int16_t heading;
flags_t f;
gyro_t gyro;
int16_t magADC[XYZ_AXIS_COUNT];
int32_t BaroAlt;

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
