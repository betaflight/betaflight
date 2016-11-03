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

#include <limits.h>

#define SERIAL_PORT_COUNT   1

extern "C" {
    #include "debug.h"

    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/serial.h"

    //#include "drivers/pwm_rx.h"
    typedef enum {
        INPUT_FILTERING_DISABLED = 0,
        INPUT_FILTERING_ENABLED
    } inputFilteringMode_e;


    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/gyro.h"

    #include "sensors/battery.h"
    #include "sensors/boardalignment.h"

    #include "io/escservo.h"
    #include "io/rc_controls.h"
    #include "io/serial.h"

    #include "telemetry/telemetry.h"

    #include "rx/rx.h"

    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/failsafe.h"
    #include "flight/gps_conversion.h"
    #include "flight/navigation_rewrite.h"

    #include "config/runtime_config.h"
    #include "config/config.h"
    #include "config/config_profile.h"
    #include "config/config_master.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct coordConversionExpectation_s {
    gpsLocation_t llh;
    t_fp_vector pos;
} coordConversionExpectation_t;

TEST(NavigationTest, CoordinateConversion)
{
    // given
    coordConversionExpectation_t testExpectations[] = {
        { {505498090, 1370165690, 10},      {0.0f, 0.0f, 0.0f} },    // this would be origin
        { {505498100, 1370165700, 20},      {11.131949f, 7.0733223f, 10.0f} },
        { {505498080, 1370165680, 0},       {-11.131949f, -7.0733223f, -10.0f} },
    };

    uint8_t testIterationCount = sizeof(testExpectations) / sizeof(testExpectations[0]);
    gpsOrigin_s origin;
    origin.valid = false;

    // expect
    for (uint8_t index = 0; index < testIterationCount; index ++) {
        coordConversionExpectation_t * testExpectation = &testExpectations[index];

        t_fp_vector pos;
        gpsConvertGeodeticToLocal(&origin, &testExpectation->llh, &pos);

        EXPECT_FLOAT_EQ(testExpectation->pos.V.X, pos.V.X);
        EXPECT_FLOAT_EQ(testExpectation->pos.V.Y, pos.V.Y);
        EXPECT_FLOAT_EQ(testExpectation->pos.V.Z, pos.V.Z);

        gpsLocation_t llh;
        gpsConvertLocalToGeodetic(&origin, &testExpectation->pos, &llh);
        
        EXPECT_EQ(testExpectation->llh.lat, llh.lat);
        EXPECT_EQ(testExpectation->llh.lon, llh.lon);
        EXPECT_EQ(testExpectation->llh.alt, llh.alt);
    }

}




// STUBS
extern "C" {

master_t masterConfig;

uint32_t rcModeActivationMask;
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

uint32_t accTimeSum ;        // keep track for integration of acc
int accSumCount;
float accVelScale;

uint16_t acc_1G;
int16_t heading;
gyro_t gyro;

int32_t accSum[XYZ_AXIS_COUNT];
int16_t accADC[XYZ_AXIS_COUNT];
int16_t gyroADC[XYZ_AXIS_COUNT];
int16_t magADC[XYZ_AXIS_COUNT];
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

uint8_t GPS_numSat;

bool persistentFlag(uint8_t mask)
{
    UNUSED(mask);
    return true;
}

uint16_t enableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags |= (mask);
}

uint16_t disableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags &= ~(mask);
}

void gyroUpdate(void) {};

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

int32_t baroCalculateAltitude(void) { return 0; }
}
