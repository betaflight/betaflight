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
#include <cmath>

extern "C" {
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "config/feature.h"
    #include "pg/pg_ids.h"

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/compass/compass.h"
    #include "drivers/sensor.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "io/gps.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"

    void imuUpdateEulerAngles(void);

    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);

    PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
        .enabledFeatures = 0
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// STUBS

extern "C" {
boxBitmask_t rcModeActivationMask;
float rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float vGyroStdDevModulus;

gyro_t gyro;
acc_t acc;
mag_t mag;

gpsSolutionData_t gpsSol;
uint16_t GPS_distanceToHome = 0;

uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

pidProfile_t *currentPidProfile;

uint16_t enableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags |= (mask);
}

uint16_t disableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags &= ~(mask);
}

bool sensors(uint32_t mask)
{
    UNUSED(mask);
    return false;
};

uint32_t millis(void) { return 0; }
uint32_t micros(void) { return 0; }

bool compassIsHealthy(quaternion *) { return true; }
bool isBaroCalibrationComplete(void) { return true; }
void performBaroCalibrationCycle(void) {}
int32_t baroCalculateAltitude(void) { return 0; }
bool gyroGetAverage(quaternion *) { return false; }
bool accGetAverage(quaternion *) { return false; }
bool accIsHealthy(quaternion *) { return false; }
bool compassGetAverage(quaternion *) { return false; }
bool isBeeperOn(void){ return true; }
}
