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
#include <ctype.h>

#include "platform.h"

#if defined(GPS) && defined(GPS_PROTO_I2C_NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/gps_i2cnav.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/hil.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/gps_private.h"
#include "io/serial.h"

#include "navigation/navigation.h"


#define GPS_I2C_POLL_RATE_HZ    20  // Poll I2C GPS at this rate

bool gpsDetectI2CNAV(void)
{
    return i2cnavGPSModuleDetect();
}

bool gpsPollI2CNAV(void)
{
    static uint32_t lastPollTime = 0;
    gpsDataI2CNAV_t gpsMsg;

    // Check for poll rate timeout
    if ((millis() - lastPollTime) < (1000 / GPS_I2C_POLL_RATE_HZ))
        return false;

    lastPollTime = millis();

    i2cnavGPSModuleRead(&gpsMsg);

    if (gpsMsg.flags.gpsOk) {
        if (gpsMsg.flags.fix3D) {
            // No fix type available - assume 3D
            gpsSol.fixType = GPS_FIX_3D;
        }
        else {
            gpsSol.fixType = GPS_NO_FIX;
        }

        // sat count
        gpsSol.numSat = gpsMsg.numSat;

        // Other data
        if (gpsMsg.flags.newData) {
            if (gpsMsg.flags.fix3D) {
                gpsSol.hdop = gpsConstrainHDOP(gpsMsg.hdop);
                gpsSol.eph = gpsConstrainEPE(gpsMsg.hdop * GPS_HDOP_TO_EPH_MULTIPLIER);
                gpsSol.epv = gpsConstrainEPE(gpsMsg.hdop * GPS_HDOP_TO_EPH_MULTIPLIER);  // i2c-nav doesn't give vdop data, fake it using hdop
                gpsSol.groundSpeed = gpsMsg.speed;
                gpsSol.groundCourse = gpsMsg.ground_course;
                gpsSol.llh.lat = gpsMsg.latitude;
                gpsSol.llh.lon = gpsMsg.longitude;
                gpsSol.llh.alt = gpsMsg.altitude;
                gpsSol.flags.validVelNE = 0;
                gpsSol.flags.validVelD = 0;
                gpsSol.flags.validEPE = 0;
            }

            gpsStats.packetCount++;

            gpsSol.flags.gpsHeartbeat = !gpsSol.flags.gpsHeartbeat;

            return true;
        }
    }

    return false;
}

bool gpsHandleI2CNAV(void)
{
    // Process state
    switch (gpsState.state) {
    default:
        return false;

    case GPS_CHANGE_BAUD:
    case GPS_CHECK_VERSION:
    case GPS_CONFIGURE:
        gpsSetState(GPS_RECEIVING_DATA);
        return false;

    case GPS_RECEIVING_DATA:
        return gpsPollI2CNAV();
    }
}

#endif
