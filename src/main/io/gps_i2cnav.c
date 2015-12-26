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
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"

#include "drivers/gps.h"
#include "drivers/gps_i2cnav.h"

#include "sensors/sensors.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"
#include "io/gps_private.h"

#include "flight/gps_conversion.h"
#include "flight/pid.h"
#include "flight/hil.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/runtime_config.h"

#if defined(GPS) && defined(GPS_PROTO_I2C_NAV)

#define GPS_I2C_POLL_RATE_HZ    20  // Poll I2C GPS at this rate

bool gpsDetectI2CNAV(void)
{
    return i2cnavGPSModuleDetect();
}

bool gpsPollI2CNAV(void)
{
    gpsDataGeneric_t gpsMsg;

    // Check for poll rate timeout
    if ((millis() - gpsState.lastMessageMs) < (1000 / GPS_I2C_POLL_RATE_HZ)) 
        return false;

    i2cnavGPSModuleRead(&gpsMsg);
    
    debug[2] = gpsMsg.flags.gpsOk;
    debug[3] = gpsMsg.flags.newData;

    if (gpsMsg.flags.gpsOk) {
        gpsSol.flags.fix3D = gpsMsg.flags.fix3D;

        // sat count
        gpsSol.numSat = gpsMsg.numSat;

        // Other data
        if (gpsMsg.flags.newData) {
            if (gpsMsg.flags.fix3D) {
                gpsSol.hdop = gpsMsg.hdop;
                gpsSol.groundSpeed = gpsMsg.speed;
                gpsSol.groundCourse = gpsMsg.ground_course;
                gpsSol.llh.lat = gpsMsg.latitude;
                gpsSol.llh.lon = gpsMsg.longitude;
                gpsSol.llh.alt = gpsMsg.altitude;
                gpsSol.flags.validVelNE = 0;
                gpsSol.flags.validVelD = 0;
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
    switch(gpsState.state) {
    default:
        return false;

    case GPS_CONFIGURE:
        gpsSetState(GPS_RECEIVING_DATA);
        return false;

    case GPS_RECEIVING_DATA:
        return gpsPollI2CNAV();
    }
}

#endif
