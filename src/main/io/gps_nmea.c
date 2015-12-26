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

#if defined(GPS) && defined(GPS_PROTO_NMEA)

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output 5 frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Now verifies checksum correctly before applying data

   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/

#define NO_FRAME   0
#define FRAME_GGA  1
#define FRAME_RMC  2
#define FRAME_GSV  3

static uint32_t grab_fields(char *src, uint8_t mult)
{                               // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        if (src[i] == '.') {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
        if (i >= 15)
            return 0; // out of bounds
    }
    return tmp;
}

typedef struct gpsDataNmea_s {
    bool fix;
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    uint16_t altitude;
    uint16_t speed;
    uint16_t ground_course;
} gpsDataNmea_t;

static bool gpsNewFrameNMEA(char c)
{
    static gpsDataNmea_t gps_Msg;

    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, gps_frame = NO_FRAME;
    static uint8_t svMessageNum = 0;
    uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;

    switch (c) {
        case '$':
            param = 0;
            offset = 0;
            parity = 0;
            break;
        case ',':
        case '*':
            string[offset] = 0;
            if (param == 0) {       //frame identification
                gps_frame = NO_FRAME;
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
                    gps_frame = FRAME_GGA;
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C')
                    gps_frame = FRAME_RMC;
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'V')
                    gps_frame = FRAME_GSV;
            }

            switch (gps_frame) {
                case FRAME_GGA:        //************* GPGGA FRAME parsing
                    switch(param) {
            //          case 1:             // Time information
            //              break;
                        case 2:
                            gps_Msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_Msg.latitude *= -1;
                            break;
                        case 4:
                            gps_Msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_Msg.longitude *= -1;
                            break;
                        case 6:
                            if (string[0] > '0') {
                                gps_Msg.fix = true;
                            } else {
                                gps_Msg.fix = false;
                            }
                            break;
                        case 7:
                            gps_Msg.numSat = grab_fields(string, 0);
                            break;
                        case 9:
                            gps_Msg.altitude = grab_fields(string, 1) * 10;     // altitude in meters added by Mis (cm)
                            break;
                    }
                    break;
                case FRAME_RMC:        //************* GPRMC FRAME parsing
                    switch(param) {
                        case 7:
                            gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                            break;
                        case 8:
                            gps_Msg.ground_course = (grab_fields(string, 1));      // ground course deg * 10
                            break;
                    }
                    break;
                case FRAME_GSV:
                    switch(param) {
                      /*case 1:
                            // Total number of messages of this type in this cycle
                            break; */
                        case 2:
                            // Message number
                            svMessageNum = grab_fields(string, 0);
                            break;
                        case 3:
                            // Total number of SVs visible
                            gpsSol.numCh = grab_fields(string, 0);
                            break;
                    }
                    if(param < 4)
                        break;

                    svPacketIdx = (param - 4) / 4 + 1; // satellite number in packet, 1-4
                    svSatNum    = svPacketIdx + (4 * (svMessageNum - 1)); // global satellite number
                    svSatParam  = param - 3 - (4 * (svPacketIdx - 1)); // parameter number for satellite

                    if(svSatNum > GPS_SV_MAXSATS)
                        break;

                    switch(svSatParam) {
                        case 1:
                            // SV PRN number
                            gpsSol.svInfo[svSatNum - 1].chn = svSatNum;
                            gpsSol.svInfo[svSatNum - 1].svid = grab_fields(string, 0);
                            break;
                      /*case 2:
                            // Elevation, in degrees, 90 maximum
                            break;
                        case 3:
                            // Azimuth, degrees from True North, 000 through 359
                            break; */
                        case 4:
                            // SNR, 00 through 99 dB (null when not tracking)
                            gpsSol.svInfo[svSatNum - 1].cno = grab_fields(string, 0);
                            gpsSol.svInfo[svSatNum - 1].quality = 0; // only used by ublox
                            break;
                    }
                    break;
            }

            param++;
            offset = 0;
            if (c == '*')
                checksum_param = 1;
            else
                parity ^= c;
            break;
        case '\r':
        case '\n':
            if (checksum_param) {   //parity checksum
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    gpsStats.packetCount++;
                    switch (gps_frame) {
                    case FRAME_GGA:
                        frameOK = 1;
                        if (STATE(GPS_FIX)) {
                            gpsSol.numSat = gps_Msg.numSat;
                            gpsSol.llh.lat = gps_Msg.latitude;
                            gpsSol.llh.lon = gps_Msg.longitude;
                            gpsSol.llh.alt = gps_Msg.altitude;
                        }

                        // NMEA does not report VELNED
                        gpsSol.flags.validVelNE = 0;
                        gpsSol.flags.validVelD = 0;
                        break;
                    case FRAME_RMC:
                        gpsSol.groundSpeed = gps_Msg.speed;
                        gpsSol.groundCourse = gps_Msg.ground_course;
                        break;
                    } // end switch
                }
                else {
                    gpsStats.errors++;
                }
            }
            checksum_param = 0;
            break;
        default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
    }
    return frameOK;
}

static bool gpsInitialize(void)
{
    // NMEA is receive-only, we can only cycle thru baud rates and check if we are receiving anything
    gpsState.baudrateIndex = gpsState.autoBaudrateIndex;
    return false;
}

static bool gpsConfigure(void)
{
    // No autoconfig, switch straight to receiving data 
    gpsSetState(GPS_RECEIVING_DATA);
    return false;
}

static bool gpsReceiveData(void)
{
    bool hasNewData = false;

    if (gpsState.gpsPort) {
        while (serialRxBytesWaiting(gpsState.gpsPort)) {
            uint8_t newChar = serialRead(gpsState.gpsPort);
            if (gpsNewFrameNMEA(newChar)) {
                gpsSol.flags.gpsHeartbeat = !gpsSol.flags.gpsHeartbeat;
                gpsSol.flags.validVelNE = 0;
                gpsSol.flags.validVelD = 0;
                hasNewData = true;
            }
        }
    }

    return hasNewData;
}

bool gpsHandleNMEA(void)
{
    // Receive data
    bool hasNewData = gpsReceiveData();

    // Process state
    switch(gpsState.state) {
    default:
        return false;

    case GPS_INITIALIZING:
        return gpsInitialize();

    case GPS_CONFIGURE:
        return gpsConfigure();

    case GPS_RECEIVING_DATA:
        return hasNewData;
    }
}

#endif
