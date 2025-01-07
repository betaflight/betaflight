/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_VIRTUAL_GPS
#include "io/gps.h"
#include "io/gps_virtual.h"
#include <stdio.h>
static gpsSolutionData_t gpsVirtualData;
void setVirtualGPS(float latitude, float longitude, float altiutude, float speed, float speed3D, float course)
{
    gpsVirtualData.numSat = 12;    // satellites_in_view
    gpsVirtualData.acc.hAcc = 500; // horizontal_pos_accuracy - convert cm to mm
    gpsVirtualData.acc.vAcc = 500; // vertical_pos_accuracy - convert cm to mm
    gpsVirtualData.acc.sAcc = 400; // horizontal_vel_accuracy - convert cm to mm
    gpsVirtualData.dop.pdop = 1; // hdop in 4.4 and earlier, pdop in 4.5 and above
    gpsVirtualData.llh.lon = (int32_t)(longitude * GPS_DEGREES_DIVIDER);
    gpsVirtualData.llh.lat = (int32_t)(latitude * GPS_DEGREES_DIVIDER);
    gpsVirtualData.llh.altCm = (int32_t)(altiutude * 100.0f); // alt
    gpsVirtualData.groundSpeed = (uint16_t)(speed * 100.0f);
    gpsVirtualData.speed3d = (uint16_t)(speed3D * 100.0f);
    gpsVirtualData.groundCourse = (uint16_t)(course * 10.0f); // incoming value expected to be in centidegrees, output value in decidegrees
}

void getVirtualGPS(gpsSolutionData_t *gpsSolData)
{
    gpsSolData->numSat = gpsVirtualData.numSat;
    gpsSolData->acc.hAcc = gpsVirtualData.acc.hAcc;
    gpsSolData->acc.vAcc = gpsVirtualData.acc.vAcc;
    gpsSolData->acc.sAcc = gpsVirtualData.acc.sAcc;
    gpsSolData->dop.pdop = gpsVirtualData.dop.pdop;
    gpsSolData->llh.lon = gpsVirtualData.llh.lon;
    gpsSolData->llh.lat = gpsVirtualData.llh.lat;
    gpsSolData->llh.altCm = gpsVirtualData.llh.altCm;
    gpsSolData->groundSpeed = gpsVirtualData.groundSpeed;
    gpsSolData->speed3d = gpsVirtualData.speed3d;
    gpsSolData->groundCourse = gpsVirtualData.groundCourse;
}
#endif