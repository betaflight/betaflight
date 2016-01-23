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

#pragma once

typedef struct {
    struct {
        unsigned gpsOk : 1;     // gps read successful
        unsigned newData : 1;   // new gps data available (lat/lon/alt)
        unsigned fix3D : 1;     // gps fix status
    } flags;
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    uint16_t altitude;
    uint16_t speed;
    uint16_t ground_course;
    uint16_t hdop;
} gpsDataI2CNAV_t;

bool i2cnavGPSModuleDetect(void);
void i2cnavGPSModuleRead(gpsDataI2CNAV_t * gpsMsg);
