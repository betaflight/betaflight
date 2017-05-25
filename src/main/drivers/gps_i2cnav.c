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

#include <platform.h>

#include "build/build_config.h"


#include "gps_i2cnav.h"

#include "drivers/gpio.h"
#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#ifndef GPS_I2C_INSTANCE
#define GPS_I2C_INSTANCE I2CDEV_1
#endif

#define I2C_GPS_ADDRESS               0x20 //7 bits

#define I2C_GPS_STATUS_00             00    //(Read only)
  #define I2C_GPS_STATUS_NEW_DATA       0x01  // New data is available (after every GGA frame)
  #define I2C_GPS_STATUS_2DFIX          0x02  // 2dfix achieved
  #define I2C_GPS_STATUS_3DFIX          0x04  // 3dfix achieved
  #define I2C_GPS_STATUS_NUMSATS        0xF0  // Number of sats in view
#define I2C_GPS_REG_VERSION           03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_LOCATION              07    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_GROUND_SPEED          31    // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE              33    // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE         35    // GPS ground course (uint16_t)
#define I2C_GPS_TIME                  39    // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)

bool i2cnavGPSModuleDetect(void)
{
    bool ack;
    uint8_t i2cGpsStatus;

    ack = i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_STATUS_00, 1, &i2cGpsStatus); /* status register */

    if (ack)
        return true;

    return false;
}

void i2cnavGPSModuleRead(gpsDataI2CNAV_t * gpsMsg)
{
    gpsMsg->flags.newData = 0;
    gpsMsg->flags.fix3D = 0;
    gpsMsg->flags.gpsOk = 0;

    uint8_t i2cGpsStatus;
    bool ack = i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_STATUS_00, 1, &i2cGpsStatus); /* status register */

    if (!ack)
        return;

    gpsMsg->flags.gpsOk = 1;
    gpsMsg->numSat = i2cGpsStatus >> 4;

    if (i2cGpsStatus & I2C_GPS_STATUS_3DFIX) {
        gpsMsg->flags.fix3D = 1;

        if (i2cGpsStatus & I2C_GPS_STATUS_NEW_DATA) {
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_LOCATION,      4, (uint8_t*)&gpsMsg->latitude);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_LOCATION + 4,  4, (uint8_t*)&gpsMsg->longitude);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_GROUND_SPEED,  2, (uint8_t*)&gpsMsg->speed);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_GROUND_COURSE, 2, (uint8_t*)&gpsMsg->ground_course);
            i2cRead(GPS_I2C_INSTANCE, I2C_GPS_ADDRESS, I2C_GPS_ALTITUDE,      2, (uint8_t*)&gpsMsg->altitude);

            gpsMsg->hdop = 0;

            gpsMsg->flags.newData = 1;
        }
    }
}
