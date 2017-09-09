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

#include <stdbool.h>

#include "config/parameter_group.h"

#define GPS_DBHZ_MIN 0
#define GPS_DBHZ_MAX 55

#define LAT 0
#define LON 1

#define GPS_DEGREES_DIVIDER 10000000L

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_I2CNAV,
    GPS_NAZA,
    GPS_UBLOX7PLUS,
    GPS_MTK,
    GPS_PROVIDER_COUNT
} gpsProvider_e;

typedef enum {
    SBAS_AUTO = 0,
    SBAS_EGNOS,
    SBAS_WAAS,
    SBAS_MSAS,
    SBAS_GAGAN,
    SBAS_NONE
} sbasMode_e;

#define SBAS_MODE_MAX SBAS_GAGAN

typedef enum {
    GPS_BAUDRATE_115200 = 0,
    GPS_BAUDRATE_57600,
    GPS_BAUDRATE_38400,
    GPS_BAUDRATE_19200,
    GPS_BAUDRATE_9600,
    GPS_BAUDRATE_COUNT
} gpsBaudRate_e;

typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON,
} gpsAutoConfig_e;

typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON
} gpsAutoBaud_e;

typedef enum {
    GPS_DYNMODEL_PEDESTRIAN = 0,
    GPS_DYNMODEL_AIR_1G,
    GPS_DYNMODEL_AIR_4G,
} gpsDynModel_e;

typedef enum {
    GPS_NO_FIX = 0,
    GPS_FIX_2D,
    GPS_FIX_3D
} gpsFixType_e;

#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600

typedef struct gpsConfig_s {
    gpsProvider_e provider;
    sbasMode_e sbasMode;
    gpsAutoConfig_e autoConfig;
    gpsAutoBaud_e autoBaud;
    gpsDynModel_e dynModel;
    uint8_t gpsMinSats;
} gpsConfig_t;

PG_DECLARE(gpsConfig_t, gpsConfig);

typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;

/* LLH Location in NEU axis system */
typedef struct gpsLocation_s {
    int32_t lat;    // Latitude * 1e+7
    int32_t lon;    // Longitude * 1e+7
    int32_t alt;    // Altitude in centimeters (meters * 100)
} gpsLocation_t;

typedef struct gpsSolutionData_s {
    struct {
        bool gpsHeartbeat;  // Toggle each update
        bool validVelNE;
        bool validVelD;
        bool validMag;
        bool validEPE;      // EPH/EPV values are valid - actual accuracy
    } flags;

    gpsFixType_e fixType;
    uint8_t numSat;

    gpsLocation_t llh;
    int16_t       magData[3];
    int16_t       velNED[3];

    int16_t groundSpeed;
    int16_t groundCourse;

    uint16_t eph;   // horizontal accuracy (cm)
    uint16_t epv;   // vertical accuracy (cm)

    uint16_t hdop;  // generic HDOP value (*100)
} gpsSolutionData_t;

typedef struct {
    uint16_t    lastMessageDt;
    uint32_t    errors;                // gps error counter - crc error/lost of data/sync etc..
    uint32_t    timeouts;
    uint32_t    packetCount;
} gpsStatistics_t;

extern gpsSolutionData_t gpsSol;
extern gpsStatistics_t   gpsStats;

struct magDev_s;
bool gpsMagDetect(struct magDev_s *mag);
void gpsPreInit(void);
void gpsInit(void);
void gpsThread(void);
void updateGpsIndicator(timeUs_t currentTimeUs);
bool isGPSHealthy(void);
struct serialPort_s;
void gpsEnablePassthrough(struct serialPort_s *gpsPassthroughPort);
