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

#ifdef USE_GPS

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "io/dashboard.h"
#include "io/gps.h"
#include "io/serial.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "sensors/sensors.h"

#define LOG_ERROR        '?'
#define LOG_IGNORED      '!'
#define LOG_SKIPPED      '>'
#define LOG_NMEA_GGA     'g'
#define LOG_NMEA_RMC     'r'
#define LOG_UBLOX_SOL    'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'

#define GPS_SV_MAXSATS   16

char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *gpsPacketLogChar = gpsPacketLog;
// **********************
// GPS
// **********************
int32_t GPS_home[2];
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
int16_t GPS_angle[ANGLE_INDEX_COUNT] = { 0, 0 };    // it's the angles that must be applied for GPS correction
float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
int16_t actual_speed[2] = { 0, 0 };
int16_t nav_takeoff_bearing;
navigationMode_e nav_mode = NAV_MODE_NONE;    // Navigation mode

// moving average filter variables
#define GPS_FILTERING              1    // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency
#ifdef GPS_FILTERING
#define GPS_FILTER_VECTOR_LENGTH 5
static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];   //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];
#endif

gpsSolutionData_t gpsSol;
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0; // SV = Space Vehicle, counter increments each time SV info is received.
uint8_t GPS_update = 0;             // it's a binary toggle to distinct a GPS position update

uint8_t GPS_numCh;                          // Number of channels
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];    // Satellite ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];     // Carrier to Noise Ratio (Signal Strength)

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)

static serialPort_t *gpsPort;

typedef struct gpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex; // see baudRate_e
    const char *ubx;
    const char *mtk;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUDRATE_115200,  BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600,    BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400,    BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200,    BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUDRATE_9600,      BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(gpsInitData) / sizeof(gpsInitData[0]))

#define DEFAULT_BAUD_RATE_INDEX 0

#ifdef USE_GPS_UBLOX
static const uint8_t ubloxInit[] = {
    //Preprocessor Pedestrian Dynamic Platform Model Option
    #if defined(GPS_UBLOX_MODE_PEDESTRIAN)
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,

    //Preprocessor Airborne_1g Dynamic Platform Model Option
    #elif defined(GPS_UBLOX_MODE_AIRBORNE_1G)
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Airborne with
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // <1g acceleration and capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28,

    //Default Airborne_4g Dynamic Platform Model
    #else
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Airborne with
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // <4g acceleration and capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C,
    #endif

    // DISABLE NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,           // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,           // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,           // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,           // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,           // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,           // RMC: Recommended Minimum data

    // Enable UBLOX messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3,           // set SVINFO MSG rate (every cycle - high bandwidth)
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x05, 0x40, 0xA7,           // set SVINFO MSG rate (evey 5 cycles - low bandwidth)
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,           // set VELNED MSG rate

    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,             // set rate to 5Hz (measurement period: 200ms, navigation rate: 1 cycle)
};

// UBlox 6 Protocol documentation - GPS.G6-SW-10018-F
// SBAS Configuration Settings Desciption, Page 4/210
// 31.21 CFG-SBAS (0x06 0x16), Page 142/210
// A.10 SBAS Configuration (UBX-CFG-SBAS), Page 198/210 - GPS.G6-SW-10018-F

#define UBLOX_SBAS_PREFIX_LENGTH 10

static const uint8_t ubloxSbasPrefix[UBLOX_SBAS_PREFIX_LENGTH] = { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00 };

#define UBLOX_SBAS_MESSAGE_LENGTH 6
typedef struct ubloxSbas_s {
    sbasMode_e mode;
    uint8_t message[UBLOX_SBAS_MESSAGE_LENGTH];
} ubloxSbas_t;



// Note: these must be defined in the same order is sbasMode_e since no lookup table is used.
static const ubloxSbas_t ubloxSbas[] = {
    // NOTE this could be optimized to save a few bytes of flash space since the same prefix is used for each command.
    { SBAS_AUTO,  { 0x00, 0x00, 0x00, 0x00, 0x31, 0xE5}},
    { SBAS_EGNOS, { 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41}},
    { SBAS_WAAS,  { 0x04, 0xE0, 0x04, 0x00, 0x19, 0x9D}},
    { SBAS_MSAS,  { 0x00, 0x02, 0x02, 0x00, 0x35, 0xEF}},
    { SBAS_GAGAN, { 0x80, 0x01, 0x00, 0x00, 0xB2, 0xE8}}
};
#endif // USE_GPS_UBLOX

typedef enum {
    GPS_UNKNOWN,
    GPS_INITIALIZING,
    GPS_CHANGE_BAUD,
    GPS_CONFIGURE,
    GPS_RECEIVING_DATA,
    GPS_LOST_COMMUNICATION
} gpsState_e;

gpsData_t gpsData;


PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

PG_RESET_TEMPLATE(gpsConfig_t, gpsConfig,
    .provider = GPS_NMEA,
    .sbasMode = SBAS_AUTO,
    .autoConfig = GPS_AUTOCONFIG_ON,
    .autoBaud = GPS_AUTOBAUD_OFF
);

static void shiftPacketLog(void)
{
    uint32_t i;

    for (i = ARRAYLEN(gpsPacketLog) - 1; i > 0 ; i--) {
        gpsPacketLog[i] = gpsPacketLog[i-1];
    }
}

static void gpsNewData(uint16_t c);
#ifdef USE_GPS_NMEA
static bool gpsNewFrameNMEA(char c);
#endif
#ifdef USE_GPS_UBLOX
static bool gpsNewFrameUBLOX(uint8_t data);
#endif

static void gpsSetState(gpsState_e state)
{
    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
    gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}

void gpsInit(void)
{
    gpsData.baudrateIndex = 0;
    gpsData.errors = 0;
    gpsData.timeouts = 0;

    memset(gpsPacketLog, 0x00, sizeof(gpsPacketLog));

    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_UNKNOWN);

    gpsData.lastMessage = millis();

    serialPortConfig_t *gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
    if (!gpsPortConfig) {
        featureClear(FEATURE_GPS);
        return;
    }

    while (gpsInitData[gpsData.baudrateIndex].baudrateIndex != gpsPortConfig->gps_baudrateIndex) {
        gpsData.baudrateIndex++;
        if (gpsData.baudrateIndex >= GPS_INIT_DATA_ENTRY_COUNT) {
            gpsData.baudrateIndex = DEFAULT_BAUD_RATE_INDEX;
            break;
        }
    }

    portMode_e mode = MODE_RXTX;
    // only RX is needed for NMEA-style GPS
#if !defined(COLIBRI_RACE) || !defined(LUX_RACE)
    if (gpsConfig()->provider == GPS_NMEA)
        mode &= ~MODE_TX;
#endif

    // no callback - buffer will be consumed in gpsUpdate()
    gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, NULL, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex], mode, SERIAL_NOT_INVERTED);
    if (!gpsPort) {
        featureClear(FEATURE_GPS);
        return;
    }

    // signal GPS "thread" to initialize when it gets to it
    gpsSetState(GPS_INITIALIZING);
}

#ifdef USE_GPS_NMEA
void gpsInitNmea(void)
{
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
    uint32_t now;
#endif
    switch (gpsData.state) {
        case GPS_INITIALIZING:
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
           now = millis();
           if (now - gpsData.state_ts < 1000)
               return;
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               serialSetBaudRate(gpsPort, 4800);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               // print our FIXED init string for the baudrate we want to be at
               serialPrint(gpsPort, "$PSRF100,1,115200,8,1,0*05\r\n");
               gpsData.state_position++;
           } else {
               // we're now (hopefully) at the correct rate, next state will switch to it
               gpsSetState(GPS_CHANGE_BAUD);
           }
           break;
#endif
        case GPS_CHANGE_BAUD:
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
           now = millis();
           if (now - gpsData.state_ts < 1000)
               return;
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               serialPrint(gpsPort, "$PSRF103,00,6,00,0*23\r\n");
               gpsData.state_position++;
           } else {
#else
            serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
#endif
            gpsSetState(GPS_RECEIVING_DATA);
#if defined(COLIBRI_RACE) || defined(LUX_RACE)
           }
#endif
            break;
    }
}
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
void gpsInitUblox(void)
{
    uint32_t now;
    // UBX will run at the serial port's baudrate, it shouldn't be "autodetected". So here we force it to that rate

    // Wait until GPS transmit buffer is empty
    if (!isSerialTransmitBufferEmpty(gpsPort))
        return;


    switch (gpsData.state) {
        case GPS_INITIALIZING:
            now = millis();
            if (now - gpsData.state_ts < GPS_BAUDRATE_CHANGE_DELAY)
                return;

            if (gpsData.state_position < GPS_INIT_ENTRIES) {
                // try different speed to INIT
                baudRate_e newBaudRateIndex = gpsInitData[gpsData.state_position].baudrateIndex;

                gpsData.state_ts = now;

                if (lookupBaudRateIndex(serialGetBaudRate(gpsPort)) != newBaudRateIndex) {
                    // change the rate if needed and wait a little
                    serialSetBaudRate(gpsPort, baudRates[newBaudRateIndex]);
                    return;
                }

                // print our FIXED init string for the baudrate we want to be at
                serialPrint(gpsPort, gpsInitData[gpsData.baudrateIndex].ubx);

                gpsData.state_position++;
            } else {
                // we're now (hopefully) at the correct rate, next state will switch to it
                gpsSetState(GPS_CHANGE_BAUD);
            }
            break;
        case GPS_CHANGE_BAUD:
            serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
            gpsSetState(GPS_CONFIGURE);
            break;
        case GPS_CONFIGURE:

            // Either use specific config file for GPS or let dynamically upload config
            if ( gpsConfig()->autoConfig == GPS_AUTOCONFIG_OFF ) {
                gpsSetState(GPS_RECEIVING_DATA);
                break;
            }

            if (gpsData.messageState == GPS_MESSAGE_STATE_IDLE) {
                gpsData.messageState++;
            }

            if (gpsData.messageState == GPS_MESSAGE_STATE_INIT) {

                if (gpsData.state_position < sizeof(ubloxInit)) {
                    serialWrite(gpsPort, ubloxInit[gpsData.state_position]);
                    gpsData.state_position++;
                } else {
                    gpsData.state_position = 0;
                    gpsData.messageState++;
                }
            }

            if (gpsData.messageState == GPS_MESSAGE_STATE_SBAS) {
                if (gpsData.state_position < UBLOX_SBAS_PREFIX_LENGTH) {
                    serialWrite(gpsPort, ubloxSbasPrefix[gpsData.state_position]);
                    gpsData.state_position++;
                } else if (gpsData.state_position < UBLOX_SBAS_PREFIX_LENGTH + UBLOX_SBAS_MESSAGE_LENGTH) {
                    serialWrite(gpsPort, ubloxSbas[gpsConfig()->sbasMode].message[gpsData.state_position - UBLOX_SBAS_PREFIX_LENGTH]);
                    gpsData.state_position++;
                } else {
                    gpsData.messageState++;
                }
            }

            if (gpsData.messageState >= GPS_MESSAGE_STATE_ENTRY_COUNT) {
                // ublox should be initialised, try receiving
                gpsSetState(GPS_RECEIVING_DATA);
            }
            break;
    }
}
#endif // USE_GPS_UBLOX

void gpsInitHardware(void)
{
    switch (gpsConfig()->provider) {
    case GPS_NMEA:
#ifdef USE_GPS_NMEA
        gpsInitNmea();
#endif
        break;

    case GPS_UBLOX:
#ifdef USE_GPS_UBLOX
        gpsInitUblox();
#endif
        break;
    }
}

static void updateGpsIndicator(timeUs_t currentTimeUs)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && (gpsSol.numSat >= 5)) {
        GPSLEDTime = currentTimeUs + 150000;
        LED1_TOGGLE;
    }
}

void gpsUpdate(timeUs_t currentTimeUs)
{
    // read out available GPS bytes
    if (gpsPort) {
        while (serialRxBytesWaiting(gpsPort))
            gpsNewData(serialRead(gpsPort));
    }

    switch (gpsData.state) {
        case GPS_UNKNOWN:
            break;

        case GPS_INITIALIZING:
        case GPS_CHANGE_BAUD:
        case GPS_CONFIGURE:
            gpsInitHardware();
            break;

        case GPS_LOST_COMMUNICATION:
            gpsData.timeouts++;
            if (gpsConfig()->autoBaud) {
                // try another rate
                gpsData.baudrateIndex++;
                gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            gpsData.lastMessage = millis();
            gpsSol.numSat = 0;
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_INITIALIZING);
            break;

        case GPS_RECEIVING_DATA:
            // check for no data/gps timeout/cable disconnection etc
            if (millis() - gpsData.lastMessage > GPS_TIMEOUT) {
                // remove GPS from capability
                sensorsClear(SENSOR_GPS);
                gpsSetState(GPS_LOST_COMMUNICATION);
            }
            break;
    }
    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTimeUs);
    }
}

static void gpsNewData(uint16_t c)
{
    if (!gpsNewFrame(c)) {
        return;
    }

    // new data received and parsed, we're in business
    gpsData.lastLastMessage = gpsData.lastMessage;
    gpsData.lastMessage = millis();
    sensorsSet(SENSOR_GPS);

    if (GPS_update == 1)
        GPS_update = 0;
    else
        GPS_update = 1;

#if 0
    debug[3] = GPS_update;
#endif

    onGpsNewData();
}

bool gpsNewFrame(uint8_t c)
{
    switch (gpsConfig()->provider) {
    case GPS_NMEA:          // NMEA
#ifdef USE_GPS_NMEA
        return gpsNewFrameNMEA(c);
#endif
        break;
    case GPS_UBLOX:         // UBX binary
#ifdef USE_GPS_UBLOX
        return gpsNewFrameUBLOX(c);
#endif
        break;
    }
    return false;
}


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


// This code is used for parsing NMEA data

/* Alex optimization
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
static uint32_t GPS_coord_to_degrees(char *coordinateString)
{
    char *p = s, *d = s;
    uint8_t min, deg = 0;
    uint16_t frac = 0, mult = 10000;

    while (*p) {                // parse the string until its end
        if (d != s) {
            frac += (*p - '0') * mult;  // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
            mult /= 10;
        }
        if (*p == '.')
            d = p;              // locate '.' char in the string
        p++;
    }
    if (p == s)
        return 0;
    while (s < d - 2) {
        deg *= 10;              // convert degrees : all chars before minutes ; for the first iteration, deg = 0
        deg += *(s++) - '0';
    }
    min = *(d - 1) - '0' + (*(d - 2) - '0') * 10;       // convert minutes : 2 previous char before '.'
    return deg * 10000000UL + (min * 100000UL + frac) * 10UL / 6;
}
*/

// helper functions
#ifdef USE_GPS_NMEA
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
                    switch (param) {
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
                                ENABLE_STATE(GPS_FIX);
                            } else {
                                DISABLE_STATE(GPS_FIX);
                            }
                            break;
                        case 7:
                            gps_Msg.numSat = grab_fields(string, 0);
                            break;
                        case 9:
                            gps_Msg.altitude = grab_fields(string, 0);     // altitude in meters added by Mis
                            break;
                    }
                    break;
                case FRAME_RMC:        //************* GPRMC FRAME parsing
                    switch (param) {
                        case 7:
                            gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                            break;
                        case 8:
                            gps_Msg.ground_course = (grab_fields(string, 1));      // ground course deg * 10
                            break;
                    }
                    break;
                case FRAME_GSV:
                    switch (param) {
                      /*case 1:
                            // Total number of messages of this type in this cycle
                            break; */
                        case 2:
                            // Message number
                            svMessageNum = grab_fields(string, 0);
                            break;
                        case 3:
                            // Total number of SVs visible
                            GPS_numCh = grab_fields(string, 0);
                            break;
                    }
                    if (param < 4)
                        break;

                    svPacketIdx = (param - 4) / 4 + 1; // satellite number in packet, 1-4
                    svSatNum    = svPacketIdx + (4 * (svMessageNum - 1)); // global satellite number
                    svSatParam  = param - 3 - (4 * (svPacketIdx - 1)); // parameter number for satellite

                    if (svSatNum > GPS_SV_MAXSATS)
                        break;

                    switch (svSatParam) {
                        case 1:
                            // SV PRN number
                            GPS_svinfo_chn[svSatNum - 1]  = svSatNum;
                            GPS_svinfo_svid[svSatNum - 1] = grab_fields(string, 0);
                            break;
                      /*case 2:
                            // Elevation, in degrees, 90 maximum
                            break;
                        case 3:
                            // Azimuth, degrees from True North, 000 through 359
                            break; */
                        case 4:
                            // SNR, 00 through 99 dB (null when not tracking)
                            GPS_svinfo_cno[svSatNum - 1] = grab_fields(string, 0);
                            GPS_svinfo_quality[svSatNum - 1] = 0; // only used by ublox
                            break;
                    }

                    GPS_svInfoReceivedCount++;

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
                shiftPacketLog();
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    *gpsPacketLogChar = LOG_IGNORED;
                    GPS_packetCount++;
                    switch (gps_frame) {
                    case FRAME_GGA:
                      *gpsPacketLogChar = LOG_NMEA_GGA;
                      frameOK = 1;
                      if (STATE(GPS_FIX)) {
                            gpsSol.llh.lat = gps_Msg.latitude;
                            gpsSol.llh.lon = gps_Msg.longitude;
                            gpsSol.numSat = gps_Msg.numSat;
                            gpsSol.llh.alt = gps_Msg.altitude;
                        }
                        break;
                    case FRAME_RMC:
                        *gpsPacketLogChar = LOG_NMEA_RMC;
                        gpsSol.groundSpeed = gps_Msg.speed;
                        gpsSol.groundCourse = gps_Msg.ground_course;
                        break;
                    } // end switch
                } else {
                    *gpsPacketLogChar = LOG_ERROR;
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
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubx_nav_status;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

typedef struct {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[16];         // 16 satellites * 12 byte
} ubx_nav_svinfo;

enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubx_protocol_bytes;

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static bool _skip_packet;
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

static bool next_fix;
static uint8_t _class;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

// Example packet sizes from UBlox u-center from a Glonass capable GPS receiver.
//15:17:55  R -> UBX NAV-STATUS,  Size  24,  'Navigation Status'
//15:17:55  R -> UBX NAV-POSLLH,  Size  36,  'Geodetic Position'
//15:17:55  R -> UBX NAV-VELNED,  Size  44,  'Velocity in WGS 84'
//15:17:55  R -> UBX NAV-CLOCK,  Size  28,  'Clock Status'
//15:17:55  R -> UBX NAV-AOPSTATUS,  Size  24,  'AOP Status'
//15:17:55  R -> UBX 03-09,  Size 208,  'Unknown'
//15:17:55  R -> UBX 03-10,  Size 336,  'Unknown'
//15:17:55  R -> UBX NAV-SOL,  Size  60,  'Navigation Solution'
//15:17:55  R -> UBX NAV,  Size 100,  'Navigation'
//15:17:55  R -> UBX NAV-SVINFO,  Size 328,  'Satellite Status and Information'

// from the UBlox6 document, the largest payout we receive i the NAV-SVINFO and the payload size
// is calculated as 8 + 12*numCh.  numCh in the case of a Glonass receiver is 28.
#define UBLOX_PAYLOAD_SIZE 344


// Receive buffer
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    ubx_nav_svinfo svinfo;
    uint8_t bytes[UBLOX_PAYLOAD_SIZE];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}


static bool UBLOX_parse_gps(void)
{
    uint32_t i;

    *gpsPacketLogChar = LOG_IGNORED;

    switch (_msg_id) {
    case MSG_POSLLH:
        *gpsPacketLogChar = LOG_UBLOX_POSLLH;
        //i2c_dataset.time                = _buffer.posllh.time;
        gpsSol.llh.lon = _buffer.posllh.longitude;
        gpsSol.llh.lat = _buffer.posllh.latitude;
        gpsSol.llh.alt = _buffer.posllh.altitude_msl / 10 / 100;  //alt in m
        if (next_fix) {
            ENABLE_STATE(GPS_FIX);
        } else {
            DISABLE_STATE(GPS_FIX);
        }
        _new_position = true;
        break;
    case MSG_STATUS:
        *gpsPacketLogChar = LOG_UBLOX_STATUS;
        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        break;
    case MSG_SOL:
        *gpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        gpsSol.numSat = _buffer.solution.satellites;
        gpsSol.hdop = _buffer.solution.position_DOP;
        break;
    case MSG_VELNED:
        *gpsPacketLogChar = LOG_UBLOX_VELNED;
        // speed_3d                        = _buffer.velned.speed_3d;  // cm/s
        gpsSol.groundSpeed = _buffer.velned.speed_2d;    // cm/s
        gpsSol.groundCourse = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
        break;
    case MSG_SVINFO:
        *gpsPacketLogChar = LOG_UBLOX_SVINFO;
        GPS_numCh = _buffer.svinfo.numCh;
        if (GPS_numCh > 16)
            GPS_numCh = 16;
        for (i = 0; i < GPS_numCh; i++) {
            GPS_svinfo_chn[i]= _buffer.svinfo.channel[i].chn;
            GPS_svinfo_svid[i]= _buffer.svinfo.channel[i].svid;
            GPS_svinfo_quality[i]=_buffer.svinfo.channel[i].quality;
            GPS_svinfo_cno[i]= _buffer.svinfo.channel[i].cno;
        }
        GPS_svInfoReceivedCount++;
        break;
    default:
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}

static bool gpsNewFrameUBLOX(uint8_t data)
{
    bool parsed = false;

    switch (_step) {
        case 0: // Sync char 1 (0xB5)
            if (PREAMBLE1 == data) {
                _skip_packet = false;
                _step++;
            }
            break;
        case 1: // Sync char 2 (0x62)
            if (PREAMBLE2 != data) {
                _step = 0;
                break;
            }
            _step++;
            break;
        case 2: // Class
            _step++;
            _class = data;
            _ck_b = _ck_a = data;   // reset the checksum accumulators
            break;
        case 3: // Id
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _msg_id = data;
            break;
        case 4: // Payload length (part 1)
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length = data; // payload length low byte
            break;
        case 5: // Payload length (part 2)
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > UBLOX_PAYLOAD_SIZE) {
                _skip_packet = true;
            }
            _payload_counter = 0;   // prepare to receive payload
            if (_payload_length == 0) {
                _step = 7;
            }
            break;
        case 6:
            _ck_b += (_ck_a += data);       // checksum byte
            if (_payload_counter < UBLOX_PAYLOAD_SIZE) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter >= _payload_length) {
                _step++;
            }
            break;
        case 7:
            _step++;
            if (_ck_a != data) {
                _skip_packet = true;          // bad checksum
                gpsData.errors++;
            }
            break;
        case 8:
            _step = 0;

            shiftPacketLog();

            if (_ck_b != data) {
                *gpsPacketLogChar = LOG_ERROR;
                gpsData.errors++;
                break;              // bad checksum
            }

            GPS_packetCount++;

            if (_skip_packet) {
                *gpsPacketLogChar = LOG_SKIPPED;
                break;
            }

            if (UBLOX_parse_gps()) {
                parsed = true;
            }
    }
    return parsed;
}
#endif // USE_GPS_UBLOX

static void gpsHandlePassthrough(uint8_t data)
{
     gpsNewData(data);
 #ifdef USE_DASHBOARD
     if (feature(FEATURE_DASHBOARD)) {
         dashboardUpdate(micros());
     }
 #endif

 }

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

    if (!(gpsPort->mode & MODE_TX))
        serialSetMode(gpsPort, gpsPort->mode | MODE_TX);

#ifdef USE_DASHBOARD
    if (feature(FEATURE_DASHBOARD)) {
        dashboardShowFixedPage(PAGE_GPS);
    }
#endif

    serialPassthrough(gpsPort, gpsPassthroughPort, &gpsHandlePassthrough, NULL);
}

float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (ABS((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}


void GPS_reset_home_position(void)
{
    if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
        GPS_home[LAT] = gpsSol.llh.lat;
        GPS_home[LON] = gpsSol.llh.lon;
        GPS_calc_longitude_scaling(gpsSol.llh.lat); // need an initial value for distance and bearing calc
#ifdef USE_NAV
        nav_takeoff_bearing = DECIDEGREES_TO_DEGREES(attitude.values.yaw);              // save takeoff heading
#endif
        // Set ground altitude
        ENABLE_STATE(GPS_FIX_HOME);
    }
}

////////////////////////////////////////////////////////////////////////////////////
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

void GPS_calculateDistanceAndDirectionToHome(void)
{
    if (STATE(GPS_FIX_HOME)) {      // If we don't have home set, do not display anything
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
        GPS_distanceToHome = dist / 100;
        GPS_directionToHome = dir / 100;
    } else {
        GPS_distanceToHome = 0;
        GPS_directionToHome = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
static void GPS_calc_velocity(void)
{
    static int16_t speed_old[2] = { 0, 0 };
    static int32_t last_coord[2] = { 0, 0 };
    static uint8_t init = 0;

    if (init) {
        float tmp = 1.0f / dTnav;
        actual_speed[GPS_X] = (float)(gpsSol.llh.lon - last_coord[LON]) * GPS_scaleLonDown * tmp;
        actual_speed[GPS_Y] = (float)(gpsSol.llh.lat - last_coord[LAT]) * tmp;

        actual_speed[GPS_X] = (actual_speed[GPS_X] + speed_old[GPS_X]) / 2;
        actual_speed[GPS_Y] = (actual_speed[GPS_Y] + speed_old[GPS_Y]) / 2;

        speed_old[GPS_X] = actual_speed[GPS_X];
        speed_old[GPS_Y] = actual_speed[GPS_Y];
    }
    init = 1;

    last_coord[LON] = gpsSol.llh.lon;
    last_coord[LAT] = gpsSol.llh.lat;
}

void onGpsNewData(void)
{
    if (!(STATE(GPS_FIX) && gpsSol.numSat >= 5)) {
        return;
    }

    if (!ARMING_FLAG(ARMED))
        DISABLE_STATE(GPS_FIX_HOME);

    if (!STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED))
        GPS_reset_home_position();

    // Apply moving average filter to GPS data
#if defined(GPS_FILTERING)
    GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
    for (int axis = 0; axis < 2; axis++) {
        GPS_read[axis] = axis == LAT ? gpsSol.llh.lat : gpsSol.llh.lon; // latest unfiltered data is in GPS_latitude and GPS_longitude
        GPS_degree[axis] = GPS_read[axis] / 10000000;   // get the degree to assure the sum fits to the int32_t

        // How close we are to a degree line ? its the first three digits from the fractions of degree
        // later we use it to Check if we are close to a degree line, if yes, disable averaging,
        fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;

        GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
        GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
        GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
        GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
        if (nav_mode == NAV_MODE_POSHOLD) {             // we use gps averaging only in poshold mode...
            if (fraction3[axis] > 1 && fraction3[axis] < 999) {
                if (axis == LAT) {
                    gpsSol.llh.lat = GPS_filtered[LAT];
                } else {
                    gpsSol.llh.lon = GPS_filtered[LON];
                }
            }
        }
    }
#endif

    //
    // Calculate time delta for navigation loop, range 0-1.0f, in seconds
    //
    // Time for calculating x,y speed and navigation pids
    static uint32_t nav_loopTimer;
    dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
    nav_loopTimer = millis();
    // prevent runup from bad GPS
    dTnav = MIN(dTnav, 1.0f);

    GPS_calculateDistanceAndDirectionToHome();
    // calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
    GPS_calc_velocity();

#ifdef USE_NAV
    navNewGpsData();
#endif
}

#endif
