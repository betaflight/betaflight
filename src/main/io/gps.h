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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"

#include "io/serial.h"

#include "pg/gps.h"

#define GPS_DEGREES_DIVIDER 10000000L
#define EARTH_ANGLE_TO_CM (111.3195f * 1000 * 100 / GPS_DEGREES_DIVIDER) // 1.113195 cm per latitude unit at the equator (111.3195km/deg)
#define GPS_X 1
#define GPS_Y 0
#define GPS_MIN_SAT_COUNT 4     // number of sats to trigger low sat count sanity check

#ifdef USE_GPS_UBLOX
typedef enum {
    UBX_VERSION_UNDEF = 0,
    UBX_VERSION_M5,
    UBX_VERSION_M6,
    UBX_VERSION_M7,
    UBX_VERSION_M8,
    UBX_VERSION_M9,
    UBX_VERSION_M10,
    UBX_VERSION_COUNT
} ubloxVersion_e;

typedef enum {
    CFG_RATE_MEAS = 0x30210001, // U2
    CFG_RATE_NAV = 0x30210002, // U2
    CFG_RATE_TIMEREF = 0x20210003, // E1
    CFG_NAVSPG_FIXMODE = 0x20110011, // E1
    CFG_NAVSPG_DYNMODEL = 0x20110021, // E1
    CFG_NAVSPG_UTCSTANDARD = 0x2011001c, // E1
    CFG_NAVSPG_CONSTR_ALT = 0x401100c1, // I4
    CFG_NAVSPG_CONSTR_ALTVAR = 0x401100c2, // U4
    CFG_NAVSPG_CONSTR_DGNSSTO = 0x201100c4, // U1
    CFG_NAVSPG_INFIL_MINELEV = 0x201100a4, // I1
    CFG_NAVSPG_INFIL_CNOTHRS = 0x201100ab, // U1
    CFG_NAVSPG_INFIL_NCNOTHRS = 0x201100aa, // U1
    CFG_NAVSPG_OUTFIL_PDOP = 0x301100b1, // U2
    CFG_NAVSPG_OUTFIL_TDOP = 0x301100b2, // U2
    CFG_NAVSPG_OUTFIL_PACC = 0x301100b3, // U2
    CFG_NAVSPG_OUTFIL_TACC = 0x301100b4, // U2
    CFG_MOT_GNSSSPEED_THRS = 0x20250038, // U1
    CFG_MOT_GNSSDIST_THRS = 0x3025003b, // U2
    CFG_ANA_USE_ANA = 0x10230001, // L
    CFG_MSGOUT_NMEA_ID_VTG_I2C = 0x209100b0, // U1
    CFG_MSGOUT_NMEA_ID_VTG_SPI = 0x209100b4, // U1
    CFG_MSGOUT_NMEA_ID_VTG_UART1 = 0x209100b1, // U1
    CFG_MSGOUT_NMEA_ID_GSV_I2C = 0x209100c4, // U1
    CFG_MSGOUT_NMEA_ID_GSV_SPI = 0x209100c8, // U1
    CFG_MSGOUT_NMEA_ID_GSV_UART1 = 0x209100c5, // U1
    CFG_MSGOUT_NMEA_ID_GLL_I2C = 0x209100c9, // U1
    CFG_MSGOUT_NMEA_ID_GLL_SPI = 0x209100cd, // U1
    CFG_MSGOUT_NMEA_ID_GLL_UART1 = 0x209100ca, // U1
    CFG_MSGOUT_NMEA_ID_GGA_I2C = 0x209100ba, // U1
    CFG_MSGOUT_NMEA_ID_GGA_SPI = 0x209100be, // U1
    CFG_MSGOUT_NMEA_ID_GGA_UART1 = 0x209100bb, // U1
    CFG_MSGOUT_NMEA_ID_GSA_I2C = 0x209100bf, // U1
    CFG_MSGOUT_NMEA_ID_GSA_SPI = 0x209100c3, // U1
    CFG_MSGOUT_NMEA_ID_GSA_UART1 = 0x209100c0, // U1
    CFG_MSGOUT_NMEA_ID_RMC_I2C = 0x209100ab, // U1
    CFG_MSGOUT_NMEA_ID_RMC_SPI = 0x209100af, // U1
    CFG_MSGOUT_NMEA_ID_RMC_UART1 = 0x209100ac, // U1
    CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006, // U1
    CFG_MSGOUT_UBX_NAV_PVT_SPI = 0x2091000a, // U1
    CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007, // U1
    CFG_MSGOUT_UBX_NAV_SAT_I2C = 0x20910015, // U1
    CFG_MSGOUT_UBX_NAV_SAT_SPI = 0x20910019, // U1
    CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016, // U1
    CFG_MSGOUT_UBX_NAV_DOP_I2C = 0x20910038, // U1
    CFG_MSGOUT_UBX_NAV_DOP_SPI = 0x2091003c, // U1
    CFG_MSGOUT_UBX_NAV_DOP_UART1 = 0x20910039, // U1
    CFG_SBAS_USE_TESTMODE = 0x10360002, // L
    CFG_SBAS_USE_RANGING = 0x10360003, // L
    CFG_SBAS_USE_DIFFCORR = 0x10360004, // L
    CFG_SBAS_USE_INTEGRITY = 0x10360005, // L
    CFG_SBAS_PRNSCANMASK = 0x50360006, // X8
    CFG_SIGNAL_GPS_ENA = 0x1031001f, // L
    CFG_SIGNAL_SBAS_ENA = 0x10310020, // L
    CFG_SIGNAL_GAL_ENA = 0x10310021, // L
    CFG_SIGNAL_BDS_ENA = 0x10310022, // L
    CFG_SIGNAL_QZSS_ENA = 0x10310024, // L
    CFG_SIGNAL_GLO_ENA = 0x10310025, // L
    CFG_PM_OPERATEMODE = 0x20d00001, // E1
} ubxValGetSetBytes_e;

/*
 * replaced by following macro, to save space; keeping for reference
typedef enum {
    SBAS_SEARCH_ALL = 0,
    SBAS_SEARCH_PRN120,
    SBAS_SEARCH_PRN121,
    // repeat to 158
    SBAS_SEARCH_PRN158,
} ubxSbasPrnScan_e;
*/

#define SBAS_SEARCH_ALL    0x0
#define SBAS_SEARCH_PRN(i) (1 << (i - 120))

typedef enum {
    UBX_VAL_LAYER_RAM = 0x01,
    UBX_VAL_LAYER_BBR = 0x02,
    UBX_VAL_LAYER_FLASH = 0x04,
} ubloxValLayer_e;

typedef enum {
    UBLOX_MODEL_PORTABLE = 0,
    UBLOX_MODEL_STATIONARY,
    UBLOX_MODEL_PEDESTRIAN,
    UBLOX_MODEL_AUTOMOTIVE,
    UBLOX_MODEL_AT_SEA,
    UBLOX_MODEL_AIRBORNE_1G,
    UBLOX_MODEL_AIRBORNE_2G,
    UBLOX_MODEL_AIRBORNE_4G,
} ubloxModel_e;

typedef enum {
    UBLOX_UTC_STANDARD_AUTO = 0,
    UBLOX_UTC_STANDARD_USNO = 3,
    UBLOX_UTC_STANDARD_EU = 5,
    UBLOX_UTC_STANDARD_SU = 6,
    UBLOX_UTC_STANDARD_NTSC = 7,
} ubloxUtcStandard_e;

struct ubloxVersion_s {
    uint32_t hw;
    const char* str;
};
extern struct ubloxVersion_s ubloxVersionMap[];

#endif // USE_GPS_UBLOX

typedef enum {
    GPS_STATE_UNKNOWN = 0,
    GPS_STATE_DETECT_BAUD,
    GPS_STATE_INITIALIZED,
    GPS_STATE_CHANGE_BAUD,
    GPS_STATE_CONFIGURE,
    GPS_STATE_RECEIVING_DATA,
    GPS_STATE_PROCESS_DATA,
    GPS_STATE_LOST_COMMUNICATION,
    GPS_STATE_COUNT
} gpsState_e;

typedef enum {
    GPS_LATITUDE,
    GPS_LONGITUDE
} gpsCoordinateType_e;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MSP
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
    GPS_BAUDRATE_9600
} gpsBaudRate_e;

typedef enum {
    GPS_AUTOCONFIG_OFF = 0,
    GPS_AUTOCONFIG_ON
} gpsAutoConfig_e;

typedef enum {
    GPS_AUTOBAUD_OFF = 0,
    GPS_AUTOBAUD_ON
} gpsAutoBaud_e;

typedef enum {
    UBLOX_ACK_IDLE = 0,
    UBLOX_ACK_WAITING,
    UBLOX_ACK_GOT_ACK,
    UBLOX_ACK_GOT_NACK
} ubloxAckState_e;

#define GPS_BAUDRATE_MAX GPS_BAUDRATE_9600

typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;

/* LLH Location in NEU axis system */
typedef struct gpsLocation_s {
    int32_t lat;                    // latitude * 1e+7
    int32_t lon;                    // longitude * 1e+7
    int32_t altCm;                  // altitude in 0.01m
} gpsLocation_t;

/* A value below 100 means great accuracy is possible with GPS satellite constellation */
typedef struct gpsDilution_s {
    uint16_t pdop;                  // positional DOP - 3D (* 100)
    uint16_t hdop;                  // horizontal DOP - 2D (* 100)
    uint16_t vdop;                  // vertical DOP   - 1D (* 100)
} gpsDilution_t;

/* Only available on U-blox protocol */
typedef struct gpsAccuracy_s {
    uint32_t hAcc;                  // horizontal accuracy in mm
    uint32_t vAcc;                  // vertical accuracy in mm
    uint32_t sAcc;                  // speed accuracy in mm/s
} gpsAccuracy_t;

typedef struct gpsSolutionData_s {
    gpsLocation_t llh;
    gpsDilution_t dop;
    gpsAccuracy_t acc;
    uint16_t speed3d;               // speed in cm/s
    uint16_t groundSpeed;           // speed in cm/s
    uint16_t groundCourse;          // degrees * 10
    uint8_t numSat;
    uint32_t time;                  // GPS msToW
    uint32_t navIntervalMs;         // interval between nav solutions in ms
} gpsSolutionData_t;

/*
* keeping this table for reference
typedef enum {
    UBX_CAP_SAT_NONE = 0x0,
    UBX_CAP_SAT_GPS = 0x0001,
    UBX_CAP_SAT_GLO = 0x0002,
    UBX_CAP_SAT_GAL = 0x0004,
    UBX_CAP_SAT_BDS = 0x0008,
    UBX_CAP_SAT_SBAS = 0x0010,
    UBX_CAP_SAT_QZSS = 0x0020,
} ubxCapabilities_e;
*/

typedef struct ubxMonVer_s {
        char swVersion[30];
        char hwVersion[10];
        char extension[300]; // variable size but not more than 300 values
} ubxMonVer_t;

typedef struct gpsData_s {
    uint32_t errors;                // gps error counter - crc error/lost of data/sync etc..
    uint32_t timeouts;
    uint32_t lastNavMessage;        // time of last valid GPS speed and position data
    uint32_t now;
    uint32_t lastMessageSent;       // time last message was sent
    uint32_t state_position;        // incremental variable for loops
    uint32_t state_ts;              // timestamp for last state_position increment
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t userBaudRateIndex;      // index into auto-detecting or current baudrate
    uint8_t tempBaudRateIndex;      // index into auto-detecting or current baudrate

    uint8_t ackWaitingMsgId;        // Message id when waiting for ACK
    ubloxAckState_e ackState;       // Ack State
    uint8_t updateRateHz;
    bool ubloxM7orAbove;
    bool ubloxM8orAbove;
    bool ubloxM9orAbove;
    bool ubloxUsingFlightModel;     // false = Acquire model, true = Flight model
#ifdef USE_GPS_UBLOX
    uint32_t lastNavSolTs;          // time stamp of last UBCX message.  Used to calculate message delta
    ubloxVersion_e platformVersion; // module platform version, mapped from reported hardware version
#endif
} gpsData_t;

extern gpsLocation_t GPS_home_llh;
extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern uint32_t GPS_distanceToHomeCm;      // distance to home point in cm
extern int16_t GPS_directionToHome;        // direction to home point in degrees * 10
extern uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters

typedef enum {
    GPS_DIRECT_TICK = 1 << 0,
    GPS_MSP_UPDATE = 1 << 1
} gpsUpdateToggle_e;

extern gpsData_t gpsData;
extern gpsSolutionData_t gpsSol;

#define GPS_SV_MAXSATS_LEGACY   16U
#define GPS_SV_MAXSATS_M8N      32U                     // must be larger than MAXSATS_LEGACY

extern uint8_t GPS_update;                              // toggles on GPS nav position update (directly or via MSP)

extern uint8_t GPS_numCh;                               // Number of svinfo channels

typedef struct GPS_svinfo_s {
    uint8_t chn;      // When NumCh is 16 or less: Channel number
                      // When NumCh is more than 16: GNSS Id
                      //   0 = GPS, 1 = SBAS, 2 = Galileo, 3 = BeiDou
                      //   4 = IMES, 5 = QZSS, 6 = Glonass
    uint8_t svid;     // Satellite ID
    uint8_t quality;  // When NumCh is 16 or less: Bitfield Qualtity
                      // When NumCh is more than 16: flags
                      //   bits 2..0: signal quality indicator
                      //     0 = no signal
                      //     1 = searching signal
                      //     2 = signal acquired
                      //     3 = signal detected but unusable
                      //     4 = code locked and time synchronized
                      //     5,6,7 = code and carrier locked and time synchronized
                      //   bit 3:
                      //     1 = signal currently being used for navigaion
                      //   bits 5..4: signal health flag
                      //     0 = unknown
                      //     1 = healthy
                      //     2 = unhealthy
                      //   bit 6:
                      //     1 = differential correction data available for this SV
                      //   bit 7:
                      //     1 = carrier smoothed pseudorange used
    uint8_t cno;      // Carrier to Noise Ratio (Signal Strength)
} GPS_svinfo_t;
extern GPS_svinfo_t GPS_svinfo[GPS_SV_MAXSATS_M8N];

#define TASK_GPS_RATE       100     // default update rate of GPS task
#define TASK_GPS_RATE_FAST  500    // update rate of GPS task while Rx buffer is not empty

#ifdef USE_DASHBOARD
// Data used *only* by the dashboard device (OLED display).
// Note this data should probably be in the dashboard module, not here. On the refactor list...

// OLED display shows a scrolling string of chars for each packet type/event received.
#define DASHBOARD_LOG_ERROR        '?'
#define DASHBOARD_LOG_IGNORED      '!'
#define DASHBOARD_LOG_SKIPPED      '>'
#define DASHBOARD_LOG_NMEA_GGA     'g'
#define DASHBOARD_LOG_NMEA_GSA     's'
#define DASHBOARD_LOG_NMEA_RMC     'r'
#define DASHBOARD_LOG_UBLOX_DOP    'D'
#define DASHBOARD_LOG_UBLOX_SOL    'O'
#define DASHBOARD_LOG_UBLOX_STATUS 'S'
#define DASHBOARD_LOG_UBLOX_SVINFO 'I'
#define DASHBOARD_LOG_UBLOX_POSLLH 'P'
#define DASHBOARD_LOG_UBLOX_VELNED 'V'
#define DASHBOARD_LOG_UBLOX_MONVER 'M'

#define GPS_PACKET_LOG_ENTRY_COUNT 21 // To make this useful we should log as many packets as we can fit characters a single line of a OLED display.
extern char dashboardGpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];  // OLED display of a char for each packet type/event received.
extern uint32_t dashboardGpsPacketCount;                        // Packet received count.
extern uint32_t dashboardGpsNavSvInfoRcvCount;                  // Count of times sat info received & updated.

#define GPS_DBHZ_MIN 0
#define GPS_DBHZ_MAX 55
#endif  // USE_DASHBOARD

void gpsInit(void);
void gpsUpdate(timeUs_t currentTimeUs);
bool gpsNewFrame(uint8_t c);
bool gpsIsHealthy(void); // Returns true when the gps state is RECEIVING_DATA
struct serialPort_s;
bool gpsPassthrough(struct serialPort_s *gpsPassthroughPort);
void onGpsNewData(void);
void GPS_reset_home_position(void);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_distance_cm_bearing(const gpsLocation_t *from, const gpsLocation_t *to, bool dist3d, uint32_t *dist, int32_t *bearing);
void GPS_latLongVectors(const gpsLocation_t *from, const gpsLocation_t *to, float *pNSDist, float *pEWDist);
void gpsSetFixState(bool state);

bool gpsHasNewData(uint16_t *stamp);
float getGpsDataIntervalSeconds(void);  // range 0.05 - 2.5s
float getGpsDataFrequencyHz(void);      // range 20Hz - 0.4Hz

baudRate_e getGpsPortActualBaudRateIndex(void);
