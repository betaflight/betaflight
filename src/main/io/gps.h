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

#include "pg/gps.h"

#define GPS_DEGREES_DIVIDER 10000000L
#define GPS_X 1
#define GPS_Y 0
#define GPS_MIN_SAT_COUNT 4      // number of sats to trigger low sat count sanity check

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
    UBLOX_AIRBORNE = 0,
    UBLOX_PEDESTRIAN,
    UBLOX_DYNAMIC
} ubloxMode_e;

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
    uint16_t speed3d;               // speed in 0.1m/s
    uint16_t groundSpeed;           // speed in 0.1m/s
    uint16_t groundCourse;          // degrees * 10
    uint8_t numSat;
} gpsSolutionData_t;

typedef struct gpsData_s {
    uint32_t errors;                // gps error counter - crc error/lost of data/sync etc..
    uint32_t timeouts;
    uint32_t lastMessage;           // last time valid GPS data was received (millis)
    uint32_t lastLastMessage;       // last-last valid GPS message. Used to calculate delta.

    uint32_t state_position;        // incremental variable for loops
    uint32_t state_ts;              // timestamp for last state_position increment
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate

    uint8_t ackWaitingMsgId;        // Message id when waiting for ACK
    uint8_t ackTimeoutCounter;      // Ack timeout counter
    ubloxAckState_e ackState;
    bool ubloxUsePVT;
    bool ubloxUseSAT;
} gpsData_t;

#define GPS_PACKET_LOG_ENTRY_COUNT 21 // To make this useful we should log as many packets as we can fit characters a single line of a OLED display.
extern char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];

extern int32_t GPS_home[2];
extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern uint32_t GPS_distanceToHomeCm;      // distance to home point in cm
extern int16_t GPS_directionToHome;        // direction to home or hol point in degrees
extern uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters
extern int16_t GPS_verticalSpeedInCmS;     // vertical speed in cm/s
extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction
extern float GPS_scaleLonDown;  // this is used to offset the shrinking longitude as we go towards the poles
extern int16_t nav_takeoff_bearing;

typedef enum {
    GPS_DIRECT_TICK = 1 << 0,
    GPS_MSP_UPDATE = 1 << 1
} gpsUpdateToggle_e;

extern gpsData_t gpsData;
extern gpsSolutionData_t gpsSol;

#define GPS_SV_MAXSATS_LEGACY   16U
#define GPS_SV_MAXSATS_M8N      32U
#define GPS_SV_MAXSATS_M9N      42U

extern uint8_t GPS_update;       // toogle to distinct a GPS position update (directly or via MSP)
extern uint32_t GPS_packetCount;
extern uint32_t GPS_svInfoReceivedCount;
extern uint8_t GPS_numCh;                               // Number of channels
extern uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS_M8N];      // When NumCh is 16 or less: Channel number
                                                        // When NumCh is more than 16: GNSS Id
                                                        //   0 = GPS, 1 = SBAS, 2 = Galileo, 3 = BeiDou
                                                        //   4 = IMES, 5 = QZSS, 6 = Glonass
extern uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS_M8N];     // Satellite ID
extern uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS_M8N];  // When NumCh is 16 or less: Bitfield Qualtity
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
extern uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS_M8N];      // Carrier to Noise Ratio (Signal Strength)

#define GPS_DBHZ_MIN 0
#define GPS_DBHZ_MAX 55

#define TASK_GPS_RATE       100
#define TASK_GPS_RATE_FAST  1000

void gpsInit(void);
void gpsUpdate(timeUs_t currentTimeUs);
bool gpsNewFrame(uint8_t c);
bool gpsIsHealthy(void); // Check for healthy communications
struct serialPort_s;
void gpsEnablePassthrough(struct serialPort_s *gpsPassthroughPort);
void onGpsNewData(void);
void GPS_reset_home_position(void);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing);
void gpsSetFixState(bool state);
float gpsGetSampleRateHz(void);
