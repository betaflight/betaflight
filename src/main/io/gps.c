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

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/serial.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/gps_rescue.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"

#define LOG_ERROR        '?'
#define LOG_IGNORED      '!'
#define LOG_SKIPPED      '>'
#define LOG_NMEA_GGA     'g'
#define LOG_NMEA_GSA     's'
#define LOG_NMEA_RMC     'r'
#define LOG_UBLOX_DOP    'D'
#define LOG_UBLOX_SOL    'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'

#define DEBUG_SERIAL_BAUD  0 // set to 1 to debug serial port baud config (/100)
#define DEBUG_UBLOX_INIT   0 // set to 1 to debug ublox initialization
#define DEBUG_UBLOX_FRAMES 0 // set to 1 to debug ublox received frames

char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *gpsPacketLogChar = gpsPacketLog;
// **********************
// GPS
// **********************
int32_t GPS_home[2];
uint16_t GPS_distanceToHome;        // distance to home point in meters
uint32_t GPS_distanceToHomeCm;
int16_t GPS_directionToHome;        // direction to home or hol point in degrees * 10
uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters
int16_t GPS_verticalSpeedInCmS;     // vertical speed in cm/s
int16_t nav_takeoff_bearing;

#define GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S 15 // 0.54 km/h 0.335 mph

gpsSolutionData_t gpsSol;
uint32_t GPS_packetCount = 0;
uint32_t GPS_svInfoReceivedCount = 0; // SV = Space Vehicle, counter increments each time SV info is received.
uint8_t GPS_update = 0;             // toogle to distinct a GPS position update (directly or via MSP)

uint8_t GPS_numCh;                              // Details on numCh/svinfo in gps.h
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS_M8N];
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS_M8N];
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS_M8N];
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS_M8N];

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)
// Timeout for waiting ACK/NAK in GPS task cycles (0.25s at 100Hz)
#define UBLOX_ACK_TIMEOUT_MAX_COUNT (25)

static serialPort_t *gpsPort;
static float gpsSampleRateHz;

typedef struct gpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex; // see baudRate_e
    const char *ubx;
    const char *mtk;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUDRATE_115200,   BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600,    BAUD_57600,  "$PUBX,41,1,0003,0001,57600,0*2D\r\n",  "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400,    BAUD_38400,  "$PUBX,41,1,0003,0001,38400,0*26\r\n",  "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200,    BAUD_19200,  "$PUBX,41,1,0003,0001,19200,0*23\r\n",  "$PMTK251,19200*22\r\n" },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUDRATE_9600,     BAUD_9600,   "$PUBX,41,1,0003,0001,9600,0*16\r\n",   "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT ARRAYLEN(gpsInitData)

#define DEFAULT_BAUD_RATE_INDEX 0

#ifdef USE_GPS_UBLOX
typedef enum {
    PREAMBLE1 = 0xB5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x02,
    MSG_STATUS = 0x03,
    MSG_DOP = 0x04,
    MSG_SOL = 0x06,
    MSG_PVT = 0x07,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_SAT = 0x35,
    MSG_CFG_MSG = 0x01,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_SBAS = 0x16,
    MSG_CFG_NAV_SETTINGS = 0x24,
    MSG_CFG_GNSS = 0x3E
} ubxProtocolBytes_e;

#define UBLOX_MODE_ENABLED    0x1
#define UBLOX_MODE_TEST       0x2

#define UBLOX_USAGE_RANGE     0x1
#define UBLOX_USAGE_DIFFCORR  0x2
#define UBLOX_USAGE_INTEGRITY 0x4

#define UBLOX_GNSS_ENABLE     0x1
#define UBLOX_GNSS_DEFAULT_SIGCFGMASK 0x10000

#define UBLOX_DYNMODE_PEDESTRIAN  3
#define UBLOX_DYNMODE_AIRBORNE_1G 6
#define UBLOX_DYNMODE_AIRBORNE_4G 8

typedef struct ubxHeader_s {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubxHeader_t;

typedef struct ubxConfigblock_s {
    uint8_t gnssId;
    uint8_t resTrkCh;
    uint8_t maxTrkCh;
    uint8_t reserved1;
    uint32_t flags;
} ubxConfigblock_t;

typedef struct ubxPollMsg_s {
    uint8_t msgClass;
    uint8_t msgID;
} ubxPollMsg_t;

typedef struct ubxCfgMsg_s {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
} ubxCfgMsg_t;

typedef struct ubxCfgRate_s {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
} ubxCfgRate_t;

typedef struct ubxCfgSbas_s {
    uint8_t mode;
    uint8_t usage;
    uint8_t maxSBAS;
    uint8_t scanmode2;
    uint32_t scanmode1;
} ubxCfgSbas_t;

typedef struct ubxCfgGnss_s {
    uint8_t msgVer;
    uint8_t numTrkChHw;
    uint8_t numTrkChUse;
    uint8_t numConfigBlocks;
    ubxConfigblock_t configblocks[7];
} ubxCfgGnss_t;

typedef struct ubxCfgNav5_s {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDOP;
    uint16_t tDOP;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgnssTimeout;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    uint8_t reserved0[2];
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    uint8_t reserved1[5];
} ubxCfgNav5_t;

typedef union ubxPayload_s {
    ubxPollMsg_t poll_msg;
    ubxCfgMsg_t cfg_msg;
    ubxCfgRate_t cfg_rate;
    ubxCfgNav5_t cfg_nav5;
    ubxCfgSbas_t cfg_sbas;
    ubxCfgGnss_t cfg_gnss;
} ubxPayload_t;

typedef struct ubxMessage_s {
    ubxHeader_t header;
    ubxPayload_t payload;
} __attribute__((packed)) ubxMessage_t;

#endif // USE_GPS_UBLOX

typedef enum {
    GPS_STATE_UNKNOWN,
    GPS_STATE_INITIALIZING,
    GPS_STATE_INITIALIZED,
    GPS_STATE_CHANGE_BAUD,
    GPS_STATE_CONFIGURE,
    GPS_STATE_RECEIVING_DATA,
    GPS_STATE_LOST_COMMUNICATION,
    GPS_STATE_COUNT
} gpsState_e;

// Max time to wait for received data
#define GPS_MAX_WAIT_DATA_RX 30

gpsData_t gpsData;

static void shiftPacketLog(void)
{
    uint32_t i;

    for (i = ARRAYLEN(gpsPacketLog) - 1; i > 0 ; i--) {
        gpsPacketLog[i] = gpsPacketLog[i-1];
    }
}

static bool isConfiguratorConnected(void)
{
    return (getArmingDisableFlags() & ARMING_DISABLED_MSP);
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
    gpsData.lastMessage = millis();
    sensorsClear(SENSOR_GPS);

    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
    gpsData.ackState = UBLOX_ACK_IDLE;
}

void gpsInit(void)
{
    gpsSampleRateHz = 0.0f;

    gpsData.baudrateIndex = 0;
    gpsData.errors = 0;
    gpsData.timeouts = 0;

    memset(gpsPacketLog, 0x00, sizeof(gpsPacketLog));

    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_STATE_UNKNOWN);

    gpsData.lastMessage = millis();

    if (gpsConfig()->provider == GPS_MSP) { // no serial ports used when GPS_MSP is configured
        gpsSetState(GPS_STATE_INITIALIZED);
        return;
    }

    const serialPortConfig_t *gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
    if (!gpsPortConfig) {
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
#if defined(GPS_NMEA_TX_ONLY)
    if (gpsConfig()->provider == GPS_NMEA) {
        mode &= ~MODE_TX;
    }
#endif

    // no callback - buffer will be consumed in gpsUpdate()
    gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, NULL, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex], mode, SERIAL_NOT_INVERTED);
    if (!gpsPort) {
        return;
    }

    // signal GPS "thread" to initialize when it gets to it
    gpsSetState(GPS_STATE_INITIALIZING);
}

#ifdef USE_GPS_NMEA
void gpsInitNmea(void)
{
    static bool atgmRestartDone = false;
#if !defined(GPS_NMEA_TX_ONLY)
    uint32_t now;
#endif
    switch (gpsData.state) {
        case GPS_STATE_INITIALIZING:
#if !defined(GPS_NMEA_TX_ONLY)
           now = millis();
           if (now - gpsData.state_ts < 1000) {
               return;
           }
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
               gpsSetState(GPS_STATE_CHANGE_BAUD);
           }
           break;
#endif
        case GPS_STATE_CHANGE_BAUD:
#if !defined(GPS_NMEA_TX_ONLY)
           now = millis();
           if (now - gpsData.state_ts < 1000) {
               return;
           }
           gpsData.state_ts = now;
           if (gpsData.state_position < 1) {
               serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
               gpsData.state_position++;
           } else if (gpsData.state_position < 2) {
               serialPrint(gpsPort, "$PSRF103,00,6,00,0*23\r\n");
                // special initialization for NMEA ATGM336 and similar GPS recivers - should be done only once
               if (!atgmRestartDone) {
                   atgmRestartDone = true;
                   serialPrint(gpsPort, "$PCAS02,100*1E\r\n");  // 10Hz refresh rate
                   serialPrint(gpsPort, "$PCAS10,0*1C\r\n");    // hot restart 
               }
               gpsData.state_position++;
           } else
#else
           {
               serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
           }
#endif
               gpsSetState(GPS_STATE_RECEIVING_DATA);
            break;
    }
}
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
static void ubloxSendByteUpdateChecksum(const uint8_t data, uint8_t *checksumA, uint8_t *checksumB)
{
    *checksumA += data;
    *checksumB += *checksumA;
    serialWrite(gpsPort, data);
}

static void ubloxSendDataUpdateChecksum(const uint8_t *data, uint8_t len, uint8_t *checksumA, uint8_t *checksumB)
{
    while (len--) {
        ubloxSendByteUpdateChecksum(*data, checksumA, checksumB);
        data++;
    }
}

static void ubloxSendMessage(const uint8_t *data, uint8_t len)
{
    uint8_t checksumA = 0, checksumB = 0;
    serialWrite(gpsPort, data[0]);
    serialWrite(gpsPort, data[1]);
    ubloxSendDataUpdateChecksum(&data[2], len - 2, &checksumA, &checksumB);
    serialWrite(gpsPort, checksumA);
    serialWrite(gpsPort, checksumB);

    // Save state for ACK waiting
    gpsData.ackWaitingMsgId = data[3]; //save message id for ACK
    gpsData.ackTimeoutCounter = 0;
    gpsData.ackState = UBLOX_ACK_WAITING;
}

static void ubloxSendConfigMessage(ubxMessage_t *message, uint8_t msg_id, uint8_t length)
{
    message->header.preamble1 = PREAMBLE1;
    message->header.preamble2 = PREAMBLE2;
    message->header.msg_class = CLASS_CFG;
    message->header.msg_id = msg_id;
    message->header.length = length;
    ubloxSendMessage((const uint8_t *) message, length + 6);
}

static void ubloxSendPollMessage(uint8_t msg_id)
{
    ubxMessage_t tx_buffer;
    tx_buffer.header.preamble1 = PREAMBLE1;
    tx_buffer.header.preamble2 = PREAMBLE2;
    tx_buffer.header.msg_class = CLASS_CFG;
    tx_buffer.header.msg_id = msg_id;
    tx_buffer.header.length = 0;
    ubloxSendMessage((const uint8_t *) &tx_buffer, 6);
}

static void ubloxSendNAV5Message(bool airborne)
{
    ubxMessage_t tx_buffer;
    tx_buffer.payload.cfg_nav5.mask = 0xFFFF;
    if (airborne) {
#if defined(GPS_UBLOX_MODE_AIRBORNE_1G)
        tx_buffer.payload.cfg_nav5.dynModel = UBLOX_DYNMODE_AIRBORNE_1G;
#else
        tx_buffer.payload.cfg_nav5.dynModel = UBLOX_DYNMODE_AIRBORNE_4G;
#endif
    } else {
        tx_buffer.payload.cfg_nav5.dynModel = UBLOX_DYNMODE_PEDESTRIAN;
    }
    tx_buffer.payload.cfg_nav5.fixMode = 3;
    tx_buffer.payload.cfg_nav5.fixedAlt = 0;
    tx_buffer.payload.cfg_nav5.fixedAltVar = 10000;
    tx_buffer.payload.cfg_nav5.minElev = 5;
    tx_buffer.payload.cfg_nav5.drLimit = 0;
    tx_buffer.payload.cfg_nav5.pDOP = 250;
    tx_buffer.payload.cfg_nav5.tDOP = 250;
    tx_buffer.payload.cfg_nav5.pAcc = 100;
    tx_buffer.payload.cfg_nav5.tAcc = 300;
    tx_buffer.payload.cfg_nav5.staticHoldThresh = 0;
    tx_buffer.payload.cfg_nav5.dgnssTimeout = 60;
    tx_buffer.payload.cfg_nav5.cnoThreshNumSVs = 0;
    tx_buffer.payload.cfg_nav5.cnoThresh = 0;
    tx_buffer.payload.cfg_nav5.reserved0[0] = 0;
    tx_buffer.payload.cfg_nav5.reserved0[1] = 0;
    tx_buffer.payload.cfg_nav5.staticHoldMaxDist = 200;
    tx_buffer.payload.cfg_nav5.utcStandard = 0;
    tx_buffer.payload.cfg_nav5.reserved1[0] = 0;
    tx_buffer.payload.cfg_nav5.reserved1[1] = 0;
    tx_buffer.payload.cfg_nav5.reserved1[2] = 0;
    tx_buffer.payload.cfg_nav5.reserved1[3] = 0;
    tx_buffer.payload.cfg_nav5.reserved1[4] = 0;

    ubloxSendConfigMessage(&tx_buffer, MSG_CFG_NAV_SETTINGS, sizeof(ubxCfgNav5_t));
}

static void ubloxSetMessageRate(uint8_t messageClass, uint8_t messageID, uint8_t rate)
{
    ubxMessage_t tx_buffer;
    tx_buffer.payload.cfg_msg.msgClass = messageClass;
    tx_buffer.payload.cfg_msg.msgID = messageID;
    tx_buffer.payload.cfg_msg.rate = rate;
    ubloxSendConfigMessage(&tx_buffer, MSG_CFG_MSG, sizeof(ubxCfgMsg_t));
}

static void ubloxSetNavRate(uint16_t measRate, uint16_t navRate, uint16_t timeRef)
{
    ubxMessage_t tx_buffer;
    tx_buffer.payload.cfg_rate.measRate = measRate;
    tx_buffer.payload.cfg_rate.navRate = navRate;
    tx_buffer.payload.cfg_rate.timeRef = timeRef;
    ubloxSendConfigMessage(&tx_buffer, MSG_CFG_RATE, sizeof(ubxCfgRate_t));
}

static void ubloxSetSbas(void)
{
    ubxMessage_t tx_buffer;

    //NOTE: default ublox config for sbas mode is: UBLOX_MODE_ENABLED, test is disabled
    tx_buffer.payload.cfg_sbas.mode = UBLOX_MODE_TEST;
    if (gpsConfig()->sbasMode != SBAS_NONE) {
        tx_buffer.payload.cfg_sbas.mode |= UBLOX_MODE_ENABLED;
    }

    //NOTE: default ublox config for sbas mode is: UBLOX_USAGE_RANGE | UBLOX_USAGE_DIFFCORR, integrity is disabled
    tx_buffer.payload.cfg_sbas.usage = UBLOX_USAGE_RANGE | UBLOX_USAGE_DIFFCORR;
    if (gpsConfig()->sbas_integrity) {
        tx_buffer.payload.cfg_sbas.usage |= UBLOX_USAGE_INTEGRITY;
    }

    tx_buffer.payload.cfg_sbas.maxSBAS = 3;
    tx_buffer.payload.cfg_sbas.scanmode2 = 0;
    switch (gpsConfig()->sbasMode) {
        case SBAS_AUTO:
            tx_buffer.payload.cfg_sbas.scanmode1 = 0;
            break;
        case SBAS_EGNOS:
            tx_buffer.payload.cfg_sbas.scanmode1 = 0x00010048; //PRN123, PRN126, PRN136
            break;
        case SBAS_WAAS:
            tx_buffer.payload.cfg_sbas.scanmode1 = 0x0004A800; //PRN131, PRN133, PRN135, PRN138
            break;
        case SBAS_MSAS:
            tx_buffer.payload.cfg_sbas.scanmode1 = 0x00020200; //PRN129, PRN137
            break;
        case SBAS_GAGAN:
            tx_buffer.payload.cfg_sbas.scanmode1 = 0x00001180; //PRN127, PRN128, PRN132
            break;
        default:
            tx_buffer.payload.cfg_sbas.scanmode1 = 0;
            break;
    }
    ubloxSendConfigMessage(&tx_buffer, MSG_CFG_SBAS, sizeof(ubxCfgSbas_t));
}

void gpsInitUblox(void)
{
    uint32_t now;
    // UBX will run at the serial port's baudrate, it shouldn't be "autodetected". So here we force it to that rate

    // Wait until GPS transmit buffer is empty
    if (!isSerialTransmitBufferEmpty(gpsPort))
        return;

    switch (gpsData.state) {
        case GPS_STATE_INITIALIZING:
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
#if DEBUG_SERIAL_BAUD
                    debug[0] = baudRates[newBaudRateIndex] / 100;
#endif
                    return;
                }

                // print our FIXED init string for the baudrate we want to be at
                serialPrint(gpsPort, gpsInitData[gpsData.baudrateIndex].ubx);

                gpsData.state_position++;
            } else {
                // we're now (hopefully) at the correct rate, next state will switch to it
                gpsSetState(GPS_STATE_CHANGE_BAUD);
            }
            break;

        case GPS_STATE_CHANGE_BAUD:
            serialSetBaudRate(gpsPort, baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex]);
#if DEBUG_SERIAL_BAUD
            debug[0] = baudRates[gpsInitData[gpsData.baudrateIndex].baudrateIndex] / 100;
#endif
            gpsSetState(GPS_STATE_CONFIGURE);
            break;

        case GPS_STATE_CONFIGURE:
            // Either use specific config file for GPS or let dynamically upload config
            if (gpsConfig()->autoConfig == GPS_AUTOCONFIG_OFF) {
                gpsSetState(GPS_STATE_RECEIVING_DATA);
                break;
            }

            if (gpsData.ackState == UBLOX_ACK_IDLE) {
                switch (gpsData.state_position) {
                    case 0:
                        gpsData.ubloxUsePVT = true;
                        gpsData.ubloxUseSAT = true;
                        ubloxSendNAV5Message(gpsConfig()->gps_ublox_mode == UBLOX_AIRBORNE);
                        break;
                    case 1: //Disable NMEA Messages
                        ubloxSetMessageRate(0xF0, 0x05, 0); // VGS: Course over ground and Ground speed
                        break;
                    case 2:
                        ubloxSetMessageRate(0xF0, 0x03, 0); // GSV: GNSS Satellites in View
                        break;
                    case 3:
                        ubloxSetMessageRate(0xF0, 0x01, 0); // GLL: Latitude and longitude, with time of position fix and status
                        break;
                    case 4:
                        ubloxSetMessageRate(0xF0, 0x00, 0); // GGA: Global positioning system fix data
                        break;
                    case 5:
                        ubloxSetMessageRate(0xF0, 0x02, 0); // GSA: GNSS DOP and Active Satellites
                        break;
                    case 6:
                        ubloxSetMessageRate(0xF0, 0x04, 0); // RMC: Recommended Minimum data
                        break;
                    case 7: //Enable UBLOX Messages
                        if (gpsData.ubloxUsePVT) {
                            ubloxSetMessageRate(CLASS_NAV, MSG_PVT, 1); // set PVT MSG rate
                        } else {
                            ubloxSetMessageRate(CLASS_NAV, MSG_SOL, 1); // set SOL MSG rate
                        }
                        break;
                    case 8:
                        if (gpsData.ubloxUsePVT) {
                            gpsData.state_position++;
                        } else {
                           ubloxSetMessageRate(CLASS_NAV, MSG_POSLLH, 1); // set POSLLH MSG rate
                        }
                        break;
                    case 9:
                        if (gpsData.ubloxUsePVT) {
                            gpsData.state_position++;
                        } else {
                            ubloxSetMessageRate(CLASS_NAV, MSG_STATUS, 1); // set STATUS MSG rate
                        }
                        break;
                    case 10:
                        if (gpsData.ubloxUsePVT) {
                            gpsData.state_position++;
                        } else {
                            ubloxSetMessageRate(CLASS_NAV, MSG_VELNED, 1); // set VELNED MSG rate
                        }
                        break;
                    case 11:
                        if (gpsData.ubloxUseSAT) {
                            ubloxSetMessageRate(CLASS_NAV, MSG_SAT, 5); // set SAT MSG rate (every 5 cycles)
                        } else {
                            ubloxSetMessageRate(CLASS_NAV, MSG_SVINFO, 5); // set SVINFO MSG rate (every 5 cycles)
                        }
                        break;
                    case 12:
                        ubloxSetMessageRate(CLASS_NAV, MSG_DOP, 1); // set DOP MSG rate
                        break;
                    case 13:
                        ubloxSetNavRate(0x64, 1, 1); // set rate to 10Hz (measurement period: 100ms, navigation rate: 1 cycle) (for 5Hz use 0xC8)
                        break;
                    case 14:
                        ubloxSetSbas();
                        break;
                    case 15:
                        if ((gpsConfig()->sbasMode == SBAS_NONE) || (gpsConfig()->gps_ublox_use_galileo)) {
                            ubloxSendPollMessage(MSG_CFG_GNSS);
                        } else {
                            gpsSetState(GPS_STATE_RECEIVING_DATA);
                        }
                        break;
                    default:
                        break;
                }
            }

            switch (gpsData.ackState) {
                case UBLOX_ACK_IDLE:
                    break;
                case UBLOX_ACK_WAITING:
                    if ((++gpsData.ackTimeoutCounter) == UBLOX_ACK_TIMEOUT_MAX_COUNT) {
                        gpsSetState(GPS_STATE_LOST_COMMUNICATION);
                    }
                    break;
                case UBLOX_ACK_GOT_ACK:
                    if (gpsData.state_position == 15) {
                        // ublox should be initialised, try receiving
                        gpsSetState(GPS_STATE_RECEIVING_DATA);
                    } else {
                        gpsData.state_position++;
                        gpsData.ackState = UBLOX_ACK_IDLE;
                    }
                    break;
                case UBLOX_ACK_GOT_NACK:
                    if (gpsData.state_position == 7) { // If we were asking for NAV-PVT...
                        gpsData.ubloxUsePVT = false;   // ...retry asking for NAV-SOL
                        gpsData.ackState = UBLOX_ACK_IDLE;
                    } else {
                        if (gpsData.state_position == 11) { // If we were asking for NAV-SAT...
                            gpsData.ubloxUseSAT = false;   // ...retry asking for NAV-SVINFO
                            gpsData.ackState = UBLOX_ACK_IDLE;
                        } else {
                            gpsSetState(GPS_STATE_CONFIGURE);
                        }
                    }
                    break;
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
    default:
        break;
    }
}

static void updateGpsIndicator(timeUs_t currentTimeUs)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && STATE(GPS_FIX) && (gpsSol.numSat >= gpsRescueConfig()->minSats)) {
        GPSLEDTime = currentTimeUs + 150000;
        LED1_TOGGLE;
    }
}

void gpsUpdate(timeUs_t currentTimeUs)
{
    static gpsState_e gpsStateDurationUs[GPS_STATE_COUNT];
    timeUs_t executeTimeUs;
    gpsState_e gpsCurrentState = gpsData.state;

    // read out available GPS bytes
    if (gpsPort) {
        while (serialRxBytesWaiting(gpsPort)) {
            if (cmpTimeUs(micros(), currentTimeUs) > GPS_MAX_WAIT_DATA_RX) {
                // Wait 1ms and come back
                rescheduleTask(TASK_SELF, TASK_PERIOD_HZ(TASK_GPS_RATE_FAST));
                return;
            }
            gpsNewData(serialRead(gpsPort));
        }
        // Restore default task rate
        rescheduleTask(TASK_SELF, TASK_PERIOD_HZ(TASK_GPS_RATE));
   } else if (GPS_update & GPS_MSP_UPDATE) { // GPS data received via MSP
        gpsSetState(GPS_STATE_RECEIVING_DATA);
        onGpsNewData();
        GPS_update &= ~GPS_MSP_UPDATE;
    }

#if DEBUG_UBLOX_INIT
    debug[0] = gpsData.state;
    debug[1] = gpsData.state_position;
    debug[2] = gpsData.ackState;
#endif

    switch (gpsData.state) {
        case GPS_STATE_UNKNOWN:
        case GPS_STATE_INITIALIZED:
            break;

        case GPS_STATE_INITIALIZING:
        case GPS_STATE_CHANGE_BAUD:
        case GPS_STATE_CONFIGURE:
            gpsInitHardware();
            break;

        case GPS_STATE_LOST_COMMUNICATION:
            gpsData.timeouts++;
            if (gpsConfig()->autoBaud) {
                // try another rate
                gpsData.baudrateIndex++;
                gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            gpsSol.numSat = 0;
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_STATE_INITIALIZING);
            break;

        case GPS_STATE_RECEIVING_DATA:
            // check for no data/gps timeout/cable disconnection etc
            if (millis() - gpsData.lastMessage > GPS_TIMEOUT) {
                gpsSetState(GPS_STATE_LOST_COMMUNICATION);
#ifdef USE_GPS_UBLOX
            } else {
                if (gpsConfig()->autoConfig == GPS_AUTOCONFIG_ON) { // Only if autoconfig is enabled
                    switch (gpsData.state_position) {
                        case 0:
                            if (!isConfiguratorConnected()) {
                                if (gpsData.ubloxUseSAT) {
                                    ubloxSetMessageRate(CLASS_NAV, MSG_SAT, 0); // disable SAT MSG
                                } else {
                                    ubloxSetMessageRate(CLASS_NAV, MSG_SVINFO, 0); // disable SVINFO MSG
                                }
                                gpsData.state_position = 1;
                            }
                            break;
                        case 1:
                            if (STATE(GPS_FIX) && (gpsConfig()->gps_ublox_mode == UBLOX_DYNAMIC)) {
                                ubloxSendNAV5Message(true);
                                gpsData.state_position = 2;
                            }
                            if (isConfiguratorConnected()) {
                                gpsData.state_position = 2;
                            }
                            break;
                        case 2:
                            if (isConfiguratorConnected()) {
                                if (gpsData.ubloxUseSAT) {
                                    ubloxSetMessageRate(CLASS_NAV, MSG_SAT, 5); // set SAT MSG rate (every 5 cycles)
                                } else {
                                    ubloxSetMessageRate(CLASS_NAV, MSG_SVINFO, 5); // set SVINFO MSG rate (every 5 cycles)
                                }
                                gpsData.state_position = 0;
                            }
                            break;
                    }
                }
#endif //USE_GPS_UBLOX
            }
            break;
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTimeUs);
    }

    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        // clear the home fix icon between arms if the user configuration is to reset home point between arms
        DISABLE_STATE(GPS_FIX_HOME);
    }

    static bool hasBeeped = false;
    if (!ARMING_FLAG(ARMED)) {
        // while disarmed, beep when requirements for a home fix are met
        // ?? should we also beep if home fix requirements first appear after arming?
        if (!hasBeeped && STATE(GPS_FIX) && gpsSol.numSat >= gpsRescueConfig()->minSats) {
            beeper(BEEPER_READY_BEEP);
            hasBeeped = true;
        }
    }

    DEBUG_SET(DEBUG_GPS_DOP, 0, gpsSol.numSat);
    DEBUG_SET(DEBUG_GPS_DOP, 1, gpsSol.dop.pdop);
    DEBUG_SET(DEBUG_GPS_DOP, 2, gpsSol.dop.hdop);
    DEBUG_SET(DEBUG_GPS_DOP, 3, gpsSol.dop.vdop);

    executeTimeUs = micros() - currentTimeUs;
    if (executeTimeUs > gpsStateDurationUs[gpsCurrentState]) {
        gpsStateDurationUs[gpsCurrentState] = executeTimeUs;
    }
    schedulerSetNextStateTime(gpsStateDurationUs[gpsData.state]);
}

static void gpsNewData(uint16_t c)
{
    if (!gpsNewFrame(c)) {
        return;
    }

    if (gpsData.state == GPS_STATE_RECEIVING_DATA) {
        // new data received and parsed, we're in business
        gpsData.lastLastMessage = gpsData.lastMessage;
        gpsData.lastMessage = millis();
        sensorsSet(SENSOR_GPS);
    }

    GPS_update ^= GPS_DIRECT_TICK;

#if DEBUG_UBLOX_INIT
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
    default:
        break;
    }
    return false;
}

// Check for healthy communications
bool gpsIsHealthy(void)
{
    return (gpsData.state == GPS_STATE_RECEIVING_DATA);
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
#define FRAME_GSA  4


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
    int isneg = 0;
    for (i = 0; src[i] != 0; i++) {
        if ((i == 0) && (src[0] == '-')) { // detect negative sign
            isneg = 1;
            continue; // jump to next character if the first one was a negative sign
        }
        if (src[i] == '.') {
            i++;
            if (mult == 0) {
                break;
            } else {
                src[i + mult] = 0;
            }
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9') {
            tmp += src[i] - '0';
        }
        if (i >= 15) {
            return 0; // out of bounds
        }
    }
    return isneg ? -tmp : tmp;    // handle negative altitudes
}

typedef struct gpsDataNmea_s {
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    int32_t altitudeCm;
    uint16_t speed;
    uint16_t pdop;
    uint16_t hdop;
    uint16_t vdop;
    uint16_t ground_course;
    uint32_t time;
    uint32_t date;
} gpsDataNmea_t;

static void parseFieldNmea(gpsDataNmea_t *data, char *str, uint8_t gpsFrame, uint8_t idx)
{
    static uint8_t svMessageNum = 0;
    uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;

    switch (gpsFrame) {

        case FRAME_GGA:        //************* GPGGA FRAME parsing
            switch (idx) {
    //          case 1:             // Time information
    //              break;
                case 2:
                    data->latitude = GPS_coord_to_degrees(str);
                    break;
                case 3:
                    if (str[0] == 'S') data->latitude *= -1;
                    break;
                case 4:
                    data->longitude = GPS_coord_to_degrees(str);
                    break;
                case 5:
                    if (str[0] == 'W') data->longitude *= -1;
                    break;
                case 6:
                    gpsSetFixState(str[0] > '0');
                    break;
                case 7:
                    data->numSat = grab_fields(str, 0);
                    break;
                case 9:
                    data->altitudeCm = grab_fields(str, 1) * 10;     // altitude in centimeters. Note: NMEA delivers altitude with 1 or 3 decimals. It's safer to cut at 0.1m and multiply by 10
                    break;
            }
            break;

        case FRAME_RMC:        //************* GPRMC FRAME parsing
            switch (idx) {
                case 1:
                    data->time = grab_fields(str, 2); // UTC time hhmmss.ss
                    break;
                case 7:
                    data->speed = ((grab_fields(str, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                    break;
                case 8:
                    data->ground_course = (grab_fields(str, 1));      // ground course deg * 10
                    break;
                case 9:
                    data->date = grab_fields(str, 0); // date dd/mm/yy
                    break;
            }
            break;

        case FRAME_GSV:
            switch (idx) {
                /*case 1:
                    // Total number of messages of this type in this cycle
                    break; */
                case 2:
                    // Message number
                    svMessageNum = grab_fields(str, 0);
                    break;
                case 3:
                    // Total number of SVs visible
                    GPS_numCh = MIN(grab_fields(str, 0), GPS_SV_MAXSATS_LEGACY);
                    break;
            }
            if (idx < 4)
                break;

            svPacketIdx = (idx - 4) / 4 + 1; // satellite number in packet, 1-4
            svSatNum    = svPacketIdx + (4 * (svMessageNum - 1)); // global satellite number
            svSatParam  = idx - 3 - (4 * (svPacketIdx - 1)); // parameter number for satellite

            if (svSatNum > GPS_SV_MAXSATS_LEGACY)
                break;

            switch (svSatParam) {
                case 1:
                    // SV PRN number
                    GPS_svinfo_chn[svSatNum - 1]  = svSatNum;
                    GPS_svinfo_svid[svSatNum - 1] = grab_fields(str, 0);
                    break;
                /*case 2:
                    // Elevation, in degrees, 90 maximum
                    break;
                case 3:
                    // Azimuth, degrees from True North, 000 through 359
                    break; */
                case 4:
                    // SNR, 00 through 99 dB (null when not tracking)
                    GPS_svinfo_cno[svSatNum - 1] = grab_fields(str, 0);
                    GPS_svinfo_quality[svSatNum - 1] = 0; // only used by ublox
                    break;
            }

            GPS_svInfoReceivedCount++;
            break;

        case FRAME_GSA:
            switch (idx) {
                case 15:
                    data->pdop = grab_fields(str, 2);  // pDOP * 100
                    break;
                case 16:
                    data->hdop = grab_fields(str, 2);  // hDOP * 100
                    break;
                case 17:
                    data->vdop = grab_fields(str, 2);  // vDOP * 100
                    break;
            }
            break;
    }
}

static bool writeGpsSolutionNmea(gpsSolutionData_t *sol, const gpsDataNmea_t *data, uint8_t gpsFrame)
{
    switch (gpsFrame) {

        case FRAME_GGA:
            *gpsPacketLogChar = LOG_NMEA_GGA;
            if (STATE(GPS_FIX)) {
                sol->llh.lat = data->latitude;
                sol->llh.lon = data->longitude;
                sol->numSat = data->numSat;
                sol->llh.altCm = data->altitudeCm;
            }
            // return only one true statement to trigger one "newGpsDataReady" flag per GPS loop
            return true;

        case FRAME_GSA:
            *gpsPacketLogChar = LOG_NMEA_GSA;
            sol->dop.pdop = data->pdop;
            sol->dop.hdop = data->hdop;
            sol->dop.vdop = data->vdop;
            return false;

        case FRAME_RMC:
            *gpsPacketLogChar = LOG_NMEA_RMC;
            sol->groundSpeed = data->speed;
            sol->groundCourse = data->ground_course;
#ifdef USE_RTC_TIME
            // This check will miss 00:00:00.00, but we shouldn't care - next report will be valid
            if(!rtcHasTime() && data->date != 0 && data->time != 0) {
                dateTime_t temp_time;
                temp_time.year = (data->date % 100) + 2000;
                temp_time.month = (data->date / 100) % 100;
                temp_time.day = (data->date / 10000) % 100;
                temp_time.hours = (data->time / 1000000) % 100;
                temp_time.minutes = (data->time / 10000) % 100;
                temp_time.seconds = (data->time / 100) % 100;
                temp_time.millis = (data->time & 100) * 10;
                rtcSetDateTime(&temp_time);
            }
#endif
            return false;

        default:
            return false;
    }
}

static bool gpsNewFrameNMEA(char c)
{
    static gpsDataNmea_t gps_msg;
    static char string[15];
    static uint8_t param = 0, offset = 0, parity = 0;
    static uint8_t checksum_param, gps_frame = NO_FRAME;
    bool isFrameOk = false;

    switch (c) {

        case '$':
            param = 0;
            offset = 0;
            parity = 0;
            break;

        case ',':
        case '*':
            string[offset] = 0;
            if (param == 0) {  // frame identification (5 chars, e.g. "GPGGA", "GNGGA", "GLGGA", ...)
                gps_frame = NO_FRAME;
                if (strcmp(&string[2], "GGA") == 0) {
                    gps_frame = FRAME_GGA;
                } else if (strcmp(&string[2], "RMC") == 0) {
                    gps_frame = FRAME_RMC;
                } else if (strcmp(&string[2], "GSV") == 0) {
                    gps_frame = FRAME_GSV;
                } else if (strcmp(&string[2], "GSA") == 0) {
                    gps_frame = FRAME_GSA;
                }
            }

            // parse string and write data into gps_msg
            parseFieldNmea(&gps_msg, string, gps_frame, param);

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
                    isFrameOk = writeGpsSolutionNmea(&gpsSol, &gps_msg, gps_frame);  // // write gps_msg into gpsSol
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
            break;
    }

    return isFrameOk;
}
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
// UBX support
typedef struct ubxNavPosllh_s {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitudeMslMm;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubxNavPosllh_t;

typedef struct ubxNavStatus_s {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubxNavStatus_t;

typedef struct ubxNavDop_s {
    uint32_t itow;              // GPS Millisecond Time of Week
    uint16_t gdop;              // Geometric DOP
    uint16_t pdop;              // Position DOP
    uint16_t tdop;              // Time DOP
    uint16_t vdop;              // Vertical DOP
    uint16_t hdop;              // Horizontal DOP
    uint16_t ndop;              // Northing DOP
    uint16_t edop;              // Easting DOP
} ubxNavDop_t;

typedef struct ubxNavSolution_s {
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
} ubxNavSolution_t;

typedef struct ubxNavPvt_s {
    uint32_t time;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t flags3;
    uint8_t reserved0[5];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
} ubxNavPvt_t;

typedef struct ubxNavVelned_s {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubxNavVelned_t;

typedef struct ubxNavSvinfoChannel_s {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubxNavSvinfoChannel_t;

typedef struct ubxNavSatSv_s {
    uint8_t gnssId;
    uint8_t svId;               // Satellite ID
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    int8_t elev;                // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int16_t prRes;              // Pseudo range residual in decimetres
    uint32_t flags;             // Bitmask
} ubxNavSatSv_t;

typedef struct ubxNavSvinfo_s {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubxNavSvinfoChannel_t channel[GPS_SV_MAXSATS_M8N];         // 32 satellites * 12 byte
} ubxNavSvinfo_t;

typedef struct ubxNavSat_s {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t version;
    uint8_t numSvs;
    uint8_t reserved0[2];
    ubxNavSatSv_t svs[GPS_SV_MAXSATS_M9N];
} ubxNavSat_t;

typedef struct ubxAck_s {
    uint8_t clsId;               // Class ID of the acknowledged message
    uint8_t msgId;               // Message ID of the acknowledged message
} ubxAck_t;

typedef enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubsNavFixType_e;

typedef enum {
    NAV_STATUS_FIX_VALID = 1,
    NAV_STATUS_TIME_WEEK_VALID = 4,
    NAV_STATUS_TIME_SECOND_VALID = 8
} ubxNavStatusBits_e;

typedef enum {
    NAV_VALID_DATE = 1,
    NAV_VALID_TIME = 2
} ubxNavPvtValid_e;

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

// from the UBlox9 document, the largest payout we receive is the NAV-SAT and the payload size
// is calculated as 8 + 12*numCh.  numCh in the case of a M9N is 42.
#define UBLOX_PAYLOAD_SIZE (8 + 12 * 42)


// Receive buffer
static union {
    ubxNavPosllh_t posllh;
    ubxNavStatus_t status;
    ubxNavDop_t dop;
    ubxNavSolution_t solution;
    ubxNavVelned_t velned;
    ubxNavPvt_t pvt;
    ubxNavSvinfo_t svinfo;
    ubxNavSat_t sat;
    ubxCfgGnss_t gnss;
    ubxAck_t ack;
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
        gpsSol.llh.altCm = _buffer.posllh.altitudeMslMm / 10;  //alt in cm
        gpsSetFixState(next_fix);
        _new_position = true;
        break;
    case MSG_STATUS:
        *gpsPacketLogChar = LOG_UBLOX_STATUS;
        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        break;
    case MSG_DOP:
        *gpsPacketLogChar = LOG_UBLOX_DOP;
        gpsSol.dop.pdop = _buffer.dop.pdop;
        gpsSol.dop.hdop = _buffer.dop.hdop;
        gpsSol.dop.vdop = _buffer.dop.vdop;
        break;
    case MSG_SOL:
        *gpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(GPS_FIX);
        gpsSol.numSat = _buffer.solution.satellites;
#ifdef USE_RTC_TIME
        //set clock, when gps time is available
        if(!rtcHasTime() && (_buffer.solution.fix_status & NAV_STATUS_TIME_SECOND_VALID) && (_buffer.solution.fix_status & NAV_STATUS_TIME_WEEK_VALID)) {
            //calculate rtctime: week number * ms in a week + ms of week + fractions of second + offset to UNIX reference year - 18 leap seconds
            rtcTime_t temp_time = (((int64_t) _buffer.solution.week)*7*24*60*60*1000) + _buffer.solution.time + (_buffer.solution.time_nsec/1000000) + 315964800000LL - 18000;
            rtcSet(&temp_time);
        }
#endif
        break;
    case MSG_VELNED:
        *gpsPacketLogChar = LOG_UBLOX_VELNED;
        gpsSol.speed3d = _buffer.velned.speed_3d;       // cm/s
        gpsSol.groundSpeed = _buffer.velned.speed_2d;    // cm/s
        gpsSol.groundCourse = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
        break;
    case MSG_PVT:
        *gpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (_buffer.pvt.flags & NAV_STATUS_FIX_VALID) && (_buffer.pvt.fixType == FIX_3D);
        gpsSol.llh.lon = _buffer.pvt.lon;
        gpsSol.llh.lat = _buffer.pvt.lat;
        gpsSol.llh.altCm = _buffer.pvt.hMSL / 10;  //alt in cm
        gpsSetFixState(next_fix);
        _new_position = true;
        gpsSol.numSat = _buffer.pvt.numSV;
        gpsSol.acc.hAcc = _buffer.pvt.hAcc;
        gpsSol.acc.vAcc = _buffer.pvt.vAcc;
        gpsSol.acc.sAcc = _buffer.pvt.sAcc;
        gpsSol.speed3d = (uint16_t) sqrtf(powf(_buffer.pvt.gSpeed / 10, 2.0f) + powf(_buffer.pvt.velD / 10, 2.0f));
        gpsSol.groundSpeed = _buffer.pvt.gSpeed / 10;    // cm/s
        gpsSol.groundCourse = (uint16_t) (_buffer.pvt.headMot / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
#ifdef USE_RTC_TIME
        //set clock, when gps time is available
        if (!rtcHasTime() && (_buffer.pvt.valid & NAV_VALID_DATE) && (_buffer.pvt.valid & NAV_VALID_TIME)) {
            dateTime_t dt;
            dt.year = _buffer.pvt.year;
            dt.month = _buffer.pvt.month;
            dt.day = _buffer.pvt.day;
            dt.hours = _buffer.pvt.hour;
            dt.minutes = _buffer.pvt.min;
            dt.seconds = _buffer.pvt.sec;
            dt.millis = (_buffer.pvt.nano > 0) ? _buffer.pvt.nano / 1000 : 0; //up to 5ms of error
            rtcSetDateTime(&dt);
        }
#endif
        break;
    case MSG_SVINFO:
        *gpsPacketLogChar = LOG_UBLOX_SVINFO;
        GPS_numCh = _buffer.svinfo.numCh;
        // If we're getting NAV-SVINFO is because we're dealing with an old receiver that does not support NAV-SAT, so we'll only
        // save up to GPS_SV_MAXSATS_LEGACY channels so the BF Configurator knows it's receiving the old sat list info format.
        if (GPS_numCh > GPS_SV_MAXSATS_LEGACY)
            GPS_numCh = GPS_SV_MAXSATS_LEGACY;
        for (i = 0; i < GPS_numCh; i++) {
            GPS_svinfo_chn[i] = _buffer.svinfo.channel[i].chn;
            GPS_svinfo_svid[i] = _buffer.svinfo.channel[i].svid;
            GPS_svinfo_quality[i] =_buffer.svinfo.channel[i].quality;
            GPS_svinfo_cno[i] = _buffer.svinfo.channel[i].cno;
        }
        for (i = GPS_numCh; i < GPS_SV_MAXSATS_LEGACY; i++) {
            GPS_svinfo_chn[i] = 0;
            GPS_svinfo_svid[i] = 0;
            GPS_svinfo_quality[i] = 0;
            GPS_svinfo_cno[i] = 0;
        }
        GPS_svInfoReceivedCount++;
        break;
    case MSG_SAT:
        *gpsPacketLogChar = LOG_UBLOX_SVINFO; // The logger won't show this is NAV-SAT instead of NAV-SVINFO
        GPS_numCh = _buffer.sat.numSvs;
        // We can receive here upto GPS_SV_MAXSATS_M9N channels, but since the majority of receivers currently in use are M8N or older,
        // it would be a waste of RAM to size the arrays that big. For now, they're sized GPS_SV_MAXSATS_M8N which means M9N won't show
        // all their channel information on BF Configurator. When M9N's are more widespread it would be a good time to increase those arrays.
        if (GPS_numCh > GPS_SV_MAXSATS_M8N)
            GPS_numCh = GPS_SV_MAXSATS_M8N;
        for (i = 0; i < GPS_numCh; i++) {
            GPS_svinfo_chn[i] = _buffer.sat.svs[i].gnssId;
            GPS_svinfo_svid[i] = _buffer.sat.svs[i].svId;
            GPS_svinfo_cno[i] = _buffer.sat.svs[i].cno;
            GPS_svinfo_quality[i] =_buffer.sat.svs[i].flags;
        }
        for (i = GPS_numCh; i < GPS_SV_MAXSATS_M8N; i++) {
            GPS_svinfo_chn[i] = 255;
            GPS_svinfo_svid[i] = 0;
            GPS_svinfo_quality[i] = 0;
            GPS_svinfo_cno[i] = 0;
        }

        // Setting the number of channels higher than GPS_SV_MAXSATS_LEGACY is the only way to tell BF Configurator we're sending the
        // enhanced sat list info without changing the MSP protocol. Also, we're sending the complete list each time even if it's empty, so
        // BF Conf can erase old entries shown on screen when channels are removed from the list.
        GPS_numCh = GPS_SV_MAXSATS_M8N;
        GPS_svInfoReceivedCount++;
        break;
    case MSG_CFG_GNSS:
        {
            bool isSBASenabled = false;
            bool isM8NwithDefaultConfig = false;

            if ((_buffer.gnss.numConfigBlocks >= 2) &&
                (_buffer.gnss.configblocks[1].gnssId == 1) && //SBAS
                (_buffer.gnss.configblocks[1].flags & UBLOX_GNSS_ENABLE)) { //enabled

                isSBASenabled = true;
            }

            if ((_buffer.gnss.numTrkChHw == 32) &&  //M8N
                (_buffer.gnss.numTrkChUse == 32) &&
                (_buffer.gnss.numConfigBlocks == 7) &&
                (_buffer.gnss.configblocks[2].gnssId == 2) && //Galileo
                (_buffer.gnss.configblocks[2].resTrkCh == 4) && //min channels
                (_buffer.gnss.configblocks[2].maxTrkCh == 8) && //max channels
                !(_buffer.gnss.configblocks[2].flags & UBLOX_GNSS_ENABLE)) { //disabled

                isM8NwithDefaultConfig = true;
            }

            const uint16_t messageSize = 4 + (_buffer.gnss.numConfigBlocks * sizeof(ubxConfigblock_t));

            ubxMessage_t tx_buffer;
            memcpy(&tx_buffer.payload, &_buffer, messageSize);

            if (isSBASenabled && (gpsConfig()->sbasMode == SBAS_NONE)) {
                tx_buffer.payload.cfg_gnss.configblocks[1].flags &= ~UBLOX_GNSS_ENABLE; //Disable SBAS
            }

            if (isM8NwithDefaultConfig && gpsConfig()->gps_ublox_use_galileo) {
                tx_buffer.payload.cfg_gnss.configblocks[2].flags |= UBLOX_GNSS_ENABLE; //Enable Galileo
            }

            ubloxSendConfigMessage(&tx_buffer, MSG_CFG_GNSS, messageSize);
        }
        break;
    case MSG_ACK_ACK:
        if ((gpsData.ackState == UBLOX_ACK_WAITING) && (_buffer.ack.msgId == gpsData.ackWaitingMsgId)) {
            gpsData.ackState = UBLOX_ACK_GOT_ACK;
        }
        break;
    case MSG_ACK_NACK:
        if ((gpsData.ackState == UBLOX_ACK_WAITING) && (_buffer.ack.msgId == gpsData.ackWaitingMsgId)) {
            gpsData.ackState = UBLOX_ACK_GOT_NACK;
        }
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
#if DEBUG_UBLOX_FRAMES
    debug[2] = _msg_id;
#endif
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
#if DEBUG_UBLOX_FRAMES
    debug[3] = _payload_length;
#endif
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
    if (featureIsEnabled(FEATURE_DASHBOARD)) {
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
    if (featureIsEnabled(FEATURE_DASHBOARD)) {
        dashboardShowFixedPage(PAGE_GPS);
    }
#endif

    serialPassthrough(gpsPort, gpsPassthroughPort, &gpsHandlePassthrough, NULL);
}

float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the distance flown and vertical speed from gps position data
//
static void GPS_calculateDistanceFlownVerticalSpeed(bool initialize)
{
    static int32_t lastCoord[2] = { 0, 0 };
    static int32_t lastAlt;
    static int32_t lastMillis;

    int currentMillis = millis();

    if (initialize) {
        GPS_distanceFlownInCm = 0;
        GPS_verticalSpeedInCmS = 0;
    } else {
        if (STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED)) {
            uint16_t speed = gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.groundSpeed;
            // Only add up movement when speed is faster than minimum threshold
            if (speed > GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S) {
                uint32_t dist;
                int32_t dir;
                GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &lastCoord[GPS_LATITUDE], &lastCoord[GPS_LONGITUDE], &dist, &dir);
                if (gpsConfig()->gps_use_3d_speed) {
                    dist = sqrtf(sq(gpsSol.llh.altCm - lastAlt) + sq(dist));
                }
                GPS_distanceFlownInCm += dist;
            }
        }
        GPS_verticalSpeedInCmS = (gpsSol.llh.altCm - lastAlt) * 1000 / (currentMillis - lastMillis);
        GPS_verticalSpeedInCmS = constrain(GPS_verticalSpeedInCmS, -1500, 1500);
    }
    lastCoord[GPS_LONGITUDE] = gpsSol.llh.lon;
    lastCoord[GPS_LATITUDE] = gpsSol.llh.lat;
    lastAlt = gpsSol.llh.altCm;
    lastMillis = currentMillis;
}

void GPS_reset_home_position(void)
// runs, if GPS is defined, on arming via tryArm() in core.c, and on gyro cal via processRcStickPositions() in rc_controls.c
{
    if (!STATE(GPS_FIX_HOME) || !gpsConfig()->gps_set_home_point_once) {
        if (STATE(GPS_FIX) && gpsSol.numSat >= gpsRescueConfig()->minSats) {
            // those checks are always true for tryArm, but may not be true for gyro cal
            GPS_home[GPS_LATITUDE] = gpsSol.llh.lat;
            GPS_home[GPS_LONGITUDE] = gpsSol.llh.lon;
            GPS_calc_longitude_scaling(gpsSol.llh.lat);
            ENABLE_STATE(GPS_FIX_HOME);
            // no point beeping success here since:
            // when triggered by tryArm, the arming beep is modified to indicate the GPS home fix status on arming, and
            // when triggered by gyro cal, the gyro cal beep takes priority over the GPS beep, so we won't hear the GPS beep
            // PS: to test for gyro cal, check for !ARMED, since we cannot be here while disarmed other than via gyro cal
        }
    }
    GPS_calculateDistanceFlownVerticalSpeed(true); // Initialize
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
    if (STATE(GPS_FIX_HOME)) {
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_home[GPS_LATITUDE], &GPS_home[GPS_LONGITUDE], &dist, &dir);
        GPS_distanceToHome = dist / 100; // m/s
        GPS_distanceToHomeCm = dist; // cm/sec
        GPS_directionToHome = dir / 10; // degrees * 10 or decidegrees
    } else {
        // If we don't have home set, do not display anything
        GPS_distanceToHome = 0;
        GPS_distanceToHomeCm = 0;
        GPS_directionToHome = 0;
    }
}

void onGpsNewData(void)
{
    static timeUs_t timeUs, lastTimeUs = 0;

    // Detect current sample rate of GPS solution
    timeUs = micros();
    gpsSampleRateHz = 1e6f / cmpTimeUs(timeUs, lastTimeUs);
    lastTimeUs = timeUs;

    if (!(STATE(GPS_FIX) && gpsSol.numSat >= GPS_MIN_SAT_COUNT)) {
        // if we don't have a 3D fix and the minimum sats, don't give data to GPS rescue
        return;
    }

    GPS_calculateDistanceAndDirectionToHome();
    if (ARMING_FLAG(ARMED)) {
        GPS_calculateDistanceFlownVerticalSpeed(false);
    }

#ifdef USE_GPS_RESCUE
    gpsRescueNewGpsData();
#endif
}

void gpsSetFixState(bool state)
{
    if (state) {
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_EVER);
    } else {
        DISABLE_STATE(GPS_FIX);
    }
}

float gpsGetSampleRateHz(void)
{
    return gpsSampleRateHz;
}

#endif // USE_GPS
