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
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build/build_config.h"


#if defined(GPS) && defined(GPS_PROTO_UBLOX)

#include "build/debug.h"


#include "common/axis.h"
#include "common/typeconversion.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gps_private.h"

//#define GPS_PROTO_UBLOX_NEO7PLUS
#define GPS_VERSION_DETECTION_TIMEOUT_MS    300
#define MAX_UBLOX_PAYLOAD_SIZE              256
#define UBLOX_BUFFER_SIZE                   MAX_UBLOX_PAYLOAD_SIZE
#define UBLOX_SBAS_MESSAGE_LENGTH           16

#define UBX_DYNMODEL_PEDESTRIAN 3
#define UBX_DYNMODEL_AIR_1G     6
#define UBX_DYNMODEL_AIR_4G     8

#define UBX_FIXMODE_2D_ONLY 1
#define UBX_FIXMODE_3D_ONLY 2
#define UBX_FIXMODE_AUTO    3

// SBAS_AUTO, SBAS_EGNOS, SBAS_WAAS, SBAS_MSAS, SBAS_GAGAN, SBAS_NONE
static const uint32_t ubloxScanMode1[] = {
    0x00000000, 0x00000851, 0x0004E004, 0x00020200, 0x00000180, 0x00000000,
};

static const char * baudInitData[GPS_BAUDRATE_COUNT] = {
    "$PUBX,41,1,0003,0001,115200,0*1E\r\n",     // GPS_BAUDRATE_115200
    "$PUBX,41,1,0003,0001,57600,0*2D\r\n",      // GPS_BAUDRATE_57600
    "$PUBX,41,1,0003,0001,38400,0*26\r\n",      // GPS_BAUDRATE_38400
    "$PUBX,41,1,0003,0001,19200,0*23\r\n",      // GPS_BAUDRATE_19200
    "$PUBX,41,1,0003,0001,9600,0*16\r\n"        // GPS_BAUDRATE_9600
};

// payload types
typedef struct {
    uint8_t mode;
    uint8_t usage;
    uint8_t maxSBAS;
    uint8_t scanmode2;
    uint32_t scanmode1;
} ubx_sbas;

typedef struct {
    uint8_t class;
    uint8_t id;
    uint8_t rate;
} ubx_msg;

typedef struct {
    uint16_t meas;
    uint16_t nav;
    uint16_t time;
} ubx_rate;

typedef union {
    uint8_t bytes[48];
    ubx_sbas sbas;
    ubx_msg msg;
    ubx_rate rate;
} ubx_payload;

// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    ubx_header header;
    ubx_payload payload;
} __attribute__((packed)) ubx_message;

typedef struct {
    char swVersion[30];      // Zero-terminated Software Version String
    char hwVersion[10];      // Zero-terminated Hardware Version String
} ubx_mon_ver;

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

typedef struct {
    uint32_t time; // GPS msToW
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t reserved1;
    uint8_t satellites;
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    int32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
    uint16_t position_DOP;
    uint16_t reserved2;
    uint16_t reserved3;
} ubx_nav_pvt;

enum {
    PREAMBLE1 = 0xB5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_MON = 0x0A,
    MSG_CLASS_UBX = 0x01,
    MSG_CLASS_NMEA = 0xF0,
    MSG_VER = 0x04,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_NMEA_GGA = 0x0,
    MSG_NMEA_GLL = 0x1,
    MSG_NMEA_GSA = 0x2,
    MSG_NMEA_GSV = 0x3,
    MSG_NMEA_RMC = 0x4,
    MSG_NMEA_VGS = 0x5,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_PVT = 0x7,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24,
    MSG_CFG_SBAS = 0x16
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

static uint8_t next_fix_type;
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


// Send buffer
static union {
    ubx_message message;
    uint8_t bytes[58];
} send_buffer;

// Receive buffer
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    ubx_nav_pvt pvt;
    ubx_nav_svinfo svinfo;
    ubx_mon_ver ver;
    uint8_t bytes[UBLOX_BUFFER_SIZE];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}

static uint8_t gpsMapFixType(bool fixValid, uint8_t ubloxFixType)
{
    if (fixValid && ubloxFixType == FIX_2D)
        return GPS_FIX_2D;
    if (fixValid && ubloxFixType == FIX_3D)
        return GPS_FIX_3D;
    return GPS_NO_FIX;
}

static void sendConfigMessageUBLOX(void)
{
    uint8_t ck_a=0, ck_b=0;
    send_buffer.message.header.preamble1=PREAMBLE1;
    send_buffer.message.header.preamble2=PREAMBLE2;
    _update_checksum(&send_buffer.bytes[2], send_buffer.message.header.length+4, &ck_a, &ck_b);
    send_buffer.bytes[send_buffer.message.header.length+6] = ck_a;
    send_buffer.bytes[send_buffer.message.header.length+7] = ck_b;
    serialWriteBuf(gpsState.gpsPort, send_buffer.bytes, send_buffer.message.header.length+8);
    //check ack/nack here
}

#ifdef GPS_PROTO_UBLOX_NEO7PLUS
static void pollVersion(void)
{
    send_buffer.message.header.msg_class = CLASS_MON;
    send_buffer.message.header.msg_id = MSG_VER;
    send_buffer.message.header.length = 0;
    sendConfigMessageUBLOX();
}
#endif

static const uint8_t default_payload[] = {
    0xFF, 0xFF, 0x03, 0x03, 0x00,           // CFG-NAV5 - Set engine settings (original MWII code)
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void configureNAV5(uint8_t dynModel, uint8_t fixMode)
{
    send_buffer.message.header.msg_class = CLASS_CFG;
    send_buffer.message.header.msg_id = MSG_CFG_NAV_SETTINGS;
    send_buffer.message.header.length = 0x24;
    memcpy(send_buffer.message.payload.bytes, default_payload, sizeof(default_payload));
    send_buffer.message.payload.bytes[2] = dynModel;
    send_buffer.message.payload.bytes[3] = fixMode;
    sendConfigMessageUBLOX();
}


static void configureMSG(uint8_t class, uint8_t id, uint8_t rate)
{
    send_buffer.message.header.msg_class = CLASS_CFG;
    send_buffer.message.header.msg_id = MSG_CFG_SET_RATE;
    send_buffer.message.header.length = 3;
    send_buffer.message.payload.msg.class = class;
    send_buffer.message.payload.msg.id = id;
    send_buffer.message.payload.msg.rate = rate;
    sendConfigMessageUBLOX();
}

/*
 * measRate in ms
 * navRate cycles
 * timeRef 0 UTC, 1 GPS
 */
static void configureRATE(uint16_t measRate)
{
    send_buffer.message.header.msg_class = CLASS_CFG;
    send_buffer.message.header.msg_id = MSG_CFG_RATE;
    send_buffer.message.header.length = 6;
    send_buffer.message.payload.rate.meas=measRate;
    send_buffer.message.payload.rate.nav=1;
    send_buffer.message.payload.rate.time=1;
    sendConfigMessageUBLOX();
}

/*
 */
static void configureSBAS(void)
{
    send_buffer.message.header.msg_class = CLASS_CFG;
    send_buffer.message.header.msg_id = MSG_CFG_SBAS;
    send_buffer.message.header.length = 8;
    send_buffer.message.payload.sbas.mode=(gpsState.gpsConfig->sbasMode == SBAS_NONE?2:3);
    send_buffer.message.payload.sbas.usage=3;
    send_buffer.message.payload.sbas.maxSBAS=3;
    send_buffer.message.payload.sbas.scanmode2=0;
    send_buffer.message.payload.sbas.scanmode1=ubloxScanMode1[gpsState.gpsConfig->sbasMode];
    sendConfigMessageUBLOX();
}

static bool gpsParceFrameUBLOX(void)
{
    switch (_msg_id) {
    case MSG_POSLLH:
        //i2c_dataset.time                = _buffer.posllh.time;
        gpsSol.llh.lon = _buffer.posllh.longitude;
        gpsSol.llh.lat = _buffer.posllh.latitude;
        gpsSol.llh.alt = _buffer.posllh.altitude_msl / 10;  //alt in cm
        gpsSol.eph = gpsConstrainEPE(_buffer.posllh.horizontal_accuracy / 10);
        gpsSol.epv = gpsConstrainEPE(_buffer.posllh.vertical_accuracy / 10);
        gpsSol.flags.validEPE = 1;
        if (next_fix_type != GPS_NO_FIX)
            gpsSol.fixType = next_fix_type;
        _new_position = true;
        break;
    case MSG_STATUS:
        next_fix_type = gpsMapFixType(_buffer.status.fix_status & NAV_STATUS_FIX_VALID, _buffer.status.fix_type);
        if (next_fix_type == GPS_NO_FIX)
            gpsSol.fixType = GPS_NO_FIX;
        break;
    case MSG_SOL:
        next_fix_type = gpsMapFixType(_buffer.solution.fix_status & NAV_STATUS_FIX_VALID, _buffer.solution.fix_type);
        if (next_fix_type == GPS_NO_FIX)
            gpsSol.fixType = GPS_NO_FIX;
        gpsSol.numSat = _buffer.solution.satellites;
        gpsSol.hdop = gpsConstrainHDOP(_buffer.solution.position_DOP);
        break;
    case MSG_VELNED:
        gpsSol.groundSpeed = _buffer.velned.speed_2d;    // cm/s
        gpsSol.groundCourse = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        gpsSol.velNED[0] = _buffer.velned.ned_north;
        gpsSol.velNED[1] = _buffer.velned.ned_east;
        gpsSol.velNED[2] = _buffer.velned.ned_down;
        gpsSol.flags.validVelNE = 1;
        gpsSol.flags.validVelD = 1;
        _new_speed = true;
        break;
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
    case MSG_PVT:
        next_fix_type = gpsMapFixType(_buffer.pvt.fix_status & NAV_STATUS_FIX_VALID, _buffer.pvt.fix_type);
        gpsSol.fixType = next_fix_type;
        gpsSol.llh.lon = _buffer.pvt.longitude;
        gpsSol.llh.lat = _buffer.pvt.latitude;
        gpsSol.llh.alt = _buffer.pvt.altitude_msl / 10;  //alt in cm
        gpsSol.velNED[X]=_buffer.pvt.ned_north / 10;  // to cm/s
        gpsSol.velNED[Y]=_buffer.pvt.ned_east / 10;   // to cm/s
        gpsSol.velNED[Z]=_buffer.pvt.ned_down / 10;   // to cm/s
        gpsSol.groundSpeed = _buffer.pvt.speed_2d / 10;    // to cm/s
        gpsSol.groundCourse = (uint16_t) (_buffer.pvt.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        gpsSol.numSat = _buffer.pvt.satellites;
        gpsSol.eph = gpsConstrainEPE(_buffer.pvt.horizontal_accuracy / 10);
        gpsSol.epv = gpsConstrainEPE(_buffer.pvt.vertical_accuracy / 10);
        gpsSol.hdop = gpsConstrainHDOP(_buffer.pvt.position_DOP);
        gpsSol.flags.validVelNE = 1;
        gpsSol.flags.validVelD = 1;
        gpsSol.flags.validEPE = 1;
        _new_position = true;
        _new_speed = true;
        break;
    case MSG_VER:
        if (_class == CLASS_MON) {
            //uint32_t swver = _buffer.ver.swVersion;
            gpsState.hwVersion = fastA2I(_buffer.ver.hwVersion);
        }
        break;
#endif
    default:
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        gpsSol.flags.gpsHeartbeat = !gpsSol.flags.gpsHeartbeat;
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
            _payload_length |= (uint16_t)(data << 8);
            if (_payload_length > MAX_UBLOX_PAYLOAD_SIZE ) {
                // we can't receive the whole packet, just log the error and start searching for the next packet.
                gpsStats.errors++;
                _step = 0;
                break;
            }
            // prepare to receive payload
            _payload_counter = 0;
            if (_payload_length == 0) {
                _step = 7;
            }
            break;
        case 6:
            _ck_b += (_ck_a += data);       // checksum byte
            if (_payload_counter < MAX_UBLOX_PAYLOAD_SIZE) {
                _buffer.bytes[_payload_counter] = data;
            }
            // NOTE: check counter BEFORE increasing so that a payload_size of 65535 is correctly handled.  This can happen if garbage data is received.
            if (_payload_counter ==  _payload_length - 1) {
                _step++;
            }
            _payload_counter++;
            break;
        case 7:
            _step++;
            if (_ck_a != data) {
                _skip_packet = true;          // bad checksum
                gpsStats.errors++;
                _step = 0;
            }
            break;
        case 8:
            _step = 0;

            if (_ck_b != data) {
                gpsStats.errors++;
                break;              // bad checksum
            }

            gpsStats.packetCount++;

            if (_skip_packet) {
                break;
            }

            if (gpsParceFrameUBLOX()) {
                parsed = true;
            }
    }

    return parsed;
}

static bool gpsConfigure(void)
{
    switch (gpsState.autoConfigStep) {
    case 0: // NAV5
        switch (gpsState.gpsConfig->dynModel) {
            case GPS_DYNMODEL_PEDESTRIAN:
                configureNAV5(UBX_DYNMODEL_PEDESTRIAN, UBX_FIXMODE_AUTO);
                break;
            case GPS_DYNMODEL_AIR_1G:   // Default to this
            default:
                configureNAV5(UBX_DYNMODEL_AIR_1G, UBX_FIXMODE_AUTO);
                break;
            case GPS_DYNMODEL_AIR_4G:
                configureNAV5(UBX_DYNMODEL_AIR_4G, UBX_FIXMODE_AUTO);
                break;
        }
        gpsState.autoConfigStep++;
        break;

    case 1: // NAVX5 - skip
        gpsState.autoConfigStep++;
        break;

    case 2: // Disable NMEA messages
        configureMSG(MSG_CLASS_NMEA, MSG_NMEA_GGA, 0);
        configureMSG(MSG_CLASS_NMEA, MSG_NMEA_GLL, 0);
        configureMSG(MSG_CLASS_NMEA, MSG_NMEA_GSA, 0);
        configureMSG(MSG_CLASS_NMEA, MSG_NMEA_GSV, 0);
        configureMSG(MSG_CLASS_NMEA, MSG_NMEA_RMC, 0);
        configureMSG(MSG_CLASS_NMEA, MSG_NMEA_VGS, 0);
        gpsState.autoConfigStep++;
        break;

    case 3: // Enable UBX messages
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        if ((gpsState.gpsConfig->provider == GPS_UBLOX) || (gpsState.hwVersion < 70000)) {
#endif
            configureMSG(MSG_CLASS_UBX, MSG_POSLLH, 1);
            configureMSG(MSG_CLASS_UBX, MSG_STATUS, 1);
            configureMSG(MSG_CLASS_UBX, MSG_SOL,    1);
            configureMSG(MSG_CLASS_UBX, MSG_VELNED, 1);
            configureMSG(MSG_CLASS_UBX, MSG_SVINFO, 0);
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
            configureMSG(MSG_CLASS_UBX, MSG_PVT,    0);
        }
        else if (gpsState.gpsConfig->provider == GPS_UBLOX7PLUS) {
            configureMSG(MSG_CLASS_UBX, MSG_POSLLH, 0);
            configureMSG(MSG_CLASS_UBX, MSG_STATUS, 0);
            configureMSG(MSG_CLASS_UBX, MSG_SOL,    0);
            configureMSG(MSG_CLASS_UBX, MSG_VELNED, 0);
            configureMSG(MSG_CLASS_UBX, MSG_SVINFO, 0);
            configureMSG(MSG_CLASS_UBX, MSG_PVT,    1);
        }
#endif
        gpsState.autoConfigStep++;
        break;

    case 4: // Configure RATE
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        if ((gpsState.gpsConfig->provider == GPS_UBLOX) || (gpsState.hwVersion < 70000)) {
#endif
            configureRATE(200); // 5Hz
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        }
        else if (gpsState.gpsConfig->provider == GPS_UBLOX7PLUS) {
            configureRATE(100); // 10Hz
        }
#endif
        gpsState.autoConfigStep++;
        break;

    case 5: // SBAS
        configureSBAS();
        gpsState.autoConfigStep++;
        break;

    default:
        // ublox should be initialised, try receiving
        gpsSetState(GPS_RECEIVING_DATA);
        break;
    }

    return false;
}

static bool gpsCheckVersion(void)
{
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
    if (gpsState.autoConfigStep == 0) {
        pollVersion();
        gpsState.autoConfigStep++;
    }
    else {
        // Wait until version found
        if (gpsState.hwVersion != 0) {
            gpsState.autoConfigStep = 0;
            gpsState.autoConfigPosition = 0;
            gpsSetState(GPS_CONFIGURE);
        }
        else if ((millis() - gpsState.lastStateSwitchMs) >= GPS_VERSION_DETECTION_TIMEOUT_MS) {
            gpsState.hwVersion = 0;
            gpsState.autoConfigStep = 0;
            gpsState.autoConfigPosition = 0;
            gpsSetState(GPS_CONFIGURE);
        }
    }
#else
    gpsState.hwVersion = 0;
    gpsSetState(GPS_CONFIGURE);
#endif
    return false;
}

static bool gpsReceiveData(void)
{
    bool hasNewData = false;

    if (gpsState.gpsPort) {
        while (serialRxBytesWaiting(gpsState.gpsPort) && !hasNewData) {
            uint8_t newChar = serialRead(gpsState.gpsPort);
            if (gpsNewFrameUBLOX(newChar)) {
                hasNewData = true;
            }
        }
    }

    return hasNewData;
}

static bool gpsInitialize(void)
{
    gpsSetState(GPS_CHANGE_BAUD);
    return false;
}

static bool gpsChangeBaud(void)
{
    if ((gpsState.gpsConfig->autoBaud != GPS_AUTOBAUD_OFF) && (gpsState.autoBaudrateIndex < GPS_BAUDRATE_COUNT)) {
        // Do the switch only if TX buffer is empty - make sure all init string was sent at the same baud
        if (isSerialTransmitBufferEmpty(gpsState.gpsPort)) {
            // Cycle through all possible bauds and send init string
            serialSetBaudRate(gpsState.gpsPort, baudRates[gpsToSerialBaudRate[gpsState.autoBaudrateIndex]]);
            serialPrint(gpsState.gpsPort, baudInitData[gpsState.baudrateIndex]);
            gpsState.autoBaudrateIndex++;
            gpsSetState(GPS_CHANGE_BAUD);   // switch to the same state to reset state transition time
        }
    }
    else {
        gpsFinalizeChangeBaud();
    }

    return false;
}

bool gpsHandleUBLOX(void)
{
    // Receive data
    bool hasNewData = gpsReceiveData();

    // Process state
    switch (gpsState.state) {
    default:
        return false;

    case GPS_INITIALIZING:
        return gpsInitialize();

    case GPS_CHANGE_BAUD:
        return gpsChangeBaud();

    case GPS_CHECK_VERSION:
        return gpsCheckVersion();

    case GPS_CONFIGURE:
        // Either use specific config file for GPS or let dynamically upload config
        if (gpsState.gpsConfig->autoConfig == GPS_AUTOCONFIG_OFF) {
            gpsSetState(GPS_RECEIVING_DATA);
            return false;
        }
        else {
            return gpsConfigure();
        }

    case GPS_RECEIVING_DATA:
        return hasNewData;
    }
}

#endif
