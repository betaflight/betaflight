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


#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/system.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gps_private.h"

#include "flight/gps_conversion.h"

#include "config/config.h"
#include "fc/runtime_config.h"

//#define GPS_PROTO_UBLOX_NEO7PLUS
#define GPS_VERSION_DETECTION_TIMEOUT_MS    300
#define MAX_UBLOX_PAYLOAD_SIZE              256
#define UBLOX_BUFFER_SIZE                   MAX_UBLOX_PAYLOAD_SIZE
#define UBLOX_SBAS_MESSAGE_LENGTH           16

static const char * baudInitData[GPS_BAUDRATE_COUNT] = {
    "$PUBX,41,1,0003,0001,115200,0*1E\r\n",     // GPS_BAUDRATE_115200
    "$PUBX,41,1,0003,0001,57600,0*2D\r\n",      // GPS_BAUDRATE_57600
    "$PUBX,41,1,0003,0001,38400,0*26\r\n",      // GPS_BAUDRATE_38400
    "$PUBX,41,1,0003,0001,19200,0*23\r\n",      // GPS_BAUDRATE_19200
    "$PUBX,41,1,0003,0001,9600,0*16\r\n"        // GPS_BAUDRATE_9600
};

#ifdef GPS_PROTO_UBLOX_NEO7PLUS
static const uint8_t ubloxVerPoll[] = {
    0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34,          // MON-VER - Poll version info
};
#endif

static const uint8_t ubloxInit_NAV5_Pedestrian[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,           // CFG-NAV5 - Set engine settings (original MWII code)
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2
};

static const uint8_t ubloxInit_NAV5_Airborne1G[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Airborne <1G
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28
};

static const uint8_t ubloxInit_NAV5_Airborne4G[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Airborne <4G
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C
};


static const uint8_t ubloxInit_MSG_NMEA[] = {
    // DISABLE NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,           // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,           // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,           // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,           // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,           // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17            // RMC: Recommended Minimum data
};

static const uint8_t ubloxInit_MSG_UBX_POSLLH[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x00, 0x3B, 0xA2,           // disable SVINFO
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67            // set VELNED MSG rate
};

#ifdef GPS_PROTO_UBLOX_NEO7PLUS
static const uint8_t ubloxInit_MSG_UBX_PVT[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // disable POSLLH
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, // disable STATUS
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xD5, // disable SOL
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xFB, // disable SVINFO
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x29, // disable VELNED
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1  // enable PVT 1 cycle
};
#endif

static const uint8_t ubloxInit_RATE_5Hz[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A  // set rate to 5Hz (measurement period: 200ms, navigation rate: 1 cycle)
};

#ifdef GPS_PROTO_UBLOX_NEO7PLUS
static const uint8_t ubloxInit_RATE_10Hz[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, // set rate to 10Hz (measurement period: 100ms, navigation rate: 1 cycle)
};
#endif

// UBlox 6 Protocol documentation - GPS.G6-SW-10018-F
// SBAS Configuration Settings Desciption, Page 4/210
// 31.21 CFG-SBAS (0x06 0x16), Page 142/210
// A.10 SBAS Configuration (UBX-CFG-SBAS), Page 198/210 - GPS.G6-SW-10018-F
typedef struct ubloxSbas_s {
    uint8_t message[UBLOX_SBAS_MESSAGE_LENGTH];
} ubloxSbas_t;

// Note: these must be defined in the same order is sbasMode_e since no lookup table is used.
static const ubloxSbas_t ubloxSbas[] = {
    {{ /* SBAS_AUTO */  0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0xC9 }},
    {{ /* SBAS_EGNOS */ 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x86, 0x25 }},
    {{ /* SBAS_WAAS */  0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x04, 0xE0, 0x04, 0x00, 0x15, 0x81 }},
    {{ /* SBAS_MSAS */  0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x02, 0x02, 0x00, 0x31, 0xD3 }},
    {{ /* SBAS_GAGAN */ 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x80, 0x01, 0x00, 0x00, 0xAE, 0xCC }},
    {{ /* SBAS_NONE */  0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x02, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0xC1 }},
};

// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

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
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_MON = 0x0A,
    MSG_VER = 0x04,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_PVT = 0x7,
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
        if(_class == CLASS_MON) {
            //uint32_t swver = _buffer.ver.swVersion;
            gpsState.hwVersion = atoi(_buffer.ver.hwVersion);
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

// Send UBLOX binary command data and wait until it is completely transmitted
static bool ubxTransmitAutoConfigCommands(const uint8_t * ubxCmdBuf, uint8_t ubxCmdSize) {
    while (serialTxBytesFree(gpsState.gpsPort) > 0) {
        if (gpsState.autoConfigPosition < ubxCmdSize) {
            serialWrite(gpsState.gpsPort, ubxCmdBuf[gpsState.autoConfigPosition]);
            gpsState.autoConfigPosition++;
        }
        else if (isSerialTransmitBufferEmpty(gpsState.gpsPort)) {
            gpsState.autoConfigStep++;
            gpsState.autoConfigPosition = 0;
            return true;
        }
        else {
            return false;
        }
    }

    return false;
}

static bool gpsConfigure(void)
{
    switch (gpsState.autoConfigStep) {
    case 0: // NAV5
        switch (gpsState.gpsConfig->dynModel) {
            case GPS_DYNMODEL_PEDESTRIAN:
                ubxTransmitAutoConfigCommands(ubloxInit_NAV5_Pedestrian, sizeof(ubloxInit_NAV5_Pedestrian));
                break;
            case GPS_DYNMODEL_AIR_1G:   // Default to this
            default:
                ubxTransmitAutoConfigCommands(ubloxInit_NAV5_Airborne1G, sizeof(ubloxInit_NAV5_Airborne1G));
                break;
            case GPS_DYNMODEL_AIR_4G:
                ubxTransmitAutoConfigCommands(ubloxInit_NAV5_Airborne4G, sizeof(ubloxInit_NAV5_Airborne4G));
                break;
        }
        break;

    case 1: // NAVX5 - skip
        //ubxTransmitAutoConfigCommands(ubloxInit_NAVX5, sizeof(ubloxInit_NAVX5));
        gpsState.autoConfigStep++;
        break;

    case 2: // Disable NMEA messages
        ubxTransmitAutoConfigCommands(ubloxInit_MSG_NMEA, sizeof(ubloxInit_MSG_NMEA));
        break;

    case 3: // Enable UBX messages
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        if (gpsState.hwVersion < 70000) {
#endif
            ubxTransmitAutoConfigCommands(ubloxInit_MSG_UBX_POSLLH, sizeof(ubloxInit_MSG_UBX_POSLLH));
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        }
        else {
            ubxTransmitAutoConfigCommands(ubloxInit_MSG_UBX_PVT, sizeof(ubloxInit_MSG_UBX_PVT));
        }
#endif
        break;

    case 4: // Configure RATE
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        if (gpsState.hwVersion < 70000) {
#endif
            ubxTransmitAutoConfigCommands(ubloxInit_RATE_5Hz, sizeof(ubloxInit_RATE_5Hz));
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
        }
        else {
            ubxTransmitAutoConfigCommands(ubloxInit_RATE_10Hz, sizeof(ubloxInit_RATE_10Hz));
        }
#endif
        break;

    case 5: // SBAS
        ubxTransmitAutoConfigCommands(ubloxSbas[gpsState.gpsConfig->sbasMode].message, UBLOX_SBAS_MESSAGE_LENGTH);
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
        ubxTransmitAutoConfigCommands(ubloxVerPoll, sizeof(ubloxVerPoll));
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
    switch(gpsState.state) {
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
