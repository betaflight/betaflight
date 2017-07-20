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
#include "build/build_config.h"


#if defined(GPS) && defined(GPS_PROTO_NAZA)

#include "build/debug.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/gps_private.h"
#include "io/serial.h"


#define NAZA_MAX_PAYLOAD_SIZE   256

typedef struct {
    uint8_t res[4]; // 0
    uint8_t fw[4]; // 4
    uint8_t hw[4]; // 8
} naza_ver;

typedef struct {
    uint16_t x; // 0
    uint16_t y; // 2
    uint16_t z; // 4
} naza_mag;

typedef struct {
    uint32_t time; // GPS msToW 0
    int32_t longitude; // 4
    int32_t latitude; // 8
    int32_t altitude_msl; // 12
    int32_t h_acc; // 16
    int32_t v_acc; // 20
    int32_t reserved;
    int32_t ned_north; // 28
    int32_t ned_east; // 32
    int32_t ned_down;  // 36
    uint16_t pdop;  // 40
    uint16_t vdop;  // 42
    uint16_t ndop; // 44
    uint16_t edop;  // 46
    uint8_t satellites; // 48
    uint8_t reserved3; //
    uint8_t fix_type; // 50
    uint8_t reserved4; //
    uint8_t fix_status; // 52
    uint8_t reserved5;
    uint8_t reserved6;
    uint8_t mask;   // 55
} naza_nav;

enum {
    HEADER1 = 0x55,
    HEADER2 = 0xAA,
    ID_NAV = 0x10,
    ID_MAG = 0x20,
    ID_VER = 0x30,
    LEN_NAV = 0x3A,
    LEN_MAG = 0x06,
} naza_protocol_bytes;

typedef enum {
    NO_FIX = 0,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_DGPS = 4
} fixType_t;

// Receive buffer
static union {
    naza_mag mag;
    naza_nav nav;
    naza_ver ver;
    uint8_t bytes[NAZA_MAX_PAYLOAD_SIZE];
} _buffernaza;

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static bool _skip_packet;
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

int32_t decodeLong(uint32_t idx, uint8_t mask)
{
    union { uint32_t l; uint8_t b[4]; } val;
    val.l=idx;
    for (int i = 0; i < 4; i++) val.b[i] ^= mask;
    return val.l;
}

int16_t decodeShort(uint16_t idx, uint8_t mask)
{
    union { uint16_t s; uint8_t b[2]; } val;
    val.s=idx;
    for (int i = 0; i < 2; i++) val.b[i] ^= mask;
    return val.s;
}

static bool NAZA_parse_gps(void)
{
    uint8_t mask;
    uint8_t mask_mag;

    switch (_msg_id) {
    case ID_NAV:
        mask = _buffernaza.nav.mask;

        //uint32_t time = decodeLong(_buffernaza.nav.time, mask);
        //uint32_t second = time & 0b00111111; time >>= 6;
        //uint32_t minute = time & 0b00111111; time >>= 6;
        //uint32_t hour = time & 0b00001111; time >>= 4;
        //uint32_t day = time & 0b00011111; time >>= 5;
        //uint32_t month = time & 0b00001111; time >>= 4;
        //uint32_t year = time & 0b01111111;

        gpsSol.llh.lon = decodeLong(_buffernaza.nav.longitude, mask);
        gpsSol.llh.lat = decodeLong(_buffernaza.nav.latitude, mask);
        gpsSol.llh.alt = decodeLong(_buffernaza.nav.altitude_msl, mask) / 10.0f;  //alt in cm

        uint8_t fixType = _buffernaza.nav.fix_type ^ mask;
        //uint8_t fixFlags = _buffernaza.nav.fix_status ^ mask;

        //uint8_t r3 = _buffernaza.nav.reserved3 ^ mask;
        //uint8_t r4 = _buffernaza.nav.reserved4 ^ mask;
        //uint8_t r5 = _buffernaza.nav.reserved5 ^ mask;
        //uint8_t r6 = _buffernaza.nav.reserved6 ^ mask;

        if (fixType == FIX_2D)
            gpsSol.fixType = GPS_FIX_2D;
        else if (fixType == FIX_3D)
            gpsSol.fixType = GPS_FIX_3D;
        else
            gpsSol.fixType = GPS_NO_FIX;

        uint32_t h_acc = decodeLong(_buffernaza.nav.h_acc, mask); // mm
        uint32_t v_acc = decodeLong(_buffernaza.nav.v_acc, mask); // mm
        //uint32_t test = decodeLong(_buffernaza.nav.reserved, mask);

        gpsSol.velNED[0] = decodeLong(_buffernaza.nav.ned_north, mask);  // cm/s
        gpsSol.velNED[1] = decodeLong(_buffernaza.nav.ned_east, mask);   // cm/s
        gpsSol.velNED[2] = decodeLong(_buffernaza.nav.ned_down, mask);   // cm/s


        uint16_t pdop = decodeShort(_buffernaza.nav.pdop, mask); // pdop
        //uint16_t vdop = decodeShort(_buffernaza.nav.vdop, mask); // vdop
        //uint16_t ndop = decodeShort(_buffernaza.nav.ndop, mask);
        //uint16_t edop = decodeShort(_buffernaza.nav.edop, mask);
        //gpsSol.hdop = sqrtf(powf(ndop,2)+powf(edop,2));
        //gpsSol.vdop = decodeShort(_buffernaza.nav.vdop, mask); // vdop

        gpsSol.hdop = gpsConstrainEPE(pdop);        // PDOP
        gpsSol.eph = gpsConstrainEPE(h_acc / 10);   // hAcc in cm
        gpsSol.epv = gpsConstrainEPE(v_acc / 10);   // vAcc in cm
        gpsSol.numSat = _buffernaza.nav.satellites;
        gpsSol.groundSpeed = sqrtf(powf(gpsSol.velNED[0], 2)+powf(gpsSol.velNED[1], 2)); //cm/s

        // calculate gps heading from VELNE
        gpsSol.groundCourse = (uint16_t) (fmodf(RADIANS_TO_DECIDEGREES(atan2_approx(gpsSol.velNED[1], gpsSol.velNED[0]))+3600.0f,3600.0f));

        gpsSol.flags.validVelNE = 1;
        gpsSol.flags.validVelD = 1;
        gpsSol.flags.validEPE = 1;

        _new_position = true;
        _new_speed = true;
        break;
    case ID_MAG:
        mask_mag = (_buffernaza.mag.z)&0xFF;
        mask_mag = (((mask_mag ^ (mask_mag >> 4)) & 0x0F) | ((mask_mag << 3) & 0xF0)) ^ (((mask_mag & 0x01) << 3) | ((mask_mag & 0x01) << 7));

        gpsSol.magData[0] = decodeShort(_buffernaza.mag.x, mask_mag);
        gpsSol.magData[1] = decodeShort(_buffernaza.mag.y, mask_mag);
        gpsSol.magData[2] = (_buffernaza.mag.z ^ (mask_mag<<8));

        gpsSol.flags.validMag = 1;
        break;
    case ID_VER:
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

static bool gpsNewFrameNAZA(uint8_t data)
{
    bool parsed = false;

    switch (_step) {
        case 0: // Sync char 1 (0x55)
            if (HEADER1 == data) {
                _skip_packet = false;
                _step++;
            }
            break;
        case 1: // Sync char 2 (0xAA)
            if (HEADER2 != data) {
                _step = 0;
                break;
            }
            _step++;
            break;
        case 2: // Id
            _step++;
            _ck_b = _ck_a = data;   // reset the checksum accumulators
            _msg_id = data;
            break;
        case 3: // Payload length
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length = data; // payload length low byte
            if (_payload_length > NAZA_MAX_PAYLOAD_SIZE) {
                // we can't receive the whole packet, just log the error and start searching for the next packet.
                gpsStats.errors++;
                _step = 0;
                break;
            }
            // prepare to receive payload
            _payload_counter = 0;
            if (_payload_length == 0) {
                _step = 6;
            }
            break;
        case 4:
            _ck_b += (_ck_a += data);       // checksum byte
            if (_payload_counter < NAZA_MAX_PAYLOAD_SIZE) {
                _buffernaza.bytes[_payload_counter] = data;
            }
            if (++_payload_counter >= _payload_length) {
                _step++;
            }
            break;
        case 5:
            _step++;
            if (_ck_a != data) {
                _skip_packet = true;          // bad checksum
                gpsStats.errors++;
            }
            break;
        case 6:
            _step = 0;
            if (_ck_b != data) {
                gpsStats.errors++;
                break;              // bad checksum
            }

            gpsStats.packetCount++;

            if (_skip_packet) {
                break;
            }

            if (NAZA_parse_gps()) {
                parsed = true;
            }
    }
    return parsed;
}

static bool gpsReceiveData(void)
{
    bool hasNewData = false;

    if (gpsState.gpsPort) {
        while (serialRxBytesWaiting(gpsState.gpsPort)) {
            uint8_t newChar = serialRead(gpsState.gpsPort);
            if (gpsNewFrameNAZA(newChar)) {
                gpsSol.flags.gpsHeartbeat = !gpsSol.flags.gpsHeartbeat;
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
    gpsFinalizeChangeBaud();
    return false;
}

bool gpsHandleNAZA(void)
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
    case GPS_CONFIGURE:
        // No autoconfig, switch straight to receiving data
        gpsSetState(GPS_RECEIVING_DATA);
        return false;

    case GPS_RECEIVING_DATA:
        return hasNewData;
    }
}

#endif
