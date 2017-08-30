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

/*
 * Crossfire constants provided by Team Black Sheep under terms of the 2-Clause BSD License
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CRSF_BAUDRATE           420000

enum { CRSF_SYNC_BYTE = 0xC8 };

enum { CRSF_FRAME_SIZE_MAX = 64 }; // 62 bytes frame plus 2 bytes frame header(<length><type>)
enum { CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6 };

typedef enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
} crsfFrameType_e;

enum {
    CRSF_DISPLAYPORT_SUBCMD_UPDATE = 0x01, // transmit displayport buffer to remote
    CRSF_DISPLAYPORT_SUBCMD_CLEAR = 0X02, // clear client screen
    CRSF_DISPLAYPORT_SUBCMD_OPEN = 0x03,  // client request to open cms menu
    CRSF_DISPLAYPORT_SUBCMD_CLOSE = 0x04,  // client request to close cms menu
    CRSF_DISPLAYPORT_SUBCMD_POLL = 0x05,  // client request to poll/refresh cms menu
};

enum {
    CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET = 1,
    CRSF_DISPLAYPORT_OPEN_COLS_OFFSET = 2,
};

enum {
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
};

enum {
    CRSF_FRAME_LENGTH_ADDRESS = 1, // length of ADDRESS field
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
    CRSF_FRAME_LENGTH_TYPE = 1, // length of TYPE field
    CRSF_FRAME_LENGTH_CRC = 1, // length of CRC field
    CRSF_FRAME_LENGTH_TYPE_CRC = 2, // length of TYPE and CRC fields combined
    CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4, // length of Extended Dest/Origin, TYPE and CRC fields combined
    CRSF_FRAME_LENGTH_NON_PAYLOAD = 4, // combined length of all fields except payload
};

enum {
    CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
    CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
    CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
};

// Clashes with CRSF_ADDRESS_FLIGHT_CONTROLLER
#define CRSF_SYNC_BYTE 0XC8

typedef enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
} crsfAddress_e;

typedef enum {
    CRSF_UINT8 = 0,
    CRSF_INT8 = 1,
    CRSF_UINT16 = 2,
    CRSF_INT16 = 3,
    CRSF_UINT32 = 4,
    CRSF_INT32 = 5,
    CRSF_UINT64 = 6,
    CRSF_INT64 = 7,
    CRSF_FLOAT = 8,
    CRSF_TEXT_SELECTION = 9,
    CRSF_STRING = 10,
    CRSF_FOLDER = 11,
    CRSF_INFO = 12,
    CRSF_COMMAND = 13,
    CRSF_VTX = 15,
    CRSF_OUT_OF_RANGE = 127,
} crsfProtocolParameterType_e;

typedef enum {
    CRSF_BROADCAST_FRAME_LENGTH_OFFSET = 1,
    CRSF_BROADCAST_FRAME_TYPE_OFFSET = 2,
    CRSF_BROADCAST_FRAME_PAYLOAD_OFFSET = 3,
} crsfBroadcastFrameOffset_e;

typedef enum {
    CRSF_EXTENDED_FRAME_LENGTH_OFFSET = 1,
    CRSF_EXTENDED_FRAME_TYPE_OFFSET = 2,
    CRSF_EXTENDED_FRAME_DESTINATION_OFFSET = 3,
    CRSF_EXTENDED_FRAME_ORIGIN_OFFSET = 4,
    CRSF_EXTENDED_FRAME_PAYLOAD_OFFSET = 5,
} crsfExtendedFrameOffset_e;

enum { CRSF_PARAMETER_WRITE_DATA_OFFSET = 1 };

typedef enum {
    CRSF_PARAM_SKIP_STRING = 0,
    CRSF_PARAM_INCLUDE_STRING = 1
} crsfProtocolParamPackString_e;


struct sbuf_s;
struct clivalue_s;
void crsfProtocolPackOutOfRangeCli(struct sbuf_s *dst);
void crsfProtocolPackFolderCli(struct sbuf_s *dst, const struct clivalue_s *value, crsfProtocolParamPackString_e paramPackString);
void crsfProtocolPackU8Cli(struct sbuf_s *dst, const struct clivalue_s *value, crsfProtocolParamPackString_e paramPackString);
void crsfProtocolUnpackU8Cli(const struct clivalue_s *value, const uint8_t *payload);
void crsfProtocolPackU16Cli(struct sbuf_s *dst, const struct clivalue_s *value, crsfProtocolParamPackString_e paramPackString);
void crsfProtocolUnpackU16Cli(const struct clivalue_s *value, const uint8_t *payload);

void crsfParameterWriteEntryHeader(struct sbuf_s *dst, uint8_t destinationAddress);
void crsfProtocolParameterRead(struct sbuf_s *dst, const uint8_t *payload, uint8_t originAddress, crsfProtocolParamPackString_e paramPackString);
void crsfProtocolParameterWrite(const uint8_t *payload);
bool crsfInterpretExtendedFrame(struct sbuf_s *dst, uint8_t *frame);
