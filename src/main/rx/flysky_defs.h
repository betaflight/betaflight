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
#include <stdint.h>

#define MAX_FLYSKY_CHANNEL_COUNT    8
#define MAX_FLYSKY_2A_CHANNEL_COUNT 14

#define FLYSKY_PAYLOAD_SIZE         21
#define FLYSKY_2A_PAYLOAD_SIZE      37

#define FLYSKY_FREQUENCY_COUNT      16
#define FLYSKY_RSSI_SAMPLE_COUNT    16

#ifndef FLYSKY_CHANNEL_COUNT
#define FLYSKY_CHANNEL_COUNT        MAX_FLYSKY_CHANNEL_COUNT
#endif

#ifndef FLYSKY_2A_CHANNEL_COUNT
#define FLYSKY_2A_CHANNEL_COUNT     MAX_FLYSKY_2A_CHANNEL_COUNT
#endif

#define TX_DELAY        500
#define BIND_TIMEOUT    200000

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t number;
    uint8_t valueL;
    uint8_t valueH;
} flySky2ASens_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t txId;
    uint32_t rxId;
    flySky2ASens_t sens[7];
} flySky2ATelemetryPkt_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t txId;
    uint32_t rxId;
    uint8_t state;
    uint8_t reserved1;
    uint8_t rfChannelMap[16];
    uint8_t reserved2[10];
} flySky2ABindPkt_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t txId;
    uint32_t rxId;
    uint8_t data[28];
} flySky2ARcDataPkt_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t txId;
    uint8_t data[16];
} flySkyRcDataPkt_t;

typedef struct {
    uint32_t packet;
    uint32_t firstPacket;
    uint32_t syncPacket;
    uint32_t telemetry;
} timings_t;

enum {
    SENSOR_INT_V = 0x00,
    SENSOR_TEMP = 0x01,
    SENSOR_MOT_RPM = 0x02,
    SENSOR_EXT_V = 0x03,
    SENSOR_RSSI = 0xFC,
    SENSOR_ERR_RATE = 0xFE
};

enum {
    FLYSKY_2A_PACKET_RC_DATA = 0x58,
    FLYSKY_2A_PACKET_BIND1 = 0xBB,
    FLYSKY_2A_PACKET_BIND2 = 0xBC,
    FLYSKY_2A_PACKET_FS_SETTINGS = 0x56,
    FLYSKY_2A_PACKET_SETTINGS = 0xAA,
    FLYSKY_2A_PACKET_TELEMETRY = 0xAA,
    FLYSKY_PACKET_RC_DATA = 0x55,
    FLYSKY_PACKET_BIND = 0xAA
};
