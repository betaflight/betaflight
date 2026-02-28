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

#include "drivers/time.h"

#include "io/serial.h"

#include "msp/msp.h"

// Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 3 MSP ports.
#ifndef MAX_MSP_PORT_COUNT
#define MAX_MSP_PORT_COUNT 3
#endif

typedef enum {
    PORT_IDLE,
    PORT_MSP_PACKET,
    PORT_CLI_ACTIVE,
    PORT_CLI_CMD
} mspPortState_e;

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} mspPacketState_e;

typedef enum {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
} mspPacketType_e;

typedef enum {
    MSP_EVALUATE_NON_MSP_DATA,
    MSP_SKIP_NON_MSP_DATA
} mspEvaluateNonMspData_e;

typedef enum {
    MSP_PENDING_NONE,
    MSP_PENDING_BOOTLOADER_ROM,
    MSP_PENDING_CLI,
    MSP_PENDING_BOOTLOADER_FLASH,
} mspPendingSystemRequest_e;

#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_OUTBUF_SIZE_MIN 512 // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1. There has been overflow issues with 320 byte buffer.

#ifdef USE_FLASHFS
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 4096
#define MSP_PORT_DATAFLASH_INFO_SIZE 16
#define MSP_PORT_OUTBUF_SIZE (MSP_PORT_DATAFLASH_BUFFER_SIZE + MSP_PORT_DATAFLASH_INFO_SIZE)
#else
#define MSP_PORT_OUTBUF_SIZE MSP_PORT_OUTBUF_SIZE_MIN // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1.
#endif

typedef struct __attribute__((packed)) {
    uint8_t size;
    uint8_t cmd;
} mspHeaderV1_t;

typedef struct __attribute__((packed)) {
    uint16_t size;
} mspHeaderJUMBO_t;

typedef struct __attribute__((packed)) {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} mspHeaderV2_t;

#define MSP_MAX_HEADER_SIZE     9

struct serialPort_s;
typedef struct mspPort_s {
    struct serialPort_s *port; // null when port unused.
    timeMs_t lastActivityMs;
    mspPendingSystemRequest_e pendingRequest;
    mspPortState_e portState;
    mspPacketState_e packetState;
    mspPacketType_e packetType;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint16_t cmdMSP;
    uint8_t cmdFlags;
    mspVersion_e mspVersion;
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum1;
    uint8_t checksum2;
    bool sharedWithTelemetry;
    mspDescriptor_t descriptor;
} mspPort_t;

#define MSP_ACTIVITY_DEFAULT_TIMEOUT_MS 5000

void mspSerialInit(void);
bool mspSerialWaiting(void);
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn, mspProcessReplyFnPtr mspProcessReplyFn);
void mspSerialAllocatePorts(void);
void mspSerialReleasePortIfAllocated(struct serialPort_s *serialPort);
void mspSerialReleaseSharedTelemetryPorts(void);
mspDescriptor_t getMspSerialPortDescriptor(const serialPortIdentifier_e portIdentifier);
int mspSerialPush(serialPortIdentifier_e port, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction, mspVersion_e mspVersion);
uint32_t mspSerialTxBytesFree(void);
timeMs_t mspSerialLastActivityMs(void);
bool mspSerialIsActiveWithin(timeMs_t timeoutMs);
bool mspSerialIsConfiguratorActive(void);
