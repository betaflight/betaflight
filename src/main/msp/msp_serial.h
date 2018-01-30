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

#include "drivers/time.h"
#include "interface/msp.h"

// Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 3 MSP ports.
#define MAX_MSP_PORT_COUNT 3

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_ARROW,
    MSP_HEADER_SIZE,
    MSP_HEADER_CMD,
    MSP_COMMAND_RECEIVED
} mspState_e;

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
    MSP_PENDING_BOOTLOADER,
    MSP_PENDING_CLI
} mspPendingSystemRequest_e;

#define MSP_PORT_INBUF_SIZE 192
#ifdef USE_FLASHFS
#ifdef STM32F1
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 1024
#else
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 4096
#endif
#define MSP_PORT_DATAFLASH_INFO_SIZE 16
#define MSP_PORT_OUTBUF_SIZE (MSP_PORT_DATAFLASH_BUFFER_SIZE + MSP_PORT_DATAFLASH_INFO_SIZE)
#else
#define MSP_PORT_OUTBUF_SIZE 256
#endif

struct serialPort_s;
typedef struct mspPort_s {
    struct serialPort_s *port; // null when port unused.
    timeMs_t lastActivityMs;
    mspPendingSystemRequest_e pendingRequest;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t cmdMSP;
    mspState_e c_state;
    mspPacketType_e packetType;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
} mspPort_t;

void mspSerialInit(void);
bool mspSerialWaiting(void);
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn, mspProcessReplyFnPtr mspProcessReplyFn);
void mspSerialAllocatePorts(void);
void mspSerialReleasePortIfAllocated(struct serialPort_s *serialPort);
int mspSerialPush(uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction);
uint32_t mspSerialTxBytesFree(void);
