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

#include "msp/msp.h"

// Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 2 MSP ports.
#define MAX_MSP_PORT_COUNT 2

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
    MSP_EVALUATE_NON_MSP_DATA,
    MSP_SKIP_NON_MSP_DATA
} mspEvaluateNonMspData_e;

#define MSP_PORT_INBUF_SIZE 64
#define MSP_PORT_OUTBUF_SIZE 256

struct serialPort_s;
typedef struct mspPort_s {
    struct serialPort_s *port; // null when port unused.
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t cmdMSP;
    mspState_e c_state;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
} mspPort_t;


void mspSerialInit(mspProcessCommandFnPtr mspProcessCommandFn);
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData);
void mspSerialAllocatePorts(void);
void mspSerialReleasePortIfAllocated(struct serialPort_s *serialPort);
