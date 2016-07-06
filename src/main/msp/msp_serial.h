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


/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * Clients SHOULD operate in READ-ONLY mode and SHOULD present a warning to the user to state
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little client functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

// Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 2 MSP ports.
#define MAX_MSP_PORT_COUNT 2

typedef enum {
    IDLE,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
    HEADER_DATA,
    MESSAGE_RECEIVED
} mspState_e;

typedef bool (*mspCommandSenderFuncPtr)(); // msp command sender function prototype

#define MSP_PORT_INBUF_SIZE 64
#define MSP_PORT_OUTBUF_SIZE 256

typedef enum {
    MSP_MODE_SERVER,
    MSP_MODE_CLIENT
} mspPortMode_e;

typedef struct mspPort_s {
    serialPort_t *port;                      // NULL when unused.
    mspPortMode_e mode;

    mspCommandSenderFuncPtr commandSenderFn;   // NULL when unused.

    mspState_e c_state;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t cmdMSP;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
} mspPort_t;

extern mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

typedef void (*mspPostProcessFuncPtr)(mspPort_t *); // msp post process function, used for gracefully handling reboots, etc.

extern mspPostProcessFuncPtr mspPostProcessFn;

void mspSerialInit(void);
void mspSerialProcess();
void mspSerialAllocatePorts(void);
void mspSerialReleasePortIfAllocated(serialPort_t *serialPort);
