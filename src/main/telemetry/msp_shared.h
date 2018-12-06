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

#include "common/streambuf.h"
#include "telemetry/crsf.h"
#include "telemetry/smartport.h"

typedef void (*mspResponseFnPtr)(uint8_t *payload);

struct mspPacket_s;
typedef struct mspPackage_s {
    sbuf_t requestFrame;
    uint8_t *requestBuffer;
    uint8_t *responseBuffer;
    struct mspPacket_s *requestPacket;
    struct mspPacket_s *responsePacket;
} mspPackage_t;

typedef union mspRxBuffer_u {
    uint8_t smartPortMspRxBuffer[SMARTPORT_MSP_RX_BUF_SIZE];
    uint8_t crsfMspRxBuffer[CRSF_MSP_RX_BUF_SIZE];
} mspRxBuffer_t;

typedef union mspTxBuffer_u {
    uint8_t smartPortMspTxBuffer[SMARTPORT_MSP_TX_BUF_SIZE];
    uint8_t crsfMspTxBuffer[CRSF_MSP_TX_BUF_SIZE];
} mspTxBuffer_t;

void initSharedMsp(void);
bool handleMspFrame(uint8_t *frameStart, int frameLength, uint8_t *skipsBeforeResponse);
bool sendMspReply(uint8_t payloadSize, mspResponseFnPtr responseFn);
