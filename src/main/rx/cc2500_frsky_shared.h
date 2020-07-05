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

#include "rx/rx_spi.h"

#define MAX_MISSING_PKT 100

#define DEBUG_DATA_ERROR_COUNT 0
#define DEBUG_DATA_MISSING_PACKETS 1
#define DEBUG_DATA_BAD_FRAME 2


#define SYNC_DELAY_MAX 9000

#define MAX_MISSING_PKT 100

#define FRSKY_RX_D16FCC_LENGTH 0x1d + 3
#define FRSKY_RX_D16LBT_LENGTH 0x20 + 3
#define FRSKY_RX_D16v2_LENGTH  0x1d + 3

enum {
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING_LOW,
    STATE_BIND_TUNING_HIGH,
    STATE_BIND_BINDING,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY,
    STATE_RESUME,
};

extern rx_spi_protocol_e spiProtocol;
extern uint8_t listLength;
extern uint32_t missingPackets;
extern timeDelta_t timeoutUs;

void initialiseData(bool inBindState);

void nextChannel(uint8_t skip);
