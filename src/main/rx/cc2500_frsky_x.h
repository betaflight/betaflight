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

#include "rx/rx.h"
#include "rx/rx_spi.h"

#define RC_CHANNEL_COUNT_FRSKY_X 16

void frSkyXSetRcData(uint16_t *rcData, const uint8_t *payload);

uint8_t frSkyXInit(void);
rx_spi_received_e frSkyXHandlePacket(uint8_t * const packet, uint8_t * const protocolState);
rx_spi_received_e frSkyXProcessFrame(uint8_t * const packet);
