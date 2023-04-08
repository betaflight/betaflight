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
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/timer.h"
#include "rx/expresslrs_common.h"

bool expressLrsSpiInit(const struct rxSpiConfig_s *rxConfig, struct rxRuntimeState_s *rxRuntimeState, rxSpiExtiConfig_t *extiConfig);
void expressLrsSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
rx_spi_received_e expressLrsDataReceived(uint8_t *payload);
rx_spi_received_e processRFPacket(volatile uint8_t *payload, uint32_t timeStampUs);
bool expressLrsIsFhssReq(void);
void expressLrsDoTelem(void);
bool expressLrsTelemRespReq(void);
void expressLrsSetRfPacketStatus(rx_spi_received_e status);
uint32_t expressLrsGetCurrentFreq(void);
volatile uint8_t *expressLrsGetRxBuffer(void);
volatile uint8_t *expressLrsGetTelemetryBuffer(void);
volatile uint8_t *expressLrsGetPayloadBuffer(void);
void expressLrsHandleTelemetryUpdate(void);
void expressLrsStop(void);
void expressLrsISR(bool runAlways);
