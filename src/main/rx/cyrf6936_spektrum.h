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

#include <stdbool.h>
#include <stdint.h>

#define DSM_BIND_TIMEOUT_US 11000
#define DSM_SYNC_TIMEOUT_US 20000
#define DSM_RECV_LONG_TIMEOUT_US 18500
#define DSM_RECV_MID_TIMEOUT_US 7500
#define DSM_RECV_SHORT_TIMEOUT_US 4500

#define DSM_MAX_BIND_PACKETS 500
#define DSM_MAX_MISSED_PACKETS 31

#define DSM_MAX_RF_CHANNEL 0x4F
#define DSM_MAX_CHANNEL_COUNT 12
#define DSM_INITIAL_BIND_CHANNEL 0x0D

#define DSM_TELEMETRY_FRAME_RPM 0x7E
#define DSM_TELEMETRY_TIMEOUT_US 1500
#define DSM_TELEMETRY_TIME_US 176000

typedef struct spektrumConfig_s {
    uint8_t protocol;
    uint8_t mfgId[4];
    uint8_t numChannels;
} spektrumConfig_t;

PG_DECLARE(spektrumConfig_t, spektrumConfig);

bool spektrumSpiInit(const struct rxSpiConfig_s *rxConfig, struct rxRuntimeState_s *rxRuntimeState, rxSpiExtiConfig_t *extiConfig);
void spektrumSpiSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
rx_spi_received_e spektrumSpiDataReceived(uint8_t *payload);
