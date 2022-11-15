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

#define ELRS_TELEMETRY_SHIFT 2
#define ELRS_TELEMETRY_BYTES_PER_CALL 5
#define ELRS_TELEMETRY_MAX_PACKAGES (255 >> ELRS_TELEMETRY_SHIFT)
#define ELRS_TELEMETRY_MAX_MISSED_PACKETS 20

#define ELRS_MSP_BYTES_PER_CALL 5
#define ELRS_MSP_BUFFER_SIZE 65
#define ELRS_MSP_MAX_PACKAGES ((ELRS_MSP_BUFFER_SIZE / ELRS_MSP_BYTES_PER_CALL) + 1)
#define ELRS_MSP_PACKET_OFFSET 5
#define ELRS_MSP_COMMAND_INDEX 7

typedef enum {
    ELRS_SENDER_IDLE = 0,
    ELRS_SENDING,
    ELRS_WAIT_UNTIL_NEXT_CONFIRM,
    ELRS_RESYNC,
    ELRS_RESYNC_THEN_SEND, // perform a RESYNC then go to SENDING
} stubbornSenderState_e;

void initTelemetry(void);
bool getNextTelemetryPayload(uint8_t *nextPayloadSize, uint8_t **payloadData);

void setTelemetryDataToTransmit(const uint8_t lengthToTransmit, uint8_t* dataToTransmit);
bool isTelemetrySenderActive(void);
uint8_t getCurrentTelemetryPayload(uint8_t *outData);
void confirmCurrentTelemetryPayload(const bool telemetryConfirmValue);
void updateTelemetryRate(const uint16_t airRate, const uint8_t tlmRatio, const uint8_t tlmBurst);

void mspReceiverResetState(void);
bool getCurrentMspConfirm(void);
void setMspDataToReceive(const uint8_t maxLength, uint8_t* dataToReceive);
void receiveMspData(const uint8_t packageIndex, const volatile uint8_t* const receiveData);
bool hasFinishedMspData(void);
void mspReceiverUnlock(void);
void processMspPacket(uint8_t *packet);
