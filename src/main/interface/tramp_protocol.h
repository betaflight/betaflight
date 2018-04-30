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

#include "drivers/serial.h"

#define TRAMP_SERIAL_OPTIONS    SERIAL_NOT_INVERTED | SERIAL_BIDIR
#define TRAMP_BAUD              9600
#define TRAMP_PAYLOAD_LENGTH    12

typedef struct trampSettings_s {
    uint16_t frequency;
    uint16_t power;
    uint8_t raceModeEnabled;
    uint8_t pitModeEnabled;
} __attribute__((packed)) trampSettings_t;

typedef struct trampFrameHeader_s {
    uint8_t syncStart;
    uint8_t command;
} __attribute__((packed)) trampFrameHeader_t;

#define TRAMP_HEADER_LENGTH sizeof(trampFrameHeader_t)

typedef struct trampFrameFooter_s {
    uint8_t crc;
    uint8_t syncStop;
} __attribute__((packed)) trampFrameFooter_t;

typedef union trampPayload_u {
    uint8_t buf[TRAMP_PAYLOAD_LENGTH];
    trampSettings_t settings;
    uint16_t frequency;
    uint16_t power;
    uint8_t active;
} trampPayload_t;

typedef struct trampFrame_s {
    trampFrameHeader_t header;
    trampPayload_t payload;
    trampFrameFooter_t footer;
} __attribute__((packed)) trampFrame_t;

#define TRAMP_FRAME_LENGTH sizeof(trampFrame_t)

STATIC_ASSERT(sizeof(trampFrameHeader_t) == 2, trampInterface_headerSizeMismatch);
STATIC_ASSERT(sizeof(trampFrame_t) == 16, trampInterface_frameSizeMismatch);

void trampFrameGetSettings(trampFrame_t *frame);
void trampFrameSetFrequency(trampFrame_t *frame, const uint16_t frequency);
void trampFrameSetPower(trampFrame_t *frame, const uint16_t power);
void trampFrameSetActiveState(trampFrame_t *frame, const bool active);
bool trampParseResponseBuffer(trampSettings_t *settings, const uint8_t *buffer, size_t bufferLen);
