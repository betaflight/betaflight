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

#define SMARTAUDIO_SERIAL_OPTIONS   SERIAL_NOT_INVERTED | SERIAL_BIDIR_NOPULL | SERIAL_STOPBITS_2
#define SMARTAUDIO_DEFAULT_BAUD     4900
#define SMARTAUDIO_MIN_BAUD         4800
#define SMARTAUDIO_MAX_BAUD         4950

typedef struct smartaudioSettings_s {
    uint8_t version;
    uint8_t unlocked;
    uint8_t channel;
    uint8_t power;
    uint16_t frequency;
    uint16_t pitmodeFrequency;
    bool userFrequencyMode;
    bool pitmodeEnabled;
    bool pitmodeInRangeActive;
    bool pitmodeOutRangeActive;
} smartaudioSettings_t;

typedef struct smartaudioFrameHeader_s {
    uint16_t startCode;
    uint8_t length;
    uint8_t command;
} __attribute__((packed)) smartaudioFrameHeader_t;

typedef struct smartaudioCommandOnlyFrame_s {
    smartaudioFrameHeader_t header;
    uint8_t crc;
} __attribute__((packed)) smartaudioCommandOnlyFrame_t;

typedef struct smartaudioU8Frame_s {
    smartaudioFrameHeader_t header;
    uint8_t payload;
    uint8_t crc;
} __attribute__((packed)) smartaudioU8Frame_t;

typedef struct smartaudioU16Frame_s {
    smartaudioFrameHeader_t header;
    uint16_t payload;
    uint8_t crc;
} __attribute__((packed)) smartaudioU16Frame_t;

typedef struct smartaudioU8ResponseFrame_s {
    smartaudioFrameHeader_t header;
    uint8_t payload;
    uint8_t reserved;
    uint8_t crc;
} __attribute__((packed)) smartaudioU8ResponseFrame_t;

typedef struct smartaudioU16ResponseFrame_s {
    smartaudioFrameHeader_t header;
    uint16_t payload;
    uint8_t reserved;
    uint8_t crc;
} __attribute__((packed)) smartaudioU16ResponseFrame_t;

typedef struct smartaudioSettingsResponseFrame_s {
    smartaudioFrameHeader_t header;
    uint8_t channel;
    uint8_t power;
    uint8_t operationMode;
    uint16_t frequency;
    uint8_t crc;
} __attribute__((packed)) smartaudioSettingsResponseFrame_t;

typedef union smartaudioFrame_u {
    smartaudioCommandOnlyFrame_t commandOnlyFrame;
    smartaudioU8Frame_t u8RequestFrame;
    smartaudioU16Frame_t u16RequestFrame;
} __attribute__((packed)) smartaudioFrame_t;

size_t smartaudioFrameGetSettings(smartaudioFrame_t *smartaudioFrame);
size_t smartaudioFrameGetPitmodeFrequency(smartaudioFrame_t *smartaudioFrame);
size_t smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power);
size_t smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel);
size_t smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency);
size_t smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings);
bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer);
