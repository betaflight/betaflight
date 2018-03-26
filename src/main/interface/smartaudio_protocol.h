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

typedef struct smartaudioPowerLevel_s {
    uint16_t mwPower;
    uint8_t valueV1;
} smartaudioPowerLevel_t;

typedef struct smartaudioFrameHeader_s {
    uint8_t syncByte;
    uint8_t headerByte;
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

enum smartaudioBand_e {
    BAND_A,
    BAND_B,
    BAND_E,
    BAND_F,
    BAND_R,
} smartaudioBand_t;

const smartaudioPowerLevel_t smartaudioPowerLevels[] = {
    {  25,   7 },
    { 200,  16 },
    { 500,  25 },
    { 800,  40 },
};

void smartaudioFrameGetSettings(smartaudioCommandOnlyFrame_t *frame);
void smartaudioFrameGetPitmodeFrequency(smartaudioU16Frame_t *frame);
void smartaudioFrameSetPower(smartaudioU8Frame_t *frame, const uint8_t power);
void smartaudioFrameSetBandChannel(smartaudioU8Frame_t *frame, const uint8_t band, const uint8_t channel);
void smartaudioFrameSetFrequency(smartaudioU16Frame_t *frame, const uint16_t frequency, const bool pitmodeFrequency);
void smartaudioFrameSetOperationMode(smartaudioU8Frame_t *frame, const uint8_t operationMode);
bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer);
