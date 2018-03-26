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

#include "platform.h"
#include "common/crc.h"
#include "interface/smartaudio_protocol.h"

#define SMARTAUDIO_SYNC_BYTE                0xAA
#define SMARTAUDIO_HEADER_BYTE              0x55
#define SMARTAUDIO_GET_PITMODE_FREQ_BYTE    (1 << 14)
#define SMARTAUDIO_SET_PITMODE_FREQ_BYTE    (1 << 15)
#define SMARTAUDIO_FREQUENCY_MASK           0x1FFF

#define SMARTAUDIO_CMD_GET_SETTINGS         0x03
#define SMARTAUDIO_CMD_SET_POWER            0x05
#define SMARTAUDIO_CMD_SET_CHANNEL          0x07
#define SMARTAUDIO_CMD_SET_FREQUENCY        0x09
#define SMARTAUDIO_CMD_SET_MODE             0x0B

#define SMARTAUDIO_RSP_GET_SETTINGS_V1      SMARTAUDIO_CMD_GET_SETTINGS >> 1
#define SMARTAUDIO_RSP_GET_SETTINGS_V2      (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x08
#define SMARTAUDIO_RSP_SET_POWER            SMARTAUDIO_CMD_SET_POWER >> 1
#define SMARTAUDIO_RSP_SET_CHANNEL          SMARTAUDIO_CMD_SET_CHANNEL >> 1
#define SMARTAUDIO_RSP_SET_FREQUENCY        SMARTAUDIO_CMD_SET_FREQUENCY >> 1
#define SMARTAUDIO_RSP_SET_MODE             SMARTAUDIO_CMD_SET_MODE >> 1

#define SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel) ((band) * (uint8_t)8 + (channel))

static void smartaudioFrameInit(const uint8_t command, smartaudioFrameHeader_t *header, const uint8_t payloadLength)
{
    header->syncByte = SMARTAUDIO_SYNC_BYTE;
    header->headerByte = SMARTAUDIO_HEADER_BYTE;
    header->length = payloadLength;
    header->command = command;
}

static void smartaudioCRC(uint8_t *crc, void *data, const uint8_t frameLength)
{
    *crc = 0;
    *crc = crc8_dvb_s2_update(*crc, data, frameLength);
}

static void smartaudioUnpackOperationMode(smartaudioSettings_t *settings, const uint8_t operationMode, const bool settingsResponse)
{
    if (settingsResponse) {
        // operation mode bit order is different between 'Get Settings' and 'Set Mode' responses.
        settings->userFrequencyMode = (bool) (operationMode & 0x01);
        settings->pitmodeEnabled = (bool) (operationMode & 0x02);
        settings->pitmodeInRangeActive = (bool) (operationMode & 0x04);
        settings->pitmodeOutRangeActive = (bool) (operationMode & 0x08);
        settings->unlocked = (bool) (operationMode & 0x10);
    } else {
        settings->pitmodeInRangeActive = (bool) (operationMode & 0x01);
        settings->pitmodeOutRangeActive = (bool) (operationMode & 0x02);
        settings->pitmodeEnabled = (bool) (operationMode & 0x04);
        settings->unlocked = (bool) (operationMode & 0x08);
    }
}

static void smartaudioUnpackFrequency(smartaudioSettings_t *settings, const uint16_t frequency)
{
    if (frequency & SMARTAUDIO_GET_PITMODE_FREQ_BYTE) {
        settings->pitmodeFrequency = frequency & SMARTAUDIO_FREQUENCY_MASK;
    } else {
        settings->frequency = frequency & SMARTAUDIO_FREQUENCY_MASK;
    }
}

static void smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

void smartaudioFrameGetSettings(smartaudioCommandOnlyFrame_t *frame)
{
    smartaudioFrameInit(SMARTAUDIO_CMD_GET_SETTINGS, &frame->header, 0);
    smartaudioCRC(&frame->crc, frame, sizeof(smartaudioFrameHeader_t));
}

void smartaudioFrameGetPitmodeFrequency(smartaudioU16Frame_t *frame)
{
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = 0x4000;
    const uint8_t frameLength = sizeof(smartaudioFrameHeader_t) + sizeof(frame->payload);
    smartaudioCRC(&frame->crc, frame, frameLength);
}

void smartaudioFrameSetPower(smartaudioU8Frame_t *frame, const uint8_t power)
{
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_POWER, &frame->header, sizeof(frame->payload));
    frame->payload = power;
    const uint8_t frameLength = sizeof(smartaudioFrameHeader_t) + sizeof(frame->payload);
    smartaudioCRC(&frame->crc, frame, frameLength);
}

void smartaudioFrameSetBandChannel(smartaudioU8Frame_t *frame, const uint8_t band, const uint8_t channel)
{
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel);
    const uint8_t frameLength = sizeof(smartaudioFrameHeader_t) + sizeof(frame->payload);
    smartaudioCRC(&frame->crc, frame, frameLength);
}

void smartaudioFrameSetFrequency(smartaudioU16Frame_t *frame, const uint16_t frequency, const bool pitmodeFrequency)
{
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = frequency | (pitmodeFrequency ? 0x8000 : 0x00);
    const uint8_t frameLength = sizeof(smartaudioFrameHeader_t) + sizeof(frame->payload);
    smartaudioCRC(&frame->crc, frame, frameLength);
}

void smartaudioFrameSetOperationMode(smartaudioU8Frame_t *frame, const uint8_t operationMode)
{
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_MODE, &frame->header, sizeof(frame->payload));
    frame->payload = operationMode;
    const uint8_t frameLength = sizeof(smartaudioFrameHeader_t) + sizeof(frame->payload);
    smartaudioCRC(&frame->crc, frame, frameLength);
}

bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer)
{
    const smartaudioFrameHeader_t *header = (const smartaudioFrameHeader_t *)buffer;
    const uint8_t fullFrameLength = sizeof(smartaudioFrameHeader_t) + header->length;
    const uint8_t checkLength = fullFrameLength - 1; // subtract crc byte from length
    const uint8_t *bufferCrc = buffer + fullFrameLength;
    uint8_t crc = 0;
    crc = crc8_dvb_s2_update(crc, buffer, checkLength);

    if (*bufferCrc != crc || header->syncByte != SMARTAUDIO_SYNC_BYTE || header->headerByte != SMARTAUDIO_HEADER_BYTE) {
        return false;
    }
    switch (header->command) {
        case SMARTAUDIO_RSP_GET_SETTINGS_V1: {
                const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
                settings->version = 1;
                smartaudioUnpackSettings(settings, resp);
            }
            break;
        case SMARTAUDIO_RSP_GET_SETTINGS_V2: {
                const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
                settings->version = 2;
                smartaudioUnpackSettings(settings, resp);
            }
            break;
        case SMARTAUDIO_RSP_SET_POWER: {
                const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
                settings->channel = (resp->payload >> 8) & 0xFF;
                settings->power = resp->payload & 0xFF;
            }
            break;
        case SMARTAUDIO_RSP_SET_CHANNEL: {
                const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t *)buffer;
                settings->channel = resp->payload;
            }
            break;
        case SMARTAUDIO_RSP_SET_FREQUENCY: {
                const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
                smartaudioUnpackFrequency(settings, resp->payload);
            }
            break;
        case SMARTAUDIO_RSP_SET_MODE: {
                const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t*)buffer;
                smartaudioUnpackOperationMode(settings, resp->payload, false);
            }
            break;
        default:
            return false;
    }
    return true;
}
