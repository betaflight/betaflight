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

#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "tramp_protocol.h"

#define TRAMP_SYNC_START            0x0F
#define TRAMP_SYNC_STOP             0x00
#define TRAMP_COMMAND_SET_FREQ      'F' // 0x46
#define TRAMP_COMMAND_SET_POWER     'P' // 0x50
#define TRAMP_COMMAND_ACTIVE_STATE  'I' // 0x49
#define TRAMP_COMMAND_GET_CONFIG    'v' // 0x76

static uint8_t trampCrc(const trampFrame_t *frame)
{
    uint8_t crc = 0;
    const uint8_t *p = (const uint8_t *)frame;
    const uint8_t *pEnd = p + (TRAMP_HEADER_LENGTH + TRAMP_PAYLOAD_LENGTH);
    for (; p != pEnd; p++) {
        crc += *p;
    }
    return crc;
}

static void trampFrameInit(uint8_t frameType, trampFrame_t *frame)
{
    frame->header.syncStart = TRAMP_SYNC_START;
    frame->header.command = frameType;
    const uint8_t emptyPayload[TRAMP_PAYLOAD_LENGTH] = { 0 };
    memcpy(frame->payload.buf, emptyPayload, sizeof(frame->payload.buf));
}

static void trampFrameClose(trampFrame_t *frame)
{
    frame->footer.crc = trampCrc(frame);
    frame->footer.syncStop = TRAMP_SYNC_STOP;
}

void trampFrameGetSettings(trampFrame_t *frame)
{
    trampFrameInit(TRAMP_COMMAND_GET_CONFIG, frame);
    trampFrameClose(frame);
}

void trampFrameSetFrequency(trampFrame_t *frame, const uint16_t frequency)
{
    trampFrameInit(TRAMP_COMMAND_SET_FREQ, frame);
    frame->payload.frequency = frequency;
    trampFrameClose(frame);
}

void trampFrameSetPower(trampFrame_t *frame, const uint16_t power)
{
    trampFrameInit(TRAMP_COMMAND_SET_POWER, frame);
    frame->payload.power = power;
    trampFrameClose(frame);
}

void trampFrameSetActiveState(trampFrame_t *frame, const bool active)
{
    trampFrameInit(TRAMP_COMMAND_ACTIVE_STATE, frame);
    frame->payload.active = (uint8_t) active;
    trampFrameClose(frame);
}

bool trampParseResponseBuffer(trampSettings_t *settings, const uint8_t *buffer, size_t bufferLen)
{
    if (bufferLen != TRAMP_FRAME_LENGTH) {
        return false;
    }
    const trampFrame_t *frame = (const trampFrame_t *)buffer;
    const uint8_t crc = trampCrc(frame);
    if (crc != frame->footer.crc) {
        return false;
    }
    memcpy(settings, &frame->payload.settings, sizeof(*settings));
    return true;
}
