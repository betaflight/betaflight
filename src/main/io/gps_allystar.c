/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 */

#include "platform.h"

#if ENABLE_GPS_ALLYSTAR

#include <string.h>

#include "io/gps_allystar.h"

#define ALLYSTAR_INTERBYTE_TIMEOUT_MS 50U
#define ALLYSTAR_FRAME_TIMEOUT_MS 2000U
#define ALLYSTAR_MAX_IGNORED_PAYLOAD 256U
#define ALLYSTAR_VELNED_MAX_AGE_MS 250U
#define ALLYSTAR_TIME_MAX_AGE_MS 60000U
#define ALLYSTAR_GPS_WEEK_MS 604800000U
#define ALLYSTAR_DAY_MS 86400000U
#define ALLYSTAR_NAV_TIME_ID 0x05
#define ALLYSTAR_NAV_VELNED_CLASS 0x01
#define ALLYSTAR_NAV_VELNED_ID 0x12
#define ALLYSTAR_CFG_CLASS 0x06
#define ALLYSTAR_CFG_MSG_ID 0x01
#define ALLYSTAR_CFG_PWRCTL2_ID 0x44

typedef enum {
    WAIT_SYNC_1,
    WAIT_SYNC_2,
    READ_CLASS,
    READ_ID,
    READ_LENGTH_LO,
    READ_LENGTH_HI,
    READ_PAYLOAD,
    DISCARD_PAYLOAD,
    READ_CHECKSUM_A,
    READ_CHECKSUM_B,
} parseState_e;

static const uint8_t cfgVelned[] = {
    0xf1, 0xd9, ALLYSTAR_CFG_CLASS, ALLYSTAR_CFG_MSG_ID,
    0x03, 0x00, ALLYSTAR_NAV_VELNED_CLASS, ALLYSTAR_NAV_VELNED_ID,
    0x01, 0x1e, 0x67,
};

static const uint8_t pollTime[] = {
    0xf1, 0xd9, ALLYSTAR_NAV_VELNED_CLASS, ALLYSTAR_NAV_TIME_ID,
    0x01, 0x00, 0x00, 0x07, 0x1c,
};

static uint32_t readU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void writeU32(uint8_t *p, uint32_t value)
{
    p[0] = value;
    p[1] = value >> 8;
    p[2] = value >> 16;
    p[3] = value >> 24;
}

static void addChecksum(allystarParser_t *parser, uint8_t byte)
{
    parser->checksumA += byte;
    parser->checksumB += parser->checksumA;
}

void allystarParserInit(allystarParser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

static void resetFrame(allystarParser_t *parser)
{
    parser->state = WAIT_SYNC_1;
    parser->payloadLength = 0;
    parser->payloadOffset = 0;
    parser->checksumA = 0;
    parser->checksumB = 0;
}

static void commitVelned(allystarParser_t *parser, uint32_t nowMs)
{
    if (parser->messageClass != ALLYSTAR_NAV_VELNED_CLASS ||
        parser->messageId != ALLYSTAR_NAV_VELNED_ID ||
        parser->payloadLength != sizeof(parser->payload)) {
        return;
    }

    const uint8_t *payload = parser->payload;
    parser->velned.iTow = readU32(payload);
    parser->velned.velN = (int32_t)readU32(payload + 4);
    parser->velned.velE = (int32_t)readU32(payload + 8);
    parser->velned.velD = (int32_t)readU32(payload + 12);
    parser->velned.speed3d = readU32(payload + 16);
    parser->velned.groundSpeed = readU32(payload + 20);
    parser->velned.heading = (int32_t)readU32(payload + 24);
    parser->velned.sAcc = readU32(payload + 28);
    parser->velnedReceivedMs = nowMs;
    parser->velnedPending = true;
}

static void commitTime(allystarParser_t *parser, uint32_t nowMs)
{
    if (parser->messageClass != ALLYSTAR_NAV_VELNED_CLASS || parser->messageId != ALLYSTAR_NAV_TIME_ID) {
        return;
    }

    const uint8_t *payload = parser->payload;
    const int16_t leapSeconds = (int16_t)((uint16_t)payload[10] | ((uint16_t)payload[11] << 8));
    const bool valid = parser->payloadLength == 16 && payload[0] == 0 &&
        (payload[1] & 0x06) == 0x06 && leapSeconds >= 0 && leapSeconds <= 100 &&
        readU32(payload + 4) < ALLYSTAR_GPS_WEEK_MS;
    if (!valid || !parser->timeValid || leapSeconds != parser->leapSeconds ||
        (uint32_t)(nowMs - parser->timeReceivedMs) >= ALLYSTAR_TIME_MAX_AGE_MS) {
        parser->velnedPending = false;
    }
    parser->timeValid = valid;
    if (valid) {
        parser->leapSeconds = leapSeconds;
        parser->timeReceivedMs = nowMs;
    }
}

bool allystarParserConsume(allystarParser_t *parser, uint8_t byte, uint32_t nowMs)
{
    if (parser->state != WAIT_SYNC_1 &&
        ((uint32_t)(nowMs - parser->lastByteMs) > ALLYSTAR_INTERBYTE_TIMEOUT_MS ||
         (uint32_t)(nowMs - parser->frameStartMs) > ALLYSTAR_FRAME_TIMEOUT_MS)) {
        resetFrame(parser);
    }
    parser->lastByteMs = nowMs;

    switch ((parseState_e)parser->state) {
    case WAIT_SYNC_1:
        if (byte != 0xf1) {
            return false;
        }
        parser->frameStartMs = nowMs;
        parser->state = WAIT_SYNC_2;
        return true;
    case WAIT_SYNC_2:
        if (byte == 0xd9) {
            parser->state = READ_CLASS;
            return true;
        }
        resetFrame(parser);
        return byte != '$';
    case READ_CLASS:
        parser->messageClass = byte;
        addChecksum(parser, byte);
        parser->state = READ_ID;
        return true;
    case READ_ID:
        parser->messageId = byte;
        addChecksum(parser, byte);
        parser->state = READ_LENGTH_LO;
        return true;
    case READ_LENGTH_LO:
        parser->payloadLength = byte;
        addChecksum(parser, byte);
        parser->state = READ_LENGTH_HI;
        return true;
    case READ_LENGTH_HI:
        parser->payloadLength |= (uint16_t)byte << 8;
        addChecksum(parser, byte);
        if (parser->payloadLength > ALLYSTAR_MAX_IGNORED_PAYLOAD) {
            resetFrame(parser);
        } else if (parser->payloadLength > sizeof(parser->payload)) {
            parser->payloadOffset = 0;
            parser->state = DISCARD_PAYLOAD;
        } else {
            parser->state = parser->payloadLength ? READ_PAYLOAD : READ_CHECKSUM_A;
        }
        return true;
    case READ_PAYLOAD:
        addChecksum(parser, byte);
        parser->payload[parser->payloadOffset++] = byte;
        if (parser->payloadOffset == parser->payloadLength) {
            parser->state = READ_CHECKSUM_A;
        }
        return true;
    case DISCARD_PAYLOAD:
        addChecksum(parser, byte);
        if (++parser->payloadOffset == parser->payloadLength) {
            parser->state = READ_CHECKSUM_A;
        }
        return true;
    case READ_CHECKSUM_A:
        if (byte != parser->checksumA) {
            resetFrame(parser);
        } else {
            parser->state = READ_CHECKSUM_B;
        }
        return true;
    case READ_CHECKSUM_B:
        if (byte == parser->checksumB) {
            commitVelned(parser, nowMs);
            commitTime(parser, nowMs);
        }
        resetFrame(parser);
        return true;
    }

    resetFrame(parser);
    return true;
}

bool allystarTakeFresh(allystarParser_t *parser, uint32_t nowMs, allystarVelned_t *velned)
{
    const bool fresh = parser->velnedPending &&
        (uint32_t)(nowMs - parser->velnedReceivedMs) <= ALLYSTAR_VELNED_MAX_AGE_MS;
    parser->velnedPending = false;
    if (fresh) {
        *velned = parser->velned;
    }
    return fresh;
}

bool allystarTakeForUtc(allystarParser_t *parser, uint32_t nowMs, uint32_t utcMs, uint32_t utcResolutionMs, allystarVelned_t *velned)
{
    if (parser->timeValid && (uint32_t)(nowMs - parser->timeReceivedMs) >= ALLYSTAR_TIME_MAX_AGE_MS) {
        parser->timeValid = false;
        parser->velnedPending = false;
    }
    if (parser->velnedPending &&
        (uint32_t)(nowMs - parser->velnedReceivedMs) > ALLYSTAR_VELNED_MAX_AGE_MS) {
        parser->velnedPending = false;
    }
    if (!parser->timeValid || !parser->velnedPending ||
        parser->velned.iTow >= ALLYSTAR_GPS_WEEK_MS || utcMs >= ALLYSTAR_DAY_MS) {
        return false;
    }
    const uint32_t velocityUtcMs = (parser->velned.iTow % ALLYSTAR_DAY_MS + ALLYSTAR_DAY_MS -
        (uint32_t)parser->leapSeconds * 1000U) % ALLYSTAR_DAY_MS;
    // A centisecond NMEA field can round or truncate a millisecond GNSS epoch.
    // Bound this allowance below the minimum 100 ms navigation period.
    const uint32_t toleranceMs = utcResolutionMs > 0 && utcResolutionMs <= 10 ? utcResolutionMs - 1 : 0;
    const uint32_t deltaMs = velocityUtcMs > utcMs ? velocityUtcMs - utcMs : utcMs - velocityUtcMs;
    if (deltaMs > toleranceMs && ALLYSTAR_DAY_MS - deltaMs > toleranceMs) {
        return false;
    }
    *velned = parser->velned;
    parser->velnedPending = false;
    return true;
}

uint8_t allystarClampRate(uint8_t rateHz)
{
    if (rateHz < 1) {
        return 1;
    }
    return rateHz > 10 ? 10 : rateHz;
}

uint8_t allystarBuildPwrctl2(uint8_t *frame, uint8_t rateHz)
{
    const uint8_t effectiveRate = allystarClampRate(rateHz);
    memset(frame, 0, ALLYSTAR_PWRCTL2_COMMAND_LENGTH);
    frame[0] = 0xf1;
    frame[1] = 0xd9;
    frame[2] = ALLYSTAR_CFG_CLASS;
    frame[3] = ALLYSTAR_CFG_PWRCTL2_ID;
    frame[4] = 16;
    frame[8] = 1;
    writeU32(frame + 10, effectiveRate);
    writeU32(frame + 14, 1000U / effectiveRate);

    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    for (unsigned i = 2; i < 22; i++) {
        checksumA += frame[i];
        checksumB += checksumA;
    }
    frame[22] = checksumA;
    frame[23] = checksumB;
    return ALLYSTAR_PWRCTL2_COMMAND_LENGTH;
}

const uint8_t *allystarVelnedCommand(void)
{
    return cfgVelned;
}

const uint8_t *allystarTimeCommand(void)
{
    return pollTime;
}

#endif // ENABLE_GPS_ALLYSTAR
