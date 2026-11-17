/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ALLYSTAR_VELNED_COMMAND_LENGTH 11
#define ALLYSTAR_TIME_COMMAND_LENGTH 9
#define ALLYSTAR_PWRCTL2_COMMAND_LENGTH 24

typedef struct allystarVelned_s {
    uint32_t iTow;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed3d;
    uint32_t groundSpeed;
    int32_t heading;
    uint32_t sAcc;
} allystarVelned_t;

typedef struct allystarParser_s {
    uint8_t state;
    uint8_t messageClass;
    uint8_t messageId;
    uint8_t checksumA;
    uint8_t checksumB;
    uint16_t payloadLength;
    uint16_t payloadOffset;
    uint8_t payload[36];
    uint32_t frameStartMs;
    uint32_t lastByteMs;
    uint32_t velnedReceivedMs;
    allystarVelned_t velned;
    bool velnedPending;
    uint32_t timeReceivedMs;
    int16_t leapSeconds;
    bool timeValid;
} allystarParser_t;

void allystarParserInit(allystarParser_t *parser);
bool allystarParserConsume(allystarParser_t *parser, uint8_t byte, uint32_t nowMs);
bool allystarTakeFresh(allystarParser_t *parser, uint32_t nowMs, allystarVelned_t *velned);
bool allystarTakeForUtc(allystarParser_t *parser, uint32_t nowMs, uint32_t utcMs, uint32_t utcResolutionMs, allystarVelned_t *velned);
uint8_t allystarClampRate(uint8_t rateHz);
uint8_t allystarBuildPwrctl2(uint8_t *frame, uint8_t rateHz);
const uint8_t *allystarVelnedCommand(void);
const uint8_t *allystarTimeCommand(void);
