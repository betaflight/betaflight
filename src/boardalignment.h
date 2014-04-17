#pragma once

typedef struct boardAlignment_s {
    int16_t rollDegrees;
    int16_t pitchDegrees;
    int16_t yawDegrees;
} boardAlignment_t;

void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation);
void initBoardAlignment(boardAlignment_t *boardAlignment);
