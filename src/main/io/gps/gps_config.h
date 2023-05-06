#pragma once

#include "platform.h"

#include "io/gps/gps.h"

typedef enum {
    UNDEF,
    M8,
    M10
} ubloxVersion_e;

ubloxVersion_e ubloxDetectVersion(serialPort_t *instance);
void sendMessage(const uint8_t *data, uint8_t len);