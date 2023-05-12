#pragma once

#define USE_GPS

#include "platform.h"

typedef enum {
    UNDEF,
    M8,
    M10
} ubloxVersion_e;

ubloxVersion_e ubloxDetectVersion(char * c);
