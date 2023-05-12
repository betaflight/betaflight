#pragma once

#ifdef USE_GPS

#include <string.h>

typedef enum {
    UNDEF,
    M5,
    M6,
    M7,
    M8,
    M9,
    M10
} ubloxVersion_e;

ubloxVersion_e ubloxDetectVersion(const char * szBuf, const uint8_t nBufSize);
char* ubloxVersionToString(ubloxVersion_e version);

#endif