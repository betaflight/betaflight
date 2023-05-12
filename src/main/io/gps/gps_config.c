
#include "platform.h"

#ifdef USE_GPS

#include "gps_config.h"

ubloxVersion_e ubloxDetectVersion(const char * szBuf, const uint8_t nBufSize) {
    ubloxVersion_e ret = UNDEF;
    // ublox_5   hwVersion 00040005
    if (strncmp(szBuf, "00040005", nBufSize) == 0) {
        ret = M5;
    } else if (strncmp(szBuf, "00040007", nBufSize) == 0) {
        ret = M6;
    } else if (strncmp(szBuf, "00070000", nBufSize) == 0) {
        ret = M7;
    } else if (strncmp(szBuf, "00080000", nBufSize) == 0) {
        ret = M8;
    } else if (strncmp(szBuf, "00190000", nBufSize) == 0) {
        ret = M9;
    } else if (strncmp(szBuf, "000A0000", nBufSize) == 0) {
        ret = M10;
    }
    return ret;
}

char* ubloxVersionToString(ubloxVersion_e version) {
    switch (version) {
        case M5:
            return "M5";
        case M6:
            return "M6";
        case M7:
            return "M7";
        case M8:
            return "M8";
        case M9:
            return "M9";
        case M10:
            return "M10";
        default:
            break;
    }
    return "UNDEF";
}

#endif
