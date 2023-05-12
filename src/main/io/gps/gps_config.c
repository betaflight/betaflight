
#include <string.h>
#include "gps_config.h"

#include "platform.h"

#ifdef USE_GPS

ubloxVersion_e ubloxDetectVersion(char * c) {
    ubloxVersion_e ret = UNDEF;
    if (strcmp(c, "000A0000") == 0) {
        ret = M10;
    } else if (strcmp(c, "00800000") == 0) {
        ret = M8;
    }
    return ret;
}

#endif
