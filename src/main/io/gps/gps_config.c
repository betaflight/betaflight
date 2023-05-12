
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "gps_config.h"

#include "platform.h"

#ifdef USE_GPS

ubloxVersion_e ubloxDetectVersion(uint16_t * c) {
    UNUSED(c);
    return UNDEF;
}

#endif