
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "gps_config.h"

#include "platform.h"

#ifdef USE_GPS

ubloxVersion_e ubloxVersion = UNDEF;

static void ubloxDetectVersion(uint16_t * c) {
    ubloxVersion = M8;
}

#endif