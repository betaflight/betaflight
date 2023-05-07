#pragma once

#define USE_GPS

#include "platform.h"

#include "io/gps/gps.h"


ubloxVersion_e ubloxDetectVersion(serialPort_t *instance);