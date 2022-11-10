#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "flight/alt_ctrl.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"

static int32_t TFMINI_Altitude = 0;
static int32_t TFMINI_estimatedAltitude = 0;

void alt_ctrl_run(timeUs_t currentTimeUs, float z_ref)
{
    if (sensors(SENSOR_RANGEFINDER) && rangefinderProcess(getCosTiltAngle())) {
        TFMINI_Altitude = rangefinderGetLatestAltitude();
    }
}