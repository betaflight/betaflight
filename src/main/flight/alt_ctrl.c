#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "flight/alt_ctrl.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"

//static int32_t TFMINI_Altitude = 0;
//static int32_t TFMINI_estimatedAltitude = 0;

float alt_ctrl_run(uint32_t z_ref)
{
    static float throttle_alt = 0.25;
    // static timeUs_t previousTimeUs = 0;
    //  const uint32_t dTime = currentTimeUs - previousTimeUs;

    z_ref = rangefinderGetLatestAltitude();

    throttle_alt = 0.405;

    // if(z_ref == 100)
    // {
//    rcData[THROTTLE] = 1300;
//    }
    return throttle_alt;
}
