#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "common/maths.h"

#include "rc_controls.h"

int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

bool areSticksInApModePosition(uint16_t ap_mode)
{
    return abs(rcCommand[ROLL]) < ap_mode && abs(rcCommand[PITCH]) < ap_mode;
}
