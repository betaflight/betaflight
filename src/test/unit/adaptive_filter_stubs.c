#include <stdbool.h>
#include <stdint.h>

#include "flight/adaptive_filter.h"

void adaptiveFilterInit(void)
{
}

void adaptiveFilterPushDterm(int axis, float dterm)
{
    (void)axis;
    (void)dterm;
}

void adaptiveFilterUpdate(float throttle, float setpointRate, float gyroError, float motorMixRange)
{
    (void)throttle;
    (void)setpointRate;
    (void)gyroError;
    (void)motorMixRange;
}

float adaptiveFilterGetCutoff(void)
{
    return 0.0f;
}

bool adaptiveFilterIsFrozen(void)
{
    return false;
}

uint8_t adaptiveFilterGetFreezeReason(void)
{
    return 0;
}

float adaptiveFilterGetNoiseLevel(void)
{
    return 0.0f;
}

adaptiveState_e adaptiveFilterGetState(void)
{
    return ADAPTIVE_STATE_HOLD;
}
