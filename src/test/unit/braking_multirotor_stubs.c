#include <stdbool.h>

#include "common/time.h"

void brakingUpdate(timeUs_t currentTimeUs)
{
    (void)currentTimeUs;
}

bool isBrakingActive(void)
{
    return false;
}

float getBrakingSetpoint(int axis)
{
    (void)axis;
    return 0.0f;
}
