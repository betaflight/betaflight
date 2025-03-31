#pragma once
#include "pid.h"
void afcsInit(const pidProfile_t *pidProfile);
void FAST_CODE afcsUpdate(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);
