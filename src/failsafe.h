#pragma once

extern int16_t failsafeCnt;
extern int16_t failsafeEvents;

bool isFailsafeIdle(void);
bool hasFailsafeTimerElapsed(void);
bool shouldFailsafeForceLanding(bool armed);
bool shouldFailsafeHaveCausedLandingByNow(void);
void updateFailsafeState(void);
