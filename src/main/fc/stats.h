
#pragma once
#ifdef USE_STATS

void statsOnArm(void);
void statsOnDisarm(void);
void statsOnLoop(void);

#else

#define statsOnArm()    do {} while (0)
#define statsOnDisarm() do {} while (0)
#define statsOnLoop()   do {} while (0)

#endif
