
#pragma once
#ifdef STATS

typedef struct statsConfig_s {
    uint32_t stats_total_time; // [s]
    uint32_t stats_total_dist; // [m]
    uint8_t  stats_enabled;
} statsConfig_t;

void statsOnArm(void);
void statsOnDisarm(void);

#else

#define statsOnArm()    do {} while (0)
#define statsOnDisarm() do {} while (0)

#endif
