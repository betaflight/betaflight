#pragma once

typedef enum GimbalFlags {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_MIXTILT = 1 << 1,
    GIMBAL_FORWARDAUX = 1 << 2,
} GimbalFlags;

typedef struct gimbalConfig_s {
    uint8_t gimbal_flags;                   // in servotilt mode, various things that affect stuff
} gimbalConfig_t;
