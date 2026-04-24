#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"

// Freeze reason bitmask — reported in debug output
typedef enum {
    ADAPTIVE_FREEZE_NONE         = 0,
    ADAPTIVE_FREEZE_DISARMED     = (1 << 0),
    ADAPTIVE_FREEZE_LEARN_DELAY  = (1 << 1),
    ADAPTIVE_FREEZE_BIG_STICK    = (1 << 2),
    ADAPTIVE_FREEZE_FLIGHT_MODE  = (1 << 3),
    ADAPTIVE_FREEZE_GYRO_ERROR   = (1 << 4),
    ADAPTIVE_FREEZE_LOW_THROTTLE = (1 << 5),
    ADAPTIVE_FREEZE_MOTOR_SAT    = (1 << 6),
} adaptiveFreezeReason_e;

// Direction of last cutoff adjustment
typedef enum {
    ADAPTIVE_STATE_HOLD    = 0,
    ADAPTIVE_STATE_LOWER   = 1,  // more filtering
    ADAPTIVE_STATE_RAISE   = 2,  // less filtering
    ADAPTIVE_STATE_FROZEN  = 3,
    ADAPTIVE_STATE_CONFIG  = 4,  // enabled, but current filter config is unsupported
} adaptiveState_e;

// Called once at init / profile change
void adaptiveFilterInit(void);

// Called every PID loop for each axis — feeds D-term value
void adaptiveFilterPushDterm(int axis, float dterm);

// Called every PID loop — feeds freeze-condition inputs
// throttle: 0..1, setpointRate: deg/s, gyroError: deg/s, motorMixRange: 0..1
void adaptiveFilterUpdate(float throttle, float setpointRate, float gyroError, float motorMixRange);

// Returns current adaptive cutoff for axis (all axes share one cutoff in v1)
float adaptiveFilterGetCutoff(void);

// Returns whether learning is currently frozen
bool adaptiveFilterIsFrozen(void);

// Returns freeze reason bitmask
uint8_t adaptiveFilterGetFreezeReason(void);

// Returns last noise estimate (for debug)
float adaptiveFilterGetNoiseLevel(void);

// Returns current state
adaptiveState_e adaptiveFilterGetState(void);
