#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "flight_common.h"

// To adjust how agressive the tuning is, adjust the AUTOTUNEMAXOSCILLATION value.  A larger
// value will result in more agressive tuning. A lower value will result in softer tuning.
// It will rock back and forth between -AUTOTUNE_TARGET_ANGLE and AUTOTUNE_TARGET_ANGLE degrees
// AUTOTUNE_D_MULTIPLIER is a multiplier that puts in a little extra D when autotuning is done. This helps damp
// the wobbles after a quick angle change.
// Always autotune on a full battery.
#define AUTOTUNE_MAX_OSCILLATION 1.0
#define AUTOTUNE_TARGET_ANGLE 20.0
#define AUTOTUNE_D_MULTIPLIER 1.2

static pidProfile_t *pidProfile;
static uint8_t pidController;
static bool rising;
static float targetAngle = 0;

static angle_index_t autoTuneAngleIndex;

float autotune(angle_index_t angleIndex, rollAndPitchInclination_t *inclination, float errorAngle)
{
    if (autoTuneAngleIndex != angleIndex) {
        // Not tuning this angle yet.
        return errorAngle;
    }

    // TODO autotune!

    if (rising) {
        targetAngle = AUTOTUNE_TARGET_ANGLE;
    }
    else  {
        targetAngle = -AUTOTUNE_TARGET_ANGLE;
    }


    return targetAngle - inclination->rawAngles[angleIndex];
}

void autotuneReset(void)
{
    targetAngle = 0;
    rising = true;
    autoTuneAngleIndex = AI_ROLL;
}

void autotuneBegin(pidProfile_t *pidProfileToTune, uint8_t pidControllerInUse)
{
    autotuneReset();

    pidProfile = pidProfileToTune;
    pidController = pidControllerInUse;
}

void autotuneEnd(void)
{

}

bool havePidsBeenUpdatedByAutotune(void)
{
    return targetAngle != 0;
}

