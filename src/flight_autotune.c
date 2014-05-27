#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system_common.h"

#include "flight_common.h"

extern int16_t debug[4];

// To adjust how aggressive the tuning is, adjust the AUTOTUNEMAXOSCILLATION value.  A larger
// value will result in more aggressive tuning. A lower value will result in softer tuning.
// It will rock back and forth between -AUTOTUNE_TARGET_ANGLE and AUTOTUNE_TARGET_ANGLE degrees
// AUTOTUNE_D_MULTIPLIER is a multiplier that puts in a little extra D when autotuning is done. This helps damp
// the wobbles after a quick angle change.
// Always autotune on a full battery.

#define AUTOTUNE_MAX_OSCILLATION 1.0f
#define AUTOTUNE_TARGET_ANGLE 20.0f
#define AUTOTUNE_D_MULTIPLIER 1.2f
#define AUTOTUNE_SETTLING_DELAY_MS 250  // 1/4 of a second.

#define AUTOTUNE_INCREASE_MULTIPLIER 1.03f
#define AUTOTUNE_DECREASE_MULTIPLIER 0.97f

#define AUTOTUNE_MINIMUM_I_VALUE 0.001f

#define YAW_GAIN_MULTIPLIER 2.0f


typedef enum {
    PHASE_IDLE = 0,
    PHASE_TUNE_ROLL,
    PHASE_TUNE_PITCH,
    PHASE_SAVE_OR_RESTORE_PIDS,
} autotunePhase_e;

static const pidIndex_e angleIndexToPidIndexMap[] = {
    PIDROLL,
    PIDPITCH
};

#define AUTOTUNE_PHASE_MAX PHASE_SAVE_OR_RESTORE_PIDS
#define AUTOTUNE_PHASE_COUNT (AUTOTUNE_PHASE_MAX + 1)

#define FIRST_TUNE_PHASE PHASE_TUNE_ROLL

static pidProfile_t *pidProfile;
static pidProfile_t pidBackup;
static uint8_t pidController;
static uint8_t pidIndex;
static bool rising;
static uint8_t cycleCount; // TODO can we replace this with an enum to improve readability.
static uint32_t timeoutAt;
static angle_index_t autoTuneAngleIndex;
static autotunePhase_e phase = PHASE_IDLE;
static autotunePhase_e nextPhase = FIRST_TUNE_PHASE;

static float targetAngle = 0;
static float targetAngleAtPeak;
static float secondPeakAngle, firstPeakAngle; // deci dgrees, 180 deg = 1800

typedef struct fp_pid {
    float p;
    float i;
    float d;
} fp_pid_t;

static fp_pid_t pid;

// These are used to convert between multiwii integer values to the float pid values used by the autotuner.
#define MULTIWII_P_MULTIPLIER 10.0f    // e.g 0.4 * 10 = 40
#define MULTIWII_I_MULTIPLIER 1000.0f  // e.g 0.030 * 1000 = 30
// Note there is no D multiplier since D values are stored and used AS-IS

bool isAutotuneIdle(void)
{
    return phase == PHASE_IDLE;
}

static void startNewCycle(void)
{
    rising = !rising;
    secondPeakAngle = firstPeakAngle = 0;
}

static void updatePidIndex(void)
{
    pidIndex = angleIndexToPidIndexMap[autoTuneAngleIndex];
}

static void updateTargetAngle(void)
{
    if (rising) {
        targetAngle = AUTOTUNE_TARGET_ANGLE;
    }
    else  {
        targetAngle = -AUTOTUNE_TARGET_ANGLE;
    }

#if 0
    debug[2] = DEGREES_TO_DECIDEGREES(targetAngle);
#endif
}

float autotune(angle_index_t angleIndex, const rollAndPitchInclination_t *inclination, float errorAngle)
{
    float currentAngle;

    if (!(phase == PHASE_TUNE_ROLL || phase == PHASE_TUNE_PITCH) || autoTuneAngleIndex != angleIndex) {
        return errorAngle;
    }

    if (pidController == 2) {
        // TODO support new baseflight pid controller
        return errorAngle;
    }

    debug[0] = 0;

#if 0
    debug[1] = inclination->rawAngles[angleIndex];
#endif

    updatePidIndex();

    if (rising) {
        currentAngle = DECIDEGREES_TO_DEGREES(inclination->rawAngles[angleIndex]);
    } else {
        targetAngle = -targetAngle;
        currentAngle = DECIDEGREES_TO_DEGREES(-inclination->rawAngles[angleIndex]);
    }

#if 1
    debug[1] = DEGREES_TO_DECIDEGREES(currentAngle);
    debug[2] = DEGREES_TO_DECIDEGREES(targetAngle);
#endif

    if (firstPeakAngle == 0) {
        // The peak will be when our angular velocity is negative.  To be sure we are in the right place,
        // we also check to make sure our angle position is greater than zero.

        if (currentAngle > secondPeakAngle) {
            // we are still going up
            secondPeakAngle = currentAngle;
            targetAngleAtPeak = targetAngle;

            debug[3] = DEGREES_TO_DECIDEGREES(secondPeakAngle);

        } else if (secondPeakAngle > 0) {
            if (cycleCount == 0) {
                // when checking the I value, we would like to overshoot the target position by half of the max oscillation.
                if (currentAngle - targetAngle < AUTOTUNE_MAX_OSCILLATION / 2) {
                    pid.i *= AUTOTUNE_INCREASE_MULTIPLIER;
                } else {
                    pid.i *= AUTOTUNE_DECREASE_MULTIPLIER;
                    if (pid.i < AUTOTUNE_MINIMUM_I_VALUE) {
                        pid.i = AUTOTUNE_MINIMUM_I_VALUE;
                    }
                }

                // go back to checking P and D
                cycleCount = 1;
                pidProfile->I8[pidIndex] = 0;
                startNewCycle();
            } else {
                // we are checking P and D values
                // set up to look for the 2nd peak
                firstPeakAngle = currentAngle;
                timeoutAt = millis() + AUTOTUNE_SETTLING_DELAY_MS;
            }
        }
    } else {
        // we saw the first peak. looking for the second

        if (currentAngle < firstPeakAngle) {
            firstPeakAngle = currentAngle;
            debug[3] = DEGREES_TO_DECIDEGREES(firstPeakAngle);
        }

        float oscillationAmplitude = secondPeakAngle - firstPeakAngle;

        uint32_t now = millis();
        int32_t signedDiff = now - timeoutAt;
        bool timedOut = signedDiff >= 0L;

        // stop looking for the 2nd peak if we time out or if we change direction again after moving by more than half the maximum oscillation
        if (timedOut || (oscillationAmplitude > AUTOTUNE_MAX_OSCILLATION / 2 && currentAngle > firstPeakAngle)) {
            // analyze the data
            // Our goal is to have zero overshoot and to have AUTOTUNEMAXOSCILLATION amplitude

            if (secondPeakAngle > targetAngleAtPeak) {
                // overshot
                debug[0] = 1;

#ifdef PREFER_HIGH_GAIN_SOLUTION
                if (oscillationAmplitude > AUTOTUNE_MAX_OSCILLATION) {
                    // we have too much oscillation, so we can't increase D, so decrease P
#endif
                    pid.p *= AUTOTUNE_DECREASE_MULTIPLIER;
#ifdef PREFER_HIGH_GAIN_SOLUTION
                } else {
                    // we don't have too much oscillation, so we can increase D
#endif
                    pid.d *= AUTOTUNE_INCREASE_MULTIPLIER;
#ifdef PREFER_HIGH_GAIN_SOLUTION
                }
#endif
            } else {
                // undershot
                debug[0] = 2;

                if (oscillationAmplitude > AUTOTUNE_MAX_OSCILLATION) {
                    // we have too much oscillation
                    pid.d *= AUTOTUNE_DECREASE_MULTIPLIER;
                } else {
                    // we don't have too much oscillation
                    pid.p *= AUTOTUNE_INCREASE_MULTIPLIER;
                }
            }

            pidProfile->P8[pidIndex] = pid.p * MULTIWII_P_MULTIPLIER;
            pidProfile->D8[pidIndex] = pid.d;

            // switch to the other direction and start a new cycle
            startNewCycle();

            if (++cycleCount == 3) {
                // switch to testing I value
                cycleCount = 0;

                pidProfile->I8[pidIndex] = pid.i  * MULTIWII_I_MULTIPLIER;
            }
        }
    }

    if (angleIndex == AI_ROLL) {
        debug[0] += 100;
    }

    updateTargetAngle();

    return targetAngle - DECIDEGREES_TO_DEGREES(inclination->rawAngles[angleIndex]);
}

void autotuneReset(void)
{
    targetAngle = 0;
    phase = PHASE_IDLE;
    nextPhase = FIRST_TUNE_PHASE;
}

void backupPids(pidProfile_t *pidProfileToTune)
{
    memcpy(&pidBackup, pidProfileToTune, sizeof(pidBackup));
}

void restorePids(pidProfile_t *pidProfileToTune)
{
    memcpy(pidProfileToTune, &pidBackup, sizeof(pidBackup));
}

void autotuneBeginNextPhase(pidProfile_t *pidProfileToTune, uint8_t pidControllerInUse)
{
    phase = nextPhase;

    if (phase == PHASE_SAVE_OR_RESTORE_PIDS) {
        restorePids(pidProfileToTune);
        return;
    }

    if (phase == FIRST_TUNE_PHASE) {
        backupPids(pidProfileToTune);
    }

    if (phase == PHASE_TUNE_ROLL) {
        autoTuneAngleIndex = AI_ROLL;
    } if (phase == PHASE_TUNE_PITCH) {
        autoTuneAngleIndex = AI_PITCH;
    }

    rising = true;
    cycleCount = 1;
    secondPeakAngle = firstPeakAngle = 0;

    pidProfile = pidProfileToTune;
    pidController = pidControllerInUse;

    updatePidIndex();
    updateTargetAngle();

    pid.p = pidProfile->P8[pidIndex] / MULTIWII_P_MULTIPLIER;
    pid.i = pidProfile->I8[pidIndex] / MULTIWII_I_MULTIPLIER;
    // divide by D multiplier to get our working value.  We'll multiply by D multiplier when we are done.
    pid.d = pidProfile->D8[pidIndex] * (1.0f / AUTOTUNE_D_MULTIPLIER);

    pidProfile->D8[pidIndex] = pid.d;
    pidProfile->I8[pidIndex] = 0;
}

void autotuneEndPhase(void)
{
    if (phase == PHASE_TUNE_ROLL || phase == PHASE_TUNE_PITCH) {

        // we leave P alone, just update I and D

        pidProfile->I8[pidIndex] = pid.i * MULTIWII_I_MULTIPLIER;

        // multiply by D multiplier.  The best D is usually a little higher than what the algroithm produces.
        pidProfile->D8[pidIndex] = (pid.d * AUTOTUNE_D_MULTIPLIER);

        pidProfile->P8[PIDYAW] = pidProfile->P8[PIDROLL] * YAW_GAIN_MULTIPLIER;
        pidProfile->I8[PIDYAW] = pidProfile->I8[PIDROLL];
        pidProfile->D8[PIDYAW] = pidProfile->D8[PIDROLL];
    }

    if (phase == AUTOTUNE_PHASE_MAX) {
        phase = PHASE_IDLE;
        nextPhase = FIRST_TUNE_PHASE;
    } else {
        nextPhase++;
    }
}

bool havePidsBeenUpdatedByAutotune(void)
{
    return targetAngle != 0;
}

