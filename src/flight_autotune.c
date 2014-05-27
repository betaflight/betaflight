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

typedef enum {
    PHASE_IDLE = 0,
    PHASE_TUNE_ROLL,
    PHASE_TUNE_PITCH,
    PHASE_SAVE_OR_RESTORE_PIDS,
} autotunePhase_e;

#define AUTOTUNE_PHASE_MAX PHASE_SAVE_OR_RESTORE_PIDS
#define AUTOTUNE_PHASE_COUNT (AUTOTUNE_PHASE_MAX + 1)

#define FIRST_TUNE_PHASE PHASE_TUNE_ROLL

static pidProfile_t *pidProfile;
static pidProfile_t pidBackup;
static uint8_t pidController;
static bool rising;
static uint8_t cycleCount; // TODO can we replace this with an enum to improve readability.
static uint32_t timeoutAt;
static angle_index_t autoTuneAngleIndex;
static autotunePhase_e phase = PHASE_IDLE;
static autotunePhase_e nextPhase = FIRST_TUNE_PHASE;

static float targetAngle = 0;
static float targetAngleAtPeak;
static float secondPeakAngle, firstPeakAngle; // deci dgrees, 180 deg = 1800

bool isAutotuneIdle(void)
{
    return phase == PHASE_IDLE;
}

static void startNewCycle(void)
{
    rising = !rising;
    secondPeakAngle = firstPeakAngle = 0;
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

float autotune(angle_index_t angleIndex, rollAndPitchInclination_t *inclination, float errorAngle)
{
    if (!(phase == PHASE_TUNE_ROLL || phase == PHASE_TUNE_PITCH) || autoTuneAngleIndex != angleIndex) {
        return errorAngle;
    }

    debug[0] = 0;

#if 0
    debug[1] = inclination->rawAngles[angleIndex];
#endif

    float currentAngle;

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
//                if (currentangle-targetangle<(FPAUTOTUNEMAXOSCILLATION>>1)) {
//                    currentivalueshifted=lib_fp_multiply(currentivalueshifted,AUTOTUNEINCREASEMULTIPLIER);
//                } else {
//                    currentivalueshifted=lib_fp_multiply(currentivalueshifted,AUTOTUNEDECREASEMULTIPLIER);
//                    if (currentivalueshifted<AUTOTUNEMINIMUMIVALUE) currentivalueshifted=AUTOTUNEMINIMUMIVALUE;
//                }

                // go back to checking P and D
                cycleCount = 1;
//                usersettings.pid_igain[autotuneindex]=0;
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
                // decrease P
                //currentpvalueshifted=lib_fp_multiply(currentpvalueshifted, AUTOTUNEDECREASEMULTIPLIER); // TODO

#ifdef PREFER_HIGH_GAIN_SOLUTION
                } else {
                    // we don't have too much oscillation, so we can increase D"
#endif

                // increase D
                //currentdvalueshifted=lib_fp_multiply(currentdvalueshifted, AUTOTUNEINCREASEMULTIPLIER); // TODO
#ifdef PREFER_HIGH_GAIN_SOLUTION
                }
#endif
            } else {
                // undershot
                debug[0] = 2;

                if (oscillationAmplitude > AUTOTUNE_MAX_OSCILLATION) {
                    // we have too much oscillation, so we should lower D
                    //currentdvalueshifted=lib_fp_multiply(currentdvalueshifted, AUTOTUNEDECREASEMULTIPLIER); // TODO
                } else {
                    // we don't have too much oscillation, so we increase P
                    //currentpvalueshifted=lib_fp_multiply(currentpvalueshifted, AUTOTUNEINCREASEMULTIPLIER); // TODO
                }
            }

            //usersettings.pid_pgain[autotuneindex]=currentpvalueshifted>>AUTOTUNESHIFT; // TODO
            //usersettings.pid_dgain[autotuneindex]=currentdvalueshifted>>AUTOTUNESHIFT; // TODO

            // switch to the other direction and start a new cycle
            startNewCycle();

            if (++cycleCount == 3) {
                // switch to testing I value
                cycleCount = 0;

                //usersettings.pid_igain[autotuneindex]=currentivalueshifted>>AUTOTUNESHIFT; // TODO
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

    updateTargetAngle();

    // TODO
//    currentpvalueshifted=usersettings.pid_pgain[autotuneindex]<<AUTOTUNESHIFT;
//    currentivalueshifted=usersettings.pid_igain[autotuneindex]<<AUTOTUNESHIFT;
//    // divide by D multiplier to get our working value.  We'll multiply by D multiplier when we are done.
//    usersettings.pid_dgain[autotuneindex]=lib_fp_multiply(usersettings.pid_dgain[autotuneindex],FPONEOVERAUTOTUNE_D_MULTIPLIER);
//    currentdvalueshifted=usersettings.pid_dgain[autotuneindex]<<AUTOTUNESHIFT;
//
//    usersettings.pid_igain[autotuneindex]=0;

}

void autotuneEndPhase(void)
{
    // TODO
//    usersettings.pid_igain[autotuneindex]=currentivalueshifted>>AUTOTUNESHIFT;
//
//    // multiply by D multiplier.  The best D is usually a little higher than what the algroithm produces.
//    usersettings.pid_dgain[autotuneindex]=lib_fp_multiply(currentdvalueshifted,FPAUTOTUNE_D_MULTIPLIER)>>AUTOTUNESHIFT;
//
//    usersettings.pid_igain[YAWINDEX]=usersettings.pid_igain[ROLLINDEX];
//    usersettings.pid_dgain[YAWINDEX]=usersettings.pid_dgain[ROLLINDEX];
//    usersettings.pid_pgain[YAWINDEX]=lib_fp_multiply(usersettings.pid_pgain[ROLLINDEX],YAWGAINMULTIPLIER);

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

