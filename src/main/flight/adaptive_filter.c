/*
 * Adaptive D-term LPF1 cutoff controller — v1 minimal implementation.
 *
 * Strategy:
 *   - Estimate D-term high-frequency noise energy using a simple first-difference
 *     (delta between consecutive D-term samples), averaged with a leaky integrator.
 *   - Every `update_ms` milliseconds, compare noise energy against two thresholds:
 *       noise > high_threshold  → lower cutoff (more filtering)
 *       noise < low_threshold   → raise cutoff (less filtering)
 *       otherwise               → hold
 *   - Cutoff is clamped to [min_hz, max_hz] and changes at most step_hz per update.
 *   - Learning is frozen under any of the conditions in adaptiveFreezeReason_e.
 *
 * All axes share a single cutoff value (v1 simplification).
 * Default OFF. When enabled, v1 only supports static PT1 D-term LPF1.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "config/config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/gyro.h"

#include "adaptive_filter.h"

// ── tuneable constants ────────────────────────────────────────────────────────
// Noise energy thresholds (arbitrary units — D-term is in deg/s·Kd scale).
// These are intentionally conservative for v1; tune via blackbox.
#define ADAPTIVE_NOISE_HIGH_THRESHOLD   2500.0f   // above → lower cutoff
#define ADAPTIVE_NOISE_LOW_THRESHOLD     800.0f   // below → raise cutoff

// Leaky integrator time constant for noise energy estimate (seconds)
#define ADAPTIVE_NOISE_TAU              0.05f

// Freeze thresholds
#define ADAPTIVE_FREEZE_STICK_THRESHOLD     200.0f    // |setpointRate| > this → freeze
#define ADAPTIVE_FREEZE_GYRO_ERROR_THRESHOLD 150.0f   // |gyroError|    > this → freeze
#define ADAPTIVE_FREEZE_THROTTLE_THRESHOLD  0.05f     // throttle < this → freeze
#define ADAPTIVE_FREEZE_MOTOR_SAT_THRESHOLD 0.95f     // motorMixRange  > this → freeze

// ── module state ─────────────────────────────────────────────────────────────
static bool     adEnabled;
static bool     adConfigValid;
static float    adCurrentCutoff;
static float    adNoiseEnergy;          // leaky-integrated noise estimate
static float    adNoiseDiffSqAccum;     // per-loop accumulator for squared D-term differences
static int      adNoiseDiffSqCount;     // number of axes accumulated this loop
static float    adPrevDterm[XYZ_AXIS_COUNT];
static bool     adPrevDtermValid[XYZ_AXIS_COUNT];

static timeUs_t adLastUpdateUs;
static timeUs_t adArmedAtUs;

static adaptiveState_e       adState;
static uint8_t               adFreezeReason;

// Config snapshot (copied from pidProfile at init)
static uint16_t adMinHz;
static uint16_t adMaxHz;
static uint16_t adStartHz;
static uint16_t adUpdateMs;
static uint8_t  adStepHz;
static uint8_t  adLearnDelaySec;

// ── helpers ───────────────────────────────────────────────────────────────────

static void applyNewCutoff(float cutoff)
{
    // Update the running D-term LPF1 filter coefficients without full re-init.
    // Only PT1 static LPF1 is accepted by adaptiveFilterInit(), so updating the
    // PT1 coefficient here cannot corrupt another filter type's union state.
    if (!adConfigValid) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter,
                              pt1FilterGain(cutoff, pidRuntime.dT));
    }
}

static bool isAdaptiveConfigSupported(const pidProfile_t *profile)
{
    if (profile->adaptive_dterm_lpf_min_hz == 0 || profile->adaptive_dterm_lpf_max_hz == 0 ||
        profile->adaptive_dterm_lpf_min_hz >= profile->adaptive_dterm_lpf_max_hz ||
        profile->adaptive_dterm_lpf_start_hz < profile->adaptive_dterm_lpf_min_hz ||
        profile->adaptive_dterm_lpf_start_hz > profile->adaptive_dterm_lpf_max_hz ||
        profile->adaptive_dterm_lpf_update_ms == 0 ||
        profile->adaptive_dterm_lpf_step_hz == 0) {
        return false;
    }

    if (profile->dterm_lpf1_type != FILTER_PT1 || profile->dterm_lpf1_static_hz == 0) {
        return false;
    }

#ifdef USE_DYN_LPF
    if (profile->dterm_lpf1_dyn_min_hz > 0) {
        return false;
    }
#endif

    return true;
}

// ── public API ────────────────────────────────────────────────────────────────

void adaptiveFilterInit(void)
{
    const pidProfile_t *profile = currentPidProfile;

    adConfigValid    = false;
    adCurrentCutoff  = 0.0f;
    adNoiseEnergy    = 0.0f;
    adNoiseDiffSqAccum = 0.0f;
    adNoiseDiffSqCount = 0;
    adLastUpdateUs   = 0;
    adArmedAtUs      = 0;
    adState          = ADAPTIVE_STATE_HOLD;
    adFreezeReason   = ADAPTIVE_FREEZE_NONE;

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        adPrevDterm[i] = 0.0f;
        adPrevDtermValid[i] = false;
    }

    adEnabled = profile->adaptive_dterm_lpf;
    if (!adEnabled) {
        return;
    }

    adMinHz        = profile->adaptive_dterm_lpf_min_hz;
    adMaxHz        = profile->adaptive_dterm_lpf_max_hz;
    adStartHz      = profile->adaptive_dterm_lpf_start_hz;
    adUpdateMs     = profile->adaptive_dterm_lpf_update_ms;
    adStepHz       = profile->adaptive_dterm_lpf_step_hz;
    adLearnDelaySec= profile->adaptive_dterm_lpf_learn_delay_s;

    adConfigValid = isAdaptiveConfigSupported(profile);
    if (!adConfigValid) {
        adState = ADAPTIVE_STATE_CONFIG;
        return;
    }

    adCurrentCutoff   = adStartHz;
    adState           = ADAPTIVE_STATE_HOLD;

    applyNewCutoff(adCurrentCutoff);
}

void adaptiveFilterPushDterm(int axis, float dterm)
{
    if (!adEnabled || !adConfigValid) {
        return;
    }

    if (!adPrevDtermValid[axis]) {
        adPrevDterm[axis] = dterm;
        adPrevDtermValid[axis] = true;
        return;
    }

    if (adFreezeReason != ADAPTIVE_FREEZE_NONE) {
        adPrevDterm[axis] = dterm;
        return;
    }

    // First-difference as high-frequency proxy: |d(D)/dt|
    const float diff = dterm - adPrevDterm[axis];
    adPrevDterm[axis] = dterm;

    // Accumulate squared difference; IIR update happens once per loop in adaptiveFilterUpdate()
    adNoiseDiffSqAccum += diff * diff;
    adNoiseDiffSqCount++;
}

void adaptiveFilterUpdate(float throttle, float setpointRate, float gyroError, float motorMixRange)
{
    if (!adEnabled) {
        adNoiseDiffSqAccum = 0.0f;
        adNoiseDiffSqCount = 0;
        return;
    }

    if (!adConfigValid) {
        adNoiseDiffSqAccum = 0.0f;
        adNoiseDiffSqCount = 0;
        DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 0, 0);
        DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 1, 0);
        DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 2, 0);
        DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 3, ADAPTIVE_STATE_CONFIG);
        return;
    }

    // Single IIR update per PID loop using averaged squared differences from all axes
    if (adNoiseDiffSqCount > 0) {
        const float avgDiffSq = adNoiseDiffSqAccum / adNoiseDiffSqCount;
        const float alpha = pidRuntime.dT / (ADAPTIVE_NOISE_TAU + pidRuntime.dT);
        adNoiseEnergy += alpha * (avgDiffSq - adNoiseEnergy);
        adNoiseDiffSqAccum = 0.0f;
        adNoiseDiffSqCount = 0;
    }

    const timeUs_t now = micros();

    // ── freeze condition evaluation ──────────────────────────────────────────
    adFreezeReason = ADAPTIVE_FREEZE_NONE;

    if (!ARMING_FLAG(ARMED)) {
        adFreezeReason |= ADAPTIVE_FREEZE_DISARMED;
        if (adArmedAtUs != 0) {
            adArmedAtUs = 0;
            adCurrentCutoff = adStartHz;
            adNoiseEnergy = 0.0f;
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                adPrevDterm[i] = 0.0f;
                adPrevDtermValid[i] = false;
            }
            applyNewCutoff(adStartHz);
        }
    } else {
        if (adArmedAtUs == 0) {
            adArmedAtUs = now;
        }
        if (cmpTimeUs(now, adArmedAtUs) < (timeDelta_t)((uint32_t)adLearnDelaySec * 1000000U)) {
            adFreezeReason |= ADAPTIVE_FREEZE_LEARN_DELAY;
        }
    }

    if (fabsf(setpointRate) > ADAPTIVE_FREEZE_STICK_THRESHOLD) {
        adFreezeReason |= ADAPTIVE_FREEZE_BIG_STICK;
    }
    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | GPS_RESCUE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE | BRAKING_MODE)) {
        adFreezeReason |= ADAPTIVE_FREEZE_FLIGHT_MODE;
    }
    if (fabsf(gyroError) > ADAPTIVE_FREEZE_GYRO_ERROR_THRESHOLD) {
        adFreezeReason |= ADAPTIVE_FREEZE_GYRO_ERROR;
    }
    if (throttle < ADAPTIVE_FREEZE_THROTTLE_THRESHOLD) {
        adFreezeReason |= ADAPTIVE_FREEZE_LOW_THROTTLE;
    }
    if (motorMixRange > ADAPTIVE_FREEZE_MOTOR_SAT_THRESHOLD) {
        adFreezeReason |= ADAPTIVE_FREEZE_MOTOR_SAT;
    }

    const bool frozen = (adFreezeReason != ADAPTIVE_FREEZE_NONE);

    // ── periodic cutoff update ───────────────────────────────────────────────
    if (adLastUpdateUs == 0) {
        adLastUpdateUs = now;
    }

    const uint32_t elapsedMs = cmpTimeUs(now, adLastUpdateUs) / 1000;
    if (elapsedMs < adUpdateMs) {
        adState = frozen ? ADAPTIVE_STATE_FROZEN : adState;
        return;
    }
    adLastUpdateUs = now;

    if (frozen) {
        adState = ADAPTIVE_STATE_FROZEN;
        // Do NOT update cutoff — hold current value
    } else if (adNoiseEnergy > ADAPTIVE_NOISE_HIGH_THRESHOLD) {
        // Too noisy → lower cutoff (more filtering)
        adCurrentCutoff -= adStepHz;
        adCurrentCutoff  = MAX(adCurrentCutoff, (float)adMinHz);
        adState = ADAPTIVE_STATE_LOWER;
        applyNewCutoff(adCurrentCutoff);
    } else if (adNoiseEnergy < ADAPTIVE_NOISE_LOW_THRESHOLD) {
        // Quiet → raise cutoff (less filtering, less delay)
        adCurrentCutoff += adStepHz;
        adCurrentCutoff  = MIN(adCurrentCutoff, (float)adMaxHz);
        adState = ADAPTIVE_STATE_RAISE;
        applyNewCutoff(adCurrentCutoff);
    } else {
        adState = ADAPTIVE_STATE_HOLD;
    }

    // ── debug output ─────────────────────────────────────────────────────────
    // [0] current adaptive cutoff (Hz * 10 for resolution)
    // [1] noise energy (scaled)
    // [2] learning allowed (1) / frozen (0)
    // [3] state + freeze reason packed: high byte = freeze reason, low byte = state
    DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 0, lrintf(adCurrentCutoff * 10.0f));
    DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 1, constrain(lrintf(adNoiseEnergy), 0, INT16_MAX));
    DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 2, frozen ? 0 : 1);
    DEBUG_SET(DEBUG_ADAPTIVE_DTERM_LPF, 3, (adFreezeReason << 8) | adState);
}

float adaptiveFilterGetCutoff(void)      { return adCurrentCutoff; }
bool  adaptiveFilterIsFrozen(void)       { return adFreezeReason != ADAPTIVE_FREEZE_NONE; }
uint8_t adaptiveFilterGetFreezeReason(void) { return adFreezeReason; }
float adaptiveFilterGetNoiseLevel(void)  { return adNoiseEnergy; }
adaptiveState_e adaptiveFilterGetState(void) { return adState; }
