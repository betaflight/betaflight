/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MOTOR_EXCITATION

#include "build/debug.h"
#include "common/chirp.h"
#include "common/maths.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "sensors/battery.h"
#ifdef USE_MOTOR_NOISE
#include "flight/motor_noise.h"
#endif

#include "motor_excitation.h"

#define RMS_AVERAGING_S        0.5f
#define DEFAULT_TIMEOUT_MS     1000
#define DEFAULT_SEED           0x9E3779B9u

static motorExcitationParams_t params;
static motorExcitationStatus_t status;

static timeUs_t startUs;
static timeUs_t runStartUs;
static timeUs_t lastKeepaliveUs;
static uint32_t durationUs;
static float rampFrom;              // external units at the start of the ramp
static float rampTo;
static float currentBase;           // external units
static uint8_t stopMask;            // motors to stop on the next mixer pass
static bool restartPending;         // params changed by the host, generator state reset in the mixer pass

// generator state
static float phase;
static float lowpass;
static float lowpassGain;
static float lowpassNorm;
static uint32_t rngState;
static uint32_t motorRng[MAX_SUPPORTED_MOTORS];     // independent white noise per motor
static float motorLowpass[MAX_SUPPORTED_MOTORS];
static chirp_t chirp;
static float rmsSq;

// external -> internal motor units (linear above the stop value, measured in the linear region)
static float internalOffset;
static float internalScale;

static float toInternal(float external)
{
    return internalOffset + internalScale * external;
}

static float rngNext(uint32_t *state)
{
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    return (float)(*state & 0xFFFFFF) / (float)0x800000 - 1.0f;   // -1..1
}

static float rngUniform(void)
{
    return rngNext(&rngState);
}

// unit-sigma lowpassed gaussian for one motor's own generator
static float motorWhite(int i)
{
    const float g = rngNext(&motorRng[i]) + rngNext(&motorRng[i]) + rngNext(&motorRng[i]);
    motorLowpass[i] += lowpassGain * (g - motorLowpass[i]);
    return constrainf(motorLowpass[i] * lowpassNorm, -3.0f, 3.0f);
}

static bool paramsValid(const motorExcitationParams_t *p, float loopHz)
{
    const float maxHz = 0.45f * loopHz;
    const float f0 = p->freq0DeciHz * 0.1f;
    const float f1 = p->freq1DeciHz * 0.1f;

    if (p->waveform >= MOTOR_EXCITATION_WAVEFORM_COUNT || (p->motorMask >> getMotorCount()) != 0) {
        return false;
    }
    if (p->waveform == MOTOR_EXCITATION_RECORD) {   // motors stay stopped: only the duration matters
        return p->durationDs > 0 && p->durationDs <= MOTOR_EXCITATION_MAX_DURATION_DS;
    }
    if (p->motorMask == 0) {
        return false;
    }
    if (p->baseCommand < MOTOR_EXCITATION_MIN_COMMAND || p->maxCommand < p->baseCommand
        || p->maxCommand > MOTOR_EXCITATION_MAX_COMMAND) {
        return false;
    }
    if (p->amplitude > MOTOR_EXCITATION_MAX_AMPLITUDE) {
        return false;
    }
    if (p->durationDs == 0 || p->durationDs > MOTOR_EXCITATION_MAX_DURATION_DS) {
        return false;
    }
    switch (p->waveform) {
    case MOTOR_EXCITATION_SINE:
    case MOTOR_EXCITATION_WHITE:
    case MOTOR_EXCITATION_SQUARE:
        return f0 > 0.0f && f0 <= maxHz;
    case MOTOR_EXCITATION_CHIRP:
        return f0 > 0.0f && f1 > 0.0f && f0 <= maxHz && f1 <= maxHz;
    default:
        return true;
    }
}

bool motorExcitationStart(const motorExcitationParams_t *p)
{
    const float loopHz = 1e6f / targetPidLooptime;
    if (ARMING_FLAG(ARMED) || !paramsValid(p, loopHz)) {
        return false;
    }

    const bool wasActive = motorExcitationIsActive();
    // a running session hands over without stopping: ramp from the current base, stop dropped motors
    if (wasActive) {
        stopMask |= params.motorMask & ~p->motorMask;
        rampFrom = currentBase;
    } else {
        rampFrom = MOTOR_EXCITATION_MIN_COMMAND;
    }

    params = *p;
    if (params.timeoutMs == 0) {
        params.timeoutMs = DEFAULT_TIMEOUT_MS;
    }
    params.timeoutMs = constrain(params.timeoutMs, MOTOR_EXCITATION_MIN_TIMEOUT_MS, MOTOR_EXCITATION_MAX_TIMEOUT_MS);

    if (params.waveform == MOTOR_EXCITATION_RECORD) {
        params.rampMs = 0;
    }
    rampTo = params.baseCommand;
    currentBase = rampFrom;
    durationUs = params.durationDs * 100000u;

    internalScale = (motorConvertFromExternal(1600) - motorConvertFromExternal(1400)) / 200.0f;
    internalOffset = motorConvertFromExternal(1600) - internalScale * 1600.0f;

    const float dt = targetPidLooptime * 1e-6f;
    const float f0 = params.freq0DeciHz * 0.1f;
    phase = 0.0f;
    lowpass = 0.0f;
    rmsSq = 0.0f;
    rngState = params.seed ? params.seed : DEFAULT_SEED;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorRng[i] = rngState ^ (DEFAULT_SEED * (uint32_t)(i + 1));
        if (motorRng[i] == 0) {
            motorRng[i] = DEFAULT_SEED;
        }
        motorLowpass[i] = 0.0f;
    }
    if (params.waveform == MOTOR_EXCITATION_WHITE) {
        lowpassGain = 1.0f - expf(-2.0f * M_PIf * f0 * dt);
        lowpassNorm = 1.0f / sqrtf(lowpassGain / (2.0f - lowpassGain));   // unit sigma after the lowpass
    }
    if (params.waveform == MOTOR_EXCITATION_CHIRP) {
        chirpInit(&chirp, f0, params.freq1DeciHz * 0.1f, params.durationDs * 0.1f, targetPidLooptime);
    }

    memset(&status, 0, sizeof(status));
    status.frequencyHz = f0;
    status.state = MOTOR_EXCITATION_RAMP;
    startUs = micros();
    lastKeepaliveUs = startUs;
    restartPending = true;
    return true;
}

static void endSession(motorExcitationEnd_e reason)
{
    if (status.state == MOTOR_EXCITATION_RAMP || status.state == MOTOR_EXCITATION_RUN) {
        status.state = MOTOR_EXCITATION_ENDED;
        status.end = reason;
        stopMask |= params.motorMask;
        DEBUG_SET(DEBUG_MOTOR_EXCITATION, 0, 0);
        DEBUG_SET(DEBUG_MOTOR_EXCITATION, 3, status.state * 16 + status.end);
    }
}

void motorExcitationKeepalive(uint16_t sessionId)
{
    if (motorExcitationIsActive() && sessionId == params.sessionId) {
        lastKeepaliveUs = micros();
    }
}

void motorExcitationMark(uint16_t sessionId, uint16_t marker)
{
    if (motorExcitationIsActive() && sessionId == params.sessionId) {
        lastKeepaliveUs = micros();
        status.marker = marker;
    }
}

void motorExcitationStop(void)
{
    endSession(MOTOR_EXCITATION_END_STOP);
}

bool motorExcitationIsActive(void)
{
    return status.state == MOTOR_EXCITATION_RAMP || status.state == MOTOR_EXCITATION_RUN;
}

const motorExcitationParams_t *motorExcitationGetParams(void)
{
    return &params;
}

const motorExcitationStatus_t *motorExcitationGetStatus(void)
{
    return &status;
}

// one sample per loop, unit amplitude (white: unit sigma)
static float nextSample(float dt)
{
    switch (params.waveform) {
    case MOTOR_EXCITATION_SINE:
    case MOTOR_EXCITATION_SQUARE: {
        phase += 2.0f * M_PIf * status.frequencyHz * dt;
        if (phase >= 2.0f * M_PIf) {
            phase -= 2.0f * M_PIf;
        }
        if (params.waveform == MOTOR_EXCITATION_SQUARE) {
            return phase < M_PIf ? 1.0f : -1.0f;
        }
        return sin_approx(phase);
    }
    case MOTOR_EXCITATION_WHITE: {
        const float g = rngUniform() + rngUniform() + rngUniform();    // ~gaussian, sigma 1
        lowpass += lowpassGain * (g - lowpass);
        return constrainf(lowpass * lowpassNorm, -3.0f, 3.0f);
    }
    case MOTOR_EXCITATION_CHIRP:
        if (!chirpUpdate(&chirp)) {
            endSession(MOTOR_EXCITATION_END_DONE);
            return 0.0f;
        }
        status.frequencyHz = chirp.fchirp;
        return sin_approx(chirp.sinarg);
    default:
        return 0.0f;
    }
}

static void applyStop(float *motorDisarmed, int motorCount, float disarmOutput)
{
    for (int i = 0; i < motorCount; i++) {
        if (stopMask & (1u << i)) {
            motorDisarmed[i] = disarmOutput;
        }
    }
    stopMask = 0;
}

void motorExcitationUpdate(float *motorDisarmed, int motorCount, float disarmOutput, timeUs_t currentTimeUs)
{
    if (status.state == MOTOR_EXCITATION_IDLE || (status.state == MOTOR_EXCITATION_ENDED && !stopMask)) {
        return;
    }

    const bool armed = ARMING_FLAG(ARMED);
    if (motorExcitationIsActive()) {
        if (armed) {
            endSession(MOTOR_EXCITATION_END_ARMED);
        } else if (cmpTimeUs(currentTimeUs, lastKeepaliveUs) > (timeDelta_t)params.timeoutMs * 1000) {
            endSession(MOTOR_EXCITATION_END_TIMEOUT);
        } else if (getBatteryState() == BATTERY_CRITICAL) {
            endSession(MOTOR_EXCITATION_END_BATTERY);
        }
    }
    if (stopMask) {
        applyStop(motorDisarmed, motorCount, disarmOutput);
    }
    if (!motorExcitationIsActive()) {
        return;
    }

    if (restartPending) {
        restartPending = false;
        startUs = currentTimeUs;
    }
    const float dt = pidGetDT();
    const bool independent = params.flags & MOTOR_EXCITATION_FLAG_INDEPENDENT;
    const timeDelta_t elapsedUs = cmpTimeUs(currentTimeUs, startUs);
    status.elapsedMs = elapsedUs / 1000;

    float sample = 0.0f;
    if (status.state == MOTOR_EXCITATION_RAMP) {
        const timeDelta_t rampUs = params.rampMs * 1000;
        if (elapsedUs >= rampUs) {
            status.state = MOTOR_EXCITATION_RUN;
            runStartUs = currentTimeUs;
            currentBase = rampTo;
        } else {
            currentBase = rampFrom + (rampTo - rampFrom) * elapsedUs / (float)rampUs;
        }
    }
    if (status.state == MOTOR_EXCITATION_RUN) {
        if (params.waveform != MOTOR_EXCITATION_CHIRP && (uint32_t)cmpTimeUs(currentTimeUs, runStartUs) >= durationUs) {
            endSession(MOTOR_EXCITATION_END_DONE);
            applyStop(motorDisarmed, motorCount, disarmOutput);
            return;
        }
        if (params.waveform == MOTOR_EXCITATION_RECORD) {   // logging window only, motors untouched
            status.loops++;
            DEBUG_SET(DEBUG_MOTOR_EXCITATION, 0, 0);
            DEBUG_SET(DEBUG_MOTOR_EXCITATION, 2, status.marker);                  //!< Host marker (RECORD)
            DEBUG_SET(DEBUG_MOTOR_EXCITATION, 3, status.state * 16 + status.end);
            DEBUG_SET(DEBUG_MOTOR_EXCITATION, 4, (int16_t)params.sessionId);
            return;
        }
        if (!(independent && params.waveform == MOTOR_EXCITATION_WHITE)) {
            sample = params.amplitude * nextSample(dt);
        }
        if (!motorExcitationIsActive()) {   // chirp finished
            applyStop(motorDisarmed, motorCount, disarmOutput);
            return;
        }
    }

    // per motor: shared sample (or the motor's own generator), sign from invertMask, clamp, write the motor test value.
    // The disarmed (motor test) value carries the signal: the mixer copies it to the output on every path,
    // and blackbox MOTOR_TEST mode sees the motor running. Status and debug follow the first motor of the mask.
    const bool running = status.state == MOTOR_EXCITATION_RUN;
    int firstMotor = -1;
    float out = currentBase;
    for (int i = 0; i < motorCount; i++) {
        if (!(params.motorMask & (1u << i))) {
            continue;
        }
        float s = sample;
        if (running && independent && params.waveform == MOTOR_EXCITATION_WHITE) {
            s = params.amplitude * motorWhite(i);
        }
        if (params.invertMask & (1u << i)) {
            s = -s;
        }
        const float raw = currentBase + s;
        const float o = constrainf(raw, MOTOR_EXCITATION_MIN_COMMAND, params.maxCommand);
        motorDisarmed[i] = toInternal(o);
        if (firstMotor < 0) {
            firstMotor = i;
            out = o;
            sample = s;
            if (running) {
                status.loops++;
                if (o != raw) {
                    status.clippedLoops++;
                }
                const float applied = o - currentBase;
                rmsSq += (dt / RMS_AVERAGING_S) * (applied * applied - rmsSq);
            }
        }
    }
    status.sample = sample;
    status.rms = sqrtf(rmsSq);

    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 0, lrintf(sample));                       //!< Signal sample [unit:command]
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 1, lrintf(out));                          //!< Applied command, base + signal after clamp [unit:command]
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 2, lrintf(MIN(status.frequencyHz * 10.0f, 32767.0f)));  //!< Signal frequency [unit:0.1 Hz]
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 3, status.state * 16 + status.end);       //!< State * 16 + end reason
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 4, (int16_t)params.sessionId);            //!< Session id
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 5, lrintf(10.0f * status.rms));           //!< Applied signal RMS [unit:0.1 command]
#ifdef USE_MOTOR_NOISE
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 6, lrintf(10.0f * motorNoiseRms(firstMotor, 0)));  //!< Measured noise index >80 Hz [unit:0.1]
    DEBUG_SET(DEBUG_MOTOR_EXCITATION, 7, lrintf(10.0f * motorNoiseRms(firstMotor, 1)));  //!< Measured noise index >300 Hz [unit:0.1]
#else
    UNUSED(firstMotor);
#endif
}

#endif // USE_MOTOR_EXCITATION
