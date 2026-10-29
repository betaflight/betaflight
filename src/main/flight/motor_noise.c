/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_MOTOR_NOISE

#include "build/debug.h"
#include "common/filter.h"
#include "common/maths.h"
#include "drivers/motor.h"
#include "flight/mixer.h"

#include "motor_noise.h"

// Band of the command that heats the windings as ripple current. A constant-amplitude sweep on a 2203.5 with a 5"
// prop (research/motor-thermal/RESULTS.md, 2026-10-27) is flat from ~20 to ~400 Hz and 0.52 at 1 kHz (winding
// inductance, command update rate). The prop follows the command below that: 0.73 at 11 Hz. The 20 Hz edge and the
// loss level (REF_K_CU) are set on three flights with a thermistor on a winding, which a wider band made worse.
#define MOTOR_NOISE_LOW_HZ    20.0f
#define MOTOR_NOISE_HIGH_HZ   2000.0f
#define MOTOR_NOISE_AVG_S     0.5f    // averaging time constant

static float fast[MAX_SUPPORTED_MOTORS], slow[MAX_SUPPORTED_MOTORS];   // band = fast - slow, two PT1 lowpasses
static float meanSq[MAX_SUPPORTED_MOTORS];         // EMA of the squared band
static bool seeded;                                 // the lowpasses start at the first sample: no start-up transient
static float kFast, kSlow, kAvg;
static float scaleToExternal;

void motorNoiseInit(float looptimeUs)
{
    const float dt = looptimeUs * 1e-6f;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        meanSq[i] = 0.0f;
    }
    seeded = false;
    kFast = pt1FilterGain(MOTOR_NOISE_HIGH_HZ, dt);
    kSlow = pt1FilterGain(MOTOR_NOISE_LOW_HZ, dt);
    kAvg = dt / MOTOR_NOISE_AVG_S;
    // internal motor units -> external 1000..2000 units, measured in the linear region
    scaleToExternal = 200.0f / (motorConvertFromExternal(1600) - motorConvertFromExternal(1400));
}

// Runs every PID loop. Kept out of the speed-optimised mixTable(), where it is inlined at several times this size.
NOINLINE void motorNoiseUpdate(const float *motorOutput, int motorCount)
{
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        const float x = motorOutput[i] * scaleToExternal;
        if (!seeded) {
            fast[i] = slow[i] = x;
        }
        fast[i] += kFast * (x - fast[i]);
        slow[i] += kSlow * (fast[i] - slow[i]);
        const float band = fast[i] - slow[i];
        meanSq[i] += kAvg * (band * band - meanSq[i]);
    }
    seeded = true;
    if (debugMode == DEBUG_MOTOR_NOISE) {
        for (int i = 0; i < 4; i++) {
            DEBUG_SET(DEBUG_MOTOR_NOISE, i, lrintf(10.0f * motorNoiseRms(i)));  //!< [index:0..3] Motor {1|2|3|4} Command Noise RMS [unit:0.1]
        }
    }
}

float motorNoiseMsq(int motor)
{
    return (motor >= 0 && motor < MAX_SUPPORTED_MOTORS) ? meanSq[motor] : 0.0f;
}

float motorNoiseRms(int motor)
{
    return sqrtf(motorNoiseMsq(motor));
}

#endif // USE_MOTOR_NOISE
