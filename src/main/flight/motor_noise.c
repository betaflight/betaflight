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

#define MOTOR_NOISE_HP0_HZ   80.0f    // below this the prop/bell still follows the command (not ripple)
#define MOTOR_NOISE_HP1_HZ  300.0f    // winding inductance starts to filter the ripple above here
#define MOTOR_NOISE_AVG_S     0.5f    // RMS averaging time constant

typedef struct {
    pt1Filter_t lp[MOTOR_NOISE_BANDS];  // highpass = input - lowpass
    float msq[MOTOR_NOISE_BANDS];       // EMA of squared highpassed value
    bool seeded;                        // lowpass state follows the first sample, no start-up transient
} motorNoiseState_t;

static motorNoiseState_t state[MAX_SUPPORTED_MOTORS];
static float kAvg;
static float scaleToExternal;

void motorNoiseInit(float looptimeUs)
{
    const float dt = looptimeUs * 1e-6f;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        pt1FilterInit(&state[i].lp[0], pt1FilterGain(MOTOR_NOISE_HP0_HZ, dt));
        pt1FilterInit(&state[i].lp[1], pt1FilterGain(MOTOR_NOISE_HP1_HZ, dt));
        state[i].msq[0] = state[i].msq[1] = 0.0f;
        state[i].seeded = false;
    }
    kAvg = dt / MOTOR_NOISE_AVG_S;
    // internal motor units -> external 1000..2000 units, measured in the linear region
    scaleToExternal = 200.0f / (motorConvertFromExternal(1600) - motorConvertFromExternal(1400));
}

void motorNoiseUpdate(const float *motorOutput, int motorCount)
{
    for (int i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        const float x = motorOutput[i] * scaleToExternal;
        if (!state[i].seeded) {
            for (int b = 0; b < MOTOR_NOISE_BANDS; b++) {
                state[i].lp[b].state = x;
            }
            state[i].seeded = true;
        }
        for (int b = 0; b < MOTOR_NOISE_BANDS; b++) {
            const float hp = x - pt1FilterApply(&state[i].lp[b], x);
            state[i].msq[b] += kAvg * (hp * hp - state[i].msq[b]);
        }
    }
    DEBUG_SET(DEBUG_MOTOR_NOISE, 0, lrintf(10.0f * motorNoiseRms(0, 0)));  //!< Motor 1 Command Noise RMS [unit:0.1]
    DEBUG_SET(DEBUG_MOTOR_NOISE, 1, lrintf(10.0f * motorNoiseRms(1, 0)));  //!< Motor 2 Command Noise RMS [unit:0.1]
    DEBUG_SET(DEBUG_MOTOR_NOISE, 2, lrintf(10.0f * motorNoiseRms(2, 0)));  //!< Motor 3 Command Noise RMS [unit:0.1]
    DEBUG_SET(DEBUG_MOTOR_NOISE, 3, lrintf(10.0f * motorNoiseRms(3, 0)));  //!< Motor 4 Command Noise RMS [unit:0.1]
}

float motorNoiseRms(int motor, int band)
{
    if (motor < 0 || motor >= MAX_SUPPORTED_MOTORS || band < 0 || band >= MOTOR_NOISE_BANDS) {
        return 0.0f;
    }
    return sqrtf(state[motor].msq[band]);
}

#endif // USE_MOTOR_NOISE
