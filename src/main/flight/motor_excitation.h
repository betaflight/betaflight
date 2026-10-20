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

/*
 * Motor excitation: bench signal generator for motor, ESC and frame identification.
 *
 * Disarmed only. A host (Configurator, bench script) starts a session over MSP for a set of motors:
 * the base command is ramped in, then a zero-mean test signal is added to it at PID loop rate.
 * The session owns those motors until it ends; on any end (duration, host stop, keepalive timeout,
 * arming, critical battery) the motors in the mask are stopped, including their motor-test value.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef enum {
    MOTOR_EXCITATION_NONE = 0,  // base command only (baseline / cool-down at fixed rpm)
    MOTOR_EXCITATION_SINE,      // amplitude = peak, f0
    MOTOR_EXCITATION_WHITE,     // amplitude = sigma, gaussian lowpassed at f0, clipped at 3 sigma, seeded
    MOTOR_EXCITATION_CHIRP,     // amplitude = peak, exponential sweep f0 -> f1 over the duration
    MOTOR_EXCITATION_SQUARE,    // amplitude = peak, f0 (step response)
    MOTOR_EXCITATION_RECORD,    // no motor output: logging window with host markers (tap test); mask may be 0
    MOTOR_EXCITATION_WAVEFORM_COUNT
} motorExcitationWaveform_e;

typedef enum {
    MOTOR_EXCITATION_IDLE = 0,
    MOTOR_EXCITATION_RAMP,
    MOTOR_EXCITATION_RUN,
    MOTOR_EXCITATION_ENDED,
} motorExcitationState_e;

typedef enum {
    MOTOR_EXCITATION_END_NONE = 0,
    MOTOR_EXCITATION_END_DONE,      // duration elapsed
    MOTOR_EXCITATION_END_STOP,      // host stop command
    MOTOR_EXCITATION_END_TIMEOUT,   // no keepalive from the host
    MOTOR_EXCITATION_END_ARMED,
    MOTOR_EXCITATION_END_BATTERY,   // battery critical
} motorExcitationEnd_e;

#define MOTOR_EXCITATION_MAX_COMMAND      1800    // hard cap, external units (bench, no props)
#define MOTOR_EXCITATION_MIN_COMMAND      1020    // floor while running: keeps DShot out of the stop value
#define MOTOR_EXCITATION_MAX_AMPLITUDE     500
#define MOTOR_EXCITATION_MAX_DURATION_DS  6000    // 600 s
#define MOTOR_EXCITATION_MIN_TIMEOUT_MS    200
#define MOTOR_EXCITATION_MAX_TIMEOUT_MS   5000

typedef struct motorExcitationParams_s {
    uint16_t sessionId;
    uint8_t motorMask;          // bit i = motor i
    uint8_t waveform;           // motorExcitationWaveform_e
    uint16_t baseCommand;       // external units 1000..2000
    uint16_t amplitude;         // external units, see waveform
    uint16_t freq0DeciHz;
    uint16_t freq1DeciHz;       // chirp end frequency
    uint16_t durationDs;        // signal duration after the ramp, 0.1 s
    uint16_t rampMs;
    uint16_t maxCommand;        // external units, clamp for base + signal
    uint16_t timeoutMs;         // keepalive timeout
    uint32_t seed;              // white noise generator seed
    uint8_t flags;              // MOTOR_EXCITATION_FLAG_*
    uint8_t invertMask;         // motors that get the negated signal (e.g. diagonals in antiphase)
} motorExcitationParams_t;

#define MOTOR_EXCITATION_FLAG_INDEPENDENT  0x01   // WHITE: an independent generator per motor (MIMO identification)

typedef struct motorExcitationStatus_s {
    motorExcitationState_e state;
    motorExcitationEnd_e end;
    uint32_t elapsedMs;         // since the start of the ramp
    float frequencyHz;          // instantaneous signal frequency (chirp), else f0
    float sample;               // last signal sample, external units
    float rms;                  // measured RMS of the applied signal after clipping, external units
    uint32_t loops;             // signal loops in RUN
    uint32_t clippedLoops;      // loops where base + signal hit a clamp
    uint16_t marker;            // last marker set by the host (RECORD sessions)
} motorExcitationStatus_t;

// Host side (MSP). Start fails (false) when armed, the parameters are invalid or no motor is selected.
bool motorExcitationStart(const motorExcitationParams_t *params);
void motorExcitationKeepalive(uint16_t sessionId);
void motorExcitationMark(uint16_t sessionId, uint16_t marker);   // also a keepalive
void motorExcitationStop(void);

bool motorExcitationIsActive(void);
const motorExcitationParams_t *motorExcitationGetParams(void);
const motorExcitationStatus_t *motorExcitationGetStatus(void);

// Tap capture for the frame wizard: the features of each tap on the frame (where it was), and its decay.
typedef struct motorExcitationTap_s {
    uint16_t seq;
    float rot[3];          // gyro rate integrated over the first 4 ms, deg
    float dv[3];           // acc integrated over the first 30 ms, g*s (velocity change / g)
    float peak;            // peak |rate|, deg/s
    float dv3[3];          // acc integrated over the first 3 ms (the push, before the support reacts), g*s
    float accPeakZ;        // signed peak of the vertical acc change in the first 3 ms, g
    uint8_t clipped;           // MOTOR_EXCITATION_TAP_CLIPPED_*: a sensor near full scale during the capture
} motorExcitationTap_t;
void motorExcitationTapCapture(float dt);   // run the tap detector for one loop (frame wizard)
uint8_t motorExcitationTapClipped(void);    // the tap being captured hit full scale so far (MOTOR_EXCITATION_TAP_CLIPPED_*)
#define MOTOR_EXCITATION_TAP_CLIPPED_ACC  (1 << 0)
#define MOTOR_EXCITATION_TAP_CLIPPED_GYRO (1 << 1)
void motorExcitationSetClipFraction(float fraction);   // full-scale share that marks a tap clipped (default 0.9)

// Mixer, at the start of every PID loop. Writes the motor test (disarmed) values of the selected motors;
// ends the session when armed, on timeout or critical battery.
void motorExcitationUpdate(float *motorDisarmed, int motorCount, float disarmOutput, timeUs_t currentTimeUs);
