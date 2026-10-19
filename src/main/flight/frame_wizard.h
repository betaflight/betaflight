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
 * Frame test wizard (bench): everything on the flight controller, no blackbox.
 *
 * Phase 1, taps: the craft is held by the battery, the user taps every arm and the frame, any order, from below
 * (from above is accepted on a clear push down). Each tap is labelled from its impulse vectors (arm 1-4 by the
 * rotation direction against the mixer, frame for the centre and between arms) and its free decay spectrum is
 * computed on board; a point is done after 5 taps with at least one peak confirmed in 3 of them.
 * Phase 2, motors: all motors at once, each with its own exponential chirp shifted in frequency, two base speeds;
 * every motor's path is recovered by synchronous detection against its own chirp; peaks present at both speeds
 * are confirmed. DShot beacons mark progress: one motor when its arm is done, all for the frame, long before the
 * motors start, three quick at the end.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/streambuf.h"
#include "common/time.h"

#define FRAME_WIZARD_REGIONS        6      // arm 1-4, frame, USB cable
#define FRAME_WIZARD_REGION_FRAME   4
#define FRAME_WIZARD_REGION_CABLE   5      // the USB cable rings too: its modes are marked in the results
#define FRAME_WIZARD_PEAK_CABLE     (1 << 7)   // motor peak axis byte / tap peak flags: matches a cable mode
#define FRAME_WIZARD_FLAG_SKIP_TAPS (1 << 0)   // bench: straight to the motors
#define FRAME_WIZARD_FLAG_TAPS_ONLY (1 << 3)   // the frame and arms by taps only: done after the taps, no motors
#define FRAME_WIZARD_FLAG_NO_SPIN_CHECK (1 << 4)   // skip the spin direction check after the first sweep

#define FRAME_WIZARD_RESPONSE_SPIN  2          // MSP2_FRAME_WIZARD_RESPONSE base index: the spin check page
#define FRAME_WIZARD_RESULT_RPM            (1 << 0)   // DShot telemetry: rotation lines masked
#define FRAME_WIZARD_RESULT_BASES_BY_RPM   (1 << 1)   // base speeds found by rotation
#define FRAME_WIZARD_RESULT_SPEED_LIMITED  (1 << 2)   // a rotation target was not reached within the command limit
#define FRAME_WIZARD_RESULT_NO_CABLE       (1 << 3)   // not on USB: the cable point was skipped

typedef enum {
    FRAME_WIZARD_END_NONE = 0,
    FRAME_WIZARD_END_DONE,
    FRAME_WIZARD_END_USER,          // abort from the host
    FRAME_WIZARD_END_KEEPALIVE,     // no status poll for KEEPALIVE_TIMEOUT_MS
    FRAME_WIZARD_END_ARMED,
    FRAME_WIZARD_END_BATTERY,
    FRAME_WIZARD_END_CABLE,         // started on USB, the cable came out
    FRAME_WIZARD_END_FREEFALL,      // dropped
    FRAME_WIZARD_END_TILT,          // turned over from the start attitude
    FRAME_WIZARD_END_ROTATION,      // spun or swung hard
    FRAME_WIZARD_END_IMPACT,        // hit something
} frameWizardEnd_e;
#define FRAME_WIZARD_SWEEP_BINS     128

typedef struct frameWizardParams_s {
    uint16_t base1;         // first base command (without DShot telemetry)
    uint16_t base2;         // second base command (without DShot telemetry)
    uint16_t amplitude;     // chirp peak, command units
    uint8_t sweepS;         // seconds per sweep
    uint16_t maxCommand;
    uint8_t flags;              // FRAME_WIZARD_FLAG_*
    uint16_t rot1Hz;            // rotation targets for the two base speeds, 0 = default
    uint16_t rot2Hz;
    uint8_t clipPct;            // full-scale share that marks a tap clipped, % (0 = 90)
} frameWizardParams_t;

struct motorExcitationTap_s;

bool frameWizardStart(const frameWizardParams_t *params);
void frameWizardAbort(void);
void frameWizardKeepalive(void);
bool frameWizardIsActive(void);

// from the tap capture (motor_excitation.c)
bool frameWizardWantsDecay(void);
void frameWizardDecaySample(const float dg[3]);
void frameWizardTapDone(const struct motorExcitationTap_s *tap);

// mixer path, every PID loop while active
void frameWizardUpdate(float *motorDisarmed, int motorCount, float disarmOutput, timeUs_t currentTimeUs);

void frameWizardWriteStatus(sbuf_t *dst);
void frameWizardWriteResult(sbuf_t *dst);
void frameWizardWriteResponse(sbuf_t *dst, int base, int motor);
void frameWizardWriteTaps(sbuf_t *dst, int first);
