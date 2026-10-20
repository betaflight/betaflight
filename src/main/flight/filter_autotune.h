/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/streambuf.h"
#include "common/time.h"
#include "pg/pg.h"

// Frame map: the modes the frame wizard confirmed, kept across reboots. The filter autotune needs it.
#define FRAME_MAP_MODES         16
#define FRAME_MAP_PART_TAPS     (1 << 0)    // frame and arms by taps
#define FRAME_MAP_PART_MOTORS   (1 << 1)    // the motor sweep
#define FRAME_MAP_SRC_TAP       (1 << 0)
#define FRAME_MAP_SRC_MOTOR     (1 << 1)
#define FRAME_MAP_SRC_CABLE     (1 << 2)    // a mode of the USB cable only (bench): never a frame mode
#define FRAME_MAP_SRC_UNSTABLE  (1 << 3)    // a tap mode few taps saw, even at its best point: the FC on its mount, the
                                            // hand, a loose part; not a frame mode the filters should count on
#define FRAME_MAP_SRC_MOTOR_ONCE (1 << 4)   // a motor mode of the last run only, not in the run before: the support or the
                                            // hold (horizontal on a bench, vertical in the hand) rather than the frame

// the motor profile of the last motor run: each motor's response peaks, the base rotations, the spin check
#define FRAME_MAP_MOTORS        4
#define FRAME_MAP_MOTOR_PEAKS   8
#define FRAME_MAP_PEAK_AXIS     0x03        // motorPeakInfo: axis of the strongest answer
#define FRAME_MAP_PEAK_SPEEDS   0x30        // seen at both base speeds (2) or one (1), << 4
#define FRAME_MAP_PEAK_CABLE    0x80        // matches a cable mode
#define FRAME_MAP_REPEAT_REL    0.06f       // a motor mode within this of one of the run before is repeated
#define FRAME_MAP_SPIN_CHECKED  0x80        // spinCheck: the spin check ran; bits 0-3 the motors against the pattern

typedef struct frameMapConfig_s {
    uint8_t parts;                          // FRAME_MAP_PART_*
    uint8_t count;
    uint16_t modeHz10[FRAME_MAP_MODES];     // Hz x 10, ascending
    uint8_t modeSrc[FRAME_MAP_MODES];       // FRAME_MAP_SRC_*
    uint8_t motorPeakCount[FRAME_MAP_MOTORS];
    uint16_t motorPeakHz10[FRAME_MAP_MOTORS][FRAME_MAP_MOTOR_PEAKS];
    uint16_t motorPeakGain[FRAME_MAP_MOTORS][FRAME_MAP_MOTOR_PEAKS];   // response x 10000
    uint8_t motorPeakInfo[FRAME_MAP_MOTORS][FRAME_MAP_MOTOR_PEAKS];    // FRAME_MAP_PEAK_*
    uint16_t rotHz10[2];                    // mean rotation at each base speed, Hz x 10, 0 = unknown
    uint8_t spinCheck;                      // FRAME_MAP_SPIN_CHECKED | motors against the pattern
    int8_t runGravity[3];                   // the last motor run: "down" in the body frame x 100 (how it was held)
} frameMapConfig_t;

PG_DECLARE(frameMapConfig_t, frameMapConfig);

// the gyro filter settings before the last applied autotune (revert)
typedef struct filterTuneBackup_s {
    uint8_t valid;
    uint8_t rpmWeights[3];
    uint16_t rpmQ;
    uint8_t dynNotchCount;
    uint16_t lpf1StaticHz;
    uint16_t lpf1DynMinHz;
    uint16_t lpf2StaticHz;
    uint8_t simplifiedGyroFilter;
    uint16_t notch1Hz;
    uint16_t notch1Cutoff;
    uint16_t rpmFadeRangeHz;
    // everything else of the gyro and D-term sections the autotune sets (to its values or to the defaults)
    uint8_t rpmHarmonics, rpmMinHz;
    uint16_t rpmLpfHz;
    uint16_t dynNotchQ, dynNotchMinHz, dynNotchMaxHz;
    uint8_t gyroLpf1Type, gyroLpf2Type;
    uint16_t gyroLpf1DynMaxHz, notch2Hz, notch2Cutoff;
    // the D-term filters of the PID profile it applied to
    uint8_t pidProfileIndex;
    uint16_t dtermLpf1DynMinHz, dtermLpf1DynMaxHz, dtermLpf1StaticHz, dtermLpf2StaticHz, dtermNotchHz, dtermNotchCutoff;
    uint8_t dtermLpf1Type, dtermLpf2Type, dtermLpf1DynExpo, simplifiedDtermFilter;
} filterTuneBackup_t;

PG_DECLARE(filterTuneBackup_t, filterTuneBackup);

typedef enum {
    FILTER_TUNE_NO_MAP = 0,     // no frame test yet: the mode does nothing
    FILTER_TUNE_IDLE,           // mode off, or not armed
    FILTER_TUNE_WAITING,        // mode on, armed, not a steady hover
    FILTER_TUNE_COLLECTING,     // steady hover, frames coming in
    FILTER_TUNE_READY,          // enough frames: land and disarm with the mode on to apply
    FILTER_TUNE_APPLIED,        // saved at the last disarm, active from the next arm
    FILTER_TUNE_DISCARDED,      // disarmed with the mode off, or without enough frames
    FILTER_TUNE_NO_TELEMETRY,   // needs DShot rotation telemetry and the RPM filter
    FILTER_TUNE_PARTIAL,        // disarmed with the mode on but too few frames: kept, the next flight adds to them
} filterTuneState_e;

void filterAutotuneInit(void);
// gyro filter loop: the gyro before and after the RPM filter
void filterAutotunePush(const float pre[3], const float post[3]);
void filterAutotuneUpdate(timeUs_t currentTimeUs);
bool filterAutotuneRevert(void);

// the frame wizard has new modes for its part (FRAME_MAP_PART_*): replace that part, keep the other; saved later.
// unstable (may be NULL): per mode, FRAME_MAP_SRC_UNSTABLE
void frameMapSetPart(uint8_t part, const float *hz, const bool *unstable, int n);
void frameMapClear(void);
// the USB cable's own modes from the last taps run with the cable step (a later motor run marks them)
void frameMapSetCable(const float *hz, int n);
bool frameMapIsCable(float hz, float rel, float minHz);

// What the pilot sees without the configurator while the FILTER TUNE mode is on: the LEDs with the filter tune
// overlay (blinking blue while collecting, green when ready or applied, red when it cannot work) and an OSD warning.
typedef enum {
    FILTER_TUNE_LED_OFF = 0,        // the dark half of a blink
    FILTER_TUNE_LED_BLUE,
    FILTER_TUNE_LED_GREEN,
    FILTER_TUNE_LED_RED,
} filterTuneLed_e;

#define FILTER_TUNE_SIGNAL_BEEPER   (1 << 0)    // MSP status: the channels this FC has for the signal
#define FILTER_TUNE_SIGNAL_LED      (1 << 1)    // LEDs with the filter tune overlay
#define FILTER_TUNE_SIGNAL_OSD      (1 << 2)

// false: no signal now (mode off, or nothing to say), the LEDs show their usual colours
bool filterAutotuneLedSignal(timeUs_t now, filterTuneLed_e *led);
// false: no warning; else the text (up to 20 characters), and whether it blinks / is critical
bool filterAutotuneOsdWarning(char *text, bool *blinking, bool *critical);

void filterAutotuneWriteStatus(sbuf_t *dst);
void frameMapWrite(sbuf_t *dst);

extern bool filterAutotuneCollecting;
