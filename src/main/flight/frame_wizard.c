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

#if defined(USE_MOTOR_EXCITATION) && defined(USE_FRAME_WIZARD)

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/motor_types.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/motor_excitation.h"
#include "flight/pid.h"
#include "sensors/acceleration.h"
#include "pg/motor.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"

#include "flight/filter_autotune.h"

#include "bench_scratch.h"
#include "frame_wizard.h"

// ---- tap labelling (from-below protocol; thresholds checked on the bench taps) ----
// Relative to the session's 90th percentiles (scaleR, scalePush, scaleAcc), so they do not depend on the frame
// size, its mass or the tapping strength. Fitted to reproduce the bench 5" absolute rules (0.07 deg, 20 deg/(m/s),
// 1 g / 0.4 g): 99.2 % the same labels on 1254 recorded taps.
#define TAP_ARM_MIN_REL       0.227f   // weaker rotation than this x scaleR: an arm only along its axis
#define TAP_SIDE_YAW          0.5f     // yaw share above this: sideways, not classified
#define TAP_CENTRE_REL        0.26f    // pushed up, R / scaleR below this x push / scalePush: centre. Separate
                                       // scales: a ratio scale blows up on taps with almost no push
#define TAP_ABOVE_PEAK_REL    0.298f   // push down with acc peak below -this x scaleAcc: tap from above
#define TAP_UNSURE_PEAK_REL   0.119f   // push down with a peak between: not sure
#define TAP_ARM_SECTOR_DEG    30.0f    // within this of an arm axis, else between arms = frame
#define TAP_WEAK_ARM_DEG      15.0f    // a weak tap (R < TAP_ARM_MIN_REL) is an arm only this close to its axis
#define TAP_NOISE_REL         0.08f    // R below this x scaleR: too weak to classify
// The USB cable is its own first step of exactly CABLE_TAPS taps: every tap of that step is a cable tap (a gentle
// frame tap looks like a cable tap by its vectors). A confirmed cable peak is a cable mode only when it is more than
// CABLE_STRUCT_REL away from every confirmed arm and frame mode: a cable tap also rings the frame modes, and weak
// cable taps add noise peaks, so only peaks of the cable alone are marked (more taps only add noise peaks).
#define CABLE_TAPS            5
#define CABLE_MATCH_REL       0.03f    // a peak this close to a cable mode is marked
#define CABLE_MATCH_HZ        4.0f
#define CABLE_STRUCT_REL      0.05f
#define SESSION_TAPS          80
#define SCALE_MIN_TAPS        8        // no labels before the session has this many taps
#define SCALE_SECTORS         3        // ... and strong taps along this many different arms (not one point only)
#define SCALE_STRONG_REL      0.3f     // a strong tap: rotation >= this x the strongest so far
#define SCALE_FALLBACK_TAPS   16       // labels from this many taps on whatever directions they have
#define LABEL_UNKNOWN         (-1)
#define LABEL_PENDING         (-3)     // not enough taps for the scales yet
#define LABEL_CLIPPED         (-4)     // a sensor hit full scale: tap softer
#define FREQ_MAX_OF_RATE      0.4f     // highest analysed frequency, of the sample (loop) rate

// ---- tap spectrum ----
#define DECAY_N               512      // decay samples at up to 4 kHz (128 ms from 8 ms after the tap)
#define DECAY_FS              4000.0f  // decimate the loop rate to about this
#define DECAY_WINDOW_S        0.150f   // decay window of the tap capture (8..158 ms)
#define SPEC_F0               20.0f
#define SPEC_STEP             4.0f
#define SPEC_BINS             400      // 20..1616 Hz, used up to 0.4 x the decay sample rate
#define PEAKS_PER_TAP         8
#define LISTEN_TIMEOUT_MS     2000     // listen ends without a request, keepalive or poll
#define LISTEN_PEAK_MIN_REL   0.0316f  // listen: peaks within 15 dB of the strongest
#define LISTEN_PEAK_PROMINENCE 4.0f    // and 6 dB over the local floor
#define SPIN_EVIDENCE_RATIO   4.0f     // without telemetry: spinning motors at least double the gyro's step noise
#define NO_POWER_S            0.5f     // motor run: the battery or the ESC telemetry gone this long ends it
#define LISTEN_MAX_HZ         1000.0f  // listen spectrum up to here (the frame modes): fewer bins, peaks sooner
#define LISTEN_FFT_N          DECAY_N  // listen: real FFT of each decay axis in place (zero padded to this)
#define LISTEN_FFT_M          (LISTEN_FFT_N / 2)
#define TAP_PEAK_FLOOR_HZ     32.0f    // the local floor of a peak: within +-this
#define TAPS_NEEDED           5
#define TAPS_FALLBACK         8        // a point with this many taps is done even without a confirmed peak
#define CONFIRM_TAPS          3
#define CONFIRM_WINDOW        12       // latest taps of a region used to confirm its peaks
#define CONFIRM_REL           0.03f    // same peak in two taps: within 3 %, but at least CONFIRM_MIN_HZ
#define CONFIRM_MIN_HZ        8.0f     // two spectrum steps: low-Q modes scatter by a few Hz
#define TAP_PEAK_MIN_HZ       25.0f    // below: fewer than 4 cycles in the decay window, hand motion
#define TAP_PEAK_PROMINENCE   2.0f     // power over the local floor (1.4 x in amplitude)
#define TAP_PEAK_MIN_REL      0.003f   // and at least -25 dB of the strongest bin above TAP_PEAK_MIN_HZ (a light
                                       // craft swings in the hand after a tap: the lowest bins dwarf the modes)
#define CONFIRMED_MAX         8
#define STABLE_MIN_TAPS       4        // a point's share of taps that saw a mode counts from this many taps
#define STABLE_MIN_SHARE      0.7f     // under this even at its best point: an unstable mode (frame map flag)

// ---- motor phase ----
#define SWEEP_F0              20.0f
#define SWEEP_BINS            FRAME_WIZARD_SWEEP_BINS
#define SPIN_RAMP_S           1.5f
#define MOTORS                4
#define MOTOR_PEAKS           8
#define MOTOR_PEAK_WINDOW     6        // bins each side (about 1/4 octave)
#define MOTOR_PEAK_PROMINENCE 1.5f     // the stronger side of a pair, or a single peak
#define MOTOR_PEAK_PROMINENCE_LOW 1.2f // the weaker side of a pair
#define MOTOR_PEAK_MIN_HZ     25.0f
#define MOTOR_PEAK_MIN_REL    0.10f    // and at least this share of the motor's strongest peak
#define CAND                  16
#define ROT_HARMONICS         4        // rotation lines k x rotation, k = 1..4
#define ROT_MASK_REL          0.03f    // a peak this close (relative, about one bin) to a rotation line is not a mode
#define KEEPALIVE_TIMEOUT_MS  1500
// With the motors live the host's ~10 Hz status poll is the cable check that works everywhere: many boards
// cannot see an unplug (no VBUS sensing, the detect pin held up by the battery's 5 V), so 5 missed polls stop them
#define KEEPALIVE_MOTORS_MS   500
// Motor safety (motors spinning or about to): limits from hand-held chirp logs (max 56 deg/s smoothed rotation,
// |acc| >= 0.83 g, 17 deg tilt) with a wide margin
#define SAFETY_ROTATION_DPS   300.0f   // smoothed |gyro| above this: spun or swung hard
#define SAFETY_ROTATION_TAU_S 0.02f
#define SAFETY_TILT_DEG       90.0f    // gravity direction this far from the one at the motor start: turned over
#define SAFETY_FREEFALL_G     0.35f    // |acc| below this ...
#define SAFETY_FREEFALL_S     0.05f    // ... this long: falling
#define SAFETY_TILT_MIN_G     0.6f     // the tilt check needs a real gravity vector
#define SAFETY_TURN_WINDOW_S  2.0f     // a turn-over needs real rotation within this window ...
#define SAFETY_TURN_MIN_DEG   40.0f    // ... at least this much; "down" flipping without it is a yank down (a fall)
#define SAFETY_IMPACT_G       6.0f     // |acc| on the 250 Hz channel above this: hit something (motors live, held
                                       // still: max 1.97 g; hits measured up to 13.7 g; the 25 Hz channel
                                       // flattened a table hit to 1.2 g)
#define LIVE_COUNT_PERIOD_S   0.25f    // one motor's live resonance count per period during the sweep
// Spin check after the first sweep: each motor alone plays one tone; with the props off the response is the
// rotor's reaction torque, pure yaw, its sign the spin direction. One tone for all: the phases compare directly.
#define SPINCHECK_TONE_HZ     36.0f    // whole periods in a segment
#define SPINCHECK_SEG_S       0.5f
#define SPINCHECK_SLOT_SEGS   2        // per motor: one segment settles, one measures
#define SPINCHECK_MIN_YAW     0.005f   // deg/s per command unit: under this the motor did not answer
#define SPINCHECK_MAX_AMPLITUDE 150.0f
#define REGION_NONE           0xFF
// base speeds by rotation (DShot telemetry), so any motor / KV / cell count gets comparable speeds
#define ROT_TARGET1_HZ        130.0f
#define ROT_TARGET2_HZ        240.0f
#define ROT_TARGET2_MIN_RATIO 1.5f     // the second speed at least this x the first
#define SEEK_RATE             150.0f   // command units per second while searching the speed
#define SEEK_TIMEOUT_S        5.0f
#define SEEK_SETTLE_S         0.5f
#define MIN_AMPLITUDE         5.0f
// automatic amplitude: keeps the coherent gyro response (amplitude x gain, geometric mean over the bins swept so
// far) at the bench 5" level, so heavy frames get more drive and light ones less
#define AGC_TARGET_DPS        1.25f
#define AGC_MIN_AMPLITUDE     10.0f
#define AGC_MAX_AMPLITUDE     100.0f
#define AGC_STEP              0.05f    // at most 5 % per bin
#define AGC_SKIP_BINS         8        // lowest bins of the first sweep (rigid body, hand) do not steer it
#define MIN_RUN_COMMAND       1030.0f

typedef enum {
    FW_IDLE = 0, FW_TAPS, FW_BEEP_START, FW_SPIN, FW_ANALYSE, FW_DONE, FW_ABORTED, FW_CABLE
} fwPhase_e;

typedef struct {
    float freq;
    float amp;
    uint8_t count;           // taps (or base speeds) that saw it
    uint8_t axis;
} fwPeak_t;

typedef struct {
    float R, yaw, push, d3z, accZ, az;     // rotation impulse, yaw share, 3 ms push, push z, acc peak z, direction
    float Rall;                            // total rotation impulse, yaw included
    float peaks[PEAKS_PER_TAP];
    uint8_t nPeaks;
    int8_t label;
    uint16_t seq;
    bool fixed;                            // labelled by the step it was tapped in (the cable), never relabelled
} fwTap_t;

typedef enum { SPIN_SEEK = 0, SPIN_SETTLE, SPIN_SWEEP, SPIN_CHECK } fwSpinStage_e;

// the large buffers live in the bench scratch area, shared with the filter autotune (disarmed vs armed). The tap
// decay and its spectrum are only used in the taps phases, the sweep gains only from the motor phase on.
typedef struct {
    fwTap_t tap[SESSION_TAPS];
    union {
        struct {
            float decay[3][DECAY_N];
            float spec[SPEC_BINS];
        } t;
        struct {
            uint16_t H[2][MOTORS][3][SWEEP_BINS];  // gain x 10000, deg/s per command unit
            // spin check: per motor and axis the response to its tone (deg/s per command unit)
            float spinRe[MOTORS][3], spinIm[MOTORS][3];
            float spinAmp;
            int spinSeg;
            bool spinDone;
        } m;
    } u;
} fwBig_t;

STATIC_ASSERT(sizeof(fwBig_t) <= BENCH_SCRATCH_BYTES, fwBig_too_large);

static fwBig_t *fwb;

static bool scratchOwned(void)
{
    return fwb && benchScratchOwnedBy(BENCH_SCRATCH_FRAME_WIZARD);
}

static struct {
    fwPhase_e phase;
    frameWizardParams_t p;
    timeMs_t lastKeepaliveMs;
    timeMs_t phaseStartMs;
    // taps
    uint8_t count[FRAME_WIZARD_REGIONS];
    uint8_t beeped;                       // region bits already announced
    uint8_t noMode;                       // region bits done by tap count without a confirmed peak
    int8_t lastLabel;
    uint16_t lastSeq;
    int tapN;
    uint16_t pendingSeq;
    float scaleR, scalePush, scaleAcc, scaleRall;
    bool cableSkipped;                    // not on USB: no cable point
    bool learning;                         // taps are collected, no labels yet (scales need several directions)
    fwPeak_t confirmed[FRAME_WIZARD_REGIONS][CONFIRMED_MAX];
    uint8_t confirmedN[FRAME_WIZARD_REGIONS];
    // decay capture and its incremental spectrum
    bool capturing;
    bool analysing;
    int decim, decimCount;
    float decayAcc[3];
    int decayN;
    int decayExpected;
    uint8_t clippedN;
    int specBin, specAxis, specBins;
    bool sweepBuffers;          // the motors started: the shared buffers hold the sweep gains, the tap spectrum is gone
    // motor phase
    int base;                             // 0 or 1
    float t;                              // seconds into the current base
    fwSpinStage_e stage;
    float stageT;
    bool byRpm;                           // base speeds searched by rotation
    bool limited;                         // a speed target was not reached within the command limit
    float cmdNow;
    float baseCmd[2], amp[2], rotTarget[2];
    float ampM[MOTORS];                   // automatic amplitude per motor
    uint8_t liveCount[MOTORS];            // resonances found so far (status), the final count after the analysis
    float liveT;
    int liveMotor;
    struct { uint8_t bin; bool rotation; float prom; } c0[MOTORS][16];   // first-speed peaks, for the live pairing
    uint8_t c0N[MOTORS];
    uint8_t clippedRegion;                // point of the last clipped tap (from the gyro direction), REGION_NONE
    uint8_t endReason;                    // frameWizardEnd_e
    float gravityRef[3];                  // unit gravity direction at the motor start
    float rotLp;                          // smoothed |gyro|
    float freefallS;
    float turnDeg;                        // rotation within the last ~SAFETY_TURN_WINDOW_S (leaky integral of |gyro|)
    int16_t endInfo[4];                   // at the stop: |acc| x100 g, vertical acc x100 g, turn deg, rotation deg/s
    float lvlSum[MOTORS];
    int lvlN[MOTORS];
    float f1;                             // highest swept frequency
    float phaseRad[MOTORS];
    int bin[MOTORS];
    float accI[MOTORS][3], accQ[MOTORS][3];
    int accN[MOTORS];
    float gMean[3];
    fwPeak_t motorPeaks[MOTORS][MOTOR_PEAKS];  // count: 2 = both base speeds, 1 = one (rotation line on the other)
    uint8_t motorPeakN[MOTORS];
    float rotSum[2];
    uint32_t rotN[2];
    float rotHz[2];                       // mean rotation per base speed (DShot telemetry), 0 = unknown
    bool haveRpm;                         // DShot telemetry: rotation lines can be masked
    float internalOffset, internalScale;
    // beeps
    struct { timeMs_t at; uint8_t index; uint8_t cmd; } beep[12];
    int beepN;
    timeMs_t quietUntilMs;
    // the real loop period: the PID loop follows the gyro clock, which can be a few % off nominal
    timeUs_t lastLoopUs;                  // loop period: moving average (the tap analysis can stretch loops)
    timeUs_t decayT0Us, decayT1Us;        // first and last decay sample: the real decay sample rate
    float dt;                // taps right after a beep are the beep itself
    bool stopPending;
    float noPowerS;                       // motor run: time without the battery or the ESC telemetry
    // spin evidence without telemetry: the gyro's sample-to-sample steps, still motors vs the first base speed
    float prevG[3];
    float quietSq, spinSq;
    uint32_t quietN, spinN;
    bool noSpin;
    // listen: taps on the craft without a run, their peaks for the Frame chart; nothing goes into the session
    bool listening;
    bool listenDirty;                     // the scratch decay / spectrum hold a listen tap, not the motor sweep
    timeMs_t listenMs;                    // last request or poll
    timeMs_t listenTapMs;
    uint16_t listenSeq;                   // taps since the listening started
    uint16_t listenTrigSeq;               // tap triggers since the listening started (before their peaks)
    timeMs_t listenTrigMs;
    uint16_t listenLatencyMs;             // last tap: trigger to its peaks (or to its clipped / no-decay end)
    uint8_t listenFlags;                  // FRAME_WIZARD_LISTEN_*
    uint8_t listenN;
    float listenHz[PEAKS_PER_TAP];
    float listenPow[PEAKS_PER_TAP];
    uint8_t listenFftAxis;                // listen FFT: the next axis, 3 = the peaks
    uint8_t listenFftStage;               // of that axis: window, FFT, split
    bool listenPublished;                 // the current tap is reported (its peaks, or clipped / no decay)
    // the spectrum grid of fwb->u.t.spec: bin b at specF0 + specStep x b
    float specF0, specStep;
} fw;

static float tablesFs;                 // decay sample rate of the spectrum being computed

static void beepAt(timeMs_t at, uint8_t index, uint8_t cmd)
{
    if (fw.beepN < (int)ARRAYLEN(fw.beep)) {
        fw.beep[fw.beepN].at = at; fw.beep[fw.beepN].index = index; fw.beep[fw.beepN].cmd = cmd;
        fw.beepN++;
    }
}

// the pitch tells the point: the frame lowest, then arm 1 .. arm 4 rising (beacon 1 .. 5)
static uint8_t regionBeacon(int region)
{
    return region == FRAME_WIZARD_REGION_FRAME ? DSHOT_CMD_BEACON1 : DSHOT_CMD_BEACON2 + region;
}

static void serviceBeeps(void)
{
    const timeMs_t now = millis();
    for (int i = 0; i < fw.beepN; ) {
        if (cmp32(now, fw.beep[i].at) >= 0) {
            if (isMotorProtocolDshot()) {
                dshotCommandWrite(fw.beep[i].index, getMotorCount(), fw.beep[i].cmd, DSHOT_CMD_TYPE_INLINE);
            }
            fw.quietUntilMs = now + 500;
            fw.beep[i] = fw.beep[--fw.beepN];
        } else {
            i++;
        }
    }
}

bool frameWizardIsActive(void)
{
    return fw.phase != FW_IDLE && fw.phase != FW_DONE && fw.phase != FW_ABORTED;
}

void frameWizardKeepalive(void)
{
    fw.lastKeepaliveMs = millis();
    if (fw.listening) {
        fw.listenMs = fw.lastKeepaliveMs;
    }
}

// listen: disarmed, no run, no motors, no beeps; taps on the craft give their peaks (frameWizardWriteListen).
// Held by requests, keepalives or polls; ends after LISTEN_TIMEOUT_MS without one, on abort, arming or a run.
bool frameWizardListen(void)
{
    if (ARMING_FLAG(ARMED) || frameWizardIsActive() || motorExcitationIsActive()) {
        return false;                                 // the gyro is enough: the spectrum is the gyro decay
    }
    if (!fw.listening) {
        fwb = benchScratchClaim(BENCH_SCRATCH_FRAME_WIZARD);
        fw.listening = true;
        fw.listenDirty = true;                        // the decay buffer shares the motor sweep's memory
        fw.capturing = false;
        fw.analysing = false;
        fw.listenSeq = 0;
        fw.listenTrigSeq = 0;
        fw.listenN = 0;
        fw.listenFlags = 0;
        fw.listenTapMs = 0;
    }
    fw.listenMs = millis();
    return true;
}

// the motors can run: a battery (when the FC measures it), and with bidirectional DShot every ESC answers (an
// unpowered ESC sends nothing). Without this a run on USB power swept still motors and stored peaks of noise.
static uint8_t motorPowerCheck(void)
{
    if (batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE && getBatteryState() == BATTERY_NOT_PRESENT) {
        return FRAME_WIZARD_END_NO_POWER;
    }
#ifdef USE_DSHOT_TELEMETRY
    if (motorConfig()->dev.useDshotTelemetry && !isDshotTelemetryActive()) {
        return FRAME_WIZARD_END_NO_SPIN;
    }
#endif
    return FRAME_WIZARD_END_NONE;
}

bool frameWizardStart(const frameWizardParams_t *params)
{
    // quad only; the taps need a calibrated accelerometer (calibrated in the Setup tab, the wizard does not)
    if (ARMING_FLAG(ARMED) || motorExcitationIsActive() || getMotorCount() != MOTORS
        || !sensors(SENSOR_ACC) || !accHasBeenCalibrated()) {
        return false;
    }
    if (!(params->flags & FRAME_WIZARD_FLAG_TAPS_ONLY) && motorPowerCheck() != FRAME_WIZARD_END_NONE) {
        return false;                                // a run with the motors: battery in, ESCs answering
    }
    memset(&fw, 0, sizeof(fw));
    fwb = benchScratchClaim(BENCH_SCRATCH_FRAME_WIZARD);
    memset(fwb, 0, sizeof(*fwb));
    fw.learning = true;                              // no labels until the taps come from several directions
    fw.p = *params;
    fw.p.base1 = constrain(fw.p.base1 ? fw.p.base1 : 1150, 1050, 1600);
    fw.p.base2 = constrain(fw.p.base2 ? fw.p.base2 : 1300, 1050, 1600);
    fw.p.amplitude = constrain(fw.p.amplitude ? fw.p.amplitude : 40, 5, 150);
    fw.p.sweepS = constrain(fw.p.sweepS ? fw.p.sweepS : 15, 5, 60);
    fw.p.maxCommand = constrain(fw.p.maxCommand ? fw.p.maxCommand : 1700, 1100, 1700);   // speeds are searched by rotation
    fw.internalScale = (motorConvertFromExternal(1600) - motorConvertFromExternal(1400)) / 200.0f;
    fw.internalOffset = motorConvertFromExternal(1600) - fw.internalScale * 1600.0f;
    fw.lastLabel = LABEL_UNKNOWN - 1;
    fw.clippedRegion = REGION_NONE;
    fw.rotTarget[0] = fw.p.rot1Hz ? fw.p.rot1Hz : ROT_TARGET1_HZ;
    fw.rotTarget[1] = fw.p.rot2Hz ? fw.p.rot2Hz : ROT_TARGET2_HZ;
    fw.f1 = FREQ_MAX_OF_RATE / pidGetDT();          // 0.4 x the loop rate (1.6 kHz at 4 kHz)
    motorExcitationSetClipFraction(fw.p.clipPct * 0.01f);
    for (int m = 0; m < MOTORS; m++) {
        fw.ampM[m] = fw.p.amplitude;
    }
    fw.cableSkipped = !usbCableIsActive();             // over Bluetooth / a UART there is no cable to tap
    if (fw.cableSkipped) {
        fw.beeped |= 1 << FRAME_WIZARD_REGION_CABLE;
        fw.noMode |= 1 << FRAME_WIZARD_REGION_CABLE;
    }
    // the cable first (on USB), then the arms and the frame in any order
    fw.phase = (fw.p.flags & FRAME_WIZARD_FLAG_SKIP_TAPS) ? FW_BEEP_START : fw.cableSkipped ? FW_TAPS : FW_CABLE;
    fw.phaseStartMs = millis();
    frameWizardKeepalive();
    return true;
}

static void fwStopMotors(float *motorDisarmed, int motorCount, float disarmOutput)
{
    for (int i = 0; i < motorCount; i++) {
        motorDisarmed[i] = disarmOutput;
    }
}

void frameWizardAbort(void)
{
    fw.listening = false;
    if (frameWizardIsActive()) {
        fw.phase = FW_ABORTED;
        fw.endReason = FRAME_WIZARD_END_USER;
        fw.stopPending = true;
    }
}

static bool motorsLive(void)
{
    return fw.phase == FW_SPIN || fw.phase == FW_BEEP_START;
}

static void gravityNow(float g[3])
{
    const float n = sqrtf(acc.accADC.v[0] * acc.accADC.v[0] + acc.accADC.v[1] * acc.accADC.v[1] + acc.accADC.v[2] * acc.accADC.v[2]);
    for (int k = 0; k < 3; k++) {
        g[k] = n > 0.0f ? acc.accADC.v[k] / n : 0.0f;
    }
}

static uint8_t safetyCheck(float dt)
{
    // the cable out (started on USB): stop at once, without waiting for the keepalive. usbCableIsActive() sees
    // an unplug through the detect pin or the bus suspend; usbCableIsInserted() stays true without VBUS sensing
    if (!fw.cableSkipped && !usbCableIsActive()) {
        return FRAME_WIZARD_END_CABLE;
    }
    if (!motorsLive()) {
        return FRAME_WIZARD_END_NONE;
    }
    const float w = sqrtf(gyro.gyroADCf[0] * gyro.gyroADCf[0] + gyro.gyroADCf[1] * gyro.gyroADCf[1] + gyro.gyroADCf[2] * gyro.gyroADCf[2]);
    fw.rotLp += (w - fw.rotLp) * MIN(1.0f, dt / SAFETY_ROTATION_TAU_S);
    fw.turnDeg += w * dt - fw.turnDeg * MIN(1.0f, dt / SAFETY_TURN_WINDOW_S);
    const float g1 = acc.dev.acc_1G ? 1.0f / acc.dev.acc_1G : 0.0f;
    const float a = sqrtf(acc.accADC.v[0] * acc.accADC.v[0] + acc.accADC.v[1] * acc.accADC.v[1] + acc.accADC.v[2] * acc.accADC.v[2]) * g1;
    float g[3];
    gravityNow(g);
    const float c = g[0] * fw.gravityRef[0] + g[1] * fw.gravityRef[1] + g[2] * fw.gravityRef[2];
    fw.endInfo[0] = lrintf(a * 100.0f);
    fw.endInfo[1] = lrintf(a * c * 100.0f);
    fw.endInfo[2] = lrintf(fw.turnDeg);
    fw.endInfo[3] = lrintf(fw.rotLp);
    if (fw.rotLp > SAFETY_ROTATION_DPS) {
        return FRAME_WIZARD_END_ROTATION;
    }
    const float at = sqrtf(acc.accTap.v[0] * acc.accTap.v[0] + acc.accTap.v[1] * acc.accTap.v[1] + acc.accTap.v[2] * acc.accTap.v[2]) * g1;
    if (at > SAFETY_IMPACT_G) {
        fw.endInfo[0] = lrintf(at * 100.0f);           // the impact level, not the smoothed one
        return FRAME_WIZARD_END_IMPACT;
    }
    fw.freefallS = a < SAFETY_FREEFALL_G ? fw.freefallS + dt : 0.0f;
    if (fw.freefallS > SAFETY_FREEFALL_S) {
        return FRAME_WIZARD_END_FREEFALL;
    }
    if (fw.phase == FW_SPIN && a > SAFETY_TILT_MIN_G) {     // in free fall the direction of "down" is noise
        if (c < cos_approx(DEGREES_TO_RADIANS(SAFETY_TILT_DEG))) {
            // "down" turned over: a real turn-over rotates the craft; a hard yank down flips the reading without it
            return fw.turnDeg >= SAFETY_TURN_MIN_DEG ? FRAME_WIZARD_END_TILT : FRAME_WIZARD_END_FREEFALL;
        }
    }
    return FRAME_WIZARD_END_NONE;
}

// ---------------- taps ----------------

bool frameWizardWantsDecay(void)
{
    return (fw.phase == FW_TAPS || fw.phase == FW_CABLE || fw.listening) && !fw.analysing;
}

static void listenDecayFull(void);

void frameWizardDecaySample(const float dg[3])
{
    if (!frameWizardWantsDecay()) {
        return;
    }
    if (!fw.capturing) {                     // first decay sample of a tap
        const float dt = fw.dt;
        fw.decim = MAX(1, lrintf(1.0f / (dt * DECAY_FS)));
        const float fs = 1.0f / (dt * fw.decim);
        fw.decayExpected = constrain((int)(DECAY_WINDOW_S * fs) - 2, 64, DECAY_N);   // slow loops fill fewer samples
        fw.capturing = true;
        fw.decayN = 0; fw.decimCount = 0;
        fw.decayAcc[0] = fw.decayAcc[1] = fw.decayAcc[2] = 0.0f;
    }
    if (fw.decayN >= fw.decayExpected) {
        return;
    }
    for (int k = 0; k < 3; k++) {
        fw.decayAcc[k] += dg[k];
    }
    if (++fw.decimCount >= fw.decim) {
        fw.decayT1Us = micros();
        if (fw.decayN == 0) {
            fw.decayT0Us = fw.decayT1Us;
        }
        for (int k = 0; k < 3; k++) {
            fwb->u.t.decay[k][fw.decayN] = fw.decayAcc[k] / fw.decim;
            fw.decayAcc[k] = 0.0f;
        }
        fw.decimCount = 0;
        fw.decayN++;
        if (fw.listening && !frameWizardIsActive() && fw.decayN >= fw.decayExpected) {
            listenDecayFull();
        }
    }
}

static void spectrumTables(float fs)
{
    tablesFs = fs;                                   // the bin coefficients come from it, one per step (no table: RAM)
    fw.specF0 = SPEC_F0;
    fw.specStep = SPEC_STEP;
    // bins up to 0.4 x the decay sample rate (no aliasing on slow loops)
    fw.specBins = constrain((int)((FREQ_MAX_OF_RATE * fs - SPEC_F0) / SPEC_STEP) + 1, 8, SPEC_BINS);
}

static void tapFeatures(const motorExcitationTap_t *r, fwTap_t *t)
{
    t->R = sqrtf(r->rot[0] * r->rot[0] + r->rot[1] * r->rot[1]);
    t->yaw = fabsf(r->rot[2]) / MAX(t->R, 1e-6f);
    const float d3 = sqrtf(r->dv3[0] * r->dv3[0] + r->dv3[1] * r->dv3[1] + r->dv3[2] * r->dv3[2]);
    t->push = d3;
    t->Rall = sqrtf(t->R * t->R + r->rot[2] * r->rot[2]);
    t->d3z = r->dv3[2];
    t->accZ = r->accPeakZ;
    t->az = RADIANS_TO_DEGREES(atan2_approx(r->rot[1], r->rot[0]));   // from below: points at the arm
}

static float percentile90(const float *v, int n)
{
    float s[SESSION_TAPS];
    for (int i = 0; i < n; i++) {
        s[i] = v[i];
    }
    for (int i = 1; i < n; i++) {                    // insertion sort, n <= SESSION_TAPS
        for (int j = i; j > 0 && s[j] < s[j - 1]; j--) {
            const float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
        }
    }
    const float x = 0.9f * (n - 1);
    const int i = (int)x;
    return i + 1 < n ? s[i] + (x - i) * (s[i + 1] - s[i]) : s[n - 1];
}

static int classifyTap(const fwTap_t *t)
{
    // thresholds relative to this session's taps (90th percentiles of rotation, push and acc peak):
    // independent of the frame size and mass and of how hard the user taps
    if (t->d3z > 0.0f && t->R * fw.scalePush < TAP_CENTRE_REL * t->push * fw.scaleR) {
        return FRAME_WIZARD_REGION_FRAME;                  // centre from below: pushed up, little rotation per push
    }
    if (t->yaw >= TAP_SIDE_YAW) {
        return LABEL_UNKNOWN;                              // sideways
    }
    float az = t->az;
    if (t->d3z < 0.0f) {
        if (t->accZ < -TAP_ABOVE_PEAK_REL * fw.scaleAcc) {
            az += 180.0f;                                  // clearly from above
        } else if (t->accZ < -TAP_UNSURE_PEAK_REL * fw.scaleAcc) {
            return LABEL_UNKNOWN;
        }
    }
    int best = 0;
    float bestDist = 999.0f;
    for (int m = 0; m < MOTORS; m++) {
        const motorMixer_t *mix = &mixerRuntime.currentMixer[m];
        const float dir = RADIANS_TO_DEGREES(atan2_approx(mix->pitch, mix->roll));
        const float d = fabsf(fmodf(az - dir + 540.0f, 360.0f) - 180.0f);
        if (d < bestDist) {
            bestDist = d; best = m;
        }
    }
    if (t->R < TAP_NOISE_REL * fw.scaleR) {
        return LABEL_UNKNOWN;                              // too weak to tell
    }
    if (t->R < TAP_ARM_MIN_REL * fw.scaleR && bestDist > TAP_WEAK_ARM_DEG) {
        return FRAME_WIZARD_REGION_FRAME;                  // weak and off every arm axis: the centre
    }
    // a weak tap along an arm axis is that arm (a gentle arm tap is not the frame)
    return bestDist > TAP_ARM_SECTOR_DEG ? FRAME_WIZARD_REGION_FRAME : best;
}

static void confirmRegion(int region)
{
    // cluster the per-tap peaks of the region's latest CONFIRM_WINDOW taps: a frequency seen in CONFIRM_TAPS taps
    // (within CONFIRM_REL of it) is confirmed. The window keeps the work per tap small (relabelling runs in one loop).
    const fwTap_t *list[CONFIRM_WINDOW];
    int n = 0;
    for (int i = fw.tapN - 1; i >= 0 && n < CONFIRM_WINDOW; i--) {
        if (fwb->tap[i].label == region) {
            list[n++] = &fwb->tap[i];
        }
    }
    fw.confirmedN[region] = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < list[i]->nPeaks; j++) {
            const float f = list[i]->peaks[j];
            bool known = false;
            for (int c = 0; c < fw.confirmedN[region]; c++) {
                if (fabsf(fw.confirmed[region][c].freq - f) <= MAX(CONFIRM_REL * f, CONFIRM_MIN_HZ)) {
                    known = true;
                }
            }
            if (known) {
                continue;
            }
            int seen = 0;
            float sum = 0.0f;
            for (int i2 = 0; i2 < n; i2++) {
                for (int j2 = 0; j2 < list[i2]->nPeaks; j2++) {
                    if (fabsf(list[i2]->peaks[j2] - f) <= MAX(CONFIRM_REL * f, CONFIRM_MIN_HZ)) {
                        seen++; sum += list[i2]->peaks[j2];
                        break;
                    }
                }
            }
            if (seen >= CONFIRM_TAPS && fw.confirmedN[region] < CONFIRMED_MAX) {
                fwPeak_t *pk = &fw.confirmed[region][fw.confirmedN[region]++];
                pk->freq = sum / seen; pk->count = seen; pk->amp = 0.0f; pk->axis = 0;
            }
        }
    }
}

static int nearestArm(float az, float *dist)
{
    int best = 0;
    float bestDist = 999.0f;
    for (int m = 0; m < MOTORS; m++) {
        const motorMixer_t *mix = &mixerRuntime.currentMixer[m];
        const float d = fabsf(fmodf(az - RADIANS_TO_DEGREES(atan2_approx(mix->pitch, mix->roll)) + 540.0f, 360.0f) - 180.0f);
        if (d < bestDist) {
            bestDist = d; best = m;
        }
    }
    *dist = bestDist;
    return best;
}

// the session so far has strong taps along SCALE_SECTORS different arms: the scales come from a mix of points,
// not from many taps on one point (all frame taps first made frame taps look like arm taps)
static bool directionsSpread(void)
{
    float rmax = 0.0f;
    for (int i = 0; i < fw.tapN; i++) {
        if (!fwb->tap[i].fixed) {
            rmax = MAX(rmax, fwb->tap[i].R);
        }
    }
    uint8_t arms = 0;
    for (int i = 0; i < fw.tapN; i++) {
        const fwTap_t *t = &fwb->tap[i];
        float dist;
        if (!t->fixed && t->R >= SCALE_STRONG_REL * rmax) {
            const int m = nearestArm(t->az, &dist);
            if (dist <= TAP_ARM_SECTOR_DEG) {
                arms |= 1 << m;
            }
        }
    }
    int n = 0;
    for (int m = 0; m < MOTORS; m++) {
        n += (arms >> m) & 1;
    }
    return n >= SCALE_SECTORS;
}

static void relabelSession(void)
{
    // every tap is labelled again with the scales of all taps so far: early taps (before the session showed
    // what an arm tap looks like) get their final label once the scales settle
    // taps of the cable step keep their label and stay out of the scales
    memset(fw.count, 0, sizeof(fw.count));
    int nf = 0;
    for (int i = 0; i < fw.tapN; i++) {
        if (fwb->tap[i].fixed) {
            fw.count[fwb->tap[i].label]++;
        } else {
            nf++;
        }
    }
    fw.learning = nf < SCALE_MIN_TAPS || (nf < SCALE_FALLBACK_TAPS && !directionsSpread());
    if (fw.learning) {
        for (int i = 0; i < fw.tapN; i++) {
            if (!fwb->tap[i].fixed) {
                fwb->tap[i].label = LABEL_PENDING;
            }
        }
        for (int r = 0; r < FRAME_WIZARD_REGIONS; r++) {
            confirmRegion(r);
        }
        return;
    }
    float v[SESSION_TAPS];
    int n = 0;
    for (int i = 0; i < fw.tapN; i++) { if (!fwb->tap[i].fixed) { v[n++] = fwb->tap[i].R; } }
    fw.scaleR = percentile90(v, n);
    n = 0;
    for (int i = 0; i < fw.tapN; i++) { if (!fwb->tap[i].fixed) { v[n++] = fwb->tap[i].push; } }
    fw.scalePush = percentile90(v, n);
    n = 0;
    for (int i = 0; i < fw.tapN; i++) { if (!fwb->tap[i].fixed) { v[n++] = fabsf(fwb->tap[i].accZ); } }
    fw.scaleAcc = percentile90(v, n);
    for (int i = 0; i < fw.tapN; i++) {
        if (fwb->tap[i].fixed) {
            continue;
        }
        fwb->tap[i].label = classifyTap(&fwb->tap[i]);
        if (fwb->tap[i].label >= 0) {
            fw.count[fwb->tap[i].label]++;
        }
    }
    for (int r = 0; r < FRAME_WIZARD_REGIONS; r++) {
        confirmRegion(r);
    }
}

static void startTapSpectrum(void);
static int tapSpectrumPeaks(float *freq, float *power, float minRel, float minProminence);

static void listenPublish(uint8_t flags, int n)
{
    fw.listenFlags = flags;
    fw.listenN = n;
    fw.listenTapMs = millis();
    fw.listenLatencyMs = MIN(cmp32(fw.listenTapMs, fw.listenTrigMs), 65534);
    fw.listenPublished = true;
    fw.listenSeq++;
}

// listen: the decay is complete (before the detector's own window ends): the spectrum starts at once
static void listenDecayFull(void)
{
    if (motorExcitationTapClipped()) {
        listenPublish(FRAME_WIZARD_LISTEN_CLIPPED, 0);
        fw.analysing = true;                          // no more decay samples for this tap
        fw.listenFftAxis = 3 + 1;                     // nothing to compute
        return;
    }
    const float span = cmpTimeUs(fw.decayT1Us, fw.decayT0Us) * 1e-6f;
    const float fs = span > 0.0f ? (fw.decayN - 1) / span : 1.0f / (fw.dt * fw.decim);
    fw.specF0 = 0.0f;
    fw.specStep = fs / LISTEN_FFT_N;
    fw.specBins = constrain((int)(MIN(LISTEN_MAX_HZ, FREQ_MAX_OF_RATE * fs) / fw.specStep) + 1, 8, LISTEN_FFT_M + 1);
    memset(fwb->u.t.spec, 0, fw.specBins * sizeof(fwb->u.t.spec[0]));
    fw.listenDirty = true;
    fw.listenFftAxis = 0;
    fw.listenFftStage = 0;
    fw.analysing = true;
}

// rotate the phasor (c, s) by (dc, ds)
#define FW_ROTATE(c, s, dc, ds) do { const float cn_ = c * dc - s * ds; s = s * dc + c * ds; c = cn_; } while (0)

// listen: the power spectrum of each decay axis added to fwb->u.t.spec. Real FFT in place: the LISTEN_FFT_N samples
// are LISTEN_FFT_M complex points (even real, odd imaginary), a radix-2 FFT of those, then the split into the real
// signal's bins. Window, FFT and split are one loop each (a whole axis in one loop stretched it to ~2 ms).
static void listenWindow(float *x)
{
    const float dc = cosf(2.0f * M_PIf / (fw.decayN - 1)), ds = sinf(2.0f * M_PIf / (fw.decayN - 1));
    float c = 1.0f, s = 0.0f;                         // Hann by a rotating phasor, zeros after the samples
    for (int n = 0; n < LISTEN_FFT_N; n++) {
        x[n] = n < fw.decayN ? x[n] * (0.5f - 0.5f * c) : 0.0f;
        FW_ROTATE(c, s, dc, ds);
    }
}

static void listenFft(float *z)
{
    const int M = LISTEN_FFT_M;
    for (int i = 1, j = 0; i < M; i++) {              // bit reversal
        int bit = M >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float t = z[2 * i]; z[2 * i] = z[2 * j]; z[2 * j] = t;
            t = z[2 * i + 1]; z[2 * i + 1] = z[2 * j + 1]; z[2 * j + 1] = t;
        }
    }
    for (int half = 1; half < M; half <<= 1) {
        const float dc = cosf(M_PIf / half), ds = -sinf(M_PIf / half);
        float c = 1.0f, s = 0.0f;                     // twiddle exp(-j pi k / half)
        for (int k = 0; k < half; k++) {
            for (int a = k; a < M; a += 2 * half) {
                const int b = a + half;
                const float tr = z[2 * b] * c - z[2 * b + 1] * s;
                const float ti = z[2 * b] * s + z[2 * b + 1] * c;
                z[2 * b] = z[2 * a] - tr;
                z[2 * b + 1] = z[2 * a + 1] - ti;
                z[2 * a] += tr;
                z[2 * a + 1] += ti;
            }
            FW_ROTATE(c, s, dc, ds);
        }
    }
}

// X[k] = E[k] + W^k O[k], E = (Z[k] + Z*[M-k]) / 2, O = (Z[k] - Z*[M-k]) / 2j, W = exp(-j 2 pi / N)
static void listenSplit(const float *z)
{
    const int M = LISTEN_FFT_M;
    const float dc = cosf(2.0f * M_PIf / LISTEN_FFT_N), ds = -sinf(2.0f * M_PIf / LISTEN_FFT_N);
    float c = 1.0f, s = 0.0f;
    for (int k = 0; k < fw.specBins; k++) {
        const int a = k & (M - 1), b = (M - k) & (M - 1);
        const float er = 0.5f * (z[2 * a] + z[2 * b]), ei = 0.5f * (z[2 * a + 1] - z[2 * b + 1]);
        const float or_ = 0.5f * (z[2 * a + 1] + z[2 * b + 1]), oi = -0.5f * (z[2 * a] - z[2 * b]);
        const float xr = er + c * or_ - s * oi, xi = ei + c * oi + s * or_;
        fwb->u.t.spec[k] += xr * xr + xi * xi;
        FW_ROTATE(c, s, dc, ds);
    }
}

// listen, one step per loop: an axis, then the peaks
static void listenStep(void)
{
    if (fw.listenFftAxis < 3) {
        float *z = fwb->u.t.decay[fw.listenFftAxis];
        if (fw.listenFftStage == 0) {
            listenWindow(z);
        } else if (fw.listenFftStage == 1) {
            listenFft(z);
        } else {
            listenSplit(z);
        }
        if (++fw.listenFftStage == 3) {
            fw.listenFftStage = 0;
            fw.listenFftAxis++;
        }
        return;
    }
    if (fw.listenFftAxis == 3) {
        // one tap lights the chart at once: only the clear peaks (the wizard pools many taps, it takes weaker ones)
        listenPublish(0, tapSpectrumPeaks(fw.listenHz, fw.listenPow, LISTEN_PEAK_MIN_REL, LISTEN_PEAK_PROMINENCE));
    }
    fw.analysing = false;
}

static void listenTapDone(const motorExcitationTap_t *tap)
{
    // the detector's window is over: a tap whose decay never completed is reported now
    const bool haveDecay = fw.capturing && fw.decayN >= fw.decayExpected;
    fw.capturing = false;
    if (!fw.listenPublished && !fw.analysing && !haveDecay) {
        listenPublish((tap->clipped ? FRAME_WIZARD_LISTEN_CLIPPED : 0) | FRAME_WIZARD_LISTEN_NO_DECAY, 0);
    }
}

void frameWizardTapTriggered(void)
{
    if (fw.listening && !frameWizardIsActive()) {
        fw.listenTrigSeq++;
        fw.listenTrigMs = millis();
        fw.listenPublished = false;
    }
}

void frameWizardTapDone(const motorExcitationTap_t *tap)
{
    if (fw.listening && !frameWizardIsActive()) {
        listenTapDone(tap);
        return;
    }
    if (fw.phase != FW_TAPS && fw.phase != FW_CABLE) {
        return;
    }
    if (fw.phase == FW_CABLE && fw.count[FRAME_WIZARD_REGION_CABLE] >= CABLE_TAPS) {
        fw.capturing = false;                        // the cable step has its taps
        return;
    }
    if (fw.analysing) {                              // the previous tap is still being analysed: not a new tap
        fw.capturing = false;
        return;
    }
    const bool haveDecay = fw.capturing && fw.decayN >= fw.decayExpected;
    fw.capturing = false;
    if (tap->clipped && cmp32(millis(), fw.quietUntilMs) >= 0 && fw.beepN == 0) {
        // a sensor hit full scale: features and spectrum are wrong. With only the accelerometer clipped the gyro
        // still gives the direction, so the UI can tell which point was hit too hard
        fw.clippedRegion = REGION_NONE;
        if (!(tap->clipped & MOTOR_EXCITATION_TAP_CLIPPED_GYRO)) {
            const float R = sqrtf(tap->rot[0] * tap->rot[0] + tap->rot[1] * tap->rot[1]);
            if (fw.scaleR <= 0.0f || R >= TAP_NOISE_REL * fw.scaleR) {
                const float az = RADIANS_TO_DEGREES(atan2_approx(tap->rot[1], tap->rot[0]));
                float bestDist = 999.0f;
                int best = 0;
                for (int m = 0; m < MOTORS; m++) {
                    const motorMixer_t *mix = &mixerRuntime.currentMixer[m];
                    const float d = fabsf(fmodf(az - RADIANS_TO_DEGREES(atan2_approx(mix->pitch, mix->roll)) + 540.0f, 360.0f) - 180.0f);
                    if (d < bestDist) {
                        bestDist = d; best = m;
                    }
                }
                fw.clippedRegion = bestDist > TAP_ARM_SECTOR_DEG ? FRAME_WIZARD_REGION_FRAME : best;
            }
        }
        fw.lastSeq = tap->seq;
        fw.lastLabel = LABEL_CLIPPED;
        fw.clippedN = MIN(fw.clippedN + 1, 255);
        return;
    }
    if (cmp32(millis(), fw.quietUntilMs) < 0 || fw.beepN > 0 || !haveDecay) {
        fw.lastSeq = tap->seq;                       // the beep itself, or no complete decay
        fw.lastLabel = LABEL_UNKNOWN;
        return;
    }
    if (fw.tapN >= SESSION_TAPS) {
        // full: the oldest tap of the most tapped point leaves, so many taps on one point never push
        // the other points out of the session
        int most = 0;
        for (int r = 1; r < FRAME_WIZARD_REGIONS; r++) {
            if (fw.count[r] > fw.count[most]) {
                most = r;
            }
        }
        int drop = 0;
        for (int i = 0; i < fw.tapN; i++) {
            if (fwb->tap[i].label == most || fwb->tap[i].label < 0) {
                drop = i;
                break;
            }
        }
        memmove(&fwb->tap[drop], &fwb->tap[drop + 1], (fw.tapN - 1 - drop) * sizeof(fwb->tap[0]));
        fw.tapN--;
    }
    tapFeatures(tap, &fwb->tap[fw.tapN]);              // counted once its spectrum is ready
    fwb->tap[fw.tapN].seq = tap->seq;
    fw.pendingSeq = tap->seq;
    startTapSpectrum();
}

static void startTapSpectrum(void)
{
    // the decay sample rate measured over this capture, not assumed from the loop rate
    const float span = cmpTimeUs(fw.decayT1Us, fw.decayT0Us) * 1e-6f;
    spectrumTables(span > 0.0f ? (fw.decayN - 1) / span : 1.0f / (fw.dt * fw.decim));
    fw.analysing = true;
    for (int n = 0; n < fw.decayN; n++) {            // Hann window applied to the data once
        const float w = 0.5f - 0.5f * cos_approx(2.0f * M_PIf * n / (fw.decayN - 1));
        for (int k = 0; k < 3; k++) {
            fwb->u.t.decay[k][n] *= w;
        }
    }
    fw.specBin = 0; fw.specAxis = 0;
    memset(fwb->u.t.spec, 0, sizeof(fwb->u.t.spec));
}

// peaks of the summed power spectrum by prominence over the local floor (a global threshold lets hand motion
// at the low end hide the modes): local maximum, power >= minProminence x the higher of the lowest values
// within +-TAP_PEAK_FLOOR_HZ, >= minRel of the strongest bin; the most prominent PEAKS_PER_TAP kept
static int tapSpectrumPeaks(float *freq, float *power, float minRel, float minProminence)
{
    const int nb = fw.specBins;
    const int floorBins = MAX(2, lrintf(TAP_PEAK_FLOOR_HZ / fw.specStep));
    float maxP = 0.0f;
    for (int b = 0; b < nb; b++) {
        if (fw.specF0 + fw.specStep * b >= TAP_PEAK_MIN_HZ) {
            maxP = MAX(maxP, fwb->u.t.spec[b]);
        }
    }
    float pq[PEAKS_PER_TAP];
    int np = 0;
    for (int b = 1; b < nb - 1; b++) {
        const float v = fwb->u.t.spec[b];
        if (fw.specF0 + fw.specStep * b < TAP_PEAK_MIN_HZ || !(v > fwb->u.t.spec[b - 1] && v >= fwb->u.t.spec[b + 1])
            || v < minRel * maxP) {
            continue;
        }
        float lowL = v, lowR = v;
        for (int d = 1; d <= floorBins; d++) {
            if (b - d >= 0) {
                lowL = MIN(lowL, fwb->u.t.spec[b - d]);
            }
            if (b + d < nb) {
                lowR = MIN(lowR, fwb->u.t.spec[b + d]);
            }
        }
        const float q = v / MAX(MAX(lowL, lowR), 1e-12f);
        if (q < minProminence) {
            continue;
        }
        // parabolic refinement of the peak position
        const float a = fwb->u.t.spec[b - 1], c = fwb->u.t.spec[b + 1];
        const float den = a - 2.0f * v + c;
        const float off = fabsf(den) > 1e-12f ? 0.5f * (a - c) / den : 0.0f;
        const float f = fw.specF0 + fw.specStep * (b + constrainf(off, -0.5f, 0.5f));
        int slot = np;
        if (np >= PEAKS_PER_TAP) {
            slot = 0;
            for (int i = 1; i < PEAKS_PER_TAP; i++) {
                if (pq[i] < pq[slot]) {
                    slot = i;
                }
            }
            if (q <= pq[slot]) {
                continue;
            }
        } else {
            np++;
        }
        freq[slot] = f; pq[slot] = q;
        if (power) {
            power[slot] = v;
        }
    }
    return np;
}

static void finishTapSpectrum(void)
{
    fwTap_t *t = &fwb->tap[fw.tapN];
    const int np = tapSpectrumPeaks(t->peaks, NULL, TAP_PEAK_MIN_REL, TAP_PEAK_PROMINENCE);
    t->nPeaks = np;
    t->fixed = fw.phase == FW_CABLE;
    t->label = t->fixed ? FRAME_WIZARD_REGION_CABLE : LABEL_PENDING;
    fw.tapN++;
    relabelSession();
    fw.lastLabel = fwb->tap[fw.tapN - 1].label;
    fw.lastSeq = fw.pendingSeq;
    for (int r = 0; r < FRAME_WIZARD_REGIONS; r++) {
        // done with a confirmed peak, or after TAPS_FALLBACK taps without one: the wizard never stalls on a point;
        // the cable is done after exactly CABLE_TAPS taps
        const bool withMode = fw.count[r] >= TAPS_NEEDED && fw.confirmedN[r] > 0;
        const bool done = r == FRAME_WIZARD_REGION_CABLE ? fw.count[r] >= CABLE_TAPS : (withMode || fw.count[r] >= TAPS_FALLBACK);
        if (done && !(fw.beeped & (1 << r))) {
            fw.beeped |= 1 << r;
            if (!withMode) {
                fw.noMode |= 1 << r;
            }
            if (r == FRAME_WIZARD_REGION_CABLE) {        // the cable: a double beep on all motors
                beepAt(millis(), ALL_MOTORS, DSHOT_CMD_BEACON4);
                beepAt(millis() + 350, ALL_MOTORS, DSHOT_CMD_BEACON4);
            } else {
                beepAt(millis(), r == FRAME_WIZARD_REGION_FRAME ? ALL_MOTORS : r, regionBeacon(r));
            }
        }
    }
    fw.analysing = false;
}

static void stepTapSpectrum(void)
{
    // one bin per loop, all three axes (about 3 x 2 x decayN multiply-adds, ~60 us at 108 MHz): the whole
    // spectrum in ~0.1 s, shorter than the gap between two taps
    const float coef = 2.0f * cos_approx(2.0f * M_PIf * (SPEC_F0 + SPEC_STEP * fw.specBin) / tablesFs);
    float p = 0.0f;
    for (int k = 0; k < 3; k++) {
        const float *x = fwb->u.t.decay[k];
        float s1 = 0.0f, s2 = 0.0f;
        for (int n = 0; n < fw.decayN; n++) {
            const float s = x[n] + coef * s1 - s2;
            s2 = s1; s1 = s;
        }
        p += s1 * s1 + s2 * s2 - coef * s1 * s2;
    }
    fwb->u.t.spec[fw.specBin] = p;
    if (++fw.specBin >= fw.specBins) {
        finishTapSpectrum();
    }
}

// a point stays done once announced, even if relabelling later moves one of its taps
#define FRAME_POINTS_MASK ((1 << FRAME_WIZARD_REGION_CABLE) - 1)   // arms and frame

#ifdef USE_FILTER_AUTOTUNE
static bool isCableMode(float f);
static uint16_t gainU16(float h);
static uint8_t spinCheckSigns(int8_t sgn[MOTORS]);
static bool spinCheckReady(void);

// the frame map (kept across reboots, needed by the filter autotune): this run's part replaces the stored one
// add a mode unless one within 3 % is there already (every point shares most modes: without merging the
// list overflowed and the last points, the frame, were lost)
static void addMode(float *hz, int *n, float f)
{
    for (int i = 0; i < *n; i++) {
        if (fabsf(hz[i] - f) <= 0.03f * f) {
            return;
        }
    }
    if (*n < FRAME_MAP_MODES * 2) {
        hz[(*n)++] = f;
    }
}

// the share of a point's taps that saw f, at the point that saw it most (points with STABLE_MIN_TAPS taps or more;
// with none, any point with CONFIRM_TAPS). The modes of the frame and the arms are there in 80..100 % of the taps
// of some point on the 5" and the 3" runs; the stray ones (the FC on its mount, the hand) in 60 % or less.
static float tapModeSupport(float f)
{
    const float w = MAX(CONFIRM_REL * f, CONFIRM_MIN_HZ);
    float best = 0.0f;
    for (int pass = 0; pass < 2 && best == 0.0f; pass++) {
        const int minTaps = pass == 0 ? STABLE_MIN_TAPS : CONFIRM_TAPS;
        for (int r = 0; r < FRAME_WIZARD_REGION_CABLE; r++) {
            int taps = 0, hits = 0;
            for (int i = 0; i < fw.tapN; i++) {
                const fwTap_t *t = &fwb->tap[i];
                if (t->label != r) {
                    continue;
                }
                taps++;
                for (int j = 0; j < t->nPeaks; j++) {
                    if (fabsf(t->peaks[j] - f) <= w) {
                        hits++;
                        break;
                    }
                }
            }
            if (taps >= minTaps) {
                best = MAX(best, (float)hits / taps);
            }
        }
    }
    return best;
}

// the motor run kept in the config: peaks per motor as on the result page, base rotations, spin check
static void storeMotorProfile(void)
{
    frameMapConfig_t *map = frameMapConfigMutable();
    for (int m = 0; m < MOTORS; m++) {
        const int n = MIN(fw.motorPeakN[m], FRAME_MAP_MOTOR_PEAKS);
        map->motorPeakCount[m] = n;
        for (int i = 0; i < FRAME_MAP_MOTOR_PEAKS; i++) {
            const fwPeak_t *p = &fw.motorPeaks[m][i];
            map->motorPeakHz10[m][i] = i < n ? lrintf(p->freq * 10.0f) : 0;
            map->motorPeakGain[m][i] = i < n ? gainU16(p->amp) : 0;
            map->motorPeakInfo[m][i] = i < n ? (p->axis | (p->count << 4) | (isCableMode(p->freq) ? FRAME_MAP_PEAK_CABLE : 0)) : 0;
        }
    }
    for (int b = 0; b < 2; b++) {
        map->rotHz10[b] = lrintf(fw.rotHz[b] * 10.0f);
    }
    for (int k = 0; k < 3; k++) {                    // how the craft was held: "down" in the body frame
        map->runGravity[k] = (int8_t)constrain(lrintf(fw.gravityRef[k] * 100.0f), -100, 100);
    }
    map->spinCheck = 0;
    if (spinCheckReady() && fwb->u.m.spinDone) {
        int8_t sgn[MOTORS];
        map->spinCheck = FRAME_MAP_SPIN_CHECKED | spinCheckSigns(sgn);
    }
}

static void storeFrameMap(uint8_t part)
{
    float hz[FRAME_MAP_MODES * 2];
    int n = 0;
    if (part == FRAME_MAP_PART_TAPS) {
        // the frame first: its modes are the ones the arms do not all share
        for (int k = 0; k < FRAME_WIZARD_REGION_CABLE; k++) {
            const int r = k == 0 ? FRAME_WIZARD_REGION_FRAME : k - 1;
            for (int i = 0; i < fw.confirmedN[r]; i++) {
                if (!isCableMode(fw.confirmed[r][i].freq)) {
                    addMode(hz, &n, fw.confirmed[r][i].freq);
                }
            }
        }
    } else {
        for (int m = 0; m < MOTORS; m++) {
            for (int i = 0; i < fw.motorPeakN[m]; i++) {
                if (!isCableMode(fw.motorPeaks[m][i].freq)) {
                    addMode(hz, &n, fw.motorPeaks[m][i].freq);
                }
            }
        }
        storeMotorProfile();
    }
    bool unstable[FRAME_MAP_MODES * 2] = { false };
    if (part == FRAME_MAP_PART_TAPS) {
        for (int i = 0; i < n; i++) {
            unstable[i] = tapModeSupport(hz[i]) < STABLE_MIN_SHARE;
        }
    }
    frameMapSetPart(part, hz, unstable, n);
    if (part == FRAME_MAP_PART_TAPS && !fw.cableSkipped) {
        // the cable's own modes, for a later motor run without the cable step
        float cable[CONFIRMED_MAX];
        int nc = 0;
        for (int i = 0; i < fw.confirmedN[FRAME_WIZARD_REGION_CABLE] && nc < CONFIRMED_MAX; i++) {
            if (isCableMode(fw.confirmed[FRAME_WIZARD_REGION_CABLE][i].freq)) {
                cable[nc++] = fw.confirmed[FRAME_WIZARD_REGION_CABLE][i].freq;
            }
        }
        frameMapSetCable(cable, nc);
    }
}
#endif

static void finishTapsOnly(timeMs_t now)
{
    for (int i = 0; i < 3; i++) {
        beepAt(now + 500 + 180 * i, ALL_MOTORS, DSHOT_CMD_BEACON1);
    }
#ifdef USE_FILTER_AUTOTUNE
    storeFrameMap(FRAME_MAP_PART_TAPS);
#endif
    fw.phase = FW_DONE;
    fw.endReason = FRAME_WIZARD_END_DONE;
}

static void startMotorBeep(timeMs_t now)
{
    memset(&fwb->u.m, 0, sizeof(fwb->u.m));          // the tap buffers are over: the sweep gains start empty
    fw.sweepBuffers = true;
    // long beep: four beacons back to back on all motors, then the motors start
    for (int i = 0; i < 4; i++) {
        beepAt(now + 600 + 260 * i, ALL_MOTORS, DSHOT_CMD_BEACON3);
    }
    fw.phase = FW_BEEP_START;
    fw.phaseStartMs = now;
}

// ---------------- motors ----------------

static float sweepFreq(int motor, float tSweep, float *frac)
{
    float x = tSweep / fw.p.sweepS + motor * (1.0f / MOTORS);
    x -= floorf(x);
    *frac = x;
    return SWEEP_F0 * expf(logf(fw.f1 / SWEEP_F0) * x);
}

static bool nearRotation(float f, float rot, float rel)
{
    if (rot <= 0.0f) {
        return false;
    }
    for (int k = 1; k <= ROT_HARMONICS; k++) {
        if (fabsf(f - k * rot) < rel * f) {
            return true;
        }
    }
    return false;
}

static void trackGyroMean(float dt)
{
    for (int k = 0; k < 3; k++) {
        fw.gMean[k] += (dt / 0.2f) * (gyro.gyroADC[k] - fw.gMean[k]);
    }
}

static void lockIn(int m, float s, float c, float amp)
{
    const float inv = 1.0f / amp;             // per command unit: the amplitude may change within a bin
    for (int k = 0; k < 3; k++) {             // synchronous detection against this motor's own chirp
        const float g = (gyro.gyroADC[k] - fw.gMean[k]) * inv;
        fw.accI[m][k] += g * s;
        fw.accQ[m][k] += g * c;
    }
    fw.accN[m]++;
}

static bool finishLock(int m, float h[3])
{
    const bool ok = fw.accN[m] > 0;
    for (int k = 0; k < 3; k++) {
        h[k] = ok ? 2.0f * sqrtf(fw.accI[m][k] * fw.accI[m][k] + fw.accQ[m][k] * fw.accQ[m][k])
                    / fw.accN[m] : 0.0f;
        fw.accI[m][k] = fw.accQ[m][k] = 0.0f;
    }
    fw.accN[m] = 0;
    return ok;
}

static uint16_t gainU16(float h)
{
    return (uint16_t)MIN(lrintf(h * 10000.0f), 65535);
}

static void finishBin(int m)
{
    const int b = fw.bin[m];
    float h[3];
    if (finishLock(m, h) && b >= 0 && b < SWEEP_BINS) {
        for (int k = 0; k < 3; k++) {
            fwb->u.m.H[fw.base][m][k][b] = gainU16(h[k]);
        }
        const float g = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
        if (g > 0.0f && (fw.base > 0 || b >= AGC_SKIP_BINS)) {     // the level carries over to the second speed
            fw.lvlSum[m] += logf(g);
            fw.lvlN[m]++;
            const float want = AGC_TARGET_DPS / expf(fw.lvlSum[m] / fw.lvlN[m]);
            fw.ampM[m] = constrainf(constrainf(want, fw.ampM[m] * (1.0f - AGC_STEP), fw.ampM[m] * (1.0f + AGC_STEP)),
                                    AGC_MIN_AMPLITUDE, AGC_MAX_AMPLITUDE);
        }
    }
}

static float chirpSample(int m, float f, float dt, float *c)
{
    fw.phaseRad[m] += 2.0f * M_PIf * f * dt;
    if (fw.phaseRad[m] > 2.0f * M_PIf) {
        fw.phaseRad[m] -= 2.0f * M_PIf;
    }
    *c = cos_approx(fw.phaseRad[m]);
    return sin_approx(fw.phaseRad[m]);
}

static float motorOutput(float cmd)
{
    cmd = constrainf(cmd, 1020.0f, fw.p.maxCommand);
    return fw.internalOffset + fw.internalScale * cmd;
}

static bool haveRpmTelemetry(void)
{
#ifdef USE_DSHOT_TELEMETRY
    return isDshotTelemetryActive();
#else
    return false;
#endif
}

static float rotationNowHz(void)
{
#ifdef USE_DSHOT_TELEMETRY
    if (isDshotTelemetryActive()) {
        return getDshotRpmAverage() / 60.0f;
    }
#endif
    return 0.0f;
}

static void startBase(int base)
{
    fw.base = base;
    fw.t = 0.0f;
    fw.stage = SPIN_SEEK;
    fw.stageT = 0.0f;
    fw.cmdNow = base == 0 ? MIN_RUN_COMMAND : fw.baseCmd[0];
    if (base == 1) {
        fw.rotTarget[1] = MAX(fw.rotTarget[1], ROT_TARGET2_MIN_RATIO * fw.rotHz[0]);
    }
}

static void spinCheckStep(float *motorDisarmed, float dt);

// the gyro's step since the last loop, squared and summed over the axes: the motors' vibration, not the hand
static float gyroStepSq(void)
{
    float s = 0.0f;
    for (int k = 0; k < 3; k++) {
        const float d = gyro.gyroADC[k] - fw.prevG[k];
        s += d * d;
        fw.prevG[k] = gyro.gyroADC[k];
    }
    return s;
}

static void spinStep(float *motorDisarmed, float dt)
{
    const float stepSq = gyroStepSq();                // every loop: the step is always from the last loop
    trackGyroMean(dt);
    fw.stageT += dt;
    if (fw.stage == SPIN_SEEK) {
        bool done;
        if (fw.byRpm) {                            // raise the command until the rotation target is reached
            const bool atLimit = fw.cmdNow >= fw.p.maxCommand - MIN_AMPLITUDE;
            done = rotationNowHz() >= fw.rotTarget[fw.base] || atLimit || fw.stageT > SEEK_TIMEOUT_S;
            if (done && rotationNowHz() < fw.rotTarget[fw.base]) {
                fw.limited = true;
            }
            if (!done) {
                fw.cmdNow += SEEK_RATE * dt;
            }
        } else {                                   // no telemetry: ramp to the given base command
            const float from = fw.base == 0 ? MIN_RUN_COMMAND : fw.p.base1;
            const float to = fw.base == 0 ? fw.p.base1 : fw.p.base2;
            fw.cmdNow = from + (to - from) * constrainf(fw.stageT / SPIN_RAMP_S, 0.0f, 1.0f);
            done = fw.stageT >= SPIN_RAMP_S;
        }
        if (done) {
            const int b = fw.base;
            fw.baseCmd[b] = fw.cmdNow;
            // the chirp must stay above the running minimum and below the limit
            fw.amp[b] = MAX(MIN(fw.cmdNow - MIN_RUN_COMMAND, fw.p.maxCommand - fw.cmdNow), MIN_AMPLITUDE);
            fw.stage = SPIN_SETTLE;
            fw.stageT = 0.0f;
        }
    } else if (fw.stage == SPIN_SETTLE) {
        if (fw.base == 0 && !fw.haveRpm) {
            fw.spinSq += stepSq;
            fw.spinN++;
        }
        if (fw.stageT >= SEEK_SETTLE_S && fw.base == 0 && !fw.haveRpm && fw.quietN && fw.spinN
            && fw.spinSq / fw.spinN < SPIN_EVIDENCE_RATIO * fw.quietSq / fw.quietN) {
            fw.noSpin = true;                         // no telemetry and no vibration: the motors are not turning
        }
        if (fw.stageT >= SEEK_SETTLE_S) {
            fw.stage = SPIN_SWEEP;
            fw.stageT = 0.0f;
            for (int m = 0; m < MOTORS; m++) {
                fw.bin[m] = -1;
            }
        }
    }
    if (fw.stage == SPIN_CHECK) {
        spinCheckStep(motorDisarmed, dt);
        return;
    }
    if (fw.stage != SPIN_SWEEP) {
        for (int m = 0; m < MOTORS; m++) {
            motorDisarmed[m] = motorOutput(fw.cmdNow);
        }
        return;
    }
    const float tSweep = fw.stageT - dt;
    if (fw.haveRpm) {
        fw.rotSum[fw.base] += rotationNowHz();
        fw.rotN[fw.base]++;
    }
    for (int m = 0; m < MOTORS; m++) {
        float frac, c;
        const float s = chirpSample(m, sweepFreq(m, tSweep, &frac), dt, &c);
        const int bin = MIN((int)(frac * SWEEP_BINS), SWEEP_BINS - 1);
        if (bin != fw.bin[m]) {
            finishBin(m);
            fw.bin[m] = bin;
        }
        const float amp = MIN(fw.ampM[m], fw.amp[fw.base]);   // automatic, within the command limits
        lockIn(m, s, c, amp);
        motorDisarmed[m] = motorOutput(fw.baseCmd[fw.base] + amp * s);
    }
}

// ---------------- spin check ----------------

static void startSpinCheck(void)
{
    fwb->u.m.spinAmp = MIN(fw.amp[0], SPINCHECK_MAX_AMPLITUDE);
    fwb->u.m.spinSeg = 0;
    fw.stage = SPIN_CHECK;
    fw.stageT = 0.0f;
    for (int m = 0; m < MOTORS; m++) {
        fw.phaseRad[m] = 0.0f;
        fw.accN[m] = 0;
        for (int k = 0; k < 3; k++) {
            fw.accI[m][k] = fw.accQ[m][k] = 0.0f;
        }
    }
}

static float spinCheckSeconds(void)
{
    return (fw.p.flags & FRAME_WIZARD_FLAG_NO_SPIN_CHECK) ? 0.0f : MOTORS * SPINCHECK_SLOT_SEGS * SPINCHECK_SEG_S;
}

static void finishSpinSegment(int seg)
{
    const int m = seg / SPINCHECK_SLOT_SEGS;
    if (m < MOTORS && seg % SPINCHECK_SLOT_SEGS == SPINCHECK_SLOT_SEGS - 1 && fw.accN[m] > 0) {   // the measuring one
        for (int k = 0; k < 3; k++) {
            fwb->u.m.spinRe[m][k] = 2.0f * fw.accI[m][k] / fw.accN[m];
            fwb->u.m.spinIm[m][k] = 2.0f * fw.accQ[m][k] / fw.accN[m];
        }
    }
    for (int i = 0; i < MOTORS; i++) {
        fw.accN[i] = 0;
        for (int k = 0; k < 3; k++) {
            fw.accI[i][k] = fw.accQ[i][k] = 0.0f;
        }
        fw.phaseRad[i] = 0.0f;                        // whole periods: the tone goes on from phase 0
    }
}

static void spinCheckStep(float *motorDisarmed, float dt)
{
    const float tS = fw.stageT - dt;                  // this sample
    const int seg = (int)(tS / SPINCHECK_SEG_S);
    if (seg != fwb->u.m.spinSeg) {
        finishSpinSegment(fwb->u.m.spinSeg);
        fwb->u.m.spinSeg = seg;
    }
    if (tS >= spinCheckSeconds()) {
        fwb->u.m.spinDone = true;
    }
    const int active = seg / SPINCHECK_SLOT_SEGS;     // one motor at a time, the others hold the base
    for (int m = 0; m < MOTORS; m++) {
        float out = fw.baseCmd[0];
        if (m == active && !fwb->u.m.spinDone) {
            float c;
            const float sn = chirpSample(m, SPINCHECK_TONE_HZ, dt, &c);
            lockIn(m, sn, c, fwb->u.m.spinAmp);
            out += fwb->u.m.spinAmp * sn;
        }
        motorDisarmed[m] = motorOutput(out);
    }
}

static bool sweepDone(void)
{
    return fw.stage == SPIN_SWEEP && fw.stageT >= fw.p.sweepS;
}

static float binFreq(float b)
{
    return SWEEP_F0 * expf(logf(fw.f1 / SWEEP_F0) * (b + 0.5f) / SWEEP_BINS);
}

typedef struct {
    int bin;
    float freq, amp, prom;
    uint8_t axis;
    bool rotation;           // on a rotation line at this speed
} fwCand_t;

static int motorPeaksAt(int base, int motor, fwCand_t *out, int max)
{
    // peaks by prominence over the local floor (the gyro response falls with frequency, so a plain
    // threshold on the maximum only ever finds the lowest bins): local maximum over +-2 bins and
    // at least MOTOR_PEAK_PROMINENCE x the higher of the lowest values within +-MOTOR_PEAK_WINDOW bins.
    // The position is refined by a parabola through the log magnitudes; peaks on a rotation line are flagged.
    float tot[SWEEP_BINS];
    for (int b = 0; b < SWEEP_BINS; b++) {
        float s = 0.0f;
        for (int k = 0; k < 3; k++) {
            const float h = fwb->u.m.H[base][motor][k][b] * 1e-4f;
            s += h * h;
        }
        tot[b] = sqrtf(s);
    }
    int n = 0;
    for (int b = MOTOR_PEAK_WINDOW; b < SWEEP_BINS - MOTOR_PEAK_WINDOW; b++) {
        if (binFreq(b) < MOTOR_PEAK_MIN_HZ || tot[b] <= 0.0f) {
            continue;
        }
        bool filled = true;                          // the whole window swept (live counts on a partial sweep)
        for (int d = -MOTOR_PEAK_WINDOW; d <= MOTOR_PEAK_WINDOW && filled; d++) {
            filled = tot[b + d] > 0.0f;
        }
        if (!filled) {
            continue;
        }
        bool isMax = true;
        for (int d = -2; d <= 2 && isMax; d++) {
            if (d && tot[b + d] > tot[b]) {
                isMax = false;
            }
        }
        if (!isMax) {
            continue;
        }
        float lowL = tot[b], lowR = tot[b];
        for (int d = 1; d <= MOTOR_PEAK_WINDOW; d++) {
            lowL = MIN(lowL, tot[b - d]);
            lowR = MIN(lowR, tot[b + d]);
        }
        const float p = tot[b] / MAX(MAX(lowL, lowR), 1e-6f);
        if (p < MOTOR_PEAK_PROMINENCE_LOW) {
            continue;
        }
        float off = 0.0f;
        if (tot[b - 1] > 0.0f && tot[b + 1] > 0.0f) {
            const float a = logf(tot[b - 1]), v = logf(tot[b]), c = logf(tot[b + 1]);
            const float den = a - 2.0f * v + c;
            off = den < -1e-6f ? constrainf(0.5f * (a - c) / den, -0.5f, 0.5f) : 0.0f;
        }
        const float f = binFreq(b + off);
        int ax = 0;
        for (int k = 1; k < 3; k++) {
            if (fwb->u.m.H[base][motor][k][b] > fwb->u.m.H[base][motor][ax][b]) {
                ax = k;
            }
        }
        int slot = n;
        if (n >= max) {
            slot = 0;
            for (int i = 1; i < max; i++) {
                if (out[i].prom < out[slot].prom) {
                    slot = i;
                }
            }
            if (p <= out[slot].prom) {
                continue;
            }
        } else {
            n++;
        }
        out[slot].bin = b; out[slot].freq = f; out[slot].amp = tot[b]; out[slot].prom = p; out[slot].axis = ax;
        out[slot].rotation = nearRotation(f, fw.rotHz[base], ROT_MASK_REL);
    }
    return n;
}

static void sortPeaksByFreq(fwPeak_t *pk, int n)
{
    for (int i = 1; i < n; i++) {
        for (int j = i; j > 0 && pk[j].freq < pk[j - 1].freq; j--) {
            const fwPeak_t t = pk[j]; pk[j] = pk[j - 1]; pk[j - 1] = t;
        }
    }
}

static bool binSwept(int base, int motor, int b)
{
    return fwb->u.m.H[base][motor][0][b] || fwb->u.m.H[base][motor][1][b] || fwb->u.m.H[base][motor][2][b];
}

static void updateLiveCount(int m)
{
    // resonances found so far, on the same rules as the final analysis: during the first speed its clear peaks;
    // during the second, the first-speed peaks still ahead plus the pairs already confirmed
    fwCand_t c[CAND];
    if (fw.base == 0) {
        fw.rotHz[0] = fw.rotN[0] ? fw.rotSum[0] / fw.rotN[0] : 0.0f;
        const int n = motorPeaksAt(0, m, c, CAND);
        int count = 0;
        for (int i = 0; i < n; i++) {
            count += !c[i].rotation && c[i].prom >= MOTOR_PEAK_PROMINENCE;
        }
        fw.liveCount[m] = MIN(count, MOTOR_PEAKS);
        return;
    }
    fw.rotHz[1] = fw.rotN[1] ? fw.rotSum[1] / fw.rotN[1] : 0.0f;
    const int n1 = motorPeaksAt(1, m, c, CAND);
    int count = 0;
    for (int i = 0; i < fw.c0N[m]; i++) {
        const int b0 = fw.c0[m][i].bin;
        if (!binSwept(1, m, b0)) {
            count += !fw.c0[m][i].rotation && fw.c0[m][i].prom >= MOTOR_PEAK_PROMINENCE;   // still ahead
            continue;
        }
        for (int j = 0; j < n1; j++) {
            if (ABS(c[j].bin - b0) <= 2 && !(c[j].rotation && fw.c0[m][i].rotation)
                && MAX(c[j].prom, fw.c0[m][i].prom) >= MOTOR_PEAK_PROMINENCE) {
                count++;
                break;
            }
        }
    }
    fw.liveCount[m] = MIN(count, MOTOR_PEAKS);
}

static void cacheFirstSpeedPeaks(void)
{
    for (int m = 0; m < MOTORS; m++) {
        fwCand_t c[CAND];
        const int n = motorPeaksAt(0, m, c, CAND);
        fw.c0N[m] = n;
        for (int i = 0; i < n; i++) {
            fw.c0[m][i].bin = c[i].bin; fw.c0[m][i].rotation = c[i].rotation; fw.c0[m][i].prom = c[i].prom;
        }
    }
}

static void analyseCoarse(void)
{
    // A mode is a pair of peaks at the two base speeds (within 2 bins): at least one side clear of rotation
    // lines, the stronger side at MOTOR_PEAK_PROMINENCE, the weaker at MOTOR_PEAK_PROMINENCE_LOW (a mode under a
    // rotation line at one speed is confirmed by the other). A single peak clear of rotation lines counts too
    // when the other speed has a rotation line there. The MOTOR_PEAKS most prominent are kept, by frequency.
    for (int m = 0; m < MOTORS; m++) {
        fwCand_t c0[CAND], c1[CAND];
        const int n0 = motorPeaksAt(0, m, c0, CAND);
        const int n1 = motorPeaksAt(1, m, c1, CAND);
        bool used1[CAND] = { false };
        fwPeak_t cand[CAND * 2];
        float prom[CAND * 2];
        int nc = 0;
        for (int i = 0; i < n0; i++) {
            int best = -1;
            for (int j = 0; j < n1; j++) {
                if (!used1[j] && ABS(c0[i].bin - c1[j].bin) <= 2
                    && (best < 0 || ABS(c0[i].bin - c1[j].bin) < ABS(c0[i].bin - c1[best].bin))) {
                    best = j;
                }
            }
            if (best >= 0) {
                const fwCand_t *a = &c0[i], *b = &c1[best];
                if ((a->rotation && b->rotation) || MAX(a->prom, b->prom) < MOTOR_PEAK_PROMINENCE) {
                    continue;
                }
                used1[best] = true;
                cand[nc].freq = sqrtf(a->freq * b->freq);
                cand[nc].amp = 0.5f * (a->amp + b->amp);
                cand[nc].axis = a->axis;
                cand[nc].count = 2;
                prom[nc++] = a->rotation ? b->prom : b->rotation ? a->prom : MIN(a->prom, b->prom);
            } else if (!c0[i].rotation && c0[i].prom >= MOTOR_PEAK_PROMINENCE
                       && nearRotation(c0[i].freq, fw.rotHz[1], ROT_MASK_REL * 1.5f)) {
                cand[nc].freq = c0[i].freq; cand[nc].amp = c0[i].amp; cand[nc].axis = c0[i].axis; cand[nc].count = 1;
                prom[nc++] = c0[i].prom;
            }
        }
        for (int j = 0; j < n1; j++) {
            if (!used1[j] && !c1[j].rotation && c1[j].prom >= MOTOR_PEAK_PROMINENCE
                && nearRotation(c1[j].freq, fw.rotHz[0], ROT_MASK_REL * 1.5f)) {
                cand[nc].freq = c1[j].freq; cand[nc].amp = c1[j].amp; cand[nc].axis = c1[j].axis; cand[nc].count = 1;
                prom[nc++] = c1[j].prom;
            }
        }
        for (int i = 1; i < nc; i++) {             // most prominent first
            for (int j = i; j > 0 && prom[j] > prom[j - 1]; j--) {
                const fwPeak_t t = cand[j]; cand[j] = cand[j - 1]; cand[j - 1] = t;
                const float tp = prom[j]; prom[j] = prom[j - 1]; prom[j - 1] = tp;
            }
        }
        // a peak far below the motor's strongest is at the noise of the sweep: it came and went between runs
        float maxAmp = 0.0f;
        for (int i = 0; i < nc; i++) {
            maxAmp = MAX(maxAmp, cand[i].amp);
        }
        int keep = 0;
        for (int i = 0; i < nc && keep < MOTOR_PEAKS; i++) {
            if (cand[i].amp >= MOTOR_PEAK_MIN_REL * maxAmp) {
                fw.motorPeaks[m][keep++] = cand[i];
            }
        }
        sortPeaksByFreq(fw.motorPeaks[m], keep);
        fw.motorPeakN[m] = keep;
    }
}

// ---------------- main loop ----------------

void frameWizardUpdate(float *motorDisarmed, int motorCount, float disarmOutput, timeUs_t currentTimeUs)
{
    const timeMs_t now = millis();
    // The chirp phase, the sweep time and the tap integrals use the real time of this loop. An estimated period
    // fails on irregular loops: a stall (blackbox flash write, tap analysis) is followed by catch-up loops with
    // almost no gap, and any clamped or averaged estimate then shifts every frequency (10 % seen on a 3").
    const float nominal = pidGetDT();
    float dt = nominal;
    if (fw.lastLoopUs != 0) {
        dt = constrainf(cmpTimeUs(currentTimeUs, fw.lastLoopUs) * 1e-6f, 0.0f, 0.05f);
        fw.dt += (constrainf(dt, 0.5f * nominal, 2.0f * nominal) - fw.dt) * MIN(1.0f, dt / 0.5f);   // reported only
    } else {
        fw.dt = nominal;
    }
    fw.lastLoopUs = currentTimeUs;
    serviceBeeps();
    if (!frameWizardIsActive()) {
        if (fw.stopPending) {                        // once, so the Motors tab works again afterwards
            fw.stopPending = false;
            fwStopMotors(motorDisarmed, motorCount, disarmOutput);
        }
        if (fw.listening) {
            if (ARMING_FLAG(ARMED) || motorExcitationIsActive() || !scratchOwned()
                || cmp32(now, fw.listenMs) > LISTEN_TIMEOUT_MS) {
                fw.listening = false;
                fw.capturing = false;
                fw.analysing = false;
            } else {
                motorExcitationTapCapture(dt);
                if (fw.analysing) {
                    listenStep();
                }
            }
        }
        return;
    }
    if (fw.phase == FW_SPIN && motorPowerCheck() != FRAME_WIZARD_END_NONE) {
        fw.noPowerS += dt;                           // a short telemetry gap is not a lost ESC
    } else {
        fw.noPowerS = 0.0f;
    }
    uint8_t end = ARMING_FLAG(ARMED) ? FRAME_WIZARD_END_ARMED
                : getBatteryState() == BATTERY_CRITICAL ? FRAME_WIZARD_END_BATTERY
                : fw.noPowerS > NO_POWER_S ? motorPowerCheck()
                : fw.noSpin ? FRAME_WIZARD_END_NO_SPIN
                : cmp32(now, fw.lastKeepaliveMs) > (motorsLive() ? KEEPALIVE_MOTORS_MS : KEEPALIVE_TIMEOUT_MS)
                    ? FRAME_WIZARD_END_KEEPALIVE
                : safetyCheck(dt);
    if (end != FRAME_WIZARD_END_NONE) {
        fw.phase = FW_ABORTED;
        fw.endReason = end;
        fw.stopPending = false;
        fwStopMotors(motorDisarmed, motorCount, disarmOutput);
        return;
    }

    switch (fw.phase) {
    case FW_TAPS:
        fwStopMotors(motorDisarmed, motorCount, disarmOutput);
        motorExcitationTapCapture(dt);
        if (fw.analysing) {
            stepTapSpectrum();
        }
        if ((fw.beeped & FRAME_POINTS_MASK) == FRAME_POINTS_MASK && !fw.analysing) {
            if (fw.p.flags & FRAME_WIZARD_FLAG_TAPS_ONLY) {
                finishTapsOnly(now);
            } else {
                startMotorBeep(now);
            }
        }
        break;
    case FW_CABLE:
        fwStopMotors(motorDisarmed, motorCount, disarmOutput);
        motorExcitationTapCapture(dt);
        if (fw.analysing) {
            stepTapSpectrum();
        }
        if ((fw.beeped & (1 << FRAME_WIZARD_REGION_CABLE)) && !fw.analysing && fw.beepN == 0) {
            fw.phase = FW_TAPS;                       // cable done: now the arms and the frame
            fw.phaseStartMs = now;
        }
        break;
    case FW_BEEP_START:
        fwStopMotors(motorDisarmed, motorCount, disarmOutput);
        if (cmp32(now, fw.phaseStartMs) > 2600 && fw.beepN == 0 && motorPowerCheck() != FRAME_WIZARD_END_NONE) {
            // the battery went (or the ESCs stopped answering) during the taps: the taps are kept, no motor run
#ifdef USE_FILTER_AUTOTUNE
            if (!(fw.p.flags & FRAME_WIZARD_FLAG_SKIP_TAPS)) {
                storeFrameMap(FRAME_MAP_PART_TAPS);
            }
#endif
            fw.endReason = motorPowerCheck();
            fw.phase = FW_DONE;
            break;
        }
        // the still craft's gyro step noise after the beeps, for the spin evidence
        if (cmp32(now, fw.phaseStartMs) > 2000) {
            fw.quietSq += gyroStepSq();
            fw.quietN++;
        } else {
            gyroStepSq();
        }
        if (cmp32(now, fw.phaseStartMs) > 2600 && fw.beepN == 0) {
            fw.phase = FW_SPIN;
            fw.haveRpm = haveRpmTelemetry();
            fw.byRpm = fw.haveRpm;
            gravityNow(fw.gravityRef);                // the attitude the user holds the craft in
            startBase(0);
            for (int m = 0; m < MOTORS; m++) {
                fw.bin[m] = -1; fw.phaseRad[m] = 0.0f; fw.accN[m] = 0;
            }
            for (int k = 0; k < 3; k++) {
                fw.gMean[k] = gyro.gyroADC[k];
            }
        }
        break;
    case FW_SPIN:
        spinStep(motorDisarmed, dt);
        if (fw.stage == SPIN_SWEEP) {
            fw.liveT += dt;
            if (fw.liveT >= LIVE_COUNT_PERIOD_S) {
                fw.liveT = 0.0f;
                updateLiveCount(fw.liveMotor);
                fw.liveMotor = (fw.liveMotor + 1) % MOTORS;
            }
        }
        fw.t += dt;
        if (fw.stage == SPIN_CHECK && fwb->u.m.spinDone) {
            startBase(1);
        } else if (sweepDone()) {
            for (int m = 0; m < MOTORS; m++) {
                finishBin(m);
                fw.bin[m] = -1;
            }
            fw.rotHz[fw.base] = fw.rotN[fw.base] ? fw.rotSum[fw.base] / fw.rotN[fw.base] : 0.0f;
            if (fw.base == 0) {
                cacheFirstSpeedPeaks();
                if (fw.p.flags & FRAME_WIZARD_FLAG_NO_SPIN_CHECK) {
                    startBase(1);
                } else {
                    startSpinCheck();
                }
            } else {
                fwStopMotors(motorDisarmed, motorCount, disarmOutput);
                fw.phase = FW_ANALYSE;
            }
        }
        break;
    case FW_ANALYSE:
        fwStopMotors(motorDisarmed, motorCount, disarmOutput);
        analyseCoarse();
        for (int m = 0; m < MOTORS; m++) {
            fw.liveCount[m] = fw.motorPeakN[m];
        }
        for (int i = 0; i < 3; i++) {
            beepAt(now + 500 + 180 * i, ALL_MOTORS, DSHOT_CMD_BEACON1);
        }
#ifdef USE_FILTER_AUTOTUNE
        if (!(fw.p.flags & FRAME_WIZARD_FLAG_SKIP_TAPS)) {
            storeFrameMap(FRAME_MAP_PART_TAPS);
        }
        storeFrameMap(FRAME_MAP_PART_MOTORS);
#endif
        fw.phase = FW_DONE;
        fw.endReason = FRAME_WIZARD_END_DONE;
        break;
    default:
        break;
    }
}

// ---------------- MSP ----------------

static uint8_t spinProgress(void)
{
    if (fw.phase == FW_DONE || fw.phase == FW_ANALYSE) {
        return 100;
    }
    if (fw.phase != FW_SPIN) {
        return 0;
    }
    const float per = SPIN_RAMP_S + SEEK_SETTLE_S + fw.p.sweepS, first = per + spinCheckSeconds();
    return (uint8_t)constrainf(100.0f * (fw.base * first + fw.t) / (first + per), 0.0f, 99.0f);
}

void frameWizardWriteStatus(sbuf_t *dst)
{
    frameWizardKeepalive();                          // polling the status keeps the wizard alive
    sbufWriteU8(dst, 4);                             // payload version (4: flags, live motor counts, clipped point)
    sbufWriteU8(dst, fw.phase);
    for (int r = 0; r < FRAME_WIZARD_REGIONS; r++) {
        sbufWriteU8(dst, MIN(fw.count[r], 255));
        sbufWriteU8(dst, fw.confirmedN[r]);
    }
    sbufWriteU8(dst, fw.beeped);
    sbufWriteU16(dst, fw.lastSeq);
    // 0 = none yet, 1 = not classified, 2..7 = arm 1-4, frame, cable, 9 = collecting the first taps, 10 = clipped
    sbufWriteU8(dst, fw.lastLabel == LABEL_PENDING ? 9 : fw.lastLabel == LABEL_CLIPPED ? 10 : (uint8_t)(fw.lastLabel + 2));
    sbufWriteU8(dst, spinProgress());
    sbufWriteU32(dst, millis());                      // lets the host correct the MCU clock error
    sbufWriteU16(dst, fw.dt > 0.0f ? lrintf(constrainf(1.0f / fw.dt, 0.0f, 65535.0f)) : 0);   // measured loop rate, Hz
    // bit 0: DShot rotation telemetry now (else no rotation mask); bit 1: accelerometer present; bit 2: calibrated;
    // bit 3: learning (taps collected, no labels until they come from several directions)
    sbufWriteU8(dst, (haveRpmTelemetry() ? 1 : 0) | (sensors(SENSOR_ACC) ? 2 : 0) | (accHasBeenCalibrated() ? 4 : 0)
                     | (fw.phase == FW_TAPS && fw.learning ? 8 : 0));
    for (int m = 0; m < MOTORS; m++) {
        sbufWriteU8(dst, fw.liveCount[m]);              // resonances found so far per motor
    }
    sbufWriteU8(dst, fw.clippedRegion);                 // point of the last too-strong tap, 0xFF unknown
    sbufWriteU8(dst, fw.endReason);                     // frameWizardEnd_e: why the wizard ended
    for (int i = 0; i < 4; i++) {
        sbufWriteU16(dst, (uint16_t)fw.endInfo[i]);     // sensors at the last safety check (frozen at the stop)
    }
}

static bool isCableMode(float f)
{
    // near a confirmed cable peak that is not a mode of the craft (no arm or frame mode within CABLE_STRUCT_REL)
    for (int i = 0; i < fw.confirmedN[FRAME_WIZARD_REGION_CABLE]; i++) {
        const float c = fw.confirmed[FRAME_WIZARD_REGION_CABLE][i].freq;
        if (fabsf(f - c) > MAX(CABLE_MATCH_REL * f, CABLE_MATCH_HZ)) {
            continue;
        }
        bool craft = false;
        for (int r = 0; r < FRAME_WIZARD_REGION_CABLE && !craft; r++) {
            for (int j = 0; j < fw.confirmedN[r]; j++) {
                if (fabsf(c - fw.confirmed[r][j].freq) <= CABLE_STRUCT_REL * c) {
                    craft = true;
                    break;
                }
            }
        }
        if (!craft) {
            return true;
        }
    }
#ifdef USE_FILTER_AUTOTUNE
    // no cable step in this run (a motor run on its own): the cable modes of the last taps run
    if (fw.confirmedN[FRAME_WIZARD_REGION_CABLE] == 0 && frameMapIsCable(f, CABLE_MATCH_REL, CABLE_MATCH_HZ)) {
        return true;
    }
#endif
    return false;
}

void frameWizardWriteResult(sbuf_t *dst)
{
    sbufWriteU8(dst, 4);                             // payload version (4: six points, tap peak flags, cable marks)
    sbufWriteU8(dst, fw.phase);
    for (int r = 0; r < FRAME_WIZARD_REGIONS; r++) {
        sbufWriteU8(dst, fw.confirmedN[r]);
        for (int i = 0; i < fw.confirmedN[r]; i++) {
            sbufWriteU16(dst, lrintf(fw.confirmed[r][i].freq * 10.0f));
            sbufWriteU8(dst, fw.confirmed[r][i].count);
            sbufWriteU8(dst, isCableMode(fw.confirmed[r][i].freq) ? 1 : 0);   // flags: bit 0 = a cable mode
        }
    }
    for (int m = 0; m < MOTORS; m++) {
        sbufWriteU8(dst, fw.motorPeakN[m]);
        for (int i = 0; i < fw.motorPeakN[m]; i++) {
            sbufWriteU16(dst, lrintf(fw.motorPeaks[m][i].freq * 10.0f));
            sbufWriteU16(dst, gainU16(fw.motorPeaks[m][i].amp));
            // axis; bits 4-5: 2 = both speeds, 1 = one speed (rotation line on the other); bit 7: a cable mode
            sbufWriteU8(dst, fw.motorPeaks[m][i].axis | (fw.motorPeaks[m][i].count << 4)
                             | (isCableMode(fw.motorPeaks[m][i].freq) ? FRAME_WIZARD_PEAK_CABLE : 0));
        }
    }
    sbufWriteU16(dst, lrintf(fw.rotHz[0] * 10.0f));
    sbufWriteU16(dst, lrintf(fw.rotHz[1] * 10.0f));
    sbufWriteU8(dst, (fw.haveRpm ? FRAME_WIZARD_RESULT_RPM : 0) | (fw.byRpm ? FRAME_WIZARD_RESULT_BASES_BY_RPM : 0)
                     | (fw.limited ? FRAME_WIZARD_RESULT_SPEED_LIMITED : 0)
                     | (fw.cableSkipped ? FRAME_WIZARD_RESULT_NO_CABLE : 0));
    sbufWriteU16(dst, lrintf(fw.baseCmd[0]));
    sbufWriteU16(dst, lrintf(fw.baseCmd[1]));
    sbufWriteU16(dst, lrintf(fw.amp[0]));
    sbufWriteU16(dst, lrintf(fw.amp[1]));
    sbufWriteU16(dst, lrintf(fw.f1));
    sbufWriteU8(dst, fw.tapN);
    sbufWriteU8(dst, fw.clippedN);
    sbufWriteU8(dst, fw.noMode);
    for (int m = 0; m < MOTORS; m++) {
        sbufWriteU8(dst, (uint8_t)lrintf(fw.ampM[m]));     // final automatic amplitude per motor
    }
}

void frameWizardWriteTaps(sbuf_t *dst, int first)
{
    if (!scratchOwned() || (first >= 200 && fw.sweepBuffers)) {
        // the buffers went to the filter autotune (or the tap spectrum is over): nothing to give
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        return;
    }
    if (first >= 200) {
        // the last tap's power spectrum, 64 bins per page (first = 200 + page): u16 bins, u8 page, then f32 bins
        // (none once listening used the buffer: its spectrum is on another grid)
        const int b0 = (first - 200) * 64;
        const int n = fw.listenDirty ? 0 : constrain(fw.specBins - b0, 0, 64);
        sbufWriteU16(dst, fw.specBins);
        sbufWriteU8(dst, first - 200);
        sbufWriteU8(dst, n);
        for (int b = b0; b < b0 + n; b++) {
            uint32_t u;
            memcpy(&u, &fwb->u.t.spec[b], 4);
            sbufWriteU32(dst, u);
        }
        return;
    }
    if (first >= 100) {
        // the session's tap features, 8 per page (first = 100 + page): u8 taps, u8 first, u8 n, then per tap
        // u16 seq and f32 R, yaw share, push, push z, acc peak z, direction, R with yaw
        const int f0 = constrain((first - 100) * 8, 0, SESSION_TAPS);
        const int n = MIN(8, MAX(fw.tapN - f0, 0));
        sbufWriteU8(dst, fw.tapN);
        sbufWriteU8(dst, f0);
        sbufWriteU8(dst, n);
        for (int i = f0; i < f0 + n; i++) {
            const fwTap_t *t = &fwb->tap[i];
            const float v[7] = { t->R, t->yaw, t->push, t->d3z, t->accZ, t->az, t->Rall };
            sbufWriteU16(dst, t->seq);
            for (int k = 0; k < 7; k++) {
                uint32_t u;
                memcpy(&u, &v[k], 4);
                sbufWriteU32(dst, u);
            }
        }
        return;
    }
    // bench: the session's taps, 16 per page: seq, label, peaks (Hz x 10)
    first = constrain(first, 0, SESSION_TAPS);
    const int n = MIN(16, MAX(fw.tapN - first, 0));
    sbufWriteU8(dst, fw.tapN);
    sbufWriteU8(dst, first);
    sbufWriteU8(dst, n);
    for (int i = first; i < first + n; i++) {
        const fwTap_t *t = &fwb->tap[i];
        sbufWriteU16(dst, t->seq);
        sbufWriteU8(dst, (uint8_t)t->label);
        sbufWriteU8(dst, t->nPeaks);
        for (int j = 0; j < PEAKS_PER_TAP; j++) {
            sbufWriteU16(dst, j < t->nPeaks ? lrintf(t->peaks[j] * 10.0f) : 0);
        }
    }
}

// spin signs per motor against the strongest answer (0: no answer); returns the motors against the quad X pattern
static uint8_t spinCheckSigns(int8_t sgn[MOTORS])
{
    static const int8_t quadX[MOTORS] = { 1, -1, -1, 1 };
    float mag[MOTORS], vote = 0.0f;
    for (int m = 0; m < MOTORS; m++) {
        mag[m] = sqrtf(sq(fwb->u.m.spinRe[m][2]) + sq(fwb->u.m.spinIm[m][2]));
    }
    int ref = 0;                                      // the strongest answer is the phase reference
    for (int m = 1; m < MOTORS; m++) {
        if (mag[m] > mag[ref]) {
            ref = m;
        }
    }
    for (int m = 0; m < MOTORS; m++) {
        const float dot = fwb->u.m.spinRe[m][2] * fwb->u.m.spinRe[ref][2] + fwb->u.m.spinIm[m][2] * fwb->u.m.spinIm[ref][2];
        sgn[m] = mag[m] < SPINCHECK_MIN_YAW ? 0 : dot >= 0.0f ? 1 : -1;
        vote += sgn[m] * quadX[m];
    }
    uint8_t wrong = 0;                                // bit per motor: against the pattern, or no answer
    for (int m = 0; m < MOTORS; m++) {
        if (sgn[m] == 0 || sgn[m] * quadX[m] * (vote >= 0.0f ? 1 : -1) < 0) {
            wrong |= 1 << m;
        }
    }
    return wrong;
}

static bool spinCheckReady(void)
{
    return scratchOwned() && !fw.listenDirty && (fw.sweepBuffers || (fw.p.flags & FRAME_WIZARD_FLAG_SKIP_TAPS));
}

// The spin check page: per motor the yaw phase against its tone and the response on each axis; the spin sign
// against the quad X pattern (motors 1 and 4 one way, 2 and 3 the other), taken from the majority so a single
// reversed motor stands out; a motor that hardly answered is flagged too.
static void writeSpinCheck(sbuf_t *dst)
{
    sbufWriteU8(dst, 2);                              // page version (2: spin check)
    if (!spinCheckReady()) {
        sbufWriteU8(dst, 0);                          // nothing (the buffers hold the taps, or are lent out)
        return;
    }
    int8_t sgn[MOTORS];
    const uint8_t wrong = spinCheckSigns(sgn);
    sbufWriteU8(dst, fwb->u.m.spinDone ? 1 : 0);
    sbufWriteU8(dst, lrintf(SPINCHECK_TONE_HZ));
    sbufWriteU16(dst, lrintf(fwb->u.m.spinAmp));
    sbufWriteU8(dst, wrong);
    for (int m = 0; m < MOTORS; m++) {
        sbufWriteU8(dst, (uint8_t)sgn[m]);
        sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(RADIANS_TO_DEGREES(atan2_approx(fwb->u.m.spinIm[m][2], fwb->u.m.spinRe[m][2])) * 10.0f));
        for (int k = 0; k < 3; k++) {
            sbufWriteU16(dst, (uint16_t)MIN(lrintf(sqrtf(sq(fwb->u.m.spinRe[m][k]) + sq(fwb->u.m.spinIm[m][k])) * 10000.0f), 65535));
        }
    }
}

void frameWizardWriteListen(sbuf_t *dst)
{
    if (fw.listening) {
        fw.listenMs = millis();                       // polling holds the listening
    }
    sbufWriteU8(dst, 1);                              // payload version
    sbufWriteU8(dst, fw.listening ? 1 : 0);
    sbufWriteU16(dst, fw.listenSeq);
    sbufWriteU16(dst, fw.listenSeq ? MIN(cmp32(millis(), fw.listenTapMs), 65534) : 65535);
    sbufWriteU8(dst, fw.listenFlags);
    float maxP = 0.0f;
    for (int i = 0; i < fw.listenN; i++) {
        maxP = MAX(maxP, fw.listenPow[i]);
    }
    sbufWriteU8(dst, fw.listenN);
    for (int i = 0; i < fw.listenN; i++) {
        sbufWriteU16(dst, lrintf(fw.listenHz[i] * 10.0f));
        const float db = maxP > 0.0f ? 10.0f * log10f(MAX(fw.listenPow[i], 1e-12f) / maxP) : 0.0f;
        sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(MAX(db, -300.0f) * 10.0f));
    }
    // appended (same version): the trigger at once, before the peaks; and the last tap's trigger-to-peaks time
    sbufWriteU16(dst, fw.listenTrigSeq);
    sbufWriteU16(dst, fw.listenTrigSeq ? MIN(cmp32(millis(), fw.listenTrigMs), 65534) : 65535);
    sbufWriteU16(dst, fw.listenSeq ? fw.listenLatencyMs : 65535);
}

void frameWizardWriteResponse(sbuf_t *dst, int base, int motor)
{
    if (base == FRAME_WIZARD_RESPONSE_SPIN) {
        writeSpinCheck(dst);
        return;
    }
    if (!spinCheckReady()) {
        for (int i = 0; i < 3 * SWEEP_BINS; i++) {
            sbufWriteU16(dst, 0);                    // no sweep gains (the buffers hold the taps, or are lent out)
        }
        return;
    }
    base = constrain(base, 0, 1);
    motor = constrain(motor, 0, MOTORS - 1);
    for (int k = 0; k < 3; k++) {
        for (int b = 0; b < SWEEP_BINS; b++) {
            sbufWriteU16(dst, fwb->u.m.H[base][motor][k][b]);
        }
    }
}

#endif
