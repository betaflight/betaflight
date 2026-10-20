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

/*
 * Filter autotune (FILTER TUNE mode).
 *
 * Needs the frame map from the frame wizard (frame and arms by taps). In a steady hover with the mode on, the gyro
 * before and after the RPM filter is taken in frames of FT_N samples at about 2 kHz (every other frame; the one in
 * between is analysed in small steps by a low priority task). Per frame:
 *   harmonic strength X[m][h]: the power at h x rotation of motor m (+-1 bin) over the median of the raw spectrum
 *     within +-25 Hz, every motor's every order line left out (orders 1..5);
 *   1x residual: the power after the RPM filter at the 1x lines over the same floor;
 *   the spectrum after the RPM filter, summed (two sums, alternate frames, for a persistence check).
 * Decisions (research/motor-thermal/autotune_sim.py is the offline replica):
 *   rpm weight h = 1 - 10^(-max_m X / 20), 5 % steps, 0 below 3 dB;
 *   rpm Q x 0.6 when the 1x residual is >= 6 dB, x 1.25 below 3 dB (250..500);
 *   residual peaks: 100..600 Hz, >= 10 dB over the higher of the lowest values within +-12 %;
 *   a peak in both sums, away from every rotation order line, takes the static gyro notch 1;
 *   score = 100 - 10 per dB of 1x spread over motors above 3 dB - 15 per residual peak above one;
 *   dynamic notches: 1 with a score >= 85 and at most one residual peak left, else 2;
 *   gyro lpf1 off; lpf2 the PT1 cutoff that takes the strongest uncovered order (4x, 5x) to 6 dB (250..500 Hz).
 * Applied when the craft is disarmed with the mode still on and enough frames; the settings before are kept
 * for a revert. Disarming with the mode off discards the result.
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FILTER_AUTOTUNE

#include "common/filter.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"
#include "config/config.h"
#include "config/feature.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/motor_noise.h"
#include "flight/motor_thermal.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "io/beeper.h"
#include "io/ledstrip.h"
#include "osd/osd.h"
#include "pg/beeper_dev.h"
#include "pg/dyn_notch.h"
#include "pg/pg_ids.h"
#include "pg/rpm_filter.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "bench_scratch.h"
#include "filter_autotune.h"

PG_REGISTER(frameMapConfig_t, frameMapConfig, PG_FRAME_MAP_CONFIG, 2);
PG_REGISTER(filterTuneBackup_t, filterTuneBackup, PG_FILTER_TUNE_BACKUP, 3);
PG_REGISTER_WITH_RESET_TEMPLATE(filterTuneConfig_t, filterTuneConfig, PG_FILTER_TUNE_CONFIG, 0);
PG_RESET_TEMPLATE(filterTuneConfig_t, filterTuneConfig,
    .heat_budget_dk = 20,                             // 2 K: research/motor-thermal/dterm, negligible next to the hover heat
);

#define FT_N            512
#define FT_BINS         (FT_N / 2 + 1)
#define FT_M            (FT_N / 2)          // complex points of the real FFT
#define FT_LOG2M        8
#define FT_MOTORS       4
#define FT_ORDERS       5                   // measured; the RPM filter covers 3
#define FT_FS_TARGET    2000.0f             // decimated sample rate target, Hz
#define FT_MIN_FRAMES   60                  // ~16 s of samples, ~32 s of steady hover
#define FT_BEACON_GUARD_MS 1500                // the result beacons after a disarm: past DSHOT_BEACON_GUARD_DELAY_US
#define FT_BEACON_GAP_MS   400
#define FT_ROT_MIN_HZ   40.0f
#define FT_RATE_MAX     60.0f               // smoothed |gyro|, deg/s
#define FT_THR_RANGE    0.06f               // throttle change within a frame
#define FT_FLOOR_HZ     25.0f
#define FT_PEAK_DB      6.0f                // a residual peak for the dynamic notch: over the side medians (+-FT_PEAK_REL)
#define FT_CAND_DB      2.5f                // a static notch candidate: low, but a wide bump can hold much power (the
                                            // share decides; a 4 dB gate dropped a 4.4 dB, 8 % bump in flight)
#define FT_PEAK_HYST    1.5f                // each half sum on its own: this much lower is still the same peak
#define FT_NOTCH_TAKE   0.85f               // static notch width: take this much of the bump's excess ...
#define FT_NOTCH_DRIFT  0.03f               // ... with the bump this far off its measured frequency
#define FT_NOTCH_KEEP_REL 0.03f             // a current static notch within this of the bump stays
#define FT_NOTCH_SHARE  3.0f                // static notch: the bump's excess holds at least this % of the 100..600 Hz power
#define FT_NOTCH_REF_HZ 400.0f              // ... at this frequency; lower down the notch costs more delay (1 / f)
#define FT_RPM_TARGET_DB 1.0f               // rpm lines: what may be left over the floor after the filter ...
#define FT_RPM_WEIGHT_DB 0.5f               // ... of it the weights take this much (rounded up), q the rest
#define FT_RPM_Q_MIN    250.0f
#define FT_RPM_Q_MAX    500.0f              // narrower only with evidence a hover cannot give (fast rpm changes)
#define FT_RPM_W_KEEP   3                   // a current weight up to this % over the need stays ...
#define FT_RPM_W_SHORT  2                   // ... or this % under it (the need moves by 1-2 % from run to run)
#define FT_RPM_Q_KEEP   0.1f                // a current q within this of the need stays
#define FT_FADE_CLEAR   0.95f               // min_hz + fade stays under this x the slowest motor's hover 1x
#define FT_LINE_REL     0.04f               // within this of a motor's rotation line (1x..5x)
#define FT_PEAK_REL     0.12f
#define FT_PEAK_SEP_REL 0.06f               // one peak per +-6 % (one notch takes it)
#define FT_PEAK_GUARD   3                   // bins on each side of a peak left out of its floor
#define FT_PEAK_MIN_HZ  100.0f
#define FT_PEAK_MAX_HZ  600.0f
#define FT_PEAKS        4
#define FT_IDEAL_SCORE  85
#define FT_SCORE_KEEP   5                   // dyn count: the current one stays within this of the line
#define FT_GYRO_SCALE   10.0f               // stored as deg/s x 10 in int16
#define FT_HIGH_HZ      600.0f              // the band above: what only lpf2 (and the D-term) take
#define FT_HIGH_TOP_REL 0.8f                // ... up to this x Nyquist (above, folded noise from the decimation)
#define FT_COMP_MAX     1000.0f             // compensation cap (+30 dB)
#define FT_HIGH_OFF_DB  3.0f                // lpf2 off when that band is at most this over the floor ...
#define FT_PEAK_HIGH_HZ 450.0f              // ... and no residual peak above this

enum { LPF2_BY_ORDERS = 0, LPF2_OFF, LPF2_KEPT_HIGH_BAND, LPF2_KEPT_HIGH_PEAK, LPF2_BY_DTERM, LPF2_NO_DATA = 255 };

bool filterAutotuneCollecting;

typedef struct {
    float hz, db;
    bool fixed;                 // in both sums, away from rotation lines: static notch candidate
    bool inMap;                 // within 8 % of a frame map mode
    float share;                // its excess over the floor, % of the 100..600 Hz power: what a notch there takes away
} ftPeak_t;

typedef struct {
    uint8_t w[3];
    uint16_t q;
    uint16_t fade;              // rpm_filter_fade_range_hz
    float trackErr;             // the notches' tracking error, a share of the line frequency
    uint8_t dyn;
    uint16_t lpf2;
    uint16_t notchHz, notchCut;
    uint8_t score;
    uint8_t badMotor;           // 0xFF none: the motor whose 1x stands out (prop balance)
    int nPeaks;
    ftPeak_t peaks[FT_PEAKS];
    int16_t highDb10;           // 600 Hz .. fs/2 after the RPM filter over the 100..600 Hz floor, dB x 10
    // the D-term filters with gyro lpf2 (decideDterm): the multiplier of the default pair, 0 = not decided
    uint8_t dMult10;
    uint16_t pdDelayUs, pdDelayRefUs;   // P+D output delay at FT_DT_REF_HZ: chosen, stock filters
    uint8_t dInPct, noisePct;   // D input and motor noise index against the stock filters, %
    uint16_t crossoverHz10;     // the measured loop crossover the delay counts at, 0.1 Hz (0 = no sweep)
    uint16_t idxMeas10, idxCur10;   // motor noise index: measured in the hover, the current chain's prediction, x 10
    uint16_t heatRef100, heatChosen100; // ripple heating of the stock chain and of the choice, 0.01 K
    uint8_t lpf2Basis;          // LPF2_BY_*
    bool valid;
} ftResult_t;

// the large buffers live in the bench scratch area, shared with the frame wizard (armed vs disarmed)
typedef struct {
    int16_t buf[3][FT_N];       // this frame: one signal, before or after the RPM filter
    float re[FT_M], im[FT_M];
    float spec[FT_BINS];
    float comp[FT_BINS];        // 1 / |H|^2 of the gyro path before the capture point: the spectra are of the raw gyro
    float welch[2][FT_BINS];    // after the RPM filter, alternate frames
    float welchPre[FT_BINS];    // before it: the high band and its floor, independent of the RPM notches
    uint8_t line[FT_BINS];
} ftBig_t;

STATIC_ASSERT(sizeof(ftBig_t) <= BENCH_SCRATCH_BYTES, ftBig_too_large);

static ftBig_t *ftb;

static bool scratchOwned(void)
{
    return ftb && benchScratchOwnedBy(BENCH_SCRATCH_FILTER_AUTOTUNE);
}

static struct {
    // filled by the gyro filter loop: one signal per frame, before / after the RPM filter by turns (RAM)
    volatile uint16_t pos;
    volatile bool full;
    uint8_t sig;                // this frame: 0 before the RPM filter, 1 after
    uint8_t decim, decimN;
    float accPre[3], accPost[3];  // the decimN loop samples averaged (anti-alias for the decimation)
    timeUs_t t0, t1;
    float rateLp, rateMax, rotMin, rotSum[FT_MOTORS];
    float thrMin, thrMax;       // from the task while filling
    // analysis of a full frame, one step per task call: real FFT as an FT_M point complex one
    int job, step;
    float compFs;
    // accumulated
    float sumX[FT_MOTORS][FT_ORDERS];
    uint16_t nX[FT_MOTORS][FT_ORDERS];
    float floor1[FT_MOTORS];    // the last frame before the RPM filter: 1x floor per motor
    float floorPre[FT_MOTORS][3];   // ... 1x..3x floors per motor
    float sumR[FT_MOTORS][3];   // after the RPM filter: 1x..3x line over that floor, dB, summed
    uint16_t nR[FT_MOTORS][3];
    bool haveFloor1;
    float sumR1;
    uint16_t nR1;
    uint16_t framesPre, framesPost, frames;
    float thrSum;
    uint16_t thrN;
    float vbatSum, noiseSum2;   // steady frames: battery voltage, the measured motor noise index squared (motor_noise.c)
    float rotFrameSum[FT_MOTORS];   // rotation of each motor in the frames before the RPM filter, summed
    float fs;
    // flow
    filterTuneState_e state;
    bool wasArmed, readyBeeped, saveMap, applied;
    uint16_t framesCarried;     // frames from earlier flights of this session (FILTER_TUNE_PARTIAL)
    uint8_t flightsCarried;
    // the result after a disarm with the mode on, by DShot beacons (any DShot FC, no buzzer needed)
    uint8_t beacon[3], beaconN, beaconI;
    timeMs_t beaconAt;
    ftResult_t result;
} ft;


// ---------------- frame map ----------------

void frameMapClear(void)
{
    memset(frameMapConfigMutable(), 0, sizeof(frameMapConfig_t));
    ft.saveMap = true;
}

// two entries merged: the tap part is unstable only while no stable tap entry is among them, the motor part is of
// one run only while no repeated motor entry is among them
static uint8_t mergeSrc(uint8_t a, uint8_t b)
{
    const bool stableTap = ((a & FRAME_MAP_SRC_TAP) && !(a & FRAME_MAP_SRC_UNSTABLE))
        || ((b & FRAME_MAP_SRC_TAP) && !(b & FRAME_MAP_SRC_UNSTABLE));
    const bool repeatedMotor = ((a & FRAME_MAP_SRC_MOTOR) && !(a & FRAME_MAP_SRC_MOTOR_ONCE))
        || ((b & FRAME_MAP_SRC_MOTOR) && !(b & FRAME_MAP_SRC_MOTOR_ONCE));
    uint8_t v = (a | b) & ~(FRAME_MAP_SRC_UNSTABLE | FRAME_MAP_SRC_MOTOR_ONCE);
    if ((v & FRAME_MAP_SRC_TAP) && !stableTap) {
        v |= FRAME_MAP_SRC_UNSTABLE;
    }
    if ((v & FRAME_MAP_SRC_MOTOR) && !repeatedMotor) {
        v |= FRAME_MAP_SRC_MOTOR_ONCE;
    }
    return v;
}

// an entry the filters can count on: a stable tap mode, or a motor mode two runs in a row saw
static bool srcStable(uint8_t s)
{
    return ((s & FRAME_MAP_SRC_TAP) && !(s & FRAME_MAP_SRC_UNSTABLE))
        || ((s & FRAME_MAP_SRC_MOTOR) && !(s & FRAME_MAP_SRC_MOTOR_ONCE));
}

void frameMapSetPart(uint8_t part, const float *hz, const bool *unstable, int n)
{
    frameMapConfig_t *map = frameMapConfigMutable();
    const uint8_t src = part == FRAME_MAP_PART_TAPS ? FRAME_MAP_SRC_TAP : FRAME_MAP_SRC_MOTOR;
    const uint8_t drop = part == FRAME_MAP_PART_TAPS ? FRAME_MAP_SRC_TAP | FRAME_MAP_SRC_UNSTABLE
        : FRAME_MAP_SRC_MOTOR | FRAME_MAP_SRC_MOTOR_ONCE;
    // the motor part: a mode the run before had too (within FRAME_MAP_REPEAT_REL: a wide mode lands a bin apart)
    // is repeated, else of this run only
    float prevMotor[FRAME_MAP_MODES];
    int nPrev = 0;
    for (int i = 0; i < map->count && part == FRAME_MAP_PART_MOTORS; i++) {
        if (map->modeSrc[i] & FRAME_MAP_SRC_MOTOR) {
            prevMotor[nPrev++] = map->modeHz10[i] / 10.0f;
        }
    }
    float f[FRAME_MAP_MODES * 3];
    uint8_t s[FRAME_MAP_MODES * 3];
    int k = 0;
    for (int i = 0; i < map->count && k < FRAME_MAP_MODES * 3; i++) {      // the other part stays
        const uint8_t keep = map->modeSrc[i] & ~drop;         // cable entries stay (their own bit)
        if (keep) {
            f[k] = map->modeHz10[i] / 10.0f;
            s[k++] = keep;
        }
    }
    for (int i = 0; i < n && k < FRAME_MAP_MODES * 3; i++) {
        f[k] = hz[i];
        uint8_t flag = unstable && unstable[i] ? FRAME_MAP_SRC_UNSTABLE : 0;
        if (part == FRAME_MAP_PART_MOTORS) {
            bool repeated = false;
            for (int j = 0; j < nPrev && !repeated; j++) {
                repeated = fabsf(prevMotor[j] - hz[i]) <= FRAME_MAP_REPEAT_REL * hz[i];
            }
            flag = repeated ? 0 : FRAME_MAP_SRC_MOTOR_ONCE;
        }
        s[k++] = src | flag;
    }
    // sort, merge within 3 %
    for (int i = 1; i < k; i++) {
        for (int j = i; j > 0 && f[j] < f[j - 1]; j--) {
            const float tf = f[j]; f[j] = f[j - 1]; f[j - 1] = tf;
            const uint8_t ts = s[j]; s[j] = s[j - 1]; s[j - 1] = ts;
        }
    }
    int m = 0;
    for (int i = 0; i < k; i++) {
        if (m > 0 && f[i] <= map->modeHz10[m - 1] / 10.0f * 1.03f) {
            map->modeSrc[m - 1] = mergeSrc(map->modeSrc[m - 1], s[i]);
            continue;
        }
        if (m >= FRAME_MAP_MODES) {
            break;
        }
        map->modeHz10[m] = lrintf(f[i] * 10.0f);
        map->modeSrc[m] = s[i];
        m++;
    }
    map->count = m;
    map->parts |= part;
    ft.saveMap = true;
}

void frameMapSetCable(const float *hz, int n)
{
    frameMapConfig_t *map = frameMapConfigMutable();
    int k = 0;                                        // drop the old cable-only entries, clear the bit elsewhere
    for (int i = 0; i < map->count; i++) {
        const uint8_t src = map->modeSrc[i] & ~FRAME_MAP_SRC_CABLE;
        if (src) {
            map->modeHz10[k] = map->modeHz10[i];
            map->modeSrc[k++] = src;
        }
    }
    map->count = k;
    for (int j = 0; j < n; j++) {
        // a cable mode is away from every craft mode (the wizard checked); keep the list sorted
        if (map->count >= FRAME_MAP_MODES) {
            break;
        }
        const uint16_t v = lrintf(hz[j] * 10.0f);
        int at = map->count;
        while (at > 0 && map->modeHz10[at - 1] > v) {
            map->modeHz10[at] = map->modeHz10[at - 1];
            map->modeSrc[at] = map->modeSrc[at - 1];
            at--;
        }
        map->modeHz10[at] = v;
        map->modeSrc[at] = FRAME_MAP_SRC_CABLE;
        map->count++;
    }
    ft.saveMap = true;
}

bool frameMapIsCable(float hz, float rel, float minHz)
{
    const frameMapConfig_t *map = frameMapConfig();
    for (int i = 0; i < map->count; i++) {
        if ((map->modeSrc[i] & FRAME_MAP_SRC_CABLE) && fabsf(map->modeHz10[i] / 10.0f - hz) <= MAX(rel * hz, minHz)) {
            return true;
        }
    }
    return false;
}

static bool mapReady(void)
{
    return frameMapConfig()->parts & FRAME_MAP_PART_TAPS;
}

static bool inMap(float hz)
{
    const frameMapConfig_t *map = frameMapConfig();
    for (int i = 0; i < map->count; i++) {
        if ((map->modeSrc[i] & (FRAME_MAP_SRC_TAP | FRAME_MAP_SRC_MOTOR)) && fabsf(map->modeHz10[i] / 10.0f - hz) <= 0.08f * hz) {
            return true;
        }
    }
    return false;
}

// the frame map has modes near hz (8 %) and none of them is stable (unstable taps, motor modes of one run only)
static bool onlyUnstableInMap(float hz)
{
    const frameMapConfig_t *map = frameMapConfig();
    bool any = false;
    for (int i = 0; i < map->count; i++) {
        const uint8_t s = map->modeSrc[i];
        if ((s & (FRAME_MAP_SRC_TAP | FRAME_MAP_SRC_MOTOR)) && fabsf(map->modeHz10[i] / 10.0f - hz) <= 0.08f * hz) {
            if (srcStable(s)) {
                return false;
            }
            any = true;
        }
    }
    return any;
}

// ---------------- capture (gyro filter loop) ----------------

static void resetFrame(void)
{
    ft.sig ^= 1;                                    // before / after the RPM filter by turns
    ft.pos = 0;
    ft.full = false;
    ft.rateMax = 0.0f;
    ft.rotMin = 1e9f;
    for (int m = 0; m < FT_MOTORS; m++) {
        ft.rotSum[m] = 0.0f;
    }
    ft.thrMin = 1e9f;
    ft.thrMax = -1e9f;
}

static inline int16_t toI16(float v)
{
    return (int16_t)constrainf(v * FT_GYRO_SCALE, -32767.0f, 32767.0f);
}

void filterAutotunePush(const float pre[3], const float post[3])
{
    if (ft.full) {
        return;
    }
    for (int a = 0; a < 3; a++) {
        ft.accPre[a] += pre[a];
        ft.accPost[a] += post[a];
    }
    if (++ft.decim < ft.decimN) {
        return;
    }
    const float inv = 1.0f / ft.decim;
    float avgPre[3], avgPost[3];
    for (int a = 0; a < 3; a++) {
        avgPre[a] = ft.accPre[a] * inv;
        avgPost[a] = ft.accPost[a] * inv;
        ft.accPre[a] = ft.accPost[a] = 0.0f;
    }
    pre = avgPre;
    post = avgPost;
    ft.decim = 0;
    const int p = ft.pos;
    if (p == 0) {
        ft.t0 = micros();
    }
    const float *x = ft.sig ? post : pre;
    float rate2 = 0.0f;
    for (int a = 0; a < 3; a++) {
        ftb->buf[a][p] = toI16(x[a]);
        rate2 += pre[a] * pre[a];
    }
    ft.rateLp += 0.02f * (sqrtf(rate2) - ft.rateLp);
    ft.rateMax = MAX(ft.rateMax, ft.rateLp);
    for (int m = 0; m < FT_MOTORS; m++) {
        const float r = getMotorFrequencyHz(m);
        ft.rotSum[m] += r;
        ft.rotMin = MIN(ft.rotMin, r);
    }
    if (p + 1 >= FT_N) {
        ft.t1 = micros();
        ft.full = true;
        ft.job = 0;
        ft.step = 0;
    }
    ft.pos = p + 1;
}

// ---------------- FFT, one step per call ----------------

static unsigned bitrev(unsigned x)
{
    unsigned r = 0;
    for (int i = 0; i < FT_LOG2M; i++) {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    return r;
}

// rotate the phasor (c, s) by (dc, ds)
#define ROTATE(c, s, dc, ds) do { const float cn_ = c * dc - s * ds; s = s * dc + c * ds; c = cn_; } while (0)

// job = axis; step 0 window + pack (even samples real, odd imaginary), 1..8 butterfly stages, 9 unpack + power.
// Returns true when the axis is done.
static bool fftStep(void)
{
    const int16_t *x = ftb->buf[ft.job];
    if (ft.step == 0) {
        float mean = 0.0f;
        for (int i = 0; i < FT_N; i++) {
            mean += x[i];
        }
        mean /= FT_N;
        const float dc = cosf(2.0f * M_PIf / FT_N), ds = sinf(2.0f * M_PIf / FT_N);
        float c = 1.0f, s = 0.0f;                     // Hann by a rotating phasor (no table: RAM)
        for (int m = 0; m < FT_M; m++) {
            const unsigned j = bitrev(m);
            ftb->re[j] = (x[2 * m] - mean) * (0.5f - 0.5f * c);
            ROTATE(c, s, dc, ds);
            ftb->im[j] = (x[2 * m + 1] - mean) * (0.5f - 0.5f * c);
            ROTATE(c, s, dc, ds);
        }
    } else if (ft.step <= FT_LOG2M) {
        const int half = 1 << (ft.step - 1), span = half << 1;
        const float dc = cosf(2.0f * M_PIf / span), ds = -sinf(2.0f * M_PIf / span);
        float c = 1.0f, s = 0.0f;                     // twiddle exp(-j 2 pi k / span)
        for (int k = 0; k < half; k++) {
            for (int a = k; a < FT_M; a += span) {
                const int b = a + half;
                const float tr = ftb->re[b] * c - ftb->im[b] * s;
                const float ti = ftb->re[b] * s + ftb->im[b] * c;
                ftb->re[b] = ftb->re[a] - tr;
                ftb->im[b] = ftb->im[a] - ti;
                ftb->re[a] += tr;
                ftb->im[a] += ti;
            }
            ROTATE(c, s, dc, ds);
        }
    } else {
        // X[k] = E[k] + W^k O[k], E = (Z[k] + Z*[M-k]) / 2, O = (Z[k] - Z*[M-k]) / 2j, W = exp(-j 2 pi / N)
        const float dc = cosf(2.0f * M_PIf / FT_N), ds = -sinf(2.0f * M_PIf / FT_N);
        float c = 1.0f, s = 0.0f;
        for (int k = 0; k < FT_BINS; k++) {
            const int a = k & (FT_M - 1), b = (FT_M - k) & (FT_M - 1);
            const float er = 0.5f * (ftb->re[a] + ftb->re[b]), ei = 0.5f * (ftb->im[a] - ftb->im[b]);
            const float or_ = 0.5f * (ftb->im[a] + ftb->im[b]), oi = -0.5f * (ftb->re[a] - ftb->re[b]);
            const float xr = er + c * or_ - s * oi, xi = ei + c * oi + s * or_;
            const float p = (xr * xr + xi * xi) * (1.0f / (FT_GYRO_SCALE * FT_GYRO_SCALE));
            ftb->spec[k] = ft.job == 0 ? p : ftb->spec[k] + p;
            ROTATE(c, s, dc, ds);
        }
        ft.step = 0;
        return true;
    }
    ft.step++;
    return false;
}

// ---------------- per-frame analysis ----------------

static float median(float *v, int n)
{
    for (int i = 1; i < n; i++) {
        for (int j = i; j > 0 && v[j] < v[j - 1]; j--) {
            const float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
        }
    }
    return n ? v[n / 2] : 0.0f;
}

static float pt1Power(float k, float w)
{
    const float a = 1.0f - k;
    return k * k / MAX(1.0f - 2.0f * a * cos_approx(w) + a * a, 1e-9f);
}

static float boxcarPower(int n, float x)          // n-sample average, x = pi f dt
{
    if (n <= 1 || fabsf(x) < 1e-6f) {
        return 1.0f;
    }
    const float h = sin_approx(n * x) / (n * sin_approx(x));
    return h * h;
}

// what the gyro went through before the capture point: the downsampling filter (gyro lpf2 at the gyro rate when
// it is on, else the average of the gyro samples of a loop) and the capture's own decimation average. The sensor's
// own lowpass stays in: the PID sees it too.
static void buildCompensation(float fs)
{
    const gyroConfig_t *g = gyroConfig();
    const float dtG = gyro.sampleLooptime * 1e-6f, dtP = gyro.targetLooptime * 1e-6f;
    const int nAvg = MAX(1, lrintf(dtP / MAX(dtG, 1e-6f)));
    const float fc = g->gyro_lpf2_static_hz;
    for (int b = 0; b < FT_BINS; b++) {
        const float f = b * fs / FT_N, wG = 2.0f * M_PIf * f * dtG;
        float h2 = 1.0f;
        if (gyro.downsampleFilterEnabled && fc > 0.0f) {
            switch (g->gyro_lpf2_type) {
            case FILTER_PT1:
                h2 = pt1Power(pt1FilterGain(fc, dtG), wG);
                break;
            case FILTER_PT2: {
                const float p = pt1Power(pt2FilterGain(fc, dtG), wG);
                h2 = p * p;
                break;
            }
            case FILTER_PT3: {
                const float p = pt1Power(pt3FilterGain(fc, dtG), wG);
                h2 = p * p * p;
                break;
            }
            default: {                                // SVF: second order, -3 dB at fc
                const float r = f / fc;
                h2 = 1.0f / (1.0f + r * r * r * r);
                break;
            }
            }
        } else {
            h2 = boxcarPower(nAvg, M_PIf * f * dtG);
        }
        h2 *= boxcarPower(ft.decimN, M_PIf * f * dtP);
        ftb->comp[b] = 1.0f / MAX(h2, 1.0f / FT_COMP_MAX);
    }
    ft.compFs = fs;
}

static void compensate(float fs)
{
    if (fabsf(fs - ft.compFs) > 0.01f * fs) {
        buildCompensation(fs);
    }
    for (int b = 0; b < FT_BINS; b++) {
        ftb->spec[b] *= ftb->comp[b];
    }
}

static float maxAround(int b)
{
    return MAX(MAX(ftb->spec[b - 1], ftb->spec[b]), ftb->spec[b + 1]);
}

// before the RPM filter: harmonic strength and the 1x floors
static void analysePre(const float *rot, float df)
{
    uint8_t *line = ftb->line;
    memset(line, 0, FT_BINS);
    for (int m = 0; m < FT_MOTORS; m++) {
        for (int h = 0; h < FT_ORDERS; h++) {
            const int b = lrintf((h + 1) * rot[m] / df);
            for (int i = MAX(b - 2, 0); i <= MIN(b + 2, FT_BINS - 1); i++) {
                line[i] = 1;
            }
        }
    }
    const int nb = lrintf(FT_FLOOR_HZ / df);
    float tmp[32];
    for (int m = 0; m < FT_MOTORS; m++) {
        for (int h = 0; h < FT_ORDERS; h++) {
            const int b = lrintf((h + 1) * rot[m] / df);
            if (b - nb < 1 || b + nb >= FT_BINS) {
                continue;
            }
            int n = 0;
            for (int i = b - nb; i <= b + nb && n < 32; i++) {
                if (!line[i]) {
                    tmp[n++] = ftb->spec[i];
                }
            }
            const float floor = n >= 4 ? median(tmp, n) : 0.0f;
            const float l = maxAround(b);
            if (floor <= 0.0f || l <= 0.0f) {
                continue;
            }
            ft.sumX[m][h] += 10.0f * log10f(l / floor);
            ft.nX[m][h]++;
            if (h == 0) {
                ft.floor1[m] = floor;
            }
            if (h < 3) {
                ft.floorPre[m][h] = floor;
            }
        }
    }
    for (int b = 0; b < FT_BINS; b++) {
        ftb->welchPre[b] += ftb->spec[b];
    }
    ft.haveFloor1 = true;
    ft.framesPre++;
}

// after the RPM filter: what is left at the 1x lines, and the spectrum for the residual peaks
static void analysePost(const float *rot, float df)
{
    for (int m = 0; m < FT_MOTORS && ft.haveFloor1; m++) {
        const int b = lrintf(rot[m] / df);
        const float r = b >= 1 && b + 1 < FT_BINS ? maxAround(b) : 0.0f;
        if (r > 0.0f && ft.floor1[m] > 0.0f) {
            ft.sumR1 += 10.0f * log10f(r / ft.floor1[m]);
            ft.nR1++;
        }
    }
    for (int m = 0; m < FT_MOTORS && ft.haveFloor1; m++) {
        for (int h = 0; h < 3; h++) {
            const int b = lrintf((h + 1) * rot[m] / df);
            const float r = b >= 1 && b + 1 < FT_BINS ? maxAround(b) : 0.0f;
            if (r > 0.0f && ft.floorPre[m][h] > 0.0f) {
                ft.sumR[m][h] += 10.0f * log10f(r / ft.floorPre[m][h]);
                ft.nR[m][h]++;
            }
        }
    }
    const int half = ft.framesPost & 1;
    for (int b = 0; b < FT_BINS; b++) {
        ftb->welch[half][b] += ftb->spec[b];
    }
    ft.framesPost++;
}

static void analyseFrame(void)
{
    const float df = ft.fs / FT_N;
    compensate(ft.fs);                                // both signals: the same path before the capture
    float rot[FT_MOTORS];
    for (int m = 0; m < FT_MOTORS; m++) {
        rot[m] = ft.rotSum[m] / FT_N;
    }
    if (ft.sig == 0) {
        analysePre(rot, df);
        for (int m = 0; m < FT_MOTORS; m++) {
            ft.rotFrameSum[m] += rot[m];
        }
    } else {
        analysePost(rot, df);
    }
    ft.frames = ft.framesPre + ft.framesPost;
}

// ---------------- decisions ----------------

static bool onLineOf(float f, float rot)
{
    for (int h = 1; h <= FT_ORDERS; h++) {
        if (fabsf(f - h * rot) <= FT_LINE_REL * f) {
            return true;
        }
    }
    return false;
}

// on a rotation line of any motor (1x..5x): the RPM filter's business, not a residual peak. Each motor on its own:
// on a frame with one slower motor its 1x sits several % under the mean rotation.
static bool onRotationLine(float f, const float *rotM)
{
    for (int m = 0; m < FT_MOTORS; m++) {
        if (onLineOf(f, rotM[m])) {
            return true;
        }
    }
    return false;
}

// the spectrum A (+ B when given), 3-bin smoothed: a mode a little wider than a bin, or one that wanders by a bin
// between the sums, reads the same (the two sums are added on the fly: no buffer, RAM)
static float smoothAt(const float *A, const float *B, int b)
{
#define P(i) (A[i] + (B ? B[i] : 0.0f))
    return 0.25f * P(b - 1) + 0.5f * P(b) + 0.25f * P(b + 1);
#undef P
}

// how far bin b stands over its neighbourhood (+-FT_PEAK_REL, the peak itself left out), dB: over the higher of the
// two side medians, so it has to rise over both sides (an RPM notch valley on one side does not make a peak).
// Medians, not the lowest bins: the lowest of a few noisy bins jumps by several dB from sum to sum.
static float peakDb(const float *A, const float *B, int b, float *refOut)
{
    const int span = MAX((int)lrintf(FT_PEAK_REL * b), FT_PEAK_GUARD + 5);    // f x REL / df = b x REL
    const int step = (span - FT_PEAK_GUARD) / 16 + 1;
    float ref = 0.0f;
    for (int side = -1; side <= 1; side += 2) {
        float v[16];
        int n = 0;
        for (int d = FT_PEAK_GUARD + 1; d <= span && n < 16; d += step) {
            const int i = b + side * d;
            if (i >= 1 && i < FT_BINS - 1) {
                v[n++] = smoothAt(A, B, i);
            }
        }
        if (n < 3) {
            return 0.0f;
        }
        ref = MAX(ref, median(v, n));
    }
    const float p = smoothAt(A, B, b);
    if (refOut) {
        *refOut = ref;
    }
    return ref > 0.0f && p > 0.0f ? 10.0f * log10f(p / ref) : 0.0f;
}

// the strongest bin within +-ds of b, by its peak height
static float bestPeakDbNear(const float *A, int b, int ds)
{
    float best = 0.0f;
    for (int i = b - ds; i <= b + ds; i++) {
        if (i >= 2 && i < FT_BINS - 2) {
            best = MAX(best, peakDb(A, NULL, i, NULL));
        }
    }
    return best;
}

static void findPeaks(const float *A, const float *B, float df, const float *rotM, ftPeak_t *out, int *nOut)
{
#define P(i) (A[i] + (B ? B[i] : 0.0f))
    int n = 0;
    const int bLo = MAX(2, (int)ceilf(FT_PEAK_MIN_HZ / df)), bHi = MIN(FT_BINS - 3, (int)(FT_PEAK_MAX_HZ / df));
    float total = 0.0f;
    for (int b = bLo; b <= bHi; b++) {
        total += P(b);
    }
    for (int b = bLo; b <= bHi; b++) {
        // one peak per +-FT_PEAK_SEP_REL (what one notch takes): the highest point there
        const int sep = MAX(2, (int)lrintf(FT_PEAK_SEP_REL * b));
        const float s = smoothAt(A, B, b);
        bool isMax = true;
        for (int i = MAX(1, b - sep); i <= MIN(FT_BINS - 2, b + sep) && isMax; i++) {
            const float si = smoothAt(A, B, i);
            isMax = i < b ? s > si : i > b ? s >= si : true;
        }
        if (!isMax || onRotationLine(b * df, rotM)) {
            continue;
        }
        float ref;
        const float db = peakDb(A, B, b, &ref);
        if (db < FT_CAND_DB) {
            continue;
        }
        // keep the FT_PEAKS strongest
        int at = n;
        if (n == FT_PEAKS) {
            int weakest = 0;
            for (int i = 1; i < n; i++) {
                if (out[i].db < out[weakest].db) {
                    weakest = i;
                }
            }
            if (out[weakest].db >= db) {
                continue;
            }
            at = weakest;
        } else {
            n++;
        }
        // the top by a parabola through the smoothed bins: the notch frequency does not step by a bin
        const float l = smoothAt(A, B, b - 1), r = smoothAt(A, B, b + 1), c = l - 2.0f * s + r;
        const float d = c < 0.0f ? constrainf(0.5f * (l - r) / c, -0.5f, 0.5f) : 0.0f;
        float excess = 0.0f;
        for (int i = MAX(1, b - sep); i <= MIN(FT_BINS - 2, b + sep); i++) {
            excess += MAX(P(i) - ref, 0.0f);
        }
        out[at].hz = (b + d) * df;
        out[at].db = db;
        out[at].share = total > 0.0f ? 100.0f * excess / total : 0.0f;
        out[at].fixed = false;
        out[at].inMap = false;
    }
    *nOut = n;
#undef P
}

// |H|^2 of a notch at f0 with quality q, at f
static float notchPower(float f, float f0, float q)
{
    const float u = f / f0, a = (1.0f - u * u) * (1.0f - u * u);
    return a / (a + (u / q) * (u / q));
}

// the share of a bump's excess over its floor that a notch at f0 with quality q takes away, the bump allowed to
// sit FT_NOTCH_DRIFT off (another flight, another battery): the bump is the run of bins over the floor around b
static float notchTake(const float *A, const float *B, int b, float ref, float f0, float q, float df)
{
#define P(i) (A[i] + (B ? B[i] : 0.0f))
    const int span = MAX(2, (int)lrintf(0.25f * b));
    int lo = b, hi = b;
    while (lo > MAX(1, b - span) && P(lo - 1) > ref) {
        lo--;
    }
    while (hi < MIN(FT_BINS - 2, b + span) && P(hi + 1) > ref) {
        hi++;
    }
    float total = 0.0f, taken = 0.0f;
    for (int i = lo; i <= hi; i++) {
        const float ex = P(i) - ref;
        if (ex <= 0.0f) {
            continue;
        }
        const float f = i * df;
        total += ex;
        taken += ex * (3.0f - notchPower(f, f0 * (1.0f - FT_NOTCH_DRIFT), q) - notchPower(f, f0, q)
                       - notchPower(f, f0 * (1.0f + FT_NOTCH_DRIFT), q)) * (1.0f / 3.0f);
    }
    return total > 0.0f ? taken / total : 0.0f;
#undef P
}

static float notchQ(float center, float cutoff)
{
    return center * cutoff / (center * center - cutoff * cutoff);
}

static float notchCutoff(float center, float q)
{
    return center * (sqrtf(1.0f + 4.0f * q * q) - 1.0f) / (2.0f * q);
}

// ---- RPM filter ----
// A line X (power over the floor) through a notch of weight w and quality q that misses it by delta (a share of
// the frequency: telemetry error and lag) keeps about (1 - w)^2 + w^2 (2 q delta)^2 of its excess. The residual
// after the filter tells delta, then q for the target follows in one step: absolute, not a ratchet.

static float fadeWeight(float f, float minHz, float fadeHz)
{
    return fadeHz > 0.0f ? constrainf((f - minHz) / fadeHz, 0.0f, 1.0f) : (f >= minHz ? 1.0f : 0.0f);
}

static float lineTrackError(float xDb, float rDb, float q, float w)
{
    const float x = powf(10.0f, xDb / 10.0f), r = powf(10.0f, rDb / 10.0f);
    const float left = MAX(r - 1.0f, 1e-3f) / MAX(x - 1.0f, 1e-3f) - (1.0f - w) * (1.0f - w);
    return sqrtf(MAX(left, 1e-6f)) / (2.0f * q * MAX(w, 1e-3f));
}

static uint8_t rpmWeightFor(float xDb)
{
    if (xDb < 3.0f) {
        return 0;
    }
    const float t = powf(10.0f, FT_RPM_WEIGHT_DB / 10.0f);
    const float w = 1.0f - sqrtf((t - 1.0f) / MAX(powf(10.0f, xDb / 10.0f) - 1.0f, 1e-3f));
    return (uint8_t)constrain((int)ceilf(100.0f * w - 1e-3f), 0, 100);
}

static void decideRpm(ftResult_t *r, float X[FT_MOTORS][FT_ORDERS], const float *rotM)
{
    const rpmFilterConfig_t *rc = rpmFilterConfig();
    const float minHz = rc->rpm_filter_min_hz, fadeCur = rc->rpm_filter_fade_range_hz, qCur = rc->rpm_filter_q / 100.0f;
    rpmFilterConfig_t def;                            // the proposal runs with the default min_hz (apply sets it)
    pgResetCopy(&def, PG_RPM_FILTER_CONFIG);
    const float minNew = def.rpm_filter_min_hz;
    // fade: the slowest motor's hover 1x clear of min_hz + fade (in the fade the 1x notch only works in part);
    // min_hz stays: lower notches would add delay at low throttle
    float rotLow = rotM[0];
    for (int m = 1; m < FT_MOTORS; m++) {
        rotLow = MIN(rotLow, rotM[m]);
    }
    const float fadeMax = FT_FADE_CLEAR * rotLow - minNew;
    r->fade = fadeCur <= fadeMax ? (uint16_t)fadeCur : (uint16_t)constrainf(fadeMax, 0.0f, fadeCur);
    // weights per order from the worst motor's line, for FT_RPM_WEIGHT_DB left
    for (int h = 0; h < 3; h++) {
        float x = 0.0f;
        for (int m = 0; m < FT_MOTORS; m++) {
            x = MAX(x, X[m][h]);
        }
        const uint8_t need = rpmWeightFor(x), cur = rc->rpm_filter_weights[h];
        r->w[h] = cur + FT_RPM_W_SHORT >= need && cur <= need + FT_RPM_W_KEEP ? cur : need;
    }
    // tracking error from the lines as they went through the current filter
    float err[FT_MOTORS * 3];
    int n = 0;
    for (int m = 0; m < FT_MOTORS; m++) {
        for (int h = 0; h < 3; h++) {
            const float w = rc->rpm_filter_weights[h] / 100.0f * fadeWeight((h + 1) * rotM[m], minHz, fadeCur);
            if (X[m][h] > 6.0f && w > 0.5f && ft.nR[m][h]) {
                err[n++] = lineTrackError(X[m][h], ft.sumR[m][h] / ft.nR[m][h], qCur, w);
            }
        }
    }
    if (n == 0) {
        r->q = rc->rpm_filter_q;
        r->trackErr = 0.0f;
        return;
    }
    const float delta = median(err, n);
    r->trackErr = delta;
    // q: the narrowest that keeps every line within the target with the new weights and fade
    const float t = powf(10.0f, FT_RPM_TARGET_DB / 10.0f);
    float q = FT_RPM_Q_MAX;
    for (int m = 0; m < FT_MOTORS; m++) {
        for (int h = 0; h < 3; h++) {
            const float w = r->w[h] / 100.0f * fadeWeight((h + 1) * rotM[m], minNew, r->fade);
            if (X[m][h] <= 6.0f || w <= 0.5f) {
                continue;
            }
            const float allowed = (t - 1.0f) / MAX(powf(10.0f, X[m][h] / 10.0f) - 1.0f, 1e-3f) - (1.0f - w) * (1.0f - w);
            if (allowed > 0.0f) {                     // else the weight alone leaves the line over: q cannot help
                q = MIN(q, sqrtf(allowed) / (2.0f * delta * w) * 100.0f);
            }
        }
    }
    q = constrainf(q, FT_RPM_Q_MIN, FT_RPM_Q_MAX);
    r->q = fabsf(q - rc->rpm_filter_q) <= FT_RPM_Q_KEEP * rc->rpm_filter_q ? rc->rpm_filter_q
                                                                            : (uint16_t)(lrintf(q / 10.0f) * 10);
}

// ---------------- the loop crossover, by a short sweep ----------------
// The P+D delay counts at the craft's own loop crossover, which a passive hover cannot give (the craft hardly moves,
// thrust -> rotation drowns in vibration). Once the noise frames are in, in ANGLE with the sticks centred, a small
// sweep (FT_SWEEP_ANGLE_DEG at FT_SWEEP_F0, less above, FT_SWEEP_F0..F1 Hz, nothing slow) goes on the rate setpoint of roll,
// then pitch. With D on the gyro (BF), the gyro the PID sees answers the setpoint as T = Cr Q / (1 + C Q), Cr = P + I
// (setpoint path), C = Cr + D (feedback), Q = the craft with the gyro filters: Q = T / (Cr - T C), L = C Q, crossover
// where |L| = 1. Sticks, throttle, attitude, rate, mode or arming off its limits ends the sweep at once.

static const float ftIdHz[] = { 10, 14, 20, 28, 40, 56, 80, 112 };
#define FT_ID_N            ((int)ARRAYLEN(ftIdHz))
#define FT_SWEEP_F0        8.0f
#define FT_SWEEP_F1        125.0f
#define FT_SWEEP_S         6.0f
#define FT_SWEEP_RAMP_S    0.3f
#define FT_SWEEP_SETTLE_S  1.0f
#define FT_SWEEP_ANGLE_DEG 0.5f              // the swing at FT_SWEEP_F0, less above (amplitude / 2 pi f). Host, closed-loop model
                                              // with the noisiest 5" hover: crossover within 3.5 % at 0.1 deg, 1.3 % at 0.5,
                                              // 1 % at 1 (the grid interpolation left)
#define FT_SWEEP_STICK     0.1f               // stick deflection that ends it
#define FT_SWEEP_THR       0.08f              // throttle change that ends it
#define FT_SWEEP_TILT_DD   150                // attitude change that ends it, 0.1 deg
#define FT_SWEEP_RATE_MAX  300.0f             // deg/s

enum { SWEEP_NONE = 0, SWEEP_ROLL, SWEEP_SETTLE, SWEEP_PITCH, SWEEP_DONE, SWEEP_ABORTED };
enum { SWEEP_END_NONE = 0, SWEEP_END_STICKS, SWEEP_END_THROTTLE, SWEEP_END_ATTITUDE, SWEEP_END_RATE, SWEEP_END_MODE };

static struct {
    volatile uint8_t phase;
    uint8_t endReason;
    bool tried;
    float t, ph, r, amp;
    float pr[FT_ID_N], pi[FT_ID_N], dr[FT_ID_N], di[FT_ID_N];
    float rr[FT_ID_N], ri[FT_ID_N], yr[FT_ID_N], yi[FT_ID_N];
    float fNow;                             // the sweep's frequency now
    float fc[2], pmDeg[2];                  // per axis, 0 = not found
    int16_t att0[2];
    float thr0;
} ftx;

typedef struct { float re, im; } ftCplx_t;

static ftCplx_t cmul(ftCplx_t a, ftCplx_t b)
{
    return (ftCplx_t){ a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re };
}

static ftCplx_t cdiv(ftCplx_t a, ftCplx_t b)
{
    const float d = MAX(b.re * b.re + b.im * b.im, 1e-20f);
    return (ftCplx_t){ (a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d };
}

static float cabs2(ftCplx_t a)
{
    return a.re * a.re + a.im * a.im;
}

static ftCplx_t pt1Cplx(float hz, float dt, float w)  // discrete PT1 at w = 2 pi f dt; hz 0 = none
{
    if (hz <= 0.0f) {
        return (ftCplx_t){ 1.0f, 0.0f };
    }
    const float k = pt1FilterGain(hz, dt), a = 1.0f - k;
    const float dr = 1.0f - a * cos_approx(w), di = a * sin_approx(w), d2 = dr * dr + di * di;
    return (ftCplx_t){ k * dr / d2, -k * di / d2 };
}

static float hoverThrottle(void)
{
    return ft.thrN ? constrainf(ft.thrSum / ft.thrN, 0.0f, 1.0f) : 0.3f;
}

// the D-term lowpass pair of a profile at the hover throttle (PT1 assumed, as the autotune sets)
static ftCplx_t dtermCplx(float dynMin, float dynMax, float stat, float lpf2, float dtP, float w)
{
    const float c1 = dynMin > 0.0f ? dynLpfCutoffFreq(hoverThrottle(), dynMin, dynMax, currentPidProfile->dterm_lpf1_dyn_expo) : stat;
    return cmul(pt1Cplx(c1, dtP, w), pt1Cplx(lpf2, dtP, w));
}

// each grid frequency is summed only while the sweep is within half a grid step of it: its dwell, not the whole sweep's
// noise (host: 2.5x less crossover error at the same amplitude)
static bool sweepGate(int j)
{
    const float half = sqrtf(ftIdHz[1] / ftIdHz[0]);
    return ftx.fNow >= ftIdHz[j] / half && ftx.fNow < ftIdHz[j] * half;
}

static void sweepStartAxis(uint8_t phase)
{
    const float dt = pidGetDT();
    ftx.t = ftx.ph = ftx.r = 0.0f;
    for (int j = 0; j < FT_ID_N; j++) {
        const float w = 2.0f * M_PIf * ftIdHz[j] * dt;
        ftx.pr[j] = 1.0f; ftx.pi[j] = 0.0f;
        ftx.dr[j] = cosf(w); ftx.di[j] = -sinf(w);      // e^{-j w}: the phasor of the DFT
        ftx.rr[j] = ftx.ri[j] = ftx.yr[j] = ftx.yi[j] = 0.0f;
    }
    ftx.phase = phase;
    filterAutotuneSweepOn = true;
}

static void sweepCrossover(int axis)
{
    const pidProfile_t *pp = currentPidProfile;
    const float kp = PTERM_SCALE * pp->pid[axis].P, ki = ITERM_SCALE * pp->pid[axis].I, kd = DTERM_SCALE * pp->pid[axis].D;
    const float dtP = pidGetDT(), fp = 1.0f / dtP;
    float prevMag = 0.0f, prevPh = 0.0f;
    ftx.fc[axis] = ftx.pmDeg[axis] = 0.0f;
    for (int j = 0; j < FT_ID_N; j++) {
        if (ftx.rr[j] * ftx.rr[j] + ftx.ri[j] * ftx.ri[j] <= 1e-6f) {
            continue;
        }
        const float w = 2.0f * M_PIf * ftIdHz[j], wd = w * dtP;
        const ftCplx_t t = cdiv((ftCplx_t){ ftx.yr[j], ftx.yi[j] }, (ftCplx_t){ ftx.rr[j], ftx.ri[j] });
        const ftCplx_t cr = { kp, -ki / w };
        const ftCplx_t der = { fp * (1.0f - cos_approx(wd)), fp * sin_approx(wd) };
        const ftCplx_t dd = cmul(der, dtermCplx(pp->dterm_lpf1_dyn_min_hz, pp->dterm_lpf1_dyn_max_hz, pp->dterm_lpf1_static_hz,
            pp->dterm_lpf2_static_hz, dtP, wd));
        const ftCplx_t c = { cr.re + kd * dd.re, cr.im + kd * dd.im };
        const ftCplx_t tc = cmul(t, c);
        const ftCplx_t q = cdiv(t, (ftCplx_t){ cr.re - tc.re, cr.im - tc.im });
        const ftCplx_t l = cmul(c, q);
        const float mag = sqrtf(cabs2(l)), ph = atan2_approx(l.im, l.re);
        if (j > 0 && prevMag >= 1.0f && mag < 1.0f) {   // the first crossing down: log interpolation
            const float x = logf(prevMag) / (logf(prevMag) - logf(MAX(mag, 1e-6f)));
            ftx.fc[axis] = ftIdHz[j - 1] * powf(ftIdHz[j] / ftIdHz[j - 1], x);
            ftx.pmDeg[axis] = 180.0f + RADIANS_TO_DEGREES(prevPh + x * (ph - prevPh));
            break;
        }
        prevMag = mag;
        prevPh = ph;
    }
}

volatile bool filterAutotuneSweepOn;

NOINLINE float filterAutotuneExcitation(int axis, float gyroFiltered)
{
    const uint8_t phase = ftx.phase;
    if (phase != SWEEP_ROLL && phase != SWEEP_PITCH && phase != SWEEP_SETTLE) {
        return 0.0f;
    }
    if (axis == FD_ROLL) {                            // once a loop: the next sample of the sweep
        const float dt = pidGetDT();
        ftx.t += dt;
        if (phase == SWEEP_SETTLE) {
            if (ftx.t >= FT_SWEEP_SETTLE_S) {
                sweepStartAxis(SWEEP_PITCH);
            }
            return 0.0f;
        }
        if (ftx.t >= FT_SWEEP_S) {
            const int a = phase == SWEEP_PITCH ? FD_PITCH : FD_ROLL;
            sweepCrossover(a);
            if (phase == SWEEP_ROLL) {
                ftx.t = 0.0f;
                ftx.phase = SWEEP_SETTLE;
            } else {
                ftx.phase = SWEEP_DONE;
                filterAutotuneSweepOn = false;
            }
            ftx.r = 0.0f;
            return 0.0f;
        }
        for (int j = 0; j < FT_ID_N; j++) {           // phasors to this sample
            const float pr = ftx.pr[j] * ftx.dr[j] - ftx.pi[j] * ftx.di[j];
            ftx.pi[j] = ftx.pr[j] * ftx.di[j] + ftx.pi[j] * ftx.dr[j];
            ftx.pr[j] = pr;
        }
        const float k = logf(FT_SWEEP_F1 / FT_SWEEP_F0) / FT_SWEEP_S;
        ftx.ph += 2.0f * M_PIf * FT_SWEEP_F0 * expf(k * ftx.t) * dt;
        if (ftx.ph > M_PIf) {
            ftx.ph -= 2.0f * M_PIf;
        }
        const float taper = MIN(1.0f, MIN(ftx.t, FT_SWEEP_S - ftx.t) / FT_SWEEP_RAMP_S);
        ftx.r = ftx.amp * taper * sin_approx(ftx.ph);
        ftx.fNow = FT_SWEEP_F0 * expf(k * ftx.t);
        for (int j = 0; j < FT_ID_N; j++) {
            if (sweepGate(j)) {
                ftx.rr[j] += ftx.r * ftx.pr[j];
                ftx.ri[j] += ftx.r * ftx.pi[j];
            }
        }
    }
    const int active = phase == SWEEP_PITCH ? FD_PITCH : FD_ROLL;
    if (phase == SWEEP_SETTLE || axis != active) {
        return 0.0f;
    }
    for (int j = 0; j < FT_ID_N; j++) {
        if (sweepGate(j)) {
            ftx.yr[j] += gyroFiltered * ftx.pr[j];
            ftx.yi[j] += gyroFiltered * ftx.pi[j];
        }
    }
    return ftx.r;
}

static bool sweepSticksCentred(void)
{
    return getRcDeflectionAbs(FD_ROLL) < FT_SWEEP_STICK && getRcDeflectionAbs(FD_PITCH) < FT_SWEEP_STICK
        && getRcDeflectionAbs(FD_YAW) < FT_SWEEP_STICK;
}

// the task: start the sweep once the frames are in (ANGLE, sticks centred), end it on anything off its limits
static void sweepService(bool modeOn)
{
    const uint8_t phase = ftx.phase;
    if (phase == SWEEP_ROLL || phase == SWEEP_SETTLE || phase == SWEEP_PITCH) {
        uint8_t end = SWEEP_END_NONE;
        if (!ARMING_FLAG(ARMED) || !modeOn || !FLIGHT_MODE(ANGLE_MODE) || failsafeIsActive()) {
            end = SWEEP_END_MODE;
        } else if (!sweepSticksCentred()) {
            end = SWEEP_END_STICKS;
        } else if (fabsf(rcCommand[THROTTLE] - ftx.thr0) > FT_SWEEP_THR) {
            end = SWEEP_END_THROTTLE;
        } else if (ABS(attitude.values.roll - ftx.att0[0]) > FT_SWEEP_TILT_DD || ABS(attitude.values.pitch - ftx.att0[1]) > FT_SWEEP_TILT_DD) {
            end = SWEEP_END_ATTITUDE;
        } else if (fabsf(gyro.gyroADCf[FD_ROLL]) > FT_SWEEP_RATE_MAX || fabsf(gyro.gyroADCf[FD_PITCH]) > FT_SWEEP_RATE_MAX) {
            end = SWEEP_END_RATE;
        }
        if (end != SWEEP_END_NONE) {
            ftx.phase = SWEEP_ABORTED;                // the PID loop sees it at once: no more offset
            ftx.endReason = end;
            filterAutotuneSweepOn = false;
        }
    }
}

static bool sweepTryStart(void)
{
    if (ftx.tried || !FLIGHT_MODE(ANGLE_MODE) || !sweepSticksCentred()) {
        return false;
    }
    ftx.tried = true;
    ftx.amp = FT_SWEEP_ANGLE_DEG * 2.0f * M_PIf * FT_SWEEP_F0;   // deg/s
    ftx.att0[0] = attitude.values.roll;
    ftx.att0[1] = attitude.values.pitch;
    ftx.thr0 = rcCommand[THROTTLE];
    ftx.endReason = SWEEP_END_NONE;
    sweepStartAxis(SWEEP_ROLL);
    return true;
}

static bool sweepActive(void)
{
    return ftx.phase == SWEEP_ROLL || ftx.phase == SWEEP_SETTLE || ftx.phase == SWEEP_PITCH;
}

// the measured crossover: the mean of the axes that gave one, 0 = none
static float sweepCrossoverHz(void)
{
    const int n = (ftx.fc[0] > 0.0f) + (ftx.fc[1] > 0.0f);
    return n ? (ftx.fc[0] + ftx.fc[1]) / n : 0.0f;
}

// ---------------- D-term filters with gyro lpf2 ----------------
// research/motor-thermal/dterm: without gyro lowpass the motor noise grows mostly through P, which the D filters do
// not reach. Searched on the hover spectrum after the RPM filter: gyro lpf2 PT1 of ftDtLpf2 x the D pair (dyn PT1
// 75..150 + PT1 150) times FT_DT_M_MIN..FT_DT_M_MAX / 10. Kept: the motor heating from the command ripple (motor_noise.c
// index -> motor_thermal.c) at most filter_tune_heat_budget over the stock chain's; chosen: the least P+D output delay
// at the measured loop crossover. The index is absolute: the hover spectrum as a PSD through the PID, plus what the
// filters do not make (the measured index over the current chain's prediction). No crossover (no sweep) or no motor
// model: D unchanged, lpf2 as decided before.

static const uint16_t ftDtLpf2[] = { 0, 1000, 800, 650, 500 };
#define FT_DT_M_MIN      5
#define FT_DT_M_MAX      15
#define FT_DT_EXPO       5                  // dynamic lowpass curve expo (BF default)

static ftCplx_t notchCplx(float f, uint16_t hz, uint16_t cut)
{
    if (!hz || cut >= hz) {
        return (ftCplx_t){ 1.0f, 0.0f };
    }
    const float q = (float)hz * cut / ((float)hz * hz - (float)cut * cut);
    const float x = (float)hz * hz - f * f, y = f * hz / q;    // (w0^2 - w^2) / (w0^2 - w^2 + j w w0 / Q)
    const float d2 = MAX(x * x + y * y, 1e-12f);
    return (ftCplx_t){ x * x / d2, -x * y / d2 };
}

// the gyro path to the PID: gyro lpf2 PT1 at the gyro rate, or (off) the average of the samples of a loop
static ftCplx_t gyroLpf2Cplx(float f, uint16_t hz, float dtG, int nAvg)
{
    const float wG = 2.0f * M_PIf * f * dtG;
    if (hz) {
        return pt1Cplx(hz, dtG, wG);
    }
    const float mag = sqrtf(boxcarPower(nAvg, 0.5f * wG)), ph = -0.5f * wG * (nAvg - 1);
    return (ftCplx_t){ mag * cos_approx(ph), mag * sin_approx(ph) };
}

static void decideDterm(ftResult_t *r, float df)
{
#if defined(USE_MOTOR_NOISE) && defined(USE_MOTOR_THERMAL)
    const pidProfile_t *pp = currentPidProfile;
    const float kp = PTERM_SCALE * 0.5f * (pp->pid[PID_ROLL].P + pp->pid[PID_PITCH].P);
    const float kd = DTERM_SCALE * 0.5f * (pp->pid[PID_ROLL].D + pp->pid[PID_PITCH].D);
    const float dtG = gyro.sampleLooptime * 1e-6f, dtP = gyro.targetLooptime * 1e-6f;
    const float f0 = sweepCrossoverHz();
    r->crossoverHz10 = lrintf(f0 * 10.0f);
    if (kd <= 0.0f || dtG <= 0.0f || dtP <= 0.0f || ft.thrN == 0 || ft.framesPost == 0 || f0 <= 0.0f) {
        return;
    }
    const int nAvg = MAX(1, lrintf(dtP / dtG));
    const float fp = 1.0f / dtP, thr = hoverThrottle();
    const gyroConfig_t *gc = gyroConfig();
    const float gLpf1Ref = dynLpfCutoffFreq(thr, GYRO_LPF1_DYN_MIN_HZ_DEFAULT, GYRO_LPF1_DYN_MAX_HZ_DEFAULT, FT_DT_EXPO);
    const float gLpf1Cur = gc->gyro_lpf1_dyn_min_hz ? dynLpfCutoffFreq(thr, gc->gyro_lpf1_dyn_min_hz, gc->gyro_lpf1_dyn_max_hz, FT_DT_EXPO)
        : gc->gyro_lpf1_static_hz;
    const float dLpf1Def = dynLpfCutoffFreq(thr, DTERM_LPF1_DYN_MIN_HZ_DEFAULT, DTERM_LPF1_DYN_MAX_HZ_DEFAULT, FT_DT_EXPO);
    // the hover PSD of one axis: the welch sums are |X|^2 of a Hann window, three axes summed, over framesPost frames
    const float psdScale = 2.0f / (ft.fs * FT_N * 0.375f * ft.framesPost * 3.0f);
    float mixK = 0.0f;                                // the mean squared roll / pitch mix of a motor
    for (int m = 0; m < FT_MOTORS; m++) {
        mixK += sq(mixerRuntime.currentMixer[m].roll) + sq(mixerRuntime.currentMixer[m].pitch);
    }
    mixK /= FT_MOTORS;                                // idx^2 = mean(mix_roll^2 + mix_pitch^2) x the per-axis variance
    enum { NK = ARRAYLEN(ftDtLpf2), NM = FT_DT_M_MAX - FT_DT_M_MIN + 1 };
    float var[NK][NM], dIn[NK][NM], varRef = 0.0f, dInRef = 0.0f, varCur = 0.0f;
    memset(var, 0, sizeof(var));
    memset(dIn, 0, sizeof(dIn));
    for (int b = 1; b < FT_BINS; b++) {
        const float f = b * df, wP = 2.0f * M_PIf * f * dtP, wG = 2.0f * M_PIf * f * dtG;
        const float psd = (ftb->welch[0][b] + ftb->welch[1][b]) * psdScale * df;
        const float s = psd * cabs2(notchCplx(f, r->notchHz, r->notchCut));
        const ftCplx_t lp80 = pt1Cplx(MOTOR_NOISE_HP0_HZ, dtP, wP);
        const float hp2 = cabs2((ftCplx_t){ 1.0f - lp80.re, -lp80.im });
        const ftCplx_t der = { fp * (1.0f - cos_approx(wP)), fp * sin_approx(wP) };
        // the stock chain: gyro lpf1 dyn + lpf2 500, the D pair x1; the current one as it flew
        const float gRef = cabs2(pt1Cplx(gLpf1Ref, dtP, wP)) * cabs2(pt1Cplx(GYRO_LPF2_HZ_DEFAULT, dtG, wG));
        const float gCur = cabs2(pt1Cplx(gLpf1Cur, dtP, wP)) * cabs2(gyroLpf2Cplx(f, gc->gyro_lpf2_static_hz, dtG, nAvg))
            * cabs2(notchCplx(f, gc->gyro_soft_notch_hz_1, gc->gyro_soft_notch_cutoff_1))
            * cabs2(notchCplx(f, gc->gyro_soft_notch_hz_2, gc->gyro_soft_notch_cutoff_2));
        const ftCplx_t dCur = cmul(der, dtermCplx(pp->dterm_lpf1_dyn_min_hz, pp->dterm_lpf1_dyn_max_hz, pp->dterm_lpf1_static_hz,
            pp->dterm_lpf2_static_hz, dtP, wP));
        varCur += hp2 * cabs2((ftCplx_t){ kp + kd * dCur.re, kd * dCur.im }) * gCur * psd;
        for (int m = FT_DT_M_MIN; m <= FT_DT_M_MAX; m++) {
            const float mm = m * 0.1f;
            const ftCplx_t dpath = cmul(der, cmul(pt1Cplx(dLpf1Def * mm, dtP, wP), pt1Cplx(DTERM_LPF2_HZ_DEFAULT * mm, dtP, wP)));
            const float pd2 = cabs2((ftCplx_t){ kp + kd * dpath.re, kd * dpath.im });
            const float d2 = cabs2(dpath);
            if (m == 10) {
                varRef += hp2 * pd2 * gRef * s;
                dInRef += d2 * gRef * s;
            }
            for (int k = 0; k < NK; k++) {
                const float g = cabs2(gyroLpf2Cplx(f, ftDtLpf2[k], dtG, nAvg)) * s;
                var[k][m - FT_DT_M_MIN] += hp2 * pd2 * g;
                dIn[k][m - FT_DT_M_MIN] += d2 * g;
            }
        }
    }
    if (varRef <= 0.0f || dInRef <= 0.0f) {
        return;
    }
    // absolute: what the filters do not make is the measured index over the current chain's prediction
    const float idxMeas2 = ft.noiseSum2 / ft.thrN, idxCur2 = mixK * varCur;
    const float other2 = MAX(idxMeas2 - idxCur2, 0.0f);
    float rpm = 0.0f;
    for (int m = 0; m < FT_MOTORS; m++) {
        rpm += ft.framesPre ? ft.rotFrameSum[m] / ft.framesPre * 60.0f / FT_MOTORS : 0.0f;
    }
    const float vbat = ft.vbatSum / ft.thrN;
    const float heatRef = motorThermalRippleRise(sqrtf(mixK * varRef + other2), rpm, vbat);
    const float budget = filterTuneConfig()->heat_budget_dk * 0.1f;
    r->idxMeas10 = (uint16_t)constrainf(sqrtf(idxMeas2) * 10.0f, 0.0f, 65535.0f);
    r->idxCur10 = (uint16_t)constrainf(sqrtf(idxCur2) * 10.0f, 0.0f, 65535.0f);
    r->heatRef100 = (uint16_t)constrainf(heatRef * 100.0f, 0.0f, 65535.0f);
    // P+D output delay at the crossover against the unfiltered Kp + Kd s
    const float w0 = 2.0f * M_PIf * f0 * dtP;
    const ftCplx_t der0 = { fp * (1.0f - cos_approx(w0)), fp * sin_approx(w0) };
    const float phIdeal = atan2_approx(kd * 2.0f * M_PIf * f0, kp);
    float best = 1e9f, delayRef = 0.0f, bestHeat = 0.0f;
    int bk = -1, bm = -1;
    for (int k = -1; k < (int)NK; k++) {              // -1: the stock chain, for the reference delay
        for (int m = FT_DT_M_MIN; m <= FT_DT_M_MAX; m++) {
            if (k < 0 && m != 10) {
                continue;
            }
            const float mm = m * 0.1f;
            const ftCplx_t hd = cmul(pt1Cplx(dLpf1Def * mm, dtP, w0), pt1Cplx(DTERM_LPF2_HZ_DEFAULT * mm, dtP, w0));
            const ftCplx_t dp = cmul(der0, hd);
            ftCplx_t g = k < 0 ? cmul(pt1Cplx(gLpf1Ref, dtP, w0), pt1Cplx(GYRO_LPF2_HZ_DEFAULT, dtG, 2.0f * M_PIf * f0 * dtG))
                : gyroLpf2Cplx(f0, ftDtLpf2[k], dtG, nAvg);
            g = cmul(g, notchCplx(f0, r->notchHz, r->notchCut));
            const ftCplx_t out = cmul(g, (ftCplx_t){ kp + kd * dp.re, kd * dp.im });
            const float delay = (phIdeal - atan2_approx(out.im, out.re)) / (2.0f * M_PIf * f0);
            if (k < 0) {
                delayRef = delay;
                continue;
            }
            const int i = m - FT_DT_M_MIN;
            const float heat = motorThermalRippleRise(sqrtf(mixK * var[k][i] + other2), rpm, vbat);
            if (heat <= heatRef + budget && delay < best) {
                best = delay;
                bk = k;
                bm = m;
                bestHeat = heat;
            }
        }
    }
    r->pdDelayRefUs = (uint16_t)constrainf(delayRef * 1e6f, 0.0f, 65535.0f);
    if (bk < 0) {
        return;                                       // nothing within the heat budget: D unchanged, lpf2 as decided
    }
    r->dMult10 = bm;
    r->lpf2 = ftDtLpf2[bk];
    r->lpf2Basis = LPF2_BY_DTERM;
    r->pdDelayUs = (uint16_t)constrainf(best * 1e6f, 0.0f, 65535.0f);
    r->heatChosen100 = (uint16_t)constrainf(bestHeat * 100.0f, 0.0f, 65535.0f);
    r->dInPct = (uint8_t)constrainf(100.0f * sqrtf(dIn[bk][bm - FT_DT_M_MIN] / dInRef), 0.0f, 255.0f);
    r->noisePct = (uint8_t)constrainf(100.0f * sqrtf(var[bk][bm - FT_DT_M_MIN] / varRef), 0.0f, 255.0f);
#else
    UNUSED(r);
    UNUSED(df);
#endif
}

static void decide(ftResult_t *r)
{
    memset(r, 0, sizeof(*r));
    r->badMotor = 0xFF;
    r->lpf2Basis = LPF2_NO_DATA;
    if (ft.framesPre == 0 || ft.framesPost == 0 || ft.fs <= 0.0f || !scratchOwned()) {
        return;                                       // no data (or the frame wizard has the buffers now)
    }
    float X[FT_MOTORS][FT_ORDERS];
    float rotM[FT_MOTORS], rotMean = 0.0f;
    for (int m = 0; m < FT_MOTORS; m++) {
        rotM[m] = ft.rotFrameSum[m] / ft.framesPre;
        rotMean += rotM[m] / FT_MOTORS;
    }
    for (int m = 0; m < FT_MOTORS; m++) {
        for (int h = 0; h < FT_ORDERS; h++) {
            X[m][h] = ft.nX[m][h] ? ft.sumX[m][h] / ft.nX[m][h] : 0.0f;
        }
    }
    decideRpm(r, X, rotM);

    // residual peaks: from both sums together, then the same place in each sum on its own, a little lower allowed
    // (hysteresis: a peak right at the threshold does not come and go with the frame order)
    const float df = ft.fs / FT_N;
    findPeaks(ftb->welch[0], ftb->welch[1], df, rotM, r->peaks, &r->nPeaks);
    // static notch: the off-line bump, there in both sums, that a notch takes the most from for its delay. The
    // benefit is the power its excess holds; the delay at the control frequencies goes as 1 / f for a fixed notch
    // shape (cutoff 0.88 f), so the merit is share x f. On the 5" logs the 380 Hz bump held 5..17 % of the band and
    // a notch there took 4..7 dB off 350..450 Hz for 0.1 ms; a 200 Hz bump with 2..6 % never beats it.
    int best = -1;
    for (int i = 0; i < r->nPeaks; i++) {
        ftPeak_t *p = &r->peaks[i];
        p->inMap = inMap(p->hz);
        const int b = lrintf(p->hz / df);
        // not on an unstable tap mode (the FC on its mount drifts with temperature): that one is the dynamic notch's
        p->fixed = p->share * p->hz >= FT_NOTCH_SHARE * FT_NOTCH_REF_HZ && !onlyUnstableInMap(p->hz)
            && bestPeakDbNear(ftb->welch[0], b, 2) >= FT_CAND_DB - FT_PEAK_HYST
            && bestPeakDbNear(ftb->welch[1], b, 2) >= FT_CAND_DB - FT_PEAK_HYST;
        if (p->fixed && (best < 0 || p->share * p->hz > r->peaks[best].share * r->peaks[best].hz)) {
            best = i;
        }
    }
    // what is left for the dynamic notch: the clear peaks besides the notched one
    int left = 0;
    bool highPeak = false;
    for (int i = 0; i < r->nPeaks; i++) {
        if (i != best && r->peaks[i].db >= FT_PEAK_DB) {
            left++;
            highPeak |= r->peaks[i].hz > FT_PEAK_HIGH_HZ;
        }
    }
    if (best >= 0) {
        // width: the narrowest notch (its delay at the control frequencies goes as 1 / q) that still takes
        // FT_NOTCH_TAKE of the bump's excess with the bump FT_NOTCH_DRIFT off. On the 5" hover the 380 Hz bump was
        // 6 % wide at half power: q 4 took 88 % for 0.11 ms, q 3 92 % for 0.15 ms, q 6 80 % for 0.07 ms.
        static const float qs[] = { 8.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.5f, 2.0f };
        const float hz = r->peaks[best].hz;
        const int b = lrintf(hz / df);
        float ref;
        peakDb(ftb->welch[0], ftb->welch[1], b, &ref);
        float q = qs[ARRAYLEN(qs) - 1];
        for (unsigned i = 0; i < ARRAYLEN(qs); i++) {
            if (notchTake(ftb->welch[0], ftb->welch[1], b, ref, hz, qs[i], df) >= FT_NOTCH_TAKE) {
                q = qs[i];
                break;
            }
        }
        // the current notch stays while it sits on the bump, still takes nearly as much and is not much wider:
        // no shuffle by a few Hz or one q step from run to run
        const gyroConfig_t *g = gyroConfig();
        const float curHz = g->gyro_soft_notch_hz_1, curCut = g->gyro_soft_notch_cutoff_1;
        const float curQ = curHz > curCut && curCut > 0.0f ? notchQ(curHz, curCut) : 0.0f;
        if (curQ > 0.0f && fabsf(curHz - hz) <= FT_NOTCH_KEEP_REL * hz && curQ >= 0.7f * q
            && notchTake(ftb->welch[0], ftb->welch[1], b, ref, curHz, curQ, df) >= FT_NOTCH_TAKE - 0.05f) {
            r->notchHz = g->gyro_soft_notch_hz_1;
            r->notchCut = g->gyro_soft_notch_cutoff_1;
        } else {
            r->notchHz = lrintf(hz);
            r->notchCut = lrintf(notchCutoff(hz, q));
        }
    }
    // score: 1x balance over the motors and what is left for the dynamic notch
    float x1min = 1e9f, x1max = -1e9f;
    int hot = 0;
    for (int m = 0; m < FT_MOTORS; m++) {
        if (X[m][0] < x1min) {
            x1min = X[m][0];
        }
        if (X[m][0] > x1max) {
            x1max = X[m][0];
            hot = m;
        }
    }
    const float spread = x1max - x1min;
    const float score = 100.0f - 10.0f * MAX(spread - 3.0f, 0.0f) - 15.0f * MAX(left + (best >= 0) - 1, 0);
    r->score = (uint8_t)constrainf(score, 0.0f, 100.0f);
    if (spread > 6.0f) {
        r->badMotor = hot;
    }
    // one dynamic notch for an ideal score, two for a good one; when the count is already the autotune's (1 or 2) it
    // stays within +-FT_SCORE_KEEP of the line, so a score right at it does not flip the count from run to run
    const uint8_t curDyn = dynNotchConfig()->dyn_notch_count;
    const float keep = (curDyn == 1 || curDyn == 2) ? FT_SCORE_KEEP : 0.0f;
    if (left > 1 || r->score < FT_IDEAL_SCORE - keep) {
        r->dyn = 2;
    } else if (r->score >= FT_IDEAL_SCORE + keep) {
        r->dyn = 1;
    } else {
        r->dyn = curDyn;
    }
    // lpf2 from the uncovered orders
    float lpf2 = 500.0f;
    for (int h = 3; h < FT_ORDERS; h++) {
        float x = 0.0f;
        for (int m = 0; m < FT_MOTORS; m++) {
            x = MAX(x, X[m][h]);
        }
        const float need = x - 6.0f;
        if (need > 0.0f) {
            lpf2 = MIN(lpf2, (h + 1) * rotMean / sqrtf(powf(10.0f, need / 10.0f) - 1.0f));
        }
    }
    r->lpf2 = (uint16_t)(constrainf(lpf2, 250.0f, 500.0f) / 10) * 10;
    r->lpf2Basis = LPF2_BY_ORDERS;

    // lpf2 off: 4x / 5x need nothing, the band above FT_HIGH_HZ is at the floor (raw gyro, rotation lines left out),
    // nothing uncovered is left and no residual peak sits up there, and a good score
    float hi = 0.0f, mid[64];
    int nHigh = 0, nm = 0;
    for (int b = 1; b < FT_BINS; b++) {
        const float f = b * df;
        if (onLineOf(f, rotMean)) {               // the mean lines only: four motors' 4x / 5x would cover the band
            continue;
        }
        const float p = ftb->welchPre[b];             // before the RPM filter: its notches would carve the floor
        if (f >= FT_HIGH_HZ && f <= FT_HIGH_TOP_REL * 0.5f * ft.fs) {
            hi += p;
            nHigh++;
        } else if (f >= FT_PEAK_MIN_HZ && nm < 64 && (b & 1)) {   // every other bin: enough for a median
            mid[nm++] = p;
        }
    }
    const float floor = nm >= 8 ? median(mid, nm) : 0.0f;
    if (nHigh > 0 && floor > 0.0f) {
        const float db = 10.0f * log10f((hi / nHigh) / floor);
        r->highDb10 = (int16_t)lrintf(constrainf(db, -300.0f, 300.0f) * 10.0f);
        if (lpf2 >= 500.0f) {                         // 4x / 5x need nothing; highPeak: one the static notch does not take
            if (highPeak) {
                r->lpf2Basis = LPF2_KEPT_HIGH_PEAK;
            } else if (db > FT_HIGH_OFF_DB) {
                r->lpf2Basis = LPF2_KEPT_HIGH_BAND;
            } else if (left <= r->dyn && r->score >= FT_IDEAL_SCORE) {
                r->lpf2 = 0;
                r->lpf2Basis = LPF2_OFF;
            }
        }
    } else {
        r->lpf2Basis = LPF2_NO_DATA;
    }
    if (r->lpf2Basis != LPF2_NO_DATA) {
        decideDterm(r, df);
    }
    r->valid = ft.framesPre >= FT_MIN_FRAMES / 2 && ft.framesPost >= FT_MIN_FRAMES / 2;
}

// ---------------- apply / revert ----------------

static void backupNow(void)
{
    filterTuneBackup_t *b = filterTuneBackupMutable();
    const rpmFilterConfig_t *rpm = rpmFilterConfig();
    const dynNotchConfig_t *dn = dynNotchConfig();
    const gyroConfig_t *g = gyroConfig();
    const pidProfile_t *pp = currentPidProfile;
    for (int h = 0; h < 3; h++) {
        b->rpmWeights[h] = rpm->rpm_filter_weights[h];
    }
    b->rpmQ = rpm->rpm_filter_q;
    b->rpmFadeRangeHz = rpm->rpm_filter_fade_range_hz;
    b->rpmHarmonics = rpm->rpm_filter_harmonics;
    b->rpmMinHz = rpm->rpm_filter_min_hz;
    b->rpmLpfHz = rpm->rpm_filter_lpf_hz;
    b->dynNotchCount = dn->dyn_notch_count;
    b->dynNotchQ = dn->dyn_notch_q;
    b->dynNotchMinHz = dn->dyn_notch_min_hz;
    b->dynNotchMaxHz = dn->dyn_notch_max_hz;
    b->lpf1StaticHz = g->gyro_lpf1_static_hz;
    b->lpf1DynMinHz = g->gyro_lpf1_dyn_min_hz;
    b->gyroLpf1DynMaxHz = g->gyro_lpf1_dyn_max_hz;
    b->gyroLpf1Type = g->gyro_lpf1_type;
    b->lpf2StaticHz = g->gyro_lpf2_static_hz;
    b->gyroLpf2Type = g->gyro_lpf2_type;
    b->simplifiedGyroFilter = g->simplified_gyro_filter;
    b->notch1Hz = g->gyro_soft_notch_hz_1;
    b->notch1Cutoff = g->gyro_soft_notch_cutoff_1;
    b->notch2Hz = g->gyro_soft_notch_hz_2;
    b->notch2Cutoff = g->gyro_soft_notch_cutoff_2;
    b->pidProfileIndex = getCurrentPidProfileIndex();
    b->dtermLpf1DynMinHz = pp->dterm_lpf1_dyn_min_hz;
    b->dtermLpf1DynMaxHz = pp->dterm_lpf1_dyn_max_hz;
    b->dtermLpf1StaticHz = pp->dterm_lpf1_static_hz;
    b->dtermLpf2StaticHz = pp->dterm_lpf2_static_hz;
    b->dtermNotchHz = pp->dterm_notch_hz;
    b->dtermNotchCutoff = pp->dterm_notch_cutoff;
    b->dtermLpf1Type = pp->dterm_lpf1_type;
    b->dtermLpf2Type = pp->dterm_lpf2_type;
    b->dtermLpf1DynExpo = pp->dterm_lpf1_dyn_expo;
    b->simplifiedDtermFilter = pp->simplified_dterm_filter;
    b->valid = 1;
}

// every parameter of the gyro and D-term sections: the decided ones, the rest to the defaults, so no hand setting works
// next to the tuned ones (the backup holds them all for the revert). The D-term filters go to the current PID profile.
static void apply(const ftResult_t *r)
{
    if (!filterTuneBackup()->valid) {                 // keep the oldest settings: revert goes back to before autotune
        backupNow();
    }
    rpmFilterConfig_t *rpm = rpmFilterConfigMutable(), rpmDef;
    dynNotchConfig_t *dn = dynNotchConfigMutable(), dnDef;
    pgResetCopy(&rpmDef, PG_RPM_FILTER_CONFIG);
    pgResetCopy(&dnDef, PG_DYN_NOTCH_CONFIG);
    gyroConfig_t *g = gyroConfigMutable();
    rpm->rpm_filter_harmonics = MAX(rpm->rpm_filter_harmonics, 3);
    for (int h = 0; h < 3; h++) {
        rpm->rpm_filter_weights[h] = r->w[h];
    }
    rpm->rpm_filter_q = r->q;
    rpm->rpm_filter_fade_range_hz = r->fade;
    rpm->rpm_filter_min_hz = rpmDef.rpm_filter_min_hz;
    rpm->rpm_filter_lpf_hz = rpmDef.rpm_filter_lpf_hz;
    dn->dyn_notch_count = r->dyn;
    dn->dyn_notch_q = dnDef.dyn_notch_q;
    dn->dyn_notch_min_hz = dnDef.dyn_notch_min_hz;
    dn->dyn_notch_max_hz = dnDef.dyn_notch_max_hz;
    g->simplified_gyro_filter = 0;
    g->gyro_lpf1_static_hz = 0;
    g->gyro_lpf1_dyn_min_hz = 0;
    g->gyro_lpf2_static_hz = r->lpf2;
    if (r->lpf2) {
        g->gyro_lpf2_type = FILTER_PT1;               // the cutoffs are for PT1
    }
    g->gyro_soft_notch_hz_1 = r->notchHz;
    g->gyro_soft_notch_cutoff_1 = r->notchHz ? r->notchCut : 0;
    g->gyro_soft_notch_hz_2 = 0;
    g->gyro_soft_notch_cutoff_2 = 0;
    if (r->dMult10) {                                 // the D pair: the defaults times the multiplier, PT1, no notch
        pidProfile_t *pp = currentPidProfile;
        pp->simplified_dterm_filter = 0;
        pp->dterm_lpf1_type = FILTER_PT1;
        pp->dterm_lpf2_type = FILTER_PT1;
        pp->dterm_lpf1_dyn_expo = FT_DT_EXPO;
        pp->dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT * r->dMult10 / 10;
        pp->dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT * r->dMult10 / 10;
        pp->dterm_lpf1_static_hz = pp->dterm_lpf1_dyn_min_hz;
        pp->dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT * r->dMult10 / 10;
        pp->dterm_notch_hz = 0;
    }
    saveConfigAndNotify();
    gyroInitFilters();
    pidInitFilters(currentPidProfile);
}

bool filterAutotuneRevert(void)
{
    const filterTuneBackup_t *b = filterTuneBackup();
    if (!b->valid || ARMING_FLAG(ARMED)) {
        return false;
    }
    rpmFilterConfig_t *rpm = rpmFilterConfigMutable();
    dynNotchConfig_t *dn = dynNotchConfigMutable();
    gyroConfig_t *g = gyroConfigMutable();
    for (int h = 0; h < 3; h++) {
        rpm->rpm_filter_weights[h] = b->rpmWeights[h];
    }
    rpm->rpm_filter_q = b->rpmQ;
    rpm->rpm_filter_fade_range_hz = b->rpmFadeRangeHz;
    rpm->rpm_filter_harmonics = b->rpmHarmonics;
    rpm->rpm_filter_min_hz = b->rpmMinHz;
    rpm->rpm_filter_lpf_hz = b->rpmLpfHz;
    dn->dyn_notch_count = b->dynNotchCount;
    dn->dyn_notch_q = b->dynNotchQ;
    dn->dyn_notch_min_hz = b->dynNotchMinHz;
    dn->dyn_notch_max_hz = b->dynNotchMaxHz;
    g->gyro_lpf1_static_hz = b->lpf1StaticHz;
    g->gyro_lpf1_dyn_min_hz = b->lpf1DynMinHz;
    g->gyro_lpf1_dyn_max_hz = b->gyroLpf1DynMaxHz;
    g->gyro_lpf1_type = b->gyroLpf1Type;
    g->gyro_lpf2_static_hz = b->lpf2StaticHz;
    g->gyro_lpf2_type = b->gyroLpf2Type;
    g->simplified_gyro_filter = b->simplifiedGyroFilter;
    g->gyro_soft_notch_hz_1 = b->notch1Hz;
    g->gyro_soft_notch_cutoff_1 = b->notch1Cutoff;
    g->gyro_soft_notch_hz_2 = b->notch2Hz;
    g->gyro_soft_notch_cutoff_2 = b->notch2Cutoff;
    pidProfile_t *pp = pidProfilesMutable(MIN(b->pidProfileIndex, PID_PROFILE_COUNT - 1));   // the profile it applied to
    pp->dterm_lpf1_dyn_min_hz = b->dtermLpf1DynMinHz;
    pp->dterm_lpf1_dyn_max_hz = b->dtermLpf1DynMaxHz;
    pp->dterm_lpf1_static_hz = b->dtermLpf1StaticHz;
    pp->dterm_lpf2_static_hz = b->dtermLpf2StaticHz;
    pp->dterm_notch_hz = b->dtermNotchHz;
    pp->dterm_notch_cutoff = b->dtermNotchCutoff;
    pp->dterm_lpf1_type = b->dtermLpf1Type;
    pp->dterm_lpf2_type = b->dtermLpf2Type;
    pp->dterm_lpf1_dyn_expo = b->dtermLpf1DynExpo;
    pp->simplified_dterm_filter = b->simplifiedDtermFilter;
    filterTuneBackupMutable()->valid = 0;
    ft.applied = false;
    saveConfigAndNotify();
    gyroInitFilters();
    pidInitFilters(currentPidProfile);
    return true;
}

// ---------------- task ----------------

static void resetSession(void)
{
    memset(ft.sumX, 0, sizeof(ft.sumX));
    memset(ft.nX, 0, sizeof(ft.nX));
    memset(ft.sumR, 0, sizeof(ft.sumR));
    memset(ft.nR, 0, sizeof(ft.nR));
    ft.sumR1 = 0.0f;
    ft.nR1 = 0;
    if (scratchOwned()) {
        memset(ftb->welch, 0, sizeof(ftb->welch));
        memset(ftb->welchPre, 0, sizeof(ftb->welchPre));
    }
    ft.frames = ft.framesPre = ft.framesPost = 0;
    ft.thrSum = 0.0f;
    ft.thrN = 0;
    ft.vbatSum = ft.noiseSum2 = 0.0f;
    memset(&ftx, 0, sizeof(ftx));
    filterAutotuneSweepOn = false;
    ft.haveFloor1 = false;
    memset(ft.rotFrameSum, 0, sizeof(ft.rotFrameSum));
    ft.compFs = 0.0f;                                // the filters may have changed (autotune applied)
    ft.readyBeeped = false;
    ft.framesCarried = 0;
    ft.flightsCarried = 0;
    resetFrame();
}

// after the disarm guard (the props stopping), short and only after a disarm with the mode on
static void queueBeacons(const uint8_t *cmd, int n)
{
    ft.beaconN = MIN(n, (int)ARRAYLEN(ft.beacon));
    memcpy(ft.beacon, cmd, ft.beaconN);
    ft.beaconI = 0;
    ft.beaconAt = millis() + FT_BEACON_GUARD_MS;
}

static void serviceBeacons(bool armed)
{
    if (armed) {
        ft.beaconN = 0;
        return;
    }
    if (ft.beaconI < ft.beaconN && cmp32(millis(), ft.beaconAt) >= 0) {
#ifdef USE_DSHOT
        if (isMotorProtocolDshot()) {
            dshotCommandWrite(ALL_MOTORS, getMotorCount(), ft.beacon[ft.beaconI], DSHOT_CMD_TYPE_INLINE);
        }
#endif
        ft.beaconI++;
        ft.beaconAt = millis() + FT_BEACON_GAP_MS;
    }
}

void filterAutotuneInit(void)
{
    // decimate the gyro filter loop to about 2 kHz
    const float loopHz = gyro.targetLooptime ? 1e6f / gyro.targetLooptime : 4000.0f;
    ft.decimN = MAX(1, lrintf(loopHz / FT_FS_TARGET));
    ft.state = mapReady() ? FILTER_TUNE_IDLE : FILTER_TUNE_NO_MAP;
    resetSession();
}

static bool telemetryOk(void)
{
    return isDshotTelemetryActive() && rpmFilterConfig()->rpm_filter_harmonics > 0;
}

void filterAutotuneUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    const bool armed = ARMING_FLAG(ARMED);
    const bool modeOn = IS_RC_MODE_ACTIVE(BOXFILTERTUNE);

    if (!armed && ft.saveMap) {                       // the frame wizard stored a new map
        ft.saveMap = false;
        saveConfigAndNotify();
        if (ft.state == FILTER_TUNE_NO_MAP && mapReady()) {
            ft.state = FILTER_TUNE_IDLE;
        }
    }

    // disarm: apply, or keep the frames for the next flight, or discard
    if (ft.wasArmed && !armed) {
        filterAutotuneCollecting = false;
        if (ft.state == FILTER_TUNE_COLLECTING || ft.state == FILTER_TUNE_READY || ft.state == FILTER_TUNE_WAITING
            || ft.state == FILTER_TUNE_EXCITING) {
            decide(&ft.result);
            if (modeOn && ft.result.valid) {
                apply(&ft.result);
                ft.applied = true;
                ft.state = FILTER_TUNE_APPLIED;
                static const uint8_t applied[] = { DSHOT_CMD_BEACON1, DSHOT_CMD_BEACON3, DSHOT_CMD_BEACON5 };
                queueBeacons(applied, ARRAYLEN(applied));        // rising: saved
            } else if (modeOn) {
                ft.state = FILTER_TUNE_PARTIAL;                  // hover more: the next flight adds to these frames
                ft.framesCarried = ft.frames;
                ft.flightsCarried++;
                static const uint8_t more[] = { DSHOT_CMD_BEACON2, DSHOT_CMD_BEACON2 };
                queueBeacons(more, ARRAYLEN(more));              // two low: not enough yet
            } else {
                ft.state = FILTER_TUNE_DISCARDED;
            }
        }
    }
    if (!ft.wasArmed && armed) {
        // the frames of an earlier flight stay while the mode stays on (the buffers are ours and the filters the same)
        const bool carryOn = modeOn && ft.state == FILTER_TUNE_PARTIAL && scratchOwned();
        if (!carryOn && (modeOn || ft.state != FILTER_TUNE_APPLIED)) {
            resetSession();
        }
    }
    ft.wasArmed = armed;
    serviceBeacons(armed);
    sweepService(modeOn);

    if (!mapReady()) {
        ft.state = FILTER_TUNE_NO_MAP;
        filterAutotuneCollecting = false;
        return;
    }
    if (!armed || !modeOn) {
        filterAutotuneCollecting = false;
        if (armed && ft.state != FILTER_TUNE_APPLIED) {
            ft.state = FILTER_TUNE_IDLE;
        } else if (ft.state == FILTER_TUNE_NO_MAP) {
            ft.state = FILTER_TUNE_IDLE;
        }
        return;
    }
    if (!telemetryOk()) {
        ft.state = FILTER_TUNE_NO_TELEMETRY;
        filterAutotuneCollecting = false;
        return;
    }

    // the sweep: no noise frames while it runs; once over (or ended), ready
    if (ft.state == FILTER_TUNE_EXCITING) {
        filterAutotuneCollecting = false;
        if (ftx.phase == SWEEP_NONE) {
            sweepTryStart();                          // waits for ANGLE and centred sticks
        }
        if (!sweepActive() && ftx.phase != SWEEP_NONE) {
            ft.state = FILTER_TUNE_READY;
            if (!ft.readyBeeped) {
                ft.readyBeeped = true;
                beeperConfirmationBeeps(2);           // enough: land and disarm with the mode on to apply
            }
        }
        return;
    }
    if (!scratchOwned()) {                            // the buffers: from the frame wizard (disarmed only) to us
        ftb = benchScratchClaim(BENCH_SCRATCH_FILTER_AUTOTUNE);
        const uint8_t sig = ft.sig;
        resetSession();
        ft.sig = sig;                                 // keep the before / after order of a plain arm
    }
    filterAutotuneCollecting = true;
    if (!ft.full) {
        const float thr = rcCommand[THROTTLE];
        ft.thrMin = MIN(ft.thrMin, thr);
        ft.thrMax = MAX(ft.thrMax, thr);
        if (ft.state != FILTER_TUNE_COLLECTING && ft.state != FILTER_TUNE_READY) {
            ft.state = FILTER_TUNE_WAITING;
        }
        return;
    }
    // a full frame: steady? then analyse it in steps
    if (ft.job == 0 && ft.step == 0) {
        const float dtUs = (float)cmpTimeUs(ft.t1, ft.t0);
        const bool steady = ft.rotMin >= FT_ROT_MIN_HZ && ft.rateMax <= FT_RATE_MAX
            && (ft.thrMax - ft.thrMin) <= FT_THR_RANGE && dtUs > 0.0f;
        if (!steady) {
            if (ft.state != FILTER_TUNE_READY) {
                ft.state = ft.frames ? FILTER_TUNE_COLLECTING : FILTER_TUNE_WAITING;
            }
            resetFrame();
            return;
        }
        ft.fs = (FT_N - 1) * 1e6f / dtUs;
        ft.thrSum += 0.5f * (ft.thrMin + ft.thrMax);   // the hover throttle: the dynamic lowpass cutoffs
        ft.vbatSum += getBatteryVoltage() * 0.01f;
#ifdef USE_MOTOR_NOISE
        float n2 = 0.0f;
        for (int m = 0; m < FT_MOTORS; m++) {
            n2 += sq(motorNoiseRms(m, 0));
        }
        ft.noiseSum2 += n2 / FT_MOTORS;
#endif
        ft.thrN++;
    }
    if (fftStep()) {
        if (++ft.job >= 3) {
            analyseFrame();
            resetFrame();
            const bool enough = ft.framesPre >= FT_MIN_FRAMES / 2 && ft.framesPost >= FT_MIN_FRAMES / 2;
            if (!enough) {
                ft.state = FILTER_TUNE_COLLECTING;
            } else if (!ftx.tried) {
                ft.state = FILTER_TUNE_EXCITING;          // the frames are in: the sweep next (ANGLE, sticks centred)
                sweepTryStart();
            } else {
                ft.state = FILTER_TUNE_READY;
                if (!ft.readyBeeped) {
                    ft.readyBeeped = true;
                    beeperConfirmationBeeps(2);           // enough: land and disarm with the mode on to apply
                }
            }
        }
    }
}

// ---------------- pilot signals ----------------

bool filterAutotuneLedSignal(timeUs_t now, filterTuneLed_e *led)
{
    if (!IS_RC_MODE_ACTIVE(BOXFILTERTUNE)) {
        return false;
    }
    const uint32_t ms = now / 1000;
    const bool slow = (ms % 1000) < 500;              // 1 Hz
    const bool fast = (ms % 250) < 125;               // 4 Hz
    switch (ft.state) {
    case FILTER_TUNE_WAITING:                          // armed, not a steady hover yet
    case FILTER_TUNE_PARTIAL:                          // landed with too few frames: fly again with the mode on
        *led = slow ? FILTER_TUNE_LED_BLUE : FILTER_TUNE_LED_OFF;
        return true;
    case FILTER_TUNE_COLLECTING:
    case FILTER_TUNE_EXCITING:
        *led = fast ? FILTER_TUNE_LED_BLUE : FILTER_TUNE_LED_OFF;
        return true;
    case FILTER_TUNE_READY:                            // land and disarm with the mode on
        *led = FILTER_TUNE_LED_GREEN;
        return true;
    case FILTER_TUNE_APPLIED:
        *led = slow ? FILTER_TUNE_LED_GREEN : FILTER_TUNE_LED_OFF;
        return true;
    case FILTER_TUNE_DISCARDED:                        // disarmed with the mode off: the frames dropped
        *led = FILTER_TUNE_LED_RED;
        return true;
    case FILTER_TUNE_NO_MAP:
    case FILTER_TUNE_NO_TELEMETRY:
        *led = fast ? FILTER_TUNE_LED_RED : FILTER_TUNE_LED_OFF;
        return true;
    default:
        return false;
    }
}

bool filterAutotuneOsdWarning(char *text, bool *blinking, bool *critical)
{
    if (!IS_RC_MODE_ACTIVE(BOXFILTERTUNE)) {
        return false;
    }
    *blinking = false;
    *critical = false;
    switch (ft.state) {
    case FILTER_TUNE_WAITING:
    case FILTER_TUNE_COLLECTING:
        if (ft.frames == 0) {
            tfp_sprintf(text, "FILTER TUNE HOVER");
        } else {
            tfp_sprintf(text, "FILTER TUNE %d/%d", MIN(ft.frames, FT_MIN_FRAMES - 1), FT_MIN_FRAMES);
        }
        return true;
    case FILTER_TUNE_PARTIAL:
        tfp_sprintf(text, "FILTER TUNE MORE %d/%d", MIN(ft.frames, FT_MIN_FRAMES - 1), FT_MIN_FRAMES);
        return true;
    case FILTER_TUNE_EXCITING:
        if (ftx.phase == SWEEP_NONE && !FLIGHT_MODE(ANGLE_MODE)) {
            tfp_sprintf(text, "FILTER TUNE ANGLE");   // the sweep needs ANGLE
            *blinking = true;
        } else {
            tfp_sprintf(text, "FILTER TUNE SWEEP");
        }
        return true;
    case FILTER_TUNE_READY:
        tfp_sprintf(text, "FILTER TUNE READY");
        *blinking = true;
        return true;
    case FILTER_TUNE_APPLIED:
        tfp_sprintf(text, "FILTER TUNE SAVED");
        return true;
    case FILTER_TUNE_DISCARDED:
        tfp_sprintf(text, "FILTER TUNE SHORT");
        *critical = true;
        return true;
    case FILTER_TUNE_NO_MAP:
        tfp_sprintf(text, "FILTER TUNE NO MAP");
        *critical = true;
        return true;
    case FILTER_TUNE_NO_TELEMETRY:
        tfp_sprintf(text, "FILTER TUNE NO RPM");
        *critical = true;
        return true;
    default:
        return false;
    }
}

static uint8_t signalChannels(void)
{
    uint8_t ch = 0;
#ifdef USE_BEEPER
    if (beeperDevConfig()->ioTag != IO_TAG_NONE) {
        ch |= FILTER_TUNE_SIGNAL_BEEPER;
    }
#endif
#ifdef USE_LED_STRIP_STATUS_MODE
    // some LED has the filter tune overlay, in the status profile
    if (featureIsEnabled(FEATURE_LED_STRIP) && ledStripConfig()->ioTag != IO_TAG_NONE
        && ledStripConfig()->ledstrip_profile == LED_PROFILE_STATUS) {
        for (int i = 0; i < LED_STRIP_MAX_LENGTH; i++) {
            if (ledGetOverlayBit(&ledStripStatusModeConfig()->ledConfigs[i], LED_OVERLAY_FILTER_TUNE)) {
                ch |= FILTER_TUNE_SIGNAL_LED;
                break;
            }
        }
    }
#endif
#ifdef USE_OSD
    if (featureIsEnabled(FEATURE_OSD) && osdGetDisplayPort(NULL) && osdWarnGetState(OSD_WARNING_FILTER_TUNE)) {
        ch |= FILTER_TUNE_SIGNAL_OSD;
    }
#endif
    return ch;
}

// ---------------- MSP ----------------

void filterAutotuneWriteStatus(sbuf_t *dst)
{
    ftResult_t r;
    if (ft.state == FILTER_TUNE_APPLIED || ft.state == FILTER_TUNE_DISCARDED) {
        r = ft.result;
    } else {
        decide(&r);
    }
    const rpmFilterConfig_t *rpm = rpmFilterConfig();
    const gyroConfig_t *g = gyroConfig();
    sbufWriteU8(dst, 1);                              // payload version
    sbufWriteU8(dst, ft.state);
    sbufWriteU8(dst, (mapReady() ? 1 : 0) | (telemetryOk() ? 2 : 0) | (IS_RC_MODE_ACTIVE(BOXFILTERTUNE) ? 4 : 0)
                     | (r.valid ? 8 : 0) | (filterTuneBackup()->valid ? 16 : 0) | (ft.applied ? 32 : 0));
    sbufWriteU16(dst, ft.frames);
    sbufWriteU16(dst, FT_MIN_FRAMES);
    sbufWriteU16(dst, lrintf(ft.fs));
    for (int m = 0; m < FT_MOTORS; m++) {
        for (int h = 0; h < FT_ORDERS; h++) {
            sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(ft.nX[m][h] ? 10.0f * ft.sumX[m][h] / ft.nX[m][h] : 0.0f));
        }
    }
    sbufWriteU16(dst, (uint16_t)(int16_t)lrintf(ft.nR1 ? 10.0f * ft.sumR1 / ft.nR1 : 0.0f));
    sbufWriteU8(dst, r.score);
    sbufWriteU8(dst, r.badMotor);
    sbufWriteU8(dst, r.nPeaks);
    for (int i = 0; i < r.nPeaks; i++) {
        sbufWriteU16(dst, lrintf(r.peaks[i].hz * 10.0f));
        sbufWriteU16(dst, lrintf(r.peaks[i].db * 10.0f));
        sbufWriteU8(dst, (r.peaks[i].fixed ? 1 : 0) | (r.peaks[i].inMap ? 2 : 0));
    }
    // proposed, then current
    for (int h = 0; h < 3; h++) {
        sbufWriteU8(dst, r.w[h]);
    }
    sbufWriteU16(dst, r.q);
    sbufWriteU8(dst, r.dyn);
    sbufWriteU16(dst, r.lpf2);
    sbufWriteU16(dst, r.notchHz);
    sbufWriteU16(dst, r.notchCut);
    for (int h = 0; h < 3; h++) {
        sbufWriteU8(dst, rpm->rpm_filter_weights[h]);
    }
    sbufWriteU16(dst, rpm->rpm_filter_q);
    sbufWriteU8(dst, dynNotchConfig()->dyn_notch_count);
    sbufWriteU16(dst, g->gyro_lpf2_static_hz);
    sbufWriteU16(dst, g->gyro_soft_notch_hz_1);
    sbufWriteU16(dst, g->gyro_soft_notch_cutoff_1);
    sbufWriteU16(dst, g->gyro_lpf1_static_hz);
    sbufWriteU16(dst, g->gyro_lpf1_dyn_min_hz);
    // appended (same payload version: readers take them only when the bytes are there)
    sbufWriteU16(dst, (uint16_t)r.highDb10);
    sbufWriteU8(dst, r.lpf2Basis);
    sbufWriteU16(dst, r.fade);                        // rpm_filter_fade_range_hz: proposed, current
    sbufWriteU16(dst, rpm->rpm_filter_fade_range_hz);
    sbufWriteU16(dst, lrintf(constrainf(r.trackErr * 1e5f, 0.0f, 65535.0f)));   // notch tracking error, 0.001 %
    sbufWriteU8(dst, signalChannels());              // FILTER_TUNE_SIGNAL_*: beeper pin, LED strip, OSD
    sbufWriteU16(dst, ft.framesCarried);             // of the frames, from earlier flights (FILTER_TUNE_PARTIAL)
    sbufWriteU8(dst, ft.flightsCarried);
    // the D-term filters: proposed multiplier x 10 (0 = unchanged), the current D pair, delays and ratios
    sbufWriteU8(dst, r.dMult10);
    sbufWriteU16(dst, currentPidProfile->dterm_lpf1_dyn_min_hz);
    sbufWriteU16(dst, currentPidProfile->dterm_lpf1_dyn_max_hz);
    sbufWriteU16(dst, currentPidProfile->dterm_lpf2_static_hz);
    sbufWriteU16(dst, r.pdDelayUs);
    sbufWriteU16(dst, r.pdDelayRefUs);
    sbufWriteU8(dst, r.dInPct);
    sbufWriteU8(dst, r.noisePct);
    // the PID profile the D-term filters go to (the current one) and, after an apply, the one they went to (0xFF none)
    sbufWriteU8(dst, getCurrentPidProfileIndex());
    sbufWriteU8(dst, filterTuneBackup()->valid ? filterTuneBackup()->pidProfileIndex : 0xFF);
    // the sweep and the heat budget
    sbufWriteU8(dst, ftx.phase);                     // SWEEP_*
    sbufWriteU8(dst, ftx.endReason);                 // SWEEP_END_*
    sbufWriteU16(dst, lrintf(ftx.amp * 10.0f));      // amplitude, 0.1 deg/s
    sbufWriteU16(dst, lrintf(ftx.fc[0] * 10.0f));    // crossover roll / pitch, 0.1 Hz (0 = none)
    sbufWriteU16(dst, lrintf(ftx.fc[1] * 10.0f));
    sbufWriteU8(dst, (uint8_t)constrainf(ftx.pmDeg[0], 0.0f, 255.0f));   // phase margin roll / pitch, deg
    sbufWriteU8(dst, (uint8_t)constrainf(ftx.pmDeg[1], 0.0f, 255.0f));
    sbufWriteU16(dst, r.idxMeas10);
    sbufWriteU16(dst, r.idxCur10);
    sbufWriteU16(dst, r.heatRef100);
    sbufWriteU16(dst, r.heatChosen100);
    sbufWriteU8(dst, filterTuneConfig()->heat_budget_dk);
}

void frameMapWrite(sbuf_t *dst)
{
    const frameMapConfig_t *map = frameMapConfig();
    sbufWriteU8(dst, 2);                              // payload version (2: the motor profile follows the modes)
    sbufWriteU8(dst, map->parts);
    sbufWriteU8(dst, map->count);
    for (int i = 0; i < map->count; i++) {
        sbufWriteU16(dst, map->modeHz10[i]);
        sbufWriteU8(dst, map->modeSrc[i]);
    }
    for (int m = 0; m < FRAME_MAP_MOTORS; m++) {
        sbufWriteU8(dst, map->motorPeakCount[m]);
        for (int i = 0; i < map->motorPeakCount[m]; i++) {
            sbufWriteU16(dst, map->motorPeakHz10[m][i]);
            sbufWriteU16(dst, map->motorPeakGain[m][i]);
            sbufWriteU8(dst, map->motorPeakInfo[m][i]);
        }
    }
    sbufWriteU16(dst, map->rotHz10[0]);
    sbufWriteU16(dst, map->rotHz10[1]);
    sbufWriteU8(dst, map->spinCheck);
    for (int k = 0; k < 3; k++) {
        sbufWriteU8(dst, (uint8_t)map->runGravity[k]);   // appended: how the craft was held in the last motor run
    }
}

#endif // USE_FILTER_AUTOTUNE
