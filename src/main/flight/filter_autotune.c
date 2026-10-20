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
#include "drivers/time.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "io/beeper.h"
#include "io/ledstrip.h"
#include "osd/osd.h"
#include "pg/beeper_dev.h"
#include "pg/dyn_notch.h"
#include "pg/pg_ids.h"
#include "pg/rpm_filter.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "bench_scratch.h"
#include "filter_autotune.h"

PG_REGISTER(frameMapConfig_t, frameMapConfig, PG_FRAME_MAP_CONFIG, 1);
PG_REGISTER(filterTuneBackup_t, filterTuneBackup, PG_FILTER_TUNE_BACKUP, 1);

#define FT_N            512
#define FT_BINS         (FT_N / 2 + 1)
#define FT_M            (FT_N / 2)          // complex points of the real FFT
#define FT_LOG2M        8
#define FT_MOTORS       4
#define FT_ORDERS       5                   // measured; the RPM filter covers 3
#define FT_FS_TARGET    2000.0f             // decimated sample rate target, Hz
#define FT_MIN_FRAMES   60                  // ~16 s of samples, ~32 s of steady hover
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

enum { LPF2_BY_ORDERS = 0, LPF2_OFF, LPF2_KEPT_HIGH_BAND, LPF2_KEPT_HIGH_PEAK, LPF2_NO_DATA = 255 };

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
    float rotFrameSum[FT_MOTORS];   // rotation of each motor in the frames before the RPM filter, summed
    float fs;
    // flow
    filterTuneState_e state;
    bool wasArmed, readyBeeped, saveMap, applied;
    ftResult_t result;
} ft;


// ---------------- frame map ----------------

void frameMapClear(void)
{
    memset(frameMapConfigMutable(), 0, sizeof(frameMapConfig_t));
    ft.saveMap = true;
}

// two entries merged: unstable only while no stable tap entry is among them (a motor one makes it a frame mode anyway)
static uint8_t mergeSrc(uint8_t a, uint8_t b)
{
    const bool stableTap = ((a & FRAME_MAP_SRC_TAP) && !(a & FRAME_MAP_SRC_UNSTABLE))
        || ((b & FRAME_MAP_SRC_TAP) && !(b & FRAME_MAP_SRC_UNSTABLE));
    const uint8_t v = (a | b) & ~FRAME_MAP_SRC_UNSTABLE;
    return (v & FRAME_MAP_SRC_TAP) && !stableTap ? v | FRAME_MAP_SRC_UNSTABLE : v;
}

void frameMapSetPart(uint8_t part, const float *hz, const bool *unstable, int n)
{
    frameMapConfig_t *map = frameMapConfigMutable();
    const uint8_t src = part == FRAME_MAP_PART_TAPS ? FRAME_MAP_SRC_TAP : FRAME_MAP_SRC_MOTOR;
    const uint8_t drop = part == FRAME_MAP_PART_TAPS ? FRAME_MAP_SRC_TAP | FRAME_MAP_SRC_UNSTABLE : FRAME_MAP_SRC_MOTOR;
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
        s[k++] = src | (unstable && unstable[i] ? FRAME_MAP_SRC_UNSTABLE : 0);
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

// the frame map has modes near hz (8 %) and every one of them is an unstable tap mode
static bool onlyUnstableInMap(float hz)
{
    const frameMapConfig_t *map = frameMapConfig();
    bool any = false;
    for (int i = 0; i < map->count; i++) {
        const uint8_t s = map->modeSrc[i];
        if ((s & (FRAME_MAP_SRC_TAP | FRAME_MAP_SRC_MOTOR)) && fabsf(map->modeHz10[i] / 10.0f - hz) <= 0.08f * hz) {
            if (!(s & FRAME_MAP_SRC_UNSTABLE)) {
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
    // fade: the slowest motor's hover 1x clear of min_hz + fade (in the fade the 1x notch only works in part);
    // min_hz stays: lower notches would add delay at low throttle
    float rotLow = rotM[0];
    for (int m = 1; m < FT_MOTORS; m++) {
        rotLow = MIN(rotLow, rotM[m]);
    }
    const float fadeMax = FT_FADE_CLEAR * rotLow - minHz;
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
            const float w = r->w[h] / 100.0f * fadeWeight((h + 1) * rotM[m], minHz, r->fade);
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
    r->valid = ft.framesPre >= FT_MIN_FRAMES / 2 && ft.framesPost >= FT_MIN_FRAMES / 2;
}

// ---------------- apply / revert ----------------

static void backupNow(void)
{
    filterTuneBackup_t *b = filterTuneBackupMutable();
    const rpmFilterConfig_t *rpm = rpmFilterConfig();
    const gyroConfig_t *g = gyroConfig();
    for (int h = 0; h < 3; h++) {
        b->rpmWeights[h] = rpm->rpm_filter_weights[h];
    }
    b->rpmQ = rpm->rpm_filter_q;
    b->rpmFadeRangeHz = rpm->rpm_filter_fade_range_hz;
    b->dynNotchCount = dynNotchConfig()->dyn_notch_count;
    b->lpf1StaticHz = g->gyro_lpf1_static_hz;
    b->lpf1DynMinHz = g->gyro_lpf1_dyn_min_hz;
    b->lpf2StaticHz = g->gyro_lpf2_static_hz;
    b->simplifiedGyroFilter = g->simplified_gyro_filter;
    b->notch1Hz = g->gyro_soft_notch_hz_1;
    b->notch1Cutoff = g->gyro_soft_notch_cutoff_1;
    b->valid = 1;
}

static void apply(const ftResult_t *r)
{
    if (!filterTuneBackup()->valid) {                 // keep the oldest settings: revert goes back to before autotune
        backupNow();
    }
    rpmFilterConfig_t *rpm = rpmFilterConfigMutable();
    gyroConfig_t *g = gyroConfigMutable();
    rpm->rpm_filter_harmonics = MAX(rpm->rpm_filter_harmonics, 3);
    for (int h = 0; h < 3; h++) {
        rpm->rpm_filter_weights[h] = r->w[h];
    }
    rpm->rpm_filter_q = r->q;
    rpm->rpm_filter_fade_range_hz = r->fade;
    dynNotchConfigMutable()->dyn_notch_count = r->dyn;
    g->simplified_gyro_filter = 0;
    g->gyro_lpf1_static_hz = 0;
    g->gyro_lpf1_dyn_min_hz = 0;
    g->gyro_lpf2_static_hz = r->lpf2;
    g->gyro_soft_notch_hz_1 = r->notchHz;
    g->gyro_soft_notch_cutoff_1 = r->notchHz ? r->notchCut : 0;
    saveConfigAndNotify();
    gyroInitFilters();
}

bool filterAutotuneRevert(void)
{
    const filterTuneBackup_t *b = filterTuneBackup();
    if (!b->valid || ARMING_FLAG(ARMED)) {
        return false;
    }
    rpmFilterConfig_t *rpm = rpmFilterConfigMutable();
    gyroConfig_t *g = gyroConfigMutable();
    for (int h = 0; h < 3; h++) {
        rpm->rpm_filter_weights[h] = b->rpmWeights[h];
    }
    rpm->rpm_filter_q = b->rpmQ;
    rpm->rpm_filter_fade_range_hz = b->rpmFadeRangeHz;
    dynNotchConfigMutable()->dyn_notch_count = b->dynNotchCount;
    g->gyro_lpf1_static_hz = b->lpf1StaticHz;
    g->gyro_lpf1_dyn_min_hz = b->lpf1DynMinHz;
    g->gyro_lpf2_static_hz = b->lpf2StaticHz;
    g->simplified_gyro_filter = b->simplifiedGyroFilter;
    g->gyro_soft_notch_hz_1 = b->notch1Hz;
    g->gyro_soft_notch_cutoff_1 = b->notch1Cutoff;
    filterTuneBackupMutable()->valid = 0;
    saveConfigAndNotify();
    gyroInitFilters();
    ft.applied = false;
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
    ft.haveFloor1 = false;
    memset(ft.rotFrameSum, 0, sizeof(ft.rotFrameSum));
    ft.compFs = 0.0f;                                // the filters may have changed (autotune applied)
    ft.readyBeeped = false;
    resetFrame();
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

    // disarm: apply or discard
    if (ft.wasArmed && !armed) {
        filterAutotuneCollecting = false;
        if (ft.state == FILTER_TUNE_COLLECTING || ft.state == FILTER_TUNE_READY || ft.state == FILTER_TUNE_WAITING) {
            decide(&ft.result);
            if (modeOn && ft.result.valid) {
                apply(&ft.result);
                ft.applied = true;
                ft.state = FILTER_TUNE_APPLIED;
            } else {
                ft.state = FILTER_TUNE_DISCARDED;
            }
        }
    }
    if (!ft.wasArmed && armed) {
        if (modeOn || ft.state != FILTER_TUNE_APPLIED) {
            resetSession();
        }
    }
    ft.wasArmed = armed;

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
    }
    if (fftStep()) {
        if (++ft.job >= 3) {
            analyseFrame();
            resetFrame();
            ft.state = ft.framesPre >= FT_MIN_FRAMES / 2 && ft.framesPost >= FT_MIN_FRAMES / 2
                ? FILTER_TUNE_READY : FILTER_TUNE_COLLECTING;
            if (ft.state == FILTER_TUNE_READY && !ft.readyBeeped) {
                ft.readyBeeped = true;
                beeperConfirmationBeeps(2);               // enough: land and disarm with the mode on to apply
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
        *led = slow ? FILTER_TUNE_LED_BLUE : FILTER_TUNE_LED_OFF;
        return true;
    case FILTER_TUNE_COLLECTING:
        *led = fast ? FILTER_TUNE_LED_BLUE : FILTER_TUNE_LED_OFF;
        return true;
    case FILTER_TUNE_READY:                            // land and disarm with the mode on
        *led = FILTER_TUNE_LED_GREEN;
        return true;
    case FILTER_TUNE_APPLIED:
        *led = slow ? FILTER_TUNE_LED_GREEN : FILTER_TUNE_LED_OFF;
        return true;
    case FILTER_TUNE_DISCARDED:                        // disarmed with the mode on but too few frames
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
}

#endif // USE_FILTER_AUTOTUNE
