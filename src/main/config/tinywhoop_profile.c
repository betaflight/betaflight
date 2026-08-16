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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TINYWHOOP

#include "common/maths.h"

#include "config/simplified_tuning.h"
#include "config/tinywhoop_profile.h"

#include "fc/controlrate_profile.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "osd/osd.h"

#include "pg/dyn_notch.h"
#include "pg/motor.h"
#include "pg/rx.h"

#include "sensors/gyro.h"

/*
 * Filter cutoffs, as a percentage of the stock (5") defaults.
 *
 * Both gyro and D term lowpasses are PT1s, whose group delay is 1/(2*pi*fc).
 * At the stock dynamic gyro cutoffs (250-500Hz) that is 0.64ms falling to
 * 0.32ms; the D term pair (75-150Hz dynamic, plus a 150Hz static second stage)
 * contributes considerably more. Raising every cutoff by a quarter removes
 * about a fifth of the filter delay in the gyro -> PID -> motor path.
 *
 * A whoop can afford this: its noise sits far higher in frequency than a 5"
 * (motors run 2-3x the RPM) and its ducted props put much less mechanical
 * energy into the frame, so the filters are not what is keeping the motors
 * cool. This is the same knob the Configurator filter sliders drive, so the
 * result is still a plain slider setting that a user can wind back down.
 *
 * Every constant in this file follows the codebase's existing convention for
 * a build-time default (see DEFAULT_PID_PROCESS_DENOM in flight/pid.c): wrap
 * it in #ifndef/#define so a board config.h, or OPTIONS="NAME=value" on the
 * command line, can override any single number without editing this file or
 * forking the profile.
 */
#ifndef TINYWHOOP_FILTER_MULTIPLIER
#define TINYWHOOP_FILTER_MULTIPLIER 125
#endif

/*
 * RC smoothing auto factor. The auto cutoff is
 *      rxRate * 1.5 / (1 + factor/10)
 * so a *lower* factor means a higher cutoff and less delay. Dropping 30 -> 20
 * moves the cutoff from 0.375x to 0.5x of the link rate: at 500Hz ELRS that is
 * 188Hz instead of 141Hz, worth roughly 0.3ms of PT3 group delay. Throttle is
 * deliberately left at the stock factor - throttle smoothness matters more than
 * throttle latency on a light craft, and whoop props react to it instantly.
 */
#ifndef TINYWHOOP_RC_SMOOTHING_AUTO_FACTOR_RPY
#define TINYWHOOP_RC_SMOOTHING_AUTO_FACTOR_RPY 20
#endif

/*
 * Dynamic notch. Whoop motors idle around 8-10k RPM and run to 40k+, so the
 * lowest useful noise peak is well above the stock 100Hz floor; lifting the
 * floor stops the tracker locking onto frame/prop-wash content that the
 * lowpasses already handle. Three notches also cost measurable loop time on an
 * F4 whoop board for little benefit when the noise is dominated by one peak.
 */
#ifndef TINYWHOOP_DYN_NOTCH_MIN_HZ
#define TINYWHOOP_DYN_NOTCH_MIN_HZ  150
#endif
#ifndef TINYWHOOP_DYN_NOTCH_MAX_HZ
#define TINYWHOOP_DYN_NOTCH_MAX_HZ  700
#endif
#ifndef TINYWHOOP_DYN_NOTCH_COUNT
#define TINYWHOOP_DYN_NOTCH_COUNT   2
#endif

/*
 * USE_TINYWHOOP_RACE selects the race variant of the profile below: a whoop
 * raced around a short indoor track wants a snappier stick response than one
 * flown freestyle, at the cost of some of the softness around centre that
 * makes freestyle flying forgiving in a cluttered room. It is a refinement of
 * the whoop profile, not a separate craft, so it requires USE_TINYWHOOP - that
 * dependency is enforced in target/common_post.h, not here: this whole file is
 * only compiled when USE_TINYWHOOP is defined, so a check placed inside it can
 * never observe the case where USE_TINYWHOOP_RACE is set without it.
 */

/*
 * Feel / movement.
 *
 * Rates use the ACTUAL scheme (as does the stock default): rcRate is centre
 * sensitivity in deg/s / 10, rates is max rate in deg/s / 10. The stock 70/670
 * is a 5" freestyle rate. Whoops are flown in small spaces where the useful
 * range is a soft centre for precision between gaps and a still-quick maximum
 * for flips, and yaw is deliberately slower than roll/pitch because a ducted
 * whoop has very little yaw authority to spare.
 *
 * The race variant trades away some of that centre softness for a flatter,
 * more immediate response suited to holding a line at speed.
 */
#ifndef TINYWHOOP_RC_RATE_ROLL_PITCH
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_RC_RATE_ROLL_PITCH   8    // 80 deg/s centre sensitivity
#else
#define TINYWHOOP_RC_RATE_ROLL_PITCH   6    // 60 deg/s centre sensitivity
#endif
#endif
#ifndef TINYWHOOP_RATE_ROLL_PITCH
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_RATE_ROLL_PITCH      75   // 750 deg/s maximum
#else
#define TINYWHOOP_RATE_ROLL_PITCH      62   // 620 deg/s maximum
#endif
#endif
#ifndef TINYWHOOP_RC_RATE_YAW
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_RC_RATE_YAW          6    // 60 deg/s centre sensitivity
#else
#define TINYWHOOP_RC_RATE_YAW          5    // 50 deg/s centre sensitivity
#endif
#endif
#ifndef TINYWHOOP_RATE_YAW
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_RATE_YAW             62   // 620 deg/s maximum
#else
#define TINYWHOOP_RATE_YAW             55   // 550 deg/s maximum
#endif
#endif

/*
 * Small props accelerate and decay far faster than a 5", so the feedforward
 * time constants that shape the response to a stick step have to be shorter -
 * pid.c documents the stock 100ms yaw hold time as being sized for a 5" and
 * notes that smaller props want to decay faster. The race variant shortens
 * the hold time further and cuts smoothing again: a track line rewards
 * immediacy over the smoothness that suits freestyle.
 */
#ifndef TINYWHOOP_FEEDFORWARD_YAW_HOLD_TIME
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_FEEDFORWARD_YAW_HOLD_TIME  40
#else
#define TINYWHOOP_FEEDFORWARD_YAW_HOLD_TIME  60
#endif
#endif
#ifndef TINYWHOOP_FEEDFORWARD_SMOOTH_FACTOR
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_FEEDFORWARD_SMOOTH_FACTOR  35   // stock 65; race wants minimal smoothing
#else
#define TINYWHOOP_FEEDFORWARD_SMOOTH_FACTOR  50   // stock 65; less smoothing, less lag
#endif
#endif
#ifndef TINYWHOOP_FEEDFORWARD_JITTER_FACTOR
#define TINYWHOOP_FEEDFORWARD_JITTER_FACTOR  10   // stock 7; whoop links are noisier at low deflection
#endif

/*
 * Thrust linearisation compensates for the strongly non-linear thrust curve of
 * a small prop, where the top of the throttle range produces much less extra
 * thrust per unit of motor output than the bottom. Without it a whoop feels
 * mushy near hover and twitchy up high. 20 is the conservative end of the
 * 20-40 range normally quoted for whoops and toothpicks; race motors are
 * typically run on lighter frames with a flatter curve, so 30 suits them.
 */
#ifndef TINYWHOOP_THRUST_LINEARIZATION
#ifdef USE_TINYWHOOP_RACE
#define TINYWHOOP_THRUST_LINEARIZATION  30
#else
#define TINYWHOOP_THRUST_LINEARIZATION  20
#endif
#endif

/*
 * Throttle boost adds high-pass filtered throttle to the motor output so that
 * throttle steps arrive without waiting for the prop to spool. Small props
 * respond quickly enough to use more of it than a 5".
 */
#ifndef TINYWHOOP_THROTTLE_BOOST
#define TINYWHOOP_THROTTLE_BOOST         8    // stock 5
#endif

/*
 * A ducted whoop stalls and desyncs more easily than an open prop when the
 * throttle is chopped, so idle sits a little above the stock brushless value.
 * Brushed whoops already default to a higher idle and are left alone.
 */
#ifndef TINYWHOOP_MOTOR_IDLE
#define TINYWHOOP_MOTOR_IDLE             650  // 6.5%, stock brushless 550
#endif

/*
 * Whoops hit things. Crashflip (turtle mode) is the difference between walking
 * over to the craft and flying on, so it is turned on with the same rate/percent
 * pairing USE_RACE_PRO already ships as its own crashflip default.
 *
 * crashflip_rate is not cosmetic: mixer.c disables both crashflip attenuators
 * outright whenever it is zero (the stock non-race default), so motor power
 * would never ease off as the craft rights itself, and a fixed 10% would either
 * fail to lift the airframe or keep pushing well past upright. crashflip_rate
 * must be set alongside crashflip_motor_percent or the feature does not do
 * anything close to what a "flip over after crash" mode is supposed to do.
 */
#ifndef TINYWHOOP_CRASHFLIP_MOTOR_PERCENT
#define TINYWHOOP_CRASHFLIP_MOTOR_PERCENT  10
#endif
#ifndef TINYWHOOP_CRASHFLIP_RATE
#define TINYWHOOP_CRASHFLIP_RATE           30
#endif
#ifndef TINYWHOOP_CRASHFLIP_AUTO_REARM
#define TINYWHOOP_CRASHFLIP_AUTO_REARM     true
#endif

#ifndef USE_SIMPLIFIED_TUNING
// Scale a stock cutoff by TINYWHOOP_FILTER_MULTIPLIER. Only needed when the
// simplified tuning sliders are not compiled in and cannot do it for us; the
// arithmetic deliberately matches simplified_tuning.c so both paths land on the
// same cutoffs.
static uint16_t tinywhoopScaleCutoff(uint16_t stockHz, uint16_t maxHz)
{
    return constrain(stockHz * TINYWHOOP_FILTER_MULTIPLIER / 100, 0, maxHz);
}
#endif

void tinywhoopProfileApplyToPidProfile(uint8_t pidProfileIndex)
{
    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    // Raise the D term filter cutoffs. The multiplier is stored as well as
    // applied, so the Configurator filter slider shows the setting that
    // produced these cutoffs rather than reading as a custom tune.
#ifdef USE_SIMPLIFIED_TUNING
    pidProfile->simplified_dterm_filter = true;
    pidProfile->simplified_dterm_filter_multiplier = TINYWHOOP_FILTER_MULTIPLIER;
    applySimplifiedTuningDtermFilters(pidProfile);
#else
    if (pidProfile->dterm_lpf1_dyn_min_hz) {
        pidProfile->dterm_lpf1_dyn_min_hz = tinywhoopScaleCutoff(DTERM_LPF1_DYN_MIN_HZ_DEFAULT, DYN_LPF_MAX_HZ);
        pidProfile->dterm_lpf1_dyn_max_hz = tinywhoopScaleCutoff(DTERM_LPF1_DYN_MAX_HZ_DEFAULT, DYN_LPF_MAX_HZ);
    }
    if (pidProfile->dterm_lpf1_static_hz) {
        pidProfile->dterm_lpf1_static_hz = tinywhoopScaleCutoff(DTERM_LPF1_DYN_MIN_HZ_DEFAULT, DYN_LPF_MAX_HZ);
    }
    if (pidProfile->dterm_lpf2_static_hz) {
        pidProfile->dterm_lpf2_static_hz = tinywhoopScaleCutoff(DTERM_LPF2_HZ_DEFAULT, LPF_MAX_HZ);
    }
#endif

    // Feel: shorter feedforward time constants for a small, fast prop.
#ifdef USE_FEEDFORWARD
    pidProfile->feedforward_yaw_hold_time = TINYWHOOP_FEEDFORWARD_YAW_HOLD_TIME;
    pidProfile->feedforward_smooth_factor = TINYWHOOP_FEEDFORWARD_SMOOTH_FACTOR;
    pidProfile->feedforward_jitter_factor = TINYWHOOP_FEEDFORWARD_JITTER_FACTOR;
#endif

    // Motor response.
#ifdef USE_THRUST_LINEARIZATION
    pidProfile->thrustLinearization = TINYWHOOP_THRUST_LINEARIZATION;
#endif
#ifdef USE_THROTTLE_BOOST
    pidProfile->throttle_boost = TINYWHOOP_THROTTLE_BOOST;
#endif
}

void tinywhoopProfileApplyToRateProfile(uint8_t rateProfileIndex)
{
    controlRateConfig_t *rateProfile = controlRateProfilesMutable(rateProfileIndex);

    // The stock default is already RATES_TYPE_ACTUAL; set it explicitly because
    // the rate values below are only meaningful in that scheme.
    rateProfile->rates_type = RATES_TYPE_ACTUAL;

    rateProfile->rcRates[FD_ROLL]  = TINYWHOOP_RC_RATE_ROLL_PITCH;
    rateProfile->rcRates[FD_PITCH] = TINYWHOOP_RC_RATE_ROLL_PITCH;
    rateProfile->rcRates[FD_YAW]   = TINYWHOOP_RC_RATE_YAW;

    rateProfile->rates[FD_ROLL]  = TINYWHOOP_RATE_ROLL_PITCH;
    rateProfile->rates[FD_PITCH] = TINYWHOOP_RATE_ROLL_PITCH;
    rateProfile->rates[FD_YAW]   = TINYWHOOP_RATE_YAW;
}

#ifdef USE_OSD
/*
 * Stock Betaflight positions every OSD element near the centre of the screen
 * and leaves all but the warnings hidden, which means a freshly flashed craft
 * shows nothing but warnings until the user lays the screen out by hand. A
 * whoop is flown on the pack it took off with and lands when the cells sag, so
 * the useful minimum is: how full is the battery, how good is the link, how
 * long have I been flying - placed in the corners, out of the way of the
 * centre of the frame.
 */
static void tinywhoopProfileApplyOsdLayout(void)
{
    osdElementConfig_t *elements = osdElementConfigMutable();

    uint16_t profileFlags = 0;
    for (unsigned i = 1; i <= OSD_PROFILE_COUNT; i++) {
        profileFlags |= OSD_PROFILE_FLAG(i);
    }

    // Canvas extents: 53x20 for HD, 30x16 for the SD (PAL) character buffer.
    // Positions past the edge of the display actually in use - an NTSC screen
    // is only 13 rows - are pulled back onto the canvas by osdInit() once the
    // display port reports its real size, so anchoring to the buffer edge is
    // safe on every combination.
#ifdef USE_OSD_HD
    const uint8_t lastCol = 52;
    const uint8_t lastRow = 19;
#else
    const uint8_t lastCol = 29;
    const uint8_t lastRow = 15;
#endif

    // Top left: link quality. Top right: flight mode.
    elements->item_pos[OSD_LINK_QUALITY] = OSD_POS(1, 1) | profileFlags;
    elements->item_pos[OSD_FLYMODE]      = OSD_POS(lastCol - 6, 1) | profileFlags;

    // Bottom left: average cell voltage - the number a whoop pilot lands on.
    // Bottom right: flight timer. Warnings keep their centred stock position.
    elements->item_pos[OSD_AVG_CELL_VOLTAGE] = OSD_POS(1, lastRow) | profileFlags;
    elements->item_pos[OSD_ITEM_TIMER_2]     = OSD_POS(lastCol - 6, lastRow) | profileFlags;
}
#endif // USE_OSD

void tinywhoopProfileApply(void)
{
    // Filters: raise the gyro cutoffs for the same reason as the D term ones.
    gyroConfig_t *gyro = gyroConfigMutable();
#ifdef USE_SIMPLIFIED_TUNING
    gyro->simplified_gyro_filter = true;
    gyro->simplified_gyro_filter_multiplier = TINYWHOOP_FILTER_MULTIPLIER;
    applySimplifiedTuningGyroFilters(gyro);
#else
    if (gyro->gyro_lpf1_dyn_min_hz) {
        gyro->gyro_lpf1_dyn_min_hz = tinywhoopScaleCutoff(GYRO_LPF1_DYN_MIN_HZ_DEFAULT, DYN_LPF_MAX_HZ);
        gyro->gyro_lpf1_dyn_max_hz = tinywhoopScaleCutoff(GYRO_LPF1_DYN_MAX_HZ_DEFAULT, DYN_LPF_MAX_HZ);
    }
    if (gyro->gyro_lpf1_static_hz) {
        gyro->gyro_lpf1_static_hz = tinywhoopScaleCutoff(GYRO_LPF1_DYN_MIN_HZ_DEFAULT, DYN_LPF_MAX_HZ);
    }
    if (gyro->gyro_lpf2_static_hz) {
        gyro->gyro_lpf2_static_hz = tinywhoopScaleCutoff(GYRO_LPF2_HZ_DEFAULT, LPF_MAX_HZ);
    }
#endif

#ifdef USE_DYN_NOTCH_FILTER
    dynNotchConfigMutable()->dyn_notch_min_hz = TINYWHOOP_DYN_NOTCH_MIN_HZ;
    dynNotchConfigMutable()->dyn_notch_max_hz = TINYWHOOP_DYN_NOTCH_MAX_HZ;
    dynNotchConfigMutable()->dyn_notch_count = TINYWHOOP_DYN_NOTCH_COUNT;
#endif

#ifdef USE_RC_SMOOTHING_FILTER
    rxConfigMutable()->rc_smoothing_auto_factor_rpy = TINYWHOOP_RC_SMOOTHING_AUTO_FACTOR_RPY;
#endif

    // A ducted prop needs a little more idle than an open one to stay spinning
    // through a hard throttle chop. Brushed targets already idle higher.
    if (motorConfig()->dev.motorProtocol != MOTOR_PROTOCOL_BRUSHED) {
        motorConfigMutable()->motorIdle = TINYWHOOP_MOTOR_IDLE;
    }

    mixerConfigMutable()->crashflip_motor_percent = TINYWHOOP_CRASHFLIP_MOTOR_PERCENT;
    mixerConfigMutable()->crashflip_rate = TINYWHOOP_CRASHFLIP_RATE;
    mixerConfigMutable()->crashflip_auto_rearm = TINYWHOOP_CRASHFLIP_AUTO_REARM;

    for (uint8_t i = 0; i < PID_PROFILE_COUNT; i++) {
        tinywhoopProfileApplyToPidProfile(i);
    }

    for (uint8_t i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        tinywhoopProfileApplyToRateProfile(i);
    }

#ifdef USE_OSD
    tinywhoopProfileApplyOsdLayout();
#endif
}

#endif // USE_TINYWHOOP
