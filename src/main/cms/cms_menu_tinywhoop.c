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
 * In-goggle tuning menu for tiny whoops.
 *
 * A whoop is flown in packs of two or three minutes in a room, not on a bench
 * next to a laptop, and the things that want changing between those packs are
 * always the same handful: how fast it turns, how sharp it feels, and how much
 * filtering is in the way. Betaflight already exposes all of it, but spread
 * across the PID, rate and filter menus. This gathers just those knobs into one
 * screen, and adds a single entry that puts the whole craft back on the whoop
 * defaults if a session of experimenting went nowhere.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_TINYWHOOP)

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_tinywhoop.h"

#include "config/config.h"
#include "config/simplified_tuning.h"
#include "config/tinywhoop_profile.h"

#include "fc/controlrate_profile.h"

#include "flight/pid.h"

#include "pg/motor.h"
#include "pg/rx.h"

#include "sensors/gyro.h"

// Staged copies of the settings on screen, written back on exit.
static uint8_t whoopRcRateRollPitch;
static uint8_t whoopRateRollPitch;
static uint8_t whoopRcRateYaw;
static uint8_t whoopRateYaw;
static uint8_t whoopRcExpo;

#ifdef USE_SIMPLIFIED_TUNING
static uint8_t whoopFilterMultiplier;
#endif
#ifdef USE_RC_SMOOTHING_FILTER
static uint8_t whoopRcSmoothingFactor;
#endif

#ifdef USE_THROTTLE_BOOST
static uint8_t whoopThrottleBoost;
#endif
#ifdef USE_THRUST_LINEARIZATION
static uint8_t whoopThrustLinearization;
#endif
static uint16_t whoopMotorIdle;

static void cmsx_TinyWhoop_readSettings(void)
{
    const controlRateConfig_t *rateProfile = currentControlRateProfile;

    whoopRcRateRollPitch = rateProfile->rcRates[FD_ROLL];
    whoopRateRollPitch = rateProfile->rates[FD_ROLL];
    whoopRcRateYaw = rateProfile->rcRates[FD_YAW];
    whoopRateYaw = rateProfile->rates[FD_YAW];
    whoopRcExpo = rateProfile->rcExpo[FD_ROLL];

#ifdef USE_SIMPLIFIED_TUNING
    whoopFilterMultiplier = gyroConfig()->simplified_gyro_filter_multiplier;
#endif
#ifdef USE_RC_SMOOTHING_FILTER
    whoopRcSmoothingFactor = rxConfig()->rc_smoothing_auto_factor_rpy;
#endif

#ifdef USE_THROTTLE_BOOST
    whoopThrottleBoost = currentPidProfile->throttle_boost;
#endif
#ifdef USE_THRUST_LINEARIZATION
    whoopThrustLinearization = currentPidProfile->thrustLinearization;
#endif
    whoopMotorIdle = motorConfig()->motorIdle;
}

static const void *cmsx_TinyWhoop_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_TinyWhoop_readSettings();

    return NULL;
}

static const void *cmsx_TinyWhoop_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    controlRateConfig_t *rateProfile = currentControlRateProfile;

    // Roll and pitch are kept together: a whoop that turns at different rates
    // on the two axes is a tuning mistake, not a preference.
    rateProfile->rcRates[FD_ROLL] = whoopRcRateRollPitch;
    rateProfile->rcRates[FD_PITCH] = whoopRcRateRollPitch;
    rateProfile->rates[FD_ROLL] = whoopRateRollPitch;
    rateProfile->rates[FD_PITCH] = whoopRateRollPitch;
    rateProfile->rcRates[FD_YAW] = whoopRcRateYaw;
    rateProfile->rates[FD_YAW] = whoopRateYaw;
    rateProfile->rcExpo[FD_ROLL] = whoopRcExpo;
    rateProfile->rcExpo[FD_PITCH] = whoopRcExpo;

#ifdef USE_THROTTLE_BOOST
    currentPidProfile->throttle_boost = whoopThrottleBoost;
#endif
#ifdef USE_THRUST_LINEARIZATION
    currentPidProfile->thrustLinearization = whoopThrustLinearization;
#endif
    motorConfigMutable()->motorIdle = whoopMotorIdle;

#ifdef USE_RC_SMOOTHING_FILTER
    rxConfigMutable()->rc_smoothing_auto_factor_rpy = whoopRcSmoothingFactor;
#endif

    // One number drives both filter sliders, so that "less filtering" is a
    // single decision rather than two that have to be kept in step. Only the
    // two filter appliers are called, never applySimplifiedTuning(): changing a
    // filter setting must not quietly rewrite the pilot's PID gains from the
    // PID sliders as well.
#ifdef USE_SIMPLIFIED_TUNING
    if (gyroConfig()->simplified_gyro_filter_multiplier != whoopFilterMultiplier) {
        gyroConfigMutable()->simplified_gyro_filter = true;
        gyroConfigMutable()->simplified_gyro_filter_multiplier = whoopFilterMultiplier;
        currentPidProfile->simplified_dterm_filter = true;
        currentPidProfile->simplified_dterm_filter_multiplier = whoopFilterMultiplier;

        applySimplifiedTuningGyroFilters(gyroConfigMutable());
        applySimplifiedTuningDtermFilters(currentPidProfile);
    }
#endif

    return NULL;
}

static const void *cmsx_TinyWhoop_restoreDefaults(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    tinywhoopProfileApply();

    // Pull the freshly applied values back onto the screen so the menu shows
    // what was just written rather than what the pilot had been editing.
    cmsx_TinyWhoop_readSettings();

    return NULL;
}

static const OSD_Entry cmsx_menuTinyWhoopEntries[] =
{
    { "-- WHOOP TUNE --", OME_Label, NULL, NULL },

    // Rates are ACTUAL: centre sensitivity and maximum rate, both in deg/s / 10.
    { "CENTER R/P", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopRcRateRollPitch, 1, 200, 1 } },
    { "RATE R/P",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopRateRollPitch,   1, 200, 1 } },
    { "CENTER YAW", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopRcRateYaw,       1, 200, 1 } },
    { "RATE YAW",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopRateYaw,         1, 200, 1 } },
    { "EXPO R/P",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopRcExpo,          0, 100, 1 } },

    { "-- RESPONSE --", OME_Label, NULL, NULL },

    // Higher multiplier = higher cutoffs = less filter delay, but more noise
    // reaching the motors. Takes effect once the filters are rebuilt.
#ifdef USE_SIMPLIFIED_TUNING
    { "FILTER PCT", OME_UINT8 | REBOOT_REQUIRED, NULL,
        &(OSD_UINT8_t) { &whoopFilterMultiplier, SIMPLIFIED_TUNING_FILTERS_MIN, SIMPLIFIED_TUNING_MAX, 5 } },
#endif
    // Lower factor = higher RC smoothing cutoff = less stick-to-motor delay.
#ifdef USE_RC_SMOOTHING_FILTER
    { "RC SMOOTH",  OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t)  { &whoopRcSmoothingFactor, 0, 250, 5 } },
#endif
#ifdef USE_THROTTLE_BOOST
    { "THR BOOST",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopThrottleBoost,        0, 100, 1 } },
#endif
#ifdef USE_THRUST_LINEARIZATION
    { "THRUST LIN", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &whoopThrustLinearization,  0, 150, 5 } },
#endif
    { "MOTOR IDLE", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t) { &whoopMotorIdle, 0, 2000, 10 } },

    { "-- PRESET --", OME_Label, NULL, NULL },
    { "WHOOP DEFAULTS", OME_Funcall | REBOOT_REQUIRED, cmsx_TinyWhoop_restoreDefaults, NULL },

    { "SAVE&REBOOT", OME_OSD_Exit, cmsMenuExit, (void *)CMS_POPUP_SAVEREBOOT },
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL }
};

CMS_Menu cmsx_menuTinyWhoop = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUWHOOP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_TinyWhoop_onEnter,
    .onExit = cmsx_TinyWhoop_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuTinyWhoopEntries,
};

#endif // USE_CMS && USE_TINYWHOOP
