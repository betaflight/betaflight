/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"
#include "config/profile.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "fc/rc_controls.h"
#include "fc/rate_profile.h"

#include "flight/pid.h"
#include "flight/imu.h"

uint32_t targetPidLooptime;

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t PIDweight[3];

int32_t lastITerm[3], ITermLimit[3];
float lastITermf[3], ITermLimitf[3];

pt1Filter_t deltaFilter[3];
pt1Filter_t yawFilter;
biquadFilter_t dtermFilterLpf[3];
biquadFilter_t dtermFilterNotch[3];
bool dtermNotchInitialised, dtermBiquadLpfInitialised;


void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
#ifdef USE_PID_MWREWRITE
void pidMultiWiiRewrite(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
#endif
#ifdef USE_PID_MW23
void pidMultiWii23(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
#endif

pidControllerFuncPtr pid_controller = pidLuxFloat;

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);

PG_RESET_TEMPLATE(pidProfile_t, pidProfile,
    .pidController = PID_CONTROLLER_LUX_FLOAT,
    .P8[PIDROLL] = 40,
    .I8[PIDROLL] = 30,
    .D8[PIDROLL] = 23,
    .P8[PIDPITCH] = 40,
    .I8[PIDPITCH] = 30,
    .D8[PIDPITCH] = 23,
    .P8[PIDYAW] = 85,
    .I8[PIDYAW] = 45,
    .D8[PIDYAW] = 0,
    .P8[PIDALT] = 50,
    .I8[PIDALT] = 0,
    .D8[PIDALT] = 0,
    .P8[PIDPOS] = 15,   // POSHOLD_P * 100
    .I8[PIDPOS] = 0,    // POSHOLD_I * 100
    .D8[PIDPOS] = 0,
    .P8[PIDPOSR] = 34,  // POSHOLD_RATE_P * 10
    .I8[PIDPOSR] = 14,  // POSHOLD_RATE_I * 100
    .D8[PIDPOSR] = 53,  // POSHOLD_RATE_D * 1000
    .P8[PIDNAVR] = 25,  // NAV_P * 10
    .I8[PIDNAVR] = 33,  // NAV_I * 100
    .D8[PIDNAVR] = 83,  // NAV_D * 1000
    .P8[PIDLEVEL] = 20,
    .I8[PIDLEVEL] = 10,
    .D8[PIDLEVEL] = 75,
    .P8[PIDMAG] = 40,
    .P8[PIDVEL] = 120,
    .I8[PIDVEL] = 45,
    .D8[PIDVEL] = 1,

    .yaw_p_limit = YAW_P_LIMIT_MAX,
    .yaw_lpf_hz = 0,
    .dterm_filter_type = FILTER_BIQUAD,
    .dterm_lpf_hz = 100,    // filtering ON by default
    .dterm_notch_hz = 260,
    .dterm_notch_cutoff = 160,
    .deltaMethod = PID_DELTA_FROM_MEASUREMENT,
    .horizon_tilt_effect = 75,
    .horizon_tilt_mode = HORIZON_TILT_MODE_SAFE,
);


static float dT;

#ifdef UNIT_TEST
STATIC_UNIT_TESTED void pidResetDt(void)
{
    dT = 0.0f;
}
#endif
float getdT(void)
{
    if (!dT) dT = (float)targetPidLooptime * 0.000001f;

    return dT;
}

void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
}

void pidInitFilters(const pidProfile_t *pidProfile)
{
    int axis;

    if (pidProfile->dterm_notch_hz && !dtermNotchInitialised) {
        for (axis = 0; axis < 3; axis++) biquadFilterInitNotch(&dtermFilterNotch[axis], pidProfile->dterm_notch_hz, targetPidLooptime, pidProfile->dterm_notch_hz, pidProfile->dterm_notch_cutoff);
        dtermNotchInitialised = true;
    }

    if (pidProfile->dterm_filter_type == FILTER_BIQUAD) {
        if (pidProfile->dterm_lpf_hz && !dtermBiquadLpfInitialised) {
            for (axis = 0; axis < 3; axis++) biquadFilterInitLPF(&dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
            dtermBiquadLpfInitialised = true;
        }
    }
}

void pidResetITerm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        lastITerm[axis] = 0;
        lastITermf[axis] = 0.0f;
    }
}

void pidSetController(pidControllerType_e type)
{
    switch (type) {
        default:
        case PID_CONTROLLER_LUX_FLOAT:
            pid_controller = pidLuxFloat;
            break;
#ifdef USE_PID_MWREWRITE
        case PID_CONTROLLER_MWREWRITE:
            pid_controller = pidMultiWiiRewrite;
            break;
#endif
#ifdef USE_PID_MW23
        case PID_CONTROLLER_MW23:
            pid_controller = pidMultiWii23;
            break;
#endif
    }
}

// calculates strength of horizon leveling; 0 = none, 100 = most leveling
int calcHorizonLevelStrength(uint16_t rxConfigMidrc, int horizonTiltEffect,
        uint8_t horizonTiltMode, int horizonSensitivity)
{
    // get raw stick positions (-500 to 500):
    const int32_t stickPosAil = getRcStickDeflection(FD_ROLL, rxConfigMidrc);
    const int32_t stickPosEle = getRcStickDeflection(FD_PITCH, rxConfigMidrc);

    // 0 at center stick, 500 at max stick deflection:
    const int32_t mostDeflectedPos = MAX(ABS(stickPosAil), ABS(stickPosEle));

    // start with 100 at center stick, 0 at max stick deflection:
    int horizonLevelStrength = (500 - mostDeflectedPos) / 5;

    // 0 at level, 900 at vertical, 1800 at inverted (degrees * 10)
    const int currentInclination = MAX(ABS(attitude.values.roll),
                                                ABS(attitude.values.pitch));

    // horizonTiltMode:  SAFE = leveling always active when sticks centered,
    // EXPERT = leveling can be totally off when inverted
    if (horizonTiltMode == HORIZON_TILT_MODE_EXPERT) {
        if (horizonTiltEffect < 175) {
            // horizonTiltEffect 0 to 125 => 2700 to 900
            //  (represents where leveling goes to zero):
            const int cutoffDeciDegrees = (175-horizonTiltEffect) * 18;
            // inclinationLevelRatio (0 to 100) is smaller (less leveling)
            //  for larger inclinations; 0 at cutoffDeciDegrees value:
            const int inclinationLevelRatio = constrain(
                              (((cutoffDeciDegrees-currentInclination)*10) /
                                           (cutoffDeciDegrees/10)), 0, 100);
            // apply configured horizon sensitivity:
            if (horizonSensitivity <= 0) {       // zero means no leveling
                horizonLevelStrength = 0;
            } else {
                // when stick is near center (horizonLevelStrength ~= 100)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0)
                //  H_sensitivity value has more effect:
                horizonLevelStrength = (horizonLevelStrength - 100) *
                        100 / horizonSensitivity + 100;
            }
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength = horizonLevelStrength * inclinationLevelRatio / 100;
        }
        else
          horizonLevelStrength = 0;
    } else {  // horizon_tilt_mode = SAFE (leveling always active when sticks centered)
        int sensitFact;
        if (horizonTiltEffect > 0) {
            // ratio of 100 to 0 (larger means more leveling):
            const int factorRatio = 100 - horizonTiltEffect;
            // inclinationLevelRatio (0 to 100) is smaller (less leveling)
            //  for larger inclinations, goes to 100 at inclination level:
            int inclinationLevelRatio =
                  ((1800-currentInclination)/18 * (100-factorRatio)) / 100 + factorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizonSensitivity * inclinationLevelRatio / 100;
        }
        else   // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizonSensitivity;

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 100)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0)
            //  sensitFact value has more effect:
            horizonLevelStrength =  (horizonLevelStrength - 100) * 100 / sensitFact + 100;
        }
    }
    return constrain(horizonLevelStrength, 0, 100);
}
