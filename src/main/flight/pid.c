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


void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
void pidMultiWiiRewrite(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);
void pidMultiWii23(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);

pidControllerFuncPtr pid_controller = pidMultiWiiRewrite;

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);

PG_RESET_TEMPLATE(pidProfile_t, pidProfile,
    .pidController = PID_CONTROLLER_MWREWRITE,
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
    .D8[PIDLEVEL] = 100,
    .P8[PIDMAG] = 40,
    .P8[PIDVEL] = 120,
    .I8[PIDVEL] = 45,
    .D8[PIDVEL] = 1,

    .yaw_p_limit = YAW_P_LIMIT_MAX,
    .dterm_lpf = 100,   // DTERM filtering ON by default
    .yaw_lpf = 80,
    .deltaMethod = PID_DELTA_FROM_MEASUREMENT,
);

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
        case PID_CONTROLLER_MWREWRITE:
            pid_controller = pidMultiWiiRewrite;
            break;
#ifndef SKIP_PID_LUXFLOAT
        case PID_CONTROLLER_LUX_FLOAT:
            pid_controller = pidLuxFloat;
            break;
#endif
#ifndef SKIP_PID_MW23
        case PID_CONTROLLER_MW23:
            pid_controller = pidMultiWii23;
            break;
#endif
    }
}
