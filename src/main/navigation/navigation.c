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
#include <math.h>


#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"


/*-----------------------------------------------------------
 * Compatibility for home position
 *-----------------------------------------------------------*/
gpsLocation_t GPS_home;
uint16_t      GPS_distanceToHome;        // distance to home point in meters
int16_t       GPS_directionToHome;       // direction to home point in degrees

#if defined(NAV)
#if defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE)
PG_DECLARE_ARRAY(navWaypoint_t, NAV_MAX_WAYPOINTS, nonVolatileWaypointList);
PG_REGISTER_ARRAY(navWaypoint_t, NAV_MAX_WAYPOINTS, nonVolatileWaypointList, PG_WAYPOINT_MISSION_STORAGE, 0);
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(navConfig_t, navConfig, PG_NAV_CONFIG, 1);

PG_RESET_TEMPLATE(navConfig_t, navConfig,
    .general = {

        .flags = {
            .use_thr_mid_for_althold = 0,
            .extra_arming_safety = 1,
            .user_control_mode = NAV_GPS_ATTI,
            .rth_alt_control_mode = NAV_RTH_AT_LEAST_ALT,
            .rth_climb_first = 1,                   // Climb first, turn after reaching safe altitude
            .rth_climb_ignore_emerg = 0,            // Ignore GPS loss on initial climb
            .rth_tail_first = 0,
            .disarm_on_landing = 0,
            .rth_allow_landing = 1,
        },

        // General navigation parameters
        .pos_failure_timeout = 5,     // 5 sec
        .waypoint_radius = 100,       // 2m diameter
        .waypoint_safe_distance = 10000,    // 100m - first waypoint should be closer than this
        .max_auto_speed = 300,             // 3 m/s = 10.8 km/h
        .max_auto_climb_rate = 500,        // 5 m/s
        .max_manual_speed = 500,
        .max_manual_climb_rate = 200,
        .land_descent_rate = 200,     // 2 m/s
        .land_slowdown_minalt = 500,  // 5 meters of altitude
        .land_slowdown_maxalt = 2000, // 20 meters of altitude
        .emerg_descent_rate = 500,    // 5 m/s
        .min_rth_distance = 500,      // If closer than 5m - land immediately
        .rth_altitude = 1000,         // 10m
        .rth_abort_threshold = 50000, // 500m - should be safe for all aircraft
    },

    // MC-specific
    .mc = {
        .max_bank_angle = 30,      // 30 deg
        .hover_throttle = 1500,
        .auto_disarm_delay = 2000,
    },

    // Fixed wing
    .fw = {
        .max_bank_angle = 20,      // 30 deg
        .max_climb_angle = 20,
        .max_dive_angle = 15,
        .cruise_throttle = 1400,
        .max_throttle = 1700,
        .min_throttle = 1200,
        .pitch_to_throttle = 10,   // pwm units per degree of pitch (10pwm units ~ 1% throttle)
        .loiter_radius = 5000,     // 50m

        //Fixed wing landing
        .land_dive_angle = 2,   // 2 degrees dive by default

        // Fixed wing launch
        .launch_velocity_thresh = 300,         // 3 m/s
        .launch_accel_thresh = 1.9f * 981,     // cm/s/s (1.9*G)
        .launch_time_thresh = 40,              // 40ms
        .launch_throttle = 1700,
        .launch_idle_throttle = 1000,          // Motor idle or MOTOR_STOP
        .launch_motor_timer = 500,             // ms
        .launch_motor_spinup_time = 100,       // ms, time to gredually increase throttle from idle to launch
        .launch_timeout = 5000,                // ms, timeout for launch procedure
        .launch_climb_angle = 18,              // 18 degrees
        .launch_max_angle = 45                 // 45 deg
    }
);

navigationPosControl_t  posControl;
navSystemStatus_t       NAV_Status;

#if defined(NAV_BLACKBOX)
int16_t navCurrentState;
int16_t navActualVelocity[3];
int16_t navDesiredVelocity[3];
int16_t navActualHeading;
int16_t navDesiredHeading;
int16_t navTargetPosition[3];
int32_t navLatestActualPosition[3];
int16_t navTargetSurface;
int16_t navActualSurface;
uint16_t navFlags;
uint16_t navEPH;
uint16_t navEPV;
int16_t navAccNEU[3];
#endif

static void updateDesiredRTHAltitude(void);
static void resetAltitudeController(void);
static void resetPositionController(void);
static void setupAltitudeController(void);
void resetNavigation(void);
void resetGCSFlags(void);

static void calcualteAndSetActiveWaypoint(const navWaypoint_t * waypoint);
static void calcualteAndSetActiveWaypointToLocalPosition(const t_fp_vector * pos);
void calculateInitialHoldPosition(t_fp_vector * pos);
void calculateFarAwayTarget(t_fp_vector * farAwayPos, int32_t yaw, int32_t distance);

void initializeRTHSanityChecker(const t_fp_vector * pos);
bool validateRTHSanityChecker(void);

/*************************************************************************************************/
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_IDLE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HEAD_HOME(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_REACHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_NEXT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_RTH_LAND(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_WAIT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_IN_PROGRESS(navigationFSMState_t previousState);

static const navigationFSMStateDescriptor_t navFSM[NAV_STATE_COUNT] = {
    /** Idle state ******************************************************/
    [NAV_STATE_IDLE] = {
        .onEntry = navOnEnteringState_NAV_STATE_IDLE,
        .timeoutMs = 0,
        .stateFlags = 0,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_LAUNCH]            = NAV_STATE_LAUNCH_INITIALIZE,
        }
    },

    /** ALTHOLD mode ***************************************************/
    [NAV_STATE_ALTHOLD_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_ANGLE_FW | NAV_REQUIRE_THRTILT,
        .mapToFlightModes = NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_ALTHOLD_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_ALTHOLD_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_ANGLE_FW | NAV_REQUIRE_THRTILT | NAV_RC_ALT,
        .mapToFlightModes = NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_ALTHOLD_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** POSHOLD_2D mode ************************************************/
    [NAV_STATE_POSHOLD_2D_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_2D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_POSHOLD_MODE,
        .mwState = MW_NAV_STATE_HOLD_INFINIT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_POSHOLD_2D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_POSHOLD_2D_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_2D_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_POSHOLD_MODE,
        .mwState = MW_NAV_STATE_HOLD_INFINIT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_POSHOLD_2D_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** POSHOLD_3D mode ************************************************/
    [NAV_STATE_POSHOLD_3D_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_REQUIRE_ANGLE | NAV_REQUIRE_THRTILT,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE,
        .mwState = MW_NAV_STATE_HOLD_INFINIT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_POSHOLD_3D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_POSHOLD_3D_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_THRTILT | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE,
        .mwState = MW_NAV_STATE_HOLD_INFINIT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_POSHOLD_3D_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** RTH_3D mode ************************************************/
    [NAV_STATE_RTH_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_INITIALIZE,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_INITIALIZE,      // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING]       = NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_CLIMB_TO_SAFE_ALT] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,     // allow pos adjustment while climbind to safe alt
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_WAIT_FOR_RTH_ALT,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_CLIMB_TO_SAFE_ALT,   // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_HEAD_HOME] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_HEAD_HOME,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_HEAD_HOME,           // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
        .timeoutMs = 500,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_SETTLE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_LANDING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_LANDING,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_CTL_LAND | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_LANDING,         // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_FINISHING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_FINISHING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_FINISHING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_FINISHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LANDED,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_FINISHED,         // re-process state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** WAYPOINT mode ************************************************/
    [NAV_STATE_WAYPOINT_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_PRE_ACTION,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_PRE_ACTION] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_WP_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_WAYPOINT_IN_PROGRESS,   // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_REACHED,       // successfully reached waypoint
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_WAYPOINT_REACHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_REACHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_WAYPOINT_REACHED,   // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_NEXT,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND] = NAV_STATE_WAYPOINT_RTH_LAND,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_WAYPOINT_RTH_LAND] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_RTH_LAND,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_CTL_LAND | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_WAYPOINT_RTH_LAND,   // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_WAYPOINT_NEXT] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_NEXT,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_PRE_ACTION,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_WP_ENROUTE,
        .mwError = MW_NAV_ERROR_FINISH,
        .onEvent = {
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** EMERGENCY LANDING ************************************************/
    [NAV_STATE_EMERGENCY_LANDING_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_LAND_START,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_IDLE,   // ALTHOLD also bails out from emergency (to IDLE, AltHold will take over from there)
        }
    },

    [NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_EMERGENCY_LANDING_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_IDLE,   // ALTHOLD also bails out from emergency (to IDLE, AltHold will take over from there)
        }
    },

    [NAV_STATE_EMERGENCY_LANDING_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .mwState = MW_NAV_STATE_LANDED,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_EMERGENCY_LANDING_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_LAUNCH_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_LAUNCH_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_LAUNCH_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_LAUNCH_WAIT,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_LAUNCH_WAIT] = {
        .onEntry = navOnEnteringState_NAV_STATE_LAUNCH_WAIT,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_LAUNCH | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_LAUNCH_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_LAUNCH_WAIT,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_LAUNCH_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_LAUNCH_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_LAUNCH_IN_PROGRESS,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_LAUNCH | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = NAV_LAUNCH_MODE,
        .mwState = MW_NAV_STATE_NONE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_LAUNCH_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },
};

static navigationFSMStateFlags_t navGetStateFlags(navigationFSMState_t state)
{
    return navFSM[state].stateFlags;
}

static flightModeFlags_e navGetMappedFlightModes(navigationFSMState_t state)
{
    return navFSM[state].mapToFlightModes;
}

navigationFSMStateFlags_t navGetCurrentStateFlags(void)
{
    return navGetStateFlags(posControl.navState);
}

/*************************************************************************************************/
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_IDLE(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    resetNavigation();
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE(navigationFSMState_t previousState)
{
    const navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    resetGCSFlags();

    if ((prevFlags & NAV_CTL_ALT) == 0) {
        resetAltitudeController();
    }

    if (((prevFlags & NAV_CTL_ALT) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0)) {
        setupAltitudeController();
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);  // This will reset surface offset
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If GCS was disabled - reset altitude setpoint
    if (posControl.flags.isGCSAssistedNavigationReset) {
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
        resetGCSFlags();
    }

    // If we enable terrain mode and surface offset is not set yet - do it
    if (posControl.flags.hasValidSurfaceSensor && posControl.flags.isTerrainFollowEnabled && posControl.desiredState.surface < 0) {
        setDesiredSurfaceOffset(posControl.actualState.surface);
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_INITIALIZE(navigationFSMState_t previousState)
{
    const navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    resetGCSFlags();

    if ((prevFlags & NAV_CTL_POS) == 0) {
        resetPositionController();
    }

    if (((prevFlags & NAV_CTL_POS) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0)) {
        t_fp_vector targetHoldPos;
        calculateInitialHoldPosition(&targetHoldPos);
        setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If GCS was disabled - reset 2D pos setpoint
    if (posControl.flags.isGCSAssistedNavigationReset) {
        t_fp_vector targetHoldPos;
        calculateInitialHoldPosition(&targetHoldPos);
        setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
        resetGCSFlags();
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE(navigationFSMState_t previousState)
{
    const navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    resetGCSFlags();

    if ((prevFlags & NAV_CTL_POS) == 0) {
        resetPositionController();
    }

    if ((prevFlags & NAV_CTL_ALT) == 0) {
        resetAltitudeController();
        setupAltitudeController();
    }

    if (((prevFlags & NAV_CTL_ALT) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0)) {
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);  // This will reset surface offset
    }

    if (((prevFlags & NAV_CTL_POS) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0)) {
        t_fp_vector targetHoldPos;
        calculateInitialHoldPosition(&targetHoldPos);
        setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If GCS was disabled - reset 2D pos setpoint
    if (posControl.flags.isGCSAssistedNavigationReset) {
        t_fp_vector targetHoldPos;
        calculateInitialHoldPosition(&targetHoldPos);
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
        setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
        resetGCSFlags();
    }

    // If we enable terrain mode and surface offset is not set yet - do it
    if (posControl.flags.hasValidSurfaceSensor && posControl.flags.isTerrainFollowEnabled && posControl.desiredState.surface < 0) {
        setDesiredSurfaceOffset(posControl.actualState.surface);
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_INITIALIZE(navigationFSMState_t previousState)
{
    navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    if (!posControl.flags.hasValidHeadingSensor || !posControl.flags.hasValidAltitudeSensor || !STATE(GPS_FIX_HOME)) {
        // Heading sensor, altitude sensor and HOME fix are mandatory for RTH. If not satisfied - switch to emergency landing
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if (STATE(FIXED_WING) && (posControl.homeDistance < navConfig()->general.min_rth_distance) && !posControl.flags.forcedRTHActivated) {
        // Prevent RTH from activating on airplanes if too close to home unless it's a failsafe RTH
        return NAV_FSM_EVENT_SWITCH_TO_IDLE;
    }

    // If we have valid position sensor or configured to ignore it's loss at initial stage - continue
    if (posControl.flags.hasValidPositionSensor || navConfig()->general.flags.rth_climb_ignore_emerg) {
        // Reset altitude and position controllers if necessary
        if ((prevFlags & NAV_CTL_POS) == 0) {
            resetPositionController();
        }

        if ((prevFlags & NAV_CTL_ALT) == 0) {
            resetAltitudeController();
            setupAltitudeController();
        }

        // If close to home - reset home position and land
        if (posControl.homeDistance < navConfig()->general.min_rth_distance) {
            setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
            setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);

            return NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING;   // NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING
        }
        else {
            t_fp_vector targetHoldPos;

            if (STATE(FIXED_WING)) {
                // Airplane - climbout before turning around
                calculateFarAwayTarget(&targetHoldPos, posControl.actualState.yaw, 100000.0f);  // 1km away
            } else {
                // Multicopter, hover and climb
                calculateInitialHoldPosition(&targetHoldPos);

                // Initialize RTH sanity check to prevent fly-aways on RTH
                // For airplanes this is delayed until climb-out is finished
                initializeRTHSanityChecker(&targetHoldPos);
            }

            setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);

            return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_CLIMB_TO_SAFE_ALT
        }
    }
    /* Position sensor failure timeout - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    /* No valid POS sensor but still within valid timeout - wait */
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!posControl.flags.hasValidHeadingSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // If we have valid pos sensor OR we are configured to ignore GPS loss
    if (posControl.flags.hasValidPositionSensor || !checkForPositionSensorTimeout() || navConfig()->general.flags.rth_climb_ignore_emerg) {
        const float rthAltitudeMargin = STATE(FIXED_WING) ?
                            MIN(100.0f, 0.10f * ABS(posControl.homeWaypointAbove.pos.V.Z - posControl.homePosition.pos.V.Z)) :  // Airplane: 10% of target altitude but no less than 1m
                            MIN( 50.0f, 0.05f * ABS(posControl.homeWaypointAbove.pos.V.Z - posControl.homePosition.pos.V.Z));   // Copters:   5% of target altitude but no less than 50cm

        if (((posControl.actualState.pos.V.Z - posControl.homeWaypointAbove.pos.V.Z) > -rthAltitudeMargin) || (!navConfig()->general.flags.rth_climb_first)) {
            // Delayed initialization for RTH sanity check on airplanes - allow to finish climb first as it can take some distance
            if (STATE(FIXED_WING)) {
                initializeRTHSanityChecker(&posControl.actualState.pos);
            }

            return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_HEAD_HOME
        }
        else {
            /* For multi-rotors execute sanity check during initial ascent as well */
            if (!STATE(FIXED_WING)) {
                if (!validateRTHSanityChecker()) {
                    return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
                }
            }

            // Climb to safe altitude and turn to correct direction
            if (STATE(FIXED_WING)) {
                setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_Z);
            }
            else {
                if (navConfig()->general.flags.rth_tail_first) {
                    setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
                }
                else {
                    setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
                }
            }

            return NAV_FSM_EVENT_NONE;
        }
    }
    /* Position sensor failure timeout - land */
    else {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!posControl.flags.hasValidHeadingSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // If we have position sensor - continue home
    if (posControl.flags.hasValidPositionSensor) {
        if (isWaypointReached(&posControl.homeWaypointAbove, true)) {
            // Successfully reached position target - update XYZ-position
            setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
            return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING
        }
        else if (!validateRTHSanityChecker()) {
            // Sanity check of RTH
            return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
        }
        else {
            // Update XYZ-position target
            if (navConfig()->general.flags.rth_tail_first && !STATE(FIXED_WING)) {
                setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
            }
            else {
                setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
            }
            return NAV_FSM_EVENT_NONE;
        }
    }
    /* Position sensor failure timeout - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    /* No valid POS sensor but still within valid timeout - wait */
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!posControl.flags.hasValidHeadingSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // If position ok OR within valid timeout - continue
    if (posControl.flags.hasValidPositionSensor || !checkForPositionSensorTimeout()) {
        // Wait until target heading is reached (with 15 deg margin for error)
        if (STATE(FIXED_WING)) {
            resetLandingDetector();
            return NAV_FSM_EVENT_SUCCESS;
        }
        else {
            if (ABS(wrap_18000(posControl.homeWaypointAbove.yaw - posControl.actualState.yaw)) < DEGREES_TO_CENTIDEGREES(15)) {
                resetLandingDetector();
                return NAV_FSM_EVENT_SUCCESS;
            }
            else if (!validateRTHSanityChecker()) {
                // Continue to check for RTH sanity during pre-landing hover
                return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
            }
            else {
                setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
                return NAV_FSM_EVENT_NONE;
            }
        }
    }
    else {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!ARMING_FLAG(ARMED)) {
        return NAV_FSM_EVENT_SUCCESS;
    }
    else if (isLandingDetected()) {
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        if (!validateRTHSanityChecker()) {
            // Continue to check for RTH sanity during landing
            return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
        }

        if (navConfig()->general.flags.rth_allow_landing) {
            float descentVelLimited = 0;

            // A safeguard - if surface altitude sensors is available and it is reading < 50cm altitude - drop to low descend speed
            if (posControl.flags.hasValidSurfaceSensor && posControl.actualState.surface < 50.0f) {
                // land_descent_rate == 200 : descend speed = 30 cm/s, gentle touchdown
                // Do not allow descent velocity slower than -30cm/s so the landing detector works.
                descentVelLimited = MIN(-0.15f * navConfig()->general.land_descent_rate, -30.0f);
            }
            else {
                // Ramp down descent velocity from 100% at maxAlt altitude to 25% from minAlt to 0cm.
                float descentVelScaling = (posControl.actualState.pos.V.Z - posControl.homePosition.pos.V.Z - navConfig()->general.land_slowdown_minalt)
                                            / (navConfig()->general.land_slowdown_maxalt - navConfig()->general.land_slowdown_minalt) * 0.75f + 0.25f;  // Yield 1.0 at 2000 alt and 0.25 at 500 alt

                descentVelScaling = constrainf(descentVelScaling, 0.25f, 1.0f);

                // Do not allow descent velocity slower than -50cm/s so the landing detector works.
                descentVelLimited = MIN(-descentVelScaling * navConfig()->general.land_descent_rate, -50.0f);
            }

            updateClimbRateToAltitudeController(descentVelLimited, ROC_TO_ALT_NORMAL);
        }

        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (navConfig()->general.flags.disarm_on_landing) {
        mwDisarm(DISARM_NAVIGATION);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_FINISHED(navigationFSMState_t previousState)
{
    // Stay in this state
    UNUSED(previousState);
    updateClimbRateToAltitudeController(-0.3f * navConfig()->general.land_descent_rate, ROC_TO_ALT_NORMAL);  // FIXME
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (posControl.waypointCount == 0 || !posControl.waypointListValid) {
        return NAV_FSM_EVENT_ERROR;
    }
    else {
        // Prepare controllers
        resetPositionController();
        resetAltitudeController();
        setupAltitudeController();

        posControl.activeWaypointIndex = 0;
        return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_PRE_ACTION
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION(navigationFSMState_t previousState)
{
    /* A helper function to do waypoint-specific action */
    UNUSED(previousState);

    switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
        case NAV_WP_ACTION_WAYPOINT:
            calcualteAndSetActiveWaypoint(&posControl.waypointList[posControl.activeWaypointIndex]);
            return NAV_FSM_EVENT_SUCCESS;       // will switch to NAV_STATE_WAYPOINT_IN_PROGRESS

        case NAV_WP_ACTION_RTH:
        default:
            initializeRTHSanityChecker(&posControl.actualState.pos);
            calcualteAndSetActiveWaypointToLocalPosition(&posControl.homeWaypointAbove.pos);
            return NAV_FSM_EVENT_SUCCESS;       // will switch to NAV_STATE_WAYPOINT_IN_PROGRESS
    };
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) {
        const bool isDoingRTH = (posControl.waypointList[posControl.activeWaypointIndex].action == NAV_WP_ACTION_RTH);

        switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
            case NAV_WP_ACTION_WAYPOINT:
            case NAV_WP_ACTION_RTH:
            default:
                if (isWaypointReached(&posControl.activeWaypoint, isDoingRTH) || isWaypointMissed(&posControl.activeWaypoint)) {
                    // Waypoint reached
                    return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_REACHED
                }
                else {
                    // Update XY-position target to active waypoint
                    setDesiredPosition(&posControl.activeWaypoint.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
                    return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
                }
                break;
        }
    }
    /* No pos sensor available for NAV_WAIT_FOR_GPS_TIMEOUT_MS - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    else {
        return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_REACHED(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
        case NAV_WP_ACTION_RTH:
            if (posControl.waypointList[posControl.activeWaypointIndex].p1 != 0) {
                return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND;
            }
            else {
                return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_WAYPOINT_NEXT
            }

        default:
            return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_WAYPOINT_NEXT
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_RTH_LAND(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    const navigationFSMEvent_t landEvent = navOnEnteringState_NAV_STATE_RTH_LANDING(previousState);
    if (landEvent == NAV_FSM_EVENT_SUCCESS) {
        // Landing controller returned success - invoke RTH finishing state and finish the waypoint
        navOnEnteringState_NAV_STATE_RTH_FINISHING(previousState);
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_NEXT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    const bool isLastWaypoint = (posControl.waypointList[posControl.activeWaypointIndex].flag == NAV_WP_FLAG_LAST) ||
                          (posControl.activeWaypointIndex >= (posControl.waypointCount - 1));

    if (isLastWaypoint) {
        // Last waypoint reached
        return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED;
    }
    else {
        // Waypoint reached, do something and move on to next waypoint
        posControl.activeWaypointIndex++;
        return NAV_FSM_EVENT_SUCCESS;   // will switch to NAV_STATE_WAYPOINT_PRE_ACTION
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) {
        return NAV_FSM_EVENT_NONE;
    }
    /* No pos sensor available for NAV_WAIT_FOR_GPS_TIMEOUT_MS - land */
    else if (checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    else {
        return NAV_FSM_EVENT_NONE;      // will re-process state in >10ms
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);

    // Emergency landing MAY use common altitude controller if vertical position is valid - initialize it
    resetAltitudeController();

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);
    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_INITIALIZE(navigationFSMState_t previousState)
{
    const timeUs_t currentTimeUs = micros();
    UNUSED(previousState);

    resetFixedWingLaunchController(currentTimeUs);

    return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_LAUNCH_WAIT
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_WAIT(navigationFSMState_t previousState)
{
    const timeUs_t currentTimeUs = micros();
    UNUSED(previousState);

    if (isFixedWingLaunchDetected()) {
        enableFixedWingLaunchController(currentTimeUs);
        return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_LAUNCH_MOTOR_DELAY
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_LAUNCH_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (isFixedWingLaunchFinishedOrAborted()) {
        return NAV_FSM_EVENT_SUCCESS;
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMState_t navSetNewFSMState(navigationFSMState_t newState)
{
    navigationFSMState_t previousState;

    previousState = posControl.navState;
    posControl.navState = newState;
    return previousState;
}

static void navProcessFSMEvents(navigationFSMEvent_t injectedEvent)
{
    const timeMs_t currentMillis = millis();
    navigationFSMState_t previousState;
    static timeMs_t lastStateProcessTime = 0;

    /* If timeout event defined and timeout reached - switch state */
    if ((navFSM[posControl.navState].timeoutMs > 0) && (navFSM[posControl.navState].onEvent[NAV_FSM_EVENT_TIMEOUT] != NAV_STATE_UNDEFINED) &&
            ((currentMillis - lastStateProcessTime) >= navFSM[posControl.navState].timeoutMs)) {
        /* Update state */
        previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[NAV_FSM_EVENT_TIMEOUT]);

        /* Call new state's entry function */
        while (navFSM[posControl.navState].onEntry) {
            navigationFSMEvent_t newEvent = navFSM[posControl.navState].onEntry(previousState);

            if ((newEvent != NAV_FSM_EVENT_NONE) && (navFSM[posControl.navState].onEvent[newEvent] != NAV_STATE_UNDEFINED)) {
                previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[newEvent]);
            }
            else {
                break;
            }
        }

        lastStateProcessTime  = currentMillis;
    }

    /* Inject new event */
    if (injectedEvent != NAV_FSM_EVENT_NONE && navFSM[posControl.navState].onEvent[injectedEvent] != NAV_STATE_UNDEFINED) {
        /* Update state */
        previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[injectedEvent]);

        /* Call new state's entry function */
        while (navFSM[posControl.navState].onEntry) {
            navigationFSMEvent_t newEvent = navFSM[posControl.navState].onEntry(previousState);

            if ((newEvent != NAV_FSM_EVENT_NONE) && (navFSM[posControl.navState].onEvent[newEvent] != NAV_STATE_UNDEFINED)) {
                previousState = navSetNewFSMState(navFSM[posControl.navState].onEvent[newEvent]);
            }
            else {
                break;
            }
        }

        lastStateProcessTime  = currentMillis;
    }

    /* Update public system state information */
    NAV_Status.mode = MW_GPS_MODE_NONE;

    if (ARMING_FLAG(ARMED)) {
        navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);

        if (navStateFlags & NAV_AUTO_RTH) {
            NAV_Status.mode = MW_GPS_MODE_RTH;
        }
        else if (navStateFlags & NAV_AUTO_WP) {
            NAV_Status.mode = MW_GPS_MODE_NAV;
        }
        else if (navStateFlags & NAV_CTL_EMERG) {
            NAV_Status.mode = MW_GPS_MODE_EMERG;
        }
        else if (navStateFlags & NAV_CTL_POS) {
            NAV_Status.mode = MW_GPS_MODE_HOLD;
        }
    }

    NAV_Status.state = navFSM[posControl.navState].mwState;
    NAV_Status.error = navFSM[posControl.navState].mwError;

    NAV_Status.flags = 0;
    if (posControl.flags.isAdjustingPosition)   NAV_Status.flags |= MW_NAV_FLAG_ADJUSTING_POSITION;
    if (posControl.flags.isAdjustingAltitude)   NAV_Status.flags |= MW_NAV_FLAG_ADJUSTING_ALTITUDE;

    NAV_Status.activeWpNumber = posControl.activeWaypointIndex + 1;
    NAV_Status.activeWpAction = 0;
    if ((posControl.activeWaypointIndex >= 0) && (posControl.activeWaypointIndex < NAV_MAX_WAYPOINTS)) {
        NAV_Status.activeWpAction = posControl.waypointList[posControl.activeWaypointIndex].action;
    }
}

/*-----------------------------------------------------------
 * Float point PID-controller implementation
 *-----------------------------------------------------------*/
// Implementation of PID with back-calculation I-term anti-windup
// Control System Design, Lecture Notes for ME 155A by Karl Johan Åström (p.228)
// http://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
float navPidApply3(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags, const float gainScaler)
{
    float newProportional, newDerivative;
    float error = setpoint - measurement;

    /* P-term */
    newProportional = error * pid->param.kP * gainScaler;

    /* D-term */
    if (pidFlags & PID_DTERM_FROM_ERROR) {
        /* Error-tracking D-term */
        newDerivative = (error - pid->last_input) / dt;
        pid->last_input = error;
    }
    else {
        /* Measurement tracking D-term */
        newDerivative = -(measurement - pid->last_input) / dt;
        pid->last_input = measurement;
    }

    newDerivative = pid->param.kD * pt1FilterApply4(&pid->dterm_filter_state, newDerivative, NAV_DTERM_CUT_HZ, dt) * gainScaler;

    if (pidFlags & PID_ZERO_INTEGRATOR) {
        pid->integrator = 0.0f;
    }

    /* Pre-calculate output and limit it if actuator is saturating */
    const float outVal = newProportional + (pid->integrator * gainScaler) + newDerivative;
    const float outValConstrained = constrainf(outVal, outMin, outMax);

    /* Update I-term */
    if (!(pidFlags & PID_ZERO_INTEGRATOR)) {
        const float newIntegrator = pid->integrator + (error * pid->param.kI * gainScaler * dt) + ((outValConstrained - outVal) * pid->param.kT * dt);

        if (pidFlags & PID_SHRINK_INTEGRATOR) {
            // Only allow integrator to shrink
            if (ABS(newIntegrator) < ABS(pid->integrator)) {
                pid->integrator = newIntegrator;
            }
        }
        else {
            pid->integrator = newIntegrator;
        }
    }

    return outValConstrained;
}

float navPidApply2(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags)
{
    return navPidApply3(pid, setpoint, measurement, dt, outMin, outMax, pidFlags, 1.0f);
}


void navPidReset(pidController_t *pid)
{
    pid->integrator = 0.0f;
    pid->last_input = 0.0f;
    pid->dterm_filter_state.state = 0.0f;
    pid->dterm_filter_state.RC = 0.0f;
}

void navPidInit(pidController_t *pid, float _kP, float _kI, float _kD)
{
    pid->param.kP = _kP;
    pid->param.kI = _kI;
    pid->param.kD = _kD;

    if (_kI > 1e-6f && _kP > 1e-6f) {
        float Ti = _kP / _kI;
        float Td = _kD / _kP;
        pid->param.kT = 2.0f / (Ti + Td);
    }
    else {
        pid->param.kI = 0.0;
        pid->param.kT = 0.0;
    }

    navPidReset(pid);
}

/*-----------------------------------------------------------
 * Float point P-controller implementation
 *-----------------------------------------------------------*/
void navPInit(pController_t *p, float _kP)
{
    p->param.kP = _kP;
}

/*-----------------------------------------------------------
 * Detects if thrust vector is facing downwards
 *-----------------------------------------------------------*/
bool isThrustFacingDownwards(void)
{
    // Tilt angle <= 80 deg; cos(80) = 0.17364817766693034885171662676931
    return (calculateCosTiltAngle() >= 0.173648178f);
}

/*-----------------------------------------------------------
 * Checks if position sensor (GPS) is failing for a specified timeout (if enabled)
 *-----------------------------------------------------------*/
bool checkForPositionSensorTimeout(void)
{
    if (navConfig()->general.pos_failure_timeout) {
        if (!posControl.flags.hasValidPositionSensor && ((millis() - posControl.lastValidPositionTimeMs) > (1000 * navConfig()->general.pos_failure_timeout))) {
            return true;
        }
        else {
            return false;
        }
    }
    else {
        // Timeout not defined, never fail
        return false;
    }
}

/*-----------------------------------------------------------
 * Processes an update to XY-position and velocity
 *-----------------------------------------------------------*/
void updateActualHorizontalPositionAndVelocity(bool hasValidSensor, float newX, float newY, float newVelX, float newVelY)
{
    posControl.actualState.pos.V.X = newX;
    posControl.actualState.pos.V.Y = newY;

    posControl.actualState.vel.V.X = newVelX;
    posControl.actualState.vel.V.Y = newVelY;
    posControl.actualState.velXY = sqrtf(sq(newVelX) + sq(newVelY));

    posControl.flags.hasValidPositionSensor = hasValidSensor;
    posControl.flags.hasValidHeadingSensor = isImuHeadingValid();

    if (hasValidSensor) {
        posControl.flags.horizontalPositionDataNew = 1;
        posControl.lastValidPositionTimeMs = millis();
    }
    else {
        posControl.flags.horizontalPositionDataNew = 0;
    }

#if defined(NAV_BLACKBOX)
    navLatestActualPosition[X] = newX;
    navLatestActualPosition[Y] = newY;
    navActualVelocity[X] = constrain(newVelX, -32678, 32767);
    navActualVelocity[Y] = constrain(newVelY, -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Processes an update to Z-position and velocity
 *-----------------------------------------------------------*/
void updateActualAltitudeAndClimbRate(bool hasValidSensor, float newAltitude, float newVelocity)
{
    posControl.actualState.pos.V.Z = newAltitude;
    posControl.actualState.vel.V.Z = newVelocity;

    posControl.flags.hasValidAltitudeSensor = hasValidSensor;

    // Update altitude that would be used when executing RTH
    if (hasValidSensor) {
        updateDesiredRTHAltitude();
        posControl.flags.verticalPositionDataNew = 1;
        posControl.lastValidAltitudeTimeMs = millis();
    }
    else {
        posControl.flags.verticalPositionDataNew = 0;
    }

#if defined(NAV_BLACKBOX)
    navLatestActualPosition[Z] = constrain(newAltitude, -32678, 32767);
    navActualVelocity[Z] = constrain(newVelocity, -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Processes an update to surface distance
 *-----------------------------------------------------------*/
void updateActualSurfaceDistance(bool hasValidSensor, float surfaceDistance, float surfaceVelocity)
{
    posControl.actualState.surface = surfaceDistance;
    posControl.actualState.surfaceVel = surfaceVelocity;

    // Update validity
    // Update validity
    if (hasValidSensor) {
        posControl.flags.hasValidSurfaceSensor = true;
        posControl.flags.surfaceDistanceDataNew = 1;
    }
    else {
        posControl.flags.hasValidSurfaceSensor = false;
        posControl.flags.surfaceDistanceDataNew = 0;
    }

    // Update minimum surface distance (landing detection threshold)
    if (ARMING_FLAG(ARMED)) {
        if (hasValidSensor && posControl.actualState.surface > 0) {
            if (posControl.actualState.surfaceMin > 0) {
                posControl.actualState.surfaceMin = MIN(posControl.actualState.surfaceMin, posControl.actualState.surface);
            }
            else {
                posControl.actualState.surfaceMin = posControl.actualState.surface;
            }
        }
    }
    else {
        posControl.actualState.surfaceMin = -1;
    }

#if defined(NAV_BLACKBOX)
    navActualSurface = surfaceDistance;
#endif
}

/*-----------------------------------------------------------
 * Processes an update to estimated heading
 *-----------------------------------------------------------*/
void updateActualHeading(int32_t newHeading)
{
    /* Update heading */
    posControl.actualState.yaw = newHeading;

    /* Precompute sin/cos of yaw angle */
    posControl.actualState.sinYaw = sin_approx(CENTIDEGREES_TO_RADIANS(newHeading));
    posControl.actualState.cosYaw = cos_approx(CENTIDEGREES_TO_RADIANS(newHeading));

    posControl.flags.headingDataNew = 1;
}

/*-----------------------------------------------------------
 * Calculates distance and bearing to destination point
 *-----------------------------------------------------------*/
uint32_t calculateDistanceToDestination(const t_fp_vector * destinationPos)
{
    const float deltaX = destinationPos->V.X - posControl.actualState.pos.V.X;
    const float deltaY = destinationPos->V.Y - posControl.actualState.pos.V.Y;

    return sqrtf(sq(deltaX) + sq(deltaY));
}

int32_t calculateBearingToDestination(const t_fp_vector * destinationPos)
{
    const float deltaX = destinationPos->V.X - posControl.actualState.pos.V.X;
    const float deltaY = destinationPos->V.Y - posControl.actualState.pos.V.Y;

    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(deltaY, deltaX)));
}

/*-----------------------------------------------------------
 * Check if waypoint is/was reached. Assume that waypoint-yaw stores initial bearing
 *-----------------------------------------------------------*/
bool isWaypointMissed(const navWaypointPosition_t * waypoint)
{
    int32_t bearingError = calculateBearingToDestination(&waypoint->pos) - waypoint->yaw;
    bearingError = wrap_18000(bearingError);

    return ABS(bearingError) > 10000; // TRUE if we passed the waypoint by 100 degrees
}

bool isWaypointReached(const navWaypointPosition_t * waypoint, const bool isWaypointHome)
{
    // We consider waypoint reached if within specified radius
    const uint32_t wpDistance = calculateDistanceToDestination(&waypoint->pos);

    if (STATE(FIXED_WING) && isWaypointHome) {
        // Airplane will do a circular loiter over home and might never approach it closer than waypoint_radius - need extra check
        return (wpDistance <= navConfig()->general.waypoint_radius)
                || (wpDistance <= (navConfig()->fw.loiter_radius * 1.10f));  // 10% margin of desired circular loiter radius
    }
    else {
        return (wpDistance <= navConfig()->general.waypoint_radius);
    }
}

static void updateHomePositionCompatibility(void)
{
    geoConvertLocalToGeodetic(&posControl.gpsOrigin, &posControl.homePosition.pos, &GPS_home);
    GPS_distanceToHome = posControl.homeDistance / 100;
    GPS_directionToHome = posControl.homeDirection / 100;
}

/*-----------------------------------------------------------
 * Reset home position to current position
 *-----------------------------------------------------------*/
static void updateDesiredRTHAltitude(void)
{
    if (ARMING_FLAG(ARMED)) {
        if (!(navGetStateFlags(posControl.navState) & NAV_AUTO_RTH)) {
            switch (navConfig()->general.flags.rth_alt_control_mode) {
            case NAV_RTH_NO_ALT:
                posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z;
                break;
            case NAV_RTH_EXTRA_ALT: // Maintain current altitude + predefined safety margin
                posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z + navConfig()->general.rth_altitude;
                break;
            case NAV_RTH_MAX_ALT:
                posControl.homeWaypointAbove.pos.V.Z = MAX(posControl.homeWaypointAbove.pos.V.Z, posControl.actualState.pos.V.Z);
                break;
            case NAV_RTH_AT_LEAST_ALT:  // Climb to at least some predefined altitude above home
                posControl.homeWaypointAbove.pos.V.Z = MAX(posControl.homePosition.pos.V.Z + navConfig()->general.rth_altitude, posControl.actualState.pos.V.Z);
                break;
            case NAV_RTH_CONST_ALT:     // Climb/descend to predefined altitude above home
            default:
                posControl.homeWaypointAbove.pos.V.Z = posControl.homePosition.pos.V.Z + navConfig()->general.rth_altitude;
                break;
            }
        }
    }
    else {
        posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z;
    }
}

/*-----------------------------------------------------------
 * RTH sanity test logic
 *-----------------------------------------------------------*/
void initializeRTHSanityChecker(const t_fp_vector * pos)
{
    const timeMs_t currentTimeMs = millis();

    posControl.rthSanityChecker.lastCheckTime = currentTimeMs;
    posControl.rthSanityChecker.initialPosition = *pos;
    posControl.rthSanityChecker.minimalDistanceToHome = calculateDistanceToDestination(&posControl.homePosition.pos);
}

bool validateRTHSanityChecker(void)
{
    const timeMs_t currentTimeMs = millis();
    bool checkResult = true;    // Between the checks return the "good" status

    // Ability to disable this
    if (navConfig()->general.rth_abort_threshold == 0) {
        return true;
    }

    // Check at 10Hz rate
    if ((currentTimeMs - posControl.rthSanityChecker.lastCheckTime) > 100) {
        const float currentDistanceToHome = calculateDistanceToDestination(&posControl.homePosition.pos);

        if (currentDistanceToHome < posControl.rthSanityChecker.minimalDistanceToHome) {
            posControl.rthSanityChecker.minimalDistanceToHome = currentDistanceToHome;
        }
        else if ((currentDistanceToHome - posControl.rthSanityChecker.minimalDistanceToHome) > navConfig()->general.rth_abort_threshold) {
            // If while doing RTH we got even farther away from home - RTH is doing something crazy
            checkResult = false;
        }

        posControl.rthSanityChecker.lastCheckTime = currentTimeMs;
    }

    return checkResult;
}

/*-----------------------------------------------------------
 * Reset home position to current position
 *-----------------------------------------------------------*/
void setHomePosition(const t_fp_vector * pos, int32_t yaw, navSetWaypointFlags_t useMask)
{
    // XY-position
    if ((useMask & NAV_POS_UPDATE_XY) != 0) {
        posControl.homePosition.pos.V.X = pos->V.X;
        posControl.homePosition.pos.V.Y = pos->V.Y;
    }

    // Z-position
    if ((useMask & NAV_POS_UPDATE_Z) != 0) {
        posControl.homePosition.pos.V.Z = pos->V.Z;
    }

    // Heading
    if ((useMask & NAV_POS_UPDATE_HEADING) != 0) {
        // Heading
        posControl.homePosition.yaw = yaw;
    }

    posControl.homeDistance = 0;
    posControl.homeDirection = 0;

    // Update target RTH altitude as a waypoint above home
    posControl.homeWaypointAbove = posControl.homePosition;
    updateDesiredRTHAltitude();

    updateHomePositionCompatibility();
    ENABLE_STATE(GPS_FIX_HOME);
}

/*-----------------------------------------------------------
 * Update home position, calculate distance and bearing to home
 *-----------------------------------------------------------*/
void updateHomePosition(void)
{
    // Disarmed and have a valid position, constantly update home
    if (!ARMING_FLAG(ARMED)) {
        if (posControl.flags.hasValidPositionSensor) {
            setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING );
        }
    }
    else {
        static bool isHomeResetAllowed = false;

        // If pilot so desires he may reset home position to current position
        if (IS_RC_MODE_ACTIVE(BOXHOMERESET)) {
            if (isHomeResetAllowed && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE) && posControl.flags.hasValidPositionSensor) {
                const navSetWaypointFlags_t homeUpdateFlags = STATE(GPS_FIX_HOME) ? (NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING) : (NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
                setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, homeUpdateFlags);
                isHomeResetAllowed = false;
            }
        }
        else {
            isHomeResetAllowed = true;
        }

        // Update distance and direction to home if armed (home is not updated when armed)
        if (STATE(GPS_FIX_HOME)) {
            posControl.homeDistance = calculateDistanceToDestination(&posControl.homePosition.pos);
            posControl.homeDirection = calculateBearingToDestination(&posControl.homePosition.pos);
            updateHomePositionCompatibility();
        }
    }
}

/*-----------------------------------------------------------
 * Update flight statistics
 *-----------------------------------------------------------*/
static void updateNavigationFlightStatistics(void)
{
    static timeMs_t previousTimeMs = 0;
    const timeMs_t currentTimeMs = millis();
    const timeDelta_t timeDeltaMs = currentTimeMs - previousTimeMs;
    previousTimeMs = currentTimeMs;

    if (ARMING_FLAG(ARMED)) {
        posControl.totalTripDistance += posControl.actualState.velXY * MS2S(timeDeltaMs);
    }
}

int32_t getTotalTravelDistance(void)
{
    return lrintf(posControl.totalTripDistance);
}

/*-----------------------------------------------------------
 * Set surface tracking target
 *-----------------------------------------------------------*/
void setDesiredSurfaceOffset(float surfaceOffset)
{
    if (surfaceOffset > 0) {
        posControl.desiredState.surface = constrainf(surfaceOffset, 1.0f, INAV_SURFACE_MAX_DISTANCE);
    }
    else {
        posControl.desiredState.surface = -1;
    }

#if defined(NAV_BLACKBOX)
    navTargetSurface = constrain(lrintf(posControl.desiredState.surface), -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Calculate platform-specific hold position (account for deceleration)
 *-----------------------------------------------------------*/
void calculateInitialHoldPosition(t_fp_vector * pos)
{
    if (STATE(FIXED_WING)) { // FIXED_WING
        calculateFixedWingInitialHoldPosition(pos);
    }
    else {
        calculateMulticopterInitialHoldPosition(pos);
    }
}

/*-----------------------------------------------------------
 * Set active XYZ-target and desired heading
 *-----------------------------------------------------------*/
void setDesiredPosition(const t_fp_vector * pos, int32_t yaw, navSetWaypointFlags_t useMask)
{
    // XY-position
    if ((useMask & NAV_POS_UPDATE_XY) != 0) {
        posControl.desiredState.pos.V.X = pos->V.X;
        posControl.desiredState.pos.V.Y = pos->V.Y;
    }

    // Z-position
    if ((useMask & NAV_POS_UPDATE_Z) != 0) {
        updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);   // Reset RoC/RoD -> altitude controller
        posControl.desiredState.surface = -1;                       // When we directly set altitude target we must reset surface tracking
        posControl.desiredState.pos.V.Z = pos->V.Z;
    }

    // Heading
    if ((useMask & NAV_POS_UPDATE_HEADING) != 0) {
        // Heading
        posControl.desiredState.yaw = yaw;
    }
    else if ((useMask & NAV_POS_UPDATE_BEARING) != 0) {
        posControl.desiredState.yaw = calculateBearingToDestination(pos);
    }
    else if ((useMask & NAV_POS_UPDATE_BEARING_TAIL_FIRST) != 0) {
        posControl.desiredState.yaw = wrap_36000(calculateBearingToDestination(pos) - 18000);
    }
}

void calculateFarAwayTarget(t_fp_vector * farAwayPos, int32_t yaw, int32_t distance)
{
    farAwayPos->V.X = posControl.actualState.pos.V.X + distance * cos_approx(CENTIDEGREES_TO_RADIANS(yaw));
    farAwayPos->V.Y = posControl.actualState.pos.V.Y + distance * sin_approx(CENTIDEGREES_TO_RADIANS(yaw));
    farAwayPos->V.Z = posControl.actualState.pos.V.Z;
}

/*-----------------------------------------------------------
 * NAV land detector
 *-----------------------------------------------------------*/
void resetLandingDetector(void)
{
    if (STATE(FIXED_WING)) { // FIXED_WING
        resetFixedWingLandingDetector();
    }
    else {
        resetMulticopterLandingDetector();
    }
}

bool isLandingDetected(void)
{
    bool landingDetected;

    if (STATE(FIXED_WING)) { // FIXED_WING
        landingDetected = isFixedWingLandingDetected();
    }
    else {
        landingDetected = isMulticopterLandingDetected();
    }

    return landingDetected;
}

/*-----------------------------------------------------------
 * Z-position controller
 *-----------------------------------------------------------*/
void updateClimbRateToAltitudeController(float desiredClimbRate, climbRateToAltitudeControllerMode_e mode)
{
    static timeUs_t lastUpdateTimeUs;
    timeUs_t currentTimeUs = micros();

    if (mode == ROC_TO_ALT_RESET) {
        lastUpdateTimeUs = currentTimeUs;
        posControl.desiredState.pos.V.Z = posControl.actualState.pos.V.Z;
    }
    else {
        if (STATE(FIXED_WING)) {
            // Fixed wing climb rate controller is open-loop. We simply move the known altitude target
            float timeDelta = US2S(currentTimeUs - lastUpdateTimeUs);

            DEBUG_SET(DEBUG_FW_CLIMB_RATE_TO_ALTITUDE, 0, desiredClimbRate);
            DEBUG_SET(DEBUG_FW_CLIMB_RATE_TO_ALTITUDE, 1, timeDelta * 1000);

            if (timeDelta <= HZ2S(MIN_POSITION_UPDATE_RATE_HZ)) {
                posControl.desiredState.pos.V.Z += desiredClimbRate * timeDelta;
                posControl.desiredState.pos.V.Z = constrainf(posControl.desiredState.pos.V.Z,           // FIXME: calculate sanity limits in a smarter way
                                                             posControl.actualState.pos.V.Z - 500,
                                                             posControl.actualState.pos.V.Z + 500);
            }
        }
        else {
            // Multicopter climb-rate control is closed-loop, it's possible to directly calculate desired altitude setpoint to yield the required RoC/RoD
            posControl.desiredState.pos.V.Z = posControl.actualState.pos.V.Z + (desiredClimbRate / posControl.pids.pos[Z].param.kP);
        }

        lastUpdateTimeUs = currentTimeUs;
    }
}

static void resetAltitudeController(void)
{
    if (STATE(FIXED_WING)) {
        resetFixedWingAltitudeController();
    }
    else {
        resetMulticopterAltitudeController();
    }
}

static void setupAltitudeController(void)
{
    if (STATE(FIXED_WING)) {
        setupFixedWingAltitudeController();
    }
    else {
        setupMulticopterAltitudeController();
    }
}

static bool adjustAltitudeFromRCInput(void)
{
    if (STATE(FIXED_WING)) {
        return adjustFixedWingAltitudeFromRCInput();
    }
    else {
        return adjustMulticopterAltitudeFromRCInput();
    }
}

/*-----------------------------------------------------------
 * Heading controller (pass-through to MAG mode)
 *-----------------------------------------------------------*/
static void resetHeadingController(void)
{
    if (STATE(FIXED_WING)) {
        resetFixedWingHeadingController();
    }
    else {
        resetMulticopterHeadingController();
    }
}

static bool adjustHeadingFromRCInput(void)
{
    if (STATE(FIXED_WING)) {
        return adjustFixedWingHeadingFromRCInput();
    }
    else {
        return adjustMulticopterHeadingFromRCInput();
    }
}

/*-----------------------------------------------------------
 * XY Position controller
 *-----------------------------------------------------------*/
static void resetPositionController(void)
{
    if (STATE(FIXED_WING)) {
        resetFixedWingPositionController();
    }
    else {
        resetMulticopterPositionController();
    }
}

static bool adjustPositionFromRCInput(void)
{
    bool retValue;

    if (STATE(FIXED_WING)) {
        retValue = adjustFixedWingPositionFromRCInput();
    }
    else {
        retValue = adjustMulticopterPositionFromRCInput();
    }

    return retValue;
}

/*-----------------------------------------------------------
 * WP controller
 *-----------------------------------------------------------*/
void resetGCSFlags(void)
{
    posControl.flags.isGCSAssistedNavigationReset = false;
    posControl.flags.isGCSAssistedNavigationEnabled = false;
}

void getWaypoint(uint8_t wpNumber, navWaypoint_t * wpData)
{
    /* Default waypoint to send */
    wpData->action = NAV_WP_ACTION_RTH;
    wpData->lat = 0;
    wpData->lon = 0;
    wpData->alt = 0;
    wpData->p1 = 0;
    wpData->p2 = 0;
    wpData->p3 = 0;
    wpData->flag = NAV_WP_FLAG_LAST;

    // WP #0 - special waypoint - HOME
    if (wpNumber == 0) {
        if (STATE(GPS_FIX_HOME)) {
            wpData->lat = GPS_home.lat;
            wpData->lon = GPS_home.lon;
            wpData->alt = GPS_home.alt;
        }
    }
    // WP #255 - special waypoint - directly get actualPosition
    else if (wpNumber == 255) {
        gpsLocation_t wpLLH;

        geoConvertLocalToGeodetic(&posControl.gpsOrigin, &posControl.actualState.pos, &wpLLH);

        wpData->lat = wpLLH.lat;
        wpData->lon = wpLLH.lon;
        wpData->alt = wpLLH.alt;
    }
    // WP #1 - #15 - common waypoints - pre-programmed mission
    else if ((wpNumber >= 1) && (wpNumber <= NAV_MAX_WAYPOINTS)) {
        if (wpNumber <= posControl.waypointCount) {
            *wpData = posControl.waypointList[wpNumber - 1];
        }
    }
}

void setWaypoint(uint8_t wpNumber, const navWaypoint_t * wpData)
{
    gpsLocation_t wpLLH;
    navWaypointPosition_t wpPos;

    // Pre-fill structure to convert to local coordinates
    wpLLH.lat = wpData->lat;
    wpLLH.lon = wpData->lon;
    wpLLH.alt = wpData->alt;

    // WP #0 - special waypoint - HOME
    if ((wpNumber == 0) && ARMING_FLAG(ARMED) && posControl.flags.hasValidPositionSensor && posControl.gpsOrigin.valid && posControl.flags.isGCSAssistedNavigationEnabled) {
        // Forcibly set home position. Note that this is only valid if already armed, otherwise home will be reset instantly
        geoConvertGeodeticToLocal(&posControl.gpsOrigin, &wpLLH, &wpPos.pos, GEO_ALT_RELATIVE);
        setHomePosition(&wpPos.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
    }
    // WP #255 - special waypoint - directly set desiredPosition
    // Only valid when armed and in poshold mode
    else if ((wpNumber == 255) && (wpData->action == NAV_WP_ACTION_WAYPOINT) &&
             ARMING_FLAG(ARMED) && posControl.flags.hasValidPositionSensor && posControl.gpsOrigin.valid && posControl.flags.isGCSAssistedNavigationEnabled &&
             (posControl.navState == NAV_STATE_POSHOLD_2D_IN_PROGRESS || posControl.navState == NAV_STATE_POSHOLD_3D_IN_PROGRESS)) {
        // Convert to local coordinates
        geoConvertGeodeticToLocal(&posControl.gpsOrigin, &wpLLH, &wpPos.pos, GEO_ALT_RELATIVE);

        navSetWaypointFlags_t waypointUpdateFlags = NAV_POS_UPDATE_XY;

        // If we received global altitude == 0, use current altitude
        if (wpData->alt != 0) {
            waypointUpdateFlags |= NAV_POS_UPDATE_Z;
        }

        if (wpData->p1 > 0 && wpData->p1 < 360) {
            waypointUpdateFlags |= NAV_POS_UPDATE_HEADING;
        }

        setDesiredPosition(&wpPos.pos, DEGREES_TO_CENTIDEGREES(wpData->p1), waypointUpdateFlags);
    }
    // WP #1 - #15 - common waypoints - pre-programmed mission
    else if ((wpNumber >= 1) && (wpNumber <= NAV_MAX_WAYPOINTS) && !ARMING_FLAG(ARMED)) {
        if (wpData->action == NAV_WP_ACTION_WAYPOINT || wpData->action == NAV_WP_ACTION_RTH) {
            // Only allow upload next waypoint (continue upload mission) or first waypoint (new mission)
            if (wpNumber == (posControl.waypointCount + 1) || wpNumber == 1) {
                posControl.waypointList[wpNumber - 1] = *wpData;
                posControl.waypointCount = wpNumber;
                posControl.waypointListValid = (wpData->flag == NAV_WP_FLAG_LAST);
            }
        }
    }
}

void resetWaypointList(void)
{
    /* Can only reset waypoint list if not armed */
    if (!ARMING_FLAG(ARMED)) {
        posControl.waypointCount = 0;
        posControl.waypointListValid = false;
    }
}

bool isWaypointListValid(void)
{
    return posControl.waypointListValid;
}

int getWaypointCount(void)
{
    return posControl.waypointCount;
}

#ifdef NAV_NON_VOLATILE_WAYPOINT_STORAGE
bool loadNonVolatileWaypointList(void)
{
    if (ARMING_FLAG(ARMED))
        return false;

    resetWaypointList();

    for (int i = 0; i < NAV_MAX_WAYPOINTS; i++) {
        // Load waypoint
        setWaypoint(i + 1, nonVolatileWaypointList(i));

        // Check if this is the last waypoint
        if (nonVolatileWaypointList(i)->flag == NAV_WP_FLAG_LAST)
            break;
    }

    // Mission sanity check failed - reset the list
    if (!posControl.waypointListValid) {
        resetWaypointList();
    }

    return posControl.waypointListValid;
}

bool saveNonVolatileWaypointList(void)
{
    if (ARMING_FLAG(ARMED) || !posControl.waypointListValid)
        return false;

    for (int i = 0; i < NAV_MAX_WAYPOINTS; i++) {
        getWaypoint(i + 1, nonVolatileWaypointListMutable(i));
    }

    saveConfigAndNotify();

    return true;
}
#endif

static void mapWaypointToLocalPosition(t_fp_vector * localPos, const navWaypoint_t * waypoint)
{
    gpsLocation_t wpLLH;

    wpLLH.lat = waypoint->lat;
    wpLLH.lon = waypoint->lon;
    wpLLH.alt = waypoint->alt;

    geoConvertGeodeticToLocal(&posControl.gpsOrigin, &wpLLH, localPos, GEO_ALT_RELATIVE);
}

static void calcualteAndSetActiveWaypointToLocalPosition(const t_fp_vector * pos)
{
    posControl.activeWaypoint.pos = *pos;

    // Calculate initial bearing towards waypoint and store it in waypoint yaw parameter (this will further be used to detect missed waypoints)
    posControl.activeWaypoint.yaw = calculateBearingToDestination(pos);

    // Set desired position to next waypoint (XYZ-controller)
    setDesiredPosition(&posControl.activeWaypoint.pos, posControl.activeWaypoint.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
}

static void calcualteAndSetActiveWaypoint(const navWaypoint_t * waypoint)
{
    t_fp_vector localPos;
    mapWaypointToLocalPosition(&localPos, waypoint);
    calcualteAndSetActiveWaypointToLocalPosition(&localPos);
}

/**
 * Returns TRUE if we are in WP mode and executing last waypoint on the list, or in RTH mode, or in PH mode
 *  In RTH mode our only and last waypoint is home
 *  In PH mode our waypoint is hold position */
bool isApproachingLastWaypoint(void)
{
    if (navGetStateFlags(posControl.navState) & NAV_AUTO_WP) {
        if (posControl.waypointCount == 0) {
            /* No waypoints */
            return true;
        }
        else if ((posControl.activeWaypointIndex == (posControl.waypointCount - 1)) ||
                 (posControl.waypointList[posControl.activeWaypointIndex].flag == NAV_WP_FLAG_LAST)) {
            return true;
        }
        else {
            return false;
        }
    }
    else if (navGetStateFlags(posControl.navState) & NAV_CTL_POS) {
        // If POS controller is active we are in Poshold or RTH mode - assume last waypoint
        return true;
    }
    else {
        return false;
    }
}

float getActiveWaypointSpeed(void)
{
    if (posControl.flags.isAdjustingPosition) {
        // In manual control mode use different cap for speed
        return navConfig()->general.max_manual_speed;
    }
    else {
        uint16_t waypointSpeed = navConfig()->general.max_auto_speed;

        if (navGetStateFlags(posControl.navState) & NAV_AUTO_WP) {
            if (posControl.waypointCount > 0 && posControl.waypointList[posControl.activeWaypointIndex].action == NAV_WP_ACTION_WAYPOINT) {
                const float wpSpecificSpeed = posControl.waypointList[posControl.activeWaypointIndex].p1;
                if (wpSpecificSpeed >= 50.0f && wpSpecificSpeed <= navConfig()->general.max_auto_speed) {
                    waypointSpeed = wpSpecificSpeed;
                }
            }
        }

        return waypointSpeed;
    }
}

/*-----------------------------------------------------------
 * A function to reset navigation PIDs and states
 *-----------------------------------------------------------*/
void resetNavigation(void)
{
    resetAltitudeController();
    resetHeadingController();
    resetPositionController();
}

/*-----------------------------------------------------------
 * Process adjustments to alt, pos and yaw controllers
 *-----------------------------------------------------------*/
static void processNavigationRCAdjustments(void)
{
    /* Process pilot's RC input. Disable all pilot's input when in FAILSAFE_MODE */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if ((navStateFlags & NAV_RC_ALT) && (!FLIGHT_MODE(FAILSAFE_MODE))) {
        posControl.flags.isAdjustingAltitude = adjustAltitudeFromRCInput();
    }
    else {
        posControl.flags.isAdjustingAltitude = false;
    }

    if ((navStateFlags & NAV_RC_POS) && (!FLIGHT_MODE(FAILSAFE_MODE))) {
        posControl.flags.isAdjustingPosition = adjustPositionFromRCInput();
    }
    else {
        posControl.flags.isAdjustingPosition = false;
    }

    if ((navStateFlags & NAV_RC_YAW) && (!FLIGHT_MODE(FAILSAFE_MODE))) {
        posControl.flags.isAdjustingHeading = adjustHeadingFromRCInput();
    }
    else {
        posControl.flags.isAdjustingHeading = false;
    }
}

/*-----------------------------------------------------------
 * A main function to call position controllers at loop rate
 *-----------------------------------------------------------*/
void applyWaypointNavigationAndAltitudeHold(void)
{
    const timeUs_t currentTimeUs = micros();

#if defined(NAV_BLACKBOX)
    navFlags = 0;
    if (posControl.flags.hasValidAltitudeSensor)    navFlags |= (1 << 0);
    if (posControl.flags.hasValidSurfaceSensor)     navFlags |= (1 << 1);
    if (posControl.flags.hasValidPositionSensor)    navFlags |= (1 << 2);
    //if (STATE(GPS_FIX))                             navFlags |= (1 << 3);
#if defined(NAV_GPS_GLITCH_DETECTION)
    if (isGPSGlitchDetected())                      navFlags |= (1 << 4);
#endif
    if (posControl.flags.hasValidHeadingSensor)     navFlags |= (1 << 5);
#endif

    // Reset all navigation requests - NAV controllers will set them if necessary
    DISABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);

    // No navigation when disarmed
    if (!ARMING_FLAG(ARMED)) {
        // If we are disarmed, abort forced RTH
        posControl.flags.forcedRTHActivated = false;
        return;
    }

    /* Reset flags */
    posControl.flags.horizontalPositionDataConsumed = 0;
    posControl.flags.verticalPositionDataConsumed = 0;

    /* Process controllers */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if (STATE(FIXED_WING)) {
        applyFixedWingNavigationController(navStateFlags, currentTimeUs);
    }
    else {
        applyMulticopterNavigationController(navStateFlags, currentTimeUs);
    }

    /* Consume position data */
    if (posControl.flags.horizontalPositionDataConsumed)
        posControl.flags.horizontalPositionDataNew = 0;

    if (posControl.flags.verticalPositionDataConsumed)
        posControl.flags.verticalPositionDataNew = 0;


#if defined(NAV_BLACKBOX)
    if (posControl.flags.isAdjustingPosition)       navFlags |= (1 << 6);
    if (posControl.flags.isAdjustingAltitude)       navFlags |= (1 << 7);
    if (posControl.flags.isAdjustingHeading)        navFlags |= (1 << 8);

    navTargetPosition[X] = constrain(lrintf(posControl.desiredState.pos.V.X), -32678, 32767);
    navTargetPosition[Y] = constrain(lrintf(posControl.desiredState.pos.V.Y), -32678, 32767);
    navTargetPosition[Z] = constrain(lrintf(posControl.desiredState.pos.V.Z), -32678, 32767);
    navTargetSurface = constrain(lrintf(posControl.desiredState.surface), -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * Set CF's FLIGHT_MODE from current NAV_MODE
 *-----------------------------------------------------------*/
void swithNavigationFlightModes(void)
{
    const flightModeFlags_e enabledNavFlightModes = navGetMappedFlightModes(posControl.navState);
    const flightModeFlags_e disabledFlightModes = (NAV_ALTHOLD_MODE | NAV_RTH_MODE | NAV_POSHOLD_MODE | NAV_WP_MODE | NAV_LAUNCH_MODE) & (~enabledNavFlightModes);
    DISABLE_FLIGHT_MODE(disabledFlightModes);
    ENABLE_FLIGHT_MODE(enabledNavFlightModes);
}

/*-----------------------------------------------------------
 * desired NAV_MODE from combination of FLIGHT_MODE flags
 *-----------------------------------------------------------*/
static bool canActivateAltHoldMode(void)
{
    return posControl.flags.hasValidAltitudeSensor;
}

static bool canActivatePosHoldMode(void)
{
    return posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor;
}

static navigationFSMEvent_t selectNavEventFromBoxModeInput(void)
{
    static bool canActivateWaypoint = false;
    static bool canActivateLaunchMode = false;

    //We can switch modes only when ARMED
    if (ARMING_FLAG(ARMED)) {
        // Flags if we can activate certain nav modes (check if we have required sensors and they provide valid data)
        bool canActivateAltHold = canActivateAltHoldMode();
        bool canActivatePosHold = canActivatePosHoldMode();

        // LAUNCH mode has priority over any other NAV mode
        if (STATE(FIXED_WING)) {
            if (IS_RC_MODE_ACTIVE(BOXNAVLAUNCH)) {     // FIXME: Only available for fixed wing aircrafts now
                if (canActivateLaunchMode) {
                    canActivateLaunchMode = false;
                    return NAV_FSM_EVENT_SWITCH_TO_LAUNCH;
                }
                else if FLIGHT_MODE(NAV_LAUNCH_MODE) {
                    // Make sure we don't bail out to IDLE
                    return NAV_FSM_EVENT_NONE;
                }
            }
            else {
                // If we were in LAUNCH mode - force switch to IDLE
                if (FLIGHT_MODE(NAV_LAUNCH_MODE)) {
                    return NAV_FSM_EVENT_SWITCH_TO_IDLE;
                }
            }
        }

        // RTH/Failsafe_RTH can override PASSTHRU
        if (posControl.flags.forcedRTHActivated || (IS_RC_MODE_ACTIVE(BOXNAVRTH) && canActivatePosHold && canActivateAltHold && STATE(GPS_FIX_HOME))) {
            // If we request forced RTH - attempt to activate it no matter what
            // This might switch to emergency landing controller if GPS is unavailable
            canActivateWaypoint = false;    // Block WP mode if we switched to RTH for whatever reason
            return NAV_FSM_EVENT_SWITCH_TO_RTH;
        }

        // PASSTHRU mode has priority over WP/PH/AH
        if (IS_RC_MODE_ACTIVE(BOXPASSTHRU)) {
            canActivateWaypoint = false;    // Block WP mode if we are in PASSTHROUGH mode
            return NAV_FSM_EVENT_SWITCH_TO_IDLE;
        }

        if (IS_RC_MODE_ACTIVE(BOXNAVWP)) {
            if ((FLIGHT_MODE(NAV_WP_MODE)) || (canActivateWaypoint && canActivatePosHold && canActivateAltHold && STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED) && posControl.waypointListValid && (posControl.waypointCount > 0)))
                return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT;
        }
        else {
            // Arm the state variable if the WP BOX mode is not enabled
            canActivateWaypoint = true;
        }

        if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD) && IS_RC_MODE_ACTIVE(BOXNAVALTHOLD)) {
            if ((FLIGHT_MODE(NAV_ALTHOLD_MODE) && FLIGHT_MODE(NAV_POSHOLD_MODE)) || (canActivatePosHold && canActivateAltHold))
                return NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D;
        }

        if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD)) {
            if ((FLIGHT_MODE(NAV_POSHOLD_MODE)) || (canActivatePosHold))
                return NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D;
        }

        if (IS_RC_MODE_ACTIVE(BOXNAVALTHOLD)) {
            if ((FLIGHT_MODE(NAV_ALTHOLD_MODE)) || (canActivateAltHold))
                return NAV_FSM_EVENT_SWITCH_TO_ALTHOLD;
        }
    }
    else {
        canActivateWaypoint = false;

        // Launch mode can only be activated if BOX is turned on prior to arming (avoid switching to LAUNCH in flight)
        canActivateLaunchMode = IS_RC_MODE_ACTIVE(BOXNAVLAUNCH);
    }

    return NAV_FSM_EVENT_SWITCH_TO_IDLE;
}

/*-----------------------------------------------------------
 * An indicator that throttle tilt compensation is forced
 *-----------------------------------------------------------*/
bool navigationRequiresThrottleTiltCompensation(void)
{
    return !STATE(FIXED_WING) && (navGetStateFlags(posControl.navState) & NAV_REQUIRE_THRTILT);
}

/*-----------------------------------------------------------
 * An indicator that ANGLE mode must be forced per NAV requirement
 *-----------------------------------------------------------*/
bool navigationRequiresAngleMode(void)
{
    const navigationFSMStateFlags_t currentState = navGetStateFlags(posControl.navState);
    return (currentState & NAV_REQUIRE_ANGLE) || ((currentState & NAV_REQUIRE_ANGLE_FW) && STATE(FIXED_WING));
}

/*-----------------------------------------------------------
 * An indicator that TURN ASSISTANCE is required for navigation
 *-----------------------------------------------------------*/
bool navigationRequiresTurnAssistance(void)
{
    const navigationFSMStateFlags_t currentState = navGetStateFlags(posControl.navState);
    if (STATE(FIXED_WING)) {
        // For airplanes turn assistant is always required when controlling position
        return (currentState & NAV_CTL_POS);
    }
    else {
        return false;
    }
}

/**
 * An indicator that NAV is in charge of heading control (a signal to disable other heading controllers)
 */
int8_t navigationGetHeadingControlState(void)
{
    // For airplanes report as manual heading control
    if (STATE(FIXED_WING)) {
        return NAV_HEADING_CONTROL_MANUAL;
    }

    // For multirotors it depends on navigation system mode
    if (navGetStateFlags(posControl.navState) & NAV_REQUIRE_MAGHOLD) {
        if (posControl.flags.isAdjustingHeading) {
            return NAV_HEADING_CONTROL_MANUAL;
        }
        else {
            return NAV_HEADING_CONTROL_AUTO;
        }
    }
    else {
        return NAV_HEADING_CONTROL_NONE;
    }
}

bool navigationBlockArming(void)
{
    const bool navBoxModesEnabled = IS_RC_MODE_ACTIVE(BOXNAVRTH) || IS_RC_MODE_ACTIVE(BOXNAVWP) || IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD);
    const bool navLaunchComboModesEnabled = IS_RC_MODE_ACTIVE(BOXNAVLAUNCH) && (IS_RC_MODE_ACTIVE(BOXNAVRTH) || IS_RC_MODE_ACTIVE(BOXNAVWP));
    bool shouldBlockArming = false;

    if (!navConfig()->general.flags.extra_arming_safety)
        return false;

    // Apply extra arming safety only if pilot has any of GPS modes configured
    if ((isUsingNavigationModes() || failsafeMayRequireNavigationMode()) && !(posControl.flags.hasValidPositionSensor && STATE(GPS_FIX_HOME))) {
        shouldBlockArming = true;
    }

    // Don't allow arming if any of NAV modes is active
    if (!ARMING_FLAG(ARMED) && navBoxModesEnabled && !navLaunchComboModesEnabled) {
        shouldBlockArming = true;
    }

    // Don't allow arming if first waypoint is farther than configured safe distance
    if (posControl.waypointCount > 0) {
        t_fp_vector startingWaypointPos;
        mapWaypointToLocalPosition(&startingWaypointPos, &posControl.waypointList[0]);

        const bool navWpMissionStartTooFar = calculateDistanceToDestination(&startingWaypointPos) > navConfig()->general.waypoint_safe_distance;

        if (navWpMissionStartTooFar) {
            shouldBlockArming = true;
        }
    }

    return shouldBlockArming;
}


bool navigationPositionEstimateIsHealthy(void)
{
    return posControl.flags.hasValidPositionSensor && STATE(GPS_FIX_HOME);
}

/**
 * Indicate ready/not ready status
 */
static void updateReadyStatus(void)
{
    static bool posReadyBeepDone = false;

    /* Beep out READY_BEEP once when position lock is firstly acquired and HOME set */
    if (navigationPositionEstimateIsHealthy() && !posReadyBeepDone) {
        beeper(BEEPER_READY_BEEP);
        posReadyBeepDone = true;
    }
}

void updateFlightBehaviorModifiers(void)
{
    if (posControl.flags.isGCSAssistedNavigationEnabled && !IS_RC_MODE_ACTIVE(BOXGCSNAV)) {
        posControl.flags.isGCSAssistedNavigationReset = true;
    }

    posControl.flags.isGCSAssistedNavigationEnabled = IS_RC_MODE_ACTIVE(BOXGCSNAV);
    posControl.flags.isTerrainFollowEnabled = IS_RC_MODE_ACTIVE(BOXSURFACE);
}

/**
 * Process NAV mode transition and WP/RTH state machine
 *  Update rate: RX (data driven or 50Hz)
 */
void updateWaypointsAndNavigationMode(void)
{
    /* Initiate home position update */
    updateHomePosition();

    /* Update flight statistics */
    updateNavigationFlightStatistics();

    /* Update NAV ready status */
    updateReadyStatus();

    // Update flight behaviour modifiers
    updateFlightBehaviorModifiers();

    // Process switch to a different navigation mode (if needed)
    navProcessFSMEvents(selectNavEventFromBoxModeInput());

    // Process pilot's RC input to adjust behaviour
    processNavigationRCAdjustments();

    // Map navMode back to enabled flight modes
    swithNavigationFlightModes();

#if defined(NAV_BLACKBOX)
    navCurrentState = (int16_t)posControl.navState;
#endif
}

/*-----------------------------------------------------------
 * NAV main control functions
 *-----------------------------------------------------------*/
void navigationUsePIDs(void)
{
    /** Multicopter PIDs */
    // Brake time parameter
    posControl.posDecelerationTime = (float)pidProfile()->bank_mc.pid[PID_POS_XY].I / 100.0f;

    // Position controller expo (taret vel expo for MC)
    posControl.posResponseExpo = constrainf((float)pidProfile()->bank_mc.pid[PID_POS_XY].D / 100.0f, 0.0f, 1.0f);

    // Initialize position hold P-controller
    for (int axis = 0; axis < 2; axis++) {
        navPInit(&posControl.pids.pos[axis], (float)pidProfile()->bank_mc.pid[PID_POS_XY].P / 100.0f);

        navPidInit(&posControl.pids.vel[axis], (float)pidProfile()->bank_mc.pid[PID_VEL_XY].P / 100.0f,
                                               (float)pidProfile()->bank_mc.pid[PID_VEL_XY].I / 100.0f,
                                               (float)pidProfile()->bank_mc.pid[PID_VEL_XY].D / 100.0f);
    }

    // Initialize altitude hold PID-controllers (pos_z, vel_z, acc_z
    navPInit(&posControl.pids.pos[Z], (float)pidProfile()->bank_mc.pid[PID_POS_Z].P / 100.0f);

    navPidInit(&posControl.pids.vel[Z], (float)pidProfile()->bank_mc.pid[PID_VEL_Z].P / 66.7f,
                                        (float)pidProfile()->bank_mc.pid[PID_VEL_Z].I / 20.0f,
                                        (float)pidProfile()->bank_mc.pid[PID_VEL_Z].D / 100.0f);

    // Initialize surface tracking PID
    navPidInit(&posControl.pids.surface, 2.0f,
                                         0.0f,
                                         0.0f);

    /** Airplane PIDs */
    // Initialize fixed wing PID controllers
    navPidInit(&posControl.pids.fw_nav, (float)pidProfile()->bank_fw.pid[PID_POS_XY].P / 100.0f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_XY].I / 100.0f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_XY].D / 100.0f);

    navPidInit(&posControl.pids.fw_alt, (float)pidProfile()->bank_fw.pid[PID_POS_Z].P / 9.80665f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_Z].I / 9.80665f,
                                        (float)pidProfile()->bank_fw.pid[PID_POS_Z].D / 9.80665f);
}

void navigationInit(void)
{
    /* Initial state */
    posControl.navState = NAV_STATE_IDLE;

    posControl.flags.horizontalPositionDataNew = 0;
    posControl.flags.verticalPositionDataNew = 0;
    posControl.flags.surfaceDistanceDataNew = 0;
    posControl.flags.headingDataNew = 0;

    posControl.flags.hasValidAltitudeSensor = 0;
    posControl.flags.hasValidPositionSensor = 0;
    posControl.flags.hasValidSurfaceSensor = 0;
    posControl.flags.hasValidHeadingSensor = 0;

    posControl.flags.forcedRTHActivated = 0;
    posControl.waypointCount = 0;
    posControl.activeWaypointIndex = 0;
    posControl.waypointListValid = false;

    /* Set initial surface invalid */
    posControl.actualState.surface = -1.0f;
    posControl.actualState.surfaceVel = 0.0f;
    posControl.actualState.surfaceMin = -1.0f;

    /* Reset statistics */
    posControl.totalTripDistance = 0.0f;

    /* Use system config */
    navigationUsePIDs();
}

/*-----------------------------------------------------------
 * Access to estimated position/velocity data
 *-----------------------------------------------------------*/
float getEstimatedActualVelocity(int axis)
{
    return posControl.actualState.vel.A[axis];
}

float getEstimatedActualPosition(int axis)
{
    return posControl.actualState.pos.A[axis];
}

/*-----------------------------------------------------------
 * Ability to execute RTH on external event
 *-----------------------------------------------------------*/
void activateForcedRTH(void)
{
    posControl.flags.forcedRTHActivated = true;
    navProcessFSMEvents(selectNavEventFromBoxModeInput());
}

void abortForcedRTH(void)
{
    posControl.flags.forcedRTHActivated = false;
    navProcessFSMEvents(selectNavEventFromBoxModeInput());
}

rthState_e getStateOfForcedRTH(void)
{
    /* If forced RTH activated and in AUTO_RTH or EMERG state */
    if (posControl.flags.forcedRTHActivated && (navGetStateFlags(posControl.navState) & (NAV_AUTO_RTH | NAV_CTL_EMERG))) {
        if (posControl.navState == NAV_STATE_RTH_FINISHED || posControl.navState == NAV_STATE_EMERGENCY_LANDING_FINISHED) {
            return RTH_HAS_LANDED;
        }
        else {
            return RTH_IN_PROGRESS;
        }
    }
    else {
        return RTH_IDLE;
    }
}

#else // NAV

#ifdef GPS
/* Fallback if navigation is not compiled in - handle GPS home coordinates */
static float GPS_scaleLonDown;
static float GPS_totalTravelDistance = 0;

static void GPS_distance_cm_bearing(int32_t currentLat1, int32_t currentLon1, int32_t destinationLat2, int32_t destinationLon2, uint32_t *dist, int32_t *bearing)
{
    const float dLat = destinationLat2 - currentLat1; // difference of latitude in 1/10 000 000 degrees
    const float dLon = (float)(destinationLon2 - currentLon1) * GPS_scaleLonDown;

    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;

    *bearing = 9000.0f + RADIANS_TO_CENTIDEGREES(atan2_approx(-dLat, dLon));      // Convert the output radians to 100xdeg

    if (*bearing < 0)
        *bearing += 36000;
}

void onNewGPSData(void)
{
    static timeMs_t previousTimeMs = 0;
    const timeMs_t currentTimeMs = millis();
    const timeDelta_t timeDeltaMs = currentTimeMs - previousTimeMs;
    previousTimeMs = currentTimeMs;

    if (!(sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5))
        return;

    if (ARMING_FLAG(ARMED)) {
        /* Update home distance and direction */
        if (STATE(GPS_FIX_HOME)) {
            uint32_t dist;
            int32_t dir;
            GPS_distance_cm_bearing(gpsSol.llh.lat, gpsSol.llh.lon, GPS_home.lat, GPS_home.lon, &dist, &dir);
            GPS_distanceToHome = dist / 100;
            GPS_directionToHome = dir / 100;
        } else {
            GPS_distanceToHome = 0;
            GPS_directionToHome = 0;
        }

        /* Update trip distance */
        GPS_totalTravelDistance += gpsSol.groundSpeed * MS2S(timeDeltaMs);
    }
    else {
        // Set home position to current GPS coordinates
        ENABLE_STATE(GPS_FIX_HOME);
        GPS_home.lat = gpsSol.llh.lat;
        GPS_home.lon = gpsSol.llh.lon;
        GPS_home.alt = gpsSol.llh.alt;
        GPS_distanceToHome = 0;
        GPS_directionToHome = 0;
        GPS_scaleLonDown = cos_approx((ABS((float)gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f);
    }
}

int32_t getTotalTravelDistance(void)
{
    return lrintf(GPS_totalTravelDistance);
}
#endif

#endif  // NAV
