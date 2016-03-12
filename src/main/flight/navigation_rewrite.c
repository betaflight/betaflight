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

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"

/*-----------------------------------------------------------
 * Compatibility for home position
 *-----------------------------------------------------------*/
gpsLocation_t GPS_home;
uint16_t      GPS_distanceToHome;        // distance to home point in meters
int16_t       GPS_directionToHome;       // direction to home point in degrees

#if defined(NAV)
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
int16_t navDebug[4];
uint16_t navFlags;
#endif

static void updateDesiredRTHAltitude(void);
static void resetAltitudeController(void);
static void resetPositionController(void);
static void setupAltitudeController(void);
void resetNavigation(void);

static void calcualteAndSetActiveWaypoint(navWaypoint_t * waypoint);
static void calcualteAndSetActiveWaypointToLocalPosition(t_fp_vector * pos);
void calculateInitialHoldPosition(t_fp_vector * pos);
void calculateFarAwayTarget(t_fp_vector * farAwayPos, int32_t yaw, int32_t distance);

/*************************************************************************************************/
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_IDLE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_HEAD_HOME(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_GPS_FAILING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HEAD_HOME(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_GPS_FAILING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_PRE_ACTION(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_REACHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED(navigationFSMState_t previousState);

static navigationFSMStateDescriptor_t navFSM[NAV_STATE_COUNT] = {
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

    /** RTH mode entry point ************************************************/
    [NAV_STATE_RTH_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_INITIALIZE,
        .timeoutMs = 10,
        .stateFlags = NAV_REQUIRE_ANGLE | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_SPOILED_GPS,    // we are stuck in this state only if GPS is compromised
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_INITIALIZE,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_RTH_2D]            = NAV_STATE_RTH_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH_3D]            = NAV_STATE_RTH_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    /** RTH_2D mode ************************************************/
    [NAV_STATE_RTH_2D_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_2D_HEAD_HOME] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_HEAD_HOME,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_2D_HEAD_HOME,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_FINISHING,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_RTH_2D_GPS_FAILING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_2D_GPS_FAILING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_GPS_FAILING,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_SPOILED_GPS,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_2D_GPS_FAILING,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_2D_FINISHING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_FINISHING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_FINISHED,
        }
    },

    [NAV_STATE_RTH_2D_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_FINISHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_2D_FINISHED,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** RTH_3D mode ************************************************/
    [NAV_STATE_RTH_3D_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,     // allow pos adjustment while climbind to safe alt
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_WAIT_FOR_RTH_ALT,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,   // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_HEAD_HOME,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_RTH_3D_GPS_FAILING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_HEAD_HOME] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_HEAD_HOME,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_HEAD_HOME,           // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_RTH_3D_GPS_FAILING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_GPS_FAILING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_GPS_FAILING,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_ENROUTE,
        .mwError = MW_NAV_ERROR_SPOILED_GPS,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_GPS_FAILING,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,
        .timeoutMs = 2500,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_SETTLE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_LANDING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_LANDING,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_LANDING,         // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_FINISHING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_FINISHING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_FINISHING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_IN_PROGRESS,
        .mwError = MW_NAV_ERROR_LANDING,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_RTH_3D_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_FINISHED,
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LANDED,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_FINISHED,         // re-process state
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
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_PROCESS_NEXT,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_PRE_ACTION,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH]               = NAV_STATE_RTH_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
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
    /* If previous controller was NOT executing NAV_CTL_ALT controller, we must reset altitude setpoint */
    if ((navGetStateFlags(previousState) & NAV_CTL_ALT) == 0) {
        resetAltitudeController();
        setupAltitudeController();

        // If low enough and surface offset valid - enter surface tracking
        if (posControl.flags.hasValidSurfaceSensor) {
            setDesiredSurfaceOffset(posControl.actualState.surface);
        }
        else {
            setDesiredSurfaceOffset(-1.0f);
        }

        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_2D_INITIALIZE(navigationFSMState_t previousState)
{
    navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

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
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_POSHOLD_3D_INITIALIZE(navigationFSMState_t previousState)
{
    navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    if ((prevFlags & NAV_CTL_POS) == 0) {
        resetPositionController();
    }

    if ((prevFlags & NAV_CTL_ALT) == 0) {
        resetAltitudeController();
        setupAltitudeController();
    }

    if (((prevFlags & NAV_CTL_ALT) == 0) || ((prevFlags & NAV_AUTO_RTH) != 0) || ((prevFlags & NAV_AUTO_WP) != 0)) {
        // If low enough and surface offset valid - enter surface tracking
        if (posControl.flags.hasValidSurfaceSensor) {
            setDesiredSurfaceOffset(posControl.actualState.surface);
        }
        else {
            setDesiredSurfaceOffset(-1.0f);
        }

        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
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
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_INITIALIZE(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    /* All good for RTH */
    if (posControl.flags.hasValidPositionSensor &&  posControl.flags.hasValidHeadingSensor && STATE(GPS_FIX_HOME)) {
        // Switch between 2D and 3D RTH depending on altitude sensor availability
        if (posControl.flags.hasValidAltitudeSensor) {
            return NAV_FSM_EVENT_SWITCH_TO_RTH_3D;
        }
        else {
            return NAV_FSM_EVENT_SWITCH_TO_RTH_2D;
        }
    }
    /* No HOME set or position sensor failure timeout - land */
    else if (!STATE(GPS_FIX_HOME) || checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    /* No valid POS sensor but still within valid timeout - wait */
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_INITIALIZE(navigationFSMState_t previousState)
{
    navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    if ((prevFlags & NAV_CTL_POS) == 0) {
        resetPositionController();
    }

    // If close to home - reset home position
    if (posControl.homeDistance < posControl.navConfig->min_rth_distance) {
        setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw);
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - switch to NAV_STATE_RTH_2D_GPS_FAILING
    if (!posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) {
        return NAV_FSM_EVENT_ERROR;         // NAV_STATE_RTH_2D_GPS_FAILING
    }

    if (isWaypointReached(&posControl.homeWaypointAbove)) {
        // Successfully reached position target
        return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_2D_FINISHING
    }
    else {
        // Update XY-position target
        if (posControl.navConfig->flags.rth_tail_first && !STATE(FIXED_WING)) {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
        }
        else {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
        }

        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_GPS_FAILING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    /* Wait for GPS to be online again */
    if (posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor && STATE(GPS_FIX_HOME)) {
        return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_2D_HEAD_HOME
    }
    /* No HOME set or position sensor failure timeout - land */
    else if (!STATE(GPS_FIX_HOME) || checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
    /* No valid POS sensor but still within valid timeout - wait */
    else {
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHED(navigationFSMState_t previousState)
{
    // Same logic as PH_2D
    return navOnEnteringState_NAV_STATE_POSHOLD_2D_IN_PROGRESS(previousState);
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_INITIALIZE(navigationFSMState_t previousState)
{
    navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

    if ((prevFlags & NAV_CTL_POS) == 0) {
        resetPositionController();
    }

    if ((prevFlags & NAV_CTL_ALT) == 0) {
        resetAltitudeController();
        setupAltitudeController();
    }

    t_fp_vector targetHoldPos;

    // If close to home - reset home position
    if (posControl.homeDistance < posControl.navConfig->min_rth_distance) {
        setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw);
        targetHoldPos = posControl.actualState.pos;
    }
    else {
        if (!STATE(FIXED_WING)) {
            // Multicopter, hover and climb
            calculateInitialHoldPosition(&targetHoldPos);
        } else {
            // Airplane - climbout before turning around
            calculateFarAwayTarget(&targetHoldPos, posControl.actualState.yaw, 100000.0f);  // 1km away
        }
    }

    setDesiredPosition(&targetHoldPos, 0, NAV_POS_UPDATE_XY);

    return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor && checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if ((posControl.actualState.pos.V.Z - posControl.homeWaypointAbove.pos.V.Z) > -50.0f) {
        return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_3D_HEAD_HOME
    }
    else {
        // Climb to safe altitude and turn to correct direction
        if (posControl.navConfig->flags.rth_tail_first && !STATE(FIXED_WING)) {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
        }
        else {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
        }

        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) {
        return NAV_FSM_EVENT_ERROR;         // NAV_STATE_RTH_3D_GPS_FAILING
    }

    if (isWaypointReached(&posControl.homeWaypointAbove)) {
        if (!STATE(FIXED_WING)) {
            // Successfully reached position target - update XYZ-position
            setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
            return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING
        } else {
            // Don't switch to landing for airplanes
            return NAV_FSM_EVENT_NONE;
        }
    }
    else {
        // Update XYZ-position target
        if (posControl.navConfig->flags.rth_tail_first && !STATE(FIXED_WING)) {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING_TAIL_FIRST);
        }
        else {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
        }
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_GPS_FAILING(navigationFSMState_t previousState)
{
    /* Same logic as for 2D GPS RTH */
    return navOnEnteringState_NAV_STATE_RTH_2D_GPS_FAILING(previousState);
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor && checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    resetLandingDetector();

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (!ARMING_FLAG(ARMED)) {
        return NAV_FSM_EVENT_SUCCESS;
    }
    else if (isLandingDetected()) {
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        // A safeguard - if sonar is available and it is reading < 50cm altitude - drop to low descend speed
        if (posControl.flags.hasValidSurfaceSensor && posControl.actualState.surface >= 0 && posControl.actualState.surface < 50.0f) {
            // land_descent_rate == 200 : descend speed = 30 cm/s, gentle touchdown
            updateAltitudeTargetFromClimbRate(-0.15f * posControl.navConfig->land_descent_rate);
        }
        else {
            // Gradually reduce descent speed depending on actual altitude.
            if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 1500.0f)) {
                updateAltitudeTargetFromClimbRate(-1.0f * posControl.navConfig->land_descent_rate);
            }
            else if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 500.0f)) {
                updateAltitudeTargetFromClimbRate(-0.5f * posControl.navConfig->land_descent_rate);
            }
            else {
                updateAltitudeTargetFromClimbRate(-0.25f * posControl.navConfig->land_descent_rate);
            }
        }

        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    updateAltitudeTargetFromClimbRate(-0.3f * posControl.navConfig->land_descent_rate);  // FIXME
    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHED(navigationFSMState_t previousState)
{
    // Stay in this state
    UNUSED(previousState);
    updateAltitudeTargetFromClimbRate(-0.3f * posControl.navConfig->land_descent_rate);  // FIXME
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
            calcualteAndSetActiveWaypointToLocalPosition(&posControl.homeWaypointAbove.pos);
            return NAV_FSM_EVENT_SUCCESS;       // will switch to NAV_STATE_WAYPOINT_IN_PROGRESS
    };
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) {
        switch (posControl.waypointList[posControl.activeWaypointIndex].action) {
            case NAV_WP_ACTION_WAYPOINT:
            case NAV_WP_ACTION_RTH:
            default:
                if (isWaypointReached(&posControl.activeWaypoint) || isWaypointMissed(&posControl.activeWaypoint)) {
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

    bool isLastWaypoint = (posControl.waypointList[posControl.activeWaypointIndex].flag == NAV_WP_FLAG_LAST) ||
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

static navigationFSMState_t navSetNewFSMState(navigationFSMState_t newState)
{
    navigationFSMState_t previousState;

    previousState = posControl.navState;
    posControl.navState = newState;
    return previousState;
}

static void navProcessFSMEvents(navigationFSMEvent_t injectedEvent)
{
    uint32_t currentMillis = millis();
    navigationFSMState_t previousState;
    static uint32_t lastStateProcessTime = 0;

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
float navPidApply2(float setpoint, float measurement, float dt, pidController_t *pid, float outMin, float outMax, bool dTermErrorTracking)
{
    float newProportional, newDerivative;
    float error = setpoint - measurement;

    /* P-term */
    newProportional = error * pid->param.kP;

    /* D-term */
    if (dTermErrorTracking) {
        /* Error-tracking D-term */
        newDerivative = (error - pid->last_input) / dt;
        pid->last_input = error;
    }
    else {
        /* Measurement tracking D-term */
        newDerivative = -(measurement - pid->last_input) / dt;
        pid->last_input = measurement;
    }

    newDerivative = pid->param.kD * filterApplyPt1(newDerivative, &pid->dterm_filter_state, NAV_DTERM_CUT_HZ, dt);

    /* Pre-calculate output and limit it if actuator is saturating */
    float outVal = newProportional + pid->integrator + newDerivative;
    float outValConstrained = constrainf(outVal, outMin, outMax);

    /* Update I-term */
    pid->integrator += (error * pid->param.kI * dt) + ((outValConstrained - outVal) * pid->param.kT * dt);

    return outValConstrained;
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
    if (posControl.navConfig->pos_failure_timeout) {
        if (!posControl.flags.hasValidPositionSensor && ((millis() - posControl.lastValidPositionTimeMs) > (1000 * posControl.navConfig->pos_failure_timeout))) {
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

    posControl.flags.hasValidPositionSensor = hasValidSensor;
    posControl.flags.hasValidHeadingSensor = isImuHeadingValid();

    if (hasValidSensor) {
        posControl.flags.horizontalPositionNewData = 1;
        posControl.lastValidPositionTimeMs = millis();
    }
    else {
        posControl.flags.horizontalPositionNewData = 0;
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
        posControl.flags.verticalPositionNewData = 1;
        posControl.lastValidAltitudeTimeMs = millis();
    }
    else {
        posControl.flags.verticalPositionNewData = 0;
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

    if (surfaceDistance > 0) {
        if (posControl.actualState.surfaceMin > 0) {
            posControl.actualState.surfaceMin = MIN(posControl.actualState.surfaceMin, surfaceDistance);
        }
        else {
            posControl.actualState.surfaceMin = surfaceDistance;
        }
    }

    posControl.flags.hasValidSurfaceSensor = hasValidSensor;

    if (hasValidSensor) {
        posControl.flags.surfaceDistanceNewData = 1;
    }
    else {
        posControl.flags.surfaceDistanceNewData = 0;
    }
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

    posControl.flags.headingNewData = 1;
}

/*-----------------------------------------------------------
 * Calculates distance and bearing to destination point
 *-----------------------------------------------------------*/
uint32_t calculateDistanceToDestination(t_fp_vector * destinationPos)
{
    float deltaX = destinationPos->V.X - posControl.actualState.pos.V.X;
    float deltaY = destinationPos->V.Y - posControl.actualState.pos.V.Y;

    return sqrtf(sq(deltaX) + sq(deltaY));
}

int32_t calculateBearingToDestination(t_fp_vector * destinationPos)
{
    float deltaX = destinationPos->V.X - posControl.actualState.pos.V.X;
    float deltaY = destinationPos->V.Y - posControl.actualState.pos.V.Y;

    return wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(deltaY, deltaX)));
}

/*-----------------------------------------------------------
 * Check if waypoint is/was reached. Assume that waypoint-yaw stores initial bearing
 *-----------------------------------------------------------*/
bool isWaypointMissed(navWaypointPosition_t * waypoint)
{
    int32_t bearingError = calculateBearingToDestination(&waypoint->pos) - waypoint->yaw;
    bearingError = wrap_18000(bearingError);

    return ABS(bearingError) > 10000; // TRUE if we passed the waypoint by 100 degrees
}

bool isWaypointReached(navWaypointPosition_t * waypoint)
{
    // We consider waypoint reached if within specified radius
    uint32_t wpDistance = calculateDistanceToDestination(&waypoint->pos);
    return (wpDistance <= posControl.navConfig->waypoint_radius);
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
            switch (posControl.navConfig->flags.rth_alt_control_style) {
            case NAV_RTH_NO_ALT:
                posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z;
                break;
            case NAX_RTH_EXTRA_ALT: // Maintain current altitude + predefined safety margin
                posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z + posControl.navConfig->rth_altitude;
                break;
            case NAV_RTH_MAX_ALT:
                posControl.homeWaypointAbove.pos.V.Z = MAX(posControl.homeWaypointAbove.pos.V.Z, posControl.actualState.pos.V.Z);
                break;
            case NAV_RTH_AT_LEAST_ALT:  // Climb to at least some predefined altitude above home
                posControl.homeWaypointAbove.pos.V.Z = MAX(posControl.homePosition.pos.V.Z + posControl.navConfig->rth_altitude, posControl.actualState.pos.V.Z);
                break;
            case NAV_RTH_CONST_ALT:     // Climb/descend to predefined altitude above home
            default:
                posControl.homeWaypointAbove.pos.V.Z = posControl.homePosition.pos.V.Z + posControl.navConfig->rth_altitude;
                break;
            }
        }
    }
    else {
        posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z;
    }
}

/*-----------------------------------------------------------
 * Reset home position to current position
 *-----------------------------------------------------------*/
void setHomePosition(t_fp_vector * pos, int32_t yaw)
{
    posControl.homePosition.pos = *pos;
    posControl.homePosition.yaw = yaw;

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
            setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw);
        }
    }
    else {
        // If pilot so desires he may reset home position to current position
        if (IS_RC_MODE_ACTIVE(BOXHOMERESET) && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE) && posControl.flags.hasValidPositionSensor) {
            setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw);
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
 * Set surface tracking target
 *-----------------------------------------------------------*/
void setDesiredSurfaceOffset(float surfaceOffset)
{
    if (surfaceOffset > 0) {
        posControl.desiredState.surface = constrainf(surfaceOffset, 20.0f, 250.0f);
    }
    else {
        posControl.desiredState.surface = -1;
    }
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
void setDesiredPosition(t_fp_vector * pos, int32_t yaw, navSetWaypointFlags_t useMask)
{
    // XY-position
    if ((useMask & NAV_POS_UPDATE_XY) != 0) {
        posControl.desiredState.pos.V.X = pos->V.X;
        posControl.desiredState.pos.V.Y = pos->V.Y;
    }

    // Z-position
    if ((useMask & NAV_POS_UPDATE_Z) != 0) {
        posControl.desiredState.surface = -1;           // When we directly set altitude target we must reset surface tracking
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

#if defined(NAV_BLACKBOX)
    navTargetPosition[X] = constrain(lrintf(posControl.desiredState.pos.V.X), -32678, 32767);
    navTargetPosition[Y] = constrain(lrintf(posControl.desiredState.pos.V.Y), -32678, 32767);
    navTargetPosition[Z] = constrain(lrintf(posControl.desiredState.pos.V.Z), -32678, 32767);
#endif
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
static uint32_t landingTimer;

void resetLandingDetector(void)
{
    landingTimer = micros();
}

bool isLandingDetected(void)
{
    bool landingDetected;

    if (STATE(FIXED_WING)) { // FIXED_WING
        landingDetected = isFixedWingLandingDetected(&landingTimer);
    }
    else {
        landingDetected = isMulticopterLandingDetected(&landingTimer);
    }

    return landingDetected;
}

/*-----------------------------------------------------------
 * Z-position controller
 *-----------------------------------------------------------*/
void updateAltitudeTargetFromClimbRate(float climbRate)
{
    // FIXME: On FIXED_WING and multicopter this should work in a different way
    // Calculate new altitude target

    /* Move surface tracking setpoint if it is set */
    if (posControl.desiredState.surface > 0.0f && posControl.actualState.surface > 0.0f && posControl.flags.hasValidSurfaceSensor) {
        posControl.desiredState.surface = constrainf(posControl.actualState.surface + (climbRate / posControl.pids.pos[Z].param.kP), 1.0f, 200.0f);
    }

    posControl.desiredState.pos.V.Z = posControl.actualState.pos.V.Z + (climbRate / posControl.pids.pos[Z].param.kP);

#if defined(NAV_BLACKBOX)
    navTargetPosition[Z] = constrain(lrintf(posControl.desiredState.pos.V.Z), -32678, 32767);
#endif
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
    if (STATE(FIXED_WING)) {
        return adjustFixedWingPositionFromRCInput();
    }
    else {
        return adjustMulticopterPositionFromRCInput();
    }

#if defined(NAV_BLACKBOX)
    navTargetPosition[X] = constrain(lrintf(posControl.desiredState.pos.V.X), -32678, 32767);
    navTargetPosition[Y] = constrain(lrintf(posControl.desiredState.pos.V.Y), -32678, 32767);
#endif
}

/*-----------------------------------------------------------
 * WP controller
 *-----------------------------------------------------------*/
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

void setWaypoint(uint8_t wpNumber, navWaypoint_t * wpData)
{
    gpsLocation_t wpLLH;
    navWaypointPosition_t wpPos;

    // Pre-fill structure to convert to local coordinates
    wpLLH.lat = wpData->lat;
    wpLLH.lon = wpData->lon;
    wpLLH.alt = wpData->alt;

    // WP #0 - special waypoint - HOME
    if ((wpNumber == 0) && ARMING_FLAG(ARMED) && posControl.flags.hasValidPositionSensor && posControl.gpsOrigin.valid) {
        // Forcibly set home position. Note that this is only valid if already armed, otherwise home will be reset instantly
        geoConvertGeodeticToLocal(&posControl.gpsOrigin, &wpLLH, &wpPos.pos, GEO_ALT_RELATIVE);
        setHomePosition(&wpPos.pos, 0);
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

        setDesiredPosition(&wpPos.pos, DEGREES_TO_DECIDEGREES(wpData->p1), waypointUpdateFlags);
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

static void calcualteAndSetActiveWaypointToLocalPosition(t_fp_vector * pos)
{
    posControl.activeWaypoint.pos = *pos;

    // Calculate initial bearing towards waypoint and store it in waypoint yaw parameter (this will further be used to detect missed waypoints)
    posControl.activeWaypoint.yaw = calculateBearingToDestination(pos);

    // Set desired position to next waypoint (XYZ-controller)
    setDesiredPosition(&posControl.activeWaypoint.pos, posControl.activeWaypoint.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
}

static void calcualteAndSetActiveWaypoint(navWaypoint_t * waypoint)
{
    gpsLocation_t wpLLH;
    t_fp_vector localPos;

    wpLLH.lat = waypoint->lat;
    wpLLH.lon = waypoint->lon;
    wpLLH.alt = waypoint->alt;

    geoConvertGeodeticToLocal(&posControl.gpsOrigin, &wpLLH, &localPos, GEO_ALT_RELATIVE);
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
    uint16_t waypointSpeed = posControl.navConfig->max_speed;

    if (navGetStateFlags(posControl.navState) & NAV_AUTO_WP) {
        if (posControl.waypointCount > 0 && posControl.waypointList[posControl.activeWaypointIndex].action == NAV_WP_ACTION_WAYPOINT) {
            waypointSpeed = posControl.waypointList[posControl.activeWaypointIndex].p1;

            if (waypointSpeed < 50 || waypointSpeed > posControl.navConfig->max_speed) {
                waypointSpeed = posControl.navConfig->max_speed;
            }
        }
    }

    return waypointSpeed;
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
    uint32_t currentTime = micros();

#if defined(NAV_BLACKBOX)
    navFlags = 0;
    if (posControl.flags.hasValidAltitudeSensor)    navFlags |= (1 << 0);
    if (posControl.flags.hasValidSurfaceSensor)     navFlags |= (1 << 1);
    if (posControl.flags.hasValidPositionSensor)    navFlags |= (1 << 2);
    if ((STATE(GPS_FIX) && gpsSol.numSat >= posControl.navConfig->inav.gps_min_sats)) navFlags |= (1 << 3);
    if (isGPSGlitchDetected())                      navFlags |= (1 << 4);
#endif

    // No navigation when disarmed
    if (!ARMING_FLAG(ARMED)) {
        // If we are disarmed, abort forced RTH
        posControl.flags.forcedRTHActivated = false;
        return;
    }

    /* Process controllers */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if (STATE(FIXED_WING)) {
        applyFixedWingNavigationController(navStateFlags, currentTime);
    }
    else {
        applyMulticopterNavigationController(navStateFlags, currentTime);
    }

#if defined(NAV_BLACKBOX)
    if (posControl.flags.isAdjustingPosition)       navFlags |= (1 << 5);
    if (posControl.flags.isAdjustingAltitude)       navFlags |= (1 << 6);
    if (posControl.flags.isAdjustingHeading)        navFlags |= (1 << 7);
#endif
}

/*-----------------------------------------------------------
 * Set CF's FLIGHT_MODE from current NAV_MODE
 *-----------------------------------------------------------*/
void swithNavigationFlightModes(void)
{
    flightModeFlags_e enabledNavFlightModes = navGetMappedFlightModes(posControl.navState);
    flightModeFlags_e disabledFlightModes = (NAV_ALTHOLD_MODE | NAV_RTH_MODE | NAV_POSHOLD_MODE | NAV_WP_MODE) & (~enabledNavFlightModes);
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
    // Flags if we can activate certain nav modes (check if we have required sensors and they provide valid data)
    bool canActivateAltHold = canActivateAltHoldMode();
    bool canActivatePosHold = canActivatePosHoldMode();

    // RTH/Failsafe_RTH can override PASSTHRU
    if (posControl.flags.forcedRTHActivated) {
        // If we request forced RTH - attempt to activate it no matter what
        // This might switch to emergency landing controller if GPS is unavailable
        return NAV_FSM_EVENT_SWITCH_TO_RTH;
    }
    else if (IS_RC_MODE_ACTIVE(BOXNAVRTH)) {
        if ((FLIGHT_MODE(NAV_RTH_MODE)) || (canActivatePosHold && STATE(GPS_FIX_HOME)))
            return NAV_FSM_EVENT_SWITCH_TO_RTH;
    }

    // PASSTHRU mode has priority over WP/PH/AH
    if (IS_RC_MODE_ACTIVE(BOXPASSTHRU)) {
        return NAV_FSM_EVENT_SWITCH_TO_IDLE;
    }

    if (IS_RC_MODE_ACTIVE(BOXNAVWP)) {
        if ((FLIGHT_MODE(NAV_WP_MODE)) || (canActivatePosHold && canActivateAltHold && STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED) && posControl.waypointListValid && (posControl.waypointCount > 0)))
            return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT;
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
bool naivationRequiresAngleMode(void)
{
    navigationFSMStateFlags_t currentState = navGetStateFlags(posControl.navState);
    return (currentState & NAV_REQUIRE_ANGLE) || ((currentState & NAV_REQUIRE_ANGLE_FW) && STATE(FIXED_WING));
}

/**
 * An indicator that NAV is in charge of heading control (a signal to disable other heading controllers)
 */
int8_t naivationGetHeadingControlState(void)
{
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

bool naivationBlockArming(void)
{
    if (!posControl.navConfig->flags.extra_arming_safety)
        return false;

    // Apply extra arming safety only if pilot has any of GPS modes configured
    if (isUsingNavigationModes() || failsafeMayRequireNavigationMode()) {
        return !(posControl.flags.hasValidPositionSensor && STATE(GPS_FIX_HOME));
    }
    else {
        return false;
    }
}

/**
 * Indicate ready/not ready status
 */
static void updateReadyStatus(void)
{
    static bool posReadyBeepDone = false;

    /* Beep out READY_BEEP once when position lock is firstly acquired and HOME set */
    if (posControl.flags.hasValidPositionSensor && STATE(GPS_FIX_HOME) && !posReadyBeepDone) {
        beeper(BEEPER_READY_BEEP);
        posReadyBeepDone = true;
    }
}

void updateFlightBehaviorModifiers(void)
{
    posControl.flags.isGCSAssistedNavigationEnabled = IS_RC_MODE_ACTIVE(BOXGCSNAV);
}

/**
 * Process NAV mode transition and WP/RTH state machine
 *  Update rate: RX (data driven or 50Hz)
 */
void updateWaypointsAndNavigationMode(void)
{
    /* Initiate home position update */
    updateHomePosition();

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
void navigationUseConfig(navConfig_t *navConfigToUse)
{
    posControl.navConfig = navConfigToUse;
}

void navigationUseRcControlsConfig(rcControlsConfig_t *initialRcControlsConfig)
{
    posControl.rcControlsConfig = initialRcControlsConfig;
}

void navigationUseRxConfig(rxConfig_t * initialRxConfig)
{
    posControl.rxConfig = initialRxConfig;
}

void navigationUseEscAndServoConfig(escAndServoConfig_t * initialEscAndServoConfig)
{
    posControl.escAndServoConfig = initialEscAndServoConfig;
}

void navigationUsePIDs(pidProfile_t *initialPidProfile)
{
    int axis;

    posControl.pidProfile = initialPidProfile;

    // Brake time parameter
    posControl.posDecelerationTime = (float)posControl.pidProfile->I8[PIDPOS] / 100.0f;

    // Position controller expo (taret vel expo for MC)
    posControl.posResponseExpo = constrainf((float)posControl.pidProfile->D8[PIDPOS] / 100.0f, 0.0f, 1.0f);

    // Initialize position hold P-controller
    for (axis = 0; axis < 2; axis++) {
        navPInit(&posControl.pids.pos[axis], (float)posControl.pidProfile->P8[PIDPOS] / 100.0f);

        navPidInit(&posControl.pids.vel[axis], (float)posControl.pidProfile->P8[PIDPOSR] / 100.0f,
                                               (float)posControl.pidProfile->I8[PIDPOSR] / 100.0f,
                                               (float)posControl.pidProfile->D8[PIDPOSR] / 100.0f);
    }

    // Initialize altitude hold PID-controllers (pos_z, vel_z, acc_z
    navPInit(&posControl.pids.pos[Z], (float)posControl.pidProfile->P8[PIDALT] / 100.0f);

    navPidInit(&posControl.pids.vel[Z], (float)posControl.pidProfile->P8[PIDVEL] / 100.0f,
                                        (float)posControl.pidProfile->I8[PIDVEL] / 100.0f,
                                        (float)posControl.pidProfile->D8[PIDVEL] / 100.0f);

    // Initialize fixed wing PID controllers
    navPidInit(&posControl.pids.fw_nav, (float)posControl.pidProfile->P8[PIDNAVR] / 100.0f,
                                        (float)posControl.pidProfile->I8[PIDNAVR] / 100.0f,
                                        (float)posControl.pidProfile->D8[PIDNAVR] / 100.0f);

    navPidInit(&posControl.pids.fw_alt, (float)posControl.pidProfile->P8[PIDALT] / 100.0f,
                                        (float)posControl.pidProfile->I8[PIDALT] / 100.0f,
                                        (float)posControl.pidProfile->D8[PIDALT] / 100.0f);
}

void navigationInit(navConfig_t *initialnavConfig,
                    pidProfile_t *initialPidProfile,
                    rcControlsConfig_t *initialRcControlsConfig,
                    rxConfig_t * initialRxConfig,
                    escAndServoConfig_t * initialEscAndServoConfig)
{
    /* Initial state */
    posControl.navState = NAV_STATE_IDLE;

    posControl.flags.horizontalPositionNewData = 0;
    posControl.flags.verticalPositionNewData = 0;
    posControl.flags.surfaceDistanceNewData = 0;
    posControl.flags.headingNewData = 0;

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

    /* Use system config */
    navigationUseConfig(initialnavConfig);
    navigationUsePIDs(initialPidProfile);
    navigationUseRcControlsConfig(initialRcControlsConfig);
    navigationUseRxConfig(initialRxConfig);
    navigationUseEscAndServoConfig(initialEscAndServoConfig);
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
 * Interface with PIDs: Angle-Command transformation
 *-----------------------------------------------------------*/
int16_t rcCommandToLeanAngle(int16_t rcCommand)
{
    if (posControl.pidProfile->pidController == PID_CONTROLLER_LUX_FLOAT) {
        // LuxFloat is the only PID controller that uses raw rcCommand as target angle
        return rcCommand;
    }
    else {
        // Most PID controllers use 2 * rcCommand as target angle for ANGLE mode
        return rcCommand * 2;
    }
}

int16_t leanAngleToRcCommand(int16_t leanAngle)
{
    if (posControl.pidProfile->pidController == PID_CONTROLLER_LUX_FLOAT) {
        // LuxFloat is the only PID controller that uses raw rcCommand as target angle
        return leanAngle;
    }
    else {
        // Most PID controllers use 2 * rcCommand as target angle for ANGLE mode
        return leanAngle / 2;
    }
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
        if (posControl.navState == NAV_STATE_RTH_2D_FINISHED || posControl.navState == NAV_STATE_RTH_3D_FINISHED || posControl.navState == NAV_STATE_EMERGENCY_LANDING_FINISHED) {
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

static void GPS_distance_cm_bearing(int32_t currentLat1, int32_t currentLon1, int32_t destinationLat2, int32_t destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = destinationLat2 - currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(destinationLon2 - currentLon1) * GPS_scaleLonDown;

    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;

    *bearing = 9000.0f + RADIANS_TO_CENTIDEGREES(atan2_approx(-dLat, dLon));      // Convert the output radians to 100xdeg

    if (*bearing < 0)
        *bearing += 36000;
}

void onNewGPSData()
{
    if (!(sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 5))
        return;

    if (ARMING_FLAG(ARMED)) {
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
#endif

#endif  // NAV
