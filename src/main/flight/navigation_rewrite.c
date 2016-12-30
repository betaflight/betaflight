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

#include "drivers/system.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "fc/runtime_config.h"
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
int16_t navTargetSurface;
int16_t navActualSurface;
uint16_t navFlags;
uint16_t navEPH;
uint16_t navEPV;
int16_t navAccNEU[3];
#endif
int16_t navDebug[4];

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
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_2D_INITIALIZE,      // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
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
        .timeoutMs = 10,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_RTH_START,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_INITIALIZE,      // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH_3D_LANDING]    = NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,
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
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
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
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
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
        .timeoutMs = 500,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .mwState = MW_NAV_STATE_LAND_SETTLE,
        .mwError = MW_NAV_ERROR_NONE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_LANDING,
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
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
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
    if (STATE(GPS_FIX_HOME)) {
        if (posControl.flags.hasValidHeadingSensor) {
            navigationFSMStateFlags_t prevFlags = navGetStateFlags(previousState);

            if ((prevFlags & NAV_CTL_POS) == 0) {
                resetPositionController();
            }

            if ((prevFlags & NAV_CTL_ALT) == 0) {
                resetAltitudeController();
                setupAltitudeController();
            }

            // Switch between 2D and 3D RTH depending on altitude sensor availability
            if (posControl.flags.hasValidAltitudeSensor) {
                // We might have GPS unavailable - in case of 3D RTH set current altitude target
                setDesiredPosition(&posControl.actualState.pos, 0, NAV_POS_UPDATE_Z);

                return NAV_FSM_EVENT_SWITCH_TO_RTH_3D;
            }
            else {
                return NAV_FSM_EVENT_SWITCH_TO_RTH_2D;
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
    else {
        // No home position set or no heading sensor
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_INITIALIZE(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (STATE(FIXED_WING) && (posControl.homeDistance < posControl.navConfig->general.min_rth_distance)) {
        // Prevent RTH from activating on airplanes if too close to home
        return NAV_FSM_EVENT_SWITCH_TO_IDLE;
    }
    else {
        if (posControl.flags.hasValidPositionSensor) {
            // If close to home - reset home position
            if (posControl.homeDistance < posControl.navConfig->general.min_rth_distance) {
                setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
            }

            return NAV_FSM_EVENT_SUCCESS;
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
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - switch to NAV_STATE_RTH_2D_GPS_FAILING
    if (!(posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor)) {
        return NAV_FSM_EVENT_ERROR;         // NAV_STATE_RTH_2D_GPS_FAILING
    }

    if (isWaypointReached(&posControl.homeWaypointAbove)) {
        // Successfully reached position target
        return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_2D_FINISHING
    }
    else {
        // Update XY-position target
        if (posControl.navConfig->general.flags.rth_tail_first && !STATE(FIXED_WING)) {
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
    else if (!STATE(GPS_FIX_HOME) || !posControl.flags.hasValidHeadingSensor || checkForPositionSensorTimeout()) {
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
    UNUSED(previousState);

    if (STATE(FIXED_WING) && (posControl.homeDistance < posControl.navConfig->general.min_rth_distance)) {
        // Prevent RTH from activating on airplanes if too close to home
        return NAV_FSM_EVENT_SWITCH_TO_IDLE;
    }
    else {
        if (posControl.flags.hasValidPositionSensor) {
            // If close to home - reset home position and land
            if (posControl.homeDistance < posControl.navConfig->general.min_rth_distance) {
                setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
                setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);

                return NAV_FSM_EVENT_SWITCH_TO_RTH_3D_LANDING;   // NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING
            }
            else {
                t_fp_vector targetHoldPos;

                if (STATE(FIXED_WING)) {
                    // Airplane - climbout before turning around
                    calculateFarAwayTarget(&targetHoldPos, posControl.actualState.yaw, 100000.0f);  // 1km away
                } else {
                    // Multicopter, hover and climb
                    calculateInitialHoldPosition(&targetHoldPos);
                }

                setDesiredPosition(&targetHoldPos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);

                return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT
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
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!(posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) && checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if (((posControl.actualState.pos.V.Z - posControl.homeWaypointAbove.pos.V.Z) > -50.0f) || (!posControl.navConfig->general.flags.rth_climb_first)) {
        return NAV_FSM_EVENT_SUCCESS;   // NAV_STATE_RTH_3D_HEAD_HOME
    }
    else {
        // Climb to safe altitude and turn to correct direction
        if (posControl.navConfig->general.flags.rth_tail_first && !STATE(FIXED_WING)) {
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
    if (!(posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor)) {
        return NAV_FSM_EVENT_ERROR;         // NAV_STATE_RTH_3D_GPS_FAILING
    }

    if (isWaypointReached(&posControl.homeWaypointAbove)) {
        // Successfully reached position target - update XYZ-position
        setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
        return NAV_FSM_EVENT_SUCCESS;       // NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING
    }
    else {
        // Update XYZ-position target
        if (posControl.navConfig->general.flags.rth_tail_first && !STATE(FIXED_WING)) {
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
    if (!(posControl.flags.hasValidPositionSensor && posControl.flags.hasValidHeadingSensor) && checkForPositionSensorTimeout()) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

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
        else {
            setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
            return NAV_FSM_EVENT_NONE;
        }
    }
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
        if (STATE(FIXED_WING)) {
            // FIXME: Continue loitering at home altitude
            return NAV_FSM_EVENT_NONE;
        }
        else {
            // A safeguard - if sonar is available and it is reading < 50cm altitude - drop to low descend speed
            if (posControl.flags.hasValidSurfaceSensor && posControl.actualState.surface >= 0 && posControl.actualState.surface < 50.0f) {
                // land_descent_rate == 200 : descend speed = 30 cm/s, gentle touchdown
                // Do not allow descent velocity slower than -30cm/s so the landing detector works.
                float descentVelLimited = MIN(-0.15f * posControl.navConfig->general.land_descent_rate, -30.0f);
                updateAltitudeTargetFromClimbRate(descentVelLimited, CLIMB_RATE_RESET_SURFACE_TARGET);
            }
            else {
                // Ramp down descent velocity from 100% at maxAlt altitude to 25% from minAlt to 0cm.
                float descentVelScaling = (posControl.actualState.pos.V.Z - posControl.homePosition.pos.V.Z - posControl.navConfig->general.land_slowdown_minalt)
                                            / (posControl.navConfig->general.land_slowdown_maxalt - posControl.navConfig->general.land_slowdown_minalt) * 0.75f + 0.25f;  // Yield 1.0 at 2000 alt and 0.25 at 500 alt

                descentVelScaling = constrainf(descentVelScaling, 0.25f, 1.0f);

                // Do not allow descent velocity slower than -50cm/s so the landing detector works.
                float descentVelLimited = MIN(-descentVelScaling * posControl.navConfig->general.land_descent_rate, -50.0f);
                updateAltitudeTargetFromClimbRate(descentVelLimited, CLIMB_RATE_RESET_SURFACE_TARGET);
            }

            return NAV_FSM_EVENT_NONE;
        }
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    if (posControl.navConfig->general.flags.disarm_on_landing) {
        mwDisarm();
    }

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHED(navigationFSMState_t previousState)
{
    // Stay in this state
    UNUSED(previousState);
    updateAltitudeTargetFromClimbRate(-0.3f * posControl.navConfig->general.land_descent_rate, CLIMB_RATE_RESET_SURFACE_TARGET);  // FIXME
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

    const navigationFSMEvent_t landEvent = navOnEnteringState_NAV_STATE_RTH_3D_LANDING(previousState);
    if (landEvent == NAV_FSM_EVENT_SUCCESS) {
        // Landing controller returned success - invoke RTH finishing state and finish the waypoint
        navOnEnteringState_NAV_STATE_RTH_3D_FINISHING(previousState);
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
    const uint32_t currentMillis = millis();
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

    newDerivative = pid->param.kD * pt1FilterApply4(&pid->dterm_filter_state, newDerivative, NAV_DTERM_CUT_HZ, dt);

    /* Pre-calculate output and limit it if actuator is saturating */
    const float outVal = newProportional + pid->integrator + newDerivative;
    const float outValConstrained = constrainf(outVal, outMin, outMax);

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
    if (posControl.navConfig->general.pos_failure_timeout) {
        if (!posControl.flags.hasValidPositionSensor && ((millis() - posControl.lastValidPositionTimeMs) > (1000 * posControl.navConfig->general.pos_failure_timeout))) {
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

    if (ARMING_FLAG(ARMED)) {
        if (surfaceDistance > 0) {
            if (posControl.actualState.surfaceMin > 0) {
                posControl.actualState.surfaceMin = MIN(posControl.actualState.surfaceMin, surfaceDistance);
            }
            else {
                posControl.actualState.surfaceMin = surfaceDistance;
            }
        }
    }
    else {
        posControl.actualState.surfaceMin = -1;
    }

    posControl.flags.hasValidSurfaceSensor = hasValidSensor;

    if (hasValidSensor) {
        posControl.flags.surfaceDistanceDataNew = 1;
    }
    else {
        posControl.flags.surfaceDistanceDataNew = 0;
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

bool isWaypointReached(const navWaypointPosition_t * waypoint)
{
    // We consider waypoint reached if within specified radius
    const uint32_t wpDistance = calculateDistanceToDestination(&waypoint->pos);
    return (wpDistance <= posControl.navConfig->general.waypoint_radius);
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
            switch (posControl.navConfig->general.flags.rth_alt_control_mode) {
            case NAV_RTH_NO_ALT:
                posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z;
                break;
            case NAV_RTH_EXTRA_ALT: // Maintain current altitude + predefined safety margin
                posControl.homeWaypointAbove.pos.V.Z = posControl.actualState.pos.V.Z + posControl.navConfig->general.rth_altitude;
                break;
            case NAV_RTH_MAX_ALT:
                posControl.homeWaypointAbove.pos.V.Z = MAX(posControl.homeWaypointAbove.pos.V.Z, posControl.actualState.pos.V.Z);
                break;
            case NAV_RTH_AT_LEAST_ALT:  // Climb to at least some predefined altitude above home
                posControl.homeWaypointAbove.pos.V.Z = MAX(posControl.homePosition.pos.V.Z + posControl.navConfig->general.rth_altitude, posControl.actualState.pos.V.Z);
                break;
            case NAV_RTH_CONST_ALT:     // Climb/descend to predefined altitude above home
            default:
                posControl.homeWaypointAbove.pos.V.Z = posControl.homePosition.pos.V.Z + posControl.navConfig->general.rth_altitude;
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
        // If pilot so desires he may reset home position to current position
        if (IS_RC_MODE_ACTIVE(BOXHOMERESET) && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE) && posControl.flags.hasValidPositionSensor) {
            if (STATE(GPS_FIX_HOME)) {
                setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
            }
            else {
                setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
            }
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
    navTargetSurface = constrain(lrintf(posControl.desiredState.surface), -32678, 32767);
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
void updateAltitudeTargetFromClimbRate(float climbRate, navUpdateAltitudeFromRateMode_e mode)
{
    // FIXME: On FIXED_WING and multicopter this should work in a different way
    // Calculate new altitude target

    /* Move surface tracking setpoint if it is set */
    if (mode == CLIMB_RATE_RESET_SURFACE_TARGET) {
        posControl.desiredState.surface = -1;
    }
    else {
        if (posControl.flags.isTerrainFollowEnabled) {
            if (posControl.actualState.surface >= 0.0f && posControl.flags.hasValidSurfaceSensor && (mode == CLIMB_RATE_UPDATE_SURFACE_TARGET)) {
                posControl.desiredState.surface = constrainf(posControl.actualState.surface + (climbRate / (posControl.pids.pos[Z].param.kP * posControl.pids.surface.param.kP)), 1.0f, INAV_SURFACE_MAX_DISTANCE);
            }
        }
        else {
            posControl.desiredState.surface = -1;
        }
    }

    posControl.desiredState.pos.V.Z = posControl.actualState.pos.V.Z + (climbRate / posControl.pids.pos[Z].param.kP);

#if defined(NAV_BLACKBOX)
    navTargetPosition[Z] = constrain(lrintf(posControl.desiredState.pos.V.Z), -32678, 32767);
    navTargetSurface = constrain(lrintf(posControl.desiredState.surface), -32678, 32767);
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
    uint16_t waypointSpeed = posControl.navConfig->general.max_speed;

    if (navGetStateFlags(posControl.navState) & NAV_AUTO_WP) {
        if (posControl.waypointCount > 0 && posControl.waypointList[posControl.activeWaypointIndex].action == NAV_WP_ACTION_WAYPOINT) {
            waypointSpeed = posControl.waypointList[posControl.activeWaypointIndex].p1;

            if (waypointSpeed < 50 || waypointSpeed > posControl.navConfig->general.max_speed) {
                waypointSpeed = posControl.navConfig->general.max_speed;
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
    const timeUs_t currentTimeUs = micros();

#if defined(NAV_BLACKBOX)
    navFlags = 0;
    if (posControl.flags.hasValidAltitudeSensor)    navFlags |= (1 << 0);
    if (posControl.flags.hasValidSurfaceSensor)     navFlags |= (1 << 1);
    if (posControl.flags.hasValidPositionSensor)    navFlags |= (1 << 2);
    if ((STATE(GPS_FIX) && gpsSol.numSat >= posControl.navConfig->estimation.gps_min_sats)) navFlags |= (1 << 3);
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
        if (posControl.flags.forcedRTHActivated || (IS_RC_MODE_ACTIVE(BOXNAVRTH) && canActivatePosHold && STATE(GPS_FIX_HOME))) {
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
bool naivationRequiresAngleMode(void)
{
    const navigationFSMStateFlags_t currentState = navGetStateFlags(posControl.navState);
    return (currentState & NAV_REQUIRE_ANGLE) || ((currentState & NAV_REQUIRE_ANGLE_FW) && STATE(FIXED_WING));
}

/**
 * An indicator that NAV is in charge of heading control (a signal to disable other heading controllers)
 */
int8_t naivationGetHeadingControlState(void)
{
    // No explicit MAG_HOLD mode for airplanes
    if ((navGetStateFlags(posControl.navState) & NAV_REQUIRE_MAGHOLD) && !STATE(FIXED_WING)) {
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
    const bool navBoxModesEnabled = IS_RC_MODE_ACTIVE(BOXNAVRTH) || IS_RC_MODE_ACTIVE(BOXNAVWP) || IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD);
    const bool navLaunchComboModesEnabled = IS_RC_MODE_ACTIVE(BOXNAVLAUNCH) && (IS_RC_MODE_ACTIVE(BOXNAVRTH) || IS_RC_MODE_ACTIVE(BOXNAVWP));
    bool shouldBlockArming = false;

    if (!posControl.navConfig->general.flags.extra_arming_safety)
        return false;

    // Apply extra arming safety only if pilot has any of GPS modes configured
    if ((isUsingNavigationModes() || failsafeMayRequireNavigationMode()) && !(posControl.flags.hasValidPositionSensor && STATE(GPS_FIX_HOME))) {
        shouldBlockArming = true;
    }

    // Don't allow arming if any of NAV modes is active
    if (!ARMING_FLAG(ARMED) && navBoxModesEnabled && !navLaunchComboModesEnabled) {
        shouldBlockArming = true;
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

void navigationUseFlight3DConfig(flight3DConfig_t * initialFlight3DConfig)
{
    posControl.flight3DConfig = initialFlight3DConfig;
}

void navigationUseRxConfig(rxConfig_t * initialRxConfig)
{
    posControl.rxConfig = initialRxConfig;
}

void navigationUsemotorConfig(motorConfig_t * initialmotorConfig)
{
    posControl.motorConfig = initialmotorConfig;
}

void navigationUsePIDs(pidProfile_t *initialPidProfile)
{
    posControl.pidProfile = initialPidProfile;

    // Brake time parameter
    posControl.posDecelerationTime = (float)posControl.pidProfile->I8[PIDPOS] / 100.0f;

    // Position controller expo (taret vel expo for MC)
    posControl.posResponseExpo = constrainf((float)posControl.pidProfile->D8[PIDPOS] / 100.0f, 0.0f, 1.0f);

    // Initialize position hold P-controller
    for (int axis = 0; axis < 2; axis++) {
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

    // Initialize surface tracking PID
    navPidInit(&posControl.pids.surface, 2.0f,
                                         1.0f,
                                         0.0f);

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
                    flight3DConfig_t * initialFlight3DConfig,
                    motorConfig_t * initialmotorConfig)
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

    /* Use system config */
    navigationUseConfig(initialnavConfig);
    navigationUsePIDs(initialPidProfile);
    navigationUseRcControlsConfig(initialRcControlsConfig);
    navigationUseRxConfig(initialRxConfig);
    navigationUsemotorConfig(initialmotorConfig);
    navigationUseFlight3DConfig(initialFlight3DConfig);
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
    const float dLat = destinationLat2 - currentLat1; // difference of latitude in 1/10 000 000 degrees
    const float dLon = (float)(destinationLon2 - currentLon1) * GPS_scaleLonDown;

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
