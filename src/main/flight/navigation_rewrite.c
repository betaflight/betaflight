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

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"

/*-----------------------------------------------------------
 * Backdoor to MW heading controller
 *-----------------------------------------------------------*/
extern int16_t magHold;

#if defined(NAV)

navigationPosControl_t   posControl;

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

static void setDesiredPositionToWaypointAndUpdateInitialBearing(navWaypointPosition_t * waypoint);

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
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_INITIALIZE(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HEAD_HOME(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_LANDING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHING(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHED(navigationFSMState_t previousState);
static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState);
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
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_THRTILT,
        .mapToFlightModes = NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_ALTHOLD_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_ALTHOLD_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_ALTHOLD_IN_PROGRESS,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_REQUIRE_THRTILT | NAV_RC_ALT,
        .mapToFlightModes = NAV_ALTHOLD_MODE,
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
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_POSHOLD_2D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_POSHOLD_2D_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_2D_IN_PROGRESS,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_REQUIRE_ANGLE | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_POSHOLD_MODE,
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
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_POSHOLD_3D_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_POSHOLD_3D_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_POSHOLD_3D_IN_PROGRESS,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_REQUIRE_ANGLE | NAV_REQUIRE_THRTILT | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_ALTHOLD_MODE | NAV_POSHOLD_MODE,
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
        .timeoutMs = 0,
        .stateFlags = NAV_REQUIRE_ANGLE | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SWITCH_TO_RTH_2D]            = NAV_STATE_RTH_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_RTH_3D]            = NAV_STATE_RTH_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** RTH_2D mode ************************************************/
    [NAV_STATE_RTH_2D_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_HEAD_HOME,
        }
    },

    [NAV_STATE_RTH_2D_HEAD_HOME] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_HEAD_HOME,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_2D_HEAD_HOME,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_FINISHING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_2D_FINISHING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_FINISHING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_2D_FINISHED,
        }
    },

    [NAV_STATE_RTH_2D_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_2D_FINISHED,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_2D_FINISHED,    // re-process the state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** RTH_3D mode ************************************************/
    [NAV_STATE_RTH_3D_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,
        }
    },

    [NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,     // allow pos adjustment while climbind to safe alt
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,   // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_HEAD_HOME,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_HEAD_HOME] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_HEAD_HOME,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_HEAD_HOME,           // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,
        .timeoutMs = 1000,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_ALT | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_LANDING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_LANDING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_LANDING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH | NAV_RC_POS | NAV_RC_YAW,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_LANDING,         // re-process state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_FINISHING,
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    [NAV_STATE_RTH_3D_FINISHING] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_FINISHING,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_RTH_3D_FINISHED,
        }
    },

    [NAV_STATE_RTH_3D_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_RTH_3D_FINISHED,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_RTH,
        .mapToFlightModes = NAV_RTH_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_RTH_3D_FINISHED,         // re-process state
            [NAV_FSM_EVENT_SWITCH_TO_IDLE]              = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_ALTHOLD]           = NAV_STATE_ALTHOLD_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D]        = NAV_STATE_POSHOLD_2D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D]        = NAV_STATE_POSHOLD_3D_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT]          = NAV_STATE_WAYPOINT_INITIALIZE,
            [NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING] = NAV_STATE_EMERGENCY_LANDING_INITIALIZE,
        }
    },

    /** WAYPOINT mode ************************************************/
    [NAV_STATE_WAYPOINT_INITIALIZE] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
            [NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED] = NAV_STATE_WAYPOINT_FINISHED,
        }
    },

    [NAV_STATE_WAYPOINT_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_ALT | NAV_CTL_POS | NAV_CTL_YAW | NAV_REQUIRE_ANGLE | NAV_REQUIRE_MAGHOLD | NAV_REQUIRE_THRTILT | NAV_AUTO_WP,
        .mapToFlightModes = NAV_WP_MODE | NAV_ALTHOLD_MODE,
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
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_WAYPOINT_IN_PROGRESS,
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
        .onEvent = {
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
            [NAV_FSM_EVENT_ERROR]                       = NAV_STATE_IDLE,
        }
    },

    [NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS] = {
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,    // re-process the state
            [NAV_FSM_EVENT_SUCCESS]                     = NAV_STATE_EMERGENCY_LANDING_FINISHED,
        }
    },

    [NAV_STATE_EMERGENCY_LANDING_FINISHED] = {
        .onEntry = navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_FINISHED,
        .timeoutMs = 0,
        .stateFlags = NAV_CTL_EMERG | NAV_REQUIRE_ANGLE,
        .mapToFlightModes = 0,
        .onEvent = {
            [NAV_FSM_EVENT_TIMEOUT]                     = NAV_STATE_EMERGENCY_LANDING_FINISHED,
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
        setDesiredSurfaceOffset(posControl.actualState.surface);
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
    if ((navGetStateFlags(previousState) & NAV_CTL_POS) == 0) {
        resetPositionController();
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
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
    if ((navGetStateFlags(previousState) & NAV_CTL_ALT) == 0) {
        resetAltitudeController();
        setupAltitudeController();

        // If low enough and surface offset valid - enter surface tracking
        setDesiredSurfaceOffset(posControl.actualState.surface);
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
    }

    if ((navGetStateFlags(previousState) & NAV_CTL_POS) == 0) {
        resetPositionController();
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);
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
    if (posControl.flags.hasValidPositionSensor && STATE(GPS_FIX_HOME)) {
        // Switch between 2D and 3D RTH depending on altitude sensor availability
        if (posControl.flags.hasValidAltitudeSensor) {
            return NAV_FSM_EVENT_SWITCH_TO_RTH_3D;
        }
        else {
            return NAV_FSM_EVENT_SWITCH_TO_RTH_2D;
        }
    }
    else {
        // No pos sensor available - land
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_INITIALIZE(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    // If close to home - reset home position
    if (posControl.homeDistance < posControl.navConfig->min_rth_distance) {
        setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw);
    }

    resetPositionController();
    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if (isWaypointReached(&posControl.homeWaypointAbove)) {
        // Successfully reached position target
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        // Update XY-position target
        setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_2D_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
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

    // If close to home - reset home position
    if (posControl.homeDistance < posControl.navConfig->min_rth_distance) {
        setHomePosition(&posControl.actualState.pos, posControl.actualState.yaw);
    }

    resetPositionController();
    resetAltitudeController();
    setupAltitudeController();

    setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_HEADING);

    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if ((posControl.actualState.pos.V.Z - posControl.homeWaypointAbove.pos.V.Z) > -50.0f) {
        return NAV_FSM_EVENT_SUCCESS;
    }

    setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_Z);

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HEAD_HOME(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if (isWaypointReached(&posControl.homeWaypointAbove)) {
        // Successfully reached position target
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        // Update XYZ-position target
        setDesiredPosition(&posControl.homeWaypointAbove.pos, 0, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_BEARING);
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    // Update XYZ-position target
    setDesiredPosition(&posControl.homeWaypointAbove.pos, posControl.homeWaypointAbove.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
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
        // Gradually reduce descent speed depending on actual altitude.
        if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 1000.0f)) {
            updateAltitudeTargetFromClimbRate(-200.0f);
        }
        else if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 250.0f)) {
            updateAltitudeTargetFromClimbRate(-100.0f);
        }
        else {
            updateAltitudeTargetFromClimbRate(-50.0f);
        }

        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHING(navigationFSMState_t previousState)
{
    UNUSED(previousState);
    return NAV_FSM_EVENT_SUCCESS;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_RTH_3D_FINISHED(navigationFSMState_t previousState)
{
    // Stay in this state
    UNUSED(previousState);
    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_INITIALIZE(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // Prepare controllers
    resetPositionController();
    resetAltitudeController();
    setupAltitudeController();

    if (posControl.waypointCount == 0) {
        // No waypoints defined, enter PH and end sequence
        setDesiredPosition(&posControl.actualState.pos, posControl.actualState.yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
        return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED;
    }
    else {
        posControl.activeWaypointIndex = 0;
        setDesiredPositionToWaypointAndUpdateInitialBearing(&posControl.waypointList[posControl.activeWaypointIndex]);
        return NAV_FSM_EVENT_SUCCESS;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_IN_PROGRESS(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    if (isWaypointReached(&posControl.waypointList[posControl.activeWaypointIndex]) || isWaypointMissed(&posControl.waypointList[posControl.activeWaypointIndex])) {
        // Waypoint reached
        return NAV_FSM_EVENT_SUCCESS;
    }
    else {
        // Update XY-position target to active waypoint
        setDesiredPosition(&posControl.waypointList[posControl.activeWaypointIndex].pos, posControl.waypointList[posControl.activeWaypointIndex].yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_BEARING);
        return NAV_FSM_EVENT_NONE;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_REACHED(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // Waypoint reached, do something and move on to next waypoint
    posControl.activeWaypointIndex++;

    if (posControl.activeWaypointIndex >= posControl.waypointCount) {
        // This is the last waypoint - finalize
        return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED;
    }
    else {
        setDesiredPositionToWaypointAndUpdateInitialBearing(&posControl.waypointList[posControl.activeWaypointIndex]);
        return NAV_FSM_EVENT_SUCCESS;
    }
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_WAYPOINT_FINISHED(navigationFSMState_t previousState)
{
    UNUSED(previousState);

    // If no position sensor available - land immediately
    if (!posControl.flags.hasValidPositionSensor) {
        return NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING;
    }

    return NAV_FSM_EVENT_NONE;
}

static navigationFSMEvent_t navOnEnteringState_NAV_STATE_EMERGENCY_LANDING_INITIALIZE(navigationFSMState_t previousState)
{
    // TODO:
    UNUSED(previousState);
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

static void navProcessFSMEvents(navigationFSMEvent_t injectedEvent)
{
    uint32_t currentMillis = millis();
    navigationFSMState_t previousState;
    static uint32_t lastStateSwitchTime = 0;

    /* If timeout event defined and timeout reached - switch state */
    if ((navFSM[posControl.navState].timeoutMs > 0) && (navFSM[posControl.navState].onEvent[NAV_FSM_EVENT_TIMEOUT] != NAV_STATE_UNDEFINED) &&
            ((currentMillis - lastStateSwitchTime) >= navFSM[posControl.navState].timeoutMs)) {
		/* Update state */
        previousState = posControl.navState;
		posControl.navState = navFSM[posControl.navState].onEvent[NAV_FSM_EVENT_TIMEOUT];

		/* Call new state's entry function */
		while (navFSM[posControl.navState].onEntry) {
			navigationFSMEvent_t newEvent = navFSM[posControl.navState].onEntry(previousState);

			if ((newEvent != NAV_FSM_EVENT_NONE) && (navFSM[posControl.navState].onEvent[newEvent] != NAV_STATE_UNDEFINED)) {
                previousState = posControl.navState;
				posControl.navState = navFSM[posControl.navState].onEvent[newEvent];
			}
			else {
				break;
			}
		}

		lastStateSwitchTime  = currentMillis;
    }

	/* Inject new event */
	if (injectedEvent != NAV_FSM_EVENT_NONE && navFSM[posControl.navState].onEvent[injectedEvent] != NAV_STATE_UNDEFINED) {
		/* Update state */
        previousState = posControl.navState;
		posControl.navState = navFSM[posControl.navState].onEvent[injectedEvent];

		/* Call new state's entry function */
		while (navFSM[posControl.navState].onEntry) {
			navigationFSMEvent_t newEvent = navFSM[posControl.navState].onEntry(previousState);

			if ((newEvent != NAV_FSM_EVENT_NONE) && (navFSM[posControl.navState].onEvent[newEvent] != NAV_STATE_UNDEFINED)) {
                previousState = posControl.navState;
				posControl.navState = navFSM[posControl.navState].onEvent[newEvent];
			}
			else {
				break;
			}
		}

		lastStateSwitchTime  = currentMillis;
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

    if (posControl.navConfig->dterm_cut_hz)
        newDerivative = pid->param.kD * filterApplyPt1(newDerivative, &pid->dterm_filter_state, posControl.navConfig->dterm_cut_hz, dt);
    else
        newDerivative = pid->param.kD * newDerivative;

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
 * Processes an update to XY-position and velocity
 *-----------------------------------------------------------*/
void updateActualHorizontalPositionAndVelocity(bool hasValidSensor, float newX, float newY, float newVelX, float newVelY)
{
    posControl.actualState.pos.V.X = newX;
    posControl.actualState.pos.V.Y = newY;

    posControl.actualState.vel.V.X = newVelX;
    posControl.actualState.vel.V.Y = newVelY;

    posControl.flags.hasValidPositionSensor = hasValidSensor;

    if (hasValidSensor) {
        posControl.flags.horizontalPositionNewData = 1;
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
void updateActualSurfaceDistance(bool hasValidSensor, float surfaceDistance)
{
    posControl.actualState.surface = surfaceDistance;
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
 * Check if waypoint is/was reached
 *-----------------------------------------------------------*/
bool isWaypointMissed(navWaypointPosition_t * waypoint)
{
    // We only can miss not home waypoint
    if (waypoint->flags.isHomeWaypoint) {
        return false;
    }
    else {
        int32_t bearingError = calculateBearingToDestination(&waypoint->pos) - waypoint->yaw;
        bearingError = wrap_18000(bearingError);

        return ABS(bearingError) > 10000; // TRUE if we passed the waypoint by 100 degrees
    }
}

bool isWaypointReached(navWaypointPosition_t * waypoint)
{
    // We consider waypoint reached if within specified radius
    uint32_t wpDistance = calculateDistanceToDestination(&waypoint->pos);
    return (wpDistance <= posControl.navConfig->waypoint_radius);
}

/*-----------------------------------------------------------
 * Compatibility for home position
 *-----------------------------------------------------------*/
gpsLocation_t GPS_home;
uint16_t      GPS_distanceToHome;        // distance to home point in meters
int16_t       GPS_directionToHome;       // direction to home point in degrees

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
        if (!(navGetStateFlags(posControl.navState) & NAV_MODE_RTH)) {
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
    posControl.homePosition.flags.isHomeWaypoint = true;
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
        else {
            DISABLE_STATE(GPS_FIX_HOME);
        }
    }
    else {
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
}

void setDesiredPositionToFarAwayTarget(int32_t yaw, int32_t distance, navSetWaypointFlags_t useMask)
{
    t_fp_vector farAwayPos;

    farAwayPos.V.X = posControl.actualState.pos.V.X + distance * cos_approx(CENTIDEGREES_TO_RADIANS(yaw));
    farAwayPos.V.Y = posControl.actualState.pos.V.Y + distance * sin_approx(CENTIDEGREES_TO_RADIANS(yaw));
    farAwayPos.V.Z = posControl.actualState.pos.V.Z;

    setDesiredPosition(&farAwayPos, yaw, useMask);
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
    if (STATE(FIXED_WING)) { // FIXED_WING
        return isFixedWingLandingDetected(&landingTimer);
    }
    else {
        return isMulticopterLandingDetected(&landingTimer);
    }
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
        posControl.desiredState.surface = constrainf(posControl.actualState.surface + (climbRate / posControl.pids.pos[Z].param.kP), 10.0f, 200.0f);
    }

    posControl.desiredState.pos.V.Z = posControl.actualState.pos.V.Z + (climbRate / posControl.pids.pos[Z].param.kP);
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

static void applyAltitudeController(uint32_t currentTime)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltitudeController(currentTime);
    }
    else {
        applyMulticopterAltitudeController(currentTime);
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
    magHold = CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw);
}

static void applyHeadingController(void)
{
    magHold = CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw);
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

static void applyPositionController(uint32_t currentTime)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingPositionController(currentTime);
    }
    else {
        applyMulticopterPositionController(currentTime);
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
}


/*-----------------------------------------------------------
 * WP controller
 *-----------------------------------------------------------*/
static void applyEmergencyLandingController(uint32_t currentTime)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingEmergencyLandingController();
    }
    else {
        applyMulticopterEmergencyLandingController(currentTime);
    }
}

/*-----------------------------------------------------------
 * WP controller
 *-----------------------------------------------------------*/
void getWaypoint(uint8_t wpNumber, int32_t * wpLat, int32_t * wpLon, int32_t * wpAlt)
{
    gpsLocation_t wpLLH;

    wpLLH.lat = 0.0f;
    wpLLH.lon = 0.0f;
    wpLLH.alt = 0.0f;

    // WP #0 - special waypoint - HOME
    if (wpNumber == 0) {
        if (STATE(GPS_FIX_HOME)) {
            wpLLH = GPS_home;
        }
    }
    // WP #255 - special waypoint - directly get actualPosition
    else if (wpNumber == 255) {
        geoConvertLocalToGeodetic(&posControl.gpsOrigin, &posControl.actualState.pos, &wpLLH);
    }
    // WP #1 - #15 - common waypoints - pre-programmed mission
    else if ((wpNumber >= 1) && (wpNumber <= NAV_MAX_WAYPOINTS)) {
        if (wpNumber <= posControl.waypointCount) {
            geoConvertLocalToGeodetic(&posControl.gpsOrigin, &posControl.waypointList[wpNumber - 1].pos, &wpLLH);
        }
    }

    *wpLat = wpLLH.lat;
    *wpLon = wpLLH.lon;
    *wpAlt = wpLLH.alt;
}

void setWaypoint(uint8_t wpNumber, int32_t wpLat, int32_t wpLon, int32_t wpAlt)
{
    gpsLocation_t wpLLH;
    navWaypointPosition_t wpPos;

    // Ignore mission updates if position estimator is not ready yet
    if (posControl.flags.hasValidPositionSensor)
        return;

    // Convert to local coordinates
    wpLLH.lat = wpLat;
    wpLLH.lon = wpLon;
    wpLLH.alt = wpAlt;
    geoConvertGeodeticToLocal(&posControl.gpsOrigin, &wpLLH, &wpPos.pos);
    wpPos.yaw = 0;  // FIXME

    // WP #0 - special waypoint - HOME
    if ((wpNumber == 0) && ARMING_FLAG(ARMED)) {
        // Forcibly set home position. Note that this is only valid if already armed, otherwise home will be reset instantly
        setHomePosition(&wpPos.pos, wpPos.yaw);
    }
    // WP #255 - special waypoint - directly set desiredPosition
    // Only valid when armed and in poshold mode
    else if ((wpNumber == 255) && ARMING_FLAG(ARMED) && (posControl.navState == NAV_STATE_POSHOLD_2D_IN_PROGRESS || posControl.navState == NAV_STATE_POSHOLD_3D_IN_PROGRESS)) {
        // If close to actualPos, use heading, if far - use bearing
        uint32_t wpDistance = calculateDistanceToDestination(&wpPos.pos);
        navSetWaypointFlags_t waypointUpdateFlags = NAV_POS_UPDATE_XY;

        // If we received global altitude == 0, use current altitude
        if (wpAlt != 0) {
            waypointUpdateFlags |= NAV_POS_UPDATE_Z;
        }

        if (wpDistance <= posControl.navConfig->waypoint_radius) {
            waypointUpdateFlags |= NAV_POS_UPDATE_HEADING;
        }
        else {
            waypointUpdateFlags |= NAV_POS_UPDATE_BEARING;
        }

        setDesiredPosition(&wpPos.pos, posControl.actualState.yaw, waypointUpdateFlags);
    }
    // WP #1 - #15 - common waypoints - pre-programmed mission
    else if ((wpNumber >= 1) && (wpNumber <= NAV_MAX_WAYPOINTS)) {
        uint8_t wpIndex = wpNumber - 1;
        /* Sanity check - can set waypoints only sequentially - one by one */
        if (wpIndex <= posControl.waypointCount) {
            wpPos.flags.isHomeWaypoint = false;
            posControl.waypointList[wpIndex] = wpPos;
            posControl.waypointCount = wpIndex + 1;
        }
    }
}

static void setDesiredPositionToWaypointAndUpdateInitialBearing(navWaypointPosition_t * waypoint)
{
    // Calculate initial bearing towards waypoint and store it in waypoint yaw parameter (this will further be used to detect missed waypoints)
    waypoint->yaw = calculateBearingToDestination(&waypoint->pos);

    // Set desired position to next waypoint (XYZ-controller)
    setDesiredPosition(&waypoint->pos, waypoint->yaw, NAV_POS_UPDATE_XY | NAV_POS_UPDATE_Z | NAV_POS_UPDATE_HEADING);
}

/**
 * Returns TRUE if we are in WP mode and executing last waypoint on the list, or in RTH mode, or in PH mode
 *  In RTH mode our only and last waypoint is home
 *  In PH mode our waypoint is hold position */
bool isApproachingLastWaypoint(void)
{
    if (navGetStateFlags(posControl.navState) & NAV_MODE_WP) {
        if (posControl.waypointCount == 0) {
            /* No waypoints, holding current position */
            return true;
        }
        else if (posControl.activeWaypointIndex == (posControl.waypointCount - 1)) {
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
    /* Process pilot's RC input */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if (navStateFlags & NAV_RC_ALT) {
        posControl.flags.isAdjustingAltitude = adjustAltitudeFromRCInput();
    }
    else {
        posControl.flags.isAdjustingAltitude = false;
    }

    if (navStateFlags & NAV_RC_POS) {
        posControl.flags.isAdjustingPosition = adjustPositionFromRCInput();
    }
    else {
        posControl.flags.isAdjustingPosition = false;
    }

    if (navStateFlags & NAV_RC_YAW) {
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
    if (posControl.flags.verticalPositionNewData)   navFlags |= (1 << 0);
    if (posControl.flags.horizontalPositionNewData) navFlags |= (1 << 1);
    if (posControl.flags.headingNewData)            navFlags |= (1 << 2);
    if (posControl.flags.hasValidAltitudeSensor)    navFlags |= (1 << 3);
    if (posControl.flags.hasValidSurfaceSensor)     navFlags |= (1 << 4);
    if (posControl.flags.hasValidPositionSensor)    navFlags |= (1 << 5);
#endif

    // No navigation when disarmed
    if (!ARMING_FLAG(ARMED)) {
        return;
    }

    /* Process controllers */
    navigationFSMStateFlags_t navStateFlags = navGetStateFlags(posControl.navState);
    if (navStateFlags & NAV_CTL_EMERG) {
        applyEmergencyLandingController(currentTime);
    }
    else {
        if (navStateFlags & NAV_CTL_ALT)
            applyAltitudeController(currentTime);

        if (navStateFlags & NAV_CTL_POS)
            applyPositionController(currentTime);

        if (navStateFlags & NAV_CTL_YAW)
            applyHeadingController();
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
    // FIXME: This should be made independent of airplane/multicopter
    if (STATE(FIXED_WING)) {
        return posControl.flags.hasValidAltitudeSensor;
    }
    else {
        return posControl.flags.hasValidAltitudeSensor;
    }
}

static bool canActivatePosHoldMode(void)
{
    // FIXME: This should be made independent of airplane/multicopter
    if (STATE(FIXED_WING)) {
        return false;
    }
    else {
        return posControl.flags.hasValidPositionSensor;
    }
}

static navigationFSMEvent_t selectNavEventFromBoxModeInput(void)
{
    // Flags if we can activate certain nav modes (check if we have required sensors and they provide valid data)
    bool canActivateAltHold = canActivateAltHoldMode();
    bool canActivatePosHold = canActivatePosHoldMode();

    if (posControl.flags.forcedRTHActivated) {
        // If we request forced RTH - attempt to activate it no matter what
        // This might switch to emergency landing controller if GPS is unavailable
        return NAV_FSM_EVENT_SWITCH_TO_RTH;
    }
    else if (IS_RC_MODE_ACTIVE(BOXNAVRTH)) {
        if (canActivatePosHold && STATE(GPS_FIX_HOME))
            return NAV_FSM_EVENT_SWITCH_TO_RTH;
    }

    if (IS_RC_MODE_ACTIVE(BOXNAVWP)) {
        return NAV_FSM_EVENT_SWITCH_TO_WAYPOINT;
    }

    if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD) && IS_RC_MODE_ACTIVE(BOXNAVALTHOLD)) {
        if (canActivatePosHold && canActivateAltHold)
            return NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D;
    }

    if (IS_RC_MODE_ACTIVE(BOXNAVPOSHOLD)) {
        if (canActivatePosHold)
            return NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D;
    }

    if (IS_RC_MODE_ACTIVE(BOXNAVALTHOLD) && canActivateAltHold) {
        return NAV_FSM_EVENT_SWITCH_TO_ALTHOLD;
    }

    return NAV_FSM_EVENT_SWITCH_TO_IDLE;
}

/*-----------------------------------------------------------
 * An indicator that throttle tilt compensation is forced
 *-----------------------------------------------------------*/
bool navigationRequiresThrottleTiltCompensation(void)
{
    return !STATE(FIXED_WING) && posControl.navConfig->flags.throttle_tilt_comp && (navGetStateFlags(posControl.navState) & NAV_REQUIRE_THRTILT);
}

/*-----------------------------------------------------------
 * An indicator that ANGLE mode must be forced per NAV requirement
 *-----------------------------------------------------------*/
bool naivationRequiresAngleMode(void)
{
    return navGetStateFlags(posControl.navState) & NAV_REQUIRE_ANGLE;
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

/**
 * Process NAV mode transition and WP/RTH state machine
 *  Update rate: RX (data driven or 50Hz)
 */
void updateWaypointsAndNavigationMode(bool isRXDataNew)
{
    // Process this on each update.
    if (isRXDataNew) {
        /* Initiate home position update */
        updateHomePosition();

        // Process switch to a different navigation mode (if needed)
        navProcessFSMEvents(selectNavEventFromBoxModeInput());

        // Process pilot's RC input to adjust behaviour
        processNavigationRCAdjustments();

        // Map navMode back to enabled flight modes
        swithNavigationFlightModes();
    }

    debug[0] = posControl.navState;
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

    // Initialize position hold PI-controller
    for (axis = 0; axis < 2; axis++) {
        navPInit(&posControl.pids.pos[axis], (float)posControl.pidProfile->P8[PIDPOS] / 100.0f);

        navPidInit(&posControl.pids.vel[axis], (float)posControl.pidProfile->P8[PIDPOSR] / 100.0f,
                                               (float)posControl.pidProfile->I8[PIDPOSR] / 100.0f,
                                               (float)posControl.pidProfile->D8[PIDPOSR] / 1000.0f);
    }

    // Initialize altitude hold PID-controllers (pos_z, vel_z, acc_z
    navPInit(&posControl.pids.pos[Z], (float)posControl.pidProfile->P8[PIDALT] / 100.0f);

    navPidInit(&posControl.pids.vel[Z], (float)posControl.pidProfile->P8[PIDVEL] / 100.0f,
                                        (float)posControl.pidProfile->I8[PIDVEL] / 100.0f,
                                        (float)posControl.pidProfile->D8[PIDVEL] / 1000.0f);

    // Initialize fixed wing PID controllers
    navPidInit(&posControl.pids.fw_nav, (float)posControl.pidProfile->P8[PIDNAVR] / 100.0f,
                                        (float)posControl.pidProfile->I8[PIDNAVR] / 100.0f,
                                        (float)posControl.pidProfile->D8[PIDNAVR] / 1000.0f);

    navPidInit(&posControl.pids.fw_alt, (float)posControl.pidProfile->P8[PIDALT] / 100.0f,
                                        (float)posControl.pidProfile->I8[PIDALT] / 100.0f,
                                        (float)posControl.pidProfile->D8[PIDALT] / 1000.0f);
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

    posControl.flags.forcedRTHActivated = 0;
    posControl.waypointCount = 0;
    posControl.activeWaypointIndex = 0;

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
    if (navGetStateFlags(posControl.navState) & NAV_MODE_RTH) {

        if (posControl.navState == NAV_STATE_RTH_2D_FINISHED || posControl.navState == NAV_STATE_RTH_3D_FINISHED) {
            return RTH_HAS_LANDED;
        }
        else if (posControl.flags.hasValidPositionSensor) {
            return RTH_IN_PROGRESS_OK;
        }
        else {
            return RTH_IN_PROGRESS_LOST_GPS;
        }
    }
    else {
        return RTH_IDLE;
    }
}

#endif  // NAV
