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

#pragma once

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR    1.113195f  // MagicEarthNumber from APM

#if defined(NAV)

#include "config/runtime_config.h"

#define LAND_DETECTOR_TRIGGER_TIME_MS       2000        // 2 seconds
#define MIN_POSITION_UPDATE_RATE_HZ         5       // Minimum position update rate at which XYZ controllers would be applied
#define NAV_THROTTLE_CUTOFF_FREQENCY_HZ     4       // low-pass filter on throttle output
#define NAV_ACCEL_CUTOFF_FREQUENCY_HZ       2       // low-pass filter on XY-acceleration target
#define NAV_FW_VEL_CUTOFF_FREQENCY_HZ       2       // low-pass filter on Z-velocity for fixed wing
#define NAV_DTERM_CUT_HZ                    10
#define NAV_ACCELERATION_XY_MAX             980.0f  // cm/s/s       // approx 45 deg lean angle

#define HZ2US(hz)   (1000000 / (hz))
#define US2S(us)    ((us) * 1e-6f)
#define MS2US(ms)   ((ms) * 1000)
#define HZ2S(hz)    US2S(HZ2US(hz))

typedef enum {
    NAV_POS_UPDATE_NONE                 = 0,
    NAV_POS_UPDATE_XY                   = 1 << 0,
    NAV_POS_UPDATE_Z                    = 1 << 1,
    NAV_POS_UPDATE_HEADING              = 1 << 2,
    NAV_POS_UPDATE_BEARING              = 1 << 3,
    NAV_POS_UPDATE_BEARING_TAIL_FIRST   = 1 << 4,
} navSetWaypointFlags_t;

typedef struct navigationFlags_s {
    bool horizontalPositionNewData;
    bool verticalPositionNewData;
    bool surfaceDistanceNewData;
    bool headingNewData;

    bool hasValidAltitudeSensor;        // Indicates that we have a working altitude sensor (got at least one valid reading from it)
    bool hasValidPositionSensor;        // Indicates that GPS is working (or not)
    bool hasValidSurfaceSensor;
    bool hasValidHeadingSensor;         // Indicate valid heading - wither mag or GPS at certain speed on airplane

    bool isAdjustingPosition;
    bool isAdjustingAltitude;
    bool isAdjustingHeading;

    // Behaviour modifiers
    bool isGCSAssistedNavigationEnabled;    // Does iNav accept WP#255 - follow-me etc.

    bool forcedRTHActivated;
} navigationFlags_t;

typedef struct {
    float kP;
    float kI;
    float kD;
    float kT;   // Tracking gain (anti-windup)
} pidControllerParam_t;

typedef struct {
    float kP;
} pControllerParam_t;

typedef struct {
    pidControllerParam_t param;
    filterStatePt1_t dterm_filter_state;  // last derivative for low-pass filter
    float integrator;       // integrator value
    float last_input;       // last input for derivative
} pidController_t;

typedef struct {
    pControllerParam_t param;
} pController_t;

typedef struct navigationPIDControllers_s {
    /* Multicopter PIDs */
    pController_t   pos[XYZ_AXIS_COUNT];
    pidController_t vel[XYZ_AXIS_COUNT];

    /* Fixed-wing PIDs */
    pidController_t fw_alt;
    pidController_t fw_nav;
} navigationPIDControllers_t;

typedef struct {
    t_fp_vector pos;
    t_fp_vector vel;
    int32_t     yaw;
    float       sinYaw;
    float       cosYaw;
    float       surface;
    float       surfaceVel;
    float       surfaceMin;
} navigationEstimatedState_t;

typedef struct {
    t_fp_vector pos;
    t_fp_vector vel;
    int32_t     yaw;
    float       surface;
} navigationDesiredState_t;

typedef enum {
    NAV_FSM_EVENT_NONE = 0,
    NAV_FSM_EVENT_TIMEOUT,

    NAV_FSM_EVENT_SUCCESS,
    NAV_FSM_EVENT_ERROR,

    NAV_FSM_EVENT_STATE_SPECIFIC,               // State-specific event
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED = NAV_FSM_EVENT_STATE_SPECIFIC,

    NAV_FSM_EVENT_SWITCH_TO_IDLE,
    NAV_FSM_EVENT_SWITCH_TO_ALTHOLD,
    NAV_FSM_EVENT_SWITCH_TO_POSHOLD_2D,
    NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D,
    NAV_FSM_EVENT_SWITCH_TO_RTH,
    NAV_FSM_EVENT_SWITCH_TO_RTH_2D,
    NAV_FSM_EVENT_SWITCH_TO_RTH_3D,
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT,
    NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING,

    NAV_FSM_EVENT_COUNT,
} navigationFSMEvent_t;

typedef enum {
    NAV_STATE_UNDEFINED = 0,                    // 0

    NAV_STATE_IDLE,                             // 1

    NAV_STATE_ALTHOLD_INITIALIZE,               // 2
    NAV_STATE_ALTHOLD_IN_PROGRESS,              // 3

    NAV_STATE_POSHOLD_2D_INITIALIZE,            // 4
    NAV_STATE_POSHOLD_2D_IN_PROGRESS,           // 5

    NAV_STATE_POSHOLD_3D_INITIALIZE,            // 6
    NAV_STATE_POSHOLD_3D_IN_PROGRESS,           // 7

    NAV_STATE_RTH_INITIALIZE,                   // 8

    NAV_STATE_RTH_2D_INITIALIZE,                // 9
    NAV_STATE_RTH_2D_HEAD_HOME,                 // 10
    NAV_STATE_RTH_2D_GPS_FAILING,               // 11
    NAV_STATE_RTH_2D_FINISHING,                 // 12
    NAV_STATE_RTH_2D_FINISHED,                  // 13

    NAV_STATE_RTH_3D_INITIALIZE,                // 14
    NAV_STATE_RTH_3D_CLIMB_TO_SAFE_ALT,         // 15
    NAV_STATE_RTH_3D_HEAD_HOME,                 // 16
    NAV_STATE_RTH_3D_GPS_FAILING,               // 17
    NAV_STATE_RTH_3D_HOVER_PRIOR_TO_LANDING,    // 18
    NAV_STATE_RTH_3D_LANDING,                   // 19
    NAV_STATE_RTH_3D_FINISHING,                 // 20
    NAV_STATE_RTH_3D_FINISHED,                  // 21

    NAV_STATE_WAYPOINT_INITIALIZE,              // 22
    NAV_STATE_WAYPOINT_PRE_ACTION,              // 23
    NAV_STATE_WAYPOINT_IN_PROGRESS,             // 24
    NAV_STATE_WAYPOINT_REACHED,                 // 25
    NAV_STATE_WAYPOINT_FINISHED,                // 26

    NAV_STATE_EMERGENCY_LANDING_INITIALIZE,     // 27
    NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS,    // 28
    NAV_STATE_EMERGENCY_LANDING_FINISHED,       // 29

    NAV_STATE_COUNT,
} navigationFSMState_t;

typedef enum {
    /* Navigation controllers */
    NAV_CTL_ALT             = (1 << 0),     // Altitude controller
    NAV_CTL_POS             = (1 << 1),     // Position controller
    NAV_CTL_YAW             = (1 << 2),
    NAV_CTL_EMERG           = (1 << 3),

    /* Navigation requirements for flight modes and controllers */
    NAV_REQUIRE_ANGLE       = (1 << 4),
    NAV_REQUIRE_ANGLE_FW    = (1 << 5),
    NAV_REQUIRE_MAGHOLD     = (1 << 6),
    NAV_REQUIRE_THRTILT     = (1 << 7),

    /* Navigation autonomous modes */
    NAV_AUTO_RTH            = (1 << 8),
    NAV_AUTO_WP             = (1 << 9),

    /* Adjustments for navigation modes from RC input */
    NAV_RC_ALT              = (1 << 10),
    NAV_RC_POS              = (1 << 12),
    NAV_RC_YAW              = (1 << 13),
} navigationFSMStateFlags_t;

typedef struct {
    navigationFSMEvent_t                (*onEntry)(navigationFSMState_t previousState);
    uint32_t                            timeoutMs;
    navSystemStatus_State_e             mwState;
    navSystemStatus_Error_e             mwError;
    navigationFSMStateFlags_t           stateFlags;
    flightModeFlags_e                   mapToFlightModes;
    navigationFSMState_t                onEvent[NAV_FSM_EVENT_COUNT];
} navigationFSMStateDescriptor_t;

typedef struct {
    /* Flags and navigation system state */
    navigationFSMState_t        navState;

    navigationFlags_t           flags;

    /* Navigation PID controllers + pre-computed flight parameters */
    navigationPIDControllers_t  pids;
    float                       posDecelerationTime;
    float                       posResponseExpo;

    /* Local system state, both actual (estimated) and desired (target setpoint)*/
    navigationEstimatedState_t  actualState;
    navigationDesiredState_t    desiredState;   // waypoint coordinates + velocity

    uint32_t                    lastValidPositionTimeMs;
    uint32_t                    lastValidAltitudeTimeMs;

    /* INAV GPS origin (position where GPS fix was first acquired) */
    gpsOrigin_s                 gpsOrigin;

    /* Home parameters (NEU coordinated), geodetic position of home (LLH) is stores in GPS_home variable */
    navWaypointPosition_t       homePosition;       // Special waypoint, stores original yaw (heading when launched)
    navWaypointPosition_t       homeWaypointAbove;  // NEU-coordinates and initial bearing + desired RTH altitude

    uint32_t                    homeDistance;   // cm
    int32_t                     homeDirection;  // deg*100

    /* Waypoint list */
    navWaypoint_t               waypointList[NAV_MAX_WAYPOINTS];
    bool                        waypointListValid;
    int8_t                      waypointCount;

    navWaypointPosition_t       activeWaypoint;     // Local position and initial bearing, filled on waypoint activation
    int8_t                      activeWaypointIndex;

    /* Internals */
    int16_t                     rcAdjustment[4];

    navConfig_t *               navConfig;
    rcControlsConfig_t *        rcControlsConfig;
    pidProfile_t *              pidProfile;
    rxConfig_t *                rxConfig;
    escAndServoConfig_t *       escAndServoConfig;
} navigationPosControl_t;

extern navigationPosControl_t posControl;

/* Internally used functions */
float navPidApply2(float setpoint, float measurement, float dt, pidController_t *pid, float outMin, float outMax, bool dTermErrorTracking);
void navPidReset(pidController_t *pid);
void navPidInit(pidController_t *pid, float _kP, float _kI, float _kD);
void navPInit(pController_t *p, float _kP);

bool isThrustFacingDownwards(void);
void updateAltitudeTargetFromClimbRate(float climbRate);
uint32_t calculateDistanceToDestination(t_fp_vector * destinationPos);
int32_t calculateBearingToDestination(t_fp_vector * destinationPos);
void resetLandingDetector(void);
bool isLandingDetected(void);

navigationFSMStateFlags_t navGetCurrentStateFlags(void);

void setHomePosition(t_fp_vector * pos, int32_t yaw);
void setDesiredPosition(t_fp_vector * pos, int32_t yaw, navSetWaypointFlags_t useMask);
void setDesiredSurfaceOffset(float surfaceOffset);
void setDesiredPositionToFarAwayTarget(int32_t yaw, int32_t distance, navSetWaypointFlags_t useMask);

bool isWaypointReached(navWaypointPosition_t * waypoint);
bool isWaypointMissed(navWaypointPosition_t * waypoint);
bool isApproachingLastWaypoint(void);
float getActiveWaypointSpeed(void);

int16_t rcCommandToLeanAngle(int16_t rcCommand);
int16_t leanAngleToRcCommand(int16_t leanAngle);

void updateActualHeading(int32_t newHeading);
void updateActualHorizontalPositionAndVelocity(bool hasValidSensor, float newX, float newY, float newVelX, float newVelY);
void updateActualAltitudeAndClimbRate(bool hasValidSensor, float newAltitude, float newVelocity);
void updateActualSurfaceDistance(bool hasValidSensor, float surfaceDistance, float surfaceVelocity);

bool checkForPositionSensorTimeout(void);

bool isGPSGlitchDetected(void);

/* Multicopter-specific functions */
void setupMulticopterAltitudeController(void);

void resetMulticopterAltitudeController(void);
void resetMulticopterPositionController(void);
void resetMulticopterHeadingController(void);

bool adjustMulticopterAltitudeFromRCInput(void);
bool adjustMulticopterHeadingFromRCInput(void);
bool adjustMulticopterPositionFromRCInput(void);

void applyMulticopterNavigationController(navigationFSMStateFlags_t navStateFlags, uint32_t currentTime);

bool isMulticopterLandingDetected(uint32_t * landingTimer);
void calculateMulticopterInitialHoldPosition(t_fp_vector * pos);

/* Fixed-wing specific functions */
void setupFixedWingAltitudeController(void);

void resetFixedWingAltitudeController(void);
void resetFixedWingPositionController(void);
void resetFixedWingHeadingController(void);

bool adjustFixedWingAltitudeFromRCInput(void);
bool adjustFixedWingHeadingFromRCInput(void);
bool adjustFixedWingPositionFromRCInput(void);

void applyFixedWingNavigationController(navigationFSMStateFlags_t navStateFlags, uint32_t currentTime);

bool isFixedWingLandingDetected(uint32_t * landingTimer);
void calculateFixedWingInitialHoldPosition(t_fp_vector * pos);

#endif