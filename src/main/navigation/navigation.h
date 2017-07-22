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

#include "common/maths.h"

#include "flight/failsafe.h"

#include "io/gps.h"

/* GPS Home location data */
extern gpsLocation_t        GPS_home;
extern uint16_t             GPS_distanceToHome;        // distance to home point in meters
extern int16_t              GPS_directionToHome;       // direction to home point in degrees

/* Navigation system updates */
void onNewGPSData(void);

#if defined(NAV)
#if defined(BLACKBOX)
#define NAV_BLACKBOX
#endif

#ifndef NAV_MAX_WAYPOINTS
#define NAV_MAX_WAYPOINTS 15
#endif

enum {
    NAV_GPS_ATTI    = 0,                    // Pitch/roll stick controls attitude (pitch/roll lean angles)
    NAV_GPS_CRUISE  = 1                     // Pitch/roll stick controls velocity (forward/right speed)
};

enum {
    NAV_RTH_NO_ALT          = 0,            // Maintain current altitude
    NAV_RTH_EXTRA_ALT       = 1,            // Maintain current altitude + predefined safety margin
    NAV_RTH_CONST_ALT       = 2,            // Climb/descend to predefined altitude
    NAV_RTH_MAX_ALT         = 3,            // Track maximum altitude and climb to it when RTH
    NAV_RTH_AT_LEAST_ALT    = 4,            // Climb to predefined altitude if below it
};

enum {
    NAV_HEADING_CONTROL_NONE = 0,
    NAV_HEADING_CONTROL_AUTO,
    NAV_HEADING_CONTROL_MANUAL
};

enum {
    NAV_RESET_ALTITUDE_NEVER = 0,
    NAV_RESET_ALTITUDE_ON_FIRST_ARM,
    NAV_RESET_ALTITUDE_ON_EACH_ARM,
};

typedef struct positionEstimationConfig_s {
    uint8_t automatic_mag_declination;
    uint8_t reset_altitude_type;
    uint8_t gravity_calibration_tolerance;    // Tolerance of gravity calibration (cm/s/s)
    uint8_t use_gps_velned;
    uint16_t gps_delay_ms;

    uint16_t max_surface_altitude;

    float w_z_baro_p;   // Weight (cutoff frequency) for barometer altitude measurements

    float w_z_surface_p;  // Weight (cutoff frequency) for surface altitude measurements
    float w_z_surface_v;  // Weight (cutoff frequency) for surface velocity measurements

    float w_z_gps_p;    // GPS altitude data is very noisy and should be used only on airplanes
    float w_z_gps_v;    // Weight (cutoff frequency) for GPS climb rate measurements

    float w_xy_gps_p;   // Weight (cutoff frequency) for GPS position measurements
    float w_xy_gps_v;   // Weight (cutoff frequency) for GPS velocity measurements

    float w_z_res_v;    // When velocity sources lost slowly decrease estimated velocity with this weight
    float w_xy_res_v;

    float w_acc_bias;   // Weight (cutoff frequency) for accelerometer bias estimation. 0 to disable.

    float max_eph_epv;  // Max estimated position error acceptable for estimation (cm)
    float baro_epv;     // Baro position error
} positionEstimationConfig_t;

PG_DECLARE(positionEstimationConfig_t, positionEstimationConfig);

typedef struct navConfig_s {

    struct {
        struct {
            uint8_t use_thr_mid_for_althold;    // Don't remember throttle when althold was initiated, assume that throttle is at Thr Mid = zero climb rate
            uint8_t extra_arming_safety;        // Forcibly apply 100% throttle tilt compensation
            uint8_t user_control_mode;          // NAV_GPS_ATTI or NAV_GPS_CRUISE
            uint8_t rth_alt_control_mode;       // Controls the logic for choosing the RTH altitude
            uint8_t rth_climb_first;            // Controls the logic for initial RTH climbout
            uint8_t rth_tail_first;             // Return to home tail first
            uint8_t disarm_on_landing;          //
            uint8_t rth_allow_landing;          // Enable landing as last stage of RTH
            uint8_t rth_climb_ignore_emerg;     // Option to ignore GPS loss on initial climb stage of RTH
        } flags;

        uint8_t  pos_failure_timeout;           // Time to wait before switching to emergency landing (0 - disable)
        uint16_t waypoint_radius;               // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
        uint16_t waypoint_safe_distance;        // Waypoint mission sanity check distance
        uint16_t max_auto_speed;                // autonomous navigation speed cm/sec
        uint16_t max_auto_climb_rate;           // max vertical speed limitation cm/sec
        uint16_t max_manual_speed;              // manual velocity control max horizontal speed
        uint16_t max_manual_climb_rate;         // manual velocity control max vertical speed
        uint16_t land_descent_rate;             // normal RTH landing descent rate
        uint16_t land_slowdown_minalt;          // Altitude to stop lowering descent rate during RTH descend
        uint16_t land_slowdown_maxalt;          // Altitude to start lowering descent rate during RTH descend
        uint16_t emerg_descent_rate;            // emergency landing descent rate
        uint16_t rth_altitude;                  // altitude to maintain when RTH is active (depends on rth_alt_control_mode) (cm)
        uint16_t min_rth_distance;              // 0 Disables. Minimal distance for RTL in cm, otherwise it will just autoland
        uint16_t rth_abort_threshold;           // Initiate emergency landing if during RTH we get this much [cm] away from home
    } general;

    struct {
        uint8_t  max_bank_angle;             // multicopter max banking angle (deg)
        uint16_t hover_throttle;             // multicopter hover throttle
        uint16_t auto_disarm_delay;          // multicopter safety delay for landing detector
    } mc;

    struct {
        uint8_t  max_bank_angle;             // Fixed wing max banking angle (deg)
        uint8_t  max_climb_angle;            // Fixed wing max banking angle (deg)
        uint8_t  max_dive_angle;             // Fixed wing max banking angle (deg)
        uint16_t cruise_throttle;            // Cruise throttle
        uint16_t min_throttle;               // Minimum allowed throttle in auto mode
        uint16_t max_throttle;               // Maximum allowed throttle in auto mode
        uint8_t  pitch_to_throttle;          // Pitch angle (in deg) to throttle gain (in 1/1000's of throttle) (*10)
        uint16_t loiter_radius;              // Loiter radius when executing PH on a fixed wing
        int8_t land_dive_angle;
        uint16_t launch_velocity_thresh;     // Velocity threshold for swing launch detection
        uint16_t launch_accel_thresh;        // Acceleration threshold for launch detection (cm/s/s)
        uint16_t launch_time_thresh;         // Time threshold for launch detection (ms)
        uint16_t launch_idle_throttle;       // Throttle to keep at launch idle
        uint16_t launch_throttle;            // Launch throttle
        uint16_t launch_motor_timer;         // Time to wait before setting launch_throttle (ms)
        uint16_t launch_motor_spinup_time;   // Time to speed-up motors from idle to launch_throttle (ESC desync prevention)
        uint16_t launch_timeout;             // Launch timeout to disable launch mode and swith to normal flight (ms)
        uint8_t  launch_climb_angle;         // Target climb angle for launch (deg)
        uint8_t  launch_max_angle;           // Max tilt angle (pitch/roll combined) to consider launch successful. Set to 180 to disable completely [deg]
    } fw;
} navConfig_t;

PG_DECLARE(navConfig_t, navConfig);

typedef struct gpsOrigin_s {
    bool    valid;
    float   scale;
    int32_t lat;    // Lattitude * 1e+7
    int32_t lon;    // Longitude * 1e+7
    int32_t alt;    // Altitude in centimeters (meters * 100)
} gpsOrigin_s;

typedef enum {
    NAV_WP_ACTION_WAYPOINT = 0x01,
    NAV_WP_ACTION_RTH      = 0x04
} navWaypointActions_e;

typedef enum {
    NAV_WP_FLAG_LAST = 0xA5
} navWaypointFlags_e;

typedef struct {
    uint8_t action;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t p1, p2, p3;
    uint8_t flag;
} navWaypoint_t;

typedef struct {
    t_fp_vector pos;
    int32_t     yaw;             // deg * 100
} navWaypointPosition_t;

/* MultiWii-compatible params for telemetry */
typedef enum {
    MW_GPS_MODE_NONE = 0,
    MW_GPS_MODE_HOLD,
    MW_GPS_MODE_RTH,
    MW_GPS_MODE_NAV,
    MW_GPS_MODE_EMERG = 15
} navSystemStatus_Mode_e;

typedef enum {
    MW_NAV_STATE_NONE = 0,                // None
    MW_NAV_STATE_RTH_START,               // RTH Start
    MW_NAV_STATE_RTH_ENROUTE,             // RTH Enroute
    MW_NAV_STATE_HOLD_INFINIT,            // PosHold infinit
    MW_NAV_STATE_HOLD_TIMED,              // PosHold timed
    MW_NAV_STATE_WP_ENROUTE,              // WP Enroute
    MW_NAV_STATE_PROCESS_NEXT,            // Process next
    MW_NAV_STATE_DO_JUMP,                 // Jump
    MW_NAV_STATE_LAND_START,              // Start Land
    MW_NAV_STATE_LAND_IN_PROGRESS,        // Land in Progress
    MW_NAV_STATE_LANDED,                  // Landed
    MW_NAV_STATE_LAND_SETTLE,             // Settling before land
    MW_NAV_STATE_LAND_START_DESCENT       // Start descent
} navSystemStatus_State_e;

typedef enum {
    MW_NAV_ERROR_NONE = 0,            //All systems clear
    MW_NAV_ERROR_TOOFAR,              //Next waypoint distance is more than safety distance
    MW_NAV_ERROR_SPOILED_GPS,         //GPS reception is compromised - Nav paused - copter is adrift !
    MW_NAV_ERROR_WP_CRC,              //CRC error reading WP data from EEPROM - Nav stopped
    MW_NAV_ERROR_FINISH,              //End flag detected, navigation finished
    MW_NAV_ERROR_TIMEWAIT,            //Waiting for poshold timer
    MW_NAV_ERROR_INVALID_JUMP,        //Invalid jump target detected, aborting
    MW_NAV_ERROR_INVALID_DATA,        //Invalid mission step action code, aborting, copter is adrift
    MW_NAV_ERROR_WAIT_FOR_RTH_ALT,    //Waiting to reach RTH Altitude
    MW_NAV_ERROR_GPS_FIX_LOST,        //Gps fix lost, aborting mission
    MW_NAV_ERROR_DISARMED,            //NAV engine disabled due disarm
    MW_NAV_ERROR_LANDING              //Landing
} navSystemStatus_Error_e;

typedef enum {
    MW_NAV_FLAG_ADJUSTING_POSITION  = 1 << 0,
    MW_NAV_FLAG_ADJUSTING_ALTITUDE  = 1 << 1,
} navSystemStatus_Flags_e;

typedef struct {
    navSystemStatus_Mode_e  mode;
    navSystemStatus_State_e state;
    navSystemStatus_Error_e error;
    navSystemStatus_Flags_e flags;
    uint8_t                 activeWpNumber;
    navWaypointActions_e    activeWpAction;
} navSystemStatus_t;

void navigationUsePIDs(void);
void navigationInit(void);

/* Position estimator update functions */
void updatePositionEstimator_BaroTopic(timeUs_t currentTimeUs);
void updatePositionEstimator_SurfaceTopic(timeUs_t currentTimeUs, float newSurfaceAlt);

/* Navigation system updates */
void updateWaypointsAndNavigationMode(void);
void updatePositionEstimator(void);
void applyWaypointNavigationAndAltitudeHold(void);

/* Functions to signal navigation requirements to main loop */
bool navigationRequiresAngleMode(void);
bool navigationRequiresThrottleTiltCompensation(void);
bool navigationRequiresTurnAssistance(void);
int8_t navigationGetHeadingControlState(void);
bool navigationBlockArming(void);
bool navigationPositionEstimateIsHealthy(void);
bool navIsCalibrationComplete(void);

/* Access to estimated position and velocity */
float getEstimatedActualVelocity(int axis);
float getEstimatedActualPosition(int axis);
int32_t getTotalTravelDistance(void);

/* Waypoint list access functions */
int getWaypointCount(void);
bool isWaypointListValid(void);
void getWaypoint(uint8_t wpNumber, navWaypoint_t * wpData);
void setWaypoint(uint8_t wpNumber, const navWaypoint_t * wpData);
void resetWaypointList(void);
bool loadNonVolatileWaypointList(void);
bool saveNonVolatileWaypointList(void);

/* Geodetic functions */
typedef enum {
    GEO_ALT_ABSOLUTE,
    GEO_ALT_RELATIVE
} geoAltitudeConversionMode_e;

typedef enum {
    GEO_ORIGIN_SET,
    GEO_ORIGIN_RESET_ALTITUDE
} geoOriginResetMode_e;

void geoSetOrigin(gpsOrigin_s * origin, const gpsLocation_t * llh, geoOriginResetMode_e resetMode);
void geoConvertGeodeticToLocal(gpsOrigin_s * origin, const gpsLocation_t * llh, t_fp_vector * pos, geoAltitudeConversionMode_e altConv);
void geoConvertLocalToGeodetic(const gpsOrigin_s * origin, const t_fp_vector * pos, gpsLocation_t * llh);
float geoCalculateMagDeclination(const gpsLocation_t * llh); // degrees units

/* Failsafe-forced RTH mode */
void activateForcedRTH(void);
void abortForcedRTH(void);
rthState_e getStateOfForcedRTH(void);

/* Compatibility data */
extern navSystemStatus_t    NAV_Status;

extern int16_t navCurrentState;
extern int16_t navActualVelocity[3];
extern int16_t navDesiredVelocity[3];
extern int16_t navTargetPosition[3];
extern int32_t navLatestActualPosition[3];
extern int16_t navTargetSurface;
extern int16_t navActualSurface;
extern uint16_t navFlags;
extern uint16_t navEPH;
extern uint16_t navEPV;
extern int16_t navAccNEU[3];

#else

#define navigationRequiresAngleMode() (0)
#define navigationGetHeadingControlState() (0)
#define navigationRequiresThrottleTiltCompensation() (0)
#define getEstimatedActualVelocity(axis) (0)

#endif
