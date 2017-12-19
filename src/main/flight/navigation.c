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

#ifdef USE_NAV

#include "build/debug.h"

#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/time.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"


PG_REGISTER_WITH_RESET_TEMPLATE(navigationConfig_t, navigationConfig, PG_NAVIGATION_CONFIG, 0);

PG_RESET_TEMPLATE(navigationConfig_t, navigationConfig,
    .gps_wp_radius = 200,
    .gps_lpf = 20,
    .nav_slew_rate = 30,
    .nav_controls_heading = 1,
    .nav_speed_min = 100,
    .nav_speed_max = 300,
    .ap_mode = 40
);

bool areSticksInApModePosition(uint16_t ap_mode);

// **********************
// GPS
// **********************
int32_t GPS_hold[2];


static int16_t nav[2];
static int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother

// When using PWM input GPS usage reduces number of available channels by 2 - see pwm_common.c/pwmInit()
void navigationInit(void)
{
    gpsUsePIDs(currentPidProfile);
}



/*-----------------------------------------------------------
 *
 * Multiwii GPS code - revision: 1097
 *
 *-----------------------------------------------------------*/
#define POSHOLD_IMAX           20       // degrees
#define POSHOLD_RATE_IMAX      20       // degrees
#define NAV_IMAX               20       // degrees

/* GPS navigation can control the heading */
#define NAV_TAIL_FIRST             0    // true - copter comes in with tail first
#define NAV_SET_TAKEOFF_HEADING    1    // true - when copter arrives to home position it rotates it's head to takeoff direction

#define GPS_LOW_SPEED_D_FILTER     1    // below .5m/s speed ignore D term for POSHOLD_RATE, theoretically this also removed D term induced noise

//static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing);
static void GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng);

static bool check_missed_wp(void);
static void GPS_calc_poshold(void);
static void GPS_calc_nav_rate(uint16_t max_speed);
static void GPS_update_crosstrack(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
static int32_t wrap_36000(int32_t angle);

static int32_t wrap_18000(int32_t error);

typedef struct {
    int16_t last_velocity;
} LeadFilter_PARAM;

typedef struct {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;

typedef struct {
    float integrator;          // integrator value
    int32_t last_input;        // last input for derivative
    float last_derivative;     // last derivative for low-pass filter
    float output;
    float derivative;
} PID;

static PID posholdPID[2];
static PID poshold_ratePID[2];

static PID_PARAM navPID_PARAM;
static PID navPID[2];

static int32_t get_P(int32_t error, PID_PARAM *pid)
{
    return (float)error * pid->kP;
}

static int32_t get_I(int32_t error, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static int32_t get_D(int32_t input, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->derivative = (input - pid->last_input) / *dt;

    // Low pass filter cut frequency for derivative calculation
    // Set to  "1 / ( 2 * PI * gps_lpf )
    float pidFilter = (1.0f / (2.0f * M_PIf * (float)navigationConfig()->gps_lpf));
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->last_derivative + (*dt / (pidFilter + *dt)) * (pid->derivative - pid->last_derivative);
    // update state
    pid->last_input = input;
    pid->last_derivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid)
{
    pid->integrator = 0;
    pid->last_input = 0;
    pid->last_derivative = 0;
}


/****************** PI and PID controllers for GPS ********************///32938 -> 33160

#define RADX100                    0.000174532925f
#define CROSSTRACK_GAIN            1
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX               3000 // 30deg max banking when navigating (just for security and testing)

static int32_t error[2];

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
// The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;

// Currently used WP
static int32_t GPS_WP[2];

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t target_bearing;
////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t original_target_bearing;
////////////////////////////////////////////////////////////////////////////////
// The location of the copter in relation to home, updated every GPS read (1deg - 100)
//static int32_t home_to_copter_bearing;
// distance between plane and home in cm
//static int32_t home_distance;
// distance between plane and next_WP in cm
static uint32_t wp_distance;

// used for slow speed wind up when start navigation;
static int16_t waypoint_speed_gov;

// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home


void navNewGpsData(void)
{
    if (FLIGHT_MODE(GPS_HOLD_MODE) || FLIGHT_MODE(GPS_HOME_MODE)) {
        // we are navigating

        // gps nav calculations, these are common for nav and poshold
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
        GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &gpsSol.llh.lat, &gpsSol.llh.lon);

        uint16_t speed;
        switch (nav_mode) {
        case NAV_MODE_POSHOLD:
            // Desired output is in nav_lat and nav_lon where 1deg inclination is 100
            GPS_calc_poshold();
            break;

        case NAV_MODE_WP:
            speed = GPS_calc_desired_speed(navigationConfig()->nav_speed_max, NAV_SLOW_NAV);    // slow navigation
            // use error as the desired rate towards the target
            // Desired output is in nav_lat and nav_lon where 1deg inclination is 100
            GPS_calc_nav_rate(speed);

            // Tail control
            if (navigationConfig()->nav_controls_heading) {
                if (NAV_TAIL_FIRST) {
                    magHold = wrap_18000(nav_bearing - 18000) / 100;
                } else {
                    magHold = nav_bearing / 100;
                }
            }
            // Are we there yet ?(within x meters of the destination)
            if ((wp_distance <= navigationConfig()->gps_wp_radius) || check_missed_wp()) {      // if yes switch to poshold mode
                nav_mode = NAV_MODE_POSHOLD;
                if (NAV_SET_TAKEOFF_HEADING) {
                    magHold = nav_takeoff_bearing;
                }
            }
            break;
        default:
            break;
        }
    }                   //end of gps calcs
}

// reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void)
{
    int i;

    for (i = 0; i < 2; i++) {
        GPS_angle[i] = 0;
        nav_rated[i] = 0;
        nav[i] = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
}

// Get the relevant P I D values and set the PID controllers
void gpsUsePIDs(pidProfile_t *pidProfile)
{
    posholdPID_PARAM.kP = (float)pidProfile->pid[PID_POS].P / 100.0f;
    posholdPID_PARAM.kI = (float)pidProfile->pid[PID_POS].I / 100.0f;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float)pidProfile->pid[PID_POSR].P / 10.0f;
    poshold_ratePID_PARAM.kI = (float)pidProfile->pid[PID_POSR].I / 100.0f;
    poshold_ratePID_PARAM.kD = (float)pidProfile->pid[PID_POSR].D / 1000.0f;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float)pidProfile->pid[PID_NAVR].P / 10.0f;
    navPID_PARAM.kI = (float)pidProfile->pid[PID_NAVR].I / 100.0f;
    navPID_PARAM.kD = (float)pidProfile->pid[PID_NAVR].D / 1000.0f;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
}

// OK here is the onboard GPS code

////////////////////////////////////////////////////////////////////////////////////
// PID based GPS navigation functions
// Author : EOSBandi
// Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
// Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t *lat, int32_t *lon)
{
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    GPS_calc_longitude_scaling(*lat);
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);

    nav_bearing = target_bearing;
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &gpsSol.llh.lat, &gpsSol.llh.lon);
    original_target_bearing = target_bearing;
    waypoint_speed_gov = navigationConfig()->nav_speed_min;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (ABS(temp) > 10000); // we passed the waypoint by 100 degrees
}


////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree
//
//static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
//  uint32_t d1;
//  int32_t  d2;
//  GPS_distance_cm_bearing(&lat1,&lon1,&lat2,&lon2,&d1,&d2);
//  *dist = d1 / 100;          //convert to meters
//  *bearing = d2 /  100;      //convert to degrees
//}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//      100     = 1m
//      1000    = 11m    = 36 feet
//      1800    = 19.80m = 60 feet
//      3000    = 33m
//      10000   = 111m
//
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;   // X Error
    error[LAT] = *target_lat - *gps_lat;        // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold(void)
{
    int32_t d;
    int32_t target_speed;
    int axis;

    for (axis = 0; axis < 2; axis++) {
        target_speed = get_P(error[axis], &posholdPID_PARAM);       // calculate desired speed from lon error
        rate_error[axis] = target_speed - actual_speed[axis];       // calc the speed error

        nav[axis] = get_P(rate_error[axis], &poshold_ratePID_PARAM) +
                    get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = get_D(error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = constrain(d, -2000, 2000);

        // get rid of noise
#if defined(GPS_LOW_SPEED_D_FILTER)
        if (ABS(actual_speed[axis]) < 50)
            d = 0;
#endif

        nav[axis] += d;
        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(uint16_t max_speed)
{
    float trig[2];
    float temp;
    int axis;

    // push us towards the original track
    GPS_update_crosstrack();

    // nav_bearing includes crosstrack
    temp = (9000l - nav_bearing) * RADX100;
    trig[GPS_X] = cos_approx(temp);
    trig[GPS_Y] = sin_approx(temp);

    for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        // P + I + D
        nav[axis] = get_P(rate_error[axis], &navPID_PARAM) +
                    get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) +
                    get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void)
{
    if (ABS(wrap_18000(target_bearing - original_target_bearing)) < 4500) {     // If we are too far off or too close we don't do track following
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sin_approx(temp) * (wp_distance * CROSSTRACK_GAIN); // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    } else {
        nav_bearing = target_bearing;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow)
{
    // max_speed is default 400 or 4m/s
    if (_slow) {
        max_speed = MIN(max_speed, wp_distance / 2);
    } else {
        max_speed = MIN(max_speed, wp_distance);
        max_speed = MAX(max_speed, navigationConfig()->nav_speed_min);      // go at least 100cm/s
    }

    // limit the ramp up of the speed
    // waypoint_speed_gov is reset to 0 at each new WP command
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (int)(100.0f * dTnav);    // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
static int32_t wrap_18000(int32_t error)
{
    if (error > 18000)
        error -= 36000;
    if (error < -18000)
        error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle)
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

void updateGpsStateForHomeAndHoldMode(void)
{
    float sin_yaw_y = sin_approx(DECIDEGREES_TO_DEGREES(attitude.values.yaw) * 0.0174532925f);
    float cos_yaw_x = cos_approx(DECIDEGREES_TO_DEGREES(attitude.values.yaw) * 0.0174532925f);
    if (navigationConfig()->nav_slew_rate) {
        nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -navigationConfig()->nav_slew_rate, navigationConfig()->nav_slew_rate); // TODO check this on uint8
        nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -navigationConfig()->nav_slew_rate, navigationConfig()->nav_slew_rate);
        GPS_angle[AI_ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
        GPS_angle[AI_PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
    } else {
        GPS_angle[AI_ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
        GPS_angle[AI_PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
    }
}

void updateGpsWaypointsAndMode(void)
{
    bool resetNavNow = false;
    static bool gpsReadyBeepDone = false;

    if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {

        //
        // process HOME mode
        //
        // HOME mode takes priority over HOLD mode.

        if (IS_RC_MODE_ACTIVE(BOXGPSHOME)) {
            if (!FLIGHT_MODE(GPS_HOME_MODE)) {

                // Transition to HOME mode
                ENABLE_FLIGHT_MODE(GPS_HOME_MODE);
                DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
                GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                nav_mode = NAV_MODE_WP;
                resetNavNow = true;
            }
        } else {
            if (FLIGHT_MODE(GPS_HOME_MODE)) {

                // Transition from HOME mode
                DISABLE_FLIGHT_MODE(GPS_HOME_MODE);
                nav_mode = NAV_MODE_NONE;
                resetNavNow = true;
            }

            //
            // process HOLD mode
            //

            if (IS_RC_MODE_ACTIVE(BOXGPSHOLD) && areSticksInApModePosition(navigationConfig()->ap_mode)) {
                if (!FLIGHT_MODE(GPS_HOLD_MODE)) {

                    // Transition to HOLD mode
                    ENABLE_FLIGHT_MODE(GPS_HOLD_MODE);
                    GPS_hold[LAT] = gpsSol.llh.lat;
                    GPS_hold[LON] = gpsSol.llh.lon;
                    GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                    nav_mode = NAV_MODE_POSHOLD;
                    resetNavNow = true;
                }
            } else {
                if (FLIGHT_MODE(GPS_HOLD_MODE)) {

                    // Transition from HOLD mode
                    DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
                    nav_mode = NAV_MODE_NONE;
                    resetNavNow = true;
                }
            }
        }
        if (!gpsReadyBeepDone) {            //if 'ready' beep not yet done
            beeper(BEEPER_READY_BEEP);      //do ready beep now
            gpsReadyBeepDone = true;        //only beep once
        }
    } else {
        if (FLIGHT_MODE(GPS_HOLD_MODE | GPS_HOME_MODE)) {

            // Transition from HOME or HOLD mode
            DISABLE_FLIGHT_MODE(GPS_HOME_MODE);
            DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
            nav_mode = NAV_MODE_NONE;
            resetNavNow = true;
        }
    }

    if (resetNavNow) {
        GPS_reset_nav();
    }
}

#endif
