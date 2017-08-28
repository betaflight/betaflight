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

// FIXME some of these are flight modes, some of these are general status indicators
typedef enum {
    ARMED                                           = (1 << 2),
    WAS_EVER_ARMED                                  = (1 << 3),

    ARMING_DISABLED_FAILSAFE_SYSTEM                 = (1 << 7),

    ARMING_DISABLED_NOT_LEVEL                       = (1 << 8),
    ARMING_DISABLED_SENSORS_CALIBRATING             = (1 << 9),
    ARMING_DISABLED_SYSTEM_OVERLOADED               = (1 << 10),
    ARMING_DISABLED_NAVIGATION_UNSAFE               = (1 << 11),
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED          = (1 << 12),
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED    = (1 << 13),
    ARMING_DISABLED_ARM_SWITCH                      = (1 << 14),
    ARMING_DISABLED_HARDWARE_FAILURE                = (1 << 15),
    ARMING_DISABLED_BOXFAILSAFE                     = (1 << 16),
    ARMING_DISABLED_BOXKILLSWITCH                   = (1 << 17),
    ARMING_DISABLED_RC_LINK                         = (1 << 18),
    ARMING_DISABLED_THROTTLE                        = (1 << 19),
    ARMING_DISABLED_CLI                             = (1 << 20),
    ARMING_DISABLED_CMS_MENU                        = (1 << 21),
    ARMING_DISABLED_OSD_MENU                        = (1 << 22),

    ARMING_DISABLED_ALL_FLAGS                       = (ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL | ARMING_DISABLED_SENSORS_CALIBRATING | ARMING_DISABLED_SYSTEM_OVERLOADED | 
                                                       ARMING_DISABLED_NAVIGATION_UNSAFE | ARMING_DISABLED_COMPASS_NOT_CALIBRATED | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED | 
                                                       ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE | ARMING_DISABLED_BOXFAILSAFE | ARMING_DISABLED_BOXKILLSWITCH |
                                                       ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE | ARMING_DISABLED_CLI | ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_OSD_MENU)
} armingFlag_e;

extern uint32_t armingFlags;

#define isArmingDisabled()          (armingFlags & (ARMING_DISABLED_ALL_FLAGS))
#define DISABLE_ARMING_FLAG(mask)   (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask)    (armingFlags |= (mask))
#define ARMING_FLAG(mask)           (armingFlags & (mask))

typedef enum {
    ANGLE_MODE      = (1 << 0),
    HORIZON_MODE    = (1 << 1),
    HEADING_MODE    = (1 << 2),
    NAV_ALTHOLD_MODE= (1 << 3), // old BARO
    NAV_RTH_MODE    = (1 << 4), // old GPS_HOME
    NAV_POSHOLD_MODE= (1 << 5), // old GPS_HOLD
    HEADFREE_MODE   = (1 << 6),
    NAV_LAUNCH_MODE = (1 << 7),
    PASSTHRU_MODE   = (1 << 8),
    FAILSAFE_MODE   = (1 << 10),
    AUTO_TUNE       = (1 << 11), // old G-Tune
    NAV_WP_MODE     = (1 << 12),
    UNUSED_MODE2    = (1 << 13),
    FLAPERON        = (1 << 14),
#ifdef USE_FLM_TURN_ASSIST
    TURN_ASSISTANT  = (1 << 15),
#endif
} flightModeFlags_e;

extern uint32_t flightModeFlags;

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))

typedef enum {
    GPS_FIX_HOME            = (1 << 0),
    GPS_FIX                 = (1 << 1),
    CALIBRATE_MAG           = (1 << 2),
    SMALL_ANGLE             = (1 << 3),
    FIXED_WING              = (1 << 4),     // set when in flying_wing or airplane mode. currently used by althold selection code
    ANTI_WINDUP             = (1 << 5),
    FLAPERON_AVAILABLE      = (1 << 6),
    NAV_MOTOR_STOP_OR_IDLE  = (1 << 7),     // navigation requests MOTOR_STOP or motor idle regardless of throttle stick, will only activate if MOTOR_STOP feature is available
    COMPASS_CALIBRATED      = (1 << 8),
    ACCELEROMETER_CALIBRATED= (1 << 9),
    PWM_DRIVER_AVAILABLE    = (1 << 10),
    HELICOPTER              = (1 << 11)
} stateFlags_t;

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define STATE(mask) (stateFlags & (mask))

extern uint32_t stateFlags;

typedef enum {
    FLM_MANUAL,
    FLM_ACRO,
    FLM_ANGLE,
    FLM_HORIZON,
    FLM_ALTITUDE_HOLD,
    FLM_POSITION_HOLD,
    FLM_RTH,
    FLM_MISSION,
    FLM_LAUNCH,
    FLM_FAILSAFE,
    FLM_COUNT
} flightModeForTelemetry_e;

flightModeForTelemetry_e getFlightModeForTelemetry(void);

uint32_t enableFlightMode(flightModeFlags_e mask);
uint32_t disableFlightMode(flightModeFlags_e mask);

bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
