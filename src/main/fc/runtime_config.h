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
    ARMED                       = (1 << 0),
    WAS_EVER_ARMED              = (1 << 1),
    WAS_ARMED_WITH_PREARM       = (1 << 2)
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

/*
 * Arming disable flags are listed in the order of criticalness.
 * (Beeper code can notify the most critical reason.)
 */
typedef enum {
    ARMING_DISABLED_NO_GYRO         = (1 << 0),
    ARMING_DISABLED_FAILSAFE        = (1 << 1),
    ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
    ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
    ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
    ARMING_DISABLED_THROTTLE        = (1 << 5),
    ARMING_DISABLED_ANGLE           = (1 << 6),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 7),
    ARMING_DISABLED_NOPREARM        = (1 << 8),
    ARMING_DISABLED_ARM_SWITCH      = (1 << 9),
    ARMING_DISABLED_LOAD            = (1 << 10),
    ARMING_DISABLED_CALIBRATING     = (1 << 11),
    ARMING_DISABLED_CLI             = (1 << 12),
    ARMING_DISABLED_CMS_MENU        = (1 << 13),
    ARMING_DISABLED_OSD_MENU        = (1 << 14),
    ARMING_DISABLED_BST             = (1 << 15),
    ARMING_DISABLED_MSP             = (1 << 16)
} armingDisableFlags_e;

#define NUM_ARMING_DISABLE_FLAGS 17

#if defined(OSD) || !defined(MINIMAL_CLI)
extern const char *armingDisableFlagNames[NUM_ARMING_DISABLE_FLAGS];
#endif

void setArmingDisabled(armingDisableFlags_e flag);
void unsetArmingDisabled(armingDisableFlags_e flag);
bool isArmingDisabled(void);
armingDisableFlags_e getArmingDisableFlags(void);

typedef enum {
    ANGLE_MODE      = (1 << 0),
    HORIZON_MODE    = (1 << 1),
    MAG_MODE        = (1 << 2),
    BARO_MODE       = (1 << 3),
    GPS_HOME_MODE   = (1 << 4),
    GPS_HOLD_MODE   = (1 << 5),
    HEADFREE_MODE   = (1 << 6),
    UNUSED_MODE     = (1 << 7), // old autotune
    PASSTHRU_MODE   = (1 << 8),
    SONAR_MODE      = (1 << 9),
    FAILSAFE_MODE   = (1 << 10)
} flightModeFlags_e;

extern uint16_t flightModeFlags;

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))

// macro to initialize map from flightModeFlags to boxId_e. Keep it in sync with flightModeFlags_e enum.
// Each boxId_e is at index of flightModeFlags_e bit, value is -1 if boxId_e does not exist.
// It is much more memory efficient than full map (uint32_t -> uint8_t)
#define FLIGHT_MODE_BOXID_MAP_INITIALIZER {                             \
        BOXANGLE, BOXHORIZON, BOXMAG, BOXBARO, BOXGPSHOME, BOXGPSHOLD,  \
        BOXHEADFREE, -1, BOXPASSTHRU, BOXSONAR, BOXFAILSAFE}  \
        /**/

typedef enum {
    GPS_FIX_HOME   = (1 << 0),
    GPS_FIX        = (1 << 1),
    CALIBRATE_MAG  = (1 << 2),
    SMALL_ANGLE    = (1 << 3),
    FIXED_WING     = (1 << 4)                    // set when in flying_wing or airplane mode. currently used by althold selection code
} stateFlags_t;

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define STATE(mask) (stateFlags & (mask))

extern uint8_t stateFlags;

uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);

bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);

void mwDisarm(void);
