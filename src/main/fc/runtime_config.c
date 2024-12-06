/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "fc/runtime_config.h"
#include "io/beeper.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;

static uint32_t enabledSensors = 0;

// Name must be no longer than OSD_WARNINGS_MAX_SIZE
// try to keep names within OSD_WARNINGS_PREFFERED_SIZE
const char *armingDisableFlagNames[]= {
    "NOGYRO",
    "FAILSAFE",
    "RXLOSS",
    "NOT_DISARMED",
    "BOXFAILSAFE",
    "RUNAWAY",
    "CRASH",
    "THROTTLE",
    "ANGLE",
    "BOOTGRACE",
    "NOPREARM",
    "LOAD",
    "CALIB",
    "CLI",
    "CMS",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESCUE_SW",
    "DSHOT_TELEM",
    "REBOOT_REQD",
    "DSHOT_BBANG",
    "NO_ACC_CAL",
    "MOTOR_PROTO",
    "FLIP_SWITCH",
    "ALT_HOLD SW",
    "POS_HOLD_SW",
    "ARMSWITCH",
};
STATIC_ASSERT(ARRAYLEN(armingDisableFlagNames) == ARMING_DISABLE_FLAGS_COUNT, armingDisableFlagNames_size_mismatch);

static armingDisableFlags_e armingDisableFlags = 0;

void setArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags | flag;
}

void unsetArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags & ~flag;
}

bool isArmingDisabled(void)
{
    return armingDisableFlags;
}

armingDisableFlags_e getArmingDisableFlags(void)
{
    return armingDisableFlags;
}

// return name for given flag
// will return first name (LSB) if multiple bits are passed
const char *getArmingDisableFlagName(armingDisableFlags_e flag)
{
    if (!flag) {
        return "NONE";
    }
    unsigned idx = ffs(flag & -flag) - 1;   // use LSB if there are multiple bits set
    return idx < ARRAYLEN(armingDisableFlagNames) ? armingDisableFlagNames[idx] : "UNKNOWN";
}

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t enableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t disableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}
