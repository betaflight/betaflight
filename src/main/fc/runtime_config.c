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

// Must be no longer than OSD_WARNINGS_MAX_SIZE (11) to be displayed fully in OSD
const char *armingDisableFlagNames[]= {
    [LOG2(ARMING_DISABLED_NO_GYRO)] = "NOGYRO",
    [LOG2(ARMING_DISABLED_FAILSAFE)] = "FAILSAFE",
    [LOG2(ARMING_DISABLED_RX_FAILSAFE)] = "RXLOSS",
    [LOG2(ARMING_DISABLED_NOT_DISARMED)] = "NOT_DISARMED",
    [LOG2(ARMING_DISABLED_BOXFAILSAFE)] = "BOXFAILSAFE",
    [LOG2(ARMING_DISABLED_RUNAWAY_TAKEOFF)] = "RUNAWAY",
    [LOG2(ARMING_DISABLED_CRASH_DETECTED)] = "CRASH",
    [LOG2(ARMING_DISABLED_THROTTLE)] = "THROTTLE",
    [LOG2(ARMING_DISABLED_ANGLE)] = "ANGLE",
    [LOG2(ARMING_DISABLED_BOOT_GRACE_TIME)] = "BOOTGRACE",
    [LOG2(ARMING_DISABLED_NOPREARM)] = "NOPREARM",
    [LOG2(ARMING_DISABLED_LOAD)] = "LOAD",
    [LOG2(ARMING_DISABLED_CALIBRATING)] = "CALIB",
    [LOG2(ARMING_DISABLED_CLI)] = "CLI",
    [LOG2(ARMING_DISABLED_CMS_MENU)] = "CMS",
    [LOG2(ARMING_DISABLED_BST)] = "BST",
    [LOG2(ARMING_DISABLED_MSP)] = "MSP",
    [LOG2(ARMING_DISABLED_PARALYZE)] = "PARALYZE",
    [LOG2(ARMING_DISABLED_GPS)] = "GPS",
    [LOG2(ARMING_DISABLED_RESC)] = "RESCUE_SW",
    [LOG2(ARMING_DISABLED_DSHOT_TELEM)] = "DSHOT_TELEM",
    [LOG2(ARMING_DISABLED_REBOOT_REQUIRED)] = "REBOOT_REQD",
    [LOG2(ARMING_DISABLED_DSHOT_BITBANG)] = "DSHOT_BBANG",
    [LOG2(ARMING_DISABLED_ACC_CALIBRATION)] = "NO_ACC_CAL",
    [LOG2(ARMING_DISABLED_MOTOR_PROTOCOL)] = "MOTOR_PROTO",
    [LOG2(ARMING_DISABLED_CRASHFLIP)] = "CRASHFLIP",
    [LOG2(ARMING_DISABLED_ARM_SWITCH)] = "ARMSWITCH",
};

STATIC_ASSERT(ARRAYLEN(armingDisableFlagNames) == ARMING_DISABLE_FLAGS_COUNT, "armingDisableFlagNames size mismatch");

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
const char *getArmingDisableFlagName(armingDisableFlags_e flag) {
    int idx = ffs(flag & -flag) - 1;   // use LSB if there are multiple bits set
    return idx >= 0 && idx < (int)ARRAYLEN(armingDisableFlagNames) ? armingDisableFlagNames[idx] : "UNKNOWN";
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
