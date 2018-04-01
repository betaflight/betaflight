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

#include "platform.h"

#include "fc/runtime_config.h"
#include "io/beeper.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;

static uint32_t enabledSensors = 0;

// Must be shorter than OSD_WARNINGS_MAX_SIZE (11) to be displayed fully in OSD
const char *armingDisableFlagNames[]= {
    "NOGYRO",
    "FAILSAFE",
    "RXLOSS",
    "BADRX",
    "BOXFAILSAFE",
    "RUNAWAY",
    "THROTTLE",
    "ANGLE",
    "BOOTGRACE",
    "NOPREARM",
    "LOAD",
    "CALIB",
    "CLI",
    "CMS",
    "OSD",
    "BST",
    "MSP",
    "ARMSWITCH"
};

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
