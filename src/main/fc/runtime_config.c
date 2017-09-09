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

uint32_t armingFlags = 0;
uint32_t stateFlags = 0;
uint32_t flightModeFlags = 0;

static uint32_t enabledSensors = 0;

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint32_t enableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint32_t disableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

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

flightModeForTelemetry_e getFlightModeForTelemetry(void)
{
    if (FLIGHT_MODE(PASSTHRU_MODE))
        return FLM_MANUAL;

    if (FLIGHT_MODE(FAILSAFE_MODE))
        return FLM_FAILSAFE;

    if (FLIGHT_MODE(NAV_RTH_MODE))
        return FLM_RTH;

    if (FLIGHT_MODE(NAV_POSHOLD_MODE))
        return FLM_POSITION_HOLD;

    if (FLIGHT_MODE(NAV_WP_MODE))
        return FLM_MISSION;

    if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
        return FLM_ALTITUDE_HOLD;

    if (FLIGHT_MODE(ANGLE_MODE))
        return FLM_ANGLE;

    if (FLIGHT_MODE(HORIZON_MODE))
        return FLM_HORIZON;

    return FLM_ACRO;
}