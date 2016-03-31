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

// System-wide
typedef struct master_t {
    // Profile configuration
    uint8_t current_profile_index;

    // Motor/ESC/Servo configuration
    flight3DConfig_t flight3DConfig;

    rxConfig_t rxConfig;

    // Flight config
    airplaneConfig_t airplaneConfig;

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

#ifdef TELEMETRY
    telemetryConfig_t telemetryConfig;
#endif

#ifdef LED_STRIP
    ledConfig_t ledConfigs[MAX_LED_STRIP_LENGTH];
    hsvColor_t colors[CONFIGURABLE_COLOR_COUNT];
#endif
} master_t;

PG_DECLARE(master_t, masterConfig);
extern controlRateConfig_t *currentControlRateProfile;
