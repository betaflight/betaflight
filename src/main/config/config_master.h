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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "common/axis.h"
#include "common/color.h"

#include "config/config_profile.h"

#include "drivers/pwm_rx.h"

#include "fc/config.h"
#include "fc/rc_controls.h"

#ifdef USE_SERVOS
#include "flight/servos.h"
#endif

#include "io/osd.h"
#include "io/ledstrip.h"

#define osdProfile(x) (&masterConfig.osdProfile)
#define osdProfileMutable(x) (&masterConfig.osdProfile)
#define pwmRxConfig(x) (&masterConfig.pwmRxConfig)
#define pwmRxConfigMutable(x) (&masterConfig.pwmRxConfig)


// System-wide
typedef struct master_s {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint8_t persistentFlags;
    uint8_t i2c_overclock;                  // Overclock i2c Bus for faster IMU readings

#ifdef ASYNC_GYRO_PROCESSING
    uint16_t accTaskFrequency;
    uint16_t attitudeTaskFrequency;
    uint8_t asyncMode;
#endif

    uint8_t throttle_tilt_compensation_strength;      // the correction that will be applied at throttle_correction_angle.

    pwmRxConfig_t pwmRxConfig;

#ifdef OSD
    osd_profile_t osdProfile;
#endif

    profile_t profile[MAX_PROFILE_COUNT];

    uint32_t beeper_off_flags;
    uint32_t preferred_beeper_off_flags;

    char name[MAX_NAME_LENGTH + 1];

    uint8_t debug_mode;

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
    /*
        do not add properties after the MAGIC_EF and CHK
        as it is assumed to exist at length-2 and length-1
    */
} master_t;

extern master_t masterConfig;
extern const profile_t *currentProfile;

void createDefaultConfig(master_t *config);
