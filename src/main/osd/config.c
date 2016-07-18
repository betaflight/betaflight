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
#include <string.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/profile.h"
#include "config/config_reset.h"
#include "config/config_system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include "fc/rate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_adjustments.h"
#include "fc/config.h"

#include "io/beeper.h"
#include "io/serial.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/navigation.h"


// FIXME remove the includes below when target specific configuration is moved out of this file
#include "sensors/battery.h"
#include "io/motor_and_servo.h"


#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif


// Default settings
STATIC_UNIT_TESTED void resetConf(void)
{
    pgResetAll(MAX_PROFILE_COUNT);
}

static void activateConfig(void)
{
}

static void validateAndFixConfig(void)
{
    if (!isSerialConfigValid(serialConfig())) {
        PG_RESET_CURRENT(serialConfig);
    }

#if defined(USE_VCP)
    serialConfig()->portConfigs[0].functionMask = FUNCTION_MSP_SERVER;
#endif
}

void readEEPROM(void)
{
    // Sanity check, read flash
    if (!scanEEPROM(true)) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    validateAndFixConfig();
    activateConfig();
}

void writeEEPROM(void)
{
    writeConfigToEEPROM();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();
}

// FIXME stub out the profile code, unused in the OSD but the msp.c parameter group code still references getCurrentProfile

uint8_t getCurrentProfile(void) { return 0; }

