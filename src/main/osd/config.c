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

#include "osd/config.h"

#include "drivers/system.h"
#include "drivers/serial.h"

#include "io/serial.h"

#include "sensors/amperage.h"

#include "sensors/battery.h"

// Default settings
STATIC_UNIT_TESTED void resetConf(void)
{
    pgResetAll(MAX_PROFILE_COUNT);

#ifdef BOARD_HAS_AMPERAGE_METER
    batteryConfig()->amperageMeterSource = AMPERAGE_METER_ADC;
#endif
#ifdef DEFAULT_FEATURES
    featureSet(DEFAULT_FEATURES);
#endif
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
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    validateAndFixConfig();
    activateConfig();
}

void writeEEPROM(void)
{
    writeConfigToEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM();
}

// FIXME stub out the profile code, unused in the OSD but the msp.c parameter group code still references getCurrentProfile

uint8_t getCurrentProfile(void) { return 0; }

