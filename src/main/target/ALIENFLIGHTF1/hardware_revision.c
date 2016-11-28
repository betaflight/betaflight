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
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "hardware_revision.h"

static const char * const hardwareRevisionNames[] = {
        "Unknown",
        "AlienFlight F1 V1",
};

uint8_t hardwareRevision = AFF1_REV_1;
uint8_t hardwareMotorType = MOTOR_UNKNOWN;

static IO_t MotorDetectPin = IO_NONE;

void detectHardwareRevision(void)
{
    MotorDetectPin = IOGetByTag(IO_TAG(MOTOR_PIN));
    IOInit(MotorDetectPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(MotorDetectPin, IOCFG_IPU);

    delayMicroseconds(10);  // allow configuration to settle

    // Check presence of brushed ESC's
    if (IORead(MotorDetectPin)) {
        hardwareMotorType = MOTOR_BRUSHLESS;
    } else {
        hardwareMotorType = MOTOR_BRUSHED;
    }
}

void updateHardwareRevision(void)
{
}

const extiConfig_t *selectMPUIntExtiConfigByHardwareRevision(void)
{
    return NULL;
}
