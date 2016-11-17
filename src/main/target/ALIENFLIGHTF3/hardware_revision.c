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
        "AlienFlight F3 V1",
        "AlienFlight F3 V2"
};

uint8_t hardwareRevision = AFF3_UNKNOWN;
uint8_t hardwareMotorType = MOTOR_UNKNOWN;

static IO_t HWDetectPin = IO_NONE;
static IO_t MotorDetectPin = IO_NONE;

void detectHardwareRevision(void)
{
    HWDetectPin = IOGetByTag(IO_TAG(HW_PIN));
    IOInit(HWDetectPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(HWDetectPin, IOCFG_IPU);

    MotorDetectPin = IOGetByTag(IO_TAG(MOTOR_PIN));
    IOInit(MotorDetectPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(MotorDetectPin, IOCFG_IPU);

    delayMicroseconds(10);  // allow configuration to settle

    // Check hardware revision
    if (IORead(HWDetectPin)) {
        hardwareRevision = AFF3_REV_1;
    } else {
        hardwareRevision = AFF3_REV_2;
    }

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
    // MPU_INT output on V1 PA15
    static const extiConfig_t alienFlightF3V1MPUIntExtiConfig = {
        .tag = IO_TAG(PA15)
    };
    // MPU_INT output on V2 PB13
    static const extiConfig_t alienFlightF3V2MPUIntExtiConfig = {
        .tag = IO_TAG(PB13)
    };

    if (hardwareRevision == AFF3_REV_1) {
        return &alienFlightF3V1MPUIntExtiConfig;
    }
    else {
        return &alienFlightF3V2MPUIntExtiConfig;
    }
}
