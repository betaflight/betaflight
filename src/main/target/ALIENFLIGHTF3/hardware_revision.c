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

#include "build_config.h"

#include "drivers/system.h"
#include "drivers/gpio.h"

#include "hardware_revision.h"

static const char * const hardwareRevisionNames[] = {
        "Unknown",
        "AlienFlight V1",
        "AlienFlight V2"
};

uint8_t hardwareRevision = UNKNOWN;

void detectHardwareRevision(void)
{
    gpio_config_t cfg = {HW_PIN, Mode_IPU, Speed_2MHz};
    RCC_AHBPeriphClockCmd(HW_PERIPHERAL, ENABLE);
    gpioInit(HW_GPIO, &cfg);

    // Check hardware revision
    delayMicroseconds(10);  // allow configuration to settle
    if (digitalIn(HW_GPIO, HW_PIN)) {
        hardwareRevision = AFF3_REV_1;
    } else {
        hardwareRevision = AFF3_REV_2;
    }
}

void updateHardwareRevision(void)
{
}
