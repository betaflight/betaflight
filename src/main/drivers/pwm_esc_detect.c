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

#include "time.h"
#include "system.h"
#include "io.h"
#include "pwm_esc_detect.h"
#include "pwm_mapping.h"
#include "timer.h"

#ifdef BRUSHED_ESC_AUTODETECT
uint8_t hardwareMotorType = MOTOR_UNKNOWN;

void detectBrushedESC(void)
{
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_MC_MOTOR) {
            IO_t MotorDetectPin = IOGetByTag(timerHardware[i].tag);
            IOInit(MotorDetectPin, OWNER_SYSTEM, RESOURCE_INPUT, 0);
            IOConfigGPIO(MotorDetectPin, IOCFG_IPU);

            delayMicroseconds(10);  // allow configuration to settle

            // Check presence of brushed ESC's
            if (IORead(MotorDetectPin)) {
                hardwareMotorType = MOTOR_BRUSHLESS;
            } else {
                hardwareMotorType = MOTOR_BRUSHED;
            }

            return;
        }
    }

    // Not found = assume brushless
    hardwareMotorType = MOTOR_BRUSHLESS;
}
#endif
