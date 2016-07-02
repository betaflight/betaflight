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

#include <platform.h>

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/serial.h"
#include "drivers/sensor.h"
#include "drivers/pwm_mapping.h"
#include "drivers/accgyro.h"

#include "io/serial_cli.h"
#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#include "mw.h"
#include "scheduler.h"

#include "statusindicator.h"

static uint32_t warningLedTimer = 0;

typedef enum {
    ARM_PREV_NONE       = 0,
    ARM_PREV_CLI        = 0x00205, //         0b1000000101  2 flashes - CLI active in the configurator
    ARM_PREV_FAILSAFE   = 0x00815, //       0b100000010101  3 flashes - Failsafe mode
    ARM_PREV_ANGLE      = 0x02055, //     0b10000001010101  4 flashes - Maximum arming angle exceeded
    ARM_PREV_CALIB      = 0x08155, //   0b1000000101010101  5 flashes - Calibration active
    ARM_PREV_OVERLOAD   = 0x20555  // 0b100000010101010101  6 flashes - System overload
} armingPreventedReason_e;

static uint32_t blinkMask = 0;

armingPreventedReason_e getArmingPreventionReason(void)
{
    if (isCalibrating()) {
        return ARM_PREV_CALIB;
    }
    if (rcModeIsActive(BOXFAILSAFE) || failsafePhase() == FAILSAFE_LANDED) {
        return ARM_PREV_FAILSAFE;
    }
    if (!imuIsAircraftArmable(armingConfig()->max_arm_angle)) {
        return ARM_PREV_ANGLE;
    }
    if (cliMode) {
        return ARM_PREV_CLI;
    }
    if (isSystemOverloaded()) {
        return ARM_PREV_OVERLOAD;
    }
    return ARM_PREV_NONE;
}


void warningLedRefresh(void)
{
    if (blinkMask <= 1) {  // skip interval for terminator bit, allow continuous on
        blinkMask = getArmingPreventionReason();
    }
    if (blinkMask) {
        if (blinkMask & 1)
            LED0_ON;
        else
            LED0_OFF;
        blinkMask >>= 1;
    }
}

void warningLedBeeper(bool on)
{
    if (!blinkMask) {
        if (on) {
            LED0_ON;
        } else {
            LED0_OFF;
        }
    }
}

void warningLedUpdate(void)
{
    uint32_t now = micros();
    if ((int32_t)(now - warningLedTimer) > 0) {
        warningLedRefresh();
        warningLedTimer = now + WARNING_LED_BLINK_DELAY;
    }
}


