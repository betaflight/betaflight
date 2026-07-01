/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * phone-config mode: an opt-in usb mode for configuring betaflight over a cable from a phone (e.g. an
 * iphone, which can't open a usb cdc serial port but can talk to a usb-cdc-ncm network gadget). the
 * `phoneconfig` cli command or the flip-and-hold gesture sets an rtc-backup flag and reboots, then
 * init() boots the full fc and adds the usb-ncm/lwip link as a scheduler task serving msp/cli over tcp.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_PHONE_CONFIG

#include "common/axis.h"
#include "common/time.h"

#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "io/phoneflash.h"

#include "sensors/acceleration.h"

bool phoneConfigCheckBootAndReset(void)
{
    static bool firstCheck = true;
    static bool phoneConfigMode = false;

    if (firstCheck) {
        firstCheck = false;
        if (persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON) == RESET_PHONECONFIG_REQUEST) {
            phoneConfigMode = true;
            // clear it so the next power-up boots normally
            persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        }
    }

    return phoneConfigMode;
}

void systemResetToPhoneConfig(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_PHONECONFIG_REQUEST);

    __disable_irq();
    NVIC_SystemReset();
}

// weak default, overridden per-mcu (phoneconfig_usb_<mcu>.c). returns 0 on success
__attribute__((weak)) uint8_t phoneConfigUsbStart(void)
{
    return 1;
}

// weak defaults, overridden by phoneconfig_net.c (lwip) and the per-mcu usb layer when present.
__attribute__((weak)) void phoneConfigNetPoll(void) { }
__attribute__((weak)) void phoneConfigUsbProcess(void) { }
__attribute__((weak)) void phoneConfigUsbLock(void) { }
__attribute__((weak)) void phoneConfigUsbUnlock(void) { }

// scheduler task: pump bytes between usb-ncm and lwip and run the lwip timers. the st usb low level
// must run only in the otg isr, so mask it around this work. TASK_SERIAL serves msp/cli over the port.
void phoneConfigNetTask(timeUs_t currentTimeUs)
{
    (void)currentTimeUs;

    phoneConfigUsbLock();
    phoneConfigUsbProcess();   // usb-ncm rx into lwip
    phoneConfigNetPoll();      // lwip timers, flush the msp tx ring
    phoneConfigUsbUnlock();

    // hand off to the ram burn engine on a reflash request (does not return unless the host never commits)
    if (phoneFlashPending() && !ARMING_FLAG(ARMED)) {
        phoneFlashRun();
    }
}

// flip-and-hold gesture: held mostly inverted for ~3 s while disarmed, reboots into phone-config.
// lets you trigger it from the phone, since the `phoneconfig` cli command needs a pc.
#define PHONE_CONFIG_GESTURE_Z_THRESHOLD  0.7f          // fraction of 1g, z < -0.7g is within ~45 deg of inverted
#define PHONE_CONFIG_GESTURE_HOLD_US      (3 * 1000000)

void phoneConfigGestureUpdate(timeUs_t currentTimeUs)
{
#if defined(USE_ACC)
    static timeUs_t invertedSinceUs = 0;

    const bool eligible = !ARMING_FLAG(ARMED) && accIsCalibrationComplete();
    const bool inverted = eligible &&
        (acc.accADC.z < -PHONE_CONFIG_GESTURE_Z_THRESHOLD * (float)acc.dev.acc_1G);

    if (!inverted) {
        invertedSinceUs = 0;
        return;
    }

    if (invertedSinceUs == 0) {
        invertedSinceUs = currentTimeUs;
    } else if (cmpTimeUs(currentTimeUs, invertedSinceUs) >= PHONE_CONFIG_GESTURE_HOLD_US) {
        systemResetToPhoneConfig();   // never returns
    }
#else
    (void)currentTimeUs;
#endif
}

#endif // USE_PHONE_CONFIG
