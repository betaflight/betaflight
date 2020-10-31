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
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#if defined(USE_USB_MSC)

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"

#include "msc/usbd_storage.h"

#include "pg/usb.h"

#define DEBOUNCE_TIME_MS 20
#define ACTIVITY_LED_PERIOD_MS 50

static IO_t mscButton;
static timeMs_t lastActiveTimeMs = 0;

void mscInit(void)
{
    if (usbDevConfig()->mscButtonPin) {
        mscButton = IOGetByTag(usbDevConfig()->mscButtonPin);
        IOInit(mscButton, OWNER_USB_MSC_PIN, 0);
        if (usbDevConfig()->mscButtonUsePullup) {
            IOConfigGPIO(mscButton, IOCFG_IPU);
        } else {
            IOConfigGPIO(mscButton, IOCFG_IPD);
        }
    }
}

bool mscCheckBootAndReset(void)
{
    static bool firstCheck = true;
    static bool mscMode;

    if (firstCheck) {
        // Cache the bootup value of RESET_MSC_REQUEST
        const uint32_t bootModeRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);
        if (bootModeRequest == RESET_MSC_REQUEST) {
            mscMode = true;
            // Ensure the next reset is to the configurator as the H7 processor retains the RTC value so
            // a brief interruption of power is not enough to switch out of MSC mode
            persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
            firstCheck = false;
        }
    }

    return mscMode;
}

void mscSetActive(void)
{
    lastActiveTimeMs = millis();
}

void mscActivityLed(void)
{
    static timeMs_t nextToggleMs = 0;
    const timeMs_t nowMs = millis();

    if (nowMs - lastActiveTimeMs > ACTIVITY_LED_PERIOD_MS) {
        LED0_OFF;
        nextToggleMs = 0;
    } else if (nowMs > nextToggleMs) {
        LED0_TOGGLE;
        nextToggleMs = nowMs + ACTIVITY_LED_PERIOD_MS;
    }
}

bool mscCheckButton(void)
{
    bool result = false;
    if (mscButton) {
        uint8_t state = IORead(mscButton);
        if (usbDevConfig()->mscButtonUsePullup) {
            result = state == 0;
        } else {
            result = state == 1;
        }
    }

    return result;
}

void mscWaitForButton(void)
{
    // In order to exit MSC mode simply disconnect the board, or push the button again.
    while (mscCheckButton());
    delay(DEBOUNCE_TIME_MS);
    while (true) {
        asm("NOP");
        if (mscCheckButton()) {
            systemResetFromMsc();
        }
        mscActivityLed();
    }
}

void systemResetToMsc(int timezoneOffsetMinutes)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_MSC_REQUEST);

    __disable_irq();

    // Persist the RTC across the reboot to use as the file timestamp
#ifdef USE_PERSISTENT_MSC_RTC
    rtcPersistWrite(timezoneOffsetMinutes);
#else
    UNUSED(timezoneOffsetMinutes);
#endif
    NVIC_SystemReset();
}

void systemResetFromMsc(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
    __disable_irq();
    NVIC_SystemReset();
}

#endif
