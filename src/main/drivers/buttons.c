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

#include "platform.h"

#if defined(USE_BUTTONS)

#include "drivers/io.h"

#include "drivers/buttons.h"

#ifdef BUTTON_A_PIN
static IO_t buttonAPin = IO_NONE;
#endif

#ifdef BUTTON_B_PIN
static IO_t buttonBPin = IO_NONE;
#endif

void buttonsInit(void)
{
#ifdef BUTTON_A_PIN
    buttonAPin = IOGetByTag(IO_TAG(BUTTON_A_PIN));
    IOInit(buttonAPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(buttonAPin, IOCFG_IPU);
#endif

#ifdef BUTTON_B_PIN
    buttonBPin = IOGetByTag(IO_TAG(BUTTON_B_PIN));
    IOInit(buttonBPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(buttonBPin, IOCFG_IPU);
#endif
}

#ifdef BUTTON_A_PIN
bool buttonAPressed(void)
{
    return !IORead(buttonAPin);
}
#endif

#ifdef BUTTON_B_PIN
bool buttonBPressed(void)
{
    return !IORead(buttonBPin);
}
#endif

#endif
