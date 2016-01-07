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

#include "sdcard.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "gpio.h"

#include "drivers/system.h"

void usbCableDetectDeinit(void)
{
#ifdef USB_DETECT_PIN
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = USB_DETECT_PIN;
    GPIO_Init(USB_DETECT_GPIO_PORT, &GPIO_InitStructure);
#endif
}

void usbCableDetectInit(void)
{
#ifdef USB_DETECT_PIN
    RCC_AHBPeriphClockCmd(USB_DETECT_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = USB_DETECT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USB_DETECT_GPIO_PORT, &GPIO_InitStructure);
#endif
}

bool usbCableIsInserted(void)
{
    bool result = false;

#ifdef USB_DETECT_PIN
    result = (GPIO_ReadInputData(USB_DETECT_GPIO_PORT) & USB_DETECT_PIN) != 0;
#endif

    return result;
}
