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

#include "drivers/usb_io.h"

#ifdef USB_IO

void usbCableDetectDeinit(void)
{
#ifdef USB_CABLE_DETECTION
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = USB_DETECT_PIN;
    GPIO_Init(USB_DETECT_GPIO_PORT, &GPIO_InitStructure);
#endif
}

void usbCableDetectInit(void)
{
#ifdef USB_CABLE_DETECTION
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

#ifdef USB_CABLE_DETECTION
    result = (GPIO_ReadInputData(USB_DETECT_GPIO_PORT) & USB_DETECT_PIN) != 0;
#endif

    return result;
}

void usbGenerateDisconnectPulse(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Pull down PA12 to create USB disconnect pulse */
#if defined(STM32F303xC)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
#endif

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, GPIO_Pin_12);

    delay(200);

    GPIO_SetBits(GPIOA, GPIO_Pin_12);
}

#endif
