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

#pragma once

// TODO - GPIO_TypeDef include
#include "platform.h"

#include "drivers/io.h"

typedef struct ioDef_s {
    ioTag_t tag;
} ioDef_t;

typedef struct ioRec_s {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    resourceOwner_e owner;
    uint8_t index;
} ioRec_t;

extern ioRec_t ioRecs[];

int IO_GPIOPortIdx(IO_t io);
int IO_GPIOPinIdx(IO_t io);

int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);

#if defined(STM32F4)
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
#endif

GPIO_TypeDef* IO_GPIO(IO_t io);
uint16_t IO_Pin(IO_t io);

#define IO_GPIOBYTAG(tag) IO_GPIO(IOGetByTag(tag))
#define IO_PINBYTAG(tag) IO_Pin(IOGetByTag(tag))
#define IO_GPIOPortIdxByTag(tag) DEFIO_TAG_GPIOID(tag)
#define IO_GPIOPinIdxByTag(tag) DEFIO_TAG_PIN(tag)

uint32_t IO_EXTI_Line(IO_t io);
ioRec_t *IO_Rec(IO_t io);
