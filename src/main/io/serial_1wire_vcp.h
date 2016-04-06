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
 *
 * Author 4712
 */
#pragma once

#include <platform.h>
#ifdef USE_SERIAL_1WIRE_VCP
#include "drivers/serial.h"
#include "drivers/buf_writer.h"
#include "drivers/pwm_mapping.h"
#include "io/serial_msp.h"

extern uint8_t escCount;

typedef struct {
    GPIO_TypeDef* gpio;
    uint16_t pinpos;
    uint16_t pin;
    gpio_config_t gpio_config_INPUT;
    gpio_config_t gpio_config_OUTPUT;
} escHardware_t;

void usb1WireInitializeVcp(void);
void usb1WireDeInitializeVcp(void);
void usb1WirePassthroughVcp(mspPort_t *mspPort, bufWriter_t *bufwriter, uint8_t escIndex);
#endif

