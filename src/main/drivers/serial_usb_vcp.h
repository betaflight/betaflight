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

#pragma once

#include "serial.h"

typedef struct {
    serialPort_t port;

} vcpPort_t;

serialPort_t *usbVcpOpen(void);

uint8_t usbVcpAvailable(serialPort_t *instance);

uint8_t usbVcpRead(serialPort_t *instance);

void usbVcpWrite(serialPort_t *instance, uint8_t ch);
void usbPrintStr(const char *str);
