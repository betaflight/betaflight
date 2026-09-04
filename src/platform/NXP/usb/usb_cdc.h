/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Bring up the PHY and the device stack. Call once, before the port is opened.
void cdcUsbInit(void);

// Stop the controller and power the PHY down so the host sees the device leave.
// Safe to call at any time.
void cdcUsbDetach(void);

// Run the device stack often enough that control transfers do not time out.
void cdcUsbTask(void);

bool cdcUsbMounted(void);

// Mounted, DTR set, not suspended.
bool cdcUsbConnected(void);

// Drop the enumeration after VBUS has gone. Does nothing when not connected.
void cdcUsbUnplugged(void);

uint32_t cdcUsbBytesAvailable(void);
uint32_t cdcUsbTxBytesFree(void);

// Whether everything written is with the endpoint; true for a port nobody reads.
bool cdcUsbTxIdle(void);

// Both return the number of bytes moved, which can be short.
uint32_t cdcUsbRead(uint8_t *buf, uint32_t length);
uint32_t cdcUsbWrite(const uint8_t *buf, uint32_t length);

// The line speed the host last asked for.
uint32_t cdcUsbBaudRate(void);

// A NULL callback turns the notification off.
void cdcUsbSetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context);
void cdcUsbSetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context);
