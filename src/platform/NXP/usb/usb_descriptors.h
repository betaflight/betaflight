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

// Endpoint 1 IN carries the CDC notifications, endpoint 2 the data.
#define EPNUM_CDC_NOTIFY    0x81
#define EPNUM_CDC_OUT       0x02
#define EPNUM_CDC_IN        0x82

// Offer the mass storage identity; set before the device stack starts.
#ifdef USE_USB_MSC
void usbDescriptorsSetMscMode(bool mscMode);
#endif
