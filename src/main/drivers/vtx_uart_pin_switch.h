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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "common/time.h"

struct serialPort_s;

// Start the existing protocol processing window; this is not a TX-complete hook.
void vtxUartPinSwitchBegin(struct serialPort_s *port, timeMs_t nowMs, uint16_t rxDelayMs);
// Poll from the flight loop. The platform reads millis() only while awaiting RX.
void vtxUartPinSwitchUpdate(void);
