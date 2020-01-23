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

#include "drivers/display.h"

#include "pg/displayport_profiles.h"

// MSP displayport V2 attribute byte bit functions
#define DISPLAYPORT_MSP_ATTR_VERSION BIT(7) // Format indicator; must be zero for V2 (and V1)
#define DISPLAYPORT_MSP_ATTR_BLINK   BIT(6) // Device local blink
#define DISPLAYPORT_MSP_ATTR_MASK    (~(DISPLAYPORT_MSP_ATTR_VERSION|DISPLAYPORT_MSP_ATTR_BLINK))

struct displayPort_s *displayPortMspInit(void);
