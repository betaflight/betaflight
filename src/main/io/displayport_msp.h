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

#include "io/serial.h"

#include "pg/displayport_profiles.h"

// MSP Display Port commands
typedef enum {
    MSP_DP_HEARTBEAT = 0,         // Release the display after clearing and updating
    MSP_DP_RELEASE = 1,         // Release the display after clearing and updating
    MSP_DP_CLEAR_SCREEN = 2,    // Clear the display
    MSP_DP_WRITE_STRING = 3,    // Write a string at given coordinates
    MSP_DP_DRAW_SCREEN = 4,     // Trigger a screen draw
    MSP_DP_OPTIONS = 5,         // Not used by Betaflight. Reserved by Ardupilot and INAV
    MSP_DP_SYS = 6,             // Display system element displayportSystemElement_e at given coordinates
    MSP_DP_COUNT,
} displayportMspCommand_e;

// MSP displayport V2 attribute byte bit functions
#define DISPLAYPORT_MSP_ATTR_VERSION BIT(7) // Format indicator; must be zero for V2 (and V1)
#define DISPLAYPORT_MSP_ATTR_BLINK   BIT(6) // Device local blink
#define DISPLAYPORT_MSP_ATTR_FONT    (BIT(0) | BIT(1)) // Select bank of 256 characters as per displayPortSeverity_e
#define DISPLAYPORT_MSP_ATTR_MASK    (~(DISPLAYPORT_MSP_ATTR_VERSION | DISPLAYPORT_MSP_ATTR_BLINK | DISPLAYPORT_MSP_ATTR_FONT))

struct displayPort_s *displayPortMspInit(void);
void displayPortMspSetSerial(serialPortIdentifier_e serialPort);

