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

#include "platform.h"

#include "drivers/can/can_types.h"
#include "drivers/io_types.h"

typedef enum {
    CANINVALID = -1,
    CANDEV_1 = 0,
    CANDEV_2,
    CANDEV_3,
    CANDEV_COUNT
} canDevice_e;

// Macros to convert between CLI device number (1-based) and canDevice_e (0-based).
#define CAN_CFG_TO_DEV(x)   ((x) - 1)
#define CAN_DEV_TO_CFG(x)   ((x) + 1)

// Maximum payload length for a Classic CAN frame.
#define CAN_CLASSIC_MAX_DLC 8

// Callback invoked from interrupt context when a message is received.
//  identifier  - 11-bit (standard) or 29-bit (extended) message identifier
//  isExtended  - true if the identifier is 29-bit
//  data        - pointer to message payload (valid only during the callback)
//  length      - payload length in bytes (0..8 for classic CAN)
typedef void (*canRxCallbackPtr)(uint32_t identifier, bool isExtended,
                                 const uint8_t *data, uint8_t length);

// Initialise the given CAN device. Returns false if the device has no
// hardware resources configured or the peripheral failed to start.
bool canInit(canDevice_e device);

// Transmit a classic CAN frame. Returns true if the frame was queued,
// false if the TX FIFO is full, the device is uninitialised, or the
// parameters are invalid.
bool canTransmit(canDevice_e device, uint32_t identifier, bool isExtended,
                 const uint8_t *data, uint8_t length);

// Register a callback for received messages on the given device.
// Passing NULL removes the callback. Only one callback per device is
// supported; registering a new callback replaces any previous one.
void canRegisterRxCallback(canDevice_e device, canRxCallbackPtr callback);
