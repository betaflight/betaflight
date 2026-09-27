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

#include "drivers/serial.h"

typedef struct {
    serialPort_t port;

    // Buffer used during bulk writes.
    uint8_t txBuf[20];
    uint8_t txAt;
    // Set if the port is in bulk write mode and can buffer.
    bool buffering;
} vcpPort_t;

void usbVcpInit(void);
serialPort_t *usbVcpOpen(void);
struct serialPort_s;
uint32_t usbVcpGetBaudRate(struct serialPort_s *instance);

// Has the device been enumerated since boot. Latches: without VBUS sensing there is no
// disconnect event, so this does not clear when the cable is pulled.
uint8_t usbVcpIsConnected(void);

// Is the host driving the link right now. Clears on unplug or host suspend.
uint8_t usbVcpIsActive(void);
