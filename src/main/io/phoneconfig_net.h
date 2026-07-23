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

#include <stdint.h>
#include <stdbool.h>

// bring up lwip, the netif, dhcp server and tcp server. `mac` is the 6-byte hardware address
void phoneConfigNetInit(const uint8_t *mac);

// pump lwip timers and deferred work, called repeatedly from the net task
void phoneConfigNetPoll(void);

// called from the usb-ncm class when an ethernet frame arrives from the host
void phoneConfigNetRxFrame(const uint8_t *frame, uint32_t length);

// called from the usb-ncm class when the usb link state changes
void phoneConfigNetLinkChange(uint8_t up);

// send an ethernet frame over usb-ncm, implemented by the per-mcu usb layer. false if the tx path
// is busy and the frame was dropped
bool phoneConfigUsbTransmitFrame(const uint8_t *frame, uint16_t length);

// drain deferred usb-ncm rx into lwip, implemented by the per-mcu usb layer
void phoneConfigUsbProcess(void);
