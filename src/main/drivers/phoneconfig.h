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

#include "common/time.h"

// true (cached, once per boot) if booted into phone-config mode. clears the request on first read
bool phoneConfigCheckBootAndReset(void);

// phoneConfigNetStart() brings up lwip and the msp-over-tcp server, call once after mspSerialInit().
// phoneConfigNetTask() is the scheduler task that pumps the usb-ncm/lwip link.
void phoneConfigNetStart(void);
void phoneConfigNetTask(timeUs_t currentTimeUs);

// sets the phone-config request flag (rtc backup register) and resets the mcu.
void systemResetToPhoneConfig(void);

// brings up the usb network device (cdc-ncm), implemented per-mcu. returns 0 on success
uint8_t phoneConfigUsbStart(void);

// scheduler task for the flip-and-hold gesture that reboots into phone-config mode
void phoneConfigGestureUpdate(timeUs_t currentTimeUs);

// mask/unmask the usb (otg) interrupt around main-loop usb work, the st usb low level is not
// re-entrant. weak no-ops unless the per-mcu usb layer overrides them.
void phoneConfigUsbLock(void);
void phoneConfigUsbUnlock(void);

// true while the usb in endpoint is still sending a datagram. the tx flush gates on this
bool phoneConfigUsbTxBusy(void);
