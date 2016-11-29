/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

typedef enum omnibusf4HardwareRevision_t {
    UNKNOWN,
    OMNIBUSF4V1, // 16mb of flash on a M25P16
    OMNIBUSF4V2 // PPM and S2_IN are re-mapped, added a BMP280, SDCARD instead of flash
} omnibusHardwareRevision_e;

extern uint8_t hardwareRevision;

void updateHardwareRevision(void);
void detectHardwareRevision(void);
void omnibusf4V1RemapTimerHardware(void);
