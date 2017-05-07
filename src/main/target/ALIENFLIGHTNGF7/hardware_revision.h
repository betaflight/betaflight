/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

typedef enum awf7HardwareRevision_t {
    AFF7_UNKNOWN = 0,
    AFF7_REV_1, // ICM-20602 (SPI), Current Sensor (ACS712), SDCard Reader
    AFF7_REV_2  // ICM-20602 (SPI), OpenSky RX (CC2510), Current Sensor (ACS712), SDCard Reader
} awf7HardwareRevision_e;

extern uint8_t hardwareRevision;

void updateHardwareRevision(void);
void detectHardwareRevision(void);
