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

/*
 * Author: Chris Hockuba (https://github.com/conkerkh)
 */

#pragma once

void mscInit(void);
bool mscCheckBootAndReset(void);
uint8_t mscStart(void);
bool mscCheckButton(void);
void mscWaitForButton(void);
void systemResetToMsc(int timezoneOffsetMinutes);
void systemResetFromMsc(void);
void mscSetActive(void);
void mscActivityLed(void);
