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

#if defined(USE_BLACKBOX_VIRTUAL) && !defined(SIMULATOR_BUILD)
#error "USE_BLACKBOX_VIRTUAL valid for SITL build only"
#endif

bool blackboxVirtualOpen(void);
void blackboxVirtualPutChar(uint8_t value);
void blackboxVirtualWrite(const uint8_t *buffer, uint32_t len);
bool blackboxVirtualFlush(void);
bool blackboxVirtualBeginLog(void);
bool blackboxVirtualEndLog(void);
void blackboxVirtualClose(void);
uint32_t blackboxVirtualLogFileNumber(void);
