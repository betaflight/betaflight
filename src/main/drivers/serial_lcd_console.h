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

// L1 — virtual serialPort_t over the LCD console terminal layer, plus an
// explicit log API for code that wants to write to the LCD regardless of
// where the global printf sink is currently pointed.

struct serialPort_s;

// Singleton accessor. Lazy-inits the L2/L3 layers on first call. Returns
// NULL only if the panel failed to come up.
struct serialPort_s *lcdConsoleSerialOpen(void);

// Write a null-terminated string straight to the LCD, bypassing the global
// printf sink. Use when CLI may be active and you still want output on the LCD.
void lcdConsolePuts(const char *s);

// Formatted print straight to the LCD; same caveat as lcdConsolePuts().
void lcdConsolePrintf(const char *fmt, ...);
