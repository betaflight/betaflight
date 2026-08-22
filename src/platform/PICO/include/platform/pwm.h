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

typedef struct picoPwmOutput_s {
    uint16_t slice;
    uint16_t channel;
    uint16_t level;      // The "compare" level, written directly (continuous update) or in pwmCompleteMotorUpdate (one-shot modes)
    bool sliceHead;    // The first motor on this slice
    bool initialised;
} picoPwmOutput_t;
