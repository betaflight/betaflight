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

#include <stdbool.h>

// Brushed reverse control (external H-bridge / dual NMOS direction pin).
//
// When enabled (USE_BRUSHED_FLIPOVERAFTERCRASH + BRUSHED_REVERSE_PIN), this module:
// - Configures the reverse GPIO early at boot and drives it to "normal" direction (safe state).
// - Allows switching between normal/reversed direction at runtime.
//
// Polarity:
// - If BRUSHED_FLIPOVERAFTERCRASH_LOW_ACTIVE is defined, "reversed" is active-low.
// - Otherwise, "reversed" is active-high.

void brushedReverseInit(void);
void brushedReverseSetReversed(bool reversed);
bool brushedReverseIsAvailable(void);

