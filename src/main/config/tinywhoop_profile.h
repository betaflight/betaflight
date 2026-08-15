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

#include <stdbool.h>

#ifdef USE_TINYWHOOP

/*
 * Tiny whoop profile.
 *
 * Betaflight's stock defaults are tuned around a 5" freestyle quad; several of
 * them are documented as such in the sources (see the feedforward_yaw_hold_time
 * comment in flight/pid.c). A 31-40mm ducted prop has an order of magnitude less
 * rotational inertia and spins two to three times faster, so it both tolerates
 * and needs different numbers: filters can run higher (less phase delay in the
 * gyro -> PID -> motor path) and the time constants that shape stick feel need
 * to decay faster.
 *
 * This module applies those deltas on top of the stock defaults. It never
 * replaces the stock reset functions - everything is a documented modification
 * of an already-reset parameter group, so a build without USE_TINYWHOOP is
 * bit-for-bit unchanged and the resulting configuration stays a plain, valid
 * Betaflight configuration (same parameter names, same MSP, same Configurator).
 */

// Apply the tiny whoop deltas to every parameter group. Called from resetConfig()
// after pgResetAll() and from the `whoop` CLI command / CMS menu.
void tinywhoopProfileApply(void);

// Apply only the pieces that belong to one PID profile / rate profile pair, so
// a single profile can be converted without disturbing the others.
void tinywhoopProfileApplyToPidProfile(uint8_t pidProfileIndex);
void tinywhoopProfileApplyToRateProfile(uint8_t rateProfileIndex);

#endif // USE_TINYWHOOP
