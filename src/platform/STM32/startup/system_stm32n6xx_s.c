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

#include "platform.h"

// Secure-side system initialisation stub for STM32N6.
// On TrustZone-enabled N6, the secure side configures SAU/IDAU before
// jumping to the non-secure firmware. For Betaflight running as a
// non-secure application this file is intentionally minimal.

void SystemInit_S(void)
{
    // Nothing to do - secure init handled by boot ROM or secure bootloader.
}
