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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// sitl_gyro.h is a header-only (static inline) helper. This translation unit
// exists so the gyro-mapping logic is also compiled as C (matching its use in
// sitl.c) alongside the C++ unit test, and to satisfy the unit-test framework's
// requirement that every test declare a non-empty _SRC.

#include "sitl_gyro.h"
