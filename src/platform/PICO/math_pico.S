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

#include "pico/asm_helper.S"

pico_default_asm_setup

.section .text.mathfunctions, "ax"

// Wrapped lrintf function, because gcc arm (13.3) / newlib is
// pulling in a long slow function that unpacks floats by hand.
// Round to nearest integer ("bankers' round", n + 0.5 -> nearest even integer).
wrapper_func lrintf
    vmov s0, r0
    vcvtn.s32.f32 s0, s0
    vmov  r0, s0
    bx lr
