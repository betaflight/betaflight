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

// ESP32-S3 minimal timer framework.
// The ESP32 uses dedicated peripherals (LEDC for PWM, RMT for DShot/LED)
// rather than general-purpose timers. This provides the minimum definitions
// needed for USE_TIMER to be defined, enabling code paths in cli.c and
// config.c that reference timerGetConfiguredByTag/timerAllocate.

// No timer channels are mapped to pins - all PWM output uses LEDC/RMT directly.
#define FULL_TIMER_CHANNEL_COUNT  0

// No hardware timers tracked for resource allocation
#define USED_TIMERS  0
#define HARDWARE_TIMER_DEFINITION_COUNT  0
