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

#include "platform.h"

#if ENABLE_DRONECAN_DNA

#include "common/time.h"

// Install the dynamic node-ID allocation handler. No-op if dronecan_dna_enabled
// is clear. Called once from dronecanInit().
void dronecanDnaInit(void);

// Drop an in-progress allocation session that has stalled past the follow-up
// timeout. Called on the dronecan task's 1 Hz boundary.
void dronecanDnaExpireSessions(timeUs_t currentTimeUs);

#endif // ENABLE_DRONECAN_DNA
