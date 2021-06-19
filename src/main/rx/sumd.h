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

// driver for SUMD receiver using UART2

// Support for SUMD and SUMD V3
// Tested with 16 channels, SUMD supports up to 16(*), SUMD V3 up to 32 (MZ-32) channels, but limit to MAX_SUPPORTED_RC_CHANNEL_COUNT (currently 8, BF 3.4)
// * According to the original SUMD V1 documentation, SUMD V1 already supports up to 32 Channels?!?

#define SUMD_MAX_CHANNEL 32
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5) // 6 channels + 5 = 17 bytes for 6 channels

struct rxConfig_s;
struct rxRuntimeState_s;

bool sumdInit(const struct rxConfig_s *initialRxConfig, struct rxRuntimeState_s *rxRuntimeState);
