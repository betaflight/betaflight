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

struct rxConfig_s;
struct rxRuntimeState_s;
float rxMspReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan);
void rxMspInit(const struct rxConfig_s *rxConfig, struct rxRuntimeState_s *rxRuntimeState);
void rxMspFrameReceive(const uint16_t *frame, int channelCount);
uint8_t rxMspOverrideFrameStatus(void);
