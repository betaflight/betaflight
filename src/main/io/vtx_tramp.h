/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#define VTX_TRAMP_MIN_BAND 1
#define VTX_TRAMP_MAX_BAND 5
#define VTX_TRAMP_MIN_CHANNEL 1
#define VTX_TRAMP_MAX_CHANNEL 8

#define VTX_TRAMP_BAND_COUNT (VTX_TRAMP_MAX_BAND - VTX_TRAMP_MIN_BAND + 1)
#define VTX_TRAMP_CHANNEL_COUNT (VTX_TRAMP_MAX_CHANNEL - VTX_TRAMP_MIN_CHANNEL + 1)

#define VTX_TRAMP_POWER_COUNT 5
#define VTX_TRAMP_DEFAULT_POWER 1

#define VTX_TRAMP_MIN_FREQUENCY_MHZ 5000             //min freq in MHz
#define VTX_TRAMP_MAX_FREQUENCY_MHZ 5999             //max freq in MHz

extern const uint16_t trampPowerTable[VTX_TRAMP_POWER_COUNT];
extern const char * const trampPowerNames[VTX_TRAMP_POWER_COUNT+1];

extern uint8_t trampBand;
extern uint8_t trampChannel;
extern uint16_t trampPower;       // Actual transmitting power
extern uint8_t trampPitMode;
extern uint32_t trampCurFreq;
extern uint16_t trampConfiguredPower; // Configured transmitting power
extern int16_t trampTemperature;

bool vtxTrampInit(void);
bool trampCommitChanges(void);
void trampSetPitMode(uint8_t onoff);
void trampSetBandAndChannel(uint8_t band, uint8_t channel);
void trampSetRFPower(uint16_t level);
