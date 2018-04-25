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

#include <stdint.h>

#include "platform.h"
#include "drivers/vtx_common.h"


extern const uint16_t vtx58frequencyTable[VTX_SETTINGS_BAND_COUNT][VTX_SETTINGS_CHANNEL_COUNT];
extern const char * const vtx58BandNames[];
extern const char * const vtx58ChannelNames[];
extern const char vtx58BandLetter[];

bool vtx58_Freq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChannel);
uint16_t vtx58_Bandchan2Freq(uint8_t band, uint8_t channel);
