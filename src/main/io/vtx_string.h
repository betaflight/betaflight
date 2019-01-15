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

bool _vtxStringFreq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChannel);
uint16_t _vtxStringBandchan2Freq(uint8_t band, uint8_t channel);

const uint16_t * vtxStringFrequencyTable(void);
const char ** vtxStringBandNames(void);
const char ** vtxStringChannelNames(void);
const char * vtxStringBandLetters(void);
