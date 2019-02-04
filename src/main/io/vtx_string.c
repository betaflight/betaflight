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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#if defined(USE_VTX_COMMON)

#define VTX_STRING_BAND_COUNT 5
#define VTX_STRING_CHAN_COUNT 8

static const uint16_t vtx58frequencyTable[VTX_STRING_BAND_COUNT][VTX_STRING_CHAN_COUNT] =
{
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
};

static const char * vtx58BandNames[] = {
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

static char const vtx58BandLetter[] = "-ABEFR";

static char const * vtx58ChannelNames[] = {
    "-", "1", "2", "3", "4", "5", "6", "7", "8",
};

const uint16_t *vtxStringFrequencyTable(void)
{
    return &vtx58frequencyTable[0][0];
}

const char **vtxStringBandNames(void)
{
    return vtx58BandNames;
}

const char **vtxStringChannelNames(void)
{
    return vtx58ChannelNames;
}

const char *vtxStringBandLetters(void)
{
    return vtx58BandLetter;
}

#endif
