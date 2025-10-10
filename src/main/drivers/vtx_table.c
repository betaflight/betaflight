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
#include <ctype.h>
#include <string.h>

#include "platform.h"

#include "drivers/vtx_table.h"

#if defined(USE_VTX_TABLE)
#include "common/printf.h"

#include "pg/vtx_table.h"
#include "drivers/vtx_common.h"
#endif

#if defined(USE_VTX_TABLE)
int            vtxTableBandCount;
int            vtxTableChannelCount;
uint16_t       vtxTableFrequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS];
const char *   vtxTableBandNames[VTX_TABLE_MAX_BANDS + 1];
char           vtxTableBandLetters[VTX_TABLE_MAX_BANDS + 1];
const char *   vtxTableChannelNames[VTX_TABLE_MAX_CHANNELS + 1];
bool           vtxTableIsFactoryBand[VTX_TABLE_MAX_BANDS];

int            vtxTablePowerLevels;
uint16_t       vtxTablePowerValues[VTX_TABLE_MAX_POWER_LEVELS];
const char *   vtxTablePowerLabels[VTX_TABLE_MAX_POWER_LEVELS + 1];

#else

int            vtxTableBandCount = VTX_TABLE_MAX_BANDS;
int            vtxTableChannelCount = VTX_TABLE_MAX_CHANNELS;
uint16_t       vtxTableFrequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS] = {
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
};
const char *   vtxTableBandNames[VTX_TABLE_MAX_BANDS + 1] = {
        "--------",
        "BOSCAM A",
        "BOSCAM B",
        "BOSCAM E",
        "FATSHARK",
        "RACEBAND",
};
char           vtxTableBandLetters[VTX_TABLE_MAX_BANDS + 1] = { '-', 'A', 'B', 'E', 'F', 'R' };
const char *   vtxTableChannelNames[VTX_TABLE_MAX_CHANNELS + 1] = {
        "-", "1", "2", "3", "4", "5", "6", "7", "8",
};
bool           vtxTableIsFactoryBand[VTX_TABLE_MAX_BANDS];
int            vtxTablePowerLevels;
uint16_t       vtxTablePowerValues[VTX_TABLE_MAX_POWER_LEVELS];
const char *   vtxTablePowerLabels[VTX_TABLE_MAX_POWER_LEVELS + 1];
#endif

void vtxTableInit(void)
{
#if defined(USE_VTX_TABLE)
    const vtxTableConfig_t *config = vtxTableConfig();

    vtxTableBandCount = config->bands;
    vtxTableChannelCount = config->channels;

    for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
        for (int channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
            vtxTableFrequency[band][channel] = config->frequency[band][channel];
        }
        vtxTableBandNames[band + 1] = config->bandNames[band];
        vtxTableBandLetters[band + 1] = config->bandLetters[band];
        vtxTableIsFactoryBand[band] = config->isFactoryBand[band];
    }

    vtxTableBandNames[0] = "--------";
    vtxTableBandLetters[0] = '-';

    for (int channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
        vtxTableChannelNames[channel + 1] = config->channelNames[channel];
    }
    vtxTableChannelNames[0] = "-";

    for (int level = 0; level < VTX_TABLE_MAX_POWER_LEVELS; level++) {
        vtxTablePowerValues[level] = config->powerValues[level];
        vtxTablePowerLabels[level + 1] = config->powerLabels[level];
    }
    vtxTablePowerLabels[0] = "---";

    vtxTablePowerLevels = config->powerLevels;
#else
    for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
        vtxTableIsFactoryBand[band] = true;
    }
    for (int powerIndex = 0; powerIndex < VTX_TABLE_MAX_POWER_LEVELS; powerIndex++) {
        vtxTablePowerValues[powerIndex] = 0;
        vtxTablePowerLabels[powerIndex] = NULL;
    }
    vtxTablePowerLevels = VTX_TABLE_MAX_POWER_LEVELS;
    vtxTableSetFactoryBands(false);
#endif
}

#ifndef USE_VTX_TABLE
void vtxTableSetFactoryBands(bool isFactory)
{
    for(int i = 0;i < VTX_TABLE_MAX_BANDS; i++) {
        vtxTableIsFactoryBand[i] = isFactory;
    }
}
#endif

#if defined(USE_VTX_TABLE)

void vtxTableStrncpyWithPad(char *dst, const char *src, int length)
{
    char c;

    while (length && (c = *src++)) {
        *dst++ = c;
        length--;
    }

    while (length--) {
        *dst++ = ' ';
    }

    *dst = 0;
}

// Prune a band to "channels"
void vtxTableConfigClearChannels(vtxTableConfig_t *config, int band, int channels)
{
    for (int channel = channels; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
        config->frequency[band][channel] = 0;
    }
}

// Clear a channel name for "channel"
static void vtxTableConfigClearChannelNames(vtxTableConfig_t *config, int channel)
{
    tfp_sprintf(config->channelNames[channel], "%d", channel + 1);
}

void vtxTableConfigClearBand(vtxTableConfig_t *config, int band)
{
    vtxTableConfigClearChannels(config, band, 0);
    for (int channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
        vtxTableConfigClearChannelNames(config, channel);
    }
    char tempbuf[6];
    tfp_sprintf(tempbuf, "BAND%d", band + 1);
    vtxTableStrncpyWithPad(config->bandNames[band], tempbuf, VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[band] = '1' + band;
    config->isFactoryBand[band] = false;
}

void vtxTableConfigClearPowerValues(vtxTableConfig_t *config, int start)
{
    for (int i = start; i < VTX_TABLE_MAX_POWER_LEVELS; i++) {
        config->powerValues[i] = 0;
    }
}

void vtxTableConfigClearPowerLabels(vtxTableConfig_t *config, int start)
{
    for (int i = start; i < VTX_TABLE_MAX_POWER_LEVELS; i++) {
        char tempbuf[4];
        tfp_sprintf(tempbuf, "LV%d", i);
        vtxTableStrncpyWithPad(config->powerLabels[i], tempbuf, VTX_TABLE_POWER_LABEL_LENGTH);
    }
}
#endif
