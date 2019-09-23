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

#include <stdint.h>

#include "platform.h"

#include "drivers/vtx_common.h"

#ifdef USE_VTX_TABLE
#define VTX_TABLE_MAX_BANDS             8 // Maximum number of bands
#define VTX_TABLE_MAX_CHANNELS          8 // Maximum number of channels per band
#define VTX_TABLE_MAX_POWER_LEVELS      8 // Maximum number of power levels
#define VTX_TABLE_CHANNEL_NAME_LENGTH   1
#define VTX_TABLE_BAND_NAME_LENGTH      8
#define VTX_TABLE_POWER_LABEL_LENGTH    3
#else
#define VTX_TABLE_MAX_BANDS             5 // default freq table has 5 bands
#define VTX_TABLE_MAX_CHANNELS          8 // and eight channels
#define VTX_TABLE_MAX_POWER_LEVELS      5 //max of VTX_TRAMP_POWER_COUNT, VTX_SMARTAUDIO_POWER_COUNT and VTX_RTC6705_POWER_COUNT
#define VTX_TABLE_CHANNEL_NAME_LENGTH   1
#define VTX_TABLE_BAND_NAME_LENGTH      8
#define VTX_TABLE_POWER_LABEL_LENGTH    3
#endif


#define VTX_TABLE_MIN_USER_FREQ         5000
#define VTX_TABLE_MAX_USER_FREQ         5999
#define VTX_TABLE_DEFAULT_BAND          4
#define VTX_TABLE_DEFAULT_CHANNEL       1
#define VTX_TABLE_DEFAULT_FREQ          5740
#define VTX_TABLE_DEFAULT_PITMODE_FREQ  0
#define VTX_TABLE_DEFAULT_POWER         1 //1-based indexing. 0 means unknown and 1 is the lowest actual power mode

struct vtxTableConfig_s;
void vtxTableInit(void);
void vtxTableStrncpyWithPad(char *dst, const char *src, int length);
void vtxTableConfigClearBand(struct vtxTableConfig_s *config, int band);
void vtxTableConfigClearPowerValues(struct vtxTableConfig_s *config, int start);
void vtxTableConfigClearPowerLabels(struct vtxTableConfig_s *config, int start);
void vtxTableConfigClearChannels(struct vtxTableConfig_s *config, int band, int channels);
#ifndef USE_VTX_TABLE
void vtxTableSetFactoryBands(bool isFactory);
#endif

extern int            vtxTableBandCount;
extern int            vtxTableChannelCount;
extern uint16_t       vtxTableFrequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS];
extern const char    *vtxTableBandNames[VTX_TABLE_MAX_BANDS + 1];
extern char           vtxTableBandLetters[VTX_TABLE_MAX_BANDS + 1];
extern const char    *vtxTableChannelNames[VTX_TABLE_MAX_CHANNELS + 1];
extern bool           vtxTableIsFactoryBand[VTX_TABLE_MAX_BANDS];
extern int            vtxTablePowerLevels;
extern uint16_t       vtxTablePowerValues[VTX_TABLE_MAX_POWER_LEVELS];
extern const char    *vtxTablePowerLabels[VTX_TABLE_MAX_POWER_LEVELS + 1];
