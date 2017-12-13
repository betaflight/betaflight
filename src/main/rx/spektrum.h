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

#define SPEKTRUM_SAT_BIND_DISABLED     0
#define SPEKTRUM_SAT_BIND_MAX         10

#define SPEK_FRAME_SIZE             16
#define SRXL_FRAME_OVERHEAD         5
#define SRXL_FRAME_SIZE_MAX         (SPEK_FRAME_SIZE + SRXL_FRAME_OVERHEAD)

// Spektrum system type values
#define SPEKTRUM_DSM2_22 0x01
#define SPEKTRUM_DSM2_11 0x12
#define SPEKTRUM_DSMX_22 0xa2
#define SPEKTRUM_DSMX_11 0xb2

// Spektrum RSSI signal strength range, in dBm
#define SPEKTRUM_RSSI_MAX         (-42)
#define SPEKTRUM_RSSI_MIN         (-92)

// Spektrum RSSI reported value limit at or below which signals a fade instead of a real RSSI
#define SPEKTRUM_RSSI_FADE_LIMIT (-100)

typedef struct
{
  int8_t  dBm;
  uint8_t reportAs;
} stru_dbm_table;


//VTX control frame bits and pieces
#define SPEKTRUM_VTX_CONTROL_FRAME_MASK 0xf000f000
#define SPEKTRUM_VTX_CONTROL_FRAME      0xe000e000

#define SPEKTRUM_VTX_CONTROL_1          (SPEK_FRAME_SIZE - 4)
#define SPEKTRUM_VTX_CONTROL_2          (SPEK_FRAME_SIZE - 3)
#define SPEKTRUM_VTX_CONTROL_3          (SPEK_FRAME_SIZE - 2)
#define SPEKTRUM_VTX_CONTROL_4          (SPEK_FRAME_SIZE - 1)

#define SPEKTRUM_VTX_BAND_MASK          0x00e00000
#define SPEKTRUM_VTX_CHANNEL_MASK       0x000f0000
#define SPEKTRUM_VTX_PIT_MODE_MASK      0x00000010
#define SPEKTRUM_VTX_REGION_MASK        0x00000008
#define SPEKTRUM_VTX_POWER_MASK         0x00000007

#define SPEKTRUM_VTX_BAND_SHIFT         21
#define SPEKTRUM_VTX_CHANNEL_SHIFT      16
#define SPEKTRUM_VTX_PIT_MODE_SHIFT     4
#define SPEKTRUM_VTX_REGION_SHIFT       3
#define SPEKTRUM_VTX_POWER_SHIFT        0

#define SPEKTRUM_VTX_BAND_COUNT  5
#define SPEKTRUM_VTX_CHAN_COUNT  8

/*
Channels vs Band according to spektrum spec.
   0    1    2    3    4    5    6    7
5740 5760 5780 5800 5820 5840 5860 5880 FatShark
5658 5695 5732 5769 5806 5843 5880 5917 Race
5705 5685 5665 5645 5885 5905 5925 5945 Band E
5733 5752 5771 5790 5809 5828 5847 5866 Band B
5865 5845 5825 5805 5785 5765 5745 5725 Band A
*/

// Band translation to BF internal vtx_common needed
// Spektrum order, zero based.
#define SPEKTRUM_VTX_BAND_FS      0
#define SPEKTRUM_VTX_BAND_RACE    1
#define SPEKTRUM_VTX_BAND_E       2
#define SPEKTRUM_VTX_BAND_B       3
#define SPEKTRUM_VTX_BAND_A       4

// Spektrum Max power index
#define SPEKTRUM_VTX_POWER_OFF    0
#define SPEKTRUM_VTX_POWER_14     1
#define SPEKTRUM_VTX_POWER_25     2
#define SPEKTRUM_VTX_POWER_99     3
#define SPEKTRUM_VTX_POWER_299    4
#define SPEKTRUM_VTX_POWER_600    5
#define SPEKTRUM_VTX_POWER_MAXIT  6
#define SPEKTRUM_VTX_POWER_MAN    7

#define SPEKTRUM_VTX_REGION_USA   0
#define SPEKTRUM_VTX_REGION_EU    1

#define SPEKTRUM_VTX_PITMODE_OFF  0 // Power on, race
#define SPEKTRUM_VTX_PITMODE_ON   1 // Power off, pit

typedef struct
{
  uint8_t band;
  uint8_t channel;
  uint8_t power;
  uint8_t region;
  uint8_t pitMode;
} stru_vtx;

void spektrumBind(rxConfig_t *rxConfig);
bool spektrumInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void srxlRxWriteTelemetryData(const void *data, int len);
bool srxlRxIsActive(void);
