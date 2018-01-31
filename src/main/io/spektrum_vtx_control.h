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



//VTX control frame bits and pieces
#define SPEKTRUM_VTX_CONTROL_FRAME_MASK 0xf000f000
#define SPEKTRUM_VTX_CONTROL_FRAME      0xe000e000

#define SPEKTRUM_VTX_CONTROL_1          (SPEK_FRAME_SIZE - 4)
#define SPEKTRUM_VTX_CONTROL_2          (SPEK_FRAME_SIZE - 3)
#define SPEKTRUM_VTX_CONTROL_3          (SPEK_FRAME_SIZE - 2)
#define SPEKTRUM_VTX_CONTROL_4          (SPEK_FRAME_SIZE - 1)
#define SPEKTRUM_VTX_CONTROL_SIZE       4

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
#define SPEKTRUM_VTX_BAND_COUNT   5

// Spektrum Max power index
#define SPEKTRUM_VTX_POWER_OFF    0
#define SPEKTRUM_VTX_POWER_14     1
#define SPEKTRUM_VTX_POWER_25     2
#define SPEKTRUM_VTX_POWER_99     3
#define SPEKTRUM_VTX_POWER_299    4
#define SPEKTRUM_VTX_POWER_600    5
#define SPEKTRUM_VTX_POWER_MAXIT  6
#define SPEKTRUM_VTX_POWER_MAN    7
#define SPEKTRUM_VTX_POWER_COUNT  8

#define SPEKTRUM_VTX_REGION_USA   0
#define SPEKTRUM_VTX_REGION_EU    1
#define SPEKTRUM_VTX_REGION_NONE  0xff

#define SPEKTRUM_VTX_PITMODE_OFF  0 // Power on, race
#define SPEKTRUM_VTX_PITMODE_ON   1 // Power off, pit

typedef struct
{
    uint8_t band;
    uint8_t channel;
    uint8_t power;
    uint8_t region;
    uint8_t pitMode;
    uint16_t powerValue;
} spektrumVtx_t;


extern const uint16_t SpektrumVtxfrequencyTable[SPEKTRUM_VTX_BAND_COUNT][SPEKTRUM_VTX_CHAN_COUNT];
extern const uint8_t spek2commonBand[SPEKTRUM_VTX_BAND_COUNT];
extern const uint8_t vtxTrampPi[SPEKTRUM_VTX_POWER_COUNT];
extern const uint8_t vtxRTC6705Pi[SPEKTRUM_VTX_POWER_COUNT];
extern const uint8_t vtxSaPi[SPEKTRUM_VTX_POWER_COUNT];
extern uint8_t SpektrumRegion;

void spektrumHandleVtxControl(uint32_t vtxControl);
void spektrumVtxControl(void);



