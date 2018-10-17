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

#define VTX_BEESIGN_MIN_BAND 1
#define VTX_BEESIGN_MAX_BAND 5
#define VTX_BEESIGN_MIN_CHANNEL 1
#define VTX_BEESIGN_MAX_CHANNEL 8

#define VTX_BEESIGN_VTX_RACE_MODE   0
#define VTX_BEESIGN_VTX_MANUAL_MODE   1
#define VTX_BEESIGN_VTX_POR_MODE   2

#define VTX_BEESIGN_MODE_COUNT  3

#define VTX_BEESIGN_BAND_COUNT (VTX_BEESIGN_MAX_BAND - VTX_BEESIGN_MIN_BAND + 1)
#define VTX_BEESIGN_CHANNEL_COUNT (VTX_BEESIGN_MAX_CHANNEL - VTX_BEESIGN_MIN_CHANNEL + 1)
#define VTX_BEESIGN_ERROR_CHANNEL       0xff

#define VTX_BEESIGN_POWER_COUNT         6
#define VTX_BEESIGN_DEFAULT_POWER       1
#define VTX_BEESIGN_MIN_POWER           1

#define VTX_BEESIGN_POR_FREQUENCY_MHZ 5584

#define VTX_BEESIGN_MIN_FREQUENCY_MHZ 5000        //min freq in MHz
#define VTX_BEESIGN_MAX_FREQUENCY_MHZ 5999        //max freq in MHz

#define BEESIGN_HDR             0xAB
#define BEESIGN_FM_MAX_LEN      64      // frame max length
#define BEESIGN_PL_MAX_LEN      60      // payload max length

#define BEESIGN_BAUD_RATE       115200

#define BEESIGN_OK              0
#define BEESIGN_ERROR           1

#define BEESIGN_TYPE_MASK_FM    0x80    // Bit - 7
#define BEESIGN_TYPE_MASK_CMD   0x7F    // Bit - 1 ~ 6

#define BEESIGN_FM_TYPE_CMD     0       // command frame
#define BEESIGN_FM_TYPE_RSP     1       // response frame

#define BEESIGN_PARA_DISP_OFF   0
#define BEESIGN_PARA_DISP_ON    1

#define BEESIGN_VOL_SRC_ADC     1
#define BEESIGN_VOL_SRC_EXT     2
#define BEESIGN_RSSI_SRC_ADC    1
#define BEESIGN_RSSI_SRC_EXT    2
#define BEESIGN_RSSI_SRC_RX     3

#define BEESIGN_VTX_MODE_RACE   0
#define BEESIGN_VTX_MODE_MANU   1
#define BEESIGN_VTX_MODE_POR    2

#define BEESIGN_VTX_UNLOCK      0x403C
#define BEESIGN_VTX_LOCK        0x403D
#define BEESIGN_VTX_MAX_CHAN    40
#define BEESIGN_VTX_POR_FREQ    5584

#define BEESIGN_OSD_MODE_OFF        0
#define BEESIGN_OSD_MODE_MINI       1
#define BEESIGN_OSD_MODE_CUSTOM     2

// VTX commands
#define BEESIGN_V_GET_STATUS    0x10
#define BEESIGN_V_SET_CHAN      0x11
#define BEESIGN_V_SET_FREQ      0x12
#define BEESIGN_V_SET_PWR       0x13
#define BEESIGN_V_SET_MODE      0x14
#define BEESIGN_V_UNLOCK        0x15
#define BEESIGN_V_LOCK          0x16

// OSD commands
#define BEESIGN_O_GET_STATUS    0x20
#define BEESIGN_O_SET_MODE      0x21
#define BEESIGN_O_SET_LAYOUT    0x22
#define BEESIGN_O_SET_HOS       0x23
#define BEESIGN_O_SET_VOS       0x24
#define BEESIGN_O_SET_DISPLAY   0x25
#define BEESIGN_O_CLR_DISPLAY   0x26
#define BEESIGN_O_UDT_FONT      0x27

#define BEESIGN_O_FONT_UNLOCK   0x29

// Audio commands
#define BEESIGN_A_GET_STATUS    0x30
#define BEESIGN_A_SET_VOL       0x31
#define BEESIGN_A_SET_RSSI      0x32

// Other commands
#define BEESIGN_S_GET_STATUS    0x70
#define BEESIGN_S_SET_VOL       0x71
#define BEESIGN_S_SET_RSSI      0x72
#define BEESIGN_M_SET_NAME      0x73
#define BEESIGN_M_SET_VOL       0x74
#define BEESIGN_M_SET_CUR       0x75
#define BEESIGN_M_SET_RSSI      0x76
#define BEESIGN_M_SAVE_SETTING  0x77


#define BEESIGN_PKT_TYPE(x)     (x)[1]
#define BEESIGN_PKT_LEN(x)      (x)[2]
#define BEESIGN_PKT_DATA(x, i)  (x)[3 + (i)]

typedef enum VTX_POWER_E {
    VTX_PWR_25 = 0,
    VTX_PWR_100,
    VTX_PWR_200,
    VTX_PWR_400,
    VTX_PWR_600,
    VTX_PWR_PIT
} vtxPower_e;

typedef struct beeSignDevice_s {
    uint8_t version;
    uint8_t channel;
    uint8_t power;
    uint8_t lastPower;
    uint8_t mode;
    uint16_t freq;
    uint16_t porfreq;
} beeSignDevice_t;

/***********************************************
 * The data frame have five parts: header, type, length, payload and CRC, and the maxium length is 64 bytes.
 * The header is fixed to 0xAB; the type segment Bit 7 of the TYPE byte is frame type (0 = command frame,
 * 1 = response frame), bit 0~6 is ID; And the LENGTH is the length of the payload (valid range is 1~60).
 * The frame format is shown in the following table.
 *
 * |------------------------------------------------------------------------------------------|
 * | header (1 byte) | type (1 byte) | length (1 byte) | payload (<= 60 bytes) | CRC (1 byte) |
 * |------------------------------------------------------------------------------------------|
 *
************************************************/
typedef struct BEESIGN_FRAME_T {
    uint8_t hdr;
    uint8_t type;
    uint8_t len;
    uint8_t *payload;
    uint8_t crc;
} beesign_frame_t;

extern const char * const bsPowerNames[];
extern const char * const bsModeNames[];
extern const uint16_t bsPowerTable[];
extern beeSignDevice_t bsDevice;


void beesignProcess(timeUs_t currentTimeUs);
bool beesignInit(void);

void bsSetVTxUnlock(void);
void bsSetVTxLock(void);


void bsSetBandAndChannel(uint8_t band, uint8_t channel);
void bsSetPower(uint8_t index);
void bsSetMode(uint8_t mode);
void bsSetFreq(uint16_t freq);
