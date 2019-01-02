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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#define VTX_SMARTAUDIO_MIN_BAND 1
#define VTX_SMARTAUDIO_MAX_BAND 5
#define VTX_SMARTAUDIO_MIN_CHANNEL 1
#define VTX_SMARTAUDIO_MAX_CHANNEL 8

#define VTX_SMARTAUDIO_BAND_COUNT (VTX_SMARTAUDIO_MAX_BAND - VTX_SMARTAUDIO_MIN_BAND + 1)
#define VTX_SMARTAUDIO_CHANNEL_COUNT (VTX_SMARTAUDIO_MAX_CHANNEL - VTX_SMARTAUDIO_MIN_CHANNEL + 1)

#define VTX_SMARTAUDIO_POWER_COUNT 4
#define VTX_SMARTAUDIO_DEFAULT_POWER 1

#define VTX_SMARTAUDIO_MIN_FREQUENCY_MHZ 5000        //min freq in MHz
#define VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ 5999        //max freq in MHz

// opmode flags, GET side
#define SA_MODE_GET_FREQ_BY_FREQ            1
#define SA_MODE_GET_PITMODE                 2
#define SA_MODE_GET_IN_RANGE_PITMODE        4
#define SA_MODE_GET_OUT_RANGE_PITMODE       8
#define SA_MODE_GET_UNLOCK                 16
#define SA_MODE_GET_DEFERRED_FREQ          32

// opmode flags, SET side
#define SA_MODE_SET_IN_RANGE_PITMODE        1 // Immediate
#define SA_MODE_SET_OUT_RANGE_PITMODE        2 // Immediate
#define SA_MODE_CLR_PITMODE                 4 // Immediate
#define SA_MODE_SET_UNLOCK                  8
#define SA_MODE_SET_LOCK                    0 // ~UNLOCK
#define SA_MODE_SET_DEFERRED_FREQ          16

// SetFrequency flags, for pit mode frequency manipulation
#define SA_FREQ_GETPIT                      (1 << 14)
#define SA_FREQ_SETPIT                      (1 << 15)
#define SA_FREQ_MASK                        (~(SA_FREQ_GETPIT|SA_FREQ_SETPIT))

// For generic API use, but here for now

typedef struct smartAudioDevice_s {
    int8_t version;
    int8_t channel;
    int8_t power;
    int8_t mode;
    uint16_t freq;
    uint16_t orfreq;
} smartAudioDevice_t;

typedef struct saPowerTable_s {
    int rfpower;
    int16_t valueV1;
    int16_t valueV2;
} saPowerTable_t;

typedef struct smartAudioStat_s {
    uint16_t pktsent;
    uint16_t pktrcvd;
    uint16_t badpre;
    uint16_t badlen;
    uint16_t crc;
    uint16_t ooopresp;
    uint16_t badcode;
} smartAudioStat_t;

extern smartAudioDevice_t saDevice;
extern saPowerTable_t saPowerTable[];
extern const char * const saPowerNames[];
extern smartAudioStat_t saStat;

extern uint16_t sa_smartbaud;
extern bool saDeferred;

int saDacToPowerIndex(int dac);
void saSetBandAndChannel(uint8_t band, uint8_t channel);
void saSetMode(int mode);
void saSetPowerByIndex(uint8_t index);
void saSetFreq(uint16_t freq);
void saSetPitFreq(uint16_t freq);
bool vtxSmartAudioInit(void);
