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

#ifdef USE_SMARTAUDIO_DPRINTF
#include "io/serial.h"
#include "common/printf.h"
#include "common/printf_serial.h"
#endif

#define VTX_SMARTAUDIO_MIN_BAND 1
#define VTX_SMARTAUDIO_MIN_CHANNEL 1


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
    bool willBootIntoPitMode;
} smartAudioDevice_t;

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
extern smartAudioStat_t saStat;

extern uint16_t sa_smartbaud;
extern bool saDeferred;

void saSetMode(int mode);
void saSetFreq(uint16_t freq);
void saSetPitFreq(uint16_t freq);
bool vtxSmartAudioInit(void);

#ifdef USE_SMARTAUDIO_DPRINTF
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART3
extern serialPort_t *debugSerialPort;
#define dprintf(x) if (debugSerialPort) tfp_printf x
#else
#define dprintf(x)
#endif // USE_SMARTAUDIO_DPRINTF
