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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "drivers/system.h"

#include "drivers/gpio.h"
#include "drivers/inverter.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/sbus.h"

#define SBUS_MAX_CHANNEL 12
#define SBUS_FRAME_SIZE 25
#define SBUS_SYNCBYTE 0x0F
#define SBUS_OFFSET 988

#define SBUS_BAUDRATE 100000

static bool sbusFrameDone = false;
static void sbusDataReceive(uint16_t c);
static uint16_t sbusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

static uint32_t sbusChannelData[SBUS_MAX_CHANNEL];

static serialPort_t *sBusPort;

void sbusUpdateSerialRxFunctionConstraint(functionConstraint_t *functionConstraint)
{
    functionConstraint->minBaudRate = SBUS_BAUDRATE;
    functionConstraint->maxBaudRate = SBUS_BAUDRATE;
    functionConstraint->requiredSerialPortFeatures = SPF_SUPPORTS_CALLBACK | SPF_SUPPORTS_SBUS_MODE;
}

bool sbusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    int b;

    sBusPort = openSerialPort(FUNCTION_SERIAL_RX, sbusDataReceive, SBUS_BAUDRATE, (portMode_t)(MODE_RX | MODE_SBUS), SERIAL_INVERTED);

    for (b = 0; b < SBUS_MAX_CHANNEL; b++)
        sbusChannelData[b] = 2 * (rxConfig->midrc - SBUS_OFFSET);
    if (callback)
        *callback = sbusReadRawRC;
    rxRuntimeConfig->channelCount = SBUS_MAX_CHANNEL;

    return sBusPort != NULL;
}

struct sbus_dat {
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
} __attribute__ ((__packed__));

typedef union {
    uint8_t in[SBUS_FRAME_SIZE];
    struct sbus_dat msg;
} sbus_msg;

static sbus_msg sbus;

// Receive ISR callback
static void sbusDataReceive(uint16_t c)
{
    uint32_t sbusTime;
    static uint32_t sbusTimeLast;
    static uint8_t sbusFramePosition;

    sbusTime = micros();
    if ((sbusTime - sbusTimeLast) > 2500) // sbus2 fast timing
        sbusFramePosition = 0;
    sbusTimeLast = sbusTime;

    if (sbusFramePosition == 0 && c != SBUS_SYNCBYTE)
        return;

    sbusFrameDone = false; // lazy main loop didnt fetch the stuff
    if (sbusFramePosition != 0)
        sbus.in[sbusFramePosition - 1] = (uint8_t)c;

    if (sbusFramePosition == SBUS_FRAME_SIZE - 1) {
        sbusFrameDone = true;
        sbusFramePosition = 0;
    } else {
        sbusFramePosition++;
    }
}

bool sbusFrameComplete(void)
{
    if (!sbusFrameDone) {
        return false;
    }
    sbusFrameDone = false;
    if ((sbus.in[22] >> 3) & 0x0001) {
        // internal failsafe enabled and rx failsafe flag set
        return false;
    }
    sbusChannelData[0] = sbus.msg.chan0;
    sbusChannelData[1] = sbus.msg.chan1;
    sbusChannelData[2] = sbus.msg.chan2;
    sbusChannelData[3] = sbus.msg.chan3;
    sbusChannelData[4] = sbus.msg.chan4;
    sbusChannelData[5] = sbus.msg.chan5;
    sbusChannelData[6] = sbus.msg.chan6;
    sbusChannelData[7] = sbus.msg.chan7;
    sbusChannelData[8] = sbus.msg.chan8;
    sbusChannelData[9] = sbus.msg.chan9;
    sbusChannelData[10] = sbus.msg.chan10;
    sbusChannelData[11] = sbus.msg.chan11;
    return true;
}

static uint16_t sbusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return sbusChannelData[chan] / 2 + SBUS_OFFSET;
}

