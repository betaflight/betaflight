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

#include "drivers/gpio.h"
#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "config/config.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

// driver for spektrum satellite receiver / sbus

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_1024_CHANNEL_COUNT 7

#define SPEK_FRAME_SIZE 16

#define SPEKTRUM_BAUDRATE 115200

static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;
static bool spekDataIncoming = false;

static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

static void spektrumDataReceive(uint16_t c);
static uint16_t spektrumReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

static serialPort_t *spektrumPort;

void spektrumUpdateSerialRxFunctionConstraint(functionConstraint_t *functionConstraint)
{
    functionConstraint->minBaudRate = SPEKTRUM_BAUDRATE;
    functionConstraint->maxBaudRate = SPEKTRUM_BAUDRATE;
    functionConstraint->requiredSerialPortFeatures = SPF_SUPPORTS_CALLBACK;
}

bool spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    switch (rxConfig->serialrx_provider) {
        case SERIALRX_SPEKTRUM2048:
            // 11 bit frames
            spek_chan_shift = 3;
            spek_chan_mask = 0x07;
            spekHiRes = true;
            rxRuntimeConfig->channelCount = SPEKTRUM_2048_CHANNEL_COUNT;
            break;
        case SERIALRX_SPEKTRUM1024:
            // 10 bit frames
            spek_chan_shift = 2;
            spek_chan_mask = 0x03;
            spekHiRes = false;
            rxRuntimeConfig->channelCount = SPEKTRUM_1024_CHANNEL_COUNT;
            break;
    }

    spektrumPort = openSerialPort(FUNCTION_SERIAL_RX, spektrumDataReceive, SPEKTRUM_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
    if (callback)
        *callback = spektrumReadRawRC;

    return spektrumPort != NULL;
}

// Receive ISR callback
static void spektrumDataReceive(uint16_t c)
{
    uint32_t spekTime;
    static uint32_t spekTimeLast, spekTimeInterval;
    static uint8_t spekFramePosition;

    spekDataIncoming = true;
    spekTime = micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;
    if (spekTimeInterval > 5000)
        spekFramePosition = 0;
    spekFrame[spekFramePosition] = (uint8_t)c;
    if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
        rcFrameComplete = true;
    } else {
        spekFramePosition++;
    }
}

bool spektrumFrameComplete(void)
{
    return rcFrameComplete;
}

static uint16_t spektrumReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;
    static uint32_t spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];
    uint8_t b;

    if (rcFrameComplete) {
        for (b = 3; b < SPEK_FRAME_SIZE; b += 2) {
            uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
            if (spekChannel < rxRuntimeConfig->channelCount && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT)
                spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
        }
        rcFrameComplete = false;
    }

    if (chan >= rxRuntimeConfig->channelCount || !spekDataIncoming) {
        return 0;
    }

    if (spekHiRes)
        data = 988 + (spekChannelData[chan] >> 1);   // 2048 mode
    else
        data = 988 + spekChannelData[chan];          // 1024 mode

    return data;
}

#ifdef SPEKTRUM_BIND
/* spektrumBind function ported from Baseflight. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
void spektrumBind(rxConfig_t *rxConfig)
{
    int i;
    gpio_config_t gpio;
    GPIO_TypeDef *spekBindPort;
    uint16_t spekBindPin;

#ifdef HARDWARE_BIND_PLUG
    // Check status of bind plug and exit if not active
    GPIO_TypeDef *hwBindPort;
    uint16_t hwBindPin;

    hwBindPort = BINDPLUG_PORT;
    hwBindPin = BINDPLUG_PIN;
    gpio.speed = Speed_2MHz;
    gpio.pin = hwBindPin;
    gpio.mode = Mode_IPU;
    gpioInit(hwBindPort, &gpio);
    delayMicroseconds(10);  // allow configuration to settle
    if (digitalIn(hwBindPort, hwBindPin))
        return;
#endif

    spekBindPort = BIND_PORT;
    spekBindPin = BIND_PIN;

    // don't try to bind if: here after soft reset or bind flag is out of range
    if (isMPUSoftReset() || rxConfig->spektrum_sat_bind == 0 || rxConfig->spektrum_sat_bind > 10)
        return;

    gpio.speed = Speed_2MHz;
    gpio.pin = spekBindPin;
    gpio.mode = Mode_Out_OD;
    gpioInit(spekBindPort, &gpio);
    // RX line, set high
    digitalHi(spekBindPort, spekBindPin);
    // Bind window is around 20-140ms after powerup
    delay(60);

    for (i = 0; i < rxConfig->spektrum_sat_bind; i++) {
        // RX line, drive low for 120us
        digitalLo(spekBindPort, spekBindPin);
        delayMicroseconds(120);
        // RX line, drive high for 120us
        digitalHi(spekBindPort, spekBindPin);
        delayMicroseconds(120);
    }

#ifndef HARDWARE_BIND_PLUG
    // If we came here as a result of hard  reset (power up, with mcfg.spektrum_sat_bind set), then reset it back to zero and write config
    // Don't reset if hardware bind plug is present
    if (!isMPUSoftReset()) {
        rxConfig->spektrum_sat_bind = 0;
        writeEEPROM(1, true);
    }
#endif

}
#endif
