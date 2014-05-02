#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system_common.h"

#include "drivers/serial_common.h"
#include "drivers/serial_uart_common.h"
#include "serial_common.h"

#include "failsafe.h"

#include "rx_common.h"
#include "rx_spektrum.h"

// driver for spektrum satellite receiver / sbus using UART2 (freeing up more motor outputs for stuff)

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_1024_CHANNEL_COUNT 7

#define SPEK_FRAME_SIZE 16

static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;
static bool spekDataIncoming = false;

volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

static void spektrumDataReceive(uint16_t c);
static uint16_t spektrumReadRawRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

failsafe_t *failsafe;

void spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback)
{
    failsafe = initialFailsafe;

    switch (rxConfig->serialrx_type) {
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

    serialPorts.rcvrport = uartOpen(USART2, spektrumDataReceive, 115200, MODE_RX);
    if (callback)
        *callback = spektrumReadRawRC;
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
        failsafe->vTable->reset();
    } else {
        spekFramePosition++;
    }
}

bool spektrumFrameComplete(void)
{
    return rcFrameComplete;
}

static uint16_t spektrumReadRawRC(rxConfig_t*rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
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
        data = rxConfig->midrc;
    } else {
        if (spekHiRes)
            data = 988 + (spekChannelData[rxConfig->rcmap[chan]] >> 1);   // 2048 mode
        else
            data = 988 + spekChannelData[rxConfig->rcmap[chan]];          // 1024 mode
    }

    return data;
}
