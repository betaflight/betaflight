#include "board.h"
#include "mw.h"

// driver for spektrum satellite receiver / sbus using UART2 (freeing up more motor outputs for stuff)

#define SPEK_MAX_CHANNEL 7
#define SPEK_FRAME_SIZE 16
static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;
static bool spekDataIncoming = false;
volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
static void spektrumDataReceive(uint16_t c);
static uint16_t spektrumReadRawRC(uint8_t chan);

// external vars (ugh)
extern int16_t failsafeCnt;

void spektrumInit(rcReadRawDataPtr *callback)
{
    switch (mcfg.serialrx_type) {
        case SERIALRX_SPEKTRUM2048:
            // 11 bit frames
            spek_chan_shift = 3;
            spek_chan_mask = 0x07;
            spekHiRes = true;
            break;
        case SERIALRX_SPEKTRUM1024:
            // 10 bit frames
            spek_chan_shift = 2;
            spek_chan_mask = 0x03;
            spekHiRes = false;
            break;
    }

    core.rcvrport = uartOpen(USART2, spektrumDataReceive, 115200, MODE_RX);
    if (callback)
        *callback = spektrumReadRawRC;
    core.numRCChannels = SPEK_MAX_CHANNEL;
}

// Receive ISR callback
static void spektrumDataReceive(uint16_t c)
{
    uint32_t spekTime;
    static uint32_t spekTimeLast, spekTimeInterval;
    static uint8_t  spekFramePosition;

    spekDataIncoming = true;
    spekTime = micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;
    if (spekTimeInterval > 5000)
        spekFramePosition = 0;
    spekFrame[spekFramePosition] = (uint8_t)c;
    if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
        rcFrameComplete = true;
        failsafeCnt = 0;   // clear FailSafe counter
    } else {
        spekFramePosition++;
    }
}

bool spektrumFrameComplete(void)
{
    return rcFrameComplete;
}

static uint16_t spektrumReadRawRC(uint8_t chan)
{
    uint16_t data;
    static uint32_t spekChannelData[SPEK_MAX_CHANNEL];
    uint8_t b;

    if (rcFrameComplete) {
        for (b = 3; b < SPEK_FRAME_SIZE; b += 2) {
            uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
            if (spekChannel < SPEK_MAX_CHANNEL)
                spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
        }
        rcFrameComplete = false;
    }

    if (chan >= SPEK_MAX_CHANNEL || !spekDataIncoming) {
        data = mcfg.midrc;
    } else {
        if (spekHiRes)
            data = 988 + (spekChannelData[mcfg.rcmap[chan]] >> 1);   // 2048 mode
        else
            data = 988 + spekChannelData[mcfg.rcmap[chan]];          // 1024 mode
    }

    return data;
}
