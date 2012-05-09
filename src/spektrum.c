#include "board.h"
#include "mw.h"

// driver for spektrum satellite receiver / sbus using UART2 (freeing up more motor outputs for stuff)

#define SPEK_MAX_CHANNEL 7
#define SPEK_FRAME_SIZE 16
static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekDataIncoming = false;
volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
static void spektrumDataReceive(uint16_t c);

// external vars (ugh)
extern int16_t failsafeCnt;

void spektrumInit(void)
{
    if (cfg.spektrum_hires) {
        // 11 bit frames
        spek_chan_shift = 3;
        spek_chan_mask = 0x07;
    } else {
        // 10 bit frames
        spek_chan_shift = 2;
        spek_chan_mask = 0x03;
    }

    uart2Init(115200, spektrumDataReceive);
}

// UART2 Receive ISR callback
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

// static const uint8_t spekRcChannelMap[SPEK_MAX_CHANNEL] = {1, 2, 3, 0, 4, 5, 6};

uint16_t spektrumReadRawRC(uint8_t chan)
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
        data = cfg.midrc;
    } else {
        if (cfg.spektrum_hires)
            data = 988 + (spekChannelData[cfg.rcmap[chan]] >> 1);   // 2048 mode
        else
            data = 988 + spekChannelData[cfg.rcmap[chan]];          // 1024 mode
    }
    
    return data;
}
