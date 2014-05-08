#include "board.h"
#include "mw.h"

// driver for SUMD receiver using UART2

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 8
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5) // 6 channels + 5 -> 17 bytes for 6 channels

static bool sumdFrameDone = false;
static void sumdDataReceive(uint16_t c);
static uint16_t sumdReadRawRC(uint8_t chan);

static uint32_t sumdChannelData[SUMD_MAX_CHANNEL];

void sumdInit(rcReadRawDataPtr *callback)
{
    core.rcvrport = uartOpen(USART2, sumdDataReceive, 115200, MODE_RX);
    if (callback)
        *callback = sumdReadRawRC;
}

static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdSize;

// Receive ISR callback
static void sumdDataReceive(uint16_t c)
{
    uint32_t sumdTime;
    static uint32_t sumdTimeLast;
    static uint8_t sumdIndex;

    sumdTime = micros();
    if ((sumdTime - sumdTimeLast) > 4000)
        sumdIndex = 0;
    sumdTimeLast = sumdTime;

    if (sumdIndex == 0) {
        if (c != SUMD_SYNCBYTE)
            return;
        else
            sumdFrameDone = false; // lazy main loop didnt fetch the stuff
    }
    if (sumdIndex == 2)
        sumdSize = (uint8_t)c;
    if (sumdIndex < SUMD_BUFFSIZE)
        sumd[sumdIndex] = (uint8_t)c;
    sumdIndex++;
    if (sumdIndex == sumdSize * 2 + 5) {
        sumdIndex = 0;
        sumdFrameDone = true;
    }
}

bool sumdFrameComplete(void)
{
    uint8_t b;

    if (sumdFrameDone) {
        sumdFrameDone = false;
        if (sumd[1] == 0x01) {
            failsafeCnt = 0;
            if (sumdSize > SUMD_MAX_CHANNEL)
                sumdSize = SUMD_MAX_CHANNEL;
            for (b = 0; b < sumdSize; b++)
                sumdChannelData[b] = ((sumd[2 * b + 3] << 8) | sumd[2 * b + 4]);
            return true;
        }
    }
    return false;
}

static uint16_t sumdReadRawRC(uint8_t chan)
{
    return sumdChannelData[mcfg.rcmap[chan]] / 8;
}
