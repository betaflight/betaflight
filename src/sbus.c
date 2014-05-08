#include "board.h"
#include "mw.h"

// driver for SBUS receiver using UART2

#define SBUS_MAX_CHANNEL 8
#define SBUS_FRAME_SIZE 25
#define SBUS_SYNCBYTE 0x0F
#define SBUS_OFFSET 988

static bool sbusFrameDone = false;
static void sbusDataReceive(uint16_t c);
static uint16_t sbusReadRawRC(uint8_t chan);

// external vars (ugh)
extern int16_t failsafeCnt;

static uint32_t sbusChannelData[SBUS_MAX_CHANNEL];

void sbusInit(rcReadRawDataPtr *callback)
{
    int b;
    for (b = 0; b < SBUS_MAX_CHANNEL; b++)
        sbusChannelData[b] = 2 * (mcfg.midrc - SBUS_OFFSET);
    core.rcvrport = uartOpen(USART2, sbusDataReceive, 100000, (portMode_t)(MODE_RX | MODE_SBUS));
    if (callback)
        *callback = sbusReadRawRC;
    core.numRCChannels = SBUS_MAX_CHANNEL;
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
    if (sbusFrameDone) {
        if (!((sbus.in[22] >> 3) & 0x0001)) { // failsave flag
            failsafeCnt = 0; // clear FailSafe counter
            sbusChannelData[0] = sbus.msg.chan0;
            sbusChannelData[1] = sbus.msg.chan1;
            sbusChannelData[2] = sbus.msg.chan2;
            sbusChannelData[3] = sbus.msg.chan3;
            sbusChannelData[4] = sbus.msg.chan4;
            sbusChannelData[5] = sbus.msg.chan5;
            sbusChannelData[6] = sbus.msg.chan6;
            sbusChannelData[7] = sbus.msg.chan7;
            // need more channels? No problem. Add them.
            sbusFrameDone = false;
            return true;
        }
        sbusFrameDone = false;
    }
    return false;
}

static uint16_t sbusReadRawRC(uint8_t chan)
{
    return sbusChannelData[mcfg.rcmap[chan]] / 2 + SBUS_OFFSET;
}
