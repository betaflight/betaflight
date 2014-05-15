#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/system_common.h"

#include "drivers/serial_common.h"
#include "drivers/serial_uart_common.h"
#include "serial_common.h"

#include "rx_common.h"
#include "rx_sumd.h"

// driver for SUMD receiver using UART2

// FIXME test support for more than 8 channels, should probably work up to 12 channels

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 16
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5) // 6 channels + 5 = 17 bytes for 6 channels

#define SUMD_BAUDRATE 115200

static bool sumdFrameDone = false;
static void sumdDataReceive(uint16_t c);
static uint16_t sumdReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

static uint32_t sumdChannelData[SUMD_MAX_CHANNEL];

static serialPort_t *sumdPort;

void sumdUpdateSerialRxFunctionConstraint(functionConstraint_t *functionConstraint)
{
    functionConstraint->minBaudRate = SUMD_BAUDRATE;
    functionConstraint->maxBaudRate = SUMD_BAUDRATE;
    functionConstraint->requiredSerialPortFeatures = SPF_SUPPORTS_CALLBACK;
}

bool sumdInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    sumdPort = openSerialPort(FUNCTION_SERIAL_RX, sumdDataReceive, SUMD_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
    if (callback)
        *callback = sumdReadRawRC;

    rxRuntimeConfig->channelCount = SUMD_MAX_CHANNEL;

    return sumdPort != NULL;
}

static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdChannels;

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
        sumdChannels = (uint8_t)c;
    if (sumdIndex < SUMD_BUFFSIZE)
        sumd[sumdIndex] = (uint8_t)c;
    sumdIndex++;
    if (sumdIndex == sumdChannels * 2 + 5) {
        sumdIndex = 0;
        sumdFrameDone = true;
    }
}

#define SUMD_OFFSET_CHANNEL_1_HIGH 3
#define SUMD_OFFSET_CHANNEL_1_LOW 4
#define SUMD_BYTES_PER_CHANNEL 2


bool sumdFrameComplete(void)
{
    uint8_t channelIndex;

    if (!sumdFrameDone) {
        return false;
    }

    sumdFrameDone = false;

    if (sumd[1] != 0x01) {
        return false;
    }

    if (sumdChannels > SUMD_MAX_CHANNEL)
        sumdChannels = SUMD_MAX_CHANNEL;

    for (channelIndex = 0; channelIndex < sumdChannels; channelIndex++) {
        sumdChannelData[channelIndex] = (
            (sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_LOW]
        );
    }
    return true;
}

static uint16_t sumdReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    return sumdChannelData[chan] / 8;
}
