#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

static bool rxMspFrameDone = false;

static uint16_t rxMspReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    return rcData[chan];
}

void rxMspFrameRecieve(void)
{
    rxMspFrameDone = true;
}

bool rxMspFrameComplete(void)
{
    if (rxMspFrameDone) {
        rxMspFrameDone = false;
        return true;
    }
    return false;
}

bool rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    rxRuntimeConfig->channelCount = 8; // See MSP_SET_RAW_RC
    if (callback)
        *callback = rxMspReadRawRC;

    return true;
}
