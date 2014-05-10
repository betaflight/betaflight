#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system_common.h"

#include "drivers/serial_common.h"
#include "drivers/serial_uart_common.h"
#include "serial_common.h"

#include "rx_common.h"
#include "rx_msp.h"

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
    if (callback)
        *callback = rxMspReadRawRC;

    return true;
}
