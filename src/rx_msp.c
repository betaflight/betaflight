#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system_common.h"

#include "drivers/serial_common.h"
#include "drivers/serial_uart_common.h"
#include "serial_common.h"

#include "failsafe.h"

#include "rx_common.h"
#include "rx_msp.h"

failsafe_t *failsafe;

static bool rxMspFrameDone = false;

static uint16_t rxMspReadRawRC(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
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
        failsafe->vTable->reset();
        rxMspFrameDone = false;
        return true;
    }
    return false;
}

void rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, failsafe_t *initialFailsafe, rcReadRawDataPtr *callback)
{
    failsafe = initialFailsafe;
    if (callback)
        *callback = rxMspReadRawRC;
}
