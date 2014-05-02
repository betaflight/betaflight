#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/system_common.h"
#include "drivers/serial_common.h"
#include "drivers/serial_uart_common.h"

#include "serial_cli.h"
#include "serial_msp.h"

#include "serial_common.h"

static serialConfig_t *serialConfig;
serialPorts_t serialPorts;

void serialInit(serialConfig_t *initialSerialConfig)
{
    serialConfig = initialSerialConfig;

    resetMainSerialPort();

    mspInit();
}

void resetMainSerialPort(void)
{
    openMainSerialPort(serialConfig->port1_baudrate);
}

void openMainSerialPort(uint32_t baudrate)
{
    serialPorts.mainport = uartOpen(USART1, NULL, baudrate, MODE_RXTX);
}

void handleSerial(void)
{
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

    mspProcess();
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr)
{
    if (sr == '#')
        cliProcess();
    else if (sr == serialConfig->reboot_character)
        systemReset(true);      // reboot to bootloader
}

