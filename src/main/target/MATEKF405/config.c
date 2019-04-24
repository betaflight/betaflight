#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "target.h"

#define BLUETOOTH_MSP_UART          SERIAL_PORT_UART5
#define BLUETOOTH_MSP_BAUDRATE      BAUD_19200

void targetConfiguration(void)
{
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = BOXARM;
    
    serialPortConfig_t *bluetoothMspUART = serialFindPortConfiguration(BLUETOOTH_MSP_UART);
    if (bluetoothMspUART) {
        bluetoothMspUART->functionMask = FUNCTION_MSP;
        bluetoothMspUART->msp_baudrateIndex = BLUETOOTH_MSP_BAUDRATE;
    }
}

#endif
