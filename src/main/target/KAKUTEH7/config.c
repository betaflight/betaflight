#include <stdint.h>
#include "platform.h"
#ifdef USE_TARGET_CONFIG
#include "flight/mixer.h"
#include "osd/osd.h"
#include "io/serial.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/motor.h"
#include "target.h"
#include "config_helper.h"

#define BLUETOOTH_MSP_UART        SERIAL_PORT_USART2
#define BLUETOOTH_MSP_BAUDRATE    BAUD_115200               

static targetSerialPortFunction_t targetSerialPortFunction[] = {
	{ SERIAL_PORT_USART1, FUNCTION_MSP },
	{ SERIAL_PORT_USART7, FUNCTION_ESC_SENSOR },
};

void targetConfiguration(void)
{
  pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED|PINIO_CONFIG_MODE_OUT_PP;
  pinioBoxConfigMutable()->permanentId[0] = BOXARM;
  serialPortConfig_t *bluetoothMspUART = serialFindPortConfigurationMutable(BLUETOOTH_MSP_UART);
  if(bluetoothMspUART){
  	bluetoothMspUART->functionMask = FUNCTION_MSP;
	bluetoothMspUART->msp_baudrateIndex = BLUETOOTH_MSP_BAUDRATE;
  	}
  
  targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
  
}

#endif

