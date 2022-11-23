#include <math.h>
#include <stdint.h>

#include "platform.h"

#include "wifi.h"

#include "io/serial.h"

#include "drivers/time.h"


static serialPort_t *wifiSerialPort = NULL;

wififind_t wifiDev;

static uint8_t wificmd01[] = {0x01, 0x56, 0x07, 0x0D, 0x0A};
//static char wificmd02[] = "AT+CWMODE=1";
// static char wificmd03[] = "AT+CWJAP="Xiaomi12","11111111a"";
// static char wificmd04[] = "AT";
// static char wificmd05[] = "AT";
// static char wificmd06[] = "AT";

bool wifi_init(void)
{
    if(!wifiATK8266Detect(&wifiDev.dev))
    {
        return false;
    }

    
    wifiDev.dev.lastValidResponseTimeMs = millis();

    return true;
}

void atk_8266_send_InitCmd(void)
{
    serialWriteBuf(wifiSerialPort, wificmd01, sizeof(wificmd01));
    //serialPrint(wifiSerialPort, wificmd02);
}


bool wifiATK8266Detect(wifiDev_t *dev)
{
    return WifiDetect(dev);
}

bool WifiDetect(wifiDev_t *dev)
{
    UNUSED(dev);
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_WIFI_ESP8266);

    if (!portConfig) {
        return false;
    }

    wifiSerialPort = openSerialPort(SERIAL_PORT_UART4, FUNCTION_WIFI_ESP8266, NULL, NULL, 115200, MODE_RXTX, 0);

    if (wifiSerialPort == NULL) {
        return false;
    }

    return true;
}

void wifiUpdate(wifiDev_t *dev)
{
    UNUSED(dev);
    static timeMs_t lastFrameReceivedMs = 0;
    const timeMs_t timeNowMs = millis();
    UNUSED(lastFrameReceivedMs);
    if(wifiSerialPort == NULL){
        return;
    }

    lastFrameReceivedMs = timeNowMs;

}