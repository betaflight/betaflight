#include <math.h>
#include <stdint.h>

#include "platform.h"

#include "wifi.h"

#include "io/serial.h"

#include "drivers/time.h"

static bool WifiDetect(wifiDev_t *dev);
static bool wifi_init();

static serialPort_t *wifiSerialPort = NULL;

wifiDev_t wifiDev;

static char wificmd01[] = "AT";
// static char wificmd02[] = "AT+CWMODE=1";
// static char wificmd03[] = "AT+CWJAP="Xiaomi12","11111111a"";
// static char wificmd04[] = "AT";
// static char wificmd05[] = "AT";
// static char wificmd06[] = "AT";

static bool wifi_init()
{
    if(!wifiATK8266Detect(&wifiDev.dev))
    {
        return false;
    }

    wifiDev.lastValidResponseTimeMs = millis();

    return true;
}

void atk_8266_send_InitCmd()
{
    serialWriteBuf(wifiSerialPort, wificmd01, sizeof(wificmd01));
}

static bool WifiDetect(wifiDev_t *dev)
{
    UNUSED(dev);
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_WIFI_ESP8266);

    if (!portConfig) {
        return false;
    }

    wifiSerialPort = openSerialPort(portConfig->identifier, FUNCTION_WIFI_ESP8266, NULL, NULL, 115200, MODE_RXTX, 0);

    if (wifiSerialPort == NULL) {
        return false;
    }

    return true;
}

bool wifiATK8266Detect(wifiDev_t *dev)
{
    return WifiDetect(dev);
}

void wifiUpdate(wifiDev_t *dev)
{
    UNUSED(dev);
    static timeMs_t lastFrameReceivedMs = 0;
    const timeMs_t timeNowMs = millis();

    if(wifiSerialPort == NULL){
        return;
    }
    // while (serialRxBytesWaiting(wifiSerialPort))
    // {
    //     uint8_t c = serialRead(wifiSerialPort);

    // }

    lastFrameReceivedMs = timeNowMs;

    // if (timeNowMs - lastFrameReceivedMs > TF_TIMEOUT_MS) {
    atk_8266_send_InitCmd();
    // }

}