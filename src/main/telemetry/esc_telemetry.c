#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "fc/config.h"
#include "config/feature.h"
#include "config/config_master.h"

#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "flight/mixer.h"

#include "sensors/battery.h"

#include "esc_telemetry.h"

#include "build/debug.h"

/*
KISS ESC TELEMETRY PROTOCOL
---------------------------

One transmission will have 10 times 8-bit bytes sent with 115200 baud and 3.6V.

Byte 0: Temperature
Byte 1: Voltage high byte
Byte 2: Voltage low byte
Byte 3: Current high byte
Byte 4: Current low byte
Byte 5: Consumption high byte
Byte 6: Consumption low byte
Byte 7: Rpm high byte
Byte 8: Rpm low byte
Byte 9: 8-bit CRC

*/
typedef struct {
    uint8_t temperature;
    uint16_t voltage;
    uint16_t current;
    uint16_t consumption;
    uint16_t rpm;
} esc_telemetry_t;

typedef enum {
    ESC_TLM_FRAME_PENDING = 1 << 0,     // 1
    ESC_TLM_FRAME_COMPLETE = 1 << 1     // 2
} escTlmFrameState_t;

typedef enum {
    ESC_TLM_TRIGGER_WAIT = 0,
    ESC_TLM_TRIGGER_READY = 1 << 0,     // 1
    ESC_TLM_TRIGGER_PENDING = 1 << 1,   // 2
} escTlmTriggerState_t;

#define ESC_TLM_BAUDRATE 115200
#define ESC_TLM_BUFFSIZE 10
#define ESC_BOOTTIME 2000               // 2 seconds
#define ESC_REQUEST_TIMEOUT 3000        // 3 seconds

static bool tlmFrameDone = false;
static bool firstCycleComplete = false;
static uint8_t tlm[ESC_TLM_BUFFSIZE] = { 0, };
static uint8_t tlmFramePosition = 0;
static serialPort_t *escTelemetryPort = NULL;
static esc_telemetry_t escTelemetryData[4];
static uint32_t escTriggerTimestamp = -1;

static uint8_t escTelemetryMotor = 99;      // motor index 0 - 3
static bool escTelemetryEnabled = false;
static escTlmTriggerState_t escTelemetryTriggerState = ESC_TLM_TRIGGER_WAIT;

static void escTelemetryDataReceive(uint16_t c);
static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);

bool isEscTelemetryEnabled(void)
{
    return escTelemetryEnabled;
}

bool escTelemetrySendTrigger(uint8_t index)
{
    if (escTelemetryTriggerState == ESC_TLM_TRIGGER_PENDING)
    {
        if (escTriggerTimestamp + ESC_REQUEST_TIMEOUT < millis())
        {
            // ESC did not repond in time, resend telemetry request
            escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;
        }
        return false;
    }

    if (escTelemetryTriggerState == ESC_TLM_TRIGGER_READY)
    {
        debug[0] = escTelemetryMotor+1;

        if (escTelemetryMotor == index) {
            escTelemetryTriggerState = ESC_TLM_TRIGGER_PENDING;
            escTriggerTimestamp = millis();
            return true;
        }
    }

    return false;
}

bool escTelemetryInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_ESC);
    if (!portConfig) {
        return false;
    }

    portOptions_t options = (SERIAL_NOT_INVERTED); //SERIAL_INVERTED

    // Initialize serial port
    escTelemetryPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_ESC, escTelemetryDataReceive, ESC_TLM_BAUDRATE, MODE_RX, options);

    if (escTelemetryPort) {
        escTelemetryEnabled = true;
    }

    return escTelemetryPort != NULL;
}

void freeEscTelemetryPort(void)
{
    closeSerialPort(escTelemetryPort);
    escTelemetryPort = NULL;
    escTelemetryEnabled = false;
}

// Receive ISR callback
static void escTelemetryDataReceive(uint16_t c)
{
    // KISS ESC sends some data during startup, ignore this for now (maybe future use)
    // startup data could be firmware version and serialnumber

    if (escTelemetryTriggerState == ESC_TLM_TRIGGER_WAIT) return;

    tlm[tlmFramePosition] = (uint8_t)c;

    debug[2]++;

    if (tlmFramePosition == ESC_TLM_BUFFSIZE - 1) {
        tlmFrameDone = true;
        tlmFramePosition = 0;
    } else {
        tlmFramePosition++;
    }
}

uint8_t escTelemetryFrameStatus(void)
{
    uint8_t frameStatus = ESC_TLM_FRAME_PENDING;
    uint16_t chksum, tlmsum;

    if (!tlmFrameDone) {
        return frameStatus;
    }

    tlmFrameDone = false;

    // Get CRC8 checksum
    chksum = get_crc8(tlm, ESC_TLM_BUFFSIZE - 1);
    tlmsum = tlm[ESC_TLM_BUFFSIZE - 1];     // last byte contains CRC value

    if (chksum == tlmsum) {
        escTelemetryData[escTelemetryMotor].temperature = tlm[0];
        escTelemetryData[escTelemetryMotor].voltage = tlm[1] << 8 | tlm[2];
        escTelemetryData[escTelemetryMotor].current = tlm[3] << 8 | tlm[4];
        escTelemetryData[escTelemetryMotor].consumption = tlm[5] << 8 | tlm[6];
        escTelemetryData[escTelemetryMotor].rpm = tlm[7] << 8 | tlm[8];

        debug[3] = escTelemetryData[escTelemetryMotor].voltage;

        frameStatus = ESC_TLM_FRAME_COMPLETE;
    }

    return frameStatus;
}

void escTelemetryProcess(uint32_t currentTime)
{
    UNUSED(currentTime);

    debug[1] = escTelemetryTriggerState;

    if (!escTelemetryEnabled) {
        return;
    }

    // Wait period of time before requesting telemetry (let the system boot first)
    if (millis() < ESC_BOOTTIME)
    {
        return;
    }
    else if (escTelemetryTriggerState == ESC_TLM_TRIGGER_WAIT)
    {
        // Ready for starting requesting telemetry
        escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;
        escTelemetryMotor = 0;
    }

    // Get received frame status
    uint8_t state = escTelemetryFrameStatus();

    if (state == ESC_TLM_FRAME_COMPLETE)
    {
        uint8_t motorCount = mixers[masterConfig.mixerMode].motorCount;

        // Wait until all ESCs are processed
        if (firstCycleComplete)
        {
            uint8_t i;
            amperage = 0;
            mAhDrawn = 0;
            for (i = 0; i < motorCount; i++)
            {
                amperage = amperage + escTelemetryData[i].current;
                mAhDrawn = mAhDrawn + escTelemetryData[i].consumption;
            }
        }

        escTelemetryMotor++;
        if (escTelemetryMotor > motorCount-1) {
            escTelemetryMotor = 0;
            firstCycleComplete = true;
        }
        escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;
    }
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;

    crc_u = crc;
    crc_u ^= crc_seed;

    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );

    return (crc_u);
}

//-- CRC

static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
    uint8_t crc = 0, i;
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}
