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
#include "drivers/pwm_output.h"
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

/*
DEBUG INFORMATION
-----------------

set debug_mode = DEBUG_ESC_TELEMETRY in cli

0: current motor index requested
1: number of timeouts
2: voltage
3: current
*/

typedef struct {
    bool skipped;
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
#define ESC_BOOTTIME 3000               // 3 seconds
#define ESC_REQUEST_TIMEOUT 1000        // 1 seconds

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

static uint16_t escVbat = 0;
static uint16_t escCurrent = 0;
static uint16_t escConsumption = 0;

static void escTelemetryDataReceive(uint16_t c);
static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
static void selectNextMotor(void);

static motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

bool isEscTelemetryActive(void)
{
    return escTelemetryEnabled;
}

uint16_t getEscTelemetryVbat(void)
{
    return escVbat / 10;
}

uint16_t getEscTelemetryCurrent(void)
{
    return escCurrent;
}

uint16_t getEscTelemetryConsumption(void)
{
    return escConsumption;
}

void pwmRequestTelemetry(uint8_t index)
{
    if (escTelemetryTriggerState == ESC_TLM_TRIGGER_PENDING)
    {
        return;
    }

    if (escTelemetryTriggerState == ESC_TLM_TRIGGER_READY)
    {
        if (debugMode == DEBUG_ESC_TELEMETRY) debug[0] = escTelemetryMotor+1;

        if (escTelemetryMotor == index) {
            escTelemetryTriggerState = ESC_TLM_TRIGGER_PENDING;
            motorDmaOutput_t * const motor = &dmaMotors[index];
            motor->requestTelemetry = true;
        }
    }
}

// bool escTelemetrySendTrigger(uint8_t index)
// {
//     if (escTelemetryTriggerState == ESC_TLM_TRIGGER_PENDING)
//     {
//         return false;
//     }
//
//     if (escTelemetryTriggerState == ESC_TLM_TRIGGER_READY)
//     {
//         if (debugMode == DEBUG_ESC_TELEMETRY) debug[0] = escTelemetryMotor+1;
//
//         if (escTelemetryMotor == index) {
//             escTelemetryTriggerState = ESC_TLM_TRIGGER_PENDING;
//             return true;
//         }
//     }
//
//     return false;
// }

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
        masterConfig.batteryConfig.currentMeterType = CURRENT_SENSOR_ESC;
        masterConfig.batteryConfig.batteryMeterType = BATTERY_SENSOR_ESC;
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
        escTelemetryData[escTelemetryMotor].skipped = false;
        escTelemetryData[escTelemetryMotor].temperature = tlm[0];
        escTelemetryData[escTelemetryMotor].voltage = tlm[1] << 8 | tlm[2];
        escTelemetryData[escTelemetryMotor].current = tlm[3] << 8 | tlm[4];
        escTelemetryData[escTelemetryMotor].consumption = tlm[5] << 8 | tlm[6];
        escTelemetryData[escTelemetryMotor].rpm = tlm[7] << 8 | tlm[8];

        frameStatus = ESC_TLM_FRAME_COMPLETE;
    }

    return frameStatus;
}

void escTelemetryProcess(uint32_t currentTime)
{
    UNUSED(currentTime);

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
        escTriggerTimestamp = millis();
    }

    if (escTriggerTimestamp + ESC_REQUEST_TIMEOUT < millis())
    {
        // ESC did not repond in time, skip to next motor
        escTelemetryData[escTelemetryMotor].skipped = true;
        selectNextMotor();
        escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;

        if (debugMode == DEBUG_ESC_TELEMETRY) debug[1]++;
    }

    // Get received frame status
    uint8_t state = escTelemetryFrameStatus();

    if (state == ESC_TLM_FRAME_COMPLETE)
    {
        // Wait until all ESCs are processed
        if (firstCycleComplete)
        {
            uint8_t i;
            escCurrent = 0;
            escConsumption = 0;
            for (i = 0; i < 4; i++)             // Motor count for Dshot limited to 4
            {
                if (!escTelemetryData[i].skipped)
                {
                    escVbat = escVbat == 0 ? escTelemetryData[i].voltage : (escVbat + escTelemetryData[i].voltage) / 2;
                    escCurrent = escCurrent + escTelemetryData[i].current;
                    escConsumption = escConsumption + escTelemetryData[i].consumption;
                }
            }
        }

        if (debugMode == DEBUG_ESC_TELEMETRY) debug[2] = escVbat;
        if (debugMode == DEBUG_ESC_TELEMETRY) debug[3] = escCurrent;

        selectNextMotor();
        escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;
    }
}

static void selectNextMotor(void)
{
    escTelemetryMotor++;
    if (escTelemetryMotor >= 4) {           // Motor count for Dshot limited to 4
        escTelemetryMotor = 0;
        firstCycleComplete = true;
    }
    escTriggerTimestamp = millis();
}

//-- CRC

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;

    crc_u = crc;
    crc_u ^= crc_seed;

    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );

    return (crc_u);
}

static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
    uint8_t crc = 0, i;
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}
