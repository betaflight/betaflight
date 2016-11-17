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

#ifdef USE_DSHOT

typedef struct {
    bool skipped;
    int16_t temperature;
    int16_t voltage;
    int16_t current;
    int16_t consumption;
    int16_t rpm;
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
#define ESC_BOOTTIME 5000               // 5 seconds
#define ESC_REQUEST_TIMEOUT 100         // 100 ms (data transfer takes only 900us)

static bool tlmFrameDone = false;
static uint8_t tlm[ESC_TLM_BUFFSIZE] = { 0, };
static uint8_t tlmFramePosition = 0;
static serialPort_t *escTelemetryPort = NULL;
static esc_telemetry_t escTelemetryData[MAX_SUPPORTED_MOTORS];
static uint32_t escTriggerTimestamp = -1;
static uint32_t escTriggerLastTimestamp = -1;
static uint8_t timeoutRetryCount = 0;

static uint8_t escTelemetryMotor = 0;      // motor index
static bool escTelemetryEnabled = false;
static escTlmTriggerState_t escTelemetryTriggerState = ESC_TLM_TRIGGER_WAIT;

static int16_t escVbat = 0;
static int16_t escCurrent = 0;
static int16_t escConsumption = 0;

static void escTelemetryDataReceive(uint16_t c);
static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
static void selectNextMotor(void);

bool isEscTelemetryActive(void)
{
    return escTelemetryEnabled;
}

int16_t getEscTelemetryVbat(void)
{
    return escVbat / 10;
}

int16_t getEscTelemetryCurrent(void)
{
    return escCurrent;
}

int16_t getEscTelemetryConsumption(void)
{
    return escConsumption;
}

bool escTelemetryInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_ESC);
    if (!portConfig) {
        return false;
    }

    portOptions_t options = (SERIAL_NOT_INVERTED);

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
    uint32_t currentTimeMs = currentTime / 1000;

    if (!escTelemetryEnabled) {
        return;
    }

    // Wait period of time before requesting telemetry (let the system boot first)
    if (millis() < ESC_BOOTTIME) {
        return;
    }
    else if (escTelemetryTriggerState == ESC_TLM_TRIGGER_WAIT) {
        // Ready for starting requesting telemetry
        escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;
        escTelemetryMotor = 0;
        escTriggerTimestamp = currentTimeMs;
        escTriggerLastTimestamp = escTriggerTimestamp;
    }
    else if (escTelemetryTriggerState == ESC_TLM_TRIGGER_READY) {
        if (debugMode == DEBUG_ESC_TELEMETRY) debug[0] = escTelemetryMotor+1;

        motorDmaOutput_t * const motor = getMotorDmaOutput(escTelemetryMotor);
        motor->requestTelemetry = true;
        escTelemetryTriggerState = ESC_TLM_TRIGGER_PENDING;
    }

    if (escTriggerTimestamp + ESC_REQUEST_TIMEOUT < currentTimeMs) {
        // ESC did not repond in time, retry
        timeoutRetryCount++;
        escTriggerTimestamp = currentTimeMs;
        escTelemetryTriggerState = ESC_TLM_TRIGGER_READY;

        if (timeoutRetryCount == 4) {
            // Not responding after 3 times, skip motor
            escTelemetryData[escTelemetryMotor].skipped = true;
            selectNextMotor();
        }

        if (debugMode == DEBUG_ESC_TELEMETRY) debug[1]++;
    }

    // Get received frame status
    uint8_t state = escTelemetryFrameStatus();

    if (state == ESC_TLM_FRAME_COMPLETE) {
        // Wait until all ESCs are processed
        if (escTelemetryMotor == getMotorCount()-1) {
            escCurrent = 0;
            escConsumption = 0;
            escVbat = 0;

            for (int i = 0; i < getMotorCount(); i++) {
                if (!escTelemetryData[i].skipped) {
                    escVbat =  i > 0 ? ((escVbat + escTelemetryData[i].voltage) / 2) : escTelemetryData[i].voltage;
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

    if (escTriggerLastTimestamp + 10000 < currentTimeMs) {
        // ESCs did not respond for 10 seconds
        // Disable ESC telemetry and fallback to onboard vbat sensor
        freeEscTelemetryPort();
        escVbat = 0;
        escCurrent = 0;
        escConsumption = 0;
    }
}

static void selectNextMotor(void)
{
    escTelemetryMotor++;
    if (escTelemetryMotor == getMotorCount()) {
        escTelemetryMotor = 0;
    }
    escTriggerTimestamp = millis();
    escTriggerLastTimestamp = escTriggerTimestamp;
}

//-- CRC

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

static uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
    uint8_t crc = 0;
    for(int i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}

#endif
