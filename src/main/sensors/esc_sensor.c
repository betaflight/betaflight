/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_ESC_SENSOR)

#include "build/debug.h"

#include "common/time.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/timer.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "esc_sensor.h"

#include "config/config.h"

#include "flight/mixer.h"

#include "io/serial.h"

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

PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
        .halfDuplex = 0
);

/*
DEBUG INFORMATION
-----------------

set debug_mode = DEBUG_ESC_SENSOR in cli

*/

enum {
    DEBUG_ESC_MOTOR_INDEX = 0,
    DEBUG_ESC_NUM_TIMEOUTS = 1,
    DEBUG_ESC_NUM_CRC_ERRORS = 2,
    DEBUG_ESC_DATA_AGE = 3,
};

typedef enum {
    ESC_SENSOR_FRAME_PENDING = 0,
    ESC_SENSOR_FRAME_COMPLETE = 1,
    ESC_SENSOR_FRAME_FAILED = 2
} escTlmFrameState_t;

typedef enum {
    ESC_SENSOR_TRIGGER_STARTUP = 0,
    ESC_SENSOR_TRIGGER_READY = 1,
    ESC_SENSOR_TRIGGER_PENDING = 2
} escSensorTriggerState_t;

#define ESC_SENSOR_BAUDRATE 115200
#define ESC_BOOTTIME 5000               // 5 seconds
#define ESC_REQUEST_TIMEOUT 100         // 100 ms (data transfer takes only 900us)

#define TELEMETRY_FRAME_SIZE 10
static uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };

static volatile uint8_t *buffer;
static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPosition = 0;

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];

static escSensorTriggerState_t escSensorTriggerState = ESC_SENSOR_TRIGGER_STARTUP;
static uint32_t escTriggerTimestamp;
static uint8_t escSensorMotor = 0;      // motor index

static escSensorData_t combinedEscSensorData;
static bool combinedDataNeedsUpdate = true;

static uint16_t totalTimeoutCount = 0;
static uint16_t totalCrcErrorCount = 0;

void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    buffer = frameBuffer;
    bufferPosition = 0;
    bufferSize = frameLength;
}

uint8_t getNumberEscBytesRead(void)
{
    return bufferPosition;
}

static bool isFrameComplete(void)
{
    return bufferPosition == bufferSize;
}

bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

escSensorData_t *getEscSensorData(uint8_t motorNumber)
{
    if (!featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return NULL;
    }

    if (motorNumber < getMotorCount()) {
        return &escSensorData[motorNumber];
    } else if (motorNumber == ESC_SENSOR_COMBINED) {
        if (combinedDataNeedsUpdate) {
            combinedEscSensorData.dataAge = 0;
            combinedEscSensorData.temperature = 0;
            combinedEscSensorData.voltage = 0;
            combinedEscSensorData.current = 0;
            combinedEscSensorData.consumption = 0;
            combinedEscSensorData.rpm = 0;

            for (int i = 0; i < getMotorCount(); i = i + 1) {
                combinedEscSensorData.dataAge = MAX(combinedEscSensorData.dataAge, escSensorData[i].dataAge);
                combinedEscSensorData.temperature = MAX(combinedEscSensorData.temperature, escSensorData[i].temperature);
                combinedEscSensorData.voltage += escSensorData[i].voltage;
                combinedEscSensorData.current += escSensorData[i].current;
                combinedEscSensorData.consumption += escSensorData[i].consumption;
                combinedEscSensorData.rpm += escSensorData[i].rpm;
            }

            combinedEscSensorData.voltage = combinedEscSensorData.voltage / getMotorCount();
            combinedEscSensorData.rpm = combinedEscSensorData.rpm / getMotorCount();

            combinedDataNeedsUpdate = false;

            DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_DATA_AGE, combinedEscSensorData.dataAge);
        }

        return &combinedEscSensorData;
    } else {
        return NULL;
    }
}

// Receive ISR callback
static void escSensorDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    // KISS ESC sends some data during startup, ignore this for now (maybe future use)
    // startup data could be firmware version and serialnumber

    if (isFrameComplete()) {
        return;
    }

    buffer[bufferPosition++] = (uint8_t)c;
}

bool escSensorInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    if (!portConfig) {
        return false;
    }

    portOptions_e options = SERIAL_NOT_INVERTED  | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

    // Initialize serial port
    escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, escSensorDataReceive, NULL, ESC_SENSOR_BAUDRATE, MODE_RX, options);

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i = i + 1) {
        escSensorData[i].dataAge = ESC_DATA_INVALID;
    }

    return escSensorPort != NULL;
}

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++) {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
}

static uint8_t decodeEscFrame(void)
{
    if (!isFrameComplete()) {
        return ESC_SENSOR_FRAME_PENDING;
    }

    // Get CRC8 checksum
    uint16_t chksum = calculateCrc8(telemetryBuffer, TELEMETRY_FRAME_SIZE - 1);
    uint16_t tlmsum = telemetryBuffer[TELEMETRY_FRAME_SIZE - 1];     // last byte contains CRC value
    uint8_t frameStatus;
    if (chksum == tlmsum) {
        escSensorData[escSensorMotor].dataAge = 0;
        escSensorData[escSensorMotor].temperature = telemetryBuffer[0];
        escSensorData[escSensorMotor].voltage = telemetryBuffer[1] << 8 | telemetryBuffer[2];
        escSensorData[escSensorMotor].current = telemetryBuffer[3] << 8 | telemetryBuffer[4];
        escSensorData[escSensorMotor].consumption = telemetryBuffer[5] << 8 | telemetryBuffer[6];
        escSensorData[escSensorMotor].rpm = telemetryBuffer[7] << 8 | telemetryBuffer[8];

        combinedDataNeedsUpdate = true;

        frameStatus = ESC_SENSOR_FRAME_COMPLETE;

        if (escSensorMotor < 4) {
            DEBUG_SET(DEBUG_ESC_SENSOR_RPM, escSensorMotor, erpmToRpm(escSensorData[escSensorMotor].rpm) / 10); // output actual rpm/10 to fit in 16bit signed.
            DEBUG_SET(DEBUG_ESC_SENSOR_TMP, escSensorMotor, escSensorData[escSensorMotor].temperature);
        }
    } else {
        frameStatus = ESC_SENSOR_FRAME_FAILED;
    }

    return frameStatus;
}

static void increaseDataAge(void)
{
    if (escSensorData[escSensorMotor].dataAge < ESC_DATA_INVALID) {
        escSensorData[escSensorMotor].dataAge++;

        combinedDataNeedsUpdate = true;
    }
}

static void selectNextMotor(void)
{
    escSensorMotor++;
    if (escSensorMotor == getMotorCount()) {
        escSensorMotor = 0;
    }
}

// XXX Review ESC sensor under refactored motor handling

void escSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    if (!escSensorPort || !motorIsEnabled()) {
        return;
    }

    switch (escSensorTriggerState) {
        case ESC_SENSOR_TRIGGER_STARTUP:
            // Wait period of time before requesting telemetry (let the system boot first)
            if (currentTimeMs >= ESC_BOOTTIME) {
                escSensorTriggerState = ESC_SENSOR_TRIGGER_READY;
            }

            break;
        case ESC_SENSOR_TRIGGER_READY:
            escTriggerTimestamp = currentTimeMs;

            startEscDataRead(telemetryBuffer, TELEMETRY_FRAME_SIZE);
            motorDmaOutput_t * const motor = getMotorDmaOutput(escSensorMotor);
            motor->protocolControl.requestTelemetry = true;
            escSensorTriggerState = ESC_SENSOR_TRIGGER_PENDING;

            DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_MOTOR_INDEX, escSensorMotor + 1);

            break;
        case ESC_SENSOR_TRIGGER_PENDING:
            if (currentTimeMs < escTriggerTimestamp + ESC_REQUEST_TIMEOUT) {
                uint8_t state = decodeEscFrame();
                switch (state) {
                    case ESC_SENSOR_FRAME_COMPLETE:
                        selectNextMotor();
                        escSensorTriggerState = ESC_SENSOR_TRIGGER_READY;

                        break;
                    case ESC_SENSOR_FRAME_FAILED:
                        increaseDataAge();

                        selectNextMotor();
                        escSensorTriggerState = ESC_SENSOR_TRIGGER_READY;

                        DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_NUM_CRC_ERRORS, ++totalCrcErrorCount);
                        break;
                    case ESC_SENSOR_FRAME_PENDING:
                        break;
                }
            } else {
                // Move on to next ESC, we'll come back to this one
                increaseDataAge();

                selectNextMotor();
                escSensorTriggerState = ESC_SENSOR_TRIGGER_READY;

                DEBUG_SET(DEBUG_ESC_SENSOR, DEBUG_ESC_NUM_TIMEOUTS, ++totalTimeoutCount);
            }

            break;
    }
}

#endif
