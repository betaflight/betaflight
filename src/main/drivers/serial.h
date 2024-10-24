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

#pragma once

#include "drivers/io.h"
#include "drivers/io_types.h"
#include "drivers/resource.h"
#include "drivers/serial_resource.h"

#include "pg/pg.h"

typedef enum {
    MODE_RX = 1 << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_e;

typedef enum {
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3,

    /*
     * Note on SERIAL_BIDIR_PP *on some MCU families*
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    // output configuration in BIDIR non-inverted mode
    // pushpull is used in UNIDIR or inverted mode by default
    // SERIAL_BIDIR must be specified explicitly
    SERIAL_BIDIR_OD        = 0 << 4, // default in BIDIR non-inverted mode
    SERIAL_BIDIR_PP        = 1 << 4, // force pushpull

    SERIAL_PULL_DEFAULT    = 0 << 5, // pulldown in inverted mode, pullup otherwise
    SERIAL_PULL_NONE       = 1 << 5, // disable pulls in RX or opendrain TX mode
    // option for Smartaudio serial port - line is in MARK state when idle - break condition.
    // DO NOT USE unless absolutely necessary
    // SERIAL_PULL_NONE has precedence
    SERIAL_PULL_SMARTAUDIO = 1 << 6, // set PULLDOWN on RX, even when not inverted.

    // If this option is set then switch the TX line to input when not in use to detect it being pulled low
    // (and prevent powering external device by TX pin)
    SERIAL_CHECK_TX        = 1 << 7,
} portOptions_e;

// Define known line control states which may be passed up by underlying serial driver callback
#define CTRL_LINE_STATE_DTR (1 << 0)
#define CTRL_LINE_STATE_RTS (1 << 1)

typedef void (*serialReceiveCallbackPtr)(uint16_t data, void *rxCallbackData);   // used by serial drivers to return frames to app
typedef void (*serialIdleCallbackPtr)();

typedef struct serialPort_s {

    const struct serialPortVTable *vTable;

    portMode_e mode;
    portOptions_e options;

    uint32_t baudRate;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    serialReceiveCallbackPtr rxCallback;
    void *rxCallbackData;

    serialIdleCallbackPtr idleCallback;

    uint8_t identifier;  // actually serialPortIdentifier_e
} serialPort_t;

typedef struct serialPinConfig_s {
    ioTag_t ioTagTx[RESOURCE_SERIAL_COUNT];
    ioTag_t ioTagRx[RESOURCE_SERIAL_COUNT];
#ifdef USE_INVERTER
    // TODO - no need for LPUART inverter
    ioTag_t ioTagInverter[RESOURCE_UART_COUNT + RESOURCE_LPUART_COUNT]; // this array is only for UARTs.
#endif
} serialPinConfig_t;

PG_DECLARE(serialPinConfig_t, serialPinConfig);

struct serialPortVTable {
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);

    uint32_t (*serialTotalRxWaiting)(const serialPort_t *instance);
    uint32_t (*serialTotalTxFree)(const serialPort_t *instance);

    uint8_t (*serialRead)(serialPort_t *instance);

    // Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
    void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);

    bool (*isSerialTransmitBufferEmpty)(const serialPort_t *instance);

    void (*setMode)(serialPort_t *instance, portMode_e mode);
    void (*setCtrlLineStateCb)(serialPort_t *instance, void (*cb)(void *instance, uint16_t ctrlLineState), void *context);
    void (*setBaudRateCb)(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context);

    void (*writeBuf)(serialPort_t *instance, const void *data, int count);
    // Optional functions used to buffer large writes.
    void (*beginWrite)(serialPort_t *instance);
    void (*endWrite)(serialPort_t *instance);
};

void serialWrite(serialPort_t *instance, uint8_t ch);
uint32_t serialRxBytesWaiting(const serialPort_t *instance);
uint32_t serialTxBytesFree(const serialPort_t *instance);
void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count);
void serialWriteBufNoFlush(serialPort_t *instance, const uint8_t *data, int count);
uint8_t serialRead(serialPort_t *instance);
void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate);
void serialSetMode(serialPort_t *instance, portMode_e mode);
void serialSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context);
void serialSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);
uint32_t serialGetBaudRate(serialPort_t *instance);

// A shim that adapts the bufWriter API to the serialWriteBuf() API.
void serialWriteBufShim(void *instance, const uint8_t *data, int count);
void serialBeginWrite(serialPort_t *instance);
void serialEndWrite(serialPort_t *instance);
