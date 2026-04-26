/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if ENABLE_LCD_CONSOLE

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "common/printf.h"

#include "drivers/lcd_console.h"
#include "drivers/serial.h"
#include "drivers/serial_lcd_console.h"

static void lcdSerialWrite(serialPort_t *instance, uint8_t ch);
static uint32_t lcdSerialTotalRxWaiting(const serialPort_t *instance);
static uint32_t lcdSerialTotalTxFree(const serialPort_t *instance);
static uint8_t lcdSerialRead(serialPort_t *instance);
static void lcdSerialSetBaudRate(serialPort_t *instance, uint32_t baudRate);
static bool lcdSerialIsTransmitBufferEmpty(const serialPort_t *instance);
static void lcdSerialSetMode(serialPort_t *instance, portMode_e mode);
static void lcdSerialSetCtrlLineStateCb(serialPort_t *instance,
        void (*cb)(void *instance, uint16_t ctrlLineState), void *context);
static void lcdSerialSetBaudRateCb(serialPort_t *instance,
        void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context);
static void lcdSerialWriteBuf(serialPort_t *instance, const void *data, int count);
static void lcdSerialBeginWrite(serialPort_t *instance);
static void lcdSerialEndWrite(serialPort_t *instance);

static const struct serialPortVTable lcdSerialVTable = {
    .serialWrite                = lcdSerialWrite,
    .serialTotalRxWaiting       = lcdSerialTotalRxWaiting,
    .serialTotalTxFree          = lcdSerialTotalTxFree,
    .serialRead                 = lcdSerialRead,
    .serialSetBaudRate          = lcdSerialSetBaudRate,
    .isSerialTransmitBufferEmpty = lcdSerialIsTransmitBufferEmpty,
    .setMode                    = lcdSerialSetMode,
    .setCtrlLineStateCb         = lcdSerialSetCtrlLineStateCb,
    .setBaudRateCb              = lcdSerialSetBaudRateCb,
    .writeBuf                   = lcdSerialWriteBuf,
    .beginWrite                 = lcdSerialBeginWrite,
    .endWrite                   = lcdSerialEndWrite,
};

static serialPort_t lcdSerialPort = {
    .vTable = &lcdSerialVTable,
};

static bool serialOpened;

serialPort_t *lcdConsoleSerialOpen(void)
{
    if (!serialOpened) {
        if (!lcdConsoleInit()) {
            return NULL;
        }
        lcdSerialPort.mode = MODE_TX;
        serialOpened = true;
    }
    return &lcdSerialPort;
}

static void lcdSerialWrite(serialPort_t *instance, uint8_t ch)
{
    UNUSED(instance);
    lcdConsolePutc(ch);
}

static uint32_t lcdSerialTotalRxWaiting(const serialPort_t *instance)
{
    UNUSED(instance);
    return 0;
}

static uint32_t lcdSerialTotalTxFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return UINT16_MAX;      // Always accept; the L2/L3 layers throttle as needed.
}

static uint8_t lcdSerialRead(serialPort_t *instance)
{
    UNUSED(instance);
    return 0;               // Write-only sink.
}

static void lcdSerialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);
}

static bool lcdSerialIsTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return !lcdConsoleIsBusy();
}

static void lcdSerialSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);
}

static void lcdSerialSetCtrlLineStateCb(serialPort_t *instance,
        void (*cb)(void *instance, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
}

static void lcdSerialSetBaudRateCb(serialPort_t *instance,
        void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);
    UNUSED(cb);
    UNUSED(context);
}

static void lcdSerialWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);
    if (count > 0) {
        lcdConsoleWrite((const uint8_t *)data, (size_t)count);
    }
}

static void lcdSerialBeginWrite(serialPort_t *instance)
{
    UNUSED(instance);
}

static void lcdSerialEndWrite(serialPort_t *instance)
{
    UNUSED(instance);
    lcdConsoleFlush();
}

// Explicit logging API — bypass the global printf sink.

void lcdConsolePuts(const char *s)
{
    if (!s) {
        return;
    }
    lcdConsoleWrite((const uint8_t *)s, strlen(s));
}

static void lcdConsolePrintfPutc(void *p, char c)
{
    UNUSED(p);
    lcdConsolePutc((uint8_t)c);
}

void lcdConsolePrintf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    tfp_format(NULL, lcdConsolePrintfPutc, fmt, va);
    va_end(va);
    lcdConsoleFlush();
}

#endif // ENABLE_LCD_CONSOLE
