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
#include <stdarg.h>
#include <stdlib.h>

#include "platform.h"

#include "common/printf.h"

#include "drivers/serial.h"
#include "io/serial.h"

#include "printf_serial.h"

#ifdef SERIAL_PORT_COUNT

static serialPort_t *printfSerialPort;

void setPrintfSerialPort(serialPort_t *serialPort)
{
    printfSerialPort = serialPort;
}

#ifdef REQUIRE_CC_ARM_PRINTF_SUPPORT

int tfp_printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int written = tfp_format(stdout_putp, stdout_putf, fmt, va);
    va_end(va);
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    return written;
}

static void _putc(void *p, char c)
{
    UNUSED(p);
    serialWrite(printfSerialPort, c);
}


void printfSerialInit(void)
{
    init_printf(NULL, _putc);
}

#else // REQUIRE_CC_ARM_PRINTF_SUPPORT

// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    serialWrite(printfSerialPort, c);
    return c;
}

void printfSerialInit(void)
{
    // Nothing to do
}
#endif

#endif // SERIAL_PORT_COUNT
