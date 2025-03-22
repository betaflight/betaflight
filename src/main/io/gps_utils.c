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

#include <stdarg.h>

#include "platform.h"

#include "common/printf.h"
#include "io/serial.h"

struct nmea_putp {
    serialPort_t* port;
    uint8_t csum;
};

static void nmea_putcf_csum(void* putp, char c)
{
    struct nmea_putp* p = putp;
    serialWrite(p->port, c);
    p->csum ^= c;
}

static void nmea_putcf(void* putp, char c)
{
    struct nmea_putp* p = putp;
    serialWrite(p->port, c);
}

static void nmeaPrintfva_raw(struct nmea_putp* putp, const char *format, va_list va)
{
    tfp_format(putp, nmea_putcf, format, va);
}

// tfp_prinf shall replace this
static void nmeaPrintf_raw(struct nmea_putp* putp, const char *format, ...)
{
    va_list va;
    va_start(va, format);
    nmeaPrintfva_raw(putp, format, va);
    va_end(va);
}

static void nmeaPrintfva(serialPort_t *port, const char *format, va_list va)
{
    struct nmea_putp putp = {
        .port = port,
        .csum = '$'             // don't checkfum '$' in header
    };
    tfp_format(&putp, nmea_putcf_csum, format, va);
    nmeaPrintf_raw(&putp, "*%02X\r\n", putp.csum);
}

// printf format (+ arguments), append "*<csum>\r\n"
// nmeaPrintf(port, "$PUBX,41,1,0003,0001,%d,0", 115200);
void nmeaPrintf(serialPort_t *port, const char *format, ...)
{
    va_list va;
    va_start(va, format);
    nmeaPrintfva(port, format, va);
    va_end(va);
}

