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
#include "pico_trace.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include "pico/platform/compiler.h"

static int depth;

static const char* prefix[]= {
    "-- ",
    "--- ",
    "---- ",
    "----- ",
    "------ ",
    "------- "
};

static const int plen = sizeof(prefix)/sizeof(prefix[0]);

#ifndef PICO_TRACE_UART_INSTANCE
#error PICO_TRACE build requires defines for PICO_TRACE_UART_INSTANCE, PICO_TRACE_TX_GPIO, PICO_TRACE_RX_GPIO
#endif

void picotrace_prefix(void)
{
    stdio_printf(prefix[depth%plen]);
}

// Wrap main to insert the initialisation code.
extern int main(int argc, char * argv[]);
extern int REAL_FUNC(main)(int argc, char * argv[]);
int WRAPPER_FUNC(main)(int argc, char * argv[])
{
    //stdio_init_all();
    stdio_uart_init_full(UART_INSTANCE(PICO_TRACE_UART_INSTANCE), 115200, PICO_TRACE_TX_GPIO, PICO_TRACE_RX_GPIO);
    tprintf("\n=== Betaflight main ===");
    depth++;
    int mr = REAL_FUNC(main)(argc, argv);
    depth--;
    tprintf("\n=== Betaflight main end ===");
    return mr;
}

#define TRACEvoidvoid(x)                        \
    extern void x(void);                        \
    extern void REAL_FUNC(x)(void);             \
    void WRAPPER_FUNC(x)(void)                  \
    {                                           \
        tprintf("Enter " #x "");                \
        depth++;REAL_FUNC(x)();depth--;         \
        tprintf("Exit  " #x "");                \
    }

#define TRACEboolvoid(x)                        \
    extern bool x(void);                        \
    extern bool REAL_FUNC(x)(void);             \
    bool WRAPPER_FUNC(x)(void)                  \
    {                                           \
        tprintf("Enter " #x "");                \
        depth++;                                \
        bool ret__ = REAL_FUNC(x)();            \
        depth--;                                \
        tprintf("Exit  " #x "");                \
        return ret__;                           \
    }

#define TRACEvoidbool(x)                              \
    extern void x(bool _);                            \
    extern void REAL_FUNC(x)(bool _);                 \
    void WRAPPER_FUNC(x)(bool xyz__)                  \
    {                                                 \
        tprintf("Enter " #x " [%d]", xyz__);          \
        depth++; REAL_FUNC(x)(xyz__); depth--;        \
        tprintf("Exit  " #x "");                      \
    }


// remember to add to PICO_WRAPPED_FUNCTIONS in PICO_trace.mk
TRACEvoidvoid(init)
    TRACEvoidvoid(initEEPROM)
    TRACEvoidvoid(isEEPROMVersionValid)
    TRACEvoidvoid(writeUnmodifiedConfigToEEPROM)
    TRACEboolvoid(resetEEPROM)
    TRACEvoidvoid(resetConfig)
    TRACEvoidvoid(pgResetAll)
    TRACEvoidbool(serialInit)
