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
#include <string.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "io/serial.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "pg/pg_ids.h"

#if defined(USE_UART) || defined(USE_LPUART) || defined(USE_SOFTSERIAL)
typedef struct serialDefaultPin_s {
    serialPortIdentifier_e ident;
    ioTag_t rxIO, txIO, inverterIO;
} serialDefaultPin_t;

static const serialDefaultPin_t serialDefaultPin[] = {
#ifdef USE_UART1
    { SERIAL_PORT_USART1, IO_TAG(UART1_RX_PIN), IO_TAG(UART1_TX_PIN), IO_TAG(INVERTER_PIN_UART1) },
#endif
#ifdef USE_UART2
    { SERIAL_PORT_USART2, IO_TAG(UART2_RX_PIN), IO_TAG(UART2_TX_PIN), IO_TAG(INVERTER_PIN_UART2) },
#endif
#ifdef USE_UART3
    { SERIAL_PORT_USART3, IO_TAG(UART3_RX_PIN), IO_TAG(UART3_TX_PIN), IO_TAG(INVERTER_PIN_UART3) },
#endif
#ifdef USE_UART4
    { SERIAL_PORT_UART4, IO_TAG(UART4_RX_PIN), IO_TAG(UART4_TX_PIN), IO_TAG(INVERTER_PIN_UART4) },
#endif
#ifdef USE_UART5
    { SERIAL_PORT_UART5, IO_TAG(UART5_RX_PIN), IO_TAG(UART5_TX_PIN), IO_TAG(INVERTER_PIN_UART5) },
#endif
#ifdef USE_UART6
    { SERIAL_PORT_USART6, IO_TAG(UART6_RX_PIN), IO_TAG(UART6_TX_PIN), IO_TAG(INVERTER_PIN_UART6) },
#endif
#ifdef USE_UART7
    { SERIAL_PORT_USART7, IO_TAG(UART7_RX_PIN), IO_TAG(UART7_TX_PIN), IO_TAG(INVERTER_PIN_UART7) },
#endif
#ifdef USE_UART8
    { SERIAL_PORT_USART8, IO_TAG(UART8_RX_PIN), IO_TAG(UART8_TX_PIN), IO_TAG(INVERTER_PIN_UART8) },
#endif
#ifdef USE_UART9
    { SERIAL_PORT_UART9, IO_TAG(UART9_RX_PIN), IO_TAG(UART9_TX_PIN), IO_TAG(INVERTER_PIN_UART9) },
#endif
#ifdef USE_UART10
    { SERIAL_PORT_USART10, IO_TAG(UART10_RX_PIN), IO_TAG(UART10_TX_PIN), IO_TAG(INVERTER_PIN_UART10) },
#endif
#ifdef USE_LPUART1
    { SERIAL_PORT_LPUART1, IO_TAG(LPUART1_RX_PIN), IO_TAG(LPUART1_TX_PIN), IO_TAG(INVERTER_PIN_LPUART1) },
#endif
#ifdef USE_SOFTSERIAL1
    { SERIAL_PORT_SOFTSERIAL1, IO_TAG(SOFTSERIAL1_RX_PIN), IO_TAG(SOFTSERIAL1_TX_PIN), IO_TAG(NONE) },
#endif
#ifdef USE_SOFTSERIAL2
    { SERIAL_PORT_SOFTSERIAL2, IO_TAG(SOFTSERIAL2_RX_PIN), IO_TAG(SOFTSERIAL2_TX_PIN), IO_TAG(NONE) },
#endif
};

PG_REGISTER_WITH_RESET_FN(serialPinConfig_t, serialPinConfig, PG_SERIAL_PIN_CONFIG, 0);

void pgResetFn_serialPinConfig(serialPinConfig_t *serialPinConfig)
{
    for (const serialDefaultPin_t *defpin = serialDefaultPin; defpin < ARRAYEND(serialDefaultPin); defpin++) {
        const int resourceIndex = serialResourceIndex(defpin->ident);
        if (resourceIndex >= 0) {
            serialPinConfig->ioTagRx[resourceIndex] = defpin->rxIO;
            serialPinConfig->ioTagTx[resourceIndex] = defpin->txIO;
#if defined(USE_INVERTER)
            // LPUART/SOFTSERIAL do not need/support inverter
            if (resourceIndex < (int)ARRAYLEN(serialPinConfig->ioTagInverter)) {
                serialPinConfig->ioTagInverter[resourceIndex] = defpin->inverterIO;
            }
#endif
        }
    }
}


#endif
#endif
