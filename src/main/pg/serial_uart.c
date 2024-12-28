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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_UART

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/serial_uart.h"

#include "drivers/io_types.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/dma_reqmap.h"

// TODO(hertz@): UARTDEV_CONFIG_MAX is measured to be exactly 8, which cannot accomodate even all the UARTs below
PG_REGISTER_ARRAY_WITH_RESET_FN(serialUartConfig_t, UARTDEV_CONFIG_MAX, serialUartConfig, PG_SERIAL_UART_CONFIG, 0);

typedef struct uartDmaopt_s {
    serialPortIdentifier_e identifier;
    int8_t txDmaopt;
    int8_t rxDmaopt;
} uartDmaopt_t;

static const uartDmaopt_t uartDmaopt[] = {
#ifdef USE_UART0
    { SERIAL_PORT_UART0, UART0_TX_DMA_OPT, UART0_RX_DMA_OPT },
#endif
#ifdef USE_UART1
    { SERIAL_PORT_USART1, UART1_TX_DMA_OPT, UART1_RX_DMA_OPT },
#endif
#ifdef USE_UART2
    { SERIAL_PORT_USART2, UART2_TX_DMA_OPT, UART2_RX_DMA_OPT },
#endif
#ifdef USE_UART3
    { SERIAL_PORT_USART3, UART3_TX_DMA_OPT, UART3_RX_DMA_OPT },
#endif
#ifdef USE_UART4
    { SERIAL_PORT_UART4, UART4_TX_DMA_OPT, UART4_RX_DMA_OPT },
#endif
#ifdef USE_UART5
    { SERIAL_PORT_UART5, UART5_TX_DMA_OPT, UART5_RX_DMA_OPT },
#endif
#ifdef USE_USART6
    { SERIAL_PORT_UART6, UART6_TX_DMA_OPT, UART6_RX_DMA_OPT },
#endif
#ifdef USE_USART7
    { SERIAL_PORT_UART7, UART7_TX_DMA_OPT, UART7_RX_DMA_OPT },
#endif
#ifdef USE_USART8
    { SERIAL_PORT_UART8, UART8_TX_DMA_OPT, UART8_RX_DMA_OPT },
#endif
#ifdef USE_UART9
    { SERIAL_PORT_UART9, UART9_TX_DMA_OPT, UART9_RX_DMA_OPT },
#endif
#ifdef USE_UART10
    { SERIAL_PORT_UART10, UART10_TX_DMA_OPT, UART10_RX_DMA_OPT },
#endif
#ifdef USE_LPUART1
    { SERIAL_PORT_LPUART1, DMA_OPT_UNUSED, DMA_OPT_UNUSED },
#endif
};

void pgResetFn_serialUartConfig(serialUartConfig_t *config)
{
    for (unsigned i = 0; i < UARTDEV_CONFIG_MAX; i++) {
        config[i].txDmaopt = DMA_OPT_UNUSED;
        config[i].rxDmaopt = DMA_OPT_UNUSED;
    }

    for (unsigned i = 0; i < ARRAYLEN(uartDmaopt); i++) {
        const int resourceIndex = serialResourceIndex(uartDmaopt[i].identifier);
        if (resourceIndex >= 0 && resourceIndex < UARTDEV_CONFIG_MAX) {  // hadle corrupted config gracefuly
            config[resourceIndex].txDmaopt = uartDmaopt[i].txDmaopt;
            config[resourceIndex].rxDmaopt = uartDmaopt[i].rxDmaopt;
        }
    }
}
#endif
