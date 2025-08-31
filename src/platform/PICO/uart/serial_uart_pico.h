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

#include "hardware/pio.h"

// The PIO block for software UARTs UART2, UART3
#define UART_PIO_INSTANCE PIO_INSTANCE(PIO_UART_INDEX)

typedef struct pioDetails_s {
    irq_num_t irqn;
    io_rw_32 *enableReg;
    io_ro_32 *statusReg;
    int rxPin;
    int txPin;
    uint16_t sm_rx; // sm number for rx (0..3)
    uint16_t sm_tx; // sm number for tx (0..3)
    uint32_t rx_intr_bit; // bit to check on interrupt enable and status registers for rx not empty
    uint32_t tx_intr_bit; // bit to check on interrupt enable and status registers for tx not full
} pioDetails_t;

#define UART_PIO_DETAILS_IDX(id) (id - SERIAL_PORT_UART2)
#define UART_PIO_DETAILS_PTR(id) (&uartPioDetails[UART_PIO_DETAILS_IDX(id)])

// Store for details, catering for UART2, UART3
extern pioDetails_t uartPioDetails[2];

// Base for PIO pin counts (0 or 16)
extern int uartPioBase;
 

bool serialUART_pio(uint32_t baudRate, portMode_e mode, portOptions_e options,
                    const uartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO);
void uartReconfigure_pio(uartPort_t *s);
void uartEnableTxInterrupt_pio(uartPort_t *uartPort);


bool serialUART_hw(uint32_t baudRate, portMode_e mode, portOptions_e options,
                   const uartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO);
void uartReconfigure_hw(uartPort_t *s);
void uartEnableTxInterrupt_hw(uartPort_t *uartPort);
