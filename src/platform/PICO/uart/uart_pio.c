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

#ifdef USE_UART

#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart_impl.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

#include "serial_uart_pico.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// The PIO block for software UARTs PIOUART0, PIOUART1
static const PIO uartPio = PIO_INSTANCE(PIO_UART_INDEX);

#define PIO_IRQ_INDEX(irqn) ((irqn) == PIO_IRQ_NUM(uartPio, 0) ? 0 : 1)

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

#if SERIAL_PIOUART_MAX > 2
#error USE_PIOUARTn only currently supported for n=0,1
#endif

// Store for details, catering for PIOUART0, PIOUART1
static pioDetails_t uartPioDetails[2];

#define UART_PIO_DETAILS_IDX(id) ((id) - SERIAL_PORT_PIOUART_FIRST)
#define UART_PIO_DETAILS_PTR(id) (&uartPioDetails[UART_PIO_DETAILS_IDX(id)])

// Base for PIO pin counts (0 or 16)
static int uartPioBase;

static int txProgram_offset = -1;
static int rxProgram_offset = -1;

// Look-ups for bit to check on status register IRQ[01]_INTS by SM
static const uint32_t rxnemptybit[4] = {
    PIO_INTR_SM0_RXNEMPTY_BITS,
    PIO_INTR_SM1_RXNEMPTY_BITS,
    PIO_INTR_SM2_RXNEMPTY_BITS,
    PIO_INTR_SM3_RXNEMPTY_BITS,
};

static const uint32_t txnfullbit[4] = {
    PIO_INTR_SM0_TXNFULL_BITS,
    PIO_INTR_SM1_TXNFULL_BITS,
    PIO_INTR_SM2_TXNFULL_BITS,
    PIO_INTR_SM3_TXNFULL_BITS,
};

// PIO-based UARTs. For now, hardwired to PIOUARTs 0,1 on PIO number UART_PIO_INDEX.
const pioUartHardware_t pioUartHardware[PIOUARTDEV_COUNT] = {
#ifdef USE_PIOUART0
    {
        .identifier = SERIAL_PORT_PIOUART0,
        .irqn = PIO_IRQ_NUM(uartPio, 0),
        .txBuffer = uartPio0TxBuffer,
        .rxBuffer = uartPio0RxBuffer,
        .txBufferSize = sizeof(uartPio0TxBuffer),
        .rxBufferSize = sizeof(uartPio0RxBuffer),
    },
#endif

#ifdef USE_PIOUART1
    {
        .identifier = SERIAL_PORT_PIOUART1,
        .irqn = PIO_IRQ_NUM(uartPio, 1),
        .txBuffer = uartPio1TxBuffer,
        .rxBuffer = uartPio1RxBuffer,
        .txBufferSize = sizeof(uartPio1TxBuffer),
        .rxBufferSize = sizeof(uartPio1RxBuffer),
    },
#endif
};

static uartPinDef_t makePinDef(ioTag_t tag)
{
    uartPinDef_t ret = { .pin = tag };
    return ret;
}

void uartPinConfigure_pio(const serialPinConfig_t *pSerialPinConfig)
{
    // software UART by PIO
    int pinIndexMin = 48;
    int pinIndexMax = -1;
    uartPioBase = 0;
    for (const pioUartHardware_t* hardware = pioUartHardware; hardware < ARRAYEND(pioUartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t* uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];
        bprintf("pico uartPinConfigure pio at %p dev = %p,  tags rx 0x%x, tx 0x%x", hardware, uartdev, cfgRx, cfgTx);
        if (!cfgRx && !cfgTx) {
            continue;
        }

        // On a single PIO block, we are restricted either to pins 0-31 or pins 16-47.
        pinIndexMin = cfgRx && (DEFIO_TAG_PIN(cfgRx) < pinIndexMin) ? DEFIO_TAG_PIN(cfgRx) : pinIndexMin;
        pinIndexMax = cfgRx && (DEFIO_TAG_PIN(cfgRx) > pinIndexMax) ? DEFIO_TAG_PIN(cfgRx) : pinIndexMax;
        pinIndexMin = cfgTx && (DEFIO_TAG_PIN(cfgTx) < pinIndexMin) ? DEFIO_TAG_PIN(cfgTx) : pinIndexMin;
        pinIndexMax = cfgTx && (DEFIO_TAG_PIN(cfgTx) > pinIndexMax) ? DEFIO_TAG_PIN(cfgTx) : pinIndexMax;
        if (pinIndexMax >= 32) {
            if (pinIndexMin < 16) {
                bprintf("* Not configuring PIOUART with identifier %d (PIO can't span pins min %d max %d)",
                        identifier, pinIndexMin, pinIndexMax);
                continue;
            } else {
                uartPioBase = 16;
            }
        }

        if (cfgRx) {
            uartdev->rx = makePinDef(cfgRx);
        }

        if (cfgTx) {
            uartdev->tx = makePinDef(cfgTx);
        }

        if (uartdev->rx.pin || uartdev->tx.pin ) {
            bprintf("uartdev %p setting hardware to %p which has txbuffer %p", uartdev, hardware, hardware->txBuffer);
            uartdev->hardware = (uartHardware_t *)hardware; // Sneak in pointer to pioUartHardware_t as a pointer to uartHardware_t
        } else {
            bprintf("** uartPinConfigure_pio no compatible rx or tx pin for this PIO UART");
        }
    }

    bprintf("pico uartPinConfigure pio%d pin min, max = %d, %d; setting gpio base to %d", PIO_NUM(uartPio), pinIndexMin, pinIndexMax, uartPioBase);
}

static bool ensurePioProgram(PIO pio, const pio_program_t *program, bool isTx)
{
    // The GPIO base must be set before adding the program.
    pio_set_gpio_base(uartPio, uartPioBase);

    if (isTx) {
        if (txProgram_offset < 0) {
            txProgram_offset = pio_add_program(pio, program);
        }

        return txProgram_offset >= 0;
   } else {
        if (rxProgram_offset < 0) {
            rxProgram_offset = pio_add_program(pio, program);
        }

        return rxProgram_offset >= 0;
    }
}

#if SERIAL_PIOUART_COUNT > 0
static void uartPioIrqHandler(uartPort_t *s, pioDetails_t *pioDetailsPtr)
{
    io_rw_32 *enableRegPtr = pioDetailsPtr->enableReg;
    io_ro_32 *statusRegPtr = pioDetailsPtr->statusReg;
    // Is RX enabled and does irq status indicate RX fifo not empty?
    if (*enableRegPtr & pioDetailsPtr->rx_intr_bit && *statusRegPtr & pioDetailsPtr->rx_intr_bit) {
        uint sm_rx = pioDetailsPtr->sm_rx;

        // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
        io_rw_8 *rxfifo_shift = (io_rw_8*)&uartPio->rxf[sm_rx] + 3;
        serialReceiveCallbackPtr rxCallback = s->port.rxCallback;
        if (rxCallback) {
            void *rxCallbackData = s->port.rxCallbackData;
            while (!pio_sm_is_rx_fifo_empty(uartPio, sm_rx)) {
                const uint8_t ch = (uint8_t)*rxfifo_shift;
                rxCallback(ch, rxCallbackData);
            }
        } else {
            volatile uint8_t *rxBuffer = s->port.rxBuffer;
            uint32_t rxBufferSize = s->port.rxBufferSize;
            while (!pio_sm_is_rx_fifo_empty(uartPio, sm_rx)) {
                const uint8_t ch = (uint8_t)*rxfifo_shift;
                rxBuffer[s->port.rxBufferHead] = ch;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % rxBufferSize;
            }
        }
    }

    // Is TX enabled?
    if (*enableRegPtr & pioDetailsPtr->tx_intr_bit) {
        uint sm_tx = pioDetailsPtr->sm_tx;
        while (!pio_sm_is_tx_fifo_full(uartPio, sm_tx)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                ///bprintf("uartIrqHandler PIO TX put %x", s->port.txBuffer[s->port.txBufferTail]);
                // Send 8 bits of data in a 32-bit word. The PIO program will emit the low 8 bits.
                pio_sm_put(uartPio, sm_tx, s->port.txBuffer[s->port.txBufferTail]);
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                // Done sending the buffer, clear the pio TX interrupt enable
                //bprintf("uart pio done put %d, disabling tx interrupt",c);
                int irqn_index = PIO_IRQ_INDEX(pioDetailsPtr->irqn);
                pio_interrupt_source_t irqSourceTX = pio_get_tx_fifo_not_full_interrupt_source(sm_tx);
                pio_set_irqn_source_enabled(uartPio, irqn_index, irqSourceTX, false);
                break;
            }
        }
    }
}
#endif

static void on_pioUART0(void)
{
///    bprintf("\n\n on_pioUART0");
#ifdef USE_PIOUART0
    uartPioIrqHandler(&pioUartDevice[PIOUARTDEV_0].port, UART_PIO_DETAILS_PTR(SERIAL_PORT_PIOUART0));
#endif
}

static void on_pioUART1(void)
{
///    bprintf("\n\n\n\non_pioUART1");
#ifdef USE_PIOUART1
    uartPioIrqHandler(&pioUartDevice[PIOUARTDEV_1].port, UART_PIO_DETAILS_PTR(SERIAL_PORT_PIOUART1));
#endif
}

bool serialUART_pio(uartPort_t *s, uint32_t baudRate, portMode_e mode, portOptions_e options,
                    const pioUartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO)
{
    // Set up details for state machine, will be finalised in uartReconfigure.
    if (options != 0) {
        bprintf("*** non-default options not yet supported for UART PIO");
        return false;
    }

    // mode and baudrate are captured on the uartPort in uartOpen, and can be addressed in uartReconfigure
    UNUSED(mode);
    UNUSED(baudRate);

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1
    pioDetails_t *uartPioDetailsPtr = UART_PIO_DETAILS_PTR(identifier);
    uartPioDetailsPtr->irqn = hardware->irqn;
    uartPioDetailsPtr->enableReg = PIO_IRQ_INDEX(hardware->irqn) == 0 ? &(uartPio->inte0) : &(uartPio->inte1);
    uartPioDetailsPtr->statusReg = PIO_IRQ_INDEX(hardware->irqn) == 0 ? &(uartPio->ints0) : &(uartPio->ints1);

    if (txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        uint32_t txPin = IO_Pin(txIO);
        bprintf("set up PIO for UART on tx pin %d", txPin);
        if (!ensurePioProgram(uartPio, &uart_tx_program, true /* Tx */)) {
            bprintf("pico serialUART_pio tx failed to add program to pio");
            return false;
        }

        const int pio_sm_tx = pio_claim_unused_sm(uartPio, false);
        if (pio_sm_tx < 0) {
            bprintf("pico serialUART_pio tx failed to claim state machine");
            return false;
        }

        uartPioDetailsPtr->txPin = txPin;
        uartPioDetailsPtr->sm_tx = pio_sm_tx;
        uartPioDetailsPtr->tx_intr_bit = txnfullbit[pio_sm_tx];
    }

    if (rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        uint32_t rxPin = IO_Pin(rxIO);
        bprintf("set up PIO for UART on rx pin %d", rxPin);
        if (!ensurePioProgram(uartPio, &uart_rx_program, false /* Rx */)) {
            bprintf("pico serialUART_pio rx failed to add program to pio");
            return false;
        }

        const int pio_sm_rx = pio_claim_unused_sm(uartPio, false);
        if (pio_sm_rx < 0) {
            bprintf("pico serialUART_pio rx failed to claim state machine");
            return false;
        }

        uartPioDetailsPtr->rxPin = rxPin;
        uartPioDetailsPtr->sm_rx = pio_sm_rx;
        uartPioDetailsPtr->rx_intr_bit = rxnemptybit[pio_sm_rx];
    }

    // TODO implement - use options here...
//    uart_set_hw_flow(uartInstance, false, false);
//    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);

    bprintf("id %d, going to set exclusive handler for irqn %d", hardware->identifier, hardware->irqn);
    irq_set_exclusive_handler(hardware->irqn, hardware->identifier == SERIAL_PORT_PIOUART0 ? on_pioUART0 : on_pioUART1);
    irq_set_enabled(hardware->irqn, true);

    // Don't enable pio irq yet, wait until a call to uartReconfigure...
    // (with current code in serial_uart.c, this prevents irq callback before rxCallback has been set)

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    bprintf("uartport %p port %p txbuffer %p from hardware %p",
            s, &s->port, s->port.txBuffer, hardware);
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    return true;
}

void uartReconfigure_pio(uartPort_t *s)
{
    bprintf("uartReconfigure for port %p with PIO %p, mode = 0x%x", s, uartPio, s->port.mode);
    const serialPortIdentifier_e identifier = s->port.identifier;
    pioDetails_t *uartPioDetailsPtr = UART_PIO_DETAILS_PTR(identifier);
    uint sm_tx = uartPioDetailsPtr->sm_tx;
    uint sm_rx = uartPioDetailsPtr->sm_rx;
    pio_interrupt_source_t irqSourceRX = pio_get_rx_fifo_not_empty_interrupt_source(sm_rx);
    pio_interrupt_source_t irqSourceTX = pio_get_tx_fifo_not_full_interrupt_source(sm_tx);
    int irqn_index = PIO_IRQ_INDEX(uartPioDetailsPtr->irqn);

    pio_set_irqn_source_enabled(uartPio, irqn_index, irqSourceRX, false);
    pio_set_irqn_source_enabled(uartPio, irqn_index, irqSourceTX, false);

    // (re)init program, with baud rate
    // Arrange GPIO including pullup for RX, assign PIO SM pins, FIFO, clock, enable SM
    bprintf("uartReconfigure_pio init rx, tx programs, pins %d, %d, (requested) baudrate %d",
            uartPioDetailsPtr->rxPin, uartPioDetailsPtr->txPin, s->port.baudRate);
    uart_rx_program_init(uartPio, sm_rx, rxProgram_offset, uartPioDetailsPtr->rxPin, s->port.baudRate);
    uart_tx_program_init(uartPio, sm_tx, txProgram_offset, uartPioDetailsPtr->txPin, s->port.baudRate);

    // TODO PIO format currently restricted to 8n1, no h/w flow control
    // TODO make use of s->port.options
    uartConfigureExternalPinInversion(s);

    // Start receiving if MODE_RX
    pio_set_irqn_source_enabled(uartPio, irqn_index, irqSourceRX, s->port.mode & MODE_RX);
}

void uartEnableTxInterrupt_pio(uartPort_t *uartPort)
{
    pioDetails_t *pioDetailsPtr = UART_PIO_DETAILS_PTR(uartPort->port.identifier);
    pio_interrupt_source_t irqSourceTX = pio_get_tx_fifo_not_full_interrupt_source(pioDetailsPtr->sm_tx);
    int irqn_index = PIO_IRQ_INDEX(pioDetailsPtr->irqn);
    // bprintf("uartEnableTxInterrupt_pio %p irqn_index %d, %p", uartPio, irqn_index, irqSourceTX);
    pio_set_irqn_source_enabled(uartPio, irqn_index, irqSourceTX, true);
}
 
#endif
