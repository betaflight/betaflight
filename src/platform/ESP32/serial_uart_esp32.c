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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

// The ESP-IDF SOC headers declare 'extern uart_dev_t UART0/UART1/UART2',
// but our platform.h redefines them as esp32_peripheral_t pointers.
// Undef our macros so the LL headers see the real hardware register structs.
#undef UART0
#undef UART1
#undef UART2

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/uart_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "hal/uart_types.h"
#include "soc/uart_struct.h"
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_struct.h"
#include "soc/interrupts.h"

// Cache the real uart_dev_t hardware register pointers before
// restoring our platform macros (which would shadow them).
static uart_dev_t *const uartHwRegs[] = {
    UART_LL_GET_HW(0),
    UART_LL_GET_HW(1),
    UART_LL_GET_HW(2),
};

// Restore our platform UART instance macros
#define UART0 (&esp32UartDev0)
#define UART1 (&esp32UartDev1)
#define UART2 (&esp32UartDev2)

// Get the uart_dev_t hardware register pointer from a port number
static uart_dev_t *uartGetHw(int portNum)
{
    return uartHwRegs[portNum];
}

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "platform/interrupt.h"

// ESP32-S3 APB clock frequency (80 MHz)
#define ESP32_APB_CLK_FREQ  80000000

// RX FIFO threshold: trigger interrupt after this many bytes received
#define UART_RXFIFO_FULL_THRESHOLD  1

// RX timeout: trigger interrupt after this many bit-periods of idle
// At 115200 baud, 10 bit-periods ~ 87us — good for low-latency response
#define UART_RX_TOUT_THRESHOLD  10

// ESP32-S3 UART GPIO matrix signal indices
static const uint8_t uartTxSignal[] = { U0TXD_OUT_IDX, U1TXD_OUT_IDX, U2TXD_OUT_IDX };
static const uint8_t uartRxSignal[] = { U0RXD_IN_IDX, U1RXD_IN_IDX, U2RXD_IN_IDX };

// CPU interrupt numbers for each UART (from platform/interrupt.h)
static const int uartCpuIntr[] = { ESP32_CPU_INTR_UART0, ESP32_CPU_INTR_UART1, ESP32_CPU_INTR_UART2 };

// Peripheral interrupt sources for each UART
static const int uartIntrSource[] = { ETS_UART0_INTR_SOURCE, ETS_UART1_INTR_SOURCE, ETS_UART2_INTR_SOURCE };

// ESP32-S3 has 3 UART controllers (UART0, UART1, UART2)
// UART buffers are defined in drivers/serial_uart.c

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART0
    {
        .identifier = SERIAL_PORT_UART0,
        .reg = (usartResource_t *)UART0,
        .rxPins = { { .pin = DEFIO_TAG_E(PA44) }, },
        .txPins = { { .pin = DEFIO_TAG_E(PA43) }, },
        .af = 0,
        .irqn = 0,
        .txPriority = 0,
        .rxPriority = 0,
        .txBuffer = uart0TxBuffer,
        .rxBuffer = uart0RxBuffer,
        .txBufferSize = sizeof(uart0TxBuffer),
        .rxBufferSize = sizeof(uart0RxBuffer),
    },
#endif
#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = (usartResource_t *)UART1,
        .rxPins = { { .pin = DEFIO_TAG_E(PA18) }, },
        .txPins = { { .pin = DEFIO_TAG_E(PA17) }, },
        .af = 0,
        .irqn = 0,
        .txPriority = 0,
        .rxPriority = 0,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_USART2,
        .reg = (usartResource_t *)UART2,
        .rxPins = { { .pin = DEFIO_TAG_E(PA21) }, },
        .txPins = { { .pin = DEFIO_TAG_E(PA10) }, },
        .af = 0,
        .irqn = 0,
        .txPriority = 0,
        .rxPriority = 0,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif
};

static int uartGetPortNum(const usartResource_t *reg)
{
    return UART_INST((const USART_TypeDef *)reg);
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    for (const uartHardware_t *hardware = uartHardware; hardware < ARRAYEND(uartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t *uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        if (uartdev == NULL || resourceIndex < 0) {
            continue;
        }

        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];
        if (!cfgRx && !cfgTx) {
            continue;
        }

        for (unsigned pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
            if (cfgRx && cfgRx == hardware->rxPins[pindex].pin) {
                uartdev->rx = hardware->rxPins[pindex];
            }
            if (cfgTx && cfgTx == hardware->txPins[pindex].pin) {
                uartdev->tx = hardware->txPins[pindex];
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
        }
    }
}

void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
    int portNum = uartGetPortNum(uartPort->USARTx);
    uart_dev_t *hw = uartGetHw(portNum);

    const bool inverted = (uartPort->port.options & SERIAL_INVERTED) != 0;

#ifdef USE_INVERTER
    // When an external inverter is available, use it and keep HW inversion disabled
    // to avoid double-inverting the signal.
    uart_ll_inverse_signal(hw, UART_SIGNAL_INV_DISABLE);
    enableInverter(uartPort->port.identifier, inverted);
#else
    // No external inverter — use ESP32-S3 hardware signal inversion
    if (inverted) {
        uart_ll_inverse_signal(hw, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    } else {
        uart_ll_inverse_signal(hw, UART_SIGNAL_INV_DISABLE);
    }
#endif
}

void uartReconfigure(uartPort_t *uartPort)
{
    int portNum = uartGetPortNum(uartPort->USARTx);
    uart_dev_t *hw = uartGetHw(portNum);

    uart_ll_set_baudrate(hw, uartPort->port.baudRate, ESP32_APB_CLK_FREQ);

    const bool twoStop = uartPort->port.options & SERIAL_STOPBITS_2;
    const bool evenParity = uartPort->port.options & SERIAL_PARITY_EVEN;

    uart_ll_set_stop_bits(hw, twoStop ? UART_STOP_BITS_2 : UART_STOP_BITS_1);
    uart_ll_set_parity(hw, evenParity ? UART_PARITY_EVEN : UART_PARITY_DISABLE);

    uartConfigureExternalPinInversion(uartPort);

    // Configure RX FIFO thresholds for low-latency receive
    uart_ll_set_rxfifo_full_thr(hw, UART_RXFIFO_FULL_THRESHOLD);
    uart_ll_set_rx_tout(hw, UART_RX_TOUT_THRESHOLD);

    // Reconfigure RX interrupts based on current mode.
    // Only touch RX bits to avoid disrupting an in-progress TX transfer.
    uart_ll_disable_intr_mask(hw, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    uart_ll_clr_intsts_mask(hw, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);

    if (uartPort->port.mode & MODE_RX) {
        uart_ll_ena_intr_mask(hw, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    }
}

void uartConfigureDma(uartDevice_t *uartdev)
{
    UNUSED(uartdev);
}

void uartIrqHandler(uartPort_t *s)
{
    int portNum = uartGetPortNum(s->USARTx);
    uart_dev_t *hw = uartGetHw(portNum);
    uint32_t status = uart_ll_get_intsts_mask(hw);

    // RX handling
    if (status & (UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT)) {
        uart_ll_clr_intsts_mask(hw, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
        uint32_t rxLen = uart_ll_get_rxfifo_len(hw);
        while (rxLen--) {
            uint8_t ch;
            uart_ll_read_rxfifo(hw, &ch, 1);
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                s->port.rxBuffer[s->port.rxBufferHead] = ch;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }

    // TX handling
    if (status & UART_INTR_TXFIFO_EMPTY) {
        uart_ll_clr_intsts_mask(hw, UART_INTR_TXFIFO_EMPTY);

        // Fill TX FIFO up to available space.
        // Note: uart_ll_get_txfifo_len() returns FREE space (FIFO_DEF_LEN - txfifo_cnt).
        uint32_t txFree = uart_ll_get_txfifo_len(hw);
        while (txFree && s->port.txBufferTail != s->port.txBufferHead) {
            uart_ll_write_txfifo(hw, (const uint8_t *)&s->port.txBuffer[s->port.txBufferTail], 1);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            txFree--;
        }

        // Disable TX interrupt when buffer is empty
        if (s->port.txBufferTail == s->port.txBufferHead) {
            uart_ll_disable_intr_mask(hw, UART_INTR_TXFIFO_EMPTY);
        }
    }
}

// Per-UART ISR wrappers for the ROM interrupt dispatch table.
// ets_isr_attach requires a void(*)(void*) signature.

static void uart0Isr(void *arg)
{
    UNUSED(arg);
    uartIrqHandler(&uartDevice[UARTDEV_0].port);
}

#ifdef USE_UART1
static void uart1Isr(void *arg)
{
    UNUSED(arg);
    uartIrqHandler(&uartDevice[UARTDEV_1].port);
}
#endif

#ifdef USE_UART2
static void uart2Isr(void *arg)
{
    UNUSED(arg);
    uartIrqHandler(&uartDevice[UARTDEV_2].port);
}
#endif

// Map uartDeviceIdx_e to the ISR wrapper function
static esp32IsrHandler_t uartIsrForDevice(uartDeviceIdx_e devIdx)
{
    switch (devIdx) {
    case UARTDEV_0: return uart0Isr;
#ifdef USE_UART1
    case UARTDEV_1: return uart1Isr;
#endif
#ifdef USE_UART2
    case UARTDEV_2: return uart2Isr;
#endif
    default: return NULL;
    }
}

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
    if (uartPort->port.txBufferTail == uartPort->port.txBufferHead) {
        return;
    }
    int portNum = uartGetPortNum(uartPort->USARTx);
    uart_dev_t *hw = uartGetHw(portNum);
    uart_ll_clr_intsts_mask(hw, UART_INTR_TXFIFO_EMPTY);
    uart_ll_ena_intr_mask(hw, UART_INTR_TXFIFO_EMPTY);
}

void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    UNUSED(options);

    uartPort_t *s = &uartdev->port;
    const uartHardware_t *hardware = uartdev->hardware;
    if (!hardware) {
        return NULL;
    }

    int portNum = uartGetPortNum(hardware->reg);
    uart_dev_t *hw = uartGetHw(portNum);

    // Enable UART peripheral clock and reset
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
    uart_ll_enable_bus_clock(portNum, true);
    uart_ll_reset_register(portNum);

    const serialPortIdentifier_e identifier = hardware->identifier;
    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier);

    // Configure TX pin via GPIO matrix (only if TX mode requested)
    if (mode & MODE_TX) {
        IO_t txIO = IOGetByTag(uartdev->tx.pin);
        if (txIO) {
            IOInit(txIO, ownerTxRx, ownerIndex);
            uint32_t txPin = IO_Pin(txIO);
            esp_rom_gpio_pad_select_gpio(txPin);
            gpio_ll_output_enable(&GPIO, txPin);
            esp_rom_gpio_connect_out_signal(txPin, uartTxSignal[portNum], false, false);
        }
    }

    // Configure RX pin via GPIO matrix (only if RX mode requested)
    if (mode & MODE_RX) {
        IO_t rxIO = IOGetByTag(uartdev->rx.pin);
        if (rxIO) {
            IOInit(rxIO, ownerTxRx + 1, ownerIndex);
            uint32_t rxPin = IO_Pin(rxIO);
            esp_rom_gpio_pad_select_gpio(rxPin);
            gpio_ll_input_enable(&GPIO, rxPin);
            gpio_ll_pullup_en(&GPIO, rxPin);
            esp_rom_gpio_connect_in_signal(rxPin, uartRxSignal[portNum], false);
        }
    }

    // Configure UART parameters
    uart_ll_set_baudrate(hw, baudRate, ESP32_APB_CLK_FREQ);
    uart_ll_set_data_bit_num(hw, UART_DATA_8_BITS);
    uart_ll_set_parity(hw, UART_PARITY_DISABLE);
    uart_ll_set_stop_bits(hw, UART_STOP_BITS_1);

    // Reset FIFOs
    uart_ll_rxfifo_rst(hw);
    uart_ll_txfifo_rst(hw);

    // Disable all interrupts before registering ISR
    uart_ll_disable_intr_mask(hw, 0xFFFFFFFF);
    uart_ll_clr_intsts_mask(hw, 0xFFFFFFFF);

    // Register ISR with ROM interrupt dispatch
    uartDeviceIdx_e devIdx = uartDeviceIdxFromIdentifier(identifier);
    esp32IsrHandler_t isrFn = uartIsrForDevice(devIdx);
    if (isrFn) {
        esp32IntrRoute(uartCpuIntr[portNum], uartIntrSource[portNum]);
        esp32IntrRegister(uartCpuIntr[portNum], isrFn, NULL);
        esp32IntrEnable(uartCpuIntr[portNum]);
    }

    // Setup port buffers and hardware reference
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    s->USARTx = hardware->reg;

    s->port.vTable = uartVTable;

    // RX interrupts are enabled later via uartReconfigure() called from uartOpen()
    return s;
}
