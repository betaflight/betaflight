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

// UART driver for HPMicro.
// Provides serialUART()/uartPinConfigure() port setup, TX/RX interrupt handling
// and a GPTMR timer that switches bidirectional half-duplex ports back to RX
// after a frame and monitors TX completion for SERIAL_CHECK_TX.

#include "platform.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef USE_UART
#include "board.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "nvic_hpmicro.h"
#include "io_hpmicro.h"

#ifdef USE_INVERTER
#include "drivers/inverter.h"
#endif

#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_gptmr_drv.h"

typedef struct uartQueue_s {
    uartDevice_t *uartList[8];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} uartQueue_t;

typedef struct uartErrorCounters_s {
    uint32_t overrun;
    uint32_t parity;
    uint32_t framing;
    uint32_t lineBreak;
} uartErrorCounters_t;

static FAST_DATA_ZERO_INIT uartQueue_t uartQueue;
static FAST_DATA_ZERO_INIT volatile uartErrorCounters_t uartErrorCounters[UARTDEV_COUNT];
static bool gptmrConfigured = false;
static uint32_t gptmrBidirBaudrate;

#define UART_MAX_FRAME_BITS 12U // 1 start + 8 data + 1 parity + 2 stop

static bool uartQueuePush(uartDevice_t *uart)
{
    const uint32_t irqState = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    uint8_t index = uartQueue.head;

    for (uint8_t i = 0; i < uartQueue.count; i++) {
        if (uartQueue.uartList[index] == uart) {
            enable_global_irq(irqState);
            return true;
        }
        index = (index + 1) % (sizeof(uartQueue.uartList) / sizeof(uartQueue.uartList[0]));
    }

    if (uartQueue.count >= sizeof(uartQueue.uartList) / sizeof(uartQueue.uartList[0])) {
        enable_global_irq(irqState);
        return false;
    }
    uartQueue.uartList[uartQueue.tail] = uart;
    uartQueue.tail = (uartQueue.tail + 1) % (sizeof(uartQueue.uartList) / sizeof(uartQueue.uartList[0]));
    uartQueue.count++;
    enable_global_irq(irqState);
    return true;
}

static uartDevice_t *uartQueuePop(void)
{
    const uint32_t irqState = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (uartQueue.count == 0) {
        enable_global_irq(irqState);
        return NULL;
    }
    uartDevice_t *uart = uartQueue.uartList[uartQueue.head];
    uartQueue.head = (uartQueue.head + 1) % (sizeof(uartQueue.uartList) / sizeof(uartQueue.uartList[0]));
    uartQueue.count--;
    enable_global_irq(irqState);
    return uart;
}

static void uartQueueRemove(uartDevice_t *uart)
{
    const uint8_t pending = uartQueue.count;

    for (uint8_t i = 0; i < pending; i++) {
        uartDevice_t *queuedUart = uartQueuePop();
        if (queuedUart != uart) {
            uartQueuePush(queuedUart);
        }
    }
}

static void timerConfig(void)
{
    uint32_t slowestBidirBaudrate = 0;

    for (size_t i = 0; i < UARTDEV_COUNT; i++) {
        const serialPort_t *port = &uartDevice[i].port.port;
        if (!(port->options & (SERIAL_BIDIR | SERIAL_CHECK_TX)) || !(port->mode & MODE_TX) || !port->baudRate) {
            continue;
        }
        if (!slowestBidirBaudrate || port->baudRate < slowestBidirBaudrate) {
            slowestBidirBaudrate = port->baudRate;
        }
    }

    if (!slowestBidirBaudrate || (gptmrConfigured && slowestBidirBaudrate == gptmrBidirBaudrate)) {
        return;
    }

    clock_add_to_group(UART_DIR_SWITCH_GPTMR_CLOCK, BOARD_RUNNING_CORE & 0x1);
    const uint32_t gptmrFreq = clock_get_frequency(UART_DIR_SWITCH_GPTMR_CLOCK);
    if (!gptmrFreq) {
        return;
    }

    gptmr_channel_config_t config;
    const uint64_t reloadTicks = ((uint64_t) gptmrFreq * UART_MAX_FRAME_BITS
                                  + slowestBidirBaudrate - 1U) / slowestBidirBaudrate;

    gptmr_channel_get_default_config(UART_DIR_SWITCH_GPTMR, &config);
    config.reload = (reloadTicks > UINT32_MAX) ? UINT32_MAX : MAX(1U, (uint32_t) reloadTicks);

    gptmr_stop_counter(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL);
    gptmr_channel_config(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL, &config, false);
    gptmr_enable_irq(UART_DIR_SWITCH_GPTMR, GPTMR_CH_RLD_IRQ_MASK(UART_DIR_SWITCH_GPTMR_CHANNEL));

    gptmrBidirBaudrate = slowestBidirBaudrate;
    gptmrConfigured = true;

    // A runtime baud-rate change can occur while a direction switch is
    // pending.  Restart the timer with the newly calculated interval.
    if (uartQueue.count) {
        gptmr_channel_reset_count(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL);
        gptmr_start_counter(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL);
    }
}

static void uartScheduleTxComplete(uartDevice_t *uart)
{
    if (!uartQueuePush(uart)) {
        return;
    }

    gptmr_channel_reset_count(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL);
    gptmr_clear_status(UART_DIR_SWITCH_GPTMR, GPTMR_CH_RLD_STAT_MASK(UART_DIR_SWITCH_GPTMR_CHANNEL));
    gptmr_start_counter(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL);
    intc_m_enable_irq_with_priority(UART_DIR_SWITCH_GPTMR_IRQ, hpmPlicPriorityFromNvic(uart->hardware->txPriority));
}

static void enableRxIrq(const uartHardware_t *hardware)
{
    intc_m_enable_irq_with_priority(hardware->irqn, hpmPlicPriorityFromNvic(hardware->rxPriority));
}

static void uartConfigureBidirPin(IO_t io, uint32_t af, bool openDrain, serialPullMode_t pull)
{
    if (!io) {
        return;
    }

    IOConfigGPIOAF(io, (openDrain) ? IOCFG_AF_OD : IOCFG_AF_PP, af);

    const uint32_t iocIndex = IOCIndex(io);
    uint32_t padCtl = HPM_IOC->PAD[iocIndex].PAD_CTL;

    padCtl &= ~(IOC_PAD_PAD_CTL_OD_MASK | IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_PS_MASK);

    if (openDrain) {
        padCtl |= IOC_PAD_PAD_CTL_OD_SET(1);
    }
    if (pull != serialPullNone) {
        padCtl |= IOC_PAD_PAD_CTL_PE_SET(1);
        if (pull == serialPullUp) {
            padCtl |= IOC_PAD_PAD_CTL_PS_SET(1);
        }
    }

    HPM_IOC->PAD[iocIndex].PAD_CTL = padCtl;
}

uartPort_t *serialUART(uartDevice_t *uart, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    if (!uart) {
        return NULL;
    }

    const uartHardware_t *hardware = uart->hardware;

    if (!hardware) {
        return NULL;            // XXX Can't happen !?
    }

    uartPort_t *s = &uart->port;

    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;
    s->checkUsartTxOutput = checkUsartTxOutput;

    if (hardware->rcc) {
        clock_add_to_group(hardware->rcc, 0);
    }

    IO_t txIO = IOGetByTag(uart->tx.pin);
    IO_t rxIO = IOGetByTag(uart->rx.pin);

    uart->txPinState = TX_PIN_IGNORE;

    const int ownerIndex = serialOwnerIndex(s->port.identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(s->port.identifier);

    if ((mode & MODE_TX) && txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
    }
    if ((mode & MODE_RX) && rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
    }

    if (options & SERIAL_BIDIR) {
        const serialPullMode_t pull = serialOptions_pull(options);

        if ((mode & MODE_TX) && txIO) {
            uartConfigureBidirPin(txIO, uart->tx.af, !serialOptions_pushPull(options), pull);
        }
        if ((mode & MODE_RX) && rxIO) {
            uartConfigureBidirPin(rxIO, uart->rx.af, false, pull);
        }
    } else {
        if ((mode & MODE_TX) && txIO) {
            if (options & SERIAL_CHECK_TX) {
                uart->txPinState = TX_PIN_ACTIVE;
                uartTxMonitor(s);
            } else {
                IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);
            }
        }
        if ((mode & MODE_RX) && rxIO) {
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP, uart->rx.af);
        }
    }

    enableRxIrq(hardware);
    return s;
}

// --- Serial/UART ---
void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    for (size_t hindex = 0; hindex < UARTDEV_COUNT; hindex++) {

        const uartHardware_t *hardware = &uartHardware[hindex];
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t *uartdev = uartDeviceFromIdentifier(identifier);
        if (!uartdev) {
            continue;
        }
        const int resourceIndex = serialResourceIndex(identifier);
        if (resourceIndex < 0) {
            continue;
        }

        for (int pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
            if (pSerialPinConfig->ioTagRx[resourceIndex]
                && (pSerialPinConfig->ioTagRx[resourceIndex] == hardware->rxPins[pindex].pin)) {
                uartdev->rx = hardware->rxPins[pindex];
            }

            if (pSerialPinConfig->ioTagTx[resourceIndex]
                && (pSerialPinConfig->ioTagTx[resourceIndex] == hardware->txPins[pindex].pin)) {
                uartdev->tx = hardware->txPins[pindex];
            }


            // Check for swapped pins
            if (pSerialPinConfig->ioTagTx[resourceIndex]
                && (pSerialPinConfig->ioTagTx[resourceIndex] == hardware->rxPins[pindex].pin)) {
                uartdev->tx = hardware->rxPins[pindex];
            }

            if (pSerialPinConfig->ioTagRx[resourceIndex]
                && (pSerialPinConfig->ioTagRx[resourceIndex] == hardware->txPins[pindex].pin)) {
                uartdev->rx = hardware->txPins[pindex];
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
        }
    }
}

#ifdef HPM6750
#define UART1_IRQn IRQn_UART0
#define UART6_IRQn IRQn_UART5
#define UART7_IRQn IRQn_UART6
#define UART8_IRQn IRQn_UART7
const uartHardware_t uartHardware[UARTDEV_COUNT] = {

#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = (usartResource_t *) HPM_UART0_BASE,
        .rxPins = {
            {
                IO_TAG(PY7),
                .af = IOC_PY07_FUNC_CTL_UART0_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PY6),
                .af = IOC_PY06_FUNC_CTL_UART0_TXD,
            },
        },
        .rcc = clock_uart0,
        .irqn = IRQn_UART0,
        .txPriority = NVIC_PRIO_SERIALUART1,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
        .identifier = SERIAL_PORT_UART6,
        .reg = (usartResource_t *) HPM_UART5_BASE,
        .rxPins = {
            {
                IO_TAG(PD6),
                .af = IOC_PD06_FUNC_CTL_UART5_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PD7),
                .af = IOC_PD07_FUNC_CTL_UART5_TXD,
            },
        },
        .rcc = clock_uart5,
        .irqn = IRQn_UART5,
        .txPriority = NVIC_PRIO_SERIALUART6,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
#ifdef USE_UART8
    {
        .identifier = SERIAL_PORT_UART8,
        .reg = (usartResource_t *) HPM_UART7_BASE,
        .rxPins = {
            {
                IO_TAG(PE30),
                .af = IOC_PE30_FUNC_CTL_UART7_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PE31),
                .af = IOC_PE31_FUNC_CTL_UART7_TXD,
            },
        },
        .rcc = clock_uart7,
        .irqn = IRQn_UART7,
        .txPriority = NVIC_PRIO_SERIALUART8,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    }
#endif

};
#endif

#ifdef HPM6360
#define UART1_IRQn IRQn_UART0
#define UART2_IRQn IRQn_UART1
#define UART3_IRQn IRQn_UART2
#define UART4_IRQn IRQn_UART3
#define UART5_IRQn IRQn_UART4
#define UART6_IRQn IRQn_UART5
#define UART7_IRQn IRQn_UART6
#define UART8_IRQn IRQn_UART7
#define UART1_CLOCK clock_uart0
#define UART2_CLOCK clock_uart1
#define UART3_CLOCK clock_uart2
#define UART4_CLOCK clock_uart3
#define UART5_CLOCK clock_uart4
#define UART6_CLOCK clock_uart5
#define UART7_CLOCK clock_uart6
#define UART8_CLOCK clock_uart7
const uartHardware_t uartHardware[UARTDEV_COUNT] = {

#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = (usartResource_t *) HPM_UART0_BASE,
        .rxPins = {
            {
                IO_TAG(PA31),
                .af = IOC_PA31_FUNC_CTL_UART0_RXD,
            },
            {
                IO_TAG(PB23),
                .af = IOC_PB23_FUNC_CTL_UART0_RXD,
            },
            {
                IO_TAG(PY7),
                .af = IOC_PY07_FUNC_CTL_UART0_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PA30),
                .af = IOC_PA30_FUNC_CTL_UART0_TXD,
            },
            {
                IO_TAG(PB22),
                .af = IOC_PB22_FUNC_CTL_UART0_TXD,
            },
            {
                IO_TAG(PY6),
                .af = IOC_PY06_FUNC_CTL_UART0_TXD,
            },
        },
        .rcc = UART1_CLOCK,
        .irqn = UART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif
#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_UART2,
        .reg = (usartResource_t *) HPM_UART1_BASE,
        .rxPins = {
            {
                IO_TAG(PA1),
                .af = IOC_PA01_FUNC_CTL_UART1_RXD,
            },
            {
                IO_TAG(PB1),
                .af = IOC_PB01_FUNC_CTL_UART1_RXD,
            },
            {
                IO_TAG(PB25),
                .af = IOC_PB25_FUNC_CTL_UART1_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PA0),
                .af = IOC_PA00_FUNC_CTL_UART1_TXD,
            },
            {
                IO_TAG(PB0),
                .af = IOC_PB00_FUNC_CTL_UART1_TXD,
            },
            {
                IO_TAG(PB24),
                .af = IOC_PB24_FUNC_CTL_UART1_TXD,
            },
        },
        .rcc = UART2_CLOCK,
        .irqn = UART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif
#ifdef USE_UART3
    {
        .identifier = SERIAL_PORT_UART3,
        .reg = (usartResource_t *) HPM_UART2_BASE,
        .rxPins = {
            {
                IO_TAG(PA3),
                .af = IOC_PA03_FUNC_CTL_UART2_RXD,
            },
            {
                IO_TAG(PB27),
                .af = IOC_PB27_FUNC_CTL_UART2_RXD,
            },
            {
                IO_TAG(PC27),
                .af = IOC_PC27_FUNC_CTL_UART2_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PA2),
                .af = IOC_PA02_FUNC_CTL_UART2_TXD,
            },
            {
                IO_TAG(PB26),
                .af = IOC_PB26_FUNC_CTL_UART2_TXD,
            },
            {
                IO_TAG(PC26),
                .af = IOC_PC26_FUNC_CTL_UART2_TXD,
            },
        },
        .rcc = UART3_CLOCK,
        .irqn = UART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
#ifdef USE_UART4
    {
        .identifier = SERIAL_PORT_UART4,
        .reg = (usartResource_t *) HPM_UART3_BASE,
        .rxPins = {
            {
                IO_TAG(PA5),
                .af = IOC_PA05_FUNC_CTL_UART3_RXD,
            },
            {
                IO_TAG(PB29),
                .af = IOC_PB29_FUNC_CTL_UART3_RXD,
            },
            {
                IO_TAG(PZ1),
                .af = IOC_PZ01_FUNC_CTL_UART3_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PA4),
                .af = IOC_PA04_FUNC_CTL_UART3_TXD,
            },
            {
                IO_TAG(PB28),
                .af = IOC_PB28_FUNC_CTL_UART3_TXD,
            },
            {
                IO_TAG(PZ0),
                .af = IOC_PZ00_FUNC_CTL_UART3_TXD,
            },
        },
        .rcc = UART4_CLOCK,
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif
#ifdef USE_UART5
    {
        .identifier = SERIAL_PORT_UART5,
        .reg = (usartResource_t *) HPM_UART4_BASE,
        .rxPins = {
            {
                IO_TAG(PC7),
                .af = IOC_PC07_FUNC_CTL_UART4_RXD,
            },
            {
                IO_TAG(PZ3),
                .af = IOC_PZ03_FUNC_CTL_UART4_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PC6),
                .af = IOC_PC06_FUNC_CTL_UART4_TXD,
            },
            {
                IO_TAG(PZ2),
                .af = IOC_PZ02_FUNC_CTL_UART4_TXD,
            },
        },
        .rcc = UART5_CLOCK,
        .irqn = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif
#ifdef USE_UART6
    {
        .identifier = SERIAL_PORT_UART6,
        .reg = (usartResource_t *) HPM_UART5_BASE,
        .rxPins = {
            {
                IO_TAG(PC9),
                .af = IOC_PC09_FUNC_CTL_UART5_RXD,
            },
            {
                IO_TAG(PA17),
                .af = IOC_PA17_FUNC_CTL_UART5_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PC8),
                .af = IOC_PC08_FUNC_CTL_UART5_TXD,
            },
            {
                IO_TAG(PA16),
                .af = IOC_PA16_FUNC_CTL_UART5_TXD,
            },
        },
        .rcc = UART6_CLOCK,
        .irqn = UART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif
#ifdef USE_UART7
    {
        .identifier = SERIAL_PORT_UART7,
        .reg = (usartResource_t *) HPM_UART6_BASE,
        .rxPins = {
            {
                IO_TAG(PA19),
                .af = IOC_PA19_FUNC_CTL_UART6_RXD,
            },
            {
                IO_TAG(PB11),
                .af = IOC_PB11_FUNC_CTL_UART6_RXD,
            },
            {
                IO_TAG(PC11),
                .af = IOC_PC11_FUNC_CTL_UART6_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PA18),
                .af = IOC_PA18_FUNC_CTL_UART6_TXD,
            },
            {
                IO_TAG(PC10),
                .af = IOC_PC10_FUNC_CTL_UART6_TXD,
            },
        },
        .rcc = UART7_CLOCK,
        .irqn = UART7_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART7,
        .rxPriority = NVIC_PRIO_SERIALUART7,
        .txBuffer = uart7TxBuffer,
        .rxBuffer = uart7RxBuffer,
        .txBufferSize = sizeof(uart7TxBuffer),
        .rxBufferSize = sizeof(uart7RxBuffer),
    },
#endif
#ifdef USE_UART8
    {
        .identifier = SERIAL_PORT_UART8,
        .reg = (usartResource_t *) HPM_UART7_BASE,
        .rxPins = {
            {
                IO_TAG(PA21),
                .af = IOC_PA21_FUNC_CTL_UART7_RXD,
            },
            {
                IO_TAG(PB13),
                .af = IOC_PB13_FUNC_CTL_UART7_RXD,
            },
            {
                IO_TAG(PC13),
                .af = IOC_PC13_FUNC_CTL_UART7_RXD,
            },
            {
                IO_TAG(PY5),
                .af = IOC_PY05_FUNC_CTL_UART7_RXD,
            },
        },
        .txPins = {
            {
                IO_TAG(PA20),
                .af = IOC_PA20_FUNC_CTL_UART7_TXD,
            },
            {
                IO_TAG(PB12),
                .af = IOC_PB12_FUNC_CTL_UART7_TXD,
            },
            {
                IO_TAG(PC12),
                .af = IOC_PC12_FUNC_CTL_UART7_TXD,
            },
            {
                IO_TAG(PY4),
                .af = IOC_PY04_FUNC_CTL_UART7_TXD,
            },
        },
        .rcc = UART8_CLOCK,
        .irqn = UART8_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART8,
        .rxPriority = NVIC_PRIO_SERIALUART8,
        .txBuffer = uart8TxBuffer,
        .rxBuffer = uart8RxBuffer,
        .txBufferSize = sizeof(uart8TxBuffer),
        .rxBufferSize = sizeof(uart8RxBuffer),
    }
#endif
};
#endif

SDK_DECLARE_EXT_ISR_M(UART_DIR_SWITCH_GPTMR_IRQ, tickMsIsr)
void tickMsIsr(void)
{
    if (gptmr_check_status(UART_DIR_SWITCH_GPTMR, GPTMR_CH_RLD_STAT_MASK(UART_DIR_SWITCH_GPTMR_CHANNEL))) {
        gptmr_clear_status(UART_DIR_SWITCH_GPTMR, GPTMR_CH_RLD_STAT_MASK(UART_DIR_SWITCH_GPTMR_CHANNEL));

        const uint8_t pending = uartQueue.count;

        for (uint8_t i = 0; i < pending; i++) {
            uartDevice_t *uart = uartQueuePop();
            if (!uart) {
                break;
            }

            UART_Type *uartPtr = (UART_Type *) uart->port.USARTx;
            if (!(uartPtr->LSR & UART_LSR_TEMT_MASK)) {
                uartQueuePush(uart);
                continue;
            }

            if (uart->port.port.options & SERIAL_BIDIR) {
                const IO_t txIO = IOGetByTag(uart->tx.pin);
                const IO_t rxIO = IOGetByTag(uart->rx.pin);

                if (txIO) {
                    HPM_IOC->PAD[IOCIndex(txIO)].FUNC_CTL = 0;
                }
                if (rxIO) {
                    HPM_IOC->PAD[IOCIndex(rxIO)].FUNC_CTL = uart->rx.af;
                }
            }
            if (uart->port.port.options & SERIAL_CHECK_TX) {
                uartTxMonitor(&uart->port);
            }
        }

        if (uartQueue.count == 0) {
            gptmr_stop_counter(UART_DIR_SWITCH_GPTMR, UART_DIR_SWITCH_GPTMR_CHANNEL);
            intc_m_disable_irq(UART_DIR_SWITCH_GPTMR_IRQ);
        }
    }
}

void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
#ifdef USE_INVERTER
    enableInverter(uartPort->port.identifier, uartPort->port.options & SERIAL_INVERTED);
#else
    UNUSED(uartPort);
#endif
}

bool checkUsartTxOutput(uartPort_t *uartPort)
{
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if (uart->txPinState == TX_PIN_MONITOR && txIO) {
        if (!IORead(txIO)) {
            return false;
        }

        uart->txPinState = TX_PIN_ACTIVE;
        IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->tx.af);
    }

    return true;
}

void uartTxMonitor(uartPort_t *uartPort)
{
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);

    if (uart->txPinState == TX_PIN_ACTIVE) {
        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(IOGetByTag(uart->tx.pin), IOCFG_IPU);
    }
}

void uartReconfigure(uartPort_t *uartPort)
{
    uart_config_t config = { 0 };
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);

    uart_default_config((UART_Type *) uartPort->USARTx, &config);
    config.src_freq_in_hz = clock_get_frequency(uart->hardware->rcc);
    config.baudrate = uartPort->port.baudRate;
    config.num_of_stop_bits = (uartPort->port.options & SERIAL_STOPBITS_2)
                              ? stop_bits_2 : stop_bits_1;
    config.parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? parity_even : parity_none;

#if defined(HPM_IP_FEATURE_UART_RX_EN) && (HPM_IP_FEATURE_UART_RX_EN == 1)
    config.rx_enable = (uartPort->port.mode & MODE_RX) != 0;
#endif
    if (uart_init((UART_Type *) uartPort->USARTx, &config) != status_success) {
        // uart_init() has already disabled all peripheral interrupts. Leave
        // the unusable UART inactive, matching the common STM32 behavior.
        return;
    }

    uartConfigureExternalPinInversion(uartPort);
    timerConfig();

    if (uartPort->port.options & SERIAL_BIDIR) {
        const IO_t txIO = IOGetByTag(uart->tx.pin);

        if ((uartPort->port.mode & MODE_TX) && txIO) {
            HPM_IOC->PAD[IOCIndex(txIO)].FUNC_CTL = 0;
        }
    }
    if (uartPort->port.mode & MODE_RX) {
        uart_enable_irq((UART_Type *) uartPort->USARTx, uart_intr_rx_data_avail_or_timeout | uart_intr_rx_line_stat);
    }
}

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
    // HPMicro UART: enabling TXE interrupt alone does not trigger an interrupt
    // while the TX FIFO is empty.  We must kick-start the transmission by
    // writing the first byte when THR is empty, then use the TXE interrupt for
    // the rest.  The interrupt may be disabled while the shift register or TX
    // FIFO is still busy, so its enable state alone does not mean THR is safe
    // to write.
    uartDevice_t *uart = container_of(uartPort, uartDevice_t, port);
    UART_Type *uartPtr = (UART_Type *) uartPort->USARTx;
    const uint32_t irqState = disable_global_irq(CSR_MSTATUS_MIE_MASK);

    if (uart_get_enabled_irq(uartPtr) & uart_intr_tx_slot_avail) {
        // TXE already enabled; nothing to do
        enable_global_irq(irqState);
        return;
    }

    if (uartPort->port.txBufferTail == uartPort->port.txBufferHead) {
        enable_global_irq(irqState);
        return;
    }

    // A previous frame may still be waiting for TEMT before its RX switch.
    // New data keeps the port in TX mode and cancels that pending switch.
    uartQueueRemove(uart);

    if (uartPort->port.options & SERIAL_BIDIR) {
        const IO_t txIO = IOGetByTag(uart->tx.pin);
        const IO_t rxIO = IOGetByTag(uart->rx.pin);

        if (txIO) {
            HPM_IOC->PAD[IOCIndex(txIO)].FUNC_CTL = uart->tx.af;
        }
        if (rxIO) {
            HPM_IOC->PAD[IOCIndex(rxIO)].FUNC_CTL = 0;
        }
    }


    //send a byte to trigger uart_intr_tx_slot_avail interrupt, otherwise, the interrupt will not be triggered if the TX FIFO is avail
    if (uart_send_byte(uartPtr, uartPort->port.txBuffer[uartPort->port.txBufferTail]) == status_success) {
        uartPort->port.txBufferTail = (uartPort->port.txBufferTail + 1) % uartPort->port.txBufferSize;
    }
    uart_enable_irq(uartPtr, uart_intr_tx_slot_avail);

    enable_global_irq(irqState);
}

FAST_CODE void uart_isr(uartDevice_t *uart)
{
    if (!uart) {
        return;
    }
    uartPort_t *s = &(uart->port);
    UART_Type *uartPtr = (UART_Type *) (s->USARTx);

    uint8_t irq_id = uart_get_irq_id(uartPtr);
    if (irq_id == uart_intr_id_rx_data_avail || irq_id == uart_intr_id_rx_timeout
        || irq_id == uart_intr_id_rx_line_stat) {
        volatile uartErrorCounters_t *errorCounters = &uartErrorCounters[uart - uartDevice];

        for (;;) {
            // LSR error bits describe the byte at the top of the RX FIFO and
            // are cleared by reading LSR, so capture them before reading RBR.
            const uint32_t lineStatus = uartPtr->LSR;

            errorCounters->overrun += (lineStatus & UART_LSR_OE_MASK) != 0;
            errorCounters->parity += (lineStatus & UART_LSR_PE_MASK) != 0;
            errorCounters->framing += (lineStatus & UART_LSR_FE_MASK) != 0;
            errorCounters->lineBreak += (lineStatus & UART_LSR_LBREAK_MASK) != 0;

            if (!(lineStatus & UART_LSR_DR_MASK)) {
                break;
            }

            const uint8_t c = uart_read_byte(uartPtr);

            if (s->port.rxCallback) {
                s->port.rxCallback(c, s->port.rxCallbackData);
            } else {
                s->port.rxBuffer[s->port.rxBufferHead] = c;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }
    if (irq_id == uart_intr_id_tx_slot_avail) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            uart_write_byte((UART_Type *) (s->USARTx), s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            uart_disable_irq((UART_Type *) (s->USARTx), uart_intr_tx_slot_avail);
            if (s->port.options & (SERIAL_BIDIR | SERIAL_CHECK_TX)) {
                uartScheduleTxComplete(uart);
            }
        }
    }
}

#ifdef USE_UART1
void uart_isr0(void)
{
    uart_isr(&uartDevice[UARTDEV_1]);
}

SDK_DECLARE_EXT_ISR_M(UART1_IRQn, uart_isr0)
#endif
#ifdef USE_UART2
void uart_isr1(void)
{
    uart_isr(&uartDevice[UARTDEV_2]);
}

SDK_DECLARE_EXT_ISR_M(UART2_IRQn, uart_isr1)
#endif
#ifdef USE_UART3
void uart_isr2(void)
{
    uart_isr(&uartDevice[UARTDEV_3]);
}

SDK_DECLARE_EXT_ISR_M(UART3_IRQn, uart_isr2)
#endif
#ifdef USE_UART4
void uart_isr3(void)
{
    uart_isr(&uartDevice[UARTDEV_4]);
}

SDK_DECLARE_EXT_ISR_M(UART4_IRQn, uart_isr3)
#endif
#ifdef USE_UART5
void uart_isr4(void)
{
    uart_isr(&uartDevice[UARTDEV_5]);
}

SDK_DECLARE_EXT_ISR_M(UART5_IRQn, uart_isr4)
#endif
#ifdef USE_UART6
void uart_isr5(void)
{
    uart_isr(&uartDevice[UARTDEV_6]);
}

SDK_DECLARE_EXT_ISR_M(UART6_IRQn, uart_isr5)
#endif
#ifdef USE_UART7
void uart_isr6(void)
{
    uart_isr(&uartDevice[UARTDEV_7]);
}

SDK_DECLARE_EXT_ISR_M(UART7_IRQn, uart_isr6)
#endif
#ifdef USE_UART8
void uart_isr7(void)
{
    uart_isr(&uartDevice[UARTDEV_8]);
}

SDK_DECLARE_EXT_ISR_M(UART8_IRQn, uart_isr7)
#endif
#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    (void) s;
}

#endif
#endif                          // USE_UART
