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

/*
 * PIOUART ports on FlexIO, 8N1 or 8N2: one shifter and timer per direction in dual 8 bit
 * baud mode, baud = clock / (2 * divisor * (TIMCMP[7:0] + 1)) with divisor 1, 16 or 256.
 * The RT10xx timer has no divider, so it has no PIOUART. Parity and inverted
 * SERIAL_BIDIR are refused.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if SERIAL_PIOUART_COUNT > 0

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "platform/nxp_clock.h"

#include "dshot_flexio.h"
#include "iomux_nxp.h"
#include "iomux_signal_def.h"

// Port n is pioUartDevice[n], so the ports are used in order from PIOUART0.
STATIC_ASSERT(SERIAL_PIOUART_COUNT == SERIAL_PIOUART_MAX, flexio_uart_ports_must_be_contiguous_from_zero);

// Register field values from the reference manual.
#define SHIFTER_MODE_RECEIVE            1u
#define SHIFTER_MODE_TRANSMIT           2u
#define SHIFTER_INPUT_FROM_PIN          0u
#define SHIFTER_START_LOW               2u
#define SHIFTER_STOP_HIGH               3u
#define SHIFTER_TIMER_POLARITY_POSEDGE  0u
#define SHIFTER_TIMER_POLARITY_NEGEDGE  1u

#define PIN_CONFIG_OUTPUT_DISABLED      0u
#define PIN_CONFIG_OUTPUT               3u
#define PIN_ACTIVE_LOW                  1u

#define TIMER_MODE_DUAL_8BIT_BAUD       1u
#define TIMER_OUT_ONE_NOT_RESET         0u
#define TIMER_OUT_ONE_ON_RESET          2u
#define TIMER_RESET_NEVER               0u
#define TIMER_RESET_ON_PIN_RISING_EDGE  4u
#define TIMER_DISABLE_ON_COMPARE        2u
#define TIMER_ENABLE_ON_TRIGGER_HIGH    2u
#define TIMER_ENABLE_ON_PIN_RISING_EDGE 4u
#define TIMER_STOP_ON_TIMER_DISABLE     2u
#define TIMER_START_ENABLED             1u
#define TIMER_TRIGGER_POLARITY_LOW      1u
#define TIMER_TRIGGER_SOURCE_INTERNAL   1u

#define TRIGGER_SHIFTER_STATUS(n)       ((4u * (n)) + 1u)
#define TRIGGER_FLEXIO_PIN(n)           (2u * (n))

// Rate error budget, in parts per thousand of the wanted rate, as for the LPUART.
#define FLEXIO_UART_BAUD_ERROR_PPT      5

#define FLEXIO_UART_DATA_BITS           8u

// The timer decrement sources that count the FlexIO clock, as TIMDEC values.
static const struct {
    uint8_t timdec;
    uint16_t divisor;
} flexioUartClockDividers[] = {
    { 0, 1 },
    { 4, 16 },
    { 5, 256 },
};

typedef struct flexioUartBlock_s {
    FLEXIO_Type *base;
    IRQn_Type irq;
    nxpClock_e clock;
} flexioUartBlock_t;

static const flexioUartBlock_t flexioUartBlocks[] = {
    { FLEXIO1, FLEXIO1_IRQn, NXP_CLK_FLEXIO1 },
    { FLEXIO2, FLEXIO2_IRQn, NXP_CLK_FLEXIO2 },
};

typedef struct flexioUartChannel_s {
    uint8_t shifter;
    uint8_t timer;
    bool held;
} flexioUartChannel_t;

typedef struct flexioUartPort_s {
    const flexioUartBlock_t *block;     // NULL while the port has no usable pins
    const nxpFlexioPin_t *rxPad;
    const nxpFlexioPin_t *txPad;
    uint8_t rxPin;                      // FlexIO pin the receiver listens on
    flexioUartChannel_t rx;
    flexioUartChannel_t tx;
} flexioUartPort_t;

static flexioUartPort_t flexioUartPorts[SERIAL_PIOUART_COUNT];

// Receive errors per port, overruns and bad stop bits alike, for a debugger to read.
uint32_t flexioUartRxErrors[SERIAL_PIOUART_COUNT];

static const uartHardware_t flexioUartHardware[SERIAL_PIOUART_COUNT] = {
#ifdef USE_PIOUART0
    {
        .identifier = SERIAL_PORT_PIOUART0,
        .txBuffer = uartPio0TxBuffer,
        .rxBuffer = uartPio0RxBuffer,
        .txBufferSize = sizeof(uartPio0TxBuffer),
        .rxBufferSize = sizeof(uartPio0RxBuffer),
    },
#endif
#ifdef USE_PIOUART1
    {
        .identifier = SERIAL_PORT_PIOUART1,
        .txBuffer = uartPio1TxBuffer,
        .rxBuffer = uartPio1RxBuffer,
        .txBufferSize = sizeof(uartPio1TxBuffer),
        .rxBufferSize = sizeof(uartPio1RxBuffer),
    },
#endif
};

#ifndef USE_DSHOT
uint32_t dshotFlexioChannelMask(const FLEXIO_Type *base)
{
    UNUSED(base);
    return 0;
}

FAST_IRQ_HANDLER void FLEXIO1_IRQHandler(void)
{
    flexioUartIrqHandler(FLEXIO1);
}

FAST_IRQ_HANDLER void FLEXIO2_IRQHandler(void)
{
    flexioUartIrqHandler(FLEXIO2);
}
#endif

static inline flexioUartPort_t *flexioUartPortOf(uartPort_t *s)
{
    return &flexioUartPorts[container_of(s, uartDevice_t, port) - pioUartDevice];
}

// The TIMDEC value and TIMCMP[7:0] for a rate, false when none is within the
// budget. The smallest divisor wins: it finds the start bit to the finest step.
static bool flexioUartDivider(uint32_t clockHz, uint32_t baudRate, uint32_t *timdec, uint32_t *compare)
{
    if (baudRate == 0) {
        return false;
    }

    for (unsigned i = 0; i < ARRAYLEN(flexioUartClockDividers); i++) {
        const uint32_t step = 2u * flexioUartClockDividers[i].divisor;
        const uint64_t ticksPerBit = ((uint64_t)clockHz + (uint64_t)baudRate * step / 2) / ((uint64_t)baudRate * step);
        if (ticksPerBit < 1 || ticksPerBit > 256) {
            continue;
        }

        const uint32_t actual = clockHz / (step * (uint32_t)ticksPerBit);
        const uint32_t error = actual > baudRate ? actual - baudRate : baudRate - actual;
        if (error <= baudRate * FLEXIO_UART_BAUD_ERROR_PPT / 1000u) {
            *timdec = flexioUartClockDividers[i].timdec;
            *compare = (uint32_t)ticksPerBit - 1u;
            return true;
        }
    }

    return false;
}

// Shifters and timers DShot and every port on the instance hold.
static void flexioUartHeld(const FLEXIO_Type *base, uint32_t *shifters, uint32_t *timers)
{
    *shifters = *timers = dshotFlexioChannelMask(base);

    for (unsigned i = 0; i < ARRAYLEN(flexioUartPorts); i++) {
        const flexioUartPort_t *port = &flexioUartPorts[i];
        if (!port->block || port->block->base != base) {
            continue;
        }
        const flexioUartChannel_t *channels[] = { &port->rx, &port->tx };
        for (unsigned c = 0; c < ARRAYLEN(channels); c++) {
            if (channels[c]->held) {
                *shifters |= 1u << channels[c]->shifter;
                *timers |= 1u << channels[c]->timer;
            }
        }
    }
}

static int flexioUartHighestFree(uint32_t held, unsigned count)
{
    const uint32_t free = ~held & ((1u << count) - 1u);
    return free ? 31 - __builtin_clz(free) : -1;
}

static bool flexioUartTakeChannel(flexioUartChannel_t *channel, const FLEXIO_Type *base)
{
    if (channel->held) {
        return true;
    }

    uint32_t shifters;
    uint32_t timers;
    flexioUartHeld(base, &shifters, &timers);

    const int shifter = flexioUartHighestFree(shifters, FLEXIO_SHIFTCTL_COUNT);
    const int timer = flexioUartHighestFree(timers, FLEXIO_TIMCTL_COUNT);
    if (shifter < 0 || timer < 0) {
        return false;
    }

    channel->shifter = shifter;
    channel->timer = timer;
    channel->held = true;

    return true;
}

void flexioUartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    for (unsigned i = 0; i < ARRAYLEN(flexioUartHardware); i++) {
        const uartHardware_t *hardware = &flexioUartHardware[i];
        const int resourceIndex = serialResourceIndex(hardware->identifier);
        if (resourceIndex < 0) {
            continue;
        }

        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];

        // Every configured pin has to be on the same instance, or the port stays off.
        for (unsigned n = 0; n < ARRAYLEN(flexioUartBlocks); n++) {
            const nxpFlexioPin_t *rxPad = NXP_FLEXIO_PIN(nxpFlexioPads, n + 1, cfgRx);
            const nxpFlexioPin_t *txPad = NXP_FLEXIO_PIN(nxpFlexioPads, n + 1, cfgTx);

            if ((rxPad || txPad) && (rxPad || !cfgRx) && (txPad || !cfgTx)) {
                flexioUartPort_t *port = &flexioUartPorts[i];
                port->block = &flexioUartBlocks[n];
                port->rxPad = rxPad;
                port->txPad = txPad;

                uartDevice_t *uartdev = &pioUartDevice[i];
                uartdev->rx.pin = cfgRx;
                uartdev->tx.pin = cfgTx;
                uartdev->hardware = hardware;
                break;
            }
        }
    }
}

uartPort_t *flexioUartOpen(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    // Channels follow the pins, as on the PICO, so a later mode change finds them.
    UNUSED(mode);

    uartPort_t *s = &uartdev->port;
    flexioUartPort_t *port = flexioUartPortOf(s);
    const uartHardware_t *hardware = uartdev->hardware;
    FLEXIO_Type *base = port->block->base;

    if ((options & SERIAL_PARITY_EVEN) || ((options & SERIAL_BIDIR) && (options & SERIAL_INVERTED))) {
        return NULL;
    }

#if defined(NXP_FLEXIO_UART_CLOCK_HZ)
    // Motors keep the root DShot needs; the rate check below decides for them.
    if (!dshotFlexioChannelMask(base) && nxpClockHz(port->block->clock) != NXP_FLEXIO_UART_CLOCK_HZ) {
        nxpClockFlexioUartRoot(port->block->clock);
    }
#endif

    uint32_t timdec;
    uint32_t compare;
    if (!flexioUartDivider(nxpClockHz(port->block->clock), baudRate, &timdec, &compare)) {
        return NULL;
    }

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);
    const bool halfDuplex = (options & SERIAL_BIDIR) && txIO;

    const flexioUartPort_t before = *port;
    if ((txIO && !flexioUartTakeChannel(&port->tx, base))
        || ((rxIO || halfDuplex) && !flexioUartTakeChannel(&port->rx, base))) {
        *port = before;
        return NULL;
    }

    const serialPortIdentifier_e identifier = s->port.identifier;
    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1
    const unsigned pull = ((const unsigned[]){ IOCFG_PULL_NONE, IOCFG_PULL_DOWN, IOCFG_PULL_UP })[serialOptions_pull(options)];

    if (txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        nxpIomuxConfigPin(&port->txPad->mux, halfDuplex ? IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_FAST, pull, 1, 1) : IOCFG_AF_PP);
    }

    if (rxIO && !halfDuplex) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        nxpIomuxConfigPin(&port->rxPad->mux, IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_FAST, pull, 0, 1));
    }

    const nxpFlexioPin_t *rxPad = halfDuplex ? port->txPad : port->rxPad;
    port->rxPin = rxPad ? rxPad->flexioPin : 0;

    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;
    uartdev->txPinState = TX_PIN_IGNORE;

    // Motors are set up before any port opens, so DShot's block reset never lands on an open port.
    nxpClockGateEnable(port->block->clock);
    base->CTRL |= FLEXIO_CTRL_FLEXEN_MASK;

    NVIC_SetPriority(port->block->irq, NVIC_PRIORITY_LEVEL(DSHOT_FLEXIO_PRIORITY));
    NVIC_EnableIRQ(port->block->irq);

    return s;
}

static inline uint32_t flexioUartShifterBit(const flexioUartChannel_t *channel)
{
    return channel->held ? 1u << channel->shifter : 0;
}

void flexioUartReconfigure(uartPort_t *s)
{
    const flexioUartPort_t *port = flexioUartPortOf(s);
    FLEXIO_Type *base = port->block->base;
    const portOptions_e options = s->port.options;
    const uint32_t rxBit = flexioUartShifterBit(&port->rx);
    const uint32_t txBit = flexioUartShifterBit(&port->tx);

    // SHIFTSIEN is shared with DShot, which changes it from its interrupt.
    ATOMIC_BLOCK(DSHOT_FLEXIO_PRIORITY) {
        base->SHIFTSIEN &= ~(rxBit | txBit);
        base->SHIFTEIEN &= ~rxBit;
    }

    // A shifter and timer are disabled before they are reprogrammed.
    if (port->tx.held) {
        base->TIMCTL[port->tx.timer] = 0;
        base->SHIFTCTL[port->tx.shifter] = 0;
    }
    if (port->rx.held) {
        base->TIMCTL[port->rx.timer] = 0;
        base->SHIFTCTL[port->rx.shifter] = 0;
    }

    // A rate set later that cannot be reached leaves the port off.
    uint32_t timdec;
    uint32_t compare;
    if (!flexioUartDivider(nxpClockHz(port->block->clock), s->port.baudRate, &timdec, &compare)) {
        return;
    }

    const uint32_t pinPolarity = (options & SERIAL_INVERTED) ? PIN_ACTIVE_LOW : 0u;

    if (port->tx.held) {
        const unsigned shifter = port->tx.shifter;
        const unsigned timer = port->tx.timer;
        const unsigned pin = port->txPad->flexioPin;
        // A second stop bit goes out as a ninth data bit that is always one.
        const uint32_t bits = FLEXIO_UART_DATA_BITS + ((options & SERIAL_STOPBITS_2) ? 1u : 0u);

        base->SHIFTCFG[shifter] = FLEXIO_SHIFTCFG_INSRC(SHIFTER_INPUT_FROM_PIN)
                                | FLEXIO_SHIFTCFG_SSTOP(SHIFTER_STOP_HIGH)
                                | FLEXIO_SHIFTCFG_SSTART(SHIFTER_START_LOW);
        base->SHIFTCTL[shifter] = FLEXIO_SHIFTCTL_TIMSEL(timer)
                                | FLEXIO_SHIFTCTL_TIMPOL(SHIFTER_TIMER_POLARITY_POSEDGE)
                                | FLEXIO_SHIFTCTL_PINCFG(PIN_CONFIG_OUTPUT)
                                | FLEXIO_SHIFTCTL_PINSEL(pin)
                                | FLEXIO_SHIFTCTL_PINPOL(pinPolarity)
                                | FLEXIO_SHIFTCTL_SMOD(SHIFTER_MODE_TRANSMIT);
        base->TIMCMP[timer] = ((bits * 2u - 1u) << 8) | compare;
        base->TIMCFG[timer] = FLEXIO_TIMCFG_TIMOUT(TIMER_OUT_ONE_NOT_RESET)
                            | FLEXIO_TIMCFG_TIMDEC(timdec)
                            | FLEXIO_TIMCFG_TIMRST(TIMER_RESET_NEVER)
                            | FLEXIO_TIMCFG_TIMDIS(TIMER_DISABLE_ON_COMPARE)
                            | FLEXIO_TIMCFG_TIMENA(TIMER_ENABLE_ON_TRIGGER_HIGH)
                            | FLEXIO_TIMCFG_TSTOP(TIMER_STOP_ON_TIMER_DISABLE)
                            | FLEXIO_TIMCFG_TSTART(TIMER_START_ENABLED);
        // The trigger is the shifter status flag, active low: a full buffer starts a character.
        base->TIMCTL[timer] = FLEXIO_TIMCTL_TRGSEL(TRIGGER_SHIFTER_STATUS(shifter))
                            | FLEXIO_TIMCTL_TRGPOL(TIMER_TRIGGER_POLARITY_LOW)
                            | FLEXIO_TIMCTL_TRGSRC(TIMER_TRIGGER_SOURCE_INTERNAL)
                            | FLEXIO_TIMCTL_PINCFG(PIN_CONFIG_OUTPUT_DISABLED)
                            | FLEXIO_TIMCTL_PINSEL(pin)
                            | FLEXIO_TIMCTL_TIMOD(TIMER_MODE_DUAL_8BIT_BAUD);
    }

    if (port->rx.held) {
        const unsigned shifter = port->rx.shifter;
        const unsigned timer = port->rx.timer;
        const unsigned pin = port->rxPin;

        base->SHIFTCFG[shifter] = FLEXIO_SHIFTCFG_INSRC(SHIFTER_INPUT_FROM_PIN)
                                | FLEXIO_SHIFTCFG_SSTOP(SHIFTER_STOP_HIGH)
                                | FLEXIO_SHIFTCFG_SSTART(SHIFTER_START_LOW);
        base->SHIFTCTL[shifter] = FLEXIO_SHIFTCTL_TIMSEL(timer)
                                | FLEXIO_SHIFTCTL_TIMPOL(SHIFTER_TIMER_POLARITY_NEGEDGE)
                                | FLEXIO_SHIFTCTL_PINCFG(PIN_CONFIG_OUTPUT_DISABLED)
                                | FLEXIO_SHIFTCTL_PINSEL(pin)
                                | FLEXIO_SHIFTCTL_PINPOL(pinPolarity)
                                | FLEXIO_SHIFTCTL_SMOD(SHIFTER_MODE_RECEIVE);
        base->TIMCMP[timer] = ((FLEXIO_UART_DATA_BITS * 2u - 1u) << 8) | compare;
        base->TIMCFG[timer] = FLEXIO_TIMCFG_TIMOUT(TIMER_OUT_ONE_ON_RESET)
                            | FLEXIO_TIMCFG_TIMDEC(timdec)
                            | FLEXIO_TIMCFG_TIMRST(TIMER_RESET_ON_PIN_RISING_EDGE)
                            | FLEXIO_TIMCFG_TIMDIS(TIMER_DISABLE_ON_COMPARE)
                            | FLEXIO_TIMCFG_TIMENA(TIMER_ENABLE_ON_PIN_RISING_EDGE)
                            | FLEXIO_TIMCFG_TSTOP(TIMER_STOP_ON_TIMER_DISABLE)
                            | FLEXIO_TIMCFG_TSTART(TIMER_START_ENABLED);
        // The timer pin is active low on a plain line, so its rising edge is the start bit.
        base->TIMCTL[timer] = FLEXIO_TIMCTL_TRGSEL(TRIGGER_FLEXIO_PIN(pin))
                            | FLEXIO_TIMCTL_TRGSRC(TIMER_TRIGGER_SOURCE_INTERNAL)
                            | FLEXIO_TIMCTL_PINCFG(PIN_CONFIG_OUTPUT_DISABLED)
                            | FLEXIO_TIMCTL_PINSEL(pin)
                            | FLEXIO_TIMCTL_PINPOL(pinPolarity ? 0u : PIN_ACTIVE_LOW)
                            | FLEXIO_TIMCTL_TIMOD(TIMER_MODE_DUAL_8BIT_BAUD);
    }

    uint32_t enable = 0;
    if (s->port.mode & MODE_RX) {
        enable |= rxBit;
    }
    if ((s->port.mode & MODE_TX) && s->port.txBufferHead != s->port.txBufferTail) {
        enable |= txBit;
    }

    ATOMIC_BLOCK(DSHOT_FLEXIO_PRIORITY) {
        base->SHIFTSIEN |= enable;
        if (enable & rxBit) {
            base->SHIFTERR = rxBit;
            base->SHIFTEIEN |= rxBit;
        }
    }
}

FAST_CODE void flexioUartEnableTxInterrupt(uartPort_t *s)
{
    const flexioUartPort_t *port = flexioUartPortOf(s);

    ATOMIC_BLOCK(DSHOT_FLEXIO_PRIORITY) {
        port->block->base->SHIFTSIEN |= flexioUartShifterBit(&port->tx);
    }
}

FAST_IRQ_HANDLER void flexioUartIrqHandler(FLEXIO_Type *base)
{
    for (unsigned i = 0; i < ARRAYLEN(flexioUartPorts); i++) {
        const flexioUartPort_t *port = &flexioUartPorts[i];
        if (!port->block || port->block->base != base || !(port->rx.held || port->tx.held)) {
            continue;
        }

        const uint32_t flags = base->SHIFTSTAT & base->SHIFTSIEN;
        uartPort_t *s = &pioUartDevice[i].port;

        // The receiver keeps running after an error; the flag is cleared and counted.
        const uint32_t errors = base->SHIFTERR & base->SHIFTEIEN & flexioUartShifterBit(&port->rx);
        if (errors) {
            base->SHIFTERR = errors;
            flexioUartRxErrors[i]++;
        }

        if (flags & flexioUartShifterBit(&port->rx)) {
            // The byte swapped alias has the received byte in its low eight bits; reading it clears the flag.
            const uint8_t ch = base->SHIFTBUFBYS[port->rx.shifter];
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                // A full ring keeps what it holds and drops the new byte.
                const uint32_t next = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
                if (next != s->port.rxBufferTail) {
                    s->port.rxBuffer[s->port.rxBufferHead] = ch;
                    s->port.rxBufferHead = next;
                }
            }
        }

        if (flags & flexioUartShifterBit(&port->tx)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                const uint32_t stopBit = (s->port.options & SERIAL_STOPBITS_2) ? 1u << FLEXIO_UART_DATA_BITS : 0u;
                base->SHIFTBUF[port->tx.shifter] = s->port.txBuffer[s->port.txBufferTail] | stopBit;
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                // The flag stays up while the buffer is empty, so stop asking.
                base->SHIFTSIEN &= ~flexioUartShifterBit(&port->tx);
            }
        }
    }
}

#endif // SERIAL_PIOUART_COUNT > 0
