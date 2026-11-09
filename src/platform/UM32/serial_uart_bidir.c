/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#ifdef USE_UART
#include "build/atomic.h"
#include "build/debug.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/time.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/serial_uart_bidir.h"

#if UM32_UART_BIDIR_ENABLED
// Reserve SDK LPTIM1 (manual LPTIM2) for UART turnaround, with no GPIO mux.
// Ordinary TIM resources used by motors, LEDs and camera control are untouched.
#define BIDIR_IRQ_PRIORITY NVIC_PRIO_SERIALUART1
#define BIDIR_ERROR_MASK (UART_EX_LSR_OE | UART_EX_LSR_PE | UART_EX_LSR_FE | UART_EX_LSR_RFE)

typedef struct {
    uartPort_t *uart;
    IO_t tx;
    IO_t rx;
    uint32_t errors;
    uint32_t deadline;
    uint8_t intervalUs;
    uint8_t af;
    ioConfig_t activeConfig;
    ioConfig_t idleConfig;
    bool transmitting;
    bool waiting;
    bool fault;
    uint16_t timeoutCount; // Survives reconfiguration; saturates to fit debug[].
} uartBidirState_t;

static uartBidirState_t ports[UARTDEV_COUNT];
static bool timerInitialized;
static uint8_t armedIntervalUs;

static ioTag_t bidirRxTag(serialPortIdentifier_e id)
{
    switch (id) {
#ifdef UART1_BIDIR_RX_PIN
    case SERIAL_PORT_UART1: return IO_TAG(UART1_BIDIR_RX_PIN);
#endif
#ifdef UART2_BIDIR_RX_PIN
    case SERIAL_PORT_UART2: return IO_TAG(UART2_BIDIR_RX_PIN);
#endif
#ifdef UART3_BIDIR_RX_PIN
    case SERIAL_PORT_UART3: return IO_TAG(UART3_BIDIR_RX_PIN);
#endif
#ifdef UART4_BIDIR_RX_PIN
    case SERIAL_PORT_UART4: return IO_TAG(UART4_BIDIR_RX_PIN);
#endif
#ifdef UART5_BIDIR_RX_PIN
    case SERIAL_PORT_UART5: return IO_TAG(UART5_BIDIR_RX_PIN);
#endif
#ifdef UART6_BIDIR_RX_PIN
    case SERIAL_PORT_UART6: return IO_TAG(UART6_BIDIR_RX_PIN);
#endif
#ifdef UART7_BIDIR_RX_PIN
    case SERIAL_PORT_UART7: return IO_TAG(UART7_BIDIR_RX_PIN);
#endif
#ifdef UART8_BIDIR_RX_PIN
    case SERIAL_PORT_UART8: return IO_TAG(UART8_BIDIR_RX_PIN);
#endif
    default: return IO_TAG_NONE;
    }
}

static bool bidirPullUp(serialPortIdentifier_e id)
{
    switch (id) {
#ifdef UART1_BIDIR_PULLUP
    case SERIAL_PORT_UART1: return true;
#endif
#ifdef UART2_BIDIR_PULLUP
    case SERIAL_PORT_UART2: return true;
#endif
#ifdef UART3_BIDIR_PULLUP
    case SERIAL_PORT_UART3: return true;
#endif
#ifdef UART4_BIDIR_PULLUP
    case SERIAL_PORT_UART4: return true;
#endif
#ifdef UART5_BIDIR_PULLUP
    case SERIAL_PORT_UART5: return true;
#endif
#ifdef UART6_BIDIR_PULLUP
    case SERIAL_PORT_UART6: return true;
#endif
#ifdef UART7_BIDIR_PULLUP
    case SERIAL_PORT_UART7: return true;
#endif
#ifdef UART8_BIDIR_PULLUP
    case SERIAL_PORT_UART8: return true;
#endif
    default: return false;
    }
}

static uartBidirState_t *state(const uartPort_t *s)
{
    const uartDeviceIdx_e index = uartDeviceIdxFromIdentifier(s->port.identifier);
    return index >= 0 && index < UARTDEV_COUNT ? &ports[index] : NULL;
}

static void reportStatus(const uartBidirState_t *p, serialPortIdentifier_e id)
{
    DEBUG_SET(DEBUG_UART_BIDIR, 0, serialOwnerIndex(id)); //!< UART number [unit:1]
    DEBUG_SET(DEBUG_UART_BIDIR, 1, p->timeoutCount); //!< TEMT timeouts [unit:1]
    DEBUG_SET(DEBUG_UART_BIDIR, 2, p->fault); //!< TX fault latched [unit:1]
}

static void armTimer(void)
{
    unsigned interval = 101;
    for (unsigned i = 0; i < UARTDEV_COUNT; i++) {
        if (ports[i].waiting && ports[i].intervalUs < interval) {
            interval = ports[i].intervalUs;
        }
    }
    // Never postpone a check for a slower port. A newly draining faster port
    // may shorten it; elapsed-time fault deadlines are independent of this.
    if (interval == 101 || ((LPTIM1->CTRL & LPTIM_CTRL_LPTEN) && interval >= armedIntervalUs)) {
        return;
    }
    LPTIM1->CTRL = 0;
    armedIntervalUs = interval;
    const uint32_t hz = HAL_RCM_GetPCLK0Freq() / 128;
    LPTIM1->TARGET = (hz * interval + 999999) / 1000000;
    LPTIM1->IF = LPTIM_IF_OVIF;
    LPTIM1->IE = LPTIM_IE_OVIE;
    LPTIM1->CTRL = LPTIM_CTRL_LPTEN;
}

void uartBidirConfigure(uartPort_t *s)
{
    ATOMIC_BLOCK(BIDIR_IRQ_PRIORITY) {
        uartBidirState_t *p = state(s);
        if (!p) { return; }
        uartDevice_t *dev = uartDeviceFromIdentifier(s->port.identifier);
        if (p->rx && p->rx != IOGetByTag(dev->rx.pin)) { IOConfigGPIO(p->rx, IOCFG_IN_FLOATING); }
        const uint16_t timeoutCount = p->timeoutCount;
        *p = (uartBidirState_t) { .timeoutCount = timeoutCount };
        reportStatus(p, s->port.identifier);
        if (!(s->port.options & SERIAL_BIDIR)) { return; }
        IO_t tx = IOGetByTag(dev->tx.pin);
        IO_t rx = IOGetByTag(bidirRxTag(s->port.identifier));
        if (!tx || !rx || tx == rx || !s->port.baudRate) { return; }
        // Validate the alternate pad/AF against this UART's hardware table.
        bool valid = false;
        for (unsigned i = 0; i < UARTHARDWARE_MAX_PINS; i++) {
            valid |= dev->hardware->rxPins[i].pin == bidirRxTag(s->port.identifier);
        }
        if (!valid) { return; }
#ifdef USE_DMA
        // DMA TX bypasses uartBidirStart(). Keep this dual-pad port on the
        // IRQ path until DMA can participate in pin ownership and TEMT drain.
        s->txDMAResource = NULL;
#endif
        if (!timerInitialized) {
            __HAL_PMU_LPTIM1_CLK_ENABLE();
            __HAL_PMU_LPTIM1_FORCE_RESET();
            __HAL_PMU_LPTIM1_RELEASE_RESET();
            LPTIM1->CFG = LPTIM_CFG_CLKSEL_1 | LPTIM_CFG_DIVSEL | LPTIM_CFG_MODE;
            HAL_NVIC_SetPriority(LPTIMER1_IRQn, NVIC_PRIORITY_BASE(BIDIR_IRQ_PRIORITY), NVIC_PRIORITY_SUB(BIDIR_IRQ_PRIORITY));
            HAL_NVIC_EnableIRQ(LPTIMER1_IRQn);
            timerInitialized = true;
        }
        p->uart = s;
        p->tx = tx;
        p->rx = rx;
        p->af = dev->hardware->af;
        // A board may require an idle-high bias on its external two-pad bus.
        // Keep explicit no-pull and inverted-port choices; TX remains an input
        // when released, so a pull-up never keeps the output driver enabled.
        serialPullMode_t pull = serialOptions_pull(s->port.options);
        if (bidirPullUp(s->port.identifier) && !(s->port.options & SERIAL_INVERTED)) {
            pull = (s->port.options & SERIAL_PULL_NONE) ? serialPullNone : serialPullUp;
        }
        p->activeConfig = pull == serialPullUp ? IOCFG_AF_PP_UP
            : (pull == serialPullDown ? IOCFG_AF_PP_PD : IOCFG_AF_PP);
        p->idleConfig = pull == serialPullUp ? IOCFG_IPU
            : (pull == serialPullDown ? IOCFG_IPD : IOCFG_IN_FLOATING);
        const uint32_t bitUs = (1000000 + s->port.baudRate - 1) / s->port.baudRate;
        p->intervalUs = bitUs < 10 ? 10 : (bitUs > 100 ? 100 : bitUs);
        IO_t normalRx = IOGetByTag(dev->rx.pin);
        if (normalRx && normalRx != tx && normalRx != rx) { IOConfigGPIO(normalRx, IOCFG_IN_FLOATING); }
        IOInit(rx, serialOwnerTxRx(s->port.identifier) + 1, serialOwnerIndex(s->port.identifier));
        IOConfigGPIO(tx, p->idleConfig);
        IOConfigGPIOAF(rx, p->activeConfig, p->af);
    }
}

bool uartBidirStart(uartPort_t *s)
{
    uartBidirState_t *p = state(s);
    if (!p || !p->uart) { return true; }
    if (p->fault) { return false; } // A hardware stall requires reconfiguration.
    p->waiting = false;
    if (!p->transmitting) {
        IOConfigGPIO(p->rx, IOCFG_IN_FLOATING);
        IOConfigGPIOAF(p->tx, p->activeConfig, p->af);
        p->transmitting = true;
    }
    return true;
}

bool uartBidirIsTransmitting(const uartPort_t *s)
{
    uartBidirState_t *p = state(s);
    return p && p->transmitting;
}

uint32_t uartBidirTakeErrors(uartPort_t *s)
{
    uartBidirState_t *p = state(s);
    if (!p) { return 0; }
    const uint32_t errors = p->errors;
    p->errors = 0;
    return errors;
}

static bool complete(uartBidirState_t *p)
{
#if defined(USE_UART7) || defined(USE_UART8)
    if (p->uart->port.identifier >= SERIAL_PORT_UART7) {
        return (((USART_TypeDef *)p->uart->USARTx)->CSR & USART_CSR_TXEMPTY) != 0;
    }
#endif
    const uint32_t lsr = ((UART_TypeDef *)p->uart->USARTx)->LSR;
    p->errors |= lsr & BIDIR_ERROR_MASK; // LSR error bits are read-to-clear.
    return (lsr & UART_EX_LSR_TEMT) != 0;
}

static void release(uartBidirState_t *p)
{
    // Discard noise accumulated while the RX pad was disconnected. Do this
    // before connecting it, so a prompt peer response is never flushed.
#if defined(USE_UART7) || defined(USE_UART8)
    if (p->uart->port.identifier >= SERIAL_PORT_UART7) {
        USART_TypeDef *reg = (USART_TypeDef *)p->uart->USARTx;
        if (reg->CSR & USART_CSR_RXRDY) { (void)reg->RHR; }
    } else
#endif
    {
        UART_TypeDef *reg = (UART_TypeDef *)p->uart->USARTx;
        for (unsigned i = 0; i < 16 && (reg->USR & UART_EX_USR_RFNE); i++) { (void)reg->RBR; }
        (void)reg->LSR;
    }
    p->errors = 0;
    // Enable the input before releasing the bus; it never drives the line.
    IOConfigGPIOAF(p->rx, p->activeConfig, p->af);
    IOConfigGPIO(p->tx, p->idleConfig);
    p->transmitting = false;
    p->waiting = false;
}

void uartBidirDrain(uartPort_t *s)
{
    uartBidirState_t *p = state(s);
    if (!p || !p->transmitting) { return; }
    if (complete(p)) { release(p); return; }
    const unsigned bits = 1 + 8 + ((s->port.options & SERIAL_PARITY_EVEN) ? 1 : 0)
        + ((s->port.options & SERIAL_STOPBITS_2) ? 2 : 1);
    // One shift-register character remains. Bound a stuck peripheral to two
    // character times plus 1 ms; this is fault recovery, not normal turnaround.
    const uint32_t frameUs = (bits * 1000000 + s->port.baudRate - 1) / s->port.baudRate;
    p->deadline = micros() + 2 * frameUs + 1000;
    p->waiting = true;
    armTimer();
}

void LPTIMER1_IRQHandler(void)
{
    ATOMIC_BLOCK(BIDIR_IRQ_PRIORITY) {
        LPTIM1->IF = LPTIM_IF_OVIF;
        LPTIM1->CTRL = 0;
        LPTIM1->IE = 0;
        for (unsigned i = 0; i < UARTDEV_COUNT; i++) {
            uartBidirState_t *p = &ports[i];
            if (!p->waiting) { continue; }
            if (p->uart->port.txBufferHead != p->uart->port.txBufferTail) {
                p->waiting = false; // New data must drain before release.
            } else if (complete(p)) {
                release(p);
            } else if ((int32_t)(micros() - p->deadline) >= 0) {
                p->fault = true;
                if (p->timeoutCount < INT16_MAX) { p->timeoutCount++; }
                release(p); // Fail silent instead of holding the shared line.
                reportStatus(p, p->uart->port.identifier);
            }
        }
        armTimer();
    }
}
#endif
#endif
