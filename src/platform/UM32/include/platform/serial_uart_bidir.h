/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

// UARTn_BIDIR_RX_PIN selects the external receive pad. UARTn_BIDIR_PULLUP
// optionally biases that port's non-inverted two-pad bus high when released.
#if defined(UART1_BIDIR_RX_PIN) || defined(UART2_BIDIR_RX_PIN) || defined(UART3_BIDIR_RX_PIN) \
    || defined(UART4_BIDIR_RX_PIN) || defined(UART5_BIDIR_RX_PIN) || defined(UART6_BIDIR_RX_PIN) \
    || defined(UART7_BIDIR_RX_PIN) || defined(UART8_BIDIR_RX_PIN)
#define UM32_UART_BIDIR_ENABLED 1
bool uartBidirCanOpen(const uartDevice_t *dev, uint32_t baudRate);
void uartBidirConfigure(uartPort_t *s);
bool uartBidirStart(uartPort_t *s);
void uartBidirDrain(uartPort_t *s);
bool uartBidirIsTransmitting(const uartPort_t *s);
uint32_t uartBidirTakeErrors(uartPort_t *s);
#else
#define UM32_UART_BIDIR_ENABLED 0
static inline bool uartBidirCanOpen(const uartDevice_t *dev, uint32_t baudRate) { (void)dev; (void)baudRate; return false; }
static inline void uartBidirConfigure(uartPort_t *s) { (void)s; }
static inline bool uartBidirStart(uartPort_t *s) { (void)s; return true; }
static inline void uartBidirDrain(uartPort_t *s) { (void)s; }
static inline bool uartBidirIsTransmitting(const uartPort_t *s) { (void)s; return false; }
static inline uint32_t uartBidirTakeErrors(uartPort_t *s) { (void)s; return 0; }
#endif
