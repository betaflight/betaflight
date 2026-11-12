/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define USB_VCP_RX_RING_SIZE 2048U

void cdc_acm_init(uint8_t busid, uintptr_t reg_base);
void cdc_acm_resume_rx(void);
bool usbVcpReceiveData(const uint8_t *data, uint32_t len);
void usbVcpResetRx(void);
uint32_t cdc_acm_tx_session(void);
uint32_t cdc_acm_queue_tx(uint32_t session, const void *data, uint32_t count, bool buffering);
void cdc_acm_flush_tx(uint32_t session);
uint32_t cdc_acm_tx_free(void);
bool cdc_acm_tx_empty(void);
uint8_t usbIsConnected(void);
uint8_t usbIsConfigured(void);
