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
void bf_usbd_ep_write_buffer(const void *data, unsigned int count);
bool bf_usbd_ep_wait_write(uint32_t timeoutMs);
bool bf_get_tx_flag(void);
uint8_t usbIsConnected(void);
uint8_t usbIsConfigured(void);
