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
