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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

void cdc_usb_write_flush(void);
int cdc_usb_write(const char *buf, int length);
int cdc_usb_read(char *buf, int length);
bool cdc_usb_init(void);
bool cdc_usb_deinit(void);
bool cdc_usb_connected(void);
bool cdc_usb_bytes_available(void);
uint32_t cdc_usb_baud_rate(void);
uint32_t cdc_usb_tx_bytes_free(void);
