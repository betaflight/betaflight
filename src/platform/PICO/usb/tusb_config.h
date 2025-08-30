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

#define CFG_TUD_ENABLED         1
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  256

// Enable Mass Storage Class for TinyUSB on PICO platform
#ifndef CFG_TUD_MSC
#define CFG_TUD_MSC             1
#endif
// Reasonable defaults; TinyUSB will fall back if unspecified
#ifndef CFG_TUD_MSC_MAXLUN
#define CFG_TUD_MSC_MAXLUN      1
#endif
#ifndef CFG_TUD_MSC_EP_BUFSIZE
#define CFG_TUD_MSC_EP_BUFSIZE  512
#endif

#define TUP_DCD_EDPT_ISO_ALLOC
#define TUP_DCD_ENDPOINT_MAX    16

#define TU_ATTR_FAST_FUNC       __attribute__((section(".time_critical.tinyusb")))

#define CFG_TUSB_MCU OPT_MCU_RP2040
