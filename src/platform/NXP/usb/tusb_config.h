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

// TinyUSB build options for i.MXRT; must stay free of Betaflight headers.

#pragma once

#define CFG_TUSB_MCU                OPT_MCU_MIMXRT1XXX
#define CFG_TUSB_OS                 OPT_OS_NONE
#define CFG_TUSB_DEBUG              0

#define CFG_TUD_ENABLED             1
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

// Controller buffers go in uncached DTCM.
#define CFG_TUD_MEM_SECTION         __attribute__((section(".dtcm_bss")))
#define CFG_TUD_MEM_DCACHE_ENABLE   0

// The SD card driver needs 32 byte cache line aligned buffers.
#define CFG_TUD_MEM_ALIGN           __attribute__((aligned(32)))

#define CFG_TUD_CDC                 1
#define CFG_TUD_MSC                 1
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

#define CFG_TUD_ENDPOINT0_SIZE      64

// One SD card block per packet.
#define CFG_TUD_MSC_EP_BUFSIZE      512

// One 512 byte high speed bulk packet per endpoint buffer, two per ring buffer.
#define CFG_TUD_CDC_EP_BUFSIZE      512
#define CFG_TUD_CDC_RX_BUFSIZE      1024
#define CFG_TUD_CDC_TX_BUFSIZE      1024
