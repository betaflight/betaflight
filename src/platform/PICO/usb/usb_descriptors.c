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
 *
 * Minimal USB descriptors for TinyUSB composite CDC + MSC device on PICO
 *
 * Based on TinyUSB examples and adapted for Betaflight.
 */

#include "platform.h"

#include "tusb_config.h"
#include <string.h>

#define _STDIO_H_
#include "tusb.h"

// Extern flag controlled by MSC start logic
extern bool pico_msc_active;

#ifndef USBD_VID
#define USBD_VID 0x2E8A // Raspberry Pi
#endif

#ifndef USBD_PID
#if PICO_RP2040
#define USBD_PID 0x000a // Raspberry Pi Pico SDK CDC for RP2040
#else
#define USBD_PID 0x0009 // Raspberry Pi Pico SDK CDC
#endif
#endif

// Device descriptor (keep CDC as it was; MSC uses same device desc)
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

const uint8_t * tud_descriptor_device_cb(void)
{
  return (const uint8_t *) &desc_device;
}

// CDC-only configuration (match original CDC descriptor: notify 0x81, OUT 0x02, IN 0x82)
enum {
  ITF_CDC_ONLY_COMM = 0,
  ITF_CDC_ONLY_DATA,
  ITF_CDC_ONLY_TOTAL
};

#define CDC_ONLY_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t cdc_only_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_CDC_ONLY_TOTAL, 0, CDC_ONLY_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 250),
  TUD_CDC_DESCRIPTOR(ITF_CDC_ONLY_COMM, 4, 0x81, 8, 0x02, 0x82, 64),
};

// MSC-only configuration (used when MSC mode is active)
enum {
  ITF_MSC_ONLY = 0,
  ITF_MSC_ONLY_TOTAL
};

#define MSC_ONLY_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t msc_only_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_MSC_ONLY_TOTAL, 0, MSC_ONLY_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 250),
  TUD_MSC_DESCRIPTOR(ITF_MSC_ONLY, 5, 0x01, 0x81, 64),
};

const uint8_t * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // single config
  return pico_msc_active ? msc_only_configuration : cdc_only_configuration;
}

// String descriptors
static char const* string_desc_arr[] = {
  (const char[]){ 0x09, 0x04 }, // 0: English (0x0409)
  FC_FIRMWARE_NAME,             // 1: Manufacturer
  USBD_PRODUCT_STRING,          // 2: Product
  "123456",                    // 3: Serial (placeholder)
  "Betaflight CDC",            // 4: CDC Interface
  "Betaflight MSC",            // 5: MSC Interface
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  uint8_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;
    const char* str = string_desc_arr[index];
    chr_count = (uint8_t) strlen(str);
    if (chr_count > 31) chr_count = 31;
    for (uint8_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = str[i];
    }
  }
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
  return _desc_str;
}


