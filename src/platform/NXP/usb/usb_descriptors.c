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
 * The high speed handling, which needs a second configuration descriptor, a
 * device qualifier and an other speed descriptor built from the first, follows
 * tinyusb/examples/device/cdc_msc/src/usb_descriptors.c
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * SPDX-License-Identifier: MIT
 */

// USB descriptors for the CDC serial port or, in mass storage mode, a disk.

// TinyUSB comes first: platform.h poisons sprintf and snprintf.
#include "tusb.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/version.h"
#include "common/utils.h"
#include "drivers/usb_descriptor.h"

#include "usb_descriptors.h"

// NXP's vendor id with a product id per mode; the configurator must list both
// pairs in src/js/protocols/devices.js.
#ifndef USBD_VID
#define USBD_VID    0x1FC9
#endif
#ifndef USBD_PID
#define USBD_PID    0xBF71
#endif

#ifndef USBD_PID_MSC
#define USBD_PID_MSC    0xBF72
#endif

enum {
    STRING_LANGUAGE = 0,
    STRING_MANUFACTURER,
    STRING_PRODUCT,
    STRING_SERIAL,
    STRING_CDC_INTERFACE,
#ifdef USE_USB_MSC
    STRING_MSC_INTERFACE,
#endif
    STRING_COUNT
};

enum {
    ITF_CDC_CONTROL = 0,
    ITF_CDC_DATA,
    ITF_COUNT
};

// Endpoint 1 IN carries the CDC notifications, endpoint 2 the data.
#define EPNUM_CDC_NOTIFY    0x81
#define EPNUM_CDC_OUT       0x02
#define EPNUM_CDC_IN        0x82

#define CDC_NOTIFY_SIZE     8

#define BULK_PACKET_SIZE_HS 512
#define BULK_PACKET_SIZE_FS 64

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// Self powered, no remote wakeup.
#define CONFIG_ATTRIBUTES   TUSB_DESC_CONFIG_ATT_SELF_POWERED
#define CONFIG_POWER_MA     100

static const tusb_desc_device_t descDevice = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // A CDC device with an interface association declares it at device level.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0200,

    .iManufacturer      = STRING_MANUFACTURER,
    .iProduct           = STRING_PRODUCT,
    .iSerialNumber      = STRING_SERIAL,

    .bNumConfigurations = 1
};

#ifdef USE_USB_MSC
static const tusb_desc_device_t descDeviceMsc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = 0,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID_MSC,
    .bcdDevice          = 0x0200,

    .iManufacturer      = STRING_MANUFACTURER,
    .iProduct           = STRING_PRODUCT,
    .iSerialNumber      = STRING_SERIAL,

    .bNumConfigurations = 1
};

static bool mscMode;

void usbDescriptorsSetMscMode(bool enabled)
{
    mscMode = enabled;
}
#endif

const uint8_t *tud_descriptor_device_cb(void)
{
#ifdef USE_USB_MSC
    if (mscMode) {
        return (const uint8_t *)&descDeviceMsc;
    }
#endif

    return (const uint8_t *)&descDevice;
}

static const uint8_t descConfigurationHs[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, CONFIG_ATTRIBUTES, CONFIG_POWER_MA),
    TUD_CDC_DESCRIPTOR(ITF_CDC_CONTROL, STRING_CDC_INTERFACE, EPNUM_CDC_NOTIFY, CDC_NOTIFY_SIZE,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, BULK_PACKET_SIZE_HS),
};

static const uint8_t descConfigurationFs[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, CONFIG_ATTRIBUTES, CONFIG_POWER_MA),
    TUD_CDC_DESCRIPTOR(ITF_CDC_CONTROL, STRING_CDC_INTERFACE, EPNUM_CDC_NOTIFY, CDC_NOTIFY_SIZE,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, BULK_PACKET_SIZE_FS),
};

#ifdef USE_USB_MSC
// Mass storage owns the whole device and takes endpoint 1 in both directions.
#define EPNUM_MSC_OUT       0x01
#define EPNUM_MSC_IN        0x81

#define MSC_CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t descConfigurationMscHs[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, MSC_CONFIG_TOTAL_LEN, CONFIG_ATTRIBUTES, CONFIG_POWER_MA),
    TUD_MSC_DESCRIPTOR(0, STRING_MSC_INTERFACE, EPNUM_MSC_OUT, EPNUM_MSC_IN, BULK_PACKET_SIZE_HS),
};

static const uint8_t descConfigurationMscFs[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, MSC_CONFIG_TOTAL_LEN, CONFIG_ATTRIBUTES, CONFIG_POWER_MA),
    TUD_MSC_DESCRIPTOR(0, STRING_MSC_INTERFACE, EPNUM_MSC_OUT, EPNUM_MSC_IN, BULK_PACKET_SIZE_FS),
};
#endif

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    UNUSED(index);

    const bool highSpeed = (tud_speed_get() == TUSB_SPEED_HIGH);

#ifdef USE_USB_MSC
    if (mscMode) {
        return highSpeed ? descConfigurationMscHs : descConfigurationMscFs;
    }
#endif

    return highSpeed ? descConfigurationHs : descConfigurationFs;
}

static const tusb_desc_device_qualifier_t descDeviceQualifier = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved          = 0
};

#ifdef USE_USB_MSC
// The qualifier must agree with the device descriptor about the class.
static const tusb_desc_device_qualifier_t descDeviceQualifierMsc = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = 0,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved          = 0
};
#endif

const uint8_t *tud_descriptor_device_qualifier_cb(void)
{
#ifdef USE_USB_MSC
    if (mscMode) {
        return (const uint8_t *)&descDeviceQualifierMsc;
    }
#endif

    return (const uint8_t *)&descDeviceQualifier;
}

// Holds the longest configuration either mode offers.
#ifdef USE_USB_MSC
#define OTHER_SPEED_BUFFER_LEN  \
    ((CONFIG_TOTAL_LEN > MSC_CONFIG_TOTAL_LEN) ? CONFIG_TOTAL_LEN : MSC_CONFIG_TOTAL_LEN)
#else
#define OTHER_SPEED_BUFFER_LEN  CONFIG_TOTAL_LEN
#endif

static uint8_t descOtherSpeedConfiguration[OTHER_SPEED_BUFFER_LEN];

const uint8_t *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    UNUSED(index);

    const bool highSpeed = (tud_speed_get() == TUSB_SPEED_HIGH);
    const uint8_t *opposite;
    uint16_t length;

#ifdef USE_USB_MSC
    if (mscMode) {
        opposite = highSpeed ? descConfigurationMscFs : descConfigurationMscHs;
        length = MSC_CONFIG_TOTAL_LEN;
    } else
#endif
    {
        opposite = highSpeed ? descConfigurationFs : descConfigurationHs;
        length = CONFIG_TOTAL_LEN;
    }

    memcpy(descOtherSpeedConfiguration, opposite, length);

    descOtherSpeedConfiguration[1] = TUSB_DESC_OTHER_SPEED_CONFIG;

    return descOtherSpeedConfiguration;
}

static const char *stringDescriptors[STRING_COUNT] = {
    [STRING_LANGUAGE]      = (const char[]){ 0x09, 0x04 },  // English, 0x0409
    [STRING_MANUFACTURER]  = FC_FIRMWARE_NAME,
    [STRING_PRODUCT]       = NULL,                          // built from the board name
    [STRING_SERIAL]        = NULL,                          // built from the die id
    [STRING_CDC_INTERFACE] = "Betaflight CDC",
#ifdef USE_USB_MSC
    [STRING_MSC_INTERFACE] = "Betaflight MSC",
#endif
};

// Longest string, in UTF-16 units after the header word.
#define STRING_CHARS_MAX    63

static uint16_t stringDescriptor[1 + STRING_CHARS_MAX];

// The unique die id, printed as hex.
static uint8_t serialString(uint16_t *out)
{
    static const char hex[] = "0123456789ABCDEF";
    const uint32_t id[3] = { U_ID_0, U_ID_1, U_ID_2 };
    uint8_t count = 0;

    for (unsigned word = 0; word < ARRAYLEN(id); word++) {
        for (int nibble = 28; nibble >= 0; nibble -= 4) {
            out[count++] = hex[(id[word] >> nibble) & 0x0f];
        }
    }

    return count;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    UNUSED(langid);

    uint8_t count;

    if (index == STRING_LANGUAGE) {
        memcpy(&stringDescriptor[1], stringDescriptors[STRING_LANGUAGE], 2);
        count = 1;
    } else if (index == STRING_SERIAL) {
        count = serialString(&stringDescriptor[1]);
    } else if (index < STRING_COUNT) {
        const char *str = stringDescriptors[index];
        if (index == STRING_PRODUCT) {
#ifdef USE_USB_MSC
            str = mscMode ? usbDescriptorMscProductString() : usbDescriptorProductString();
#else
            str = usbDescriptorProductString();
#endif
        }
        count = 0;
        while (count < STRING_CHARS_MAX && str[count]) {
            stringDescriptor[1 + count] = (uint8_t)str[count];
            count++;
        }
    } else {
        return NULL;
    }

    stringDescriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));

    return stringDescriptor;
}
