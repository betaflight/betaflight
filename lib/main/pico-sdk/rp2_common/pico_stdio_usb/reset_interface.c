/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "tusb.h"

#include "pico/bootrom.h"

#if !defined(LIB_TINYUSB_HOST)

#if PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE && !(PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL || PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT)
#warning PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE has been selected but neither PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL nor PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT have been selected.
#endif

#if PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
#include "pico/stdio_usb/reset_interface.h"
#include "hardware/watchdog.h"
#include "device/usbd_pvt.h"

static uint8_t itf_num;

#if PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR
// Support for Microsoft OS 2.0 descriptor
#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

#define MS_OS_20_DESC_LEN  166
#define USBD_ITF_RPI_RESET 2

uint8_t const desc_bos[] =
{
    // total length, number of device caps
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

    // Microsoft OS 2.0 descriptor
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1)
};

TU_VERIFY_STATIC(sizeof(desc_bos) == BOS_TOTAL_LEN, "Incorrect size");

uint8_t const * tud_descriptor_bos_cb(void)
{
    return desc_bos;
}

static const uint8_t desc_ms_os_20[] =
{
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Function Subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), USBD_ITF_RPI_RESET, 0, U16_TO_U8S_LE(0x009C),

    // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible
    
    // MS OS 2.0 Registry property descriptor: length, type
    U16_TO_U8S_LE(0x0080), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0001), U16_TO_U8S_LE(0x0028), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUID" in UTF-16
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x004E), // wPropertyDataLength
    // Vendor-defined Property Data: {bc7398c1-73cd-4cb7-98b8-913a8fca7bf6}
    '{', 0,     'b', 0,     'c', 0,     '7', 0,     '3', 0,     '9', 0,
    '8', 0,     'c', 0,     '1', 0,     '-', 0,     '7', 0,     '3', 0,
    'c', 0,     'd', 0,     '-', 0,     '4', 0,     'c', 0,     'b', 0,
    '7', 0,     '-', 0,     '9', 0,     '8', 0,     'b', 0,     '8', 0,
    '-', 0,     '9', 0,     '1', 0,     '3', 0,     'a', 0,     '8', 0,
    'f', 0,     'c', 0,     'a', 0,     '7', 0,     'b', 0,     'f', 0,
    '6', 0,     '}', 0,       0, 0
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bRequest == 1 && request->wIndex == 7) {
        // Get Microsoft OS 2.0 compatible descriptor
        return tud_control_xfer(rhport, request, (void*)(uintptr_t) desc_ms_os_20, sizeof(desc_ms_os_20));
    } else {
        return false;
    }

    // stall unknown request
    return false;
}
#endif

static void resetd_init(void) {
}

static void resetd_reset(uint8_t __unused rhport) {
    itf_num = 0;
}

static uint16_t resetd_open(uint8_t __unused rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass &&
              RESET_INTERFACE_SUBCLASS == itf_desc->bInterfaceSubClass &&
              RESET_INTERFACE_PROTOCOL == itf_desc->bInterfaceProtocol, 0);

    uint16_t const drv_len = sizeof(tusb_desc_interface_t);
    TU_VERIFY(max_len >= drv_len, 0);

    itf_num = itf_desc->bInterfaceNumber;
    return drv_len;
}

// Support for parameterized reset via vendor interface control request
static bool resetd_control_xfer_cb(uint8_t __unused rhport, uint8_t stage, tusb_control_request_t const * request) {
    // nothing to do with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->wIndex == itf_num) {

#if PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL
        if (request->bRequest == RESET_REQUEST_BOOTSEL) {
#ifdef PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED
            uint gpio_mask = 1u << PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED;
#else
            uint gpio_mask = 0u;
#endif
#if !PICO_STDIO_USB_RESET_BOOTSEL_FIXED_ACTIVITY_LED
            if (request->wValue & 0x100) {
                gpio_mask = 1u << (request->wValue >> 9u);
            }
#endif
            reset_usb_boot(gpio_mask, (request->wValue & 0x7f) | PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK);
            // does not return, otherwise we'd return true
        }
#endif

#if PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT
        if (request->bRequest == RESET_REQUEST_FLASH) {
            watchdog_reboot(0, 0, PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS);
            return true;
        }
#endif

    }
    return false;
}

static bool resetd_xfer_cb(uint8_t __unused rhport, uint8_t __unused ep_addr, xfer_result_t __unused result, uint32_t __unused xferred_bytes) {
    return true;
}

static usbd_class_driver_t const _resetd_driver =
{
#if CFG_TUSB_DEBUG >= 2
    .name = "RESET",
#endif
    .init             = resetd_init,
    .reset            = resetd_reset,
    .open             = resetd_open,
    .control_xfer_cb  = resetd_control_xfer_cb,
    .xfer_cb          = resetd_xfer_cb,
    .sof              = NULL
};

// Implement callback to add our custom driver
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &_resetd_driver;
}
#endif

#if PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE
// Support for default BOOTSEL reset by changing baud rate
void tud_cdc_line_coding_cb(__unused uint8_t itf, cdc_line_coding_t const* p_line_coding) {
    if (p_line_coding->bit_rate == PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE) {
#ifdef PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED
        const uint gpio_mask = 1u << PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED;
#else
        const uint gpio_mask = 0u;
#endif
        reset_usb_boot(gpio_mask, PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK);
    }
}
#endif

#endif