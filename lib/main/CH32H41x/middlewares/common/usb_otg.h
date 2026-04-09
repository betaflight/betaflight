/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USB_OTG_H
#define USB_OTG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief usb otg controller hardware or gpio id simulator init.
 *
 * @return On success will return 0, and others indicate fail.
 */
int usb_otg_init(uint8_t busid);
/**
 * @brief usb otg controller hardware or gpio id simulator deinit.
 *
 * @return On success will return 0, and others indicate fail.
 */
int usb_otg_deinit(uint8_t busid);
/**
 * @brief get current role mode.
 *
 * @return return USBOTG_MODE_HOST or USBOTG_MODE_DEVICE.
 */
uint8_t usbotg_get_current_mode(uint8_t busid);

/* called by user */
void USBOTG_IRQHandler(uint8_t busid);

#ifdef __cplusplus
}
#endif

#endif /* USB_OTG_H */