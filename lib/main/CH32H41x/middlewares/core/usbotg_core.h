/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBOTG_CORE_H
#define USBOTG_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define USBOTG_MODE_UNKNOWN 0
#define USBOTG_MODE_OTG     1
#define USBOTG_MODE_HOST    2
#define USBOTG_MODE_DEVICE  3

#include "usbd_core.h"
#include "usbh_core.h"
#include "usb_otg.h"

int usbotg_initialize(uint8_t otg_mode, uint8_t busid, uint32_t reg_base,
                      int (*usbh_initialize)(uint8_t busid, uint32_t reg_base),
                      int (*usbd_initialize)(uint8_t busid, uint32_t reg_base));
int usbotg_deinitialize(uint8_t busid, uint32_t reg_base);

/* called by user */
void usbotg_trigger_role_change(uint8_t busid);

#ifdef __cplusplus
}
#endif

#endif /* USBOTG_CORE_H */