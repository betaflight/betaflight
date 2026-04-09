/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbotg_core.h"

struct usbotg_core_priv {
    uint8_t busid;
    uint32_t reg_base;
    usb_osal_sem_t change_sem;
    usb_osal_thread_t change_thread;
    bool usbh_initialized;
    bool usbd_initialized;
    int (*usbh_initialize)(uint8_t busid, uint32_t reg_base);
    int (*usbd_initialize)(uint8_t busid, uint32_t reg_base);
} g_usbotg_core[CONFIG_USBHOST_MAX_BUS];

static void usbotg_host_initialize(uint8_t busid)
{
    if (g_usbotg_core[busid].usbd_initialized) {
        g_usbotg_core[busid].usbd_initialized = false;
        usbd_deinitialize(busid);
    }
    if (g_usbotg_core[busid].usbh_initialize && !g_usbotg_core[busid].usbh_initialized) {
        g_usbotg_core[busid].usbh_initialized = true;
        g_usbotg_core[busid].usbh_initialize(g_usbotg_core[busid].busid, g_usbotg_core[busid].reg_base);
    }
}

static void usbotg_device_initialize(uint8_t busid)
{
    if (g_usbotg_core[busid].usbh_initialized) {
        g_usbotg_core[busid].usbh_initialized = false;
        usbh_deinitialize(busid);
    }
    if (g_usbotg_core[busid].usbd_initialize && !g_usbotg_core[busid].usbd_initialize) {
        g_usbotg_core[busid].usbd_initialized = true;
        g_usbotg_core[busid].usbd_initialize(g_usbotg_core[busid].busid, g_usbotg_core[busid].reg_base);
    }
}

static void usbotg_rolechange_thread(void *argument)
{
    uint8_t busid = (uint8_t)(uintptr_t)argument;

    usb_otg_init(busid);

    while (1) {
        if (usb_osal_sem_take(g_usbotg_core[busid].change_sem, USB_OSAL_WAITING_FOREVER) == 0) {
            if (usbotg_get_current_mode(busid) == USBOTG_MODE_HOST) {
                usbotg_host_initialize(busid);
            } else if (usbotg_get_current_mode(busid) == USBOTG_MODE_DEVICE) {
                usbotg_device_initialize(busid);
            }
        }
    }
}

int usbotg_initialize(uint8_t otg_mode, uint8_t busid, uint32_t reg_base,
                      int (*usbh_initialize)(uint8_t busid, uint32_t reg_base),
                      int (*usbd_initialize)(uint8_t busid, uint32_t reg_base))
{
    char thread_name[32] = { 0 };

    if (busid >= CONFIG_USBHOST_MAX_BUS) {
        USB_LOG_ERR("bus overflow\r\n");
        while (1) {
        }
    }

    g_usbotg_core[busid].busid = busid;
    g_usbotg_core[busid].reg_base = reg_base;
    g_usbotg_core[busid].usbh_initialize = usbh_initialize;
    g_usbotg_core[busid].usbd_initialize = usbd_initialize;

    if (otg_mode == USBOTG_MODE_OTG) {
        g_usbotg_core[busid].change_sem = usb_osal_sem_create(0);
        if (g_usbotg_core[busid].change_sem == NULL) {
            USB_LOG_ERR("Failed to create change_sem\r\n");
            return -1;
        }

        snprintf(thread_name, 32, "usbotg%u", busid);
        g_usbotg_core[busid].change_thread = usb_osal_thread_create(thread_name, CONFIG_USBHOST_PSC_STACKSIZE, CONFIG_USBHOST_PSC_PRIO, usbotg_rolechange_thread, (void *)(uintptr_t)busid);
        if (g_usbotg_core[busid].change_thread == NULL) {
            USB_LOG_ERR("Failed to create usbotg thread\r\n");
            return -1;
        }
    } else {
        if (otg_mode == USBOTG_MODE_HOST) {
            usbotg_host_initialize(busid);
        } else if (otg_mode == USBOTG_MODE_DEVICE) {
            usbotg_device_initialize(busid);
        }
    }

    return 0;
}

int usbotg_deinitialize(uint8_t busid, uint32_t reg_base)
{
    if (g_usbotg_core[busid].usbd_initialized) {
        g_usbotg_core[busid].usbd_initialized = false;
        usbd_deinitialize(busid);
    }

    if (g_usbotg_core[busid].usbh_initialized) {
        g_usbotg_core[busid].usbh_initialized = false;
        usbh_deinitialize(busid);
    }

    if (g_usbotg_core[busid].change_sem) {
        usb_otg_deinit(busid);
        usb_osal_sem_delete(g_usbotg_core[busid].change_sem);
    }

    if (g_usbotg_core[busid].change_thread) {
        usb_osal_thread_delete(g_usbotg_core[busid].change_thread);
    }

    return 0;
}

void usbotg_trigger_role_change(uint8_t busid)
{
    usb_osal_sem_give(g_usbotg_core[busid].change_sem);
}

void USBOTG_IRQHandler(uint8_t busid)
{
    if (usbotg_get_current_mode(busid) == USBOTG_MODE_HOST) {
        USBH_IRQHandler(busid);
    } else if (usbotg_get_current_mode(busid) == USBOTG_MODE_DEVICE) {
        USBD_IRQHandler(busid);
    }
}