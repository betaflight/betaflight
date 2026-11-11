/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "cdc_acm.h"

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include "platform.h"
#include "build/atomic.h"
#include "build/version.h"

#include "drivers/nvic.h"
#include "pg/pg.h"
#include "pg/usb.h"
#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
#endif


extern uint32_t millis(void);
#if defined ( __GNUC__ )
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpointer-arith"
#endif

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USB_TIMEOUT  200

#define USBD_VID             0x396A
#define USBD_PID             0x1000
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define CDC_ACM_TX_BUFFER_SIZE 2048
#define CDC_ACM_RX_BUFFER_SIZE 2048

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

volatile bool flag_con = false, flag_configed = false;

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

_Static_assert(CDC_MAX_MPS < USB_VCP_RX_RING_SIZE, "A CDC packet must fit in the VCP RX ring");

#ifdef CONFIG_USBDEV_ADVANCE_DESC
static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02)
};

static const uint8_t device_quality_descriptor[] = {
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* Langid */
    "UMIC",                       /* Manufacturer */
    "UMIC CDC BF",                    /* Product */
    "2026123456",                 /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    UNUSED(speed);
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    UNUSED(speed);
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    UNUSED(speed);
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    UNUSED(speed);

    if (index > 3) {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};
#else
/*!< global descriptor */
static const uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ' ', 0x00,                  /* wcChar9 */
    'C', 0x00,                  /* wcChar10 */
    'D', 0x00,                  /* wcChar11 */
    'C', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'D', 0x00,                  /* wcChar14 */
    'E', 0x00,                  /* wcChar15 */
    'M', 0x00,                  /* wcChar16 */
    'O', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '2', 0x00,                  /* wcChar3 */
    '1', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '3', 0x00,                  /* wcChar6 */
    '4', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    '6', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
#endif
    0x00
};
#endif

volatile uint32_t max_tx = 0, max_rx = 0;
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[CDC_ACM_RX_BUFFER_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[CDC_ACM_TX_BUFFER_SIZE];
static volatile bool ep_tx_busy_flag;
static volatile uint32_t cdcRxPendingLength;

static void cdc_acm_arm_rx(uint8_t busid, uint8_t ep)
{
    usbd_ep_start_read(busid, ep, read_buffer, usbd_get_ep_mps(busid, ep));
}

void cdc_acm_resume_rx(void)
{
    if (cdcRxPendingLength == 0) {
        return;
    }

    // Serialize the retry with USB OUT completions and USB reset events.
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        const uint32_t pending = cdcRxPendingLength;
        if (!usb_device_is_configured(0)) {
            cdcRxPendingLength = 0;
        } else if (pending != 0 && usbVcpReceiveData(read_buffer, pending)) {
            cdcRxPendingLength = 0;
            cdc_acm_arm_rx(0, CDC_OUT_EP);
        }
    }
}

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
        case USBD_EVENT_DEINIT:
            cdcRxPendingLength = 0;
            ep_tx_busy_flag = false;
            break;
        case USBD_EVENT_CONNECTED:
			flag_con = true;
            break;
        case USBD_EVENT_DISCONNECTED:
			flag_con  = false;
            cdcRxPendingLength = 0;
            ep_tx_busy_flag = false;
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            cdcRxPendingLength = 0;
            ep_tx_busy_flag = false;
            cdc_acm_arm_rx(busid, CDC_OUT_EP);
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if (!usb_device_is_configured(busid)) {
        return;
    }

    if (nbytes > max_rx) {
        max_rx = nbytes;
    }

    if (!usbVcpReceiveData(read_buffer, nbytes)) {
        // Hold this packet and leave OUT unarmed until the VCP reader frees space.
        cdcRxPendingLength = nbytes;
        return;
    }

    cdc_acm_arm_rx(busid, ep);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if (ep_tx_busy_flag && nbytes != 0 && usb_device_is_configured(busid)) {
        const uint16_t packetSize = usbd_get_ep_mps(busid, ep);
        if (packetSize != 0 && nbytes % packetSize == 0 && usbd_ep_start_write(busid, ep, NULL, 0) == 0) {
            // Keep the endpoint busy until the terminating zero-length packet completes.
            return;
        }
    }
    ep_tx_busy_flag = false;
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

void cdc_acm_init(uint8_t busid, uintptr_t reg_base)
{
#ifdef CONFIG_USBDEV_ADVANCE_DESC
    usbd_desc_register(busid, &cdc_descriptor);
#else
    usbd_desc_register(busid, cdc_descriptor);
#endif
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

volatile uint8_t dtr_enable = 0;

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;
    (void)dtr;
}

bool bf_usbd_ep_wait_write(uint32_t timeoutMs)
{
    const uint32_t start = millis();
    while (ep_tx_busy_flag) {
        if (!usb_device_is_configured(0) || millis() - start >= timeoutMs) {
            return false;
        }
    }
    return true;
}

void bf_usbd_ep_write_buffer(const void *data, unsigned int count)
{
    if (count == 0 || !bf_usbd_ep_wait_write(USB_TIMEOUT)) {
        return;
    }

    const uint8_t *src = data;
    while (count != 0 && usb_device_is_configured(0)) {
        const unsigned int chunk = MIN(count, (unsigned int)sizeof(write_buffer));
        memcpy(write_buffer, src, chunk);

        ATOMIC_BLOCK(NVIC_PRIO_USB) {
            if (!usb_device_is_configured(0)) {
                return;
            }
            ep_tx_busy_flag = true;
            if (usbd_ep_start_write(0, CDC_IN_EP, write_buffer, chunk) != 0) {
                ep_tx_busy_flag = false;
                return;
            }
        }

        // A timed-out transfer still owns write_buffer until its completion callback.
        if (!bf_usbd_ep_wait_write(USB_TIMEOUT)) {
            return;
        }
        src += chunk;
        count -= chunk;
    }
}

bool bf_get_tx_flag(void)
{
    return ep_tx_busy_flag && usb_device_is_configured(0);
}

uint8_t usbIsConnected(void)
{
	return 1;
//    if (flag_con)
//        return 1;
//    else
//        return 0;
}

uint8_t usbIsConfigured(void)
{
    return 1;
}

#if defined ( __GNUC__ )
  #pragma GCC diagnostic pop
#endif
