/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbd_rndis.h"
#include "rndis_protocol.h"
#include "mongoose.h"

#ifdef CONFIG_USBDEV_RNDIS_USING_LWIP
#error Do not define CONFIG_USBDEV_RNDIS_USING_LWIP in this demo
#endif

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_RNDIS_DESCRIPTOR_LEN)

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_RNDIS_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },    /* Langid */
    "CherryUSB",                     /* Manufacturer */
    "CherryUSB RNDIS MONGOOSE DEMO", /* Product */
    "2025123456",                    /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return config_descriptor_fs;
    } else {
        return NULL;
    }
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;

    return device_quality_descriptor;
}

static const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return other_speed_config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return other_speed_config_descriptor_fs;
    } else {
        return NULL;
    }
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

static uint8_t rndis_mac[6] = { 0x20, 0x89, 0x84, 0x6A, 0x96, 0xAA };

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_rndis_rx_buffer[USB_ALIGN_UP(CONFIG_USBDEV_RNDIS_ETH_MAX_FRAME_SIZE, CONFIG_USB_ALIGN_SIZE)];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_rndis_tx_buffer[USB_ALIGN_UP(CONFIG_USBDEV_RNDIS_ETH_MAX_FRAME_SIZE, CONFIG_USB_ALIGN_SIZE)];

volatile bool g_rndis_tx_busy_flag = false;

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            g_rndis_tx_busy_flag = false;
            usbd_rndis_start_read(g_rndis_rx_buffer, sizeof(g_rndis_rx_buffer));
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

static struct mg_tcpip_if *s_ifp;

void usbd_rndis_data_recv_done(uint32_t len)
{
    (void)len;

    rndis_data_packet_t *hdr;
    uint8_t *buf;

    hdr = (rndis_data_packet_t *)g_rndis_rx_buffer;
    buf = (uint8_t *)hdr + hdr->DataOffset + sizeof(rndis_generic_msg_t);

    mg_tcpip_qwrite((void *)buf, hdr->DataLength, s_ifp);

    usbd_rndis_start_read(g_rndis_rx_buffer, sizeof(g_rndis_rx_buffer));
}

void usbd_rndis_data_send_done(uint32_t len)
{
    (void)len;

    g_rndis_tx_busy_flag = false;
}

static size_t usb_tx(const void *buf, size_t len, struct mg_tcpip_if *ifp)
{
    (void)ifp;
    rndis_data_packet_t *hdr;

    if (!usb_device_is_configured(0))
        return 0;

    hdr = (rndis_data_packet_t *)g_rndis_tx_buffer;

    memset(hdr, 0, sizeof(rndis_data_packet_t));
    hdr->MessageType = REMOTE_NDIS_PACKET_MSG;
    hdr->MessageLength = sizeof(rndis_data_packet_t) + len;
    hdr->DataOffset = sizeof(rndis_data_packet_t) - sizeof(rndis_generic_msg_t);
    hdr->DataLength = len;

    memcpy(g_rndis_tx_buffer + sizeof(rndis_data_packet_t), buf, len);

    g_rndis_tx_busy_flag = true;
    usbd_rndis_start_write(g_rndis_tx_buffer, sizeof(rndis_data_packet_t) + len);
    while (g_rndis_tx_busy_flag) {
    }
    return len;
}

static bool usb_poll(struct mg_tcpip_if *ifp, bool s1)
{
    (void)ifp;

    return s1 ? usb_device_is_configured(0) : false;
}

struct mg_tcpip_driver driver = { .tx = usb_tx, .poll = usb_poll };

struct usbd_interface intf0;
struct usbd_interface intf1;

struct mg_mgr mgr; // Initialise Mongoose event manager

void cdc_rndis_mongoose_init(uint8_t busid, uint32_t reg_base)
{
    mg_mgr_init(&mgr); // and attach it to the interface

    static struct mg_tcpip_if mif = { .mac = { 2, 0, 1, 2, 3, 0x77 },
                                      .enable_dhcp_server = true,
                                      .driver = &driver,
                                      .recv_queue.size = 4096 };
    s_ifp = &mif;

    mif.ip = mg_htonl(MG_U32(192, 168, 7, 1));
    mif.mask = mg_htonl(MG_U32(255, 255, 255, 0));
    mg_tcpip_init(&mgr, &mif);

    web_init(&mgr);

    usbd_desc_register(busid, &cdc_descriptor);
    usbd_add_interface(busid, usbd_rndis_init_intf(&intf0, CDC_OUT_EP, CDC_IN_EP, CDC_INT_EP, rndis_mac));
    usbd_add_interface(busid, usbd_rndis_init_intf(&intf1, CDC_OUT_EP, CDC_IN_EP, CDC_INT_EP, rndis_mac));
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

// call mg_mgr_poll(&mgr, 0); in main loop