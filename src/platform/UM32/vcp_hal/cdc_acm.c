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
#include "common/printf.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "pg/pg.h"
#include "pg/usb.h"
#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
#endif


#if defined ( __GNUC__ )
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpointer-arith"
#endif

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83
#define CDC_CONTROL_INTERFACE 0

#define USBD_VID             0x396A
#define USBD_PID             0x1000
#define USBD_MAX_POWER     100

#define CDC_ACM_TX_BUFFER_SIZE 2048
#define CDC_ACM_RX_BUFFER_SIZE 2048

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

_Static_assert(CDC_MAX_MPS < USB_VCP_RX_RING_SIZE, "A CDC packet must fit in the VCP RX ring");

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(CDC_CONTROL_INTERFACE, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02)
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

static char serialNumber[3 * 8 + 1];

static const char *const string_descriptors[] = {
    [USB_STRING_LANGID_INDEX] = (const char[]){ 0x09, 0x04 },
    [USB_STRING_MFC_INDEX] = "UMIC",
    [USB_STRING_PRODUCT_INDEX] = "UMIC CDC BF",
    [USB_STRING_SERIAL_INDEX] = serialNumber,
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

    return index < ARRAYLEN(string_descriptors) ? string_descriptors[index] : NULL;
}

const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};

volatile uint32_t max_tx = 0, max_rx = 0;
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[CDC_ACM_RX_BUFFER_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t write_buffer[CDC_ACM_TX_BUFFER_SIZE];
static volatile uint32_t cdcRxPendingLength;
static void (*cdcCtrlLineStateCb)(void *context, uint16_t state);
static void *cdcCtrlLineStateContext;
static bool cdcDtr;
static void (*cdcBaudRateCb)(serialPort_t *context, uint32_t baud);
static serialPort_t *cdcBaudRateContext;
static struct cdc_line_coding cdcLineCoding = {
    .dwDTERate = 2000000,
    .bDataBits = 8,
};

// Head includes buffered writes. Ready marks the end of the data available to
// the endpoint. The IN endpoint owns bytes at the tail until its callback.
static uint32_t cdcTxHead;
static uint32_t cdcTxReady;
static uint32_t cdcTxTail;
static uint32_t cdcTxInFlight;
static volatile uint32_t cdcTxSession;
static volatile bool cdcEverConfigured;

// Called with the USB interrupt masked or from the interrupt.
static void cdc_acm_start_tx(uint8_t busid)
{
    const uint32_t pending = cdcTxReady - cdcTxTail;
    if (!pending || cdcTxInFlight || !usb_device_is_configured(busid)) {
        return;
    }

    const uint32_t offset = cdcTxTail % sizeof(write_buffer);
    const uint32_t count = MIN(pending, sizeof(write_buffer) - offset);
    if (usbd_ep_start_write(busid, CDC_IN_EP, &write_buffer[offset], count) == 0) {
        cdcTxInFlight = count;
    }
}

static void cdc_acm_reset_session(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        cdcRxPendingLength = 0;
        usbVcpResetRx();
        cdcTxHead = 0;
        cdcTxReady = 0;
        cdcTxTail = 0;
        cdcTxInFlight = 0;
        cdcTxSession++;
        cdcCtrlLineStateCb = NULL;
        cdcCtrlLineStateContext = NULL;
        cdcDtr = false;
        cdcBaudRateCb = NULL;
        cdcBaudRateContext = NULL;
    }
}

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
        case USBD_EVENT_INIT:
        case USBD_EVENT_RESET:
        case USBD_EVENT_DEINIT:
        case USBD_EVENT_DISCONNECTED:
            cdc_acm_reset_session();
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            cdc_acm_reset_session();
            cdcEverConfigured = true;
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
    if (!cdcTxInFlight) {
        return;
    }

    if (nbytes != 0 && usb_device_is_configured(busid)) {
        const uint16_t packetSize = usbd_get_ep_mps(busid, ep);
        if (packetSize != 0 && nbytes % packetSize == 0 && usbd_ep_start_write(busid, ep, NULL, 0) == 0) {
            // Keep the endpoint busy until the terminating zero-length packet completes.
            return;
        }
    }
    cdcTxTail += cdcTxInFlight;
    cdcTxInFlight = 0;
    cdc_acm_start_tx(busid);
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
static usbd_request_handler cdcClassRequestHandler;

static int cdc_acm_class_request(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len)
{
    if (setup->bRequest == CDC_REQUEST_SET_LINE_CODING) {
        if (setup->wIndex != CDC_CONTROL_INTERFACE ||
            (setup->bmRequestType & USB_REQUEST_DIR_MASK) != USB_REQUEST_DIR_OUT ||
            setup->wLength != sizeof(struct cdc_line_coding) || !*data) {
            return -1;
        }

        // CherryUSB uses these fields to index its diagnostic tables.
        struct cdc_line_coding coding;
        memcpy(&coding, *data, sizeof(coding));
        if (coding.bCharFormat > 2 || coding.bParityType > 4) {
            return -1;
        }
    }

    return cdcClassRequestHandler(busid, setup, data, len);
}

void cdc_acm_init(uint8_t busid, uintptr_t reg_base)
{
    tfp_sprintf(serialNumber, "%08X%08X%08X", (unsigned int)U_ID_0, (unsigned int)U_ID_1, (unsigned int)U_ID_2);
    usbd_desc_register(busid, &cdc_descriptor);
    usbd_cdc_acm_init_intf(busid, &intf0);
    usbd_cdc_acm_init_intf(busid, &intf1);
    cdcClassRequestHandler = intf0.class_interface_handler;
    intf0.class_interface_handler = cdc_acm_class_request;
    intf1.class_interface_handler = cdc_acm_class_request;
    usbd_add_interface(busid, &intf0);
    usbd_add_interface(busid, &intf1);
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

void cdc_acm_set_ctrl_line_state_cb(void (*cb)(void *context, uint16_t state), void *context)
{
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        cdcCtrlLineStateCb = cb;
        cdcCtrlLineStateContext = cb ? context : NULL;
    }
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    if (busid == 0 && intf == CDC_CONTROL_INTERFACE && usb_device_is_configured(busid)) {
        cdcDtr = dtr;
    }
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    // CherryUSB calls the DTR hook first for each SET_CONTROL_LINE_STATE request.
    if (busid == 0 && intf == CDC_CONTROL_INTERFACE && usb_device_is_configured(busid) && cdcCtrlLineStateCb) {
        const uint16_t state = (cdcDtr ? CTRL_LINE_STATE_DTR : 0) | (rts ? CTRL_LINE_STATE_RTS : 0);
        cdcCtrlLineStateCb(cdcCtrlLineStateContext, state);
    }
}

void cdc_acm_set_baud_rate_cb(void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        cdcBaudRateCb = cb;
        cdcBaudRateContext = cb ? context : NULL;
    }
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *lineCoding)
{
    if (busid != 0 || intf != CDC_CONTROL_INTERFACE || !usb_device_is_configured(busid)) {
        return;
    }

    const uint32_t baud = lineCoding->dwDTERate;
    // The UART HAL multiplies baud by up to 128 in its divider calculations.
    if (baud == 0 || baud > UINT32_MAX / 128U) {
        return;
    }

    cdcLineCoding = *lineCoding;
    if (cdcBaudRateCb) {
        cdcBaudRateCb(cdcBaudRateContext, baud);
    }
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *lineCoding)
{
    (void)busid;
    (void)intf;
    *lineCoding = cdcLineCoding;
}

uint32_t cdc_acm_tx_session(void)
{
    return cdcTxSession;
}

// Return the bytes copied into the queue. Submitting the queue to the endpoint
// and releasing its storage are handled by start_tx and the IN callback.
uint32_t cdc_acm_queue_tx(uint32_t session, const void *data, uint32_t count, bool buffering)
{
    uint32_t copied = 0;
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        if (session != cdcTxSession || !usb_device_is_configured(0)) {
            return 0;
        }

        const uint32_t offset = cdcTxHead % sizeof(write_buffer);
        const uint32_t free = sizeof(write_buffer) - (cdcTxHead - cdcTxTail);
        copied = MIN(count, MIN(free, sizeof(write_buffer) - offset));
        if (copied) {
            memcpy(&write_buffer[offset], data, copied);
            cdcTxHead += copied;
        }

        if (!buffering || cdcTxHead - cdcTxTail == sizeof(write_buffer)) {
            cdcTxReady = cdcTxHead;
        }
        cdc_acm_start_tx(0);
    }
    return copied;
}

void cdc_acm_flush_tx(uint32_t session)
{
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        if (session == cdcTxSession && usb_device_is_configured(0)) {
            cdcTxReady = cdcTxHead;
            cdc_acm_start_tx(0);
        }
    }
}

static uint32_t cdc_acm_tx_count(void)
{
    uint32_t count = 0;
    ATOMIC_BLOCK(NVIC_PRIO_USB) {
        if (usb_device_is_configured(0)) {
            cdc_acm_start_tx(0);
            count = cdcTxHead - cdcTxTail;
        }
    }
    return count;
}

uint32_t cdc_acm_tx_free(void)
{
    return sizeof(write_buffer) - cdc_acm_tx_count();
}

bool cdc_acm_tx_empty(void)
{
    return cdc_acm_tx_count() == 0;
}

uint8_t usbIsConnected(void)
{
    return cdcEverConfigured;
}

uint8_t usbIsConfigured(void)
{
    return usb_device_is_configured(0);
}

#if defined ( __GNUC__ )
  #pragma GCC diagnostic pop
#endif
