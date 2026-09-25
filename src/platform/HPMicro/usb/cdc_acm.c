/*
 * Copyright (c) 2022-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

// USB CDC ACM (virtual COM port) device: USB descriptors, bulk IN/OUT
// endpoint handling with TX/RX ring buffers, and glue callbacks for
// Betaflight's serial driver.

#include "platform.h"

#include "build/version.h"

#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "hpm_csr_regs.h"
#include "usb_descriptor_hpm.h"

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x83
#define CDC_ACM_TX_BUFFER_SIZE 2048
#define CDC_ACM_RX_BUFFER_SIZE 2048
/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

static volatile bool flagConnected = false;
static volatile bool flagConfigured = false;

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_CDC_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const char *const string_descriptors[] = {
    (const char[]) { 0x09, 0x04 },      /* Langid */
    FC_FIRMWARE_NAME,           /* Manufacturer */
    USBD_PRODUCT_STRING,        /* Product */
    NULL,                       /* Serial Number: generated from OTP UUID */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void) speed;

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
    (void) speed;

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
    (void) speed;

    if (index == USB_STRING_SERIAL_INDEX) {
        return hpmUsbGetSerialNumber();
    }
    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

static const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t readBuffer[CDC_ACM_RX_BUFFER_SIZE];
static uint8_t writeBuffer[CDC_ACM_TX_BUFFER_SIZE];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t txBuffer[CDC_ACM_TX_BUFFER_SIZE];
static uint8_t *readBufferPtr;
static volatile uint32_t readBufferAvailable;
static volatile uint32_t writeBufferRd = 0;
static volatile uint32_t writeBufferWr = 0;
static volatile bool epTxBusyFlag;
static uint8_t cdcBusid;

static void cdcStartRead(uint8_t busid)
{
    readBufferPtr = &readBuffer[0];
    usbd_ep_start_read(busid, CDC_OUT_EP, readBufferPtr, usbd_get_ep_mps(busid, CDC_OUT_EP));
}

static void cdcResetTransferState(void)
{
    readBufferAvailable = 0;
    writeBufferRd = 0;
    writeBufferWr = 0;
    epTxBusyFlag = false;
}

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
    case USBD_EVENT_RESET:
        flagConfigured = false;
        cdcResetTransferState();
        break;
    case USBD_EVENT_CONNECTED:
        flagConnected = true;
        break;
    case USBD_EVENT_DISCONNECTED:
        flagConnected = false;
        flagConfigured = false;
        cdcResetTransferState();
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        flagConfigured = true;
        readBufferAvailable = 0;
        /* setup first out ep read transfer */
        cdcStartRead(busid);
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
    (void) ep;

    readBufferPtr = &readBuffer[0];
    readBufferAvailable = nbytes;

    /* Keep a non-empty OUT packet in readBuffer until the VCP consumer has
     * drained it.  Leaving the endpoint unarmed makes USB apply backpressure
     * instead of allowing a later packet to overwrite unread data. */
    if (!nbytes) {
        cdcStartRead(busid);
    }
}

/*
 * Drain the TX ring buffer into txBuffer and start the IN transfer.  Shared
 * by the thread-side flush (bf_usbd_ep_start_write) and the USB ISR
 * (usbd_cdc_acm_bulk_in).  The critical section keeps the busy-flag check,
 * the ring-to-txBuffer copy, the writeBufferRd advance and the endpoint
 * write atomic against both contexts.  A transfer already in flight owns
 * txBuffer and drains the ring itself from its completion callback, so the
 * bytes simply stay queued in the ring until then.
 */
static void cdcDrainTxRingAndStart(void)
{
    const uint32_t irqState = disable_global_irq(CSR_MSTATUS_MIE_MASK);

    if (!epTxBusyFlag && writeBufferRd != writeBufferWr) {
        epTxBusyFlag = true;
        uint32_t txLen = 0;
        while (writeBufferRd != writeBufferWr && txLen < CDC_ACM_TX_BUFFER_SIZE) {
            txBuffer[txLen++] = writeBuffer[writeBufferRd];
            writeBufferRd = (writeBufferRd + 1) % CDC_ACM_TX_BUFFER_SIZE;
        }
        usbd_ep_start_write(cdcBusid, CDC_IN_EP, txBuffer, txLen);
    }

    enable_global_irq(irqState);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void) busid;
    (void) ep;
    (void) nbytes;

    /* If the last IN transfer was an exact multiple of the MPS, the host is
     * waiting for a zero-length packet. Otherwise we can drain any further
     * bytes that arrived in the TX ring buffer while the transfer was in
     * flight. */
    if (nbytes % usbd_get_ep_mps(busid, ep) == 0 && nbytes) {
        usbd_ep_start_write(busid, ep, NULL, 0);
        return;
    }

    /* The previous transfer completed, so txBuffer is free again.  Clearing
     * the busy flag lets the shared helper start the next transfer if any
     * bytes were queued meanwhile; it leaves the flag clear when the ring is
     * empty. */
    epTxBusyFlag = false;
    cdcDrainTxRingAndStart();
}

/*!< endpoint call back */
static struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

static struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

static struct cdc_line_coding lineCoding = {
    .dwDTERate = 115200,
    .bCharFormat = 0,
    .bParityType = 0,
    .bDataBits = 8,
};

static uint16_t ctrlLineState;
static void (*ctrlLineStateCb)(void *context, uint16_t ctrlLineState);
static void *ctrlLineStateCbContext;
static void (*baudRateCb)(void *context, uint32_t baud);
static void *baudRateCbContext;

/* function ------------------------------------------------------------------*/

void cdc_acm_init(uint8_t busid, uint32_t reg_base)
{
    cdcBusid = busid;

    usbd_desc_register(busid, &cdc_descriptor);
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void) busid;
    (void) intf;

    lineCoding = *line_coding;
    if (baudRateCb) {
        baudRateCb(baudRateCbContext, lineCoding.dwDTERate);
    }
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void) busid;
    (void) intf;

    *line_coding = lineCoding;
}

static void cdcSetCtrlLineState(uint16_t mask, bool enabled)
{
    if (enabled) {
        ctrlLineState |= mask;
    } else {
        ctrlLineState &= ~mask;
    }
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void) busid;
    (void) intf;

    cdcSetCtrlLineState(1U << 0, dtr);
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    (void) busid;
    (void) intf;

    cdcSetCtrlLineState(1U << 1, rts);

    /* CherryUSB calls the DTR hook followed by the RTS hook for each
     * SET_CONTROL_LINE_STATE request. Notify Betaflight here so it observes
     * the complete state once per request. */
    if (ctrlLineStateCb) {
        ctrlLineStateCb(ctrlLineStateCbContext, ctrlLineState);
    }
}

uint32_t bf_usbd_cdc_baud_rate(void)
{
    return lineCoding.dwDTERate;
}

void bf_usbd_cdc_set_baud_rate_cb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

void bf_usbd_cdc_set_ctrl_line_state_cb(void (*cb)(void *context, uint16_t state), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}

uint32_t bf_usbd_ep_write_buffer(const void *data, int count)
{
    const uint8_t *src = (const uint8_t *) data;
    int written = 0;
    const uint32_t irqState = disable_global_irq(CSR_MSTATUS_MIE_MASK);

    for (; written < count; written++) {
        uint32_t nextWr = (writeBufferWr + 1) % CDC_ACM_TX_BUFFER_SIZE;

        if (nextWr == writeBufferRd) {
            // TX ring buffer full: let the caller flush and retry the rest.
            break;
        }
        writeBuffer[writeBufferWr] = src[written];
        writeBufferWr = nextWr;
    }
    enable_global_irq(irqState);
    return written;
}

uint32_t bf_usbd_ep_tx_free(void)
{
    return (writeBufferRd - writeBufferWr - 1 + CDC_ACM_TX_BUFFER_SIZE) % CDC_ACM_TX_BUFFER_SIZE;
}

void bf_usbd_ep_start_write(void)
{
    /* Thread context: a transfer already in flight will drain the ring from
     * its own completion callback, so never queue a second write on the
     * active endpoint. */
    cdcDrainTxRingAndStart();
}

bool bfIsTxBusy(void)
{
    return epTxBusyFlag;
}

bool bfIsTxEmpty(void)
{
    return writeBufferRd == writeBufferWr && !epTxBusyFlag;
}

uint32_t bf_usbd_ep_rx_available(void)
{
    return readBufferAvailable;
}

uint8_t bf_usbd_ep_read(void)
{
    uint8_t data = 0;
    const uint32_t irqState = disable_global_irq(CSR_MSTATUS_MIE_MASK);

    if (readBufferAvailable) {
        data = *readBufferPtr++;
        readBufferAvailable--;
        if (!readBufferAvailable && flagConfigured) {
            cdcStartRead(cdcBusid);
        }
    }
    enable_global_irq(irqState);

    return data;
}

uint8_t usbIsConnected(void)
{
    return (flagConnected) ? 1 : 0;
}

uint8_t usbIsConfigured(void)
{
    return (flagConfigured) ? 1 : 0;
}
