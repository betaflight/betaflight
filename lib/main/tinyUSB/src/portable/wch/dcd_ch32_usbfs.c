/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Matthew Tran
 * Copyright (c) 2024 hathach
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && defined(TUP_USBIP_WCH_USBFS) && CFG_TUD_WCH_USBIP_USBFS

#include "device/dcd.h"
#include "ch32_usbfs_reg.h"

/* private defines */
#define EP_MAX (8)

#define EP_DMA(ep)     ((&USBOTG_FS->UEP0_DMA)[ep])
#define EP_TX_LEN(ep)  ((&USBOTG_FS->UEP0_TX_LEN)[2 * ep])
#define EP_TX_CTRL(ep) ((&USBOTG_FS->UEP0_TX_CTRL)[4 * ep])
#define EP_RX_CTRL(ep) ((&USBOTG_FS->UEP0_RX_CTRL)[4 * ep])

/* private data */
struct usb_xfer {
  bool valid;
  uint8_t* buffer;
  size_t len;
  size_t processed_len;
  size_t max_size;
};

static struct {
  bool ep0_tog;
  bool isochronous[EP_MAX];
  struct usb_xfer xfer[EP_MAX][2];
  TU_ATTR_ALIGNED(4) uint8_t buffer[EP_MAX][2][64];
  TU_ATTR_ALIGNED(4) struct {
    // OUT transfers >64 bytes will overwrite queued IN data!
    uint8_t out[64];
    uint8_t in[1023];
    uint8_t pad;
  } ep3_buffer;
} data;

/* private helpers */
static void update_in(uint8_t rhport, uint8_t ep, bool force) {
  struct usb_xfer* xfer = &data.xfer[ep][TUSB_DIR_IN];
  if (xfer->valid) {
    if (force || xfer->len) {
      size_t len = TU_MIN(xfer->max_size, xfer->len);
      if (ep == 0) {
        memcpy(data.buffer[ep][TUSB_DIR_OUT], xfer->buffer, len); // ep0 uses same chunk
      } else if (ep == 3) {
        memcpy(data.ep3_buffer.in, xfer->buffer, len);
      } else {
        memcpy(data.buffer[ep][TUSB_DIR_IN], xfer->buffer, len);
      }
      xfer->buffer += len;
      xfer->len -= len;
      xfer->processed_len += len;

      EP_TX_LEN(ep) = len;
      if (ep == 0) {
        EP_TX_CTRL(0) = USBFS_EP_T_RES_ACK | (data.ep0_tog ? USBFS_EP_T_TOG : 0);
        data.ep0_tog = !data.ep0_tog;
      } else if (data.isochronous[ep]) {
        EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK)) | USBFS_EP_T_RES_NYET;
      } else {
        EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK)) | USBFS_EP_T_RES_ACK;
      }
    } else {
      xfer->valid = false;
      EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK)) | USBFS_EP_T_RES_NAK;
      dcd_event_xfer_complete(
          rhport, ep | TUSB_DIR_IN_MASK, xfer->processed_len,
          XFER_RESULT_SUCCESS, true);
    }
  }
}

static void update_out(uint8_t rhport, uint8_t ep, size_t rx_len) {
  struct usb_xfer* xfer = &data.xfer[ep][TUSB_DIR_OUT];
  if (xfer->valid) {
    size_t len = TU_MIN(xfer->max_size, TU_MIN(xfer->len, rx_len));
    if (ep == 3) {
      memcpy(xfer->buffer, data.ep3_buffer.out, len);
    } else {
      memcpy(xfer->buffer, data.buffer[ep][TUSB_DIR_OUT], len);
    }
    xfer->buffer += len;
    xfer->len -= len;
    xfer->processed_len += len;

    if (xfer->len == 0 || len < xfer->max_size) {
      xfer->valid = false;
      dcd_event_xfer_complete(rhport, ep, xfer->processed_len, XFER_RESULT_SUCCESS, true);
    }

    if (ep == 0) {
      EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
    }
  }
}

/* public functions */
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  // init registers
  USBOTG_FS->BASE_CTRL = USBFS_CTRL_SYS_CTRL | USBFS_CTRL_INT_BUSY | USBFS_CTRL_DMA_EN;
  USBOTG_FS->UDEV_CTRL = USBFS_UDEV_CTRL_PD_DIS | USBFS_UDEV_CTRL_PORT_EN;
  USBOTG_FS->DEV_ADDR = 0x00;

  USBOTG_FS->INT_FG = 0xFF;
  USBOTG_FS->INT_EN = USBFS_INT_EN_BUS_RST | USBFS_INT_EN_TRANSFER | USBFS_INT_EN_SUSPEND;

  // setup endpoint 0
  EP_DMA(0) = (uint32_t) &data.buffer[0][0];
  EP_TX_LEN(0) = 0;
  EP_TX_CTRL(0) = USBFS_EP_T_RES_NAK;
  EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;

  // enable other endpoints but NAK everything
  USBOTG_FS->UEP4_1_MOD = 0xCC;
  USBOTG_FS->UEP2_3_MOD = 0xCC;
  USBOTG_FS->UEP5_6_MOD = 0xCC;
  USBOTG_FS->UEP7_MOD = 0x0C;

  for (uint8_t ep = 1; ep < EP_MAX; ep++) {
    EP_DMA(ep) = (uint32_t) &data.buffer[ep][0];
    EP_TX_LEN(ep) = 0;
    EP_TX_CTRL(ep) = USBFS_EP_T_AUTO_TOG | USBFS_EP_T_RES_NAK;
    EP_RX_CTRL(ep) = USBFS_EP_R_AUTO_TOG | USBFS_EP_R_RES_NAK;
  }
  EP_DMA(3) = (uint32_t) &data.ep3_buffer.out[0];

  dcd_connect(rhport);

  return true;
}

void dcd_int_handler(uint8_t rhport) {
  (void) rhport;
  uint8_t status = USBOTG_FS->INT_FG;
  if (status & USBFS_INT_FG_TRANSFER) {
    uint8_t ep = USBFS_INT_ST_MASK_UIS_ENDP(USBOTG_FS->INT_ST);
    uint8_t token = USBFS_INT_ST_MASK_UIS_TOKEN(USBOTG_FS->INT_ST);

    switch (token) {
      case PID_OUT: {
        uint16_t rx_len = USBOTG_FS->RX_LEN;
        update_out(rhport, ep, rx_len);
        break;
      }

      case PID_IN:
        update_in(rhport, ep, false);
        break;

      case PID_SETUP:
        // setup clears stall
        EP_TX_CTRL(0) = USBFS_EP_T_RES_NAK;
        EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;

        data.ep0_tog = true;
        dcd_event_setup_received(rhport, &data.buffer[0][TUSB_DIR_OUT][0], true);
        break;
    }

    USBOTG_FS->INT_FG = USBFS_INT_FG_TRANSFER;
  } else if (status & USBFS_INT_FG_BUS_RST) {
    data.ep0_tog = true;
    data.xfer[0][TUSB_DIR_OUT].max_size = 64;
    data.xfer[0][TUSB_DIR_IN].max_size = 64;

    dcd_event_bus_signal(rhport, DCD_EVENT_BUS_RESET, true);

    USBOTG_FS->DEV_ADDR = 0x00;
    EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;

    USBOTG_FS->INT_FG = USBFS_INT_FG_BUS_RST;
  } else if (status & USBFS_INT_FG_SUSPEND) {
    dcd_event_t event = {.rhport = rhport, .event_id = DCD_EVENT_SUSPEND};
    dcd_event_handler(&event, true);
    USBOTG_FS->INT_FG = USBFS_INT_FG_SUSPEND;
  }
}

void dcd_int_enable(uint8_t rhport) {
  (void) rhport;
  NVIC_EnableIRQ(USBHD_IRQn);
}

void dcd_int_disable(uint8_t rhport) {
  (void) rhport;
  NVIC_DisableIRQ(USBHD_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void) dev_addr;
  dcd_edpt_xfer(rhport, 0x80, NULL, 0); // zlp status response
}

void dcd_remote_wakeup(uint8_t rhport) {
  (void) rhport;
  // TODO optional
}

void dcd_connect(uint8_t rhport) {
  (void) rhport;
  USBOTG_FS->BASE_CTRL |= USBFS_CTRL_DEV_PUEN;
}

void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
  USBOTG_FS->BASE_CTRL &= ~USBFS_CTRL_DEV_PUEN;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void) rhport;
  (void) en;

  // TODO implement later
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const* request) {
  (void) rhport;
  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
      request->bRequest == TUSB_REQ_SET_ADDRESS) {
    USBOTG_FS->DEV_ADDR = (uint8_t) request->wValue;
  }
  EP_TX_CTRL(0) = USBFS_EP_T_RES_NAK;
  EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_ep) {
  (void) rhport;
  uint8_t ep = tu_edpt_number(desc_ep->bEndpointAddress);
  uint8_t dir = tu_edpt_dir(desc_ep->bEndpointAddress);
  TU_ASSERT(ep < EP_MAX);

  data.isochronous[ep] = desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS;
  data.xfer[ep][dir].max_size = tu_edpt_packet_size(desc_ep);

  if (ep != 0) {
    if (dir == TUSB_DIR_OUT) {
      if (data.isochronous[ep]) {
        EP_RX_CTRL(ep) = USBFS_EP_R_AUTO_TOG | USBFS_EP_R_RES_NYET;
      } else {
        EP_RX_CTRL(ep) = USBFS_EP_R_AUTO_TOG | USBFS_EP_R_RES_ACK;
      }
    } else {
      EP_TX_LEN(ep) = 0;
      EP_TX_CTRL(ep) = USBFS_EP_T_AUTO_TOG | USBFS_EP_T_RES_NAK;
    }
  }
  return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
  (void) rhport;
  // TODO optional
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  (void) ep_addr;
  // TODO optional
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  (void) rhport;
  uint8_t ep = tu_edpt_number(ep_addr);
  uint8_t dir = tu_edpt_dir(ep_addr);

  struct usb_xfer* xfer = &data.xfer[ep][dir];
  dcd_int_disable(rhport);
  xfer->valid = true;
  xfer->buffer = buffer;
  xfer->len = total_bytes;
  xfer->processed_len = 0;
  dcd_int_enable(rhport);

  if (dir == TUSB_DIR_IN) {
    update_in(rhport, ep, true);
  }
  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  uint8_t ep = tu_edpt_number(ep_addr);
  uint8_t dir = tu_edpt_dir(ep_addr);
  if (ep == 0) {
    if (dir == TUSB_DIR_OUT) {
      EP_RX_CTRL(0) = USBFS_EP_R_RES_STALL;
    } else {
      EP_TX_LEN(0) = 0;
      EP_TX_CTRL(0) = USBFS_EP_T_RES_STALL;
    }
  } else {
    if (dir == TUSB_DIR_OUT) {
      EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBFS_EP_R_RES_MASK) | USBFS_EP_R_RES_STALL;
    } else {
      EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBFS_EP_T_RES_MASK) | USBFS_EP_T_RES_STALL;
    }
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  uint8_t ep = tu_edpt_number(ep_addr);
  uint8_t dir = tu_edpt_dir(ep_addr);
  if (ep == 0) {
    if (dir == TUSB_DIR_OUT) {
      EP_RX_CTRL(0) = USBFS_EP_R_RES_ACK;
    }
  } else {
    if (dir == TUSB_DIR_OUT) {
      EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~(USBFS_EP_R_RES_MASK | USBFS_EP_R_TOG)) | USBFS_EP_R_RES_ACK;
    } else {
      EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~(USBFS_EP_T_RES_MASK | USBFS_EP_T_TOG)) | USBFS_EP_T_RES_NAK;
    }
  }
}

#endif
