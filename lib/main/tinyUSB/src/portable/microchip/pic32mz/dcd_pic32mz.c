/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Jerzy Kasenberg
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

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_PIC32MZ

#include <common/tusb_common.h>
#include <device/dcd.h>

#include <xc.h>
#include "usbhs_registers.h"

#define USB_REGS  ((usbhs_registers_t *) (_USB_BASE_ADDRESS))

// Maximum number of endpoints, could be trimmed down in tusb_config to reduce RAM usage.
#ifndef EP_MAX
#define EP_MAX            8
#endif


typedef enum {
  EP0_STAGE_NONE,
  EP0_STAGE_SETUP_IN_DATA,
  EP0_STAGE_SETUP_OUT_NO_DATA,
  EP0_STAGE_SETUP_OUT_DATA,
  EP0_STAGE_DATA_IN,
  EP0_STAGE_DATA_IN_LAST_PACKET_FILLED,
  EP0_STAGE_DATA_IN_SENT,
  EP0_STAGE_DATA_OUT,
  EP0_STAGE_DATA_OUT_COMPLETE,
  EP0_STAGE_STATUS_IN,
  EP0_STAGE_ADDRESS_CHANGE,
} ep0_stage_t;

typedef struct {
  uint8_t * buffer;
  // Total length of current transfer
  uint16_t total_len;
  // Bytes transferred so far
  uint16_t transferred;
  uint16_t max_packet_size;
  uint16_t fifo_size;
  // Packet size sent or received so far. It is used to modify transferred field
  // after ACK is received or when filling ISO endpoint with size larger then
  // FIFO size.
  uint16_t last_packet_size;
  uint8_t ep_addr;
} xfer_ctl_t;

static struct
{
  // Current FIFO RAM address used for FIFO allocation
  uint16_t fifo_addr_top;
  // EP0 transfer stage
  ep0_stage_t ep0_stage;
  // Device address
  uint8_t dev_addr;
  xfer_ctl_t xfer_status[EP_MAX][2];
} _dcd;

// Two endpoint 0 descriptor definition for unified dcd_edpt_open()
static tusb_desc_endpoint_t const ep0OUT_desc =
{
  .bLength          = sizeof(tusb_desc_endpoint_t),
  .bDescriptorType  = TUSB_DESC_ENDPOINT,

  .bEndpointAddress = 0x00,
  .bmAttributes     = { .xfer = TUSB_XFER_CONTROL },
  .wMaxPacketSize   = CFG_TUD_ENDPOINT0_SIZE,
  .bInterval        = 0
};

static tusb_desc_endpoint_t const ep0IN_desc =
{
  .bLength          = sizeof(tusb_desc_endpoint_t),
  .bDescriptorType  = TUSB_DESC_ENDPOINT,

  .bEndpointAddress = 0x80,
  .bmAttributes     = { .xfer = TUSB_XFER_CONTROL },
  .wMaxPacketSize   = CFG_TUD_ENDPOINT0_SIZE,
  .bInterval        = 0
};

#define XFER_CTL_BASE(_ep, _dir) &_dcd.xfer_status[_ep][_dir]

static void ep0_set_stage(ep0_stage_t stage)
{
  _dcd.ep0_stage = stage;
}

static ep0_stage_t ep0_get_stage(void)
{
  return _dcd.ep0_stage;
}

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  // Disable endpoint interrupts for now
  USB_REGS->INTRRXEbits.w = 0;
  USB_REGS->INTRTXEbits.w = 0;
  // Enable Reset/Suspend/Resume interrupts only
  USB_REGS->INTRUSBEbits.w = 7;

  dcd_connect(rhport);
  return true;
}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;

  USBCRCONbits.USBIE = 1;
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;

  USBCRCONbits.USBIE = 0;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;

  ep0_set_stage(EP0_STAGE_ADDRESS_CHANGE);
  // Store address it will be used later after status stage is done
  _dcd.dev_addr = dev_addr;
  // Confirm packet now, address will be set when status stage is detected
  USB_REGS->EPCSR[0].CSR0L_DEVICEbits.w = (USBHS_EP0_DEVICE_SERVICED_RXPKTRDY | USBHS_EP0_DEVICE_DATAEND);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

  USB_REGS->POWERbits.RESUME = 1;
#if CFG_TUSB_OS != OPT_OS_NONE
  osal_task_delay(10);
#else
  // TODO: Wait in non blocking mode
  unsigned cnt = 2000;
  while (cnt--) __asm__("nop");
#endif
  USB_REGS->POWERbits.RESUME = 0;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;

  USB_REGS->POWERbits.HSEN = TUD_OPT_HIGH_SPEED ? 1 : 0;
  USB_REGS->POWERbits.SOFTCONN = 1;
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;

  USB_REGS->POWERbits.SOFTCONN = 1;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

TU_ATTR_ALWAYS_INLINE static inline bool is_in_isr(void)
{
  return (_CP0_GET_STATUS() & (_CP0_STATUS_EXL_MASK | _CP0_STATUS_IPL_MASK)) != 0;
}

static void epn_rx_configure(uint8_t endpoint, uint16_t endpointSize,
                             uint16_t fifoAddress, uint8_t fifoSize,
                             uint32_t transferType)
{
  uint8_t old_index = USB_REGS->INDEXbits.ENDPOINT;

  // Select endpoint register set (same register address is used for all endpoints.
  USB_REGS->INDEXbits.ENDPOINT = endpoint;

  // Configure the Endpoint size
  USB_REGS->INDEXED_EPCSR.RXMAXPbits.RXMAXP = endpointSize;

  // Set up the fifo address.
  USB_REGS->RXFIFOADDbits.RXFIFOAD = fifoAddress;

  // Resets the endpoint data toggle to 0
  USB_REGS->INDEXED_EPCSR.RXCSRL_DEVICEbits.CLRDT = 1;

  // Set up the FIFO size
  USB_REGS->RXFIFOSZbits.RXFIFOSZ = fifoSize;

  USB_REGS->INDEXED_EPCSR.RXCSRH_DEVICEbits.ISO = transferType == 1 ? 1 : 0;
  // Disable NYET Handshakes for interrupt endpoints
  USB_REGS->INDEXED_EPCSR.RXCSRH_DEVICEbits.DISNYET = transferType == 3 ? 1 : 0;

  // Restore the index register.
  USB_REGS->INDEXbits.ENDPOINT = old_index;

  // Enable the endpoint interrupt.
  USB_REGS->INTRRXEbits.w |= (1 << endpoint);
}

static void epn_tx_configure(uint8_t endpoint, uint16_t endpointSize,
                             uint16_t fifoAddress, uint8_t fifoSize,
                             uint32_t transferType)
{
  uint8_t old_index = USB_REGS->INDEXbits.ENDPOINT;

  // Select endpoint register set (same register address is used for all endpoints.
  USB_REGS->INDEXbits.ENDPOINT = endpoint;

  // Configure the Endpoint size
  USB_REGS->INDEXED_EPCSR.TXMAXPbits.TXMAXP = endpointSize;

  // Set up the fifo address
  USB_REGS->TXFIFOADDbits.TXFIFOAD = fifoAddress;

  // Resets the endpoint data toggle to 0
  USB_REGS->INDEXED_EPCSR.TXCSRL_DEVICEbits.CLRDT = 1;

  // Set up the FIFO size
  USB_REGS->TXFIFOSZbits.TXFIFOSZ = fifoSize;

  USB_REGS->INDEXED_EPCSR.TXCSRH_DEVICEbits.ISO = 1 == transferType ? 1 : 0;

  // Restore the index register
  USB_REGS->INDEXbits.ENDPOINT = old_index;

  // Enable the interrupt
  USB_REGS->INTRTXEbits.w |=  (1 << endpoint);
}

static void tx_fifo_write(uint8_t endpoint, uint8_t const * buffer, size_t count)
{
  size_t i;
  volatile uint8_t * fifo_reg;

  fifo_reg = (volatile uint8_t *) (&USB_REGS->FIFO[endpoint]);

  for (i = 0; i < count; i++)
  {
    *fifo_reg = buffer[i];
  }
}

static int rx_fifo_read(uint8_t epnum, uint8_t * buffer)
{
  uint32_t i;
  uint32_t count;
  volatile uint8_t * fifo_reg;

  fifo_reg = (volatile uint8_t *) (&USB_REGS->FIFO[epnum]);

  count = USB_REGS->EPCSR[epnum].RXCOUNTbits.RXCNT;

  for (i = 0; i < count; i++)
  {
    buffer[i] = fifo_reg[i & 3];
  }

  return count;
}

static void xfer_complete(xfer_ctl_t * xfer, uint8_t result, bool in_isr)
{
  dcd_event_xfer_complete(0, xfer->ep_addr, xfer->transferred, result, in_isr);
}

static void ep0_fill_tx(xfer_ctl_t * xfer_in)
{
  uint16_t left = xfer_in->total_len - xfer_in->transferred;

  if (left)
  {
    xfer_in->last_packet_size = tu_min16(xfer_in->max_packet_size, left);
    tx_fifo_write(0, xfer_in->buffer + xfer_in->transferred, xfer_in->last_packet_size);
    xfer_in->transferred += xfer_in->last_packet_size;
    left = xfer_in->total_len - xfer_in->transferred;
  }

  if (xfer_in->last_packet_size < xfer_in->max_packet_size || left == 0)
  {
    switch (ep0_get_stage())
    {
      case EP0_STAGE_SETUP_IN_DATA:
      case EP0_STAGE_DATA_IN:
      case EP0_STAGE_DATA_IN_SENT:
        ep0_set_stage(EP0_STAGE_DATA_IN_LAST_PACKET_FILLED);
        USB_REGS->EPCSR[0].CSR0L_DEVICEbits.TXPKTRDY = 1;
        break;
      case EP0_STAGE_SETUP_OUT_NO_DATA:
        ep0_set_stage(EP0_STAGE_STATUS_IN);
        USB_REGS->EPCSR[0].CSR0L_DEVICEbits.w = (USBHS_EP0_DEVICE_SERVICED_RXPKTRDY | USBHS_EP0_DEVICE_DATAEND);
        break;
      case EP0_STAGE_DATA_OUT_COMPLETE:
        ep0_set_stage(EP0_STAGE_STATUS_IN);
        USB_REGS->EPCSR[0].CSR0L_DEVICEbits.w = (USBHS_EP0_DEVICE_SERVICED_RXPKTRDY | USBHS_EP0_DEVICE_DATAEND);
        break;
      default:
        break;
    }
  }
  else
  {
    switch (ep0_get_stage())
    {
      case EP0_STAGE_SETUP_IN_DATA:
        ep0_set_stage(EP0_STAGE_DATA_IN);
        // fall through
      case EP0_STAGE_DATA_IN:
        USB_REGS->EPCSR[0].CSR0L_DEVICEbits.TXPKTRDY = 1;
        break;
      default:
        break;
    }
  }
}

static void epn_fill_tx(xfer_ctl_t * xfer_in, uint8_t epnum)
{
  uint16_t left = xfer_in->total_len - xfer_in->transferred;
  if (left)
  {
    xfer_in->last_packet_size = tu_min16(xfer_in->max_packet_size, left);
    tx_fifo_write(epnum, xfer_in->buffer + xfer_in->transferred, xfer_in->last_packet_size);
  }
  USB_REGS->EPCSR[epnum].TXCSRL_DEVICEbits.TXPKTRDY = 1;
}

static bool ep0_xfer(xfer_ctl_t * xfer, int dir)
{
  if (dir == TUSB_DIR_OUT)
  {
    if (xfer->total_len)
    {
      switch (_dcd.ep0_stage)
      {
        case EP0_STAGE_DATA_OUT_COMPLETE:
        case EP0_STAGE_SETUP_OUT_DATA:
          ep0_set_stage(EP0_STAGE_DATA_OUT);
          USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SVCRPR = 1;
          break;
        default:
          TU_ASSERT(0);
      }
    }
    else
    {
      switch (_dcd.ep0_stage)
      {
        case EP0_STAGE_DATA_IN_SENT:
          ep0_set_stage(EP0_STAGE_NONE);
          // fall through
        case EP0_STAGE_NONE:
          xfer_complete(xfer, XFER_RESULT_SUCCESS, true);
          break;
        default:
          break;
      }
    }
  }
  else // IN
  {
    ep0_fill_tx(xfer);
  }

  return true;
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(desc_edpt->bEndpointAddress);
  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);

  TU_ASSERT(epnum < EP_MAX);

  xfer->max_packet_size = tu_edpt_packet_size(desc_edpt);
  xfer->fifo_size = xfer->max_packet_size;
  xfer->ep_addr = desc_edpt->bEndpointAddress;

  if (epnum != 0)
  {
    if (dir == TUSB_DIR_OUT)
    {
      epn_rx_configure(epnum, xfer->max_packet_size, _dcd.fifo_addr_top, __builtin_ctz(xfer->fifo_size) - 3, desc_edpt->bmAttributes.xfer);
      _dcd.fifo_addr_top += (xfer->fifo_size + 7) >> 3;
    }
    else
    {
      epn_tx_configure(epnum, xfer->max_packet_size, _dcd.fifo_addr_top, __builtin_ctz(xfer->fifo_size) - 3, desc_edpt->bmAttributes.xfer);
      _dcd.fifo_addr_top += (xfer->fifo_size + 7) >> 3;
    }
  }
  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;

  // Reserve EP0 FIFO address
  _dcd.fifo_addr_top = 64 >> 3;
  for (int i = 1; i < EP_MAX; ++i)
  {
    tu_memclr(&_dcd.xfer_status[i], sizeof(_dcd.xfer_status[i]));
  }
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  (void) ep_addr;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);
  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  (void) rhport;

  xfer->buffer = buffer;
  xfer->total_len = total_bytes;
  xfer->last_packet_size = 0;
  xfer->transferred = 0;

  if (epnum == 0)
  {
    return ep0_xfer(xfer, dir);
  }
  if (dir == TUSB_DIR_OUT)
  {
    USB_REGS->INTRRXEbits.w |= (1u << epnum);
  }
  else // IN
  {
    epn_fill_tx(xfer, epnum);
  }

  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);
  (void) rhport;

  if (epnum == 0)
  {
    USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SENDSTALL = 1;
  }
  else
  {
    if (dir == TUSB_DIR_OUT)
    {
      USB_REGS->EPCSR[epnum].RXCSRL_DEVICEbits.SENDSTALL = 1;
    }
    else
    {
      USB_REGS->EPCSR[epnum].TXCSRL_DEVICEbits.SENDSTALL = 1;
    }
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);
  (void) rhport;

  if (epnum == 0)
  {
    USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SENDSTALL = 0;
  }
  else
  {
    if (dir == TUSB_DIR_OUT)
    {
      USB_REGS->EPCSR[epnum].RXCSRL_DEVICEbits.w &= ~(USBHS_EP_DEVICE_RX_SENT_STALL | USBHS_EP_DEVICE_RX_SEND_STALL);
      USB_REGS->EPCSR[epnum].RXCSRL_DEVICEbits.CLRDT = 1;
    }
    else
    {
      USB_REGS->EPCSR[epnum].TXCSRL_DEVICEbits.w &= ~(USBHS_EP_DEVICE_TX_SENT_STALL | USBHS_EP_DEVICE_TX_SEND_STALL);
      USB_REGS->EPCSR[epnum].TXCSRL_DEVICEbits.CLRDT = 1;
    }
  }
}

/*------------------------------------------------------------------*/
/* Interrupt Handler
 *------------------------------------------------------------------*/

static void ep0_handle_rx(void)
{
  int transferred;
  xfer_ctl_t * xfer = XFER_CTL_BASE(0, TUSB_DIR_OUT);

  TU_ASSERT(xfer->buffer,);

  transferred = rx_fifo_read(0, xfer->buffer + xfer->transferred);
  xfer->transferred += transferred;
  TU_ASSERT(xfer->transferred <= xfer->total_len,);
  if (transferred < xfer->max_packet_size || xfer->transferred == xfer->total_len)
  {
    ep0_set_stage(EP0_STAGE_DATA_OUT_COMPLETE);
    xfer_complete(xfer, XFER_RESULT_SUCCESS, true);
  }
  else
  {
    USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SVCRPR = 1;
  }
}

static void epn_handle_rx_int(uint8_t epnum)
{
  uint8_t ep_status;
  int transferred;
  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, TUSB_DIR_OUT);

  ep_status = USB_REGS->EPCSR[epnum].RXCSRL_DEVICEbits.w;
  if (ep_status & USBHS_EP_DEVICE_RX_SENT_STALL)
  {
    USB_REGS->EPCSR[epnum].RXCSRL_DEVICEbits.w &= ~USBHS_EP_DEVICE_RX_SENT_STALL;
  }

  if (ep_status & USBHS_EP0_HOST_RXPKTRDY)
  {
    TU_ASSERT(xfer->buffer != NULL,);

    transferred = rx_fifo_read(epnum, xfer->buffer + xfer->transferred);
    USB_REGS->EPCSR[epnum].RXCSRL_HOSTbits.RXPKTRDY = 0;
    xfer->transferred += transferred;
    TU_ASSERT(xfer->transferred <= xfer->total_len,);
    if (transferred < xfer->max_packet_size || xfer->transferred == xfer->total_len)
    {
      USB_REGS->INTRRXEbits.w &= ~(1u << epnum);
      xfer_complete(xfer, XFER_RESULT_SUCCESS, true);
    }
  }
}

static void epn_handle_tx_int(uint8_t epnum)
{
  uint8_t ep_status = USB_REGS->EPCSR[epnum].TXCSRL_DEVICEbits.w;
  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, TUSB_DIR_IN);

  if (ep_status & USBHS_EP_DEVICE_TX_SENT_STALL)
  {
    USB_REGS->EPCSR[epnum].TXCSRL_DEVICEbits.w &= ~USBHS_EP_DEVICE_TX_SENT_STALL;
  }
  else
  {
    xfer->transferred += xfer->last_packet_size;
    TU_ASSERT(xfer->transferred <= xfer->total_len,);
    if (xfer->last_packet_size < xfer->max_packet_size || xfer->transferred == xfer->total_len)
    {
      xfer->last_packet_size = 0;
      xfer_complete(xfer, XFER_RESULT_SUCCESS, true);
    }
    else
    {
      epn_fill_tx(xfer, epnum);
    }
  }
}

static void ep0_handle_int(void)
{
  __USBHS_CSR0L_DEVICE_t  ep0_status;
  union {
    tusb_control_request_t request;
    uint32_t setup_buffer[2];
  } setup_packet;
  xfer_ctl_t * xfer_in = XFER_CTL_BASE(0, TUSB_DIR_IN);
  uint8_t old_index = USB_REGS->INDEXbits.ENDPOINT;

  // Select EP0 registers
  USB_REGS->INDEXbits.ENDPOINT = 0;

  ep0_status = USB_REGS->EPCSR[0].CSR0L_DEVICEbits;

  if (ep0_status.SENTSTALL)
  {
    // Stall was sent. Reset the endpoint 0 state.
    // Clear the sent stall bit.
    ep0_set_stage(EP0_STAGE_NONE);
    USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SENTSTALL = 0;
  }

  if (ep0_status.SETUPEND)
  {
    // This means the current control transfer end prematurely. We don't
    // need to end any transfers. The device layer will manage the
    // premature transfer end. We clear the SetupEnd bit and reset the
    // driver control transfer state machine to waiting for next setup
    // packet from host.
    USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SVSSETEND = 1;
    ep0_set_stage(EP0_STAGE_NONE);
  }

  if (ep0_status.RXPKTRDY)
  {
    switch (ep0_get_stage())
    {
      default:
        // Data arrived at unexpected state, this must be setup stage packet after all.
        // Fall through
      case EP0_STAGE_NONE:
        // This means we were expecting a SETUP packet and we got one.
        setup_packet.setup_buffer[0] = USB_REGS->FIFO[0];
        setup_packet.setup_buffer[1] = USB_REGS->FIFO[0];
        if (setup_packet.request.bmRequestType_bit.direction == TUSB_DIR_OUT)
        {
          // SVCRPR is not set yet, it will be set later when out xfer is started
          // Till then NAKs will hold incommint data
          ep0_set_stage(setup_packet.request.wLength == 0 ? EP0_STAGE_SETUP_OUT_NO_DATA : EP0_STAGE_SETUP_OUT_DATA);
        }
        else
        {
          USB_REGS->EPCSR[0].CSR0L_DEVICEbits.SVCRPR = 1;
          ep0_set_stage(EP0_STAGE_SETUP_IN_DATA);
        }
        dcd_event_setup_received(0, &setup_packet.request.bmRequestType, true);
        break;
      case EP0_STAGE_DATA_OUT:
        ep0_handle_rx();
        break;
    }
  }
  else
  {
    switch (ep0_get_stage())
    {
      case EP0_STAGE_STATUS_IN:
        // Status was just sent, this concludes request, notify client
        ep0_set_stage(EP0_STAGE_NONE);
        xfer_complete(xfer_in, XFER_RESULT_SUCCESS, true);
        break;
      case EP0_STAGE_DATA_IN:
        // Packet sent, fill more data
        ep0_fill_tx(xfer_in);
        break;
      case EP0_STAGE_DATA_IN_LAST_PACKET_FILLED:
        ep0_set_stage(EP0_STAGE_DATA_IN_SENT);
        xfer_complete(xfer_in, XFER_RESULT_SUCCESS, true);
        break;
      case EP0_STAGE_ADDRESS_CHANGE:
        // Status stage after set address request finished, address can be changed
        USB_REGS->FADDRbits.FUNC = _dcd.dev_addr;
        ep0_set_stage(EP0_STAGE_NONE);
        break;
      default:
        break;
    }
  }
  // Restore register index
  USB_REGS->INDEXbits.ENDPOINT = old_index;
}

void dcd_int_handler(uint8_t rhport)
{
  int i;
  uint8_t mask;
  __USBCSR2bits_t csr2_bits;
  uint16_t rxints = USB_REGS->INTRRX & USB_REGS->INTRRXEbits.w;
  uint16_t txints = USB_REGS->INTRTX;
  csr2_bits = USBCSR2bits;
  (void) rhport;

  IFS4CLR = _IFS4_USBIF_MASK;

  if (csr2_bits.SOFIF && csr2_bits.SOFIE)
  {
    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  }
  if (csr2_bits.RESETIF)
  {
    dcd_edpt_open(0, &ep0OUT_desc);
    dcd_edpt_open(0, &ep0IN_desc);
    dcd_event_bus_reset(0, USB_REGS->POWERbits.HSMODE ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL, true);
  }
  if (csr2_bits.SUSPIF)
  {
    dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
  }
  if (csr2_bits.RESUMEIF)
  {
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
  }
  // INTRTX has bit for EP0
  if (txints & 1)
  {
    txints ^= 1;
    ep0_handle_int();
  }
  for (mask = 0x02, i = 1; rxints != 0 && mask != 0; mask <<= 1, ++i)
  {
    if (rxints & mask)
    {
      rxints ^= mask;
      epn_handle_rx_int(i);
    }
  }
  for (mask = 0x02, i = 1; txints != 0 && mask != 0; mask <<= 1, ++i)
  {
    if (txints & mask)
    {
      txints ^= mask;
      epn_handle_tx_int(i);
    }
  }
}

#endif
