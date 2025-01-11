/*
* The MIT License (MIT)
*
* Copyright (c) 2018, hathach (tinyusb.org)
* Copyright (c) 2021, HiFiPhile
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

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_SAMX7X

#include "device/dcd.h"
#include "sam.h"
#include "common_usb_regs.h"
//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// Since TinyUSB doesn't use SOF for now, and this interrupt too often (1ms interval)
// We disable SOF for now until needed later on
#ifndef USE_SOF
#  define USE_SOF         0
#endif

// Dual bank can improve performance, but need 2 times bigger packet buffer
// As SAM7x has only 4KB packet buffer, use with caution !
// Enable in FS mode as packets are smaller
#ifndef USE_DUAL_BANK
#  if TUD_OPT_HIGH_SPEED
#    define USE_DUAL_BANK   0
#  else
#    define USE_DUAL_BANK   1
#  endif
#endif

#define EP_GET_FIFO_PTR(ep, scale) (((TU_XSTRCAT(TU_STRCAT(uint, scale),_t) (*)[0x8000 / ((scale) / 8)])FIFO_RAM_ADDR)[(ep)])

// DMA Channel Transfer Descriptor
typedef struct {
  volatile uint32_t next_desc;
  volatile uint32_t buff_addr;
  volatile uint32_t chnl_ctrl;
  uint32_t padding;
} dma_desc_t;

// Transfer control context
typedef struct {
  uint8_t * buffer;
  uint16_t total_len;
  uint16_t queued_len;
  uint16_t max_packet_size;
  uint8_t interval;
  tu_fifo_t * fifo;
} xfer_ctl_t;

static tusb_speed_t get_speed(void);
static void dcd_transmit_packet(xfer_ctl_t * xfer, uint8_t ep_ix);

// DMA descriptors shouldn't be placed in ITCM !
CFG_TUD_MEM_SECTION static dma_desc_t dma_desc[6];

static xfer_ctl_t xfer_status[EP_MAX];

static const tusb_desc_endpoint_t ep0_desc =
{
  .bEndpointAddress = 0x00,
  .wMaxPacketSize   = CFG_TUD_ENDPOINT0_SIZE,
};

TU_ATTR_ALWAYS_INLINE static inline void CleanInValidateCache(uint32_t *addr, int32_t size)
{
  if (SCB->CCR & SCB_CCR_DC_Msk)
  {
    SCB_CleanInvalidateDCache_by_Addr(addr, size);
  }
  else
  {
    __DSB();
    __ISB();
  }
}
//------------------------------------------------------------------
// Device API
//------------------------------------------------------------------

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  dcd_connect(rhport);
  return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ((IRQn_Type) ID_USBHS);
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ((IRQn_Type) ID_USBHS);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) dev_addr;
  // DCD can only set address after status for this request is complete
  // do it at dcd_edpt0_status_complete()

  // Response with zlp status
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
  (void) rhport;
  USB_REG->DEVCTRL |= DEVCTRL_RMWKUP;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  dcd_int_disable(rhport);
  // Enable the USB controller in device mode
  USB_REG->CTRL = CTRL_UIMOD | CTRL_USBE;
  while (!(USB_REG->SR & SR_CLKUSABLE));
#if TUD_OPT_HIGH_SPEED
  USB_REG->DEVCTRL &= ~DEVCTRL_SPDCONF;
#else
  USB_REG->DEVCTRL |= DEVCTRL_SPDCONF_LOW_POWER;
#endif
  // Enable the End Of Reset, Suspend & Wakeup interrupts
  USB_REG->DEVIER = (DEVIER_EORSTES | DEVIER_SUSPES | DEVIER_WAKEUPES);
#if USE_SOF
  USB_REG->DEVIER = DEVIER_SOFES;
#endif
  // Clear the End Of Reset, SOF & Wakeup interrupts
  USB_REG->DEVICR = (DEVICR_EORSTC | DEVICR_SOFC | DEVICR_WAKEUPC);
  // Manually set the Suspend Interrupt
  USB_REG->DEVIFR |= DEVIFR_SUSPS;
  // Ack the Wakeup Interrupt
  USB_REG->DEVICR = DEVICR_WAKEUPC;
  // Attach the device
  USB_REG->DEVCTRL &= ~DEVCTRL_DETACH;
  // Freeze USB clock
  USB_REG->CTRL |= CTRL_FRZCLK;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  dcd_int_disable(rhport);
  // Disable all endpoints
  USB_REG->DEVEPT &= ~(0x3FF << DEVEPT_EPEN0_Pos);
  // Unfreeze USB clock
  USB_REG->CTRL &= ~CTRL_FRZCLK;
  while (!(USB_REG->SR & SR_CLKUSABLE));
  // Clear all the pending interrupts
  USB_REG->DEVICR = DEVICR_Msk;
  // Disable all interrupts
  USB_REG->DEVIDR = DEVIDR_Msk;
  // Detach the device
  USB_REG->DEVCTRL |= DEVCTRL_DETACH;
  // Disable the device address
  USB_REG->DEVCTRL &=~(DEVCTRL_ADDEN | DEVCTRL_UADD);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

static tusb_speed_t get_speed(void)
{
  switch (USB_REG->SR & SR_SPEED) {
  case SR_SPEED_FULL_SPEED:
  default:
    return TUSB_SPEED_FULL;
  case SR_SPEED_HIGH_SPEED:
    return TUSB_SPEED_HIGH;
  case SR_SPEED_LOW_SPEED:
    return TUSB_SPEED_LOW;
  }
}

static void dcd_ep_handler(uint8_t ep_ix)
{
  uint32_t int_status = USB_REG->DEVEPTISR[ep_ix];
  int_status &= USB_REG->DEVEPTIMR[ep_ix];

  uint16_t count = (USB_REG->DEVEPTISR[ep_ix] &
                    DEVEPTISR_BYCT) >> DEVEPTISR_BYCT_Pos;
  xfer_ctl_t *xfer = &xfer_status[ep_ix];

  if (ep_ix == 0U)
  {
    static uint8_t ctrl_dir;

    if (int_status & DEVEPTISR_CTRL_RXSTPI)
    {
      ctrl_dir = (USB_REG->DEVEPTISR[0] & DEVEPTISR_CTRL_CTRLDIR) >> DEVEPTISR_CTRL_CTRLDIR_Pos;
      // Setup packet should always be 8 bytes. If not, ignore it, and try again.
      if (count == 8)
      {
        uint8_t *ptr = EP_GET_FIFO_PTR(0,8);
        dcd_event_setup_received(0, ptr, true);
      }
      // Ack and disable SETUP interrupt
      USB_REG->DEVEPTICR[0] = DEVEPTICR_CTRL_RXSTPIC;
      USB_REG->DEVEPTIDR[0] = DEVEPTIDR_CTRL_RXSTPEC;
    }
    if (int_status & DEVEPTISR_RXOUTI)
    {
      uint8_t *ptr = EP_GET_FIFO_PTR(0,8);

      if (count && xfer->total_len)
      {
        uint16_t remain = xfer->total_len - xfer->queued_len;
        if (count > remain)
        {
          count = remain;
        }
        if (xfer->buffer)
        {
          memcpy(xfer->buffer + xfer->queued_len, ptr, count);
        } else
        {
          tu_fifo_write_n(xfer->fifo, ptr, count);
        }
        xfer->queued_len = (uint16_t)(xfer->queued_len + count);
      }
      // Acknowledge the interrupt
      USB_REG->DEVEPTICR[0] = DEVEPTICR_RXOUTIC;
      if ((count < xfer->max_packet_size) || (xfer->queued_len == xfer->total_len))
      {
        // RX COMPLETE
        dcd_event_xfer_complete(0, 0, xfer->queued_len, XFER_RESULT_SUCCESS, true);
        // Disable the interrupt
        USB_REG->DEVEPTIDR[0] = DEVEPTIDR_RXOUTEC;
        // Re-enable SETUP interrupt
        if (ctrl_dir == 1)
        {
          USB_REG->DEVEPTIER[0] = DEVEPTIER_CTRL_RXSTPES;
        }
      }
    }
    if (int_status & DEVEPTISR_TXINI)
    {
      // Disable the interrupt
      USB_REG->DEVEPTIDR[0] = DEVEPTIDR_TXINEC;
      if ((xfer->total_len != xfer->queued_len))
      {
        // TX not complete
        dcd_transmit_packet(xfer, 0);
      } else
      {
        // TX complete
        dcd_event_xfer_complete(0, 0x80 + 0, xfer->total_len, XFER_RESULT_SUCCESS, true);
        // Re-enable SETUP interrupt
        if (ctrl_dir == 0)
        {
          USB_REG->DEVEPTIER[0] = DEVEPTIER_CTRL_RXSTPES;
        }
      }
    }
  } else
  {
    if (int_status & DEVEPTISR_RXOUTI)
    {
      if (count && xfer->total_len)
      {
        uint16_t remain = xfer->total_len - xfer->queued_len;
        if (count > remain)
        {
          count = remain;
        }
        uint8_t *ptr = EP_GET_FIFO_PTR(ep_ix,8);
        if (xfer->buffer)
        {
          memcpy(xfer->buffer + xfer->queued_len, ptr, count);
        } else {
          tu_fifo_write_n(xfer->fifo, ptr, count);
        }
        xfer->queued_len = (uint16_t)(xfer->queued_len + count);
      }
      // Clear the FIFO control flag to receive more data.
      USB_REG->DEVEPTIDR[ep_ix] = DEVEPTIDR_FIFOCONC;
      // Acknowledge the interrupt
      USB_REG->DEVEPTICR[ep_ix] = DEVEPTICR_RXOUTIC;
      if ((count < xfer->max_packet_size) || (xfer->queued_len == xfer->total_len))
      {
        // RX COMPLETE
        dcd_event_xfer_complete(0, ep_ix, xfer->queued_len, XFER_RESULT_SUCCESS, true);
        // Disable the interrupt
        USB_REG->DEVEPTIDR[ep_ix] = DEVEPTIDR_RXOUTEC;
        // Though the host could still send, we don't know.
      }
    }
    if (int_status & DEVEPTISR_TXINI)
    {
      // Acknowledge the interrupt
      USB_REG->DEVEPTICR[ep_ix] = DEVEPTICR_TXINIC;
      if ((xfer->total_len != xfer->queued_len))
      {
        // TX not complete
        dcd_transmit_packet(xfer, ep_ix);
      } else
      {
        // TX complete
        dcd_event_xfer_complete(0, 0x80 + ep_ix, xfer->total_len, XFER_RESULT_SUCCESS, true);
        // Disable the interrupt
        USB_REG->DEVEPTIDR[ep_ix] = DEVEPTIDR_TXINEC;
      }
    }
  }
}

static void dcd_dma_handler(uint8_t ep_ix)
{
  uint32_t status = USB_REG->DEVDMA[ep_ix - 1].DEVDMASTATUS;
  if (status & DEVDMASTATUS_CHANN_ENB)
  {
    return; // Ignore EOT_STA interrupt
  }
  // Disable DMA interrupt
  USB_REG->DEVIDR = DEVIDR_DMA_1 << (ep_ix - 1);

  xfer_ctl_t *xfer = &xfer_status[ep_ix];
  uint16_t count = xfer->total_len - ((status & DEVDMASTATUS_BUFF_COUNT) >> DEVDMASTATUS_BUFF_COUNT_Pos);
  if(USB_REG->DEVEPTCFG[ep_ix] & DEVEPTCFG_EPDIR)
  {
    dcd_event_xfer_complete(0, 0x80 + ep_ix, count, XFER_RESULT_SUCCESS, true);
  } else
  {
    dcd_event_xfer_complete(0, ep_ix, count, XFER_RESULT_SUCCESS, true);
  }
}

void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;
  uint32_t int_status = USB_REG->DEVISR;
  int_status &= USB_REG->DEVIMR;
  // End of reset interrupt
  if (int_status & DEVISR_EORST)
  {
    // Unfreeze USB clock
    USB_REG->CTRL &= ~CTRL_FRZCLK;
    while(!(USB_REG->SR & SR_CLKUSABLE));
    // Reset all endpoints
    for (int ep_ix = 1; ep_ix < EP_MAX; ep_ix++)
    {
      USB_REG->DEVEPT |= 1 << (DEVEPT_EPRST0_Pos + ep_ix);
      USB_REG->DEVEPT &=~(1 << (DEVEPT_EPRST0_Pos + ep_ix));
    }
    dcd_edpt_open (0, &ep0_desc);
    USB_REG->DEVICR = DEVICR_EORSTC;
    USB_REG->DEVICR = DEVICR_WAKEUPC;
    USB_REG->DEVICR = DEVICR_SUSPC;
    USB_REG->DEVIER = DEVIER_SUSPES;

    dcd_event_bus_reset(rhport, get_speed(), true);
  }
  // End of Wakeup interrupt
  if (int_status & DEVISR_WAKEUP)
  {
    USB_REG->CTRL &= ~CTRL_FRZCLK;
    while (!(USB_REG->SR & SR_CLKUSABLE));
    USB_REG->DEVICR = DEVICR_WAKEUPC;
    USB_REG->DEVIDR = DEVIDR_WAKEUPEC;
    USB_REG->DEVIER = DEVIER_SUSPES;

    dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
  }
  // Suspend interrupt
  if (int_status & DEVISR_SUSP)
  {
    // Unfreeze USB clock
    USB_REG->CTRL &= ~CTRL_FRZCLK;
    while (!(USB_REG->SR & SR_CLKUSABLE));
    USB_REG->DEVICR = DEVICR_SUSPC;
    USB_REG->DEVIDR = DEVIDR_SUSPEC;
    USB_REG->DEVIER = DEVIER_WAKEUPES;
    USB_REG->CTRL |= CTRL_FRZCLK;

    dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
  }
#if USE_SOF
  if(int_status & DEVISR_SOF)
  {
    USB_REG->DEVICR = DEVICR_SOFC;

    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  }
#endif
  // Endpoints interrupt
  for (int ep_ix = 0; ep_ix < EP_MAX; ep_ix++)
  {
    if (int_status & (DEVISR_PEP_0 << ep_ix))
    {
      dcd_ep_handler(ep_ix);
    }
  }
  // Endpoints DMA interrupt
  for (int ep_ix = 0; ep_ix < EP_MAX; ep_ix++)
  {
    if (EP_DMA_SUPPORT(ep_ix))
    {
      if (int_status & (DEVISR_DMA_1 << (ep_ix - 1)))
      {
        dcd_dma_handler(ep_ix);
      }
    }
  }
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;

  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bRequest == TUSB_REQ_SET_ADDRESS )
  {
    uint8_t const dev_addr = (uint8_t) request->wValue;

    USB_REG->DEVCTRL |= dev_addr | DEVCTRL_ADDEN;
  }
}

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(ep_desc->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(ep_desc->bEndpointAddress);
  uint16_t const epMaxPktSize = tu_edpt_packet_size(ep_desc);
  tusb_xfer_type_t const eptype = (tusb_xfer_type_t)ep_desc->bmAttributes.xfer;
  uint8_t fifoSize = 0;                       // FIFO size
  uint16_t defaultEndpointSize = 8;           // Default size of Endpoint
  // Find upper 2 power number of epMaxPktSize
  if (epMaxPktSize)
  {
    while (defaultEndpointSize < epMaxPktSize)
    {
      fifoSize++;
      defaultEndpointSize <<= 1;
    }
  }
  xfer_status[epnum].max_packet_size = epMaxPktSize;

  USB_REG->DEVEPT |= 1 << (DEVEPT_EPRST0_Pos + epnum);
  USB_REG->DEVEPT &=~(1 << (DEVEPT_EPRST0_Pos + epnum));

  if (epnum == 0)
  {
    // Enable the control endpoint - Endpoint 0
    USB_REG->DEVEPT |= DEVEPT_EPEN0;
    // Configure the Endpoint 0 configuration register
    USB_REG->DEVEPTCFG[0] =
      (
       (fifoSize << DEVEPTCFG_EPSIZE_Pos)            |
       (TUSB_XFER_CONTROL << DEVEPTCFG_EPTYPE_Pos)   |
       (DEVEPTCFG_EPBK_1_BANK << DEVEPTCFG_EPBK_Pos) |
       DEVEPTCFG_ALLOC
       );
    USB_REG->DEVEPTIER[0] = DEVEPTIER_RSTDTS;
    USB_REG->DEVEPTIDR[0] = DEVEPTIDR_CTRL_STALLRQC;
    if (DEVEPTISR_CFGOK == (USB_REG->DEVEPTISR[0] & DEVEPTISR_CFGOK))
    {
      // Endpoint configuration is successful
      USB_REG->DEVEPTIER[0] = DEVEPTIER_CTRL_RXSTPES;
      // Enable Endpoint 0 Interrupts
      USB_REG->DEVIER = DEVIER_PEP_0;
      return true;
    } else
    {
      // Endpoint configuration is not successful
      return false;
    }
  } else
  {
    // Enable the endpoint
    USB_REG->DEVEPT |= ((0x01 << epnum) << DEVEPT_EPEN0_Pos);
    // Set up the maxpacket size, fifo start address fifosize
    // and enable the interrupt. CLear the data toggle.
    // AUTOSW is needed for DMA ack !
    USB_REG->DEVEPTCFG[epnum] =
      (
       (fifoSize << DEVEPTCFG_EPSIZE_Pos)            |
       (eptype  << DEVEPTCFG_EPTYPE_Pos)             |
       (DEVEPTCFG_EPBK_1_BANK << DEVEPTCFG_EPBK_Pos) |
       DEVEPTCFG_AUTOSW |
       ((dir & 0x01) << DEVEPTCFG_EPDIR_Pos)
       );
    if (eptype == TUSB_XFER_ISOCHRONOUS)
    {
      USB_REG->DEVEPTCFG[epnum] |= DEVEPTCFG_NBTRANS_1_TRANS;
    }
#if USE_DUAL_BANK
    if (eptype == TUSB_XFER_ISOCHRONOUS || eptype == TUSB_XFER_BULK)
    {
      USB_REG->DEVEPTCFG[epnum] |= DEVEPTCFG_EPBK_2_BANK;
    }
#endif
    USB_REG->DEVEPTCFG[epnum] |= DEVEPTCFG_ALLOC;
    USB_REG->DEVEPTIER[epnum] = DEVEPTIER_RSTDTS;
    USB_REG->DEVEPTIDR[epnum] = DEVEPTIDR_CTRL_STALLRQC;
    if (DEVEPTISR_CFGOK == (USB_REG->DEVEPTISR[epnum] & DEVEPTISR_CFGOK))
    {
      USB_REG->DEVIER = ((0x01 << epnum) << DEVIER_PEP_0_Pos);
      return true;
    } else
    {
      // Endpoint configuration is not successful
      return false;
    }
  }
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  uint8_t const epnum  = tu_edpt_number(ep_addr);

  // Disable endpoint interrupt
  USB_REG->DEVIDR = 1 << (DEVIDR_PEP_0_Pos + epnum);
  // Disable EP
  USB_REG->DEVEPT &=~(1 << (DEVEPT_EPEN0_Pos + epnum));
}

static void dcd_transmit_packet(xfer_ctl_t * xfer, uint8_t ep_ix)
{
  uint16_t len = (uint16_t)(xfer->total_len - xfer->queued_len);
  if (len)
  {
    if (len > xfer->max_packet_size)
    {
      len = xfer->max_packet_size;
    }
    uint8_t *ptr = EP_GET_FIFO_PTR(ep_ix,8);
    if(xfer->buffer)
    {
      memcpy(ptr, xfer->buffer + xfer->queued_len, len);
    }
    else
    {
      tu_fifo_read_n(xfer->fifo, ptr, len);
    }
    __DSB();
    __ISB();
    xfer->queued_len = (uint16_t)(xfer->queued_len + len);
  }
  if (ep_ix == 0U)
  {
    // Control endpoint: clear the interrupt flag to send the data
    USB_REG->DEVEPTICR[0] = DEVEPTICR_TXINIC;
  } else
  {
    // Other endpoint types: clear the FIFO control flag to send the data
    USB_REG->DEVEPTIDR[ep_ix] = DEVEPTIDR_FIFOCONC;
  }
  USB_REG->DEVEPTIER[ep_ix] = DEVEPTIER_TXINES;
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = &xfer_status[epnum];

  xfer->buffer = buffer;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;
  xfer->fifo = NULL;

  if (EP_DMA_SUPPORT(epnum) && total_bytes != 0)
  {
    // Force the CPU to flush the buffer. We increase the size by 32 because the call aligns the
    // address to 32-byte boundaries.
    CleanInValidateCache((uint32_t*) tu_align((uint32_t) buffer, 4), total_bytes + 31);
    uint32_t udd_dma_ctrl = total_bytes << DEVDMACONTROL_BUFF_LENGTH_Pos;
    if (dir == TUSB_DIR_OUT)
    {
      udd_dma_ctrl |= DEVDMACONTROL_END_TR_IT | DEVDMACONTROL_END_TR_EN;
    } else {
      udd_dma_ctrl |= DEVDMACONTROL_END_B_EN;
    }
    USB_REG->DEVDMA[epnum - 1].DEVDMAADDRESS = (uint32_t)buffer;
    udd_dma_ctrl |= DEVDMACONTROL_END_BUFFIT | DEVDMACONTROL_CHANN_ENB;
    // Disable IRQs to have a short sequence
    // between read of EOT_STA and DMA enable
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (!(USB_REG->DEVDMA[epnum - 1].DEVDMASTATUS & DEVDMASTATUS_END_TR_ST))
    {
      USB_REG->DEVDMA[epnum - 1].DEVDMACONTROL = udd_dma_ctrl;
      USB_REG->DEVIER = DEVIER_DMA_1 << (epnum - 1);
      __set_PRIMASK(irq_state);
      return true;
    }
    __set_PRIMASK(irq_state);

    // Here a ZLP has been received
    // and the DMA transfer must be not started.
    // It is the end of transfer
    return false;
  } else
  {
    if (dir == TUSB_DIR_OUT)
    {
      USB_REG->DEVEPTIER[epnum] = DEVEPTIER_RXOUTES;
    } else
    {
      dcd_transmit_packet(xfer,epnum);
    }
  }
  return true;
}

// The number of bytes has to be given explicitly to allow more flexible control of how many
// bytes should be written and second to keep the return value free to give back a boolean
// success message. If total_bytes is too big, the FIFO will copy only what is available
// into the USB buffer!
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = &xfer_status[epnum];
  if(epnum == 0x80)
    xfer = &xfer_status[EP_MAX];

  xfer->buffer = NULL;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;
  xfer->fifo = ff;

  if (EP_DMA_SUPPORT(epnum) && total_bytes != 0)
  {
    tu_fifo_buffer_info_t info;
    uint32_t udd_dma_ctrl_lin = DEVDMACONTROL_CHANN_ENB;
    uint32_t udd_dma_ctrl_wrap = DEVDMACONTROL_CHANN_ENB | DEVDMACONTROL_END_BUFFIT;
    if (dir == TUSB_DIR_OUT)
    {
      tu_fifo_get_write_info(ff, &info);
      udd_dma_ctrl_lin |= DEVDMACONTROL_END_TR_IT | DEVDMACONTROL_END_TR_EN;
      udd_dma_ctrl_wrap |= DEVDMACONTROL_END_TR_IT | DEVDMACONTROL_END_TR_EN;
    } else {
      tu_fifo_get_read_info(ff, &info);
      if(info.len_wrap == 0)
      {
        udd_dma_ctrl_lin |= DEVDMACONTROL_END_B_EN;
      }
      udd_dma_ctrl_wrap |= DEVDMACONTROL_END_B_EN;
    }

    // Clean invalidate cache of linear part
    CleanInValidateCache((uint32_t*) tu_align((uint32_t) info.ptr_lin, 4), info.len_lin + 31);

    USB_REG->DEVDMA[epnum - 1].DEVDMAADDRESS = (uint32_t)info.ptr_lin;
    if (info.len_wrap)
    {
      // Clean invalidate cache of wrapped part
      CleanInValidateCache((uint32_t*) tu_align((uint32_t) info.ptr_wrap, 4), info.len_wrap + 31);

      dma_desc[epnum - 1].next_desc = 0;
      dma_desc[epnum - 1].buff_addr = (uint32_t)info.ptr_wrap;
      dma_desc[epnum - 1].chnl_ctrl =
        udd_dma_ctrl_wrap | (info.len_wrap << DEVDMACONTROL_BUFF_LENGTH_Pos);
      // Clean cache of wrapped DMA descriptor
      CleanInValidateCache((uint32_t*)&dma_desc[epnum - 1], sizeof(dma_desc_t));

      udd_dma_ctrl_lin |= DEVDMASTATUS_DESC_LDST;
      USB_REG->DEVDMA[epnum - 1].DEVDMANXTDSC = (uint32_t)&dma_desc[epnum - 1];
    } else {
      udd_dma_ctrl_lin |= DEVDMACONTROL_END_BUFFIT;
    }
    udd_dma_ctrl_lin |= (info.len_lin << DEVDMACONTROL_BUFF_LENGTH_Pos);
    // Disable IRQs to have a short sequence
    // between read of EOT_STA and DMA enable
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (!(USB_REG->DEVDMA[epnum - 1].DEVDMASTATUS & DEVDMASTATUS_END_TR_ST))
    {
      USB_REG->DEVDMA[epnum - 1].DEVDMACONTROL = udd_dma_ctrl_lin;
      USB_REG->DEVIER = DEVIER_DMA_1 << (epnum - 1);
      __set_PRIMASK(irq_state);
      return true;
    }
    __set_PRIMASK(irq_state);

    // Here a ZLP has been received
    // and the DMA transfer must be not started.
    // It is the end of transfer
    return false;
  } else
  {
    if (dir == TUSB_DIR_OUT)
    {
      USB_REG->DEVEPTIER[epnum] = DEVEPTIER_RXOUTES;
    } else
    {
      dcd_transmit_packet(xfer,epnum);
    }
  }
  return true;
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(ep_addr);
  USB_REG->DEVEPTIER[epnum] = DEVEPTIER_CTRL_STALLRQS;
  // Re-enable SETUP interrupt
  if (epnum == 0)
  {
    USB_REG->DEVEPTIER[0] = DEVEPTIER_CTRL_RXSTPES;
  }
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(ep_addr);
  USB_REG->DEVEPTIDR[epnum] = DEVEPTIDR_CTRL_STALLRQC;
  USB_REG->DEVEPTIER[epnum] = HSTPIPIER_RSTDTS;
}

#endif
