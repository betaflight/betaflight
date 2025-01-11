/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
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

#if CFG_TUSB_MCU == OPT_MCU_SAMG

#include "sam.h"
#include "device/dcd.h"

// TODO should support (SAM3S || SAM4S || SAM4E || SAMG55)

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define EP_COUNT    6

// Transfer descriptor
typedef struct
{
  uint8_t* buffer;
  // tu_fifo_t* ff; // TODO support dcd_edpt_xfer_fifo API
  uint16_t total_len;
  volatile uint16_t actual_len;
  uint16_t  epsize;
} xfer_desc_t;

// Endpoint 0-5, each can only be either OUT or In
xfer_desc_t _dcd_xfer[EP_COUNT];

TU_ATTR_ALWAYS_INLINE static inline void xfer_epsize_set(xfer_desc_t* xfer, uint16_t epsize) {
  xfer->epsize = epsize;
}

TU_ATTR_ALWAYS_INLINE static inline void xfer_begin(xfer_desc_t* xfer, uint8_t * buffer, uint16_t total_bytes) {
  xfer->buffer     = buffer;
  // xfer->ff         = NULL; // TODO support dcd_edpt_xfer_fifo API
  xfer->total_len  = total_bytes;
  xfer->actual_len = 0;
}

TU_ATTR_ALWAYS_INLINE static inline void xfer_end(xfer_desc_t* xfer) {
  xfer->buffer     = NULL;
  // xfer->ff         = NULL; // TODO support dcd_edpt_xfer_fifo API
  xfer->total_len  = 0;
  xfer->actual_len = 0;
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t xfer_packet_len(xfer_desc_t* xfer) {
  // also cover zero-length packet
  return tu_min16(xfer->total_len - xfer->actual_len, xfer->epsize);
}

TU_ATTR_ALWAYS_INLINE static inline void xfer_packet_done(xfer_desc_t* xfer) {
  uint16_t const xact_len = xfer_packet_len(xfer);
  xfer->buffer += xact_len;
  xfer->actual_len += xact_len;
}

//------------- Transaction helpers -------------//

// Write data to EP FIFO, return number of written bytes
static void xact_ep_write(uint8_t epnum, uint8_t* buffer, uint16_t xact_len) {
  for(uint16_t i=0; i<xact_len; i++) {
    UDP->UDP_FDR[epnum] = (uint32_t) buffer[i];
  }
}

// Read data from EP FIFO
static void xact_ep_read(uint8_t epnum, uint8_t* buffer, uint16_t xact_len) {
  for(uint16_t i=0; i<xact_len; i++) {
    buffer[i] = (uint8_t) UDP->UDP_FDR[epnum];
  }
}


//! Bitmap for all status bits in CSR that are not affected by a value 1.
#define CSR_NO_EFFECT_1_ALL (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1 | UDP_CSR_STALLSENT | UDP_CSR_RXSETUP | UDP_CSR_TXCOMP)

// Per Specs: CSR need synchronization each write
TU_ATTR_ALWAYS_INLINE static inline void csr_write(uint8_t epnum, uint32_t value) {
  uint32_t const csr = value;
  UDP->UDP_CSR[epnum] = csr;

  volatile uint32_t nop_count;
  for (nop_count = 0; nop_count < 20; nop_count ++) {
    __NOP();
  }
}

// Per Specs: CSR need synchronization each write
TU_ATTR_ALWAYS_INLINE static inline void csr_set(uint8_t epnum, uint32_t mask)
{
  csr_write(epnum, UDP->UDP_CSR[epnum] | CSR_NO_EFFECT_1_ALL | mask);
}

// Per Specs: CSR need synchronization each write
TU_ATTR_ALWAYS_INLINE static inline void csr_clear(uint8_t epnum, uint32_t mask) {
  csr_write(epnum, (UDP->UDP_CSR[epnum] | CSR_NO_EFFECT_1_ALL) & ~mask);
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Set up endpoint 0, clear all other endpoints
static void bus_reset(void)
{
  tu_memclr(_dcd_xfer, sizeof(_dcd_xfer));

  xfer_epsize_set(&_dcd_xfer[0], CFG_TUD_ENDPOINT0_SIZE);

  // Enable EP0 control
  csr_write(0, UDP_CSR_EPEDS_Msk);

  // Enable interrupt : EP0, Suspend, Resume, Wakeup
  UDP->UDP_IER = UDP_IER_EP0INT_Msk | UDP_IER_RXSUSP_Msk | UDP_IER_RXRSM_Msk | UDP_IER_WAKEUP_Msk;

  // Enable transceiver
  UDP->UDP_TXVC &= ~UDP_TXVC_TXVDIS_Msk;
}

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  tu_memclr(_dcd_xfer, sizeof(_dcd_xfer));
  dcd_connect(rhport);
  return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(UDP_IRQn);
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(UDP_IRQn);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;

  // Response with zlp status
  dcd_edpt_xfer(rhport, 0x80, NULL, 0);

  // DCD can only set address after status for this request is complete.
  // do it at dcd_edpt0_status_complete()
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
  (void) rhport;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;

  // Enable pull-up, disable transceiver
  UDP->UDP_TXVC = UDP_TXVC_PUON | UDP_TXVC_TXVDIS_Msk;
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;

  // disable both pullup and transceiver
  UDP->UDP_TXVC = UDP_TXVC_TXVDIS_Msk;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
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
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD )
  {
    if (request->bRequest == TUSB_REQ_SET_ADDRESS)
    {
      uint8_t const dev_addr = (uint8_t) request->wValue;

      // Enable addressed state
      UDP->UDP_GLB_STAT |= UDP_GLB_STAT_FADDEN_Msk;

      // Set new address & Function enable bit
      UDP->UDP_FADDR = UDP_FADDR_FEN_Msk | UDP_FADDR_FADD(dev_addr);
    }
    else if (request->bRequest == TUSB_REQ_SET_CONFIGURATION)
    {
      // Configured State
      UDP->UDP_GLB_STAT |= UDP_GLB_STAT_CONFG_Msk;
    }
  }
}

// Configure endpoint's registers according to descriptor
// SAMG doesn't support a same endpoint number with IN and OUT
//    e.g EP1 OUT & EP1 IN cannot exist together
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_desc->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(ep_desc->bEndpointAddress);

  // TODO Isochronous is not supported yet
  TU_VERIFY(ep_desc->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS);
  TU_VERIFY(epnum < EP_COUNT);

  // Must not already enabled
  TU_ASSERT((UDP->UDP_CSR[epnum] & UDP_CSR_EPEDS_Msk) == 0);

  xfer_epsize_set(&_dcd_xfer[epnum], tu_edpt_packet_size(ep_desc));

  // Configure type and enable EP
  csr_write(epnum, UDP_CSR_EPEDS_Msk | UDP_CSR_EPTYPE(ep_desc->bmAttributes.xfer + 4*dir));

  // Enable EP Interrupt for IN
  if (dir == TUSB_DIR_IN) UDP->UDP_IER |= (1 << epnum);

  return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport; (void) ep_addr;
  // TODO implement dcd_edpt_close()
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_desc_t* xfer = &_dcd_xfer[epnum];
  xfer_begin(xfer, buffer, total_bytes);

  if (dir == TUSB_DIR_OUT)
  {
    // Enable interrupt when starting OUT transfer
    if (epnum != 0) UDP->UDP_IER |= (1 << epnum);
  }
  else
  {
    xact_ep_write(epnum, xfer->buffer, xfer_packet_len(xfer));

    // TX ready for transfer
    csr_set(epnum, UDP_CSR_TXPKTRDY_Msk);
  }

  return true;
}

#if 0 // TODO support dcd_edpt_xfer_fifo API
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void) rhport;
  return true;
}
#endif

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  // For EP0 USBD will stall both EP0 Out and In with 0x00 and 0x80
  // only handle one by skipping 0x80
  if ( ep_addr == tu_edpt_addr(0, TUSB_DIR_IN_MASK) ) return;

  uint8_t const epnum = tu_edpt_number(ep_addr);

  // Set force stall bit
  csr_set(epnum, UDP_CSR_FORCESTALL_Msk);
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);

  // clear stall
  csr_clear(epnum, UDP_CSR_FORCESTALL_Msk);

  // must also reset EP to clear data toggle
  UDP->UDP_RST_EP |= (1 << epnum);
  UDP->UDP_RST_EP &= ~(1 << epnum);
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport)
{
  uint32_t const intr_mask   = UDP->UDP_IMR;
  uint32_t const intr_status = UDP->UDP_ISR & intr_mask;

  // clear interrupt
  UDP->UDP_ICR = intr_status;

  // Bus reset
  if (intr_status & UDP_ISR_ENDBUSRES_Msk)
  {
    bus_reset();
    dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
  }

  // SOF
//  if (intr_status & UDP_ISR_SOFINT_Msk) dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);

  // Suspend
  if (intr_status & UDP_ISR_RXSUSP_Msk) dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);

  // Resume
  if (intr_status & UDP_ISR_RXRSM_Msk)  dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);

  // Wakeup
  if (intr_status & UDP_ISR_WAKEUP_Msk)  dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);

  //------------- Endpoints -------------//

  if ( intr_status & TU_BIT(0) )
  {
    // setup packet
    if ( UDP->UDP_CSR[0] & UDP_CSR_RXSETUP )
    {
      // get setup from FIFO
      uint8_t setup[8];
      for(uint8_t i=0; i<sizeof(setup); i++)
      {
        setup[i] = (uint8_t) UDP->UDP_FDR[0];
      }

      // notify usbd
      dcd_event_setup_received(rhport, setup, true);

      // Set EP direction bit according to DATA stage
      // MUST only be set before RXSETUP is clear per specs
      if ( tu_edpt_dir(setup[0]) )
      {
        csr_set(0, UDP_CSR_DIR_Msk);
      }
      else
      {
        csr_clear(0, UDP_CSR_DIR_Msk);
      }

      // Clear Setup, stall and other on-going transfer bits
      csr_clear(0, UDP_CSR_RXSETUP_Msk | UDP_CSR_TXPKTRDY_Msk | UDP_CSR_TXCOMP_Msk | UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1 | UDP_CSR_STALLSENT_Msk | UDP_CSR_FORCESTALL_Msk);
    }
  }

  for(uint8_t epnum = 0; epnum < EP_COUNT; epnum++)
  {
    if ( intr_status & TU_BIT(epnum) )
    {
      xfer_desc_t* xfer = &_dcd_xfer[epnum];

      //------------- Endpoint IN -------------//
      if (UDP->UDP_CSR[epnum] & UDP_CSR_TXCOMP_Msk)
      {
        xfer_packet_done(xfer);

        uint16_t const xact_len = xfer_packet_len(xfer);

        if (xact_len)
        {
          // write to EP fifo
#if 0 // TODO support dcd_edpt_xfer_fifo
          if (xfer->ff)
          {
            tu_fifo_read_n_const_addr_full_words(xfer->ff, (void *) &UDP->UDP_FDR[epnum], xact_len);
          }
          else
#endif
          {
            xact_ep_write(epnum, xfer->buffer, xact_len);
          }

          // TX ready for transfer
          csr_set(epnum, UDP_CSR_TXPKTRDY_Msk);
        }else
        {
          // xfer is complete
          dcd_event_xfer_complete(rhport, epnum | TUSB_DIR_IN_MASK, xfer->actual_len, XFER_RESULT_SUCCESS, true);

          // Required since control OUT can happen right after before stack handle this event
          xfer_end(xfer);
        }

        // Clear TX Complete bit
        csr_clear(epnum, UDP_CSR_TXCOMP_Msk);
      }

      //------------- Endpoint OUT -------------//
      // Ping-Pong is a MUST for Bulk/Iso
      // NOTE: When both Bank0 and Bank1 are both set, there is no way to know which one comes first
      uint32_t const banks_complete = UDP->UDP_CSR[epnum] & (UDP_CSR_RX_DATA_BK0_Msk | UDP_CSR_RX_DATA_BK1_Msk);
      if (banks_complete)
      {
        uint16_t const xact_len = (uint16_t) ((UDP->UDP_CSR[epnum] & UDP_CSR_RXBYTECNT_Msk) >> UDP_CSR_RXBYTECNT_Pos);

        // Read from EP fifo
#if 0 // TODO support dcd_edpt_xfer_fifo API
        if (xfer->ff)
        {
          tu_fifo_write_n_const_addr_full_words(xfer->ff, (const void *) &UDP->UDP_FDR[epnum], xact_len);
        }
        else
#endif
        {
          xact_ep_read(epnum, xfer->buffer, xact_len);
        }

        xfer_packet_done(xfer);

        if ( 0 == xfer_packet_len(xfer) )
        {
          // Disable OUT EP interrupt when transfer is complete
          if (epnum != 0) UDP->UDP_IDR |= (1 << epnum);

          dcd_event_xfer_complete(rhport, epnum, xfer->actual_len, XFER_RESULT_SUCCESS, true);
          xfer_end(xfer);
        }

        // Clear DATA Bank0/1 bit
        csr_clear(epnum, banks_complete);
      }

      // Stall sent to host
      if (UDP->UDP_CSR[epnum] & UDP_CSR_STALLSENT_Msk)
      {
        csr_clear(epnum, UDP_CSR_STALLSENT_Msk);
      }
    }
  }
}

#endif
