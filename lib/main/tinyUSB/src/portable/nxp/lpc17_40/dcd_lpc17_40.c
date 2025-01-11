/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#if CFG_TUD_ENABLED && \
    (CFG_TUSB_MCU == OPT_MCU_LPC175X_6X || CFG_TUSB_MCU == OPT_MCU_LPC177X_8X || CFG_TUSB_MCU == OPT_MCU_LPC40XX)

#include "device/dcd.h"
#include "dcd_lpc17_40.h"
#include "chip.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
#define DCD_ENDPOINT_MAX 32

typedef struct TU_ATTR_ALIGNED(4)
{
  //------------- Word 0 -------------//
  uint32_t next;

  //------------- Word 1 -------------//
  uint16_t atle_mode       : 2; // 00: normal, 01: ATLE (auto length extraction)
  uint16_t next_valid      : 1;
  uint16_t                 : 1; ///< reserved
  uint16_t isochronous     : 1; // is an iso endpoint
  uint16_t max_packet_size : 11;

  volatile uint16_t buflen; // bytes for non-iso, number of packets for iso endpoint

  //------------- Word 2 -------------//
  volatile uint32_t buffer;

  //------------- Word 3 -------------//
  volatile uint16_t retired                : 1; // initialized to zero
  volatile uint16_t status                 : 4;
  volatile uint16_t iso_last_packet_valid  : 1;
  volatile uint16_t atle_lsb_extracted     : 1;	// used in ATLE mode
  volatile uint16_t atle_msb_extracted     : 1;	// used in ATLE mode
  volatile uint16_t atle_mess_len_position : 6; // used in ATLE mode
  uint16_t                                 : 2;

  volatile uint16_t present_count;  // For non-iso : The number of bytes transferred by the DMA engine
                                    // For iso : number of packets

  //------------- Word 4 -------------//
  //	uint32_t iso_packet_size_addr;		// iso only, can be omitted for non-iso
}dma_desc_t;

TU_VERIFY_STATIC( sizeof(dma_desc_t) == 16, "size is not correct"); // TODO not support ISO for now

typedef struct
{
  // must be 128 byte aligned
  volatile dma_desc_t* udca[DCD_ENDPOINT_MAX];

  // TODO DMA does not support control transfer (0-1 are not used, offset to reduce memory)
  dma_desc_t dd[DCD_ENDPOINT_MAX];

  struct
  {
    uint8_t* out_buffer;
    uint8_t  out_bytes;
    volatile bool out_received; // indicate if data is already received in endpoint

    uint8_t  in_bytes;
  } control;

} dcd_data_t;

CFG_TUD_MEM_SECTION TU_ATTR_ALIGNED(128) static dcd_data_t _dcd;


//--------------------------------------------------------------------+
// SIE Command
//--------------------------------------------------------------------+
static void sie_cmd_code (sie_cmdphase_t phase, uint8_t code_data)
{
  LPC_USB->DevIntClr = (DEV_INT_COMMAND_CODE_EMPTY_MASK | DEV_INT_COMMAND_DATA_FULL_MASK);
  LPC_USB->CmdCode   = (phase << 8) | (code_data << 16);

  uint32_t const wait_flag = (phase == SIE_CMDPHASE_READ) ? DEV_INT_COMMAND_DATA_FULL_MASK : DEV_INT_COMMAND_CODE_EMPTY_MASK;
  while ((LPC_USB->DevIntSt & wait_flag) == 0) {}

  LPC_USB->DevIntClr = wait_flag;
}

static void sie_write (uint8_t cmd_code, uint8_t data_len, uint8_t data)
{
  sie_cmd_code(SIE_CMDPHASE_COMMAND, cmd_code);

  if (data_len)
  {
    sie_cmd_code(SIE_CMDPHASE_WRITE, data);
  }
}

static uint8_t sie_read (uint8_t cmd_code)
{
  sie_cmd_code(SIE_CMDPHASE_COMMAND , cmd_code);
  sie_cmd_code(SIE_CMDPHASE_READ    , cmd_code);
  return (uint8_t) LPC_USB->CmdData;
}

//--------------------------------------------------------------------+
// PIPE HELPER
//--------------------------------------------------------------------+
static inline uint8_t ep_addr2idx(uint8_t ep_addr)
{
  return 2*(ep_addr & 0x0F) + ((ep_addr & TUSB_DIR_IN_MASK) ? 1 : 0);
}

static void set_ep_size(uint8_t ep_id, uint16_t max_packet_size)
{
  // follows example in 11.10.4.2
  LPC_USB->ReEp    |= TU_BIT(ep_id);
  LPC_USB->EpInd    = ep_id; // select index before setting packet size
  LPC_USB->MaxPSize = max_packet_size;

  while ((LPC_USB->DevIntSt & DEV_INT_ENDPOINT_REALIZED_MASK) == 0) {}
  LPC_USB->DevIntClr = DEV_INT_ENDPOINT_REALIZED_MASK;
}


//--------------------------------------------------------------------+
// CONTROLLER API
//--------------------------------------------------------------------+
static void bus_reset(void)
{
  // step 7 : slave mode set up
  LPC_USB->EpIntClr     = 0xFFFFFFFF; // clear all pending interrupt
  LPC_USB->DevIntClr    = 0xFFFFFFFF; // clear all pending interrupt
  LPC_USB->EpIntEn      = 0x03UL;     // control endpoint cannot use DMA, non-control all use DMA
  LPC_USB->EpIntPri     = 0x03UL;     // fast for control endpoint

  // step 8 : DMA set up
  LPC_USB->EpDMADis     = 0xFFFFFFFF; // firstly disable all dma
  LPC_USB->DMARClr      = 0xFFFFFFFF; // clear all pending interrupt
  LPC_USB->EoTIntClr    = 0xFFFFFFFF;
  LPC_USB->NDDRIntClr   = 0xFFFFFFFF;
  LPC_USB->SysErrIntClr = 0xFFFFFFFF;

  tu_memclr(&_dcd, sizeof(dcd_data_t));
}

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  //------------- user manual 11.13 usb device controller initialization -------------//
  // step 6 : set up control endpoint
  set_ep_size(0, CFG_TUD_ENDPOINT0_SIZE);
  set_ep_size(1, CFG_TUD_ENDPOINT0_SIZE);

  bus_reset();

  LPC_USB->DevIntEn = (DEV_INT_DEVICE_STATUS_MASK | DEV_INT_ENDPOINT_FAST_MASK | DEV_INT_ENDPOINT_SLOW_MASK | DEV_INT_ERROR_MASK);
  LPC_USB->UDCAH = (uint32_t) _dcd.udca;
  LPC_USB->DMAIntEn = (DMA_INT_END_OF_XFER_MASK /*| DMA_INT_NEW_DD_REQUEST_MASK*/ | DMA_INT_ERROR_MASK);

  dcd_connect(rhport);

  // Clear pending IRQ
  NVIC_ClearPendingIRQ(USB_IRQn);

  return true;
}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(USB_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(USB_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  // Response with status first before changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);

  sie_write(SIE_CMDCODE_SET_ADDRESS, 1, 0x80 | dev_addr); // 7th bit is : device_enable

  // Also Set Configure Device to enable non-control endpoint response
  sie_write(SIE_CMDCODE_CONFIGURE_DEVICE, 1, 1);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  sie_write(SIE_CMDCODE_DEVICE_STATUS, 1, SIE_DEV_STATUS_CONNECT_STATUS_MASK);
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  sie_write(SIE_CMDCODE_DEVICE_STATUS, 1, 0);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// CONTROL HELPER
//--------------------------------------------------------------------+
static inline uint8_t byte2dword(uint8_t bytes)
{
  return (bytes + 3) / 4; // length in dwords
}

static void control_ep_write(void const * buffer, uint8_t len)
{
  uint32_t const * buf32 = (uint32_t const *) buffer;

  LPC_USB->Ctrl   = USBCTRL_WRITE_ENABLE_MASK; // logical endpoint = 0
  LPC_USB->TxPLen = (uint32_t) len;

  for (uint8_t count = 0; count < byte2dword(len); count++)
  {
    LPC_USB->TxData = *buf32; // NOTE: cortex M3 have no problem with alignment
    buf32++;
  }

  LPC_USB->Ctrl = 0;

  // select control IN & validate the endpoint
  sie_write(SIE_CMDCODE_ENDPOINT_SELECT+1, 0, 0);
  sie_write(SIE_CMDCODE_BUFFER_VALIDATE  , 0, 0);
}

static uint8_t control_ep_read(void * buffer, uint8_t len)
{
  LPC_USB->Ctrl = USBCTRL_READ_ENABLE_MASK; // logical endpoint = 0
  while ((LPC_USB->RxPLen & USBRXPLEN_PACKET_READY_MASK) == 0) {} // TODO blocking, should have timeout

  len = tu_min8(len, (uint8_t) (LPC_USB->RxPLen & USBRXPLEN_PACKET_LENGTH_MASK) );
  uint32_t *buf32 = (uint32_t*) buffer;

  for (uint8_t count=0; count < byte2dword(len); count++)
  {
    *buf32 = LPC_USB->RxData;
    buf32++;
  }

  LPC_USB->Ctrl = 0;

  // select control OUT & clear the endpoint
  sie_write(SIE_CMDCODE_ENDPOINT_SELECT+0, 0, 0);
  sie_write(SIE_CMDCODE_BUFFER_CLEAR     , 0, 0);

  return len;
}

//--------------------------------------------------------------------+
// DCD Endpoint Port
//--------------------------------------------------------------------+

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t const ep_id = ep_addr2idx(p_endpoint_desc->bEndpointAddress);

  // Endpoint type is fixed to endpoint number
  // 1: interrupt, 2: Bulk, 3: Iso and so on
  switch ( p_endpoint_desc->bmAttributes.xfer )
  {
    case TUSB_XFER_INTERRUPT:
      TU_ASSERT((epnum % 3) == 1);
      break;

    case TUSB_XFER_BULK:
      TU_ASSERT((epnum % 3) == 2 || (epnum == 15));
      break;

    case TUSB_XFER_ISOCHRONOUS:
      TU_ASSERT((epnum % 3) == 0 && (epnum != 0) && (epnum != 15));
      break;

    default:
      break;
  }

  //------------- Realize Endpoint with Max Packet Size -------------//
  const uint16_t ep_size = tu_edpt_packet_size(p_endpoint_desc);
  set_ep_size(ep_id, ep_size);

  //------------- first DD prepare -------------//
  dma_desc_t* const dd = &_dcd.dd[ep_id];
  tu_memclr(dd, sizeof(dma_desc_t));

  dd->isochronous = (p_endpoint_desc->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) ? 1 : 0;
  dd->max_packet_size = ep_size;
  dd->retired = 1; // invalid at first

  sie_write(SIE_CMDCODE_ENDPOINT_SET_STATUS + ep_id, 1, 0);    // clear all endpoint status

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

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  if ( tu_edpt_number(ep_addr) == 0 )
  {
    sie_write(SIE_CMDCODE_ENDPOINT_SET_STATUS+0, 1, SIE_SET_ENDPOINT_STALLED_MASK | SIE_SET_ENDPOINT_CONDITION_STALLED_MASK);
  }else
  {
    uint8_t ep_id = ep_addr2idx( ep_addr );
    sie_write(SIE_CMDCODE_ENDPOINT_SET_STATUS+ep_id, 1, SIE_SET_ENDPOINT_STALLED_MASK);
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  uint8_t ep_id = ep_addr2idx(ep_addr);

  sie_write(SIE_CMDCODE_ENDPOINT_SET_STATUS+ep_id, 1, 0);
}

static bool control_xact(uint8_t rhport, uint8_t dir, uint8_t * buffer, uint8_t len)
{
  (void) rhport;

  if ( dir )
  {
    _dcd.control.in_bytes = len;
    control_ep_write(buffer, len);
  }else
  {
    if ( _dcd.control.out_received )
    {
      // Already received the DATA OUT packet
      _dcd.control.out_received = false;
      _dcd.control.out_buffer = NULL;
      _dcd.control.out_bytes  = 0;

      uint8_t received = control_ep_read(buffer, len);
      dcd_event_xfer_complete(0, 0, received, XFER_RESULT_SUCCESS, true);
    }else
    {
      _dcd.control.out_buffer = buffer;
      _dcd.control.out_bytes  = len;
    }
  }

  return true;
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
  // Control transfer is not DMA support, and must be done in slave mode
  if ( tu_edpt_number(ep_addr) == 0 )
  {
    return control_xact(rhport, tu_edpt_dir(ep_addr), buffer, (uint8_t) total_bytes);
  }
  else
  {
    uint8_t ep_id = ep_addr2idx(ep_addr);
    dma_desc_t* dd = &_dcd.dd[ep_id];

    // Prepare DMA descriptor
    // Isochronous & max packet size must be preserved, Other fields of dd should be clear
    uint16_t const ep_size = dd->max_packet_size;
    uint8_t  is_iso = dd->isochronous;

    tu_memclr(dd, sizeof(dma_desc_t));
    dd->isochronous = is_iso;
    dd->max_packet_size = ep_size;
    dd->buffer = (uint32_t) buffer;
    dd->buflen = total_bytes;

    _dcd.udca[ep_id] = dd;

    if ( ep_id % 2 )
    {
      // Clear EP interrupt before Enable DMA
      LPC_USB->EpIntEn &= ~TU_BIT(ep_id);
      LPC_USB->EpDMAEn = TU_BIT(ep_id);

      // endpoint IN need to actively raise DMA request
      LPC_USB->DMARSet = TU_BIT(ep_id);
    }else
    {
      // Enable DMA
      LPC_USB->EpDMAEn = TU_BIT(ep_id);
    }

    return true;
  }
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

// handle control xfer (slave mode)
static void control_xfer_isr(uint8_t rhport, uint32_t ep_int_status)
{
  // Control out complete
  if ( ep_int_status & TU_BIT(0) )
  {
    bool is_setup = sie_read(SIE_CMDCODE_ENDPOINT_SELECT+0) & SIE_SELECT_ENDPOINT_SETUP_RECEIVED_MASK;

    LPC_USB->EpIntClr = TU_BIT(0);

    if (is_setup)
    {
      uint8_t setup_packet[8];
      control_ep_read(setup_packet, 8); // TODO read before clear setup above

      dcd_event_setup_received(rhport, setup_packet, true);
    }
    else if ( _dcd.control.out_buffer )
    {
      // software queued transfer previously
      uint8_t received = control_ep_read(_dcd.control.out_buffer, _dcd.control.out_bytes);

      _dcd.control.out_buffer = NULL;
      _dcd.control.out_bytes = 0;

      dcd_event_xfer_complete(rhport, 0, received, XFER_RESULT_SUCCESS, true);
    }else
    {
      // hardware auto ack packet -> mark as received
      _dcd.control.out_received = true;
    }
  }

  // Control In complete
  if ( ep_int_status & TU_BIT(1) )
  {
    LPC_USB->EpIntClr = TU_BIT(1);
    dcd_event_xfer_complete(rhport, TUSB_DIR_IN_MASK, _dcd.control.in_bytes, XFER_RESULT_SUCCESS, true);
  }
}

// handle bus event signal
static void bus_event_isr(uint8_t rhport)
{
  uint8_t const dev_status = sie_read(SIE_CMDCODE_DEVICE_STATUS);
  if (dev_status & SIE_DEV_STATUS_RESET_MASK)
  {
    bus_reset();
    dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
  }

  if (dev_status & SIE_DEV_STATUS_CONNECT_CHANGE_MASK)
  {
    // device is disconnected, require using VBUS (P1_30)
    dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
  }

  if (dev_status & SIE_DEV_STATUS_SUSPEND_CHANGE_MASK)
  {
    if (dev_status & SIE_DEV_STATUS_SUSPEND_MASK)
    {
      dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
    }
    else
    {
      dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
    }
  }
}

// Helper to complete a DMA descriptor for non-control transfer
static void dd_complete_isr(uint8_t rhport, uint8_t ep_id)
{
  dma_desc_t* const dd = &_dcd.dd[ep_id];
  uint8_t result = (dd->status == DD_STATUS_NORMAL || dd->status == DD_STATUS_DATA_UNDERUN) ? XFER_RESULT_SUCCESS : XFER_RESULT_FAILED;
  uint8_t const ep_addr = (ep_id / 2) | ((ep_id & 0x01) ? TUSB_DIR_IN_MASK : 0);

  dcd_event_xfer_complete(rhport, ep_addr, dd->present_count, result, true);
}

// main USB IRQ handler
void dcd_int_handler(uint8_t rhport)
{
  uint32_t const dev_int_status = LPC_USB->DevIntSt & LPC_USB->DevIntEn;
  LPC_USB->DevIntClr = dev_int_status;// Acknowledge handled interrupt

  // Bus event
  if (dev_int_status & DEV_INT_DEVICE_STATUS_MASK)
  {
    bus_event_isr(rhport);
  }

  // Endpoint interrupt
  uint32_t const ep_int_status = LPC_USB->EpIntSt & LPC_USB->EpIntEn;

  // Control Endpoint are fast
  if (dev_int_status & DEV_INT_ENDPOINT_FAST_MASK)
  {
    // Note clear USBEpIntClr will also clear the setup received bit --> clear after handle setup packet
    // Only clear USBEpIntClr 1 endpoint each, and should wait for CDFULL bit set
    control_xfer_isr(rhport, ep_int_status);
  }

  // non-control IN are slow
  if (dev_int_status & DEV_INT_ENDPOINT_SLOW_MASK)
  {
    for ( uint8_t ep_id = 3; ep_id < DCD_ENDPOINT_MAX; ep_id += 2 )
    {
      if ( tu_bit_test(ep_int_status, ep_id) )
      {
        LPC_USB->EpIntClr = TU_BIT(ep_id);

        // Clear Ep interrupt for next DMA
        LPC_USB->EpIntEn &= ~TU_BIT(ep_id);

        dd_complete_isr(rhport, ep_id);
      }
    }
  }

  // DMA transfer complete (RAM <-> EP) for Non-Control
  // OUT: USB transfer is fully complete
  // IN : UBS transfer is still on-going -> enable EpIntEn to know when it is complete
  uint32_t const dma_int_status = LPC_USB->DMAIntSt & LPC_USB->DMAIntEn;
  if (dma_int_status & DMA_INT_END_OF_XFER_MASK)
  {
    uint32_t const eot = LPC_USB->EoTIntSt;
    LPC_USB->EoTIntClr = eot; // acknowledge interrupt source

    for ( uint8_t ep_id = 2; ep_id < DCD_ENDPOINT_MAX; ep_id++ )
    {
      if ( tu_bit_test(eot, ep_id) )
      {
        if ( ep_id & 0x01 )
        {
          // IN enable EpInt for end of usb transfer
          LPC_USB->EpIntEn |= TU_BIT(ep_id);
        }else
        {
          // OUT
          dd_complete_isr(rhport, ep_id);
        }
      }
    }
  }

  // Errors
  if ( (dev_int_status & DEV_INT_ERROR_MASK) || (dma_int_status & DMA_INT_ERROR_MASK) )
  {
    uint32_t error_status = sie_read(SIE_CMDCODE_READ_ERROR_STATUS);
    (void) error_status;
    TU_BREAKPOINT();
  }
}

#endif
