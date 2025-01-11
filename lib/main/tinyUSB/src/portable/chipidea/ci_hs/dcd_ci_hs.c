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

#if CFG_TUD_ENABLED && defined(TUP_USBIP_CHIPIDEA_HS)

#include "device/dcd.h"
#include "ci_hs_type.h"

#if CFG_TUSB_MCU == OPT_MCU_MIMXRT1XXX
  #include "ci_hs_imxrt.h"

#if CFG_TUD_MEM_DCACHE_ENABLE
bool dcd_dcache_clean(void const* addr, uint32_t data_size) {
  return imxrt_dcache_clean(addr, data_size);
}

bool dcd_dcache_invalidate(void const* addr, uint32_t data_size) {
  return imxrt_dcache_invalidate(addr, data_size);
}

bool dcd_dcache_clean_invalidate(void const* addr, uint32_t data_size) {
  return imxrt_dcache_clean_invalidate(addr, data_size);
}
#endif

#elif TU_CHECK_MCU(OPT_MCU_LPC18XX, OPT_MCU_LPC43XX)
  #include "ci_hs_lpc18_43.h"

#elif TU_CHECK_MCU(OPT_MCU_MCXN9)
  // MCX N9 only port 1 use this controller
  #include "ci_hs_mcx.h"

#else
  #error "Unsupported MCUs"
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// ENDPTCTRL
enum {
  ENDPTCTRL_STALL          = TU_BIT(0),
  ENDPTCTRL_TOGGLE_INHIBIT = TU_BIT(5), // used for test only
  ENDPTCTRL_TOGGLE_RESET   = TU_BIT(6),
  ENDPTCTRL_ENABLE         = TU_BIT(7)
};

enum {
  ENDPTCTRL_TYPE_POS  = 2, // Endpoint type is 2-bit field
};

// USBSTS, USBINTR
enum {
  INTR_USB         = TU_BIT(0),
  INTR_ERROR       = TU_BIT(1),
  INTR_PORT_CHANGE = TU_BIT(2),
  INTR_RESET       = TU_BIT(6),
  INTR_SOF         = TU_BIT(7),
  INTR_SUSPEND     = TU_BIT(8),
  INTR_NAK         = TU_BIT(16)
};

// Queue Transfer Descriptor
typedef struct
{
  // Word 0: Next QTD Pointer
  uint32_t next; ///< Next link pointer This field contains the physical memory address of the next dTD to be processed

  // Word 1: qTQ Token
  uint32_t                      : 3  ;
  volatile uint32_t xact_err    : 1  ;
  uint32_t                      : 1  ;
  volatile uint32_t buffer_err  : 1  ;
  volatile uint32_t halted      : 1  ;
  volatile uint32_t active      : 1  ;
  uint32_t                      : 2  ;
  uint32_t iso_mult_override    : 2  ; ///< This field can be used for transmit ISOs to override the MULT field in the dQH. This field must be zero for all packet types that are not transmit-ISO.
  uint32_t                      : 3  ;
  uint32_t int_on_complete      : 1  ;
  volatile uint32_t total_bytes : 15 ;
  uint32_t                      : 1  ;

  // Word 2-6: Buffer Page Pointer List, Each element in the list is a 4K page aligned, physical memory address. The lower 12 bits in each pointer are reserved (except for the first one) as each memory pointer must reference the start of a 4K page
  uint32_t buffer[5]; ///< buffer1 has frame_n for TODO Isochronous

  //--------------------------------------------------------------------+
  // TD is 32 bytes aligned but occupies only 28 bytes
  // Therefore there are 4 bytes padding that we can use.
  //--------------------------------------------------------------------+
  uint16_t expected_bytes;
  uint8_t reserved[2];
} dcd_qtd_t;

TU_VERIFY_STATIC( sizeof(dcd_qtd_t) == 32, "size is not correct");

// Queue Head
typedef struct
{
  // Word 0: Capabilities and Characteristics
  uint32_t                         : 15 ; ///< Number of packets executed per transaction descriptor 00 - Execute N transactions as demonstrated by the USB variable length protocol where N is computed using Max_packet_length and the Total_bytes field in the dTD. 01 - Execute one transaction 10 - Execute two transactions 11 - Execute three transactions Remark: Non-isochronous endpoints must set MULT = 00. Remark: Isochronous endpoints must set MULT = 01, 10, or 11 as needed.
  uint32_t int_on_setup            : 1  ; ///< Interrupt on setup This bit is used on control type endpoints to indicate if USBINT is set in response to a setup being received.
  uint32_t max_packet_size         : 11 ; ///< Endpoint's wMaxPacketSize
  uint32_t                         : 2  ;
  uint32_t zero_length_termination : 1  ; ///< This bit is used for non-isochronous endpoints to indicate when a zero-length packet is received to terminate transfers in case the total transfer length is “multiple”. 0 - Enable zero-length packet to terminate transfers equal to a multiple of Max_packet_length (default). 1 - Disable zero-length packet on transfers that are equal in length to a multiple Max_packet_length.
  uint32_t iso_mult                : 2  ; ///<

  // Word 1: Current qTD Pointer
  volatile uint32_t qtd_addr;

  // Word 2-9: Transfer Overlay
  volatile dcd_qtd_t qtd_overlay;

  // Word 10-11: Setup request (control OUT only)
  volatile tusb_control_request_t setup_request;

  //--------------------------------------------------------------------+
  // QHD is 64 bytes aligned but occupies only 48 bytes
  // Therefore there are 16 bytes padding that we can use.
  //--------------------------------------------------------------------+
  tu_fifo_t * ff;
  uint8_t reserved[12];
} dcd_qhd_t;

TU_VERIFY_STATIC( sizeof(dcd_qhd_t) == 64, "size is not correct");

//--------------------------------------------------------------------+
// Variables
//--------------------------------------------------------------------+

#define QTD_NEXT_INVALID 0x01

typedef struct {
  // Must be at 2K alignment
  // Each endpoint with direction (IN/OUT) occupies a queue head
  // for portability, TinyUSB only queue 1 TD for each Qhd
  dcd_qhd_t qhd[TUP_DCD_ENDPOINT_MAX][2] TU_ATTR_ALIGNED(64);
  dcd_qtd_t qtd[TUP_DCD_ENDPOINT_MAX][2] TU_ATTR_ALIGNED(32);
}dcd_data_t;

CFG_TUD_MEM_SECTION TU_ATTR_ALIGNED(2048)
static dcd_data_t _dcd_data;

//--------------------------------------------------------------------+
// Prototypes and Helper Functions
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE
static inline uint8_t ci_ep_count(ci_hs_regs_t const* dcd_reg)
{
  return dcd_reg->DCCPARAMS & DCCPARAMS_DEN_MASK;
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

/// follows LPC43xx User Manual 23.10.3
static void bus_reset(uint8_t rhport)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);

  // The reset value for all endpoint types is the control endpoint. If one endpoint
  // direction is enabled and the paired endpoint of opposite direction is disabled, then the
  // endpoint type of the unused direction must be changed from the control type to any other
  // type (e.g. bulk). Leaving an un-configured endpoint control will cause undefined behavior
  // for the data PID tracking on the active endpoint.
  uint8_t const ep_count = ci_ep_count(dcd_reg);
  for( uint8_t i=1; i < ep_count; i++)
  {
    dcd_reg->ENDPTCTRL[i] = (TUSB_XFER_BULK << ENDPTCTRL_TYPE_POS) | (TUSB_XFER_BULK << (16+ENDPTCTRL_TYPE_POS));
  }

  //------------- Clear All Registers -------------//
  dcd_reg->ENDPTNAK       = dcd_reg->ENDPTNAK;
  dcd_reg->ENDPTNAKEN     = 0;
  dcd_reg->USBSTS         = dcd_reg->USBSTS;
  dcd_reg->ENDPTSETUPSTAT = dcd_reg->ENDPTSETUPSTAT;
  dcd_reg->ENDPTCOMPLETE  = dcd_reg->ENDPTCOMPLETE;

  while (dcd_reg->ENDPTPRIME) {}
  dcd_reg->ENDPTFLUSH = 0xFFFFFFFF;
  while (dcd_reg->ENDPTFLUSH) {}

  // read reset bit in portsc

  //------------- Queue Head & Queue TD -------------//
  tu_memclr(&_dcd_data, sizeof(dcd_data_t));

  //------------- Set up Control Endpoints (0 OUT, 1 IN) -------------//
  _dcd_data.qhd[0][0].zero_length_termination = _dcd_data.qhd[0][1].zero_length_termination = 1;
  _dcd_data.qhd[0][0].max_packet_size  = _dcd_data.qhd[0][1].max_packet_size  = CFG_TUD_ENDPOINT0_SIZE;
  _dcd_data.qhd[0][0].qtd_overlay.next = _dcd_data.qhd[0][1].qtd_overlay.next = QTD_NEXT_INVALID;

  _dcd_data.qhd[0][0].int_on_setup = 1; // OUT only

  dcd_dcache_clean_invalidate(&_dcd_data, sizeof(dcd_data_t));
}

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  tu_memclr(&_dcd_data, sizeof(dcd_data_t));

  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);

  TU_ASSERT(ci_ep_count(dcd_reg) <= TUP_DCD_ENDPOINT_MAX);

  // Reset controller
  dcd_reg->USBCMD |= USBCMD_RESET;
  while( dcd_reg->USBCMD & USBCMD_RESET ) {}

  // Set mode to device, must be set immediately after reset
  uint32_t usbmode = dcd_reg->USBMODE & ~USBMOD_CM_MASK;
  usbmode |= USBMODE_CM_DEVICE;
  dcd_reg->USBMODE = usbmode;

  dcd_reg->OTGSC = OTGSC_VBUS_DISCHARGE | OTGSC_OTG_TERMINATION;

#if !TUD_OPT_HIGH_SPEED
  dcd_reg->PORTSC1 = PORTSC1_FORCE_FULL_SPEED;
#endif

  dcd_dcache_clean_invalidate(&_dcd_data, sizeof(dcd_data_t));

  dcd_reg->ENDPTLISTADDR = (uint32_t) _dcd_data.qhd; // Endpoint List Address has to be 2K alignment
  dcd_reg->USBSTS  = dcd_reg->USBSTS;
  dcd_reg->USBINTR = INTR_USB | INTR_ERROR | INTR_PORT_CHANGE | INTR_SUSPEND;

  uint32_t usbcmd = dcd_reg->USBCMD;
  usbcmd &= ~USBCMD_INTR_THRESHOLD_MASK; // Interrupt Threshold Interval = 0
  usbcmd |= USBCMD_RUN_STOP; // run

  dcd_reg->USBCMD = usbcmd;

  return true;
}

void dcd_int_enable(uint8_t rhport)
{
  CI_DCD_INT_ENABLE(rhport);
}

void dcd_int_disable(uint8_t rhport)
{
  CI_DCD_INT_DISABLE(rhport);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  // Response with status first before changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);

  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_reg->DEVICEADDR = (dev_addr << 25) | TU_BIT(24);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_reg->PORTSC1 |= PORTSC1_FORCE_PORT_RESUME;
}

void dcd_connect(uint8_t rhport)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_reg->USBCMD |= USBCMD_RUN_STOP;
}

void dcd_disconnect(uint8_t rhport)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_reg->USBCMD &= ~USBCMD_RUN_STOP;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  if (en) {
      dcd_reg->USBINTR |= INTR_SOF;
  } else {
      dcd_reg->USBINTR &= ~INTR_SOF;
  }
}

//--------------------------------------------------------------------+
// HELPER
//--------------------------------------------------------------------+

static void qtd_init(dcd_qtd_t* p_qtd, void * data_ptr, uint16_t total_bytes)
{
  dcd_dcache_clean_invalidate((uint32_t*) tu_align((uint32_t) data_ptr, 4), total_bytes);

  tu_memclr(p_qtd, sizeof(dcd_qtd_t));

  p_qtd->next            = QTD_NEXT_INVALID;
  p_qtd->active          = 1;
  p_qtd->total_bytes     = p_qtd->expected_bytes = total_bytes;
  p_qtd->int_on_complete = true;

  if (data_ptr != NULL)
  {
    p_qtd->buffer[0] = (uint32_t) data_ptr;

    uint32_t const bufend = p_qtd->buffer[0] + total_bytes;
    for(uint8_t i=1; i<5; i++)
    {
      uint32_t const next_page = tu_align4k( p_qtd->buffer[i-1] ) + 4096;
      if ( bufend <= next_page ) break;

      p_qtd->buffer[i] = next_page;

      // TODO page[1] FRAME_N for ISO transfer
    }
  }
}

//--------------------------------------------------------------------+
// DCD Endpoint Port
//--------------------------------------------------------------------+
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum  = tu_edpt_number(ep_addr);
  uint8_t const dir    = tu_edpt_dir(ep_addr);

  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_reg->ENDPTCTRL[epnum] |= ENDPTCTRL_STALL << (dir ? 16 : 0);

  // flush to abort any primed buffer
  dcd_reg->ENDPTFLUSH = TU_BIT(epnum + (dir ? 16 : 0));
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  // data toggle also need to be reset
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_reg->ENDPTCTRL[epnum] |= ENDPTCTRL_TOGGLE_RESET << ( dir ? 16 : 0 );
  dcd_reg->ENDPTCTRL[epnum] &= ~(ENDPTCTRL_STALL << ( dir  ? 16 : 0));
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  uint8_t const epnum = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);

  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);

  // Must not exceed max endpoint number
  TU_ASSERT(epnum < ci_ep_count(dcd_reg));

  //------------- Prepare Queue Head -------------//
  dcd_qhd_t * p_qhd = &_dcd_data.qhd[epnum][dir];
  tu_memclr(p_qhd, sizeof(dcd_qhd_t));

  p_qhd->zero_length_termination = 1;
  p_qhd->max_packet_size         = tu_edpt_packet_size(p_endpoint_desc);
  if (p_endpoint_desc->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS)
  {
    p_qhd->iso_mult = 1;
  }

  p_qhd->qtd_overlay.next        = QTD_NEXT_INVALID;

  dcd_dcache_clean_invalidate(&_dcd_data, sizeof(dcd_data_t));

  // Enable EP Control
  uint32_t const epctrl = (p_endpoint_desc->bmAttributes.xfer << ENDPTCTRL_TYPE_POS) | ENDPTCTRL_ENABLE | ENDPTCTRL_TOGGLE_RESET;

  if ( dir == TUSB_DIR_OUT )
  {
    dcd_reg->ENDPTCTRL[epnum] = (dcd_reg->ENDPTCTRL[epnum] & 0xFFFF0000u) | epctrl;
  }else
  {
    dcd_reg->ENDPTCTRL[epnum] = (dcd_reg->ENDPTCTRL[epnum] & 0x0000FFFFu) | (epctrl << 16);
  }

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);

  // Disable all non-control endpoints
  uint8_t const ep_count = ci_ep_count(dcd_reg);
  for (uint8_t epnum = 1; epnum < ep_count; epnum++)
  {
    _dcd_data.qhd[epnum][TUSB_DIR_OUT].qtd_overlay.halted = 1;
    _dcd_data.qhd[epnum][TUSB_DIR_IN ].qtd_overlay.halted = 1;

    dcd_reg->ENDPTFLUSH = TU_BIT(epnum) |  TU_BIT(epnum+16);
    dcd_reg->ENDPTCTRL[epnum] = (TUSB_XFER_BULK << ENDPTCTRL_TYPE_POS) | (TUSB_XFER_BULK << (16+ENDPTCTRL_TYPE_POS));
  }
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum  = tu_edpt_number(ep_addr);
  uint8_t const dir    = tu_edpt_dir(ep_addr);

  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);

  _dcd_data.qhd[epnum][dir].qtd_overlay.halted = 1;

  // Flush EP
  uint32_t const flush_mask = TU_BIT(epnum + (dir ? 16 : 0));
  dcd_reg->ENDPTFLUSH = flush_mask;
  while(dcd_reg->ENDPTFLUSH & flush_mask);

  // Clear EP enable
  dcd_reg->ENDPTCTRL[epnum] &=~(ENDPTCTRL_ENABLE << (dir ? 16 : 0));
}

static void qhd_start_xfer(uint8_t rhport, uint8_t epnum, uint8_t dir)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
  dcd_qhd_t* p_qhd = &_dcd_data.qhd[epnum][dir];
  dcd_qtd_t* p_qtd = &_dcd_data.qtd[epnum][dir];

  p_qhd->qtd_overlay.halted = false;            // clear any previous error
  p_qhd->qtd_overlay.next   = (uint32_t) p_qtd; // link qtd to qhd

  // flush cache
  dcd_dcache_clean_invalidate(&_dcd_data, sizeof(dcd_data_t));

  if ( epnum == 0 )
  {
    // follows UM 24.10.8.1.1 Setup packet handling using setup lockout mechanism
    // wait until ENDPTSETUPSTAT before priming data/status in response TODO add time out
    while(dcd_reg->ENDPTSETUPSTAT & TU_BIT(0)) {}
  }

  // start transfer
  dcd_reg->ENDPTPRIME = TU_BIT(epnum + (dir ? 16 : 0));
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  dcd_qhd_t* p_qhd = &_dcd_data.qhd[epnum][dir];
  dcd_qtd_t* p_qtd = &_dcd_data.qtd[epnum][dir];

  // Prepare qtd
  qtd_init(p_qtd, buffer, total_bytes);

  // Start qhd transfer
  p_qhd->ff = NULL;
  qhd_start_xfer(rhport, epnum, dir);

  return true;
}

#if !CFG_TUD_MEM_DCACHE_ENABLE
// fifo has to be aligned to 4k boundary
// It's incompatible with dcache enabled transfer, since neither address nor size is aligned to cache line
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  dcd_qhd_t * p_qhd = &_dcd_data.qhd[epnum][dir];
  dcd_qtd_t * p_qtd = &_dcd_data.qtd[epnum][dir];

  tu_fifo_buffer_info_t fifo_info;

  if (dir)
  {
    tu_fifo_get_read_info(ff, &fifo_info);
  } else
  {
    tu_fifo_get_write_info(ff, &fifo_info);
  }

  if ( fifo_info.len_lin >= total_bytes )
  {
    // Linear length is enough for this transfer
    qtd_init(p_qtd, fifo_info.ptr_lin, total_bytes);
  }
  else
  {
    // linear part is not enough

    // prepare TD up to linear length
    qtd_init(p_qtd, fifo_info.ptr_lin, fifo_info.len_lin);

    if ( !tu_offset4k((uint32_t) fifo_info.ptr_wrap) && !tu_offset4k(tu_fifo_depth(ff)) )
    {
      // If buffer is aligned to 4K & buffer size is multiple of 4K
      // We can make use of buffer page array to also combine the linear + wrapped length
      p_qtd->total_bytes = p_qtd->expected_bytes = total_bytes;

      for(uint8_t i = 1, page = 0; i < 5; i++)
      {
        // pick up buffer array where linear ends
        if (p_qtd->buffer[i] == 0)
        {
          p_qtd->buffer[i] = (uint32_t) fifo_info.ptr_wrap + 4096 * page;
          page++;
        }
      }
    }
    else
    {
      // TODO we may need to carry the wrapped length after the linear part complete
      // for now only transfer up to linear part
    }
  }

  // Start qhd transfer
  p_qhd->ff = ff;
  qhd_start_xfer(rhport, epnum, dir);

  return true;
}
#endif

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

static void process_edpt_complete_isr(uint8_t rhport, uint8_t epnum, uint8_t dir)
{
  dcd_qhd_t * p_qhd = &_dcd_data.qhd[epnum][dir];
  dcd_qtd_t * p_qtd = &_dcd_data.qtd[epnum][dir];

  uint8_t result = p_qtd->halted ? XFER_RESULT_STALLED :
      ( p_qtd->xact_err || p_qtd->buffer_err ) ? XFER_RESULT_FAILED : XFER_RESULT_SUCCESS;

  if ( result != XFER_RESULT_SUCCESS )
  {
    ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);
    // flush to abort error buffer
    dcd_reg->ENDPTFLUSH = TU_BIT(epnum + (dir ? 16 : 0));
  }

  uint16_t const xferred_bytes = p_qtd->expected_bytes - p_qtd->total_bytes;

  if (p_qhd->ff)
  {
    if (dir == TUSB_DIR_IN)
    {
      tu_fifo_advance_read_pointer(p_qhd->ff, xferred_bytes);
    } else
    {
      tu_fifo_advance_write_pointer(p_qhd->ff, xferred_bytes);
    }
  }

  // only number of bytes in the IOC qtd
  dcd_event_xfer_complete(rhport, tu_edpt_addr(epnum, dir), xferred_bytes, result, true);
}

void dcd_int_handler(uint8_t rhport)
{
  ci_hs_regs_t* dcd_reg = CI_HS_REG(rhport);

  uint32_t const int_enable = dcd_reg->USBINTR;
  uint32_t const int_status = dcd_reg->USBSTS & int_enable;
  dcd_reg->USBSTS = int_status; // Acknowledge handled interrupt

  // disabled interrupt sources
  if (int_status == 0) return;

  // Set if the port controller enters the full or high-speed operational state.
  // either from Bus Reset or Suspended state
	if (int_status & INTR_PORT_CHANGE)
	{
	  // TU_LOG2("PortChange %08lx\r\n", dcd_reg->PORTSC1);

	  // Reset interrupt is not enabled, we manually check if Port Change is due
	  // to connection / disconnection
	  if ( dcd_reg->USBSTS & INTR_RESET )
	  {
	    dcd_reg->USBSTS = INTR_RESET;

	    if (dcd_reg->PORTSC1 & PORTSC1_CURRENT_CONNECT_STATUS)
	    {
	      uint32_t const speed = (dcd_reg->PORTSC1 & PORTSC1_PORT_SPEED) >> PORTSC1_PORT_SPEED_POS;
	      bus_reset(rhport);
	      dcd_event_bus_reset(rhport, (tusb_speed_t) speed, true);
	    }else
	    {
	      dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
	    }
	  }
	  else
	  {
	    // Triggered by resuming from suspended state
	    if ( !(dcd_reg->PORTSC1 & PORTSC1_SUSPEND) )
	    {
	      dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
	    }
	  }
	}

  if (int_status & INTR_SUSPEND)
  {
    // TU_LOG2("Suspend %08lx\r\n", dcd_reg->PORTSC1);

    if (dcd_reg->PORTSC1 & PORTSC1_SUSPEND)
    {
      // Note: Host may delay more than 3 ms before and/or after bus reset before doing enumeration.
      // Skip suspend event if we are not addressed
      if ((dcd_reg->DEVICEADDR >> 25) & 0x0f)
      {
        dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
      }
    }
  }

  if (int_status & INTR_USB)
  {
    // Make sure we read the latest version of _dcd_data.
    dcd_dcache_clean_invalidate(&_dcd_data, sizeof(dcd_data_t));

    uint32_t const edpt_complete = dcd_reg->ENDPTCOMPLETE;
    dcd_reg->ENDPTCOMPLETE = edpt_complete; // acknowledge

    // 23.10.12.3 Failed QTD also get ENDPTCOMPLETE set
    // nothing to do, we will submit xfer as error to usbd
    // if (int_status & INTR_ERROR) { }

    if ( edpt_complete )
    {
      for(uint8_t epnum = 0; epnum < TUP_DCD_ENDPOINT_MAX; epnum++)
      {
        if ( tu_bit_test(edpt_complete, epnum)    ) process_edpt_complete_isr(rhport, epnum, TUSB_DIR_OUT);
        if ( tu_bit_test(edpt_complete, epnum+16) ) process_edpt_complete_isr(rhport, epnum, TUSB_DIR_IN);
      }
    }

    // Set up Received
    // 23.10.10.2 Operational model for setup transfers
    // Must be after normal transfer complete since it is possible to have both previous control status + new setup
    // in the same frame and we should handle previous status first.
    if (dcd_reg->ENDPTSETUPSTAT) {
      dcd_reg->ENDPTSETUPSTAT = dcd_reg->ENDPTSETUPSTAT;
      dcd_event_setup_received(rhport, (uint8_t *) (uintptr_t) &_dcd_data.qhd[0][0].setup_request, true);
    }
  }

  if (int_status & INTR_SOF)
  {
    const uint32_t frame = dcd_reg->FRINDEX;
    dcd_event_sof(rhport, frame, true);
  }
}

#endif
