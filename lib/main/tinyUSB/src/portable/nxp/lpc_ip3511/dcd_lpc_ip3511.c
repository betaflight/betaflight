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

/* Since 2012 starting with LPC11uxx, NXP start to use common USB Device Controller with code name LPC IP3511
 * for almost their new MCUs. Currently supported and tested families are
 * - LPC11U68, LPC11U37
 * - LPC1347
 * - LPC51U68
 * - LPC54114
 * - LPC55s69
 */
#if CFG_TUD_ENABLED && defined(TUP_USBIP_IP3511)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+

#if TU_CHECK_MCU(OPT_MCU_LPC11UXX, OPT_MCU_LPC13XX, OPT_MCU_LPC15XX)
  // LPCOpen
  #ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-parameter"
  #pragma GCC diagnostic ignored "-Wstrict-prototypes"
  #endif

  #include "chip.h"

  #ifdef __GNUC__
  #pragma GCC diagnostic pop
  #endif

#else
  // SDK
  #include "fsl_device_registers.h"
  #define INCLUDE_FSL_DEVICE_REGISTERS
#endif

#include "device/dcd.h"

//--------------------------------------------------------------------+
// IP3511 Registers
//--------------------------------------------------------------------+

typedef struct {
  __IO uint32_t DEVCMDSTAT;    // Device Command/Status register, offset: 0x0
  __I  uint32_t INFO;          // Info register, offset: 0x4
  __IO uint32_t EPLISTSTART;   // EP Command/Status List start address, offset: 0x8
  __IO uint32_t DATABUFSTART;  // Data buffer start address, offset: 0xC
  __IO uint32_t LPM;           // Link Power Management register, offset: 0x10
  __IO uint32_t EPSKIP;        // Endpoint skip, offset: 0x14
  __IO uint32_t EPINUSE;       // Endpoint Buffer in use, offset: 0x18
  __IO uint32_t EPBUFCFG;      // Endpoint Buffer Configuration register, offset: 0x1C
  __IO uint32_t INTSTAT;       // interrupt status register, offset: 0x20
  __IO uint32_t INTEN;         // interrupt enable register, offset: 0x24
  __IO uint32_t INTSETSTAT;    // set interrupt status register, offset: 0x28
       uint8_t RESERVED_0[8];
  __I  uint32_t EPTOGGLE;      // Endpoint toggle register, offset: 0x34
} dcd_registers_t;

// Max nbytes for each control/bulk/interrupt transfer
enum {
  NBYTES_ISO_FS_MAX = 1023, // FS ISO
  NBYTES_ISO_HS_MAX = 1024, // HS ISO
  NBYTES_CBI_FS_MAX = 64,   // FS control/bulk/interrupt. TODO some FS can do burst with higher size e.g 1024. Need to test
  NBYTES_CBI_HS_MAX = 32767 // can be up to all 15-bit, but only tested with 4096
};

enum {
  INT_SOF_MASK           = TU_BIT(30),
  INT_DEVICE_STATUS_MASK = TU_BIT(31)
};

enum {
  DEVCMDSTAT_DEVICE_ADDR_MASK    = TU_BIT(7 )-1,
  DEVCMDSTAT_DEVICE_ENABLE_MASK  = TU_BIT(7 ),
  DEVCMDSTAT_SETUP_RECEIVED_MASK = TU_BIT(8 ),
  DEVCMDSTAT_DEVICE_CONNECT_MASK = TU_BIT(16), // reflect the soft-connect only, does not reflect the actual attached state
  DEVCMDSTAT_DEVICE_SUSPEND_MASK = TU_BIT(17),
                                               // 23-22 is link speed (only available for HighSpeed port)
  DEVCMDSTAT_CONNECT_CHANGE_MASK = TU_BIT(24),
  DEVCMDSTAT_SUSPEND_CHANGE_MASK = TU_BIT(25),
  DEVCMDSTAT_RESET_CHANGE_MASK   = TU_BIT(26),
  DEVCMDSTAT_VBUS_DEBOUNCED_MASK = TU_BIT(28),
};

enum {
  DEVCMDSTAT_SPEED_SHIFT = 22
};

//--------------------------------------------------------------------+
// Endpoint Command/Status List
//--------------------------------------------------------------------+

// EP Command/Status field definition
enum {
  EPCS_TYPE         = TU_BIT(26),
  EPCS_RF_TV        = TU_BIT(27),
  EPCS_TOGGLE_RESET = TU_BIT(28),
  EPCS_STALL        = TU_BIT(29),
  EPCS_DISABLED     = TU_BIT(30),
  EPCS_ACTIVE       = TU_BIT(31),
};

// Endpoint Command/Status
typedef union TU_ATTR_PACKED
{
  // Full and High speed has different bit layout for buffer_offset and nbytes
  // TODO FS/HS layout depends on the max speed of controller e.g
  // lpc55s69 PORT0 is only FS but actually has the same layout as HS on port1

  // Buffer (aligned 64) = DATABUFSTART [31:22]  | buffer_offset [21:6]
  volatile struct {
    uint32_t offset      : 16;
    uint32_t nbytes      : 10;
    uint32_t TU_RESERVED : 6;
  } buffer_fs;

  // Buffer (aligned 64) = USB_RAM [31:17] | buffer_offset [16:6]
  volatile struct {
    uint32_t offset      : 11 ;
    uint32_t nbytes      : 15 ;
    uint32_t TU_RESERVED : 6  ;
  } buffer_hs;

  volatile struct {
    uint32_t TU_RESERVED  : 26;
    uint32_t type         : 1 ;
    uint32_t rf_tv        : 1 ; // rate feedback or toggle value
    uint32_t toggle_reset : 1 ;
    uint32_t stall        : 1 ;
    uint32_t disable      : 1 ;
    uint32_t active       : 1 ;
  } cmd_sts;
}ep_cmd_sts_t;

TU_VERIFY_STATIC( sizeof(ep_cmd_sts_t) == 4, "size is not correct" );

// Software transfer management
typedef struct
{
  uint16_t total_bytes;
  uint16_t xferred_bytes;

  uint16_t nbytes;

  // prevent unaligned access on Highspeed port on USB_SRAM
  uint16_t TU_RESERVED;
}xfer_dma_t;

// Absolute max of endpoints pairs for all port
// - 11 13 15 51 54 has 5x2 endpoints
// - 55 usb0 (FS) has 5x2 endpoints, usb1 (HS) has 6x2 endpoints
#define MAX_EP_PAIRS  6

// NOTE data will be transferred as soon as dcd get request by dcd_pipe(_queue)_xfer using double buffering.
// current_td is used to keep track of number of remaining & xferred bytes of the current request.
typedef struct
{
  // 256 byte aligned, 2 for double buffer (not used)
  // Each cmd_sts can only transfer up to DMA_NBYTES_MAX bytes each
  ep_cmd_sts_t ep[2*MAX_EP_PAIRS][2];
  xfer_dma_t dma[2*MAX_EP_PAIRS];

  TU_ATTR_ALIGNED(64) uint8_t setup_packet[8];
}dcd_data_t;

// EP list must be 256-byte aligned
//    Some MCU controller may require this variable to be placed in specific SRAM region.
//    For example: LPC55s69 port1 Highspeed must be USB_RAM (0x40100000)
//    Use CFG_TUD_MEM_SECTION to place it accordingly.
CFG_TUD_MEM_SECTION TU_ATTR_ALIGNED(256) static dcd_data_t _dcd;

// Dummy buffer to fix ZLPs overwriting the buffer (probably an USB/DMA controller bug)
// TODO find way to save memory
CFG_TUD_MEM_SECTION TU_ATTR_ALIGNED(64) static uint8_t dummy[8];

//--------------------------------------------------------------------+
// Multiple Controllers
//--------------------------------------------------------------------+

typedef struct
{
  dcd_registers_t* regs;   // registers
  const bool is_highspeed; // max link speed
  const IRQn_Type irqnum;  // IRQ number
  const uint8_t ep_pairs;  // Max bi-directional Endpoints
}dcd_controller_t;

#ifdef INCLUDE_FSL_DEVICE_REGISTERS

static const dcd_controller_t _dcd_controller[] = {
    { .regs = (dcd_registers_t*) USB0_BASE  , .is_highspeed = false, .irqnum = USB0_IRQn, .ep_pairs = FSL_FEATURE_USB_EP_NUM    },
  #if defined(FSL_FEATURE_SOC_USBHSD_COUNT) && FSL_FEATURE_SOC_USBHSD_COUNT
    { .regs = (dcd_registers_t*) USBHSD_BASE, .is_highspeed = true, .irqnum = USB1_IRQn, .ep_pairs = FSL_FEATURE_USBHSD_EP_NUM }
  #endif
};

#else

static const dcd_controller_t _dcd_controller[] = {
  { .regs = (dcd_registers_t*) LPC_USB0_BASE, .is_highspeed = false, .irqnum = USB0_IRQn, .ep_pairs = 5 },
};

#endif

#if defined(FSL_FEATURE_SOC_USBHSD_COUNT) && FSL_FEATURE_SOC_USBHSD_COUNT
  #define IP3511_HAS_HIGHSPEED
#endif

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline uint16_t get_buf_offset(void const * buffer) {
  uint32_t addr = (uint32_t) buffer;
  TU_ASSERT( (addr & 0x3f) == 0, 0 );
  return ( (addr >> 6) & 0xFFFFUL ) ;
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t ep_addr2id(uint8_t ep_addr) {
  return 2*(ep_addr & 0x0F) + ((ep_addr & TUSB_DIR_IN_MASK) ? 1 : 0);
}

TU_ATTR_ALWAYS_INLINE static inline bool ep_is_iso(ep_cmd_sts_t* ep_cs, bool is_highspeed) {
  return is_highspeed ? (ep_cs[0].cmd_sts.type && !ep_cs[0].cmd_sts.rf_tv) : ep_cs->cmd_sts.type;
}

TU_ATTR_ALWAYS_INLINE TU_ATTR_UNUSED static inline bool ep_is_bulk(ep_cmd_sts_t* ep_cs) {
  return (ep_cs[0].cmd_sts.type == 0) && (ep_cs[0].cmd_sts.rf_tv == 0);
}

TU_ATTR_ALWAYS_INLINE static inline ep_cmd_sts_t* get_ep_cs(uint8_t ep_id) {
  return _dcd.ep[ep_id];
}

TU_ATTR_ALWAYS_INLINE static inline bool rhport_is_highspeed(uint8_t rhport) {
  return _dcd_controller[rhport].is_highspeed;
}

//--------------------------------------------------------------------+
// CONTROLLER API
//--------------------------------------------------------------------+

static void prepare_setup_packet(uint8_t rhport) {
  uint16_t const buf_offset = get_buf_offset(_dcd.setup_packet);
  if ( _dcd_controller[rhport].is_highspeed ) {
    _dcd.ep[0][1].buffer_hs.offset = buf_offset;
  } else {
    _dcd.ep[0][1].buffer_fs.offset = buf_offset;
  }
}

static void edpt_reset(uint8_t rhport, uint8_t ep_id)
{
  (void) rhport;
  tu_memclr(&_dcd.ep[ep_id], sizeof(_dcd.ep[ep_id]));
}

static void edpt_reset_all(uint8_t rhport)
{
  for (uint8_t ep_id = 0; ep_id < 2*_dcd_controller[rhport].ep_pairs; ++ep_id)
  {
    edpt_reset(rhport, ep_id);
  }
  prepare_setup_packet(rhport);
}
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  edpt_reset_all(rhport);

  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;

  dcd_reg->EPLISTSTART  = (uint32_t) _dcd.ep;
  dcd_reg->DATABUFSTART = tu_align((uint32_t) &_dcd, TU_BIT(22)); // 22-bit alignment
  dcd_reg->INTSTAT     |= dcd_reg->INTSTAT; // clear all pending interrupt
  dcd_reg->INTEN        = INT_DEVICE_STATUS_MASK;
  dcd_reg->DEVCMDSTAT  |= DEVCMDSTAT_DEVICE_ENABLE_MASK | DEVCMDSTAT_DEVICE_CONNECT_MASK |
                           DEVCMDSTAT_RESET_CHANGE_MASK | DEVCMDSTAT_CONNECT_CHANGE_MASK | DEVCMDSTAT_SUSPEND_CHANGE_MASK;

  NVIC_ClearPendingIRQ(_dcd_controller[rhport].irqnum);

  return true;
}

void dcd_int_enable(uint8_t rhport)
{
  NVIC_EnableIRQ(_dcd_controller[rhport].irqnum);
}

void dcd_int_disable(uint8_t rhport)
{
  NVIC_DisableIRQ(_dcd_controller[rhport].irqnum);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;

  // Response with status first before changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);

  dcd_reg->DEVCMDSTAT &= ~DEVCMDSTAT_DEVICE_ADDR_MASK;
  dcd_reg->DEVCMDSTAT |= dev_addr;
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}

void dcd_connect(uint8_t rhport)
{
  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;
  dcd_reg->DEVCMDSTAT |= DEVCMDSTAT_DEVICE_CONNECT_MASK;
}

void dcd_disconnect(uint8_t rhport)
{
  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;
  dcd_reg->DEVCMDSTAT &= ~DEVCMDSTAT_DEVICE_CONNECT_MASK;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// DCD Endpoint Port
//--------------------------------------------------------------------+
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  // TODO cannot able to STALL Control OUT endpoint !!!!! FIXME try some walk-around
  uint8_t const ep_id = ep_addr2id(ep_addr);
  _dcd.ep[ep_id][0].cmd_sts.stall = 1;
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const ep_id = ep_addr2id(ep_addr);

  _dcd.ep[ep_id][0].cmd_sts.stall        = 0;
  _dcd.ep[ep_id][0].cmd_sts.toggle_reset = 1;
  _dcd.ep[ep_id][0].cmd_sts.rf_tv        = 0;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  //------------- Prepare Queue Head -------------//
  uint8_t ep_id = ep_addr2id(p_endpoint_desc->bEndpointAddress);
  ep_cmd_sts_t* ep_cs = get_ep_cs(ep_id);

  // Check if endpoint is available
  TU_ASSERT( ep_cs[0].cmd_sts.disable && ep_cs[1].cmd_sts.disable );

  edpt_reset(rhport, ep_id);

  switch (p_endpoint_desc->bmAttributes.xfer) {
    case TUSB_XFER_ISOCHRONOUS:
      ep_cs[0].cmd_sts.type = 1;
      break;

    case TUSB_XFER_INTERRUPT:
      // What is interrupt endpoint in rate feedback mode ?
      if ( rhport_is_highspeed(rhport) ) {
        ep_cs[0].cmd_sts.type = 1;
        ep_cs[0].cmd_sts.rf_tv = 1;
      }
      break;

    case TUSB_XFER_BULK:
      // nothing to do both type and rf_tv are 0
      break;

    default: break;
  }

  // Enable EP interrupt
  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;
  dcd_reg->INTEN |= TU_BIT(ep_id);

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  for (uint8_t ep_id = 0; ep_id < 2*_dcd_controller[rhport].ep_pairs; ++ep_id)
  {
    _dcd.ep[ep_id][0].cmd_sts.active = _dcd.ep[ep_id][0].cmd_sts.active = 0; // TODO proper way is to EPSKIP then wait ep[][].active then write ep[][].disable (see table 778 in LPC55S69 Use Manual)
    _dcd.ep[ep_id][0].cmd_sts.disable = _dcd.ep[ep_id][1].cmd_sts.disable = 1;
  }
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t ep_id = ep_addr2id(ep_addr);
  _dcd.ep[ep_id][0].cmd_sts.active = _dcd.ep[ep_id][0].cmd_sts.active = 0; // TODO proper way is to EPSKIP then wait ep[][].active then write ep[][].disable (see table 778 in LPC55S69 Use Manual)
  _dcd.ep[ep_id][0].cmd_sts.disable = _dcd.ep[ep_id][1].cmd_sts.disable = 1;
}

static void prepare_ep_xfer(uint8_t rhport, uint8_t ep_id, uint16_t buf_offset, uint16_t total_bytes) {
  uint16_t nbytes;
  ep_cmd_sts_t* ep_cs = get_ep_cs(ep_id);

  const bool is_iso = ep_is_iso(ep_cs, _dcd_controller[rhport].is_highspeed);

  if ( rhport_is_highspeed(rhport) ) {
    nbytes = tu_min16(total_bytes, is_iso ? NBYTES_ISO_HS_MAX : NBYTES_CBI_HS_MAX);
    #if TU_CHECK_MCU(OPT_MCU_LPC54)
    // LPC54 Errata USB.1: In USB high-speed device mode, the NBytes field does not decrement after BULK OUT transfer.
    // Suggested Work-around: Program the NByte to the max packet size (512)
    // Actual Work-around: round up NByte to multiple of 4.
    // Note: this can cause buffer overflowed and corrupt data if host send more data than total_bytes
    if ( (ep_id > 1) && (ep_id & 0x01) == 0 && ep_is_bulk(ep_cs) ) {
      if ( nbytes & 0x03 ) {
        nbytes = tu_align4(nbytes) + 4;
      }
    }
    #endif

    ep_cs[0].buffer_hs.offset = buf_offset;
    ep_cs[0].buffer_hs.nbytes = nbytes;
  }else {
    nbytes = tu_min16(total_bytes, is_iso ? NBYTES_ISO_FS_MAX : NBYTES_CBI_FS_MAX);
    ep_cs[0].buffer_fs.offset = buf_offset;
    ep_cs[0].buffer_fs.nbytes = nbytes;
  }

  _dcd.dma[ep_id].nbytes = nbytes;
  ep_cs[0].cmd_sts.active = 1;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  uint8_t const ep_id = ep_addr2id(ep_addr);

  if (!buffer || total_bytes == 0) {
    // Although having no data, ZLPs can cause buffer overwritten to zeroes. Probably due to USB/DMA controller side
    // effect/bug. Assigned buffer offset to (valid) dummy to prevent overwriting to DATABUFSTART
    buffer = (uint8_t *) (uint32_t) dummy;
  }

  tu_memclr(&_dcd.dma[ep_id], sizeof(xfer_dma_t));
  _dcd.dma[ep_id].total_bytes = total_bytes;

  prepare_ep_xfer(rhport, ep_id, get_buf_offset(buffer), total_bytes);

  return true;
}

//--------------------------------------------------------------------+
// IRQ
//--------------------------------------------------------------------+
static void bus_reset(uint8_t rhport)
{
  tu_memclr(&_dcd, sizeof(dcd_data_t));
  edpt_reset_all(rhport);

  // disable all endpoints as specified by LPC55S69 UM Table 778
  for(uint8_t ep_id = 0; ep_id < 2*MAX_EP_PAIRS; ep_id++)
  {
    _dcd.ep[ep_id][0].cmd_sts.disable = _dcd.ep[ep_id][1].cmd_sts.disable = 1;
  }

  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;

  dcd_reg->EPINUSE      = 0;
  dcd_reg->EPBUFCFG     = 0;
  dcd_reg->EPSKIP       = 0xFFFFFFFF;

  dcd_reg->INTSTAT      = dcd_reg->INTSTAT;                               // clear all pending interrupt
  dcd_reg->DEVCMDSTAT  |= DEVCMDSTAT_SETUP_RECEIVED_MASK;                    // clear setup received interrupt
  dcd_reg->INTEN        = INT_DEVICE_STATUS_MASK | TU_BIT(0) | TU_BIT(1); // enable device status & control endpoints
}

static void process_xfer_isr(uint8_t rhport, uint32_t int_status) {
  uint8_t const max_ep = 2*_dcd_controller[rhport].ep_pairs;

  for(uint8_t ep_id = 0; ep_id < max_ep; ep_id++ ) {
    if ( tu_bit_test(int_status, ep_id) ) {
      ep_cmd_sts_t * ep_cs = &_dcd.ep[ep_id][0];
      xfer_dma_t* xfer_dma = &_dcd.dma[ep_id];

      if ( ep_id <= 1 ) {
        // For control endpoint, we need to manually clear Active bit
        ep_cs->cmd_sts.active = 0;
      }

      uint16_t buf_offset;
      uint16_t buf_nbytes;

      if ( rhport_is_highspeed(rhport) ) {
        buf_offset = ep_cs->buffer_hs.offset;
        buf_nbytes = ep_cs->buffer_hs.nbytes;

        #if TU_CHECK_MCU(OPT_MCU_LPC54)
        // LPC54 Errata USB.2: In USB high-speed device mode, the NBytes field is not correct after BULK IN transfer
        // There is no work-around. For EP in transfer, the NByte value can be ignored after a packet is transmitted.
        if ( (ep_id > 1) && (ep_id & 0x01) == 1 && ep_is_bulk(ep_cs) ) {
          buf_nbytes = 0;
        }
        #endif
      } else {
        buf_offset = ep_cs->buffer_fs.offset;
        buf_nbytes = ep_cs->buffer_fs.nbytes;
      }

      xfer_dma->xferred_bytes += xfer_dma->nbytes - buf_nbytes;

      if ( (buf_nbytes == 0) && (xfer_dma->total_bytes > xfer_dma->xferred_bytes) ) {
        // There is more data to transfer
        // buff_offset has been already increased by hw to correct value for next transfer
        prepare_ep_xfer(rhport, ep_id, buf_offset, xfer_dma->total_bytes - xfer_dma->xferred_bytes);
      } else {
        // for detecting ZLP
        xfer_dma->total_bytes = xfer_dma->xferred_bytes;

        uint8_t const ep_addr = tu_edpt_addr(ep_id / 2, ep_id & 0x01);

        // TODO no way determine if the transfer is failed or not
        dcd_event_xfer_complete(rhport, ep_addr, xfer_dma->xferred_bytes, XFER_RESULT_SUCCESS, true);
      }
    }
  }
}

void dcd_int_handler(uint8_t rhport)
{
  dcd_registers_t* dcd_reg = _dcd_controller[rhport].regs;

  uint32_t const cmd_stat = dcd_reg->DEVCMDSTAT;

  uint32_t int_status = dcd_reg->INTSTAT & dcd_reg->INTEN;
  dcd_reg->INTSTAT = int_status; // Acknowledge handled interrupt

  if (int_status == 0) return;

  //------------- Device Status -------------//
  if ( int_status & INT_DEVICE_STATUS_MASK )
  {
    dcd_reg->DEVCMDSTAT |= DEVCMDSTAT_RESET_CHANGE_MASK | DEVCMDSTAT_CONNECT_CHANGE_MASK | DEVCMDSTAT_SUSPEND_CHANGE_MASK;

    if ( cmd_stat & DEVCMDSTAT_RESET_CHANGE_MASK) // bus reset
    {
      bus_reset(rhport);

      tusb_speed_t speed = TUSB_SPEED_FULL;
      if ( _dcd_controller[rhport].is_highspeed ) {
        // 0 : reserved, 1 : full, 2 : high, 3: super
        if ( 2 == ((cmd_stat >> DEVCMDSTAT_SPEED_SHIFT) & 0x3UL) ) {
          speed= TUSB_SPEED_HIGH;
        }
      }

      dcd_event_bus_reset(rhport, speed, true);
    }

    if (cmd_stat & DEVCMDSTAT_CONNECT_CHANGE_MASK)
    {
      // device disconnect
      if (cmd_stat & DEVCMDSTAT_DEVICE_ADDR_MASK)
      {
        // debouncing as this can be set when device is powering
        dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
      }
    }

    if (cmd_stat & DEVCMDSTAT_SUSPEND_CHANGE_MASK)
    {
      // suspend signal, bus idle for more than 3ms
      // Note: Host may delay more than 3 ms before and/or after bus reset before doing enumeration.
      if (cmd_stat & DEVCMDSTAT_DEVICE_ADDR_MASK)
      {
        dcd_event_bus_signal(rhport, (cmd_stat & DEVCMDSTAT_DEVICE_SUSPEND_MASK) ? DCD_EVENT_SUSPEND : DCD_EVENT_RESUME, true);
      }
    }
  }

  // Setup Receive
  if ( tu_bit_test(int_status, 0) && (cmd_stat & DEVCMDSTAT_SETUP_RECEIVED_MASK) )
  {
    // Follow UM flowchart to clear Active & Stall on both Control IN/OUT endpoints
    _dcd.ep[0][0].cmd_sts.active = _dcd.ep[1][0].cmd_sts.active = 0;
    _dcd.ep[0][0].cmd_sts.stall = _dcd.ep[1][0].cmd_sts.stall = 0;

    dcd_reg->DEVCMDSTAT |= DEVCMDSTAT_SETUP_RECEIVED_MASK;

    dcd_event_setup_received(rhport, _dcd.setup_packet, true);

    // keep waiting for next setup
    prepare_setup_packet(rhport);

    // clear bit0
    int_status = tu_bit_clear(int_status, 0);
  }

  // Endpoint transfer complete interrupt
  process_xfer_isr(rhport, int_status);
}

#endif
